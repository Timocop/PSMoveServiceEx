# PSVR built-in LED tracking

This document describes the single-camera, seated PSVR tracking path in
PSMoveServiceEx. It is intended for the original PSVR headsets (CUH-ZVR1 and
CUH-ZVR2) used with a normal Windows webcam, PSMoveServiceEx, and an existing
SteamVR display driver such as iVRy.

The feature is designed to provide absolute position and slow orientation
correction from the headset's blue tracking lights while retaining the
headset IMU for low-latency orientation and prediction.

## Scope and guarantees

The supported first configuration is:

- one fixed camera in front of a seated user;
- seven front PSVR light regions (A through G);
- a calibrated camera at 30 FPS or faster;
- approximately 0.4 m to 2.5 m headset-to-camera distance;
- modest seated head translation and rotation;
- the existing Morpheus HMD stream consumed by iVRy or the companion virtual
  device manager.

This is not lighthouse-class tracking. A single RGB camera has no hardware
clock shared with the headset, the front light pattern is nearly planar, and
consumer webcams often have rolling shutters and buffered frames. The service
therefore rejects ambiguous optical solutions instead of publishing large pose
jumps. During a short optical dropout, IMU orientation continues while
translation is predicted by the existing position filter.

The checked-in LED coordinates are a versioned bootstrap model, not Sony
factory calibration. They are based on the historical PSMoveServiceEx
measurements. Physical-camera and headset validation is required before a
release is labelled hardware validated.

## Data flow

```text
PSVR sensor report (two samples, 24-bit 1 MHz ticks)
  -> unwrap headset ticks
  -> map headset time to the host monotonic clock
  -> accelerometer/gyro calibration
  -> high-rate orientation filter and prediction

Webcam frame
  -> capture/delivery timestamp minus configured camera latency
  -> HSV segmentation for the PSVR blue
  -> contour and centroid extraction
  -> project the configured LED model from the IMU orientation prior
  -> deterministic translation hypotheses and one-to-one LED/blob assignment
  -> bounded outlier subsets and seeded iterative PnP
  -> positive-depth, inlier, reprojection, IMU-angle, and continuity checks
  -> optical position and orientation at the measurement timestamp

Latest headset IMU state + delayed optical solution
  -> reject duplicate, future, or over-age camera measurements
  -> rewind the IMU orientation to exposure time
  -> carry the optical pose forward by the measured IMU motion
  -> stamp the correction on the latest applied IMU state
  -> complementary optical/IMU orientation correction
  -> existing position filter
  -> prediction to the PSMoveServiceEx HMD stream
  -> iVRy/SteamVR
```

The optical solver is deliberately independent of image segmentation. It
accepts camera intrinsics, a 3D LED model, unordered 2D observations, an IMU
orientation prior, and an optional previous optical position. This makes the
mathematics deterministic and allows synthetic tests without a headset.

## Pose acquisition and tracking

All seven front light regions are enabled for built-in tracking. Three points,
which the old placeholder path used, cannot provide a robust monocular pose.
Rear regions H and I are excluded from the default seated profile.

For an already tracked headset, the previous optical translation and
exposure-time IMU orientation project the LED model. For initial acquisition
or recovery, a bounded five-heading search retains IMU pitch and roll while
testing deterministic translation hypotheses derived from every usable
model/blob pair. Near-parallel rays and physically impossible depths are
discarded before exact scoring.

A private-dummy Hungarian assignment finds the maximum-cardinality,
minimum-error one-to-one LED/blob mapping. The best unique mappings are tested
with every meaningful one- or two-outlier subset using seeded iterative PnP.
Every result is rematched against all visible points and refined at most twice.
No randomized OpenCV RANSAC or global RNG state is used, so repeated frames
have a fixed work bound and deterministic correspondence ordering.

The result is accepted only if:

- enough unique LEDs are inliers;
- every used point is in front of the camera;
- translation is inside the configured camera depth range;
- reprojection RMS is below the configured threshold;
- optical orientation is consistent with the IMU prior;
- a tracked solution does not exceed the configured frame-to-frame
  translation jump.

Failing any gate produces no optical measurement. It never produces a zero
pose marked as valid.

## Time alignment

Each PSVR sensor report contains two device-timed samples. The device counter
is treated as a 24-bit, 1 MHz clock and unwrapped across rollover. A robust
one-way clock mapper follows the minimum observed device-to-host offset quickly
and follows increases slowly, which rejects most USB scheduling delay while
still allowing clock drift.

Generic webcam backends generally expose delivery time rather than the start
of exposure. Each camera profile therefore stores `frame_latency_ms`.
Optical measurements use:

```text
camera delivery timestamp - frame_latency_ms
```

Start at zero if latency is unknown. During the hardware pass, tune latency
while making slow yaw movements and observing the projected-model overlay.
Incorrect latency appears as a direction-dependent reprojection trail.

An accepted camera solution is never inserted behind already-applied headset
samples. The service evaluates the headset orientation at both the exposure
time and the latest applied IMU timestamp, applies that local/right-side
rotation delta to the optical orientation, and advances optical position with
the current filtered velocity. It then fuses the correction at the latest IMU
timestamp. Duplicate frames, timestamps newer than that filter state, and
frames older than `OpticalTracking.Solver.MaxFrameAgeMs` are skipped. A frame
that is momentarily newer than the filter can be reconsidered on the following
service tick; a frame is remembered as consumed only after it is accepted.

This is a bounded constant-rate/constant-velocity alignment, not a full filter
history replay. The default maximum age is 250 ms, and lowering it is sensible
after real camera latency is known. It avoids adding a webcam-sized delay to
the low-latency IMU orientation, but rapid reversals inside a heavily buffered
frame remain a real single-camera limitation.

There is no claim of hardware synchronization between a PSVR and a generic
webcam.

## Camera setup

Generic webcams are opt-in. This prevents service startup from opening a camera
that belongs to another application.

1. Start PSMoveServiceEx once and read the console log. Each Windows camera is
   listed as `Generic webcam available` with a friendly name and a `wmf_...`
   stable ID. This discovery reads device attributes only; it does not start a
   disabled camera.
2. In Config Tool, open **Advanced Settings > Trackers**, paste the wanted
   `wmf_...` value into **Webcam stable ID**, enable **Generic webcam**, save,
   and restart the service.
3. Confirm that the log reports `Opened generic webcam` with the expected
   friendly name and exact native mode. The closest supported BGR-convertible
   mode to 640 x 480 at 30 FPS is selected initially and its full Media
   Foundation mode identity is persisted.
4. Mount the webcam rigidly in front of the seated position. Avoid a very wide
   side angle. Do not change USB port after setup without checking the stable
   ID again.
5. Finalize the camera mode before calibration. Higher frame rate is usually
   more useful than resolution above 640 x 480. A saved width, height, or frame
   rate change clears the exact mode selection and is applied on the next
   service restart.
6. Run the existing tracker lens calibration for that exact camera mode and
   field of view. The service records the full native mode with the intrinsics
   and refuses PSVR point-cloud poses if they no longer match.
7. Set exposure and gain from the normal tracker settings. Zero requests the
   camera's automatic mode; values 1 through 255 select a normalized manual
   value when the driver exposes that control. Unsupported controls return an
   error. Disable automatic focus, white balance, digital zoom, stabilization,
   and face framing in the camera vendor's software where possible.
8. Select the Morpheus `Built-in Tracking Lights` method.
9. Start with LED intensity 50. Lower it if light regions merge or bloom; raise
   it if regions disappear during movement.
10. Tune the Blue HSV preset with the camera preview until A-G are distinct and
    background blue objects are excluded.
11. Use the `ComplementaryOpticalARG` orientation filter so optical orientation
    can correct slow gyro yaw drift.
12. Calibrate/recenter the tracker and seated forward direction through the
    normal PSMoveServiceEx flow.

Camera intrinsics are mandatory for useful depth. A default focal length can
make an image look plausible while producing the wrong scale.

## Configuration

Morpheus configuration stores:

- built-in LED mask and intensity;
- minimum inlier count;
- association and reprojection thresholds;
- acquisition depth range;
- maximum orientation disagreement and translation jump;
- maximum accepted camera-frame age;
- the versioned nine-point optical model.

Tracker configuration stores the per-camera frame latency. Camera configuration
continues to be keyed by the stable tracker identifier, so lens and latency
values do not silently follow a different webcam when Windows enumeration order
changes.

Windows configuration files are under
`%APPDATA%\PSMoveService`. The selected camera uses
`PS3EyeTrackerConfig_wmf_<stable hash>.json`. Stop the service before manually
editing it:

- `video_mode` is the exact native mode chosen by the service;
- `frame_width`, `frame_height`, and `frame_rate` are used to select the nearest
  mode when `video_mode` is empty;
- `frame_latency_ms` is the non-negative delivery-to-exposure correction;
- `calibration_video_mode` is written by lens calibration and should not be
  copied between modes or cameras.

Prefer Config Tool over manual JSON edits. If a manual mode change is needed,
clear `video_mode`, set the requested dimensions and rate, restart, verify the
new exact mode in the log, and recalibrate.

Conservative defaults target front-facing seated use. Advanced values should
be changed only with the diagnostic overlay or recorded replay available.

## iVRy and SteamVR

Tracking remains inside PSMoveServiceEx and is published through the existing
Morpheus HMD stream. This avoids registering a second SteamVR HMD beside iVRy.
Use iVRy's PSMoveServiceEx tracking integration where available.

The companion Virtual Device Manager remains an alternative consumer for
testing. Do not enable an additional HMD-class virtual device at the same time
as iVRy until the hardware integration pass confirms which driver owns the HMD.

## Validation

Hardware-free validation currently covers:

- 24-bit tick rollover, gaps, out-of-order samples, and host-clock mapping;
- exact synthetic projections using both the independent reference geometry
  and the service's checked-in Morpheus bootstrap geometry;
- pixel noise, missing LEDs, false blobs, and unordered input;
- the service's signed camera-intrinsic convention;
- nonzero lens distortion;
- rejection of a fresh mirrored/wrong translation prior and a 90-degree
  IMU-inconsistent orientation;
- four-point continuation and stale-prior reacquisition;
- deterministic webcam timestamp mapping, latest-frame replacement, exact
  mode/interlace identity, stable camera IDs, and empty-allowlist safety;
- delayed-frame fusion math, including noncommuting IMU motion, duplicate and
  out-of-order suppression, stale/future rejection, and position alignment.

The Windows Media Foundation implementation, configuration migration, and
protocol changes have been reviewed, but the Windows-only code has not been
compiled on this machine because MSVC and the Windows SDK are not installed.
Those items are validation gates, not hardware-free test claims.

The first hardware session must still verify:

- a native Windows build and service startup;
- CUH-ZVR1 or CUH-ZVR2 enumeration;
- the visual A-G mapping and useful intensity range;
- real contour centroids, bloom, and partial occlusion;
- camera control support and actual buffering latency;
- scale and reprojection accuracy across the seated motion envelope;
- yaw correction without oscillation;
- iVRy ownership, recentering, and SteamVR pose validity;
- at least 20 minutes of seated tracking without a large pose discontinuity.

Record the camera feed and service diagnostics during that session. A replay
fixture is the fastest way to turn any failure into a deterministic regression
test.

## Design references

- [Sony CUH-ZVR1 instruction manual](https://manuals.playstation.net/document/pdf/CUH-ZVR1U_2.pdf)
- [Sony CUH-ZVR2 instruction manual](https://manuals.playstation.net/document/pdf/CUH-ZVR2H_T_2.pdf)
- [Monado PSVR driver documentation](https://monado.pages.freedesktop.org/monado/group__drv__psvr.html)
- [OpenHMD PSVR driver source](https://github.com/OpenHMD/OpenHMD/tree/master/src/drv_psvr)
- [OpenCV camera calibration and 3D reconstruction](https://docs.opencv.org/3.1.0/d9/d0c/group__calib3d.html)
- [Microsoft video-capture device enumeration](https://learn.microsoft.com/en-us/windows/win32/medfound/enumerating-video-capture-devices)
- [Microsoft Source Reader asynchronous mode](https://learn.microsoft.com/en-us/windows/win32/medfound/source-reader)
- [iVRy PSMoveServiceEx tracker integration](https://store.steampowered.com/app/1257790/iVRy_PSMoveServiceEx_Tracker_for_SteamVR/)
