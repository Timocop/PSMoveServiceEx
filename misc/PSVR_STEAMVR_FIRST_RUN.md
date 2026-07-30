# PSVR webcam 6DoF: SteamVR first run

> **Prerequisite: obtain and fully extract a matching custom Win64 test
> package first.** See [Windows trial build](WINDOWS_TRIAL_BUILD.md).
>
> The webcam LED tracker described here is not present in stock/upstream
> PSMoveServiceEx binaries, and installing iVRy does not add it. The package
> must keep its matching `PSMoveService.exe`, `PSMoveConfigTool.exe`,
> `PSMoveClient_CAPI.dll`, `assets` folder, and `BUILD_INFO.txt` together. Do
> not move only the executables or mix files with another PSMoveServiceEx
> release.

This guide is for a seated CUH-ZVR1 or CUH-ZVR2 PSVR, one fixed Windows webcam,
the custom PSMoveServiceEx package, and iVRy as the SteamVR HMD driver. iVRy
continues to handle the display and SteamVR connection; the custom
PSMoveServiceEx build calculates the webcam/IMU pose.

The first test is intentionally conservative: use one front-facing camera,
about 0.4 m to 2.5 m from the headset, and aim for at least 30 FPS.

## Before starting

Have these ready:

- 64-bit Windows 10 or 11, SteamVR, the iVRy base driver, and iVRy's PSVR Lite
  or Premium DLC;
- the fully extracted custom package described above;
- a PSVR that already powers on and displays through its processing unit;
- iVRy and the
  [iVRy PSMoveServiceEx Tracker for SteamVR](https://store.steampowered.com/app/1257790/iVRy_PSMoveServiceEx_Tracker_for_SteamVR/)
  installed;
- one webcam mounted rigidly, centred in front of the seated position, and as
  close to level as practical;
- the
  [A4 checkerboard](calibration/distortion/Checkerboard_A4.pdf), printed at
  100% scale on flat paper or card.

Close SteamVR, iVRy, camera applications, and every other copy of
PSMoveServiceEx before configuring the custom package. Extract the package to
its own folder instead of copying it over an existing installation.

Before changing settings, copy `%APPDATA%\PSMoveService` to a clearly named
backup folder if it already exists. Stock and custom builds share this
configuration location.

For the iVRy display baseline:

1. Connect the processor box's headset HDMI directly to the graphics card,
   without an adapter. Connect the monitor to a separate graphics-card output;
   do not use the processor box's TV output.
2. In Windows Display Settings, use **Extend these displays**, keep the monitor
   as the primary display, and do not use **Duplicate**.
3. Install the free tracker DLC linked above. Launch iVRy once from its
   **Play/Launch** button in Steam so it can register its drivers and services.
   During iVRy's first PSVR calibration, leave the headset flat, level, and
   facing forward until its LEDs stop flashing. Then close SteamVR.
4. In Windows **Settings > Privacy & security > Camera**, enable camera access
   and **Let desktop apps access your camera**.

For this test, never choose iVRy's **Configure PSMoveService** launch option:
it starts the bundled stock service. The DLC is used only as the iVRy bridge;
all configuration below uses the custom package.

## 1. Check PSVR and webcam detection

1. Connect and power the PSVR processing unit, connect the PSVR USB cable, and
   connect the webcam.
2. From the custom package folder, run `PSMoveService.exe`. For a more useful
   first-test log, PowerShell users can run:

   ```powershell
   .\PSMoveService.exe --log_level debug
   ```

3. Check the service window for the camera, Morpheus, and successful-startup
   messages. Typical lines include:

   ```text
   Generic webcam available: "<camera name>" stable_id=wmf_...
   Opening MorpheusHMD(...)
   Turning on MorpheusHMD power.
   Turning on MorpheusHMD VR-Mode.
   Startup successful! Entering main loop...
   ```

4. Copy the complete `wmf_...` stable ID for the intended webcam.
5. While the custom service is running, open the matching
   `PSMoveConfigTool.exe`.
6. Open **Advanced Settings** > **Trackers**. Enable **Generic webcam**, paste
   the ID into **Webcam stable ID**, and choose **Save Settings**.
7. Follow the on-screen `Restart PSMoveServiceEx to apply changes.`
   instruction: close the service window, start the custom
   `PSMoveService.exe --log_level debug` again, and let Config Tool reconnect.
8. Confirm that the restarted service reports:

   ```text
   Opened generic webcam "<camera name>" stable_id=wmf_... mode=...
   ```

The friendly name, stable ID, and selected native mode should all describe the
camera you mounted. Do not change its USB port after this point unless you
check the ID again.

## 2. Calibrate the exact webcam mode

Camera calibration is required for useful depth and scale. It belongs to the
exact native camera mode: changing resolution, frame rate, or USB camera later
requires another calibration.

1. In Config Tool, open **Tracker Settings** and select the generic webcam.
   Confirm **Tracker Driver: Generic Webcam**.
2. Use **Test Video Feed** to check that the view is live and covers the whole
   seated movement area. Check the displayed **Tracker Frame Rate** and the
   service's `Opened generic webcam` line. The generic-webcam default requests
   a native 640 x 480 mode at about 30 FPS.
3. If the opened mode is not 640 x 480, open **Head-mounted Displays**, select
   the Morpheus HMD, then open **Calibration > Calibrate Tracking Colors**.
   Expand the left **Advanced Settings** and use the controls beside
   **Frame Width** to request `640` and beside **Frame Rate** to request about
   `30` FPS. Leave that screen, restart the custom service with
   `--log_level debug`, and verify the new `Opened generic webcam` line.
4. Before lens calibration, use the webcam's own software to disable
   autofocus, digital zoom, stabilization, face tracking, and auto-framing.
   Keep these fixed afterward: changing focus or field of view invalidates the
   calibration. Automatic white balance may also make the blue mask unstable.
5. Select **Calibrate Tracker Distortion**. The current
   distortion-calibration screen rejects resolutions other than 640 x 480.
6. Measure one printed checkerboard square and enter its real size in
   **Square Length (mm)**. Select **Pattern Mode** > **Paper (Big)**. This is a
   9 x 6 internal-corner pattern.
7. Keep the page flat and move it through different positions and angles while
   keeping all corners visible. The calibrator automatically gathers 12 good
   samples.
8. Wait for `Calibration complete!` and record the exact `Error: ...` value.
   There is not yet a hardware-validated pass threshold, so use **Test Tracking
   Pose** in step 5 as the final gate. If corners were detected poorly or the
   result looks implausible, choose **Redo Calibration** and collect more
   varied views.

Leave **Fixed Aspect Ratio** and **Fixed Focal Length** at their defaults for
the first run unless the camera's known calibration requires otherwise.

## 3. Calibrate the PSVR IMU and enable its lights

1. Open **Head-mounted Display Settings** > **Settings**. Confirm
   **HMD Type: Morpheus**.
2. In the **Calibration** tab:
   - choose **Calibrate Gyroscope**, put the headset completely still on a
     stable surface, and leave it untouched while the tool waits for stability
     and samples;
   - choose **Calibrate Accelerometer**, place the headset flat and level,
     select **Start Sampling**, do not touch it until `Sampling complete!`,
     and select **OK**.
3. Set **Tracking Method** to **Built-in Tracking Lights**.
4. Enable all seven front regions, **A** through **G**. Fewer than five cannot
   produce the intended robust pose.
5. Set **LED Intensity** to `50` as a starting value.
6. Open **Filters** and set **Orientation Filter** to
   **ComplementaryOpticalARG**. Leave **Position Filter** at
   **PositionKalman** for the first run.

If gyro calibration says
`HMD destabilized! Waiting for stabilization...`, make the surface more stable
and wait; do not hold the headset during sampling.

## 4. Tune the blue-light mask

The goal is seven separate headset regions with no background detections.

1. Open **Tracker Settings** > **Head-mounted Displays** and select
   **HMD: 0 (Morpheus) - Blue**.
2. Open **Calibration** > **Calibrate Tracking Colors**.
3. Expand **Color Detection**, then click **Manually Detect Colors**.
4. Use the **Video Preview** choices:
   - **Color (BGR)** to inspect the camera image;
   - **Hue, Saturation, Value (HSV)** to understand the selected colour;
   - **Masked** to judge the actual tracking regions.
5. In the left **Advanced Settings**, adjust camera exposure and gain. `0`
   requests the camera's automatic mode; `1` through `255` request a normalized
   manual value if the webcam driver supports that control.
6. In the right **Advanced Settings**, under **Tracking Color: Blue**, adjust
   **Hue Angle**, **Hue Range**, **Saturation Center**, **Saturation Range**,
   **Value Center**, and **Value Range** until **Masked** shows seven separate,
   stable light regions and as little else as possible.

If adjacent lights merge into one region, reduce exposure first and then
reduce **LED Intensity**. If lights vanish when the headset moves, increase
exposure or intensity slightly and widen the mask only as much as necessary.
After exposure and gain changes, confirm the displayed **Tracker Frame Rate**
is still close to 30 FPS. If it drops substantially, shorten exposure or add
room light without introducing blue reflections, then re-check the mask.

## 5. Prove 6DoF before opening SteamVR

1. Open **Tracker Settings** > **Head-mounted Displays** > **Testing**.
2. Run **Test Tracking Colors**. The intended result is:

   ```text
   Tracker #0: OK
   ```

3. Run **Test Tracking Pose** and open **Tracker Video**. Slowly move the
   headset left/right, up/down, and nearer/farther, then add gentle yaw, pitch,
   and roll. The on-screen HMD model should follow all six axes without large
   jumps.

Do not look for **Calibrate Tracker Poses** with a real Morpheus HMD: that
button is currently exposed only for `VirtualHMD`. For this first
single-camera test, keep the webcam centred, straight, level, and fixed, then
set seated forward with the iVRy/SteamVR recenter step. Do not hide a wrong
axis or obviously wrong scale with a large playspace offset; collect the
evidence listed below instead.

Do not continue to SteamVR if the test says `Tracker #0: FAIL`.

## 6. Start iVRy and SteamVR in this order

1. Exit Config Tool, especially any live video or testing screen.
2. Leave the custom `PSMoveService.exe` running.
3. Do not enable a Virtual Device Manager HMD at the same time as iVRy. There
   should be only one SteamVR HMD provider.
4. Launch SteamVR. Open **iVRy Settings**, find **Tracking**, and set
   **Tracker** to **PSMoveService**.
5. Close and restart SteamVR while leaving the custom service running.
6. If SteamVR asks for Room Setup, choose **Standing Only**, sit in the intended
   position, and enter the seated eye height when asked.
7. Face the physical forward direction and hold the keyboard's numpad minus
   key to perform iVRy's recenter.

A successful first run has exactly one HMD in SteamVR, low-latency rotation
from the PSVR IMU, and position that follows the webcam pose test. A short
optical occlusion may briefly predict or hold position, but it should not
cause a large teleport.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| **Built-in Tracking Lights** or webcam options are missing | The Config Tool or service is a stock or mismatched binary. Use both executables from the same custom package. |
| No `Generic webcam available` line | Close all camera apps; check Windows camera privacy permissions and the webcam driver; then restart the service. |
| Windows N/KN cannot enumerate or open a normal webcam | Install Microsoft's Media Feature Pack for the installed Windows version, reboot, and retry. |
| Camera is listed but not opened | Recheck **Generic webcam**, paste the complete **Webcam stable ID**, save, and restart. The device must expose a BGR-convertible mode. |
| Distortion calibration refuses to start | Select a native 640 x 480 mode, restart the service, verify the opened mode in the log, and calibrate that exact mode. |
| The mask looks good but `Tracker #0: FAIL` | The saved intrinsics probably do not match the currently opened native mode. Re-run **Calibrate Tracker Distortion** without changing the mode afterward. |
| LED regions merge or bloom | Lower camera exposure first, then lower **LED Intensity**. |
| LEDs flicker or disappear | Raise exposure or intensity slightly, retune the Blue HSV ranges, and disable the webcam's automatic image features. |
| Pose lags or leaves direction-dependent trails | Keep `frame_latency_ms` at `0` for the initial report. Camera buffering must be measured before this value is tuned. |
| Config Tool has 6DoF but SteamVR has rotation only | Check that iVRy **Tracking > Tracker** is **PSMoveService** and that no second HMD provider owns the pose. |
| SteamVR shows duplicate HMDs | Disable the Virtual Device Manager HMD while using iVRy. |
| Scale, axes, or origin are obviously wrong | Stop and record the evidence below rather than compensating with large offsets. |

Timing diagnostics worth reporting exactly include:

```text
Skipping HMD optical sample: missing capture timestamp
capture timestamp is newer than the filter state
capture timestamp is too old
```

## What to send after the first test

Please report:

- `BUILD_INFO.txt`; Windows version; PSVR model (CUH-ZVR1 or CUH-ZVR2);
  webcam make/model and distance; and SteamVR, iVRy, and tracker-DLC versions;
- the `Generic webcam available`, `Opened generic webcam`, Morpheus startup,
  selected mode, and any warning lines;
- distortion-calibration error, exposure, gain, LED intensity, and whether all
  seven regions remain separate in **Masked** view; include the observed
  **Tracker Frame Rate** after exposure tuning;
- the result of **Test Tracking Colors** and **Test Tracking Pose**;
- whether each movement direction is correct, whether scale feels plausible,
  and whether there are jumps, drift, lag, or occlusion failures;
- whether SteamVR receives position, and whether it shows one or two HMDs;
- a screenshot showing iVRy **Tracking** set to **PSMoveService**.

Attach these files after stopping the service:

- `PSMoveServiceEx.log` from the custom package's working folder;
- `%APPDATA%\PSMoveService\TrackerManagerConfig.json`;
- `%APPDATA%\PSMoveService\PS3EyeTrackerConfig_wmf_*.json`;
- `%APPDATA%\PSMoveService\MorpheusHMDConfig.json`.

From the camera JSON, call out the exact `video_mode`,
`calibration_video_mode`, and `frame_latency_ms` values.

Screenshots or short recordings of **Masked**, **Tracker Video**,
**Test Tracking Pose**, and the SteamVR mirror are especially useful. Remove
unrelated personal information from logs or screenshots before sharing them.

For solver details, configuration fields, limitations, and the full validation
checklist, see [PSVR built-in LED tracking](PSVR_LED_TRACKING.md).
