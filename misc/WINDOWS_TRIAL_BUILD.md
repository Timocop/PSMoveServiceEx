# Windows trial build

The supported trial package is produced by
`.github/workflows/windows-build.yml`. It is a portable, unsigned Win64 ZIP;
it is not an installer.

## Downloading a workflow build

1. Open the **Actions** tab for the branch containing the change.
2. Open a successful **Windows trial build** run for the intended commit.
3. Download the `PSMoveServiceEx-PSVR-LED-Win64-*` artifact.
4. In PowerShell, compare the downloaded ZIP with the adjacent checksum:

   ```powershell
   Get-FileHash .\PSMoveServiceEx-PSVR-LED-Win64.zip -Algorithm SHA256
   Get-Content .\PSMoveServiceEx-PSVR-LED-Win64.zip.sha256
   ```

   The two hexadecimal hashes must match.
5. Extract the ZIP completely to a new folder. Do not run either executable
   from inside the ZIP and do not mix these files into a different
   PSMoveServiceEx release.
6. Check `BUILD_INFO.txt` in the extracted `PSMoveService` folder. Its commit
   must match the commit being tested.

The package contains `PSMoveService.exe`, `PSMoveConfigTool.exe`,
`PSMoveClient_CAPI.dll`, the Config Tool assets, both PSVR setup guides, and
`SHA256SUMS.txt` for the extracted files. It also includes a root-level
`calibration` folder so the first-run guide's printable checkerboard works
offline. Windows configuration remains in `%APPDATA%\PSMoveService`; replacing
or moving the portable program folder does not delete calibration data.

Because the package is unsigned test software, Windows SmartScreen may show an
unknown-publisher warning. Do not bypass a warning for a ZIP obtained anywhere
other than this repository's workflow run or an explicitly linked prerelease.

## Reproducing the build

The workflow uses:

- the `windows-2022` GitHub-hosted image and Visual Studio 2022;
- the repository's `x64-windows-static-v142` overlay triplet, matching the
  service's static MSVC runtime while keeping Boost 1.83 on its supported v142
  toolset;
- the vcpkg baseline recorded in `vcpkg.json`;
- OpenCV 4.6.0, Protobuf 3.21.12, Boost 1.83.0, SDL2, libusb, and Eigen3.

Use the immutable vcpkg tool revision recorded in the workflow and set
`VCPKG_ROOT` to that checkout. The manifest baseline independently selects the
pinned port versions. The equivalent configure command is:

```powershell
$env:VCPKG_OVERLAY_TRIPLETS = (Resolve-Path .\cmake\triplets).Path

cmake -S . -B build `
  -G "Visual Studio 17 2022" `
  -A x64 `
  -T v142 `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows-static-v142 `
  -DVCPKG_OVERLAY_TRIPLETS="$env:VCPKG_OVERLAY_TRIPLETS" `
  -DPSMOVE_USE_SYSTEM_DEPENDENCIES=ON `
  -DProtobuf_USE_STATIC_LIBS=ON
```

Build the runtime and deterministic tracking tests:

```powershell
cmake --build build --config Release --parallel 2 --target `
  PSMoveService PSMoveConfigTool PSMoveClient_CAPI `
  test_morpheus_sensor_clock test_hmd_point_cloud_pose_solver `
  test_hmd_optical_pose_fusion test_tracker_video_source_helpers `
  test_generic_webcam_enumerator_helpers

ctest --test-dir build -C Release --output-on-failure
```

Do not use the repository's VC14 batch files for this package. They remain for
legacy development and expect Visual Studio 2015, Boost 1.61, and a hard-coded
OpenCV 3.1 library layout.

## Known build compatibility note

The pinned `libstem_gamepad` dependency exposes `Gamepad_init()` without the
newer boolean argument used by one historical PSMoveServiceEx call. Clean
checkouts now use that dependency's supported no-argument API. Consequently,
the advanced `gamepad_api_xinput_only` setting cannot suppress all DirectInput
enumeration inside libstem and the service logs this limitation when the
setting is enabled. This does not affect PSVR headset, webcam, HID, or optical
tracking.
