@echo off

:: Abort if theres no build available.
IF NOT EXIST build goto no_build

:: Input new version.
set /p version= "Please input a new version (e.g: 1.2.3.4 or 1.0) :" 
echo Generating new app_version.txt file with version: '%version%'.

:: Remove old zip and temp folder file.
IF EXIST ".\updater\PSMoveServiceUpdateSFX.dat" del /S /Q ".\updater\PSMoveServiceUpdateSFX.dat"
IF EXIST ".\updater\PSMoveService\" rmdir /S /Q ".\updater\PSMoveService\"

:: Create temp folders.
IF NOT EXIST ".\updater\PSMoveService\" mkdir ".\updater\PSMoveService\"
IF NOT EXIST ".\updater\PSMoveService\testing\" mkdir ".\updater\PSMoveService\testing\"
IF NOT EXIST ".\updater\PSMoveService\assets\calibration\" mkdir ".\updater\PSMoveService\assets\calibration\"

:: Copy misc calibration files to psmoveconfigtool.
xcopy /S /I /Y ".\misc\calibration" ".\updater\PSMoveService\assets\calibration"

:: Move files to updater.
copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\updater\PSMoveService\"
copy /Y ".\build\src\psmoveservice\Release\*.exe" ".\updater\PSMoveService\"
copy /Y ".\build\src\psmoveconfigtool\Release\*.exe" ".\updater\PSMoveService\"
xcopy /S /I /Y ".\src\psmoveconfigtool\assets" ".\updater\PSMoveService\assets"

:: Move testing binaries to testing directory.
copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\updater\PSMoveService\testing\"
copy /Y ".\build\src\tests\Release\*.exe" ".\updater\PSMoveService\testing\"

:: Copy LICENSE.
copy /Y ".\LICENSE" ".\updater\PSMoveService\LICENSE.txt"

:: Create zip file
.\7zip\7za.exe a -sfx7z_sfx ".\updater\PSMoveServiceUpdateSFX.dat" -w ".\updater\PSMoveService\*" 

:: Remove temp folder
rmdir /S /Q ".\updater\PSMoveService"

pause
goto exit

:no_build
echo No build directory found!
goto failure

:exit
EXIT /B 0

:failure
pause
EXIT /B 1