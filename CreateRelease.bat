@echo off

IF NOT EXIST build goto no_build

IF EXIST ".\release\PSMoveService\" rmdir /S /Q ".\release\PSMoveService\"

IF NOT EXIST ".\release\PSMoveService\" mkdir ".\release\PSMoveService\"
IF NOT EXIST ".\release\PSMoveService\testing\" mkdir ".\release\PSMoveService\testing\"

IF NOT EXIST ".\release\PSMoveService\assets\calibration\" mkdir ".\release\PSMoveService\assets\calibration\"

:: Copy misc calibration files to psmoveconfigtool.
xcopy /S /I /Y ".\misc\calibration" ".\release\PSMoveService\assets\calibration"

:: Move files to release.
copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\release\PSMoveService\"
copy /Y ".\build\src\psmoveservice\Release\*.exe" ".\release\PSMoveService\"
copy /Y ".\build\src\psmoveconfigtool\Release\*.exe" ".\release\PSMoveService\"
xcopy /S /I /Y ".\src\psmoveconfigtool\assets" ".\release\PSMoveService\assets"

:: Move testing binaries to testing directory.
copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\release\PSMoveService\testing\"
copy /Y ".\build\src\tests\Release\*.exe" ".\release\PSMoveService\testing\"

:: Copy LICENSE.
copy /Y ".\LICENSE" ".\release\PSMoveService\LICENSE.txt"

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