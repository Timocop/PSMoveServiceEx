@echo off

IF NOT EXIST build goto no_build

IF EXIST ".\release\PSMoveService\" rmdir /S /Q ".\release\PSMoveService\"

IF NOT EXIST ".\release\PSMoveService\" mkdir ".\release\PSMoveService\"
IF NOT EXIST ".\release\PSMoveService\testing\" mkdir ".\release\PSMoveService\testing\"

copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\release\PSMoveService\"
copy /Y ".\build\src\psmoveservice\Release\*.exe" ".\release\PSMoveService\"
copy /Y ".\build\src\psmoveconfigtool\Release\*.exe" ".\release\PSMoveService\"
xcopy /S /I /Y ".\src\psmoveconfigtool\assets" ".\release\PSMoveService\assets"

copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\release\PSMoveService\testing\"
copy /Y ".\build\src\tests\Release\*.exe" ".\release\PSMoveService\testing\"

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