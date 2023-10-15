@echo off

:: Abort if theres no build available.
IF NOT EXIST build goto no_build

:: Remove old zip and temp folder file.
IF EXIST ".\release\PSMoveClient_CAPI.zip" del /S /Q ".\release\PSMoveClient_CAPI.zip"
IF EXIST ".\release\PSMoveClient_CAPI\" rmdir /S /Q ".\release\PSMoveClient_CAPI\"

:: Create temp folders.
IF NOT EXIST ".\release\PSMoveClient_CAPI\" mkdir ".\release\PSMoveClient_CAPI\"
IF NOT EXIST ".\release\PSMoveClient_CAPI\bin\" mkdir ".\release\PSMoveClient_CAPI\bin\"
IF NOT EXIST ".\release\PSMoveClient_CAPI\include\" mkdir ".\release\PSMoveClient_CAPI\include\"
IF NOT EXIST ".\release\PSMoveClient_CAPI\lib\" mkdir ".\release\PSMoveClient_CAPI\lib\"

:: Move binaries.
copy /Y ".\build\src\psmoveclient\Release\*.dll" ".\release\PSMoveClient_CAPI\bin\"

:: Move includes.
copy /Y ".\src\psmoveclient\ClientConstants.h" ".\release\PSMoveClient_CAPI\include\"
copy /Y ".\src\psmoveclient\ClientGeometry_CAPI.h" ".\release\PSMoveClient_CAPI\include\"
copy /Y ".\src\psmoveclient\PSMoveClient_CAPI.h" ".\release\PSMoveClient_CAPI\include\"
copy /Y ".\src\psmoveclient\PSMoveClient_export.h" ".\release\PSMoveClient_CAPI\include\"
copy /Y ".\src\psmoveprotocol\ProtocolVersion.h" ".\release\PSMoveClient_CAPI\include\"
copy /Y ".\src\psmoveprotocol\SharedConstants.h" ".\release\PSMoveClient_CAPI\include\"

:: Move libs.
copy /Y ".\build\src\psmoveclient\Release\*.lib" ".\release\PSMoveClient_CAPI\lib\"

:: Copy LICENSE.
copy /Y ".\LICENSE" ".\release\PSMoveClient_CAPI\LICENSE.txt"

:: Create zip file
.\7zip\7za.exe a -tzip ".\release\PSMoveClient_CAPI.zip" -w ".\release\PSMoveClient_CAPI" 

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