@echo off
setlocal
if not defined VSCMD_VER (
    for /f "usebackq delims=" %%v in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do call "%%v\VC\Auxiliary\Build\vcvars64.bat" >nul
)
where cl >nul 2>nul || exit /b 1
cd /d "%~dp0..\.."
if not exist build\motor-tests mkdir build\motor-tests
set "OPTIONS=/nologo /std:c11 /utf-8 /W4 /WX /I Motor/tests /I ../Core/Inc /Fo:build/motor-tests/"
cl %OPTIONS% /Fe:build/motor-tests/test_pid.exe Motor/tests/test_pid.c ../Core/Src/pid.c || exit /b 1
build\motor-tests\test_pid.exe || exit /b 1
cl %OPTIONS% /Fe:build/motor-tests/test_motor.exe Motor/tests/test_motor.c ../Core/Src/motor.c ../Core/Src/pid.c || exit /b 1
build\motor-tests\test_motor.exe || exit /b 1
cl %OPTIONS% /DMOTOR_TEST_HAL_ONLY /Fe:build/motor-tests/test_encoder.exe Motor/tests/test_encoder.c Motor/tests/test_motor.c ../Core/Src/motor.c ../Core/Src/pid.c || exit /b 1
build\motor-tests\test_encoder.exe || exit /b 1
cl %OPTIONS% /Fe:build/motor-tests/test_command.exe Motor/tests/test_command.c ../Core/Src/motor_command.c ../Core/Src/pid.c || exit /b 1
build\motor-tests\test_command.exe || exit /b 1
cl /I Motor/tests/uart %OPTIONS% /Fe:build/motor-tests/test_uart.exe Motor/tests/test_uart.c ../Core/Src/motor_uart.c ../Core/Src/motor_command.c ../Core/Src/pid.c || exit /b 1
build\motor-tests\test_uart.exe || exit /b 1
exit /b 0
