@echo off
rem Puts the Visual C++ build tools on PATH for the script that calls this one.
set "VCVARS="
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
"%VSWHERE%" -all -products * -find VC\Auxiliary\Build\vcvars64.bat > "%TEMP%\simulate3d-vcvars.txt"
set /p VCVARS=<"%TEMP%\simulate3d-vcvars.txt"
del "%TEMP%\simulate3d-vcvars.txt"
if not defined VCVARS (
    echo No Visual C++ build tools found.
    exit /b 1
)
rem vcvars64.bat looks for vswhere.exe on PATH for optional parts and complains when it is not there.
call "%VCVARS%" >nul 2>&1
where cl >nul 2>&1 || (
    echo vcvars64.bat did not put cl.exe on PATH.
    exit /b 1
)
exit /b 0
