@echo off
rem Builds and runs the TrajectoryFit tests with whichever Visual C++ build tools are installed.
setlocal
set "HERE=%~dp0"
set "ROOT=%HERE%.."
call "%ROOT%\tools\vcvars.cmd" || exit /b 1
if not exist "%HERE%bin" mkdir "%HERE%bin"

cl /nologo /std:c++17 /EHsc /W4 /WX /permissive- /I"%ROOT%\standin" /I"%ROOT%" "%HERE%TrajectoryFitTests.cpp" "%ROOT%\TrajectoryFit.cpp" /Fo"%HERE%bin\\" /Fe"%HERE%bin\TrajectoryFitTests.exe" || exit /b 1
"%HERE%bin\TrajectoryFitTests.exe"
