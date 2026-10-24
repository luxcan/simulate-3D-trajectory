@echo off
rem Builds FitCsv and the sample generator, writes the sample inputs, then fits each one from the
rem launch point in Singapore to its impact point. Inputs and outputs land next to this script.
setlocal
set "HERE=%~dp0"
set "ROOT=%HERE%.."
call "%ROOT%\tools\vcvars.cmd" || exit /b 1
if not exist "%HERE%bin" mkdir "%HERE%bin"

set "FLAGS=/nologo /std:c++17 /EHsc /W4 /WX /permissive- /I"%ROOT%\standin" /I"%ROOT%" /I"%ROOT%\app" /I"%ROOT%\tests" /Fo"%HERE%bin\\""
cl %FLAGS% "%ROOT%\app\FitCsv.cpp" "%ROOT%\app\TrajectoryCsv.cpp" "%ROOT%\TrajectoryFit.cpp" /Fe"%HERE%bin\FitCsv.exe" || exit /b 1
cl %FLAGS% "%HERE%MakeSamples.cpp" "%ROOT%\app\TrajectoryCsv.cpp" /Fe"%HERE%bin\MakeSamples.exe" || exit /b 1

"%HERE%bin\MakeSamples.exe" "%HERE%." || exit /b 1
for /f "usebackq tokens=1-8" %%a in ("%HERE%cases.txt") do (
    "%HERE%bin\FitCsv.exe" "%HERE%%%a" "%HERE%%%b" %%c %%d %%e %%f %%g %%h || exit /b 1
)
