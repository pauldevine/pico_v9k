@echo off
REM Build script for DMA Test Program
REM Requires OpenWatcom C compiler

echo Building DMA Test Program...

REM Set OpenWatcom environment if not already set
if "%WATCOM%"=="" goto needwatcom

REM Compile the C file
echo Compiling dmatest.c...
wcc -bt=dos -ms -0 -os -zq -d0 dmatest.c
if errorlevel 1 goto error

REM Link the object file
echo Linking dmatest.exe...
wlink system dos name dmatest.exe file dmatest.obj
if errorlevel 1 goto error

echo Build successful!
echo Run DMATEST.EXE to test DMA registers
echo Run DMATEST.EXE -i for interactive mode
goto end

:needwatcom
echo ERROR: OpenWatcom environment not set
echo Please run OWSETENV.BAT or set WATCOM environment variable
goto end

:error
echo Build failed!

:end