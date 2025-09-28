@echo off
REM Run DMA tests and capture output
REM Usage: RUNTEST.BAT

echo DMA Board Test Results > testlog.txt
echo ====================== >> testlog.txt
echo. >> testlog.txt
echo Date: >> testlog.txt
date /t >> testlog.txt
echo Time: >> testlog.txt
time /t >> testlog.txt
echo. >> testlog.txt

echo Running DMA board tests...
echo Please wait...

REM Run the test program and append output to log
dmatest.exe >> testlog.txt

REM Check error level
if errorlevel 1 goto failed

echo.
echo Tests completed successfully!
echo Results saved to TESTLOG.TXT
goto end

:failed
echo.
echo Some tests failed!
echo Check TESTLOG.TXT for details

:end
echo.
type testlog.txt | more