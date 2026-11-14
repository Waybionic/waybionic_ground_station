@echo off
rem Checks that this laptop can start the ground station: Docker, the bundled Python and the
rem bridges. It starts nothing and does not touch the arm.
call "%~dp0scripts\windows-arm.cmd" -SelfTest
echo.
pause
