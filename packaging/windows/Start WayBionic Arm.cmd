@echo off
rem Starts the WayBionic ground station for the real arm. Plug in the carrier and the controller
rem first. To name the carrier's COM port: run this from a Command Prompt as "Start WayBionic Arm.cmd" COM5
call "%~dp0scripts\windows-arm.cmd" %*
echo.
pause
