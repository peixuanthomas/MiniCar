@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0serial_monitor.ps1" -Port COM4 -Baud 115200
pause
