@echo off
powershell.exe -NoProfile -File "%~dp0Start-FollowDemo.ps1"
if errorlevel 1 pause
