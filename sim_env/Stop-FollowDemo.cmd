@echo off
powershell.exe -NoProfile -File "%~dp0Stop-FollowDemo.ps1"
if errorlevel 1 pause
