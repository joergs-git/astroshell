@echo off
:: AstroShell Dome ASCOM Driver — Unregistration Script
:: Run as Administrator

echo ============================================
echo  AstroShell Dome ASCOM Driver - Uninstall
echo ============================================
echo.

:: Check for admin privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: This script must be run as Administrator.
    echo Right-click and select "Run as administrator".
    pause
    exit /b 1
)

:: Find regasm
set REGASM=%SystemRoot%\Microsoft.NET\Framework64\v4.0.30319\RegAsm.exe
if not exist "%REGASM%" (
    set REGASM=%SystemRoot%\Microsoft.NET\Framework\v4.0.30319\RegAsm.exe
)
if not exist "%REGASM%" (
    echo ERROR: RegAsm.exe not found.
    pause
    exit /b 1
)

set SCRIPTDIR=%~dp0

echo Unregistering ASCOM.AstroShellDome.dll ...
"%REGASM%" /unregister "%SCRIPTDIR%ASCOM.AstroShellDome.dll"
if %errorlevel% neq 0 (
    echo.
    echo WARNING: Unregistration may have failed. The driver might not have been registered.
    pause
    exit /b 1
)

echo.
echo SUCCESS: AstroShell Dome driver unregistered.
echo.
pause
