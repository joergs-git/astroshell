@echo off
:: AstroShell Dome ASCOM Driver — Registration Script
:: Run as Administrator

echo ============================================
echo  AstroShell Dome ASCOM Driver - Install
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

:: Find regasm from .NET Framework 4.x (64-bit)
set REGASM=%SystemRoot%\Microsoft.NET\Framework64\v4.0.30319\RegAsm.exe
if not exist "%REGASM%" (
    set REGASM=%SystemRoot%\Microsoft.NET\Framework\v4.0.30319\RegAsm.exe
)
if not exist "%REGASM%" (
    echo ERROR: RegAsm.exe not found. Is .NET Framework 4.x installed?
    pause
    exit /b 1
)

:: Get script directory
set SCRIPTDIR=%~dp0

echo Registering ASCOM.AstroShellDome.dll ...
"%REGASM%" /codebase "%SCRIPTDIR%ASCOM.AstroShellDome.dll"
if %errorlevel% neq 0 (
    echo.
    echo ERROR: Registration failed. Check that the DLL exists and ASCOM Platform is installed.
    pause
    exit /b 1
)

echo.
echo SUCCESS: AstroShell Dome driver registered.
echo You can now select "AstroShell Dome" in NINA under Equipment ^> Dome.
echo.
pause
