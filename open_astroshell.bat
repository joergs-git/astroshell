@echo off
REM ============================================================================
REM open_astroshell.bat - Open Astroshell Dome via NINA Sequence Instruction
REM ============================================================================
REM
REM Part of: https://github.com/joergs-git/astroshell
REM Copyright (c) 2025 joergsflow
REM
REM Opens both dome halves (West first, then East) by sending HTTP commands
REM to the Astroshell controller at 192.168.1.177. Sends a Pushover
REM notification upon completion.
REM
REM Usage: Called as an external script instruction within a NINA sequence,
REM        typically at the start of an imaging session.
REM
REM Astroshell HTTP Endpoints:
REM   $1 = Close East    $2 = Open East
REM   $3 = Close West    $4 = Open West
REM   $5 = Stop
REM
REM Requirements: curl (included in Windows 10+)
REM ============================================================================
REM --- Open West half first ---
echo [%date% %time%] Opening West...
curl -s -X GET http://192.168.1.177/$4 >nul 2>&1
REM --- Wait for West to clear before opening East ---
timeout /t 3 /nobreak >nul
REM --- Open East half ---
echo [%date% %time%] Opening East...
curl -s -X GET http://192.168.1.177/$2 >nul 2>&1
REM --- Send Pushover notification ---
echo [%date% %time%] Sending notification...
curl -s ^
  --form-string "token=YOURPUSHOVERTOKEN" ^
  --form-string "user=YOURPUSHOVERUSER" ^
  --form-string "title=NINA open dome Instruction" ^
  --form-string "message=Astroshell opening (West+East)" ^
  --form-string "url=http://YOURWHATEVERURLYOULIKE" ^
  --form-string "priority=1" ^
  https://api.pushover.net/1/messages.json >nul 2>&1
echo [%date% %time%] Done.
exit /b 0
