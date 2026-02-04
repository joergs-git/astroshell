@echo off
REM ============================================================================
REM close_astroshell.bat - Close Astroshell Dome via NINA Sequence Instruction
REM ============================================================================
REM
REM Part of: https://github.com/joergs-git/astroshell
REM Copyright (c) 2025 joergsflow
REM
REM Closes both dome halves (West first, then East) by sending HTTP commands
REM to the Astroshell controller at 192.168.1.177. Sends a Pushover
REM notification upon completion.
REM
REM Usage: Called as an external script instruction within a NINA sequence,
REM        typically triggered by safety events (e.g. power failure, weather).
REM
REM Astroshell HTTP Endpoints:
REM   $1 = Close East    $2 = Open East
REM   $3 = Close West    $4 = Open West
REM   $5 = Stop
REM
REM Requirements: curl (included in Windows 10+)
REM ============================================================================
REM --- Close West half first ---
echo [%date% %time%] Closing West...
curl -s -X GET http://192.168.1.177/$3 >nul 2>&1
REM --- Wait for West to clear before closing East ---
timeout /t 3 /nobreak >nul
REM --- Close East half ---
echo [%date% %time%] Closing East...
curl -s -X GET http://192.168.1.177/$1 >nul 2>&1
REM --- Send Pushover notification ---
echo [%date% %time%] Sending notification...
curl -s ^
  --form-string "token=YOURPUSHOVERTOKEN" ^
  --form-string "user=YOURPUSHOVERUSER" ^
  --form-string "title=NINA close dome Instruction" ^
  --form-string "message=Astroshell closing (West+East)" ^
  --form-string "url=http://YOURWHATEVERURLIFYOUHAVEONE" ^
  --form-string "priority=1" ^
  https://api.pushover.net/1/messages.json >nul 2>&1
echo [%date% %time%] Done.
exit /b 0
