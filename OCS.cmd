@echo off
rem ==================================================================
rem  OCS.cmd -- double-click: the Operator Control Station on THIS
rem  laptop, with the fleet map opened in the default browser.
rem
rem  Runs in: Windows. Two servers, each in its own visible window:
rem    "Fleet link"     FLEET_LINK.cmd - system python, owns UDP 14550
rem                     and COM8, feeds the map (127.0.0.1:8098/fleet)
rem    "OCS (ocs_web)"  python -m ocs_web from .venv - the RoboCommand
rem                     bridge and its pages on http://127.0.0.1:8099
rem  Either one that is already answering is left alone - no second
rem  copy. Close a window (or Ctrl+C in it) to stop that server.
rem
rem  The fleet link cannot start while QGroundControl holds UDP 14550
rem  or COM8; its window then says so and waits. The map still opens
rem  and shows "fleet_link not answering" until it is up.
rem
rem  Extra arguments go to ocs_web, e.g.
rem    OCS.cmd --local-broker            in-process team broker (bench)
rem    OCS.cmd --inject --local-broker   plus the fake-data buttons
rem  Set OCS_NO_FLEET=1 to skip the fleet link, OCS_NO_BROWSER=1 to
rem  skip opening the browser.
rem
rem  The probe is curl.exe and the windows are ordinary visible ones:
rem  this PC's malware protection deletes launchers that hide a
rem  background window or script web requests (2026-09-30).
rem ==================================================================
setlocal EnableDelayedExpansion
set "PORT=8099"
set "PREV="
rem --port N or --port=N (the for loop splits on = as well)
for %%a in (%*) do (
  if "!PREV!"=="--port" set "PORT=%%~a"
  set "PREV=%%~a"
)
set "PROBE=http://127.0.0.1:%PORT%/"

if not defined OCS_NO_FLEET call "%~dp0FLEET_LINK.cmd"

curl.exe -sf -o nul -m 1 "%PROBE%"
if not errorlevel 1 (
  echo The OCS is already running on port %PORT%.
  goto :open
)

if not exist "%~dp0.venv\Scripts\python.exe" goto :novenv

echo Starting the OCS server in its own window...
rem cmd /k keeps that window open if the server exits, so a startup
rem error (bad config, port in use) can be read instead of vanishing.
rem Started from this folder: bridge.toml and its run/ paths are relative.
pushd "%~dp0"
start "OCS (ocs_web)" cmd /k .venv\Scripts\python.exe -m ocs_web --config bridge.toml %*
popd

set "UP="
echo Waiting for it to answer on port %PORT% (up to 15 s)...
for /l %%n in (1,1,15) do if not defined UP call :probe
if not defined UP goto :noanswer

:open
if defined OCS_NO_BROWSER goto :done
start "" "http://127.0.0.1:%PORT%/map"
:done
exit /b 0

:novenv
echo.
echo *** %~dp0.venv is missing. Create it once, from the OCS folder:
echo         python -m venv .venv
echo         .venv\Scripts\python.exe -m pip install -r requirements.txt
echo.
pause
exit /b 2

:noanswer
echo.
echo *** Nothing answered on port %PORT% within 15 s. Read the window
echo     titled "OCS (ocs_web)" for the reason.
echo.
pause
exit /b 2

rem one round of the wait. ping is the one-second sleep: timeout.exe
rem refuses to run when stdin is not a console and would end the wait
rem instantly.
:probe
curl.exe -sf -o nul -m 1 "%PROBE%" && set "UP=1"
if not defined UP ping -n 2 127.0.0.1 >nul
exit /b 0
