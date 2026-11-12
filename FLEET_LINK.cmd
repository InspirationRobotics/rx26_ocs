@echo off
rem FLEET_LINK.cmd -- start the OCS fleet link in its own window, unless it is already up.
rem
rem   FLEET_LINK.cmd                        the usual: COM8, UDP 14550, status on 8098
rem   FLEET_LINK.cmd --serial COM9          any fleet_link option is passed straight through
rem   FLEET_LINK.cmd --sim-boat HOST:PORT   SIMULATION ONLY: mirror the radio to the WSL2 Crusader sim,
rem                                         whose replies are then TRANSMITTED ON THE REAL RADIO. Never
rem                                         at competition. HOST comes from: python -m fleet.wsl_ip
rem                                         (add --sim-boat-rx-only to keep the sim off the air).
rem                                         fleet\README.md "Sim boat on the radio"
rem Options only count on a FRESH start: if fleet_link already answers, this file says so and
rem exits, and the options you gave are NOT applied.
rem
rem "Already running" means something answers GET /fleet on the DEFAULT status port,
rem 8098, so if you start it with --status-port run python -m fleet.fleet_link yourself.
rem The check is curl.exe, which ships with Windows 10 and later. Keep this file plain cmd:
rem this PC's antivirus deletes .cmd files that call out to a script host or launch hidden
rem processes, so the only launcher here is cmd's own `start`, in a visible window.
rem Uses the OCS .venv, the same reproducible laptop environment OCS.cmd uses.
rem A historical sibling .deps folder remains a development fallback for this checkout.
setlocal
curl.exe -s -f -m 2 -o NUL http://127.0.0.1:8098/fleet
if not errorlevel 1 (
    echo Fleet link is already running - http://127.0.0.1:8098/fleet answers.
    if not "%~1"=="" echo The options you gave were NOT applied. Stop it with ctrl-C in its window, then run this again.
    exit /b 0
)
set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"
if exist "%~dp0..\competition-comms\.deps\pymavlink" set "PYTHONPATH=%~dp0..\competition-comms\.deps;%PYTHONPATH%"
"%PY%" -c "import pymavlink, serial" >NUL 2>&1
if errorlevel 1 (
    echo.
    echo The OCS Python cannot import pymavlink and pyserial, so fleet_link cannot run.
    echo Create/update this repo's environment once:
    echo   python -m venv .venv
    echo   .venv\Scripts\python.exe -m pip install -r requirements.txt
    echo.
    pause
    exit /b 1
)
pushd "%~dp0"
set FLEET_LINK_PAUSE=1
start "Fleet link" "%PY%" -m fleet.fleet_link --config fleet\fleet.toml %*
popd
exit /b 0
