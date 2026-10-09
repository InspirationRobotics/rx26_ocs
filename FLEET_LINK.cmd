@echo off
rem FLEET_LINK.cmd -- start the OCS fleet link in its own window, unless it is already up.
rem
rem   FLEET_LINK.cmd                        the usual: COM8, UDP 14550, status on 8098
rem   FLEET_LINK.cmd --serial COM9          any fleet_link option is passed straight through
rem
rem "Already running" means something answers GET /fleet on the DEFAULT status port,
rem 8098, so if you start it with --status-port run python -m fleet.fleet_link yourself.
rem The check is curl.exe, which ships with Windows 10 and later. Keep this file plain cmd:
rem this PC's antivirus deletes .cmd files that call out to a script host or launch hidden
rem processes, so the only launcher here is cmd's own `start`, in a visible window.
rem Needs the SYSTEM python (pymavlink and pyserial are installed there, not in .venv);
rem the check below says so in this window instead of letting the new one flash shut.
setlocal
curl.exe -s -f -m 2 -o NUL http://127.0.0.1:8098/fleet
if not errorlevel 1 (
    echo Fleet link is already running - http://127.0.0.1:8098/fleet answers.
    exit /b 0
)
python -c "import pymavlink, serial" >NUL 2>&1
if errorlevel 1 (
    echo.
    echo The python on PATH cannot import pymavlink and pyserial, so fleet_link cannot run.
    echo It needs the SYSTEM python - not OCS\.venv, and not a python that is not installed.
    echo.
    pause
    exit /b 1
)
pushd "%~dp0"
set FLEET_LINK_PAUSE=1
start "Fleet link" python -m fleet.fleet_link --config fleet\fleet.toml %*
popd
exit /b 0
