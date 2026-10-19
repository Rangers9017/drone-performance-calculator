@echo off
title DroneCalc Pro
cd /d "%~dp0"
py -3 drone_calc.py
if errorlevel 1 (
    echo.
    echo DroneCalc could not start. Install Python 3 and the packages in requirements.txt.
    pause
)

