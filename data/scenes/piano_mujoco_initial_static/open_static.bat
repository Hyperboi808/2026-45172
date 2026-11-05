@echo off
pushd "%~dp0"
python view_static.py
if errorlevel 1 pause
popd
