@echo off
rem 曲谱服务器：把本目录里的 .json 曲谱通过局域网 HTTP 发出去，
rem 手机 App 的「从网址导入」直接填下面打印出来的网址就能下载。
rem
rem 用途：演示「在线曲谱下载」时不依赖任何外网服务，断网也能演。
rem 前提：手机和电脑在同一个 WiFi 下。
rem
rem 这个文件必须保持纯 ASCII —— cmd.exe 遇到中文注释会因代码页乱码，
rem 把 set / for 行解析坏（表现是 'xxx' is not recognized）。

setlocal enabledelayedexpansion
set PY=
if exist "%~dp0..\..\python\python.exe" set PY=%~dp0..\..\python\python.exe
if "!PY!"=="" (
  for /f "delims=" %%i in ('where python 2^>nul') do if "!PY!"=="" set PY=%%i
)
if "!PY!"=="" (
  echo.
  echo   [X] no python found. Install python or edit this file to set PY=
  echo.
  pause
  exit /b 1
)

echo.
echo  ============================================
echo   Score server - keep this window OPEN
echo  ============================================
echo.
echo  Put one of these URLs into the app's "from URL" box:
echo.

for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /c:"IPv4"') do (
  set IP=%%a
  set IP=!IP: =!
  if not "!IP!"=="" (
    echo     http://!IP!:8000/qingtian.json
    echo     http://!IP!:8000/huahai.json
    echo     http://!IP!:8000/tuihou.json
  )
)

echo.
echo  (Ctrl+C in this window to stop the server)
echo.

cd /d "%~dp0"
"!PY!" -m http.server 8000 --bind 0.0.0.0

pause
