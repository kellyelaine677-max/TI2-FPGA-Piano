@echo off
rem TI2 Piano - one click debug build.
rem Keep this file ASCII-only: comments in Chinese get mangled by cmd.exe
rem (wrong code page) and the batch parser then fails on the set lines.
setlocal
set JAVA_HOME=D:\Android\android-studio\jbr
set ANDROID_HOME=D:\Android\Sdk
set ANDROID_SDK_ROOT=D:\Android\Sdk
set GRADLE_USER_HOME=D:\Android\.gradle

rem Use the locally installed Gradle 8.0 instead of the wrapper: the wrapper
rem re-downloads gradle-8.0-bin.zip from services.gradle.org every time and
rem that host times out from behind the GFW (10 s -> BUILD FAILED).
if exist "D:\Android\gradle-8.0\bin\gradle.bat" (
  call "D:\Android\gradle-8.0\bin\gradle.bat" -p "%~dp0" assembleDebug --console=plain
) else (
  call "%~dp0gradlew.bat" -p "%~dp0" assembleDebug --console=plain
)
if errorlevel 1 (
  echo BUILD FAILED
  exit /b 1
)

copy /Y "%~dp0app\build\outputs\apk\debug\app-debug.apk" "%~dp0Ti2Piano-debug.apk" >nul
echo.
echo BUILD OK -^> Ti2Piano-debug.apk
endlocal
