
@echo off
setlocal

title CAN Monitor - Portable Builder

echo.
echo ============================================
echo          CAN MONITOR PORTABLE BUILDER
echo ============================================
echo.

REM ------------------------------------------------
REM Check Python
REM ------------------------------------------------
where py >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python launcher was not found.
    echo.
    echo Please install Python 3.x first.
    echo Make sure "Add Python to PATH" is enabled.
    echo.
    pause
    exit /b 1
)

echo [1/5] Checking Python...
py --version

REM ------------------------------------------------
REM Upgrade pip
REM ------------------------------------------------
echo.
echo [2/5] Installing required packages...
py -m pip install --upgrade pip

if errorlevel 1 (
    echo.
    echo [ERROR] pip update failed.
    pause
    exit /b 1
)

REM ------------------------------------------------
REM Install dependencies
REM ------------------------------------------------
py -m pip install pyserial pyinstaller

if errorlevel 1 (
    echo.
    echo [ERROR] Required packages could not be installed.
    pause
    exit /b 1
)

REM ------------------------------------------------
REM Check source file
REM ------------------------------------------------
echo.
echo [3/5] Checking source file...

if not exist "seriallogger.py" (
    echo.
    echo [ERROR] seriallogger.py was not found.
    echo.
    echo Put this build.bat in the same folder as:
    echo     seriallogger.py
    echo.
    pause
    exit /b 1
)

REM ------------------------------------------------
REM Clean previous build
REM ------------------------------------------------
echo.
echo [4/5] Cleaning previous build...

if exist "build" rmdir /s /q "build"
if exist "dist\CAN_Monitor" rmdir /s /q "dist\CAN_Monitor"
if exist "CAN_Monitor.spec" del /q "CAN_Monitor.spec"

REM ------------------------------------------------
REM Build portable folder
REM ------------------------------------------------
echo.
echo [5/5] Building portable application...
echo.

py -m PyInstaller ^
    --noconfirm ^
    --clean ^
    --windowed ^
    --onedir ^
    --name CAN_Monitor ^
    seriallogger.py

if errorlevel 1 (
    echo.
    echo ============================================
    echo              BUILD FAILED
    echo ============================================
    echo.
    pause
    exit /b 1
)

echo.
echo ============================================
echo              BUILD SUCCESSFUL
echo ============================================
echo.
echo Portable application:
echo.
echo     dist\CAN_Monitor\
echo.
echo Run:
echo.
echo     dist\CAN_Monitor\CAN_Monitor.exe
echo.
echo Copy the ENTIRE CAN_Monitor folder
echo to another Windows PC.
echo.
echo Python installation is NOT required.
echo ============================================
echo.

pause