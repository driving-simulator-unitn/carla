REM @echo off
setlocal

rem BAT script that downloads and installs a ready to use
rem FlatBuffers interfaces for the ZMQ communication with the Simulator.
rem Run it through a cmd with the x64 Visual C++ Toolset enabled.

set LOCAL_PATH=%~dp0
set FILE_N=    -[%~n0]:

rem Print batch params (debug purpose)
echo %FILE_N% [Batch params]: %*

rem ============================================================================
rem -- Parse arguments ---------------------------------------------------------
rem ============================================================================

:arg-parse
if not "%1"=="" (
    if "%1"=="--build-dir" (
        set BUILD_DIR=%~dpn2
        shift
    )
    if "%1"=="-h" (
        goto help
    )
    if "%1"=="--help" (
        goto help
    )
    shift
    goto :arg-parse
)
rem If not set set the build dir to the current dir
if "%BUILD_DIR%" == "" set BUILD_DIR=%~dp0
if not "%BUILD_DIR:~-1%"=="\" set BUILD_DIR=%BUILD_DIR%\

rem ============================================================================
rem -- Get FlatBuffers interfaces
rem ============================================================================
set FB_REPO=https://github.com/DRIVEWISE/DrivingSimulatorInterfaces.git
set FB_BASENAME=fb_interfaces
set FB_INSTALL_DIR=%BUILD_DIR%%FB_BASENAME%-install

if not exist %FB_INSTALL_DIR% (
    echo %FILE_N% Cloning FB.
    call git clone --depth 1 %FB_REPO% %FB_INSTALL_DIR%
) else (
    goto already_build
)

goto success

rem ============================================================================
rem -- Messages and Errors -----------------------------------------------------
rem ============================================================================

:help
    echo %FILE_N% Download and install a the FlatBuffers interfaces.
    goto eof
:already_build
    echo %FILE_N% A FlatBuffers interfaces installation already exists.
    echo %FILE_N% Delete "%FB_INSTALL_DIR%" if you want to force a rebuild.
    goto good_exit
:success
    echo.
    echo %FILE_N% FlatBuffers interfaces has been successfully installed in "%FB_INSTALL_DIR%"!
    goto good_exit

:good_exit
    echo %FILE_N% Exiting...
    rem A return value used for checking for errors
    endlocal & set install_fb_interfaces=%FB_INSTALL_DIR%
    exit /b 0

