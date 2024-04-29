REM @echo off
setlocal

rem BAT script that downloads and installs a ready to use
rem x64 chrono for CARLA (carla.org).
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
rem -- Get Libzmq
rem ============================================================================
set ZMQ_TAG=v4.3.5
set ZMQ_REPO=https://github.com/zeromq/libzmq.git
set ZMQ_BASENAME=zmq-%ZMQ_TAG%
set ZMQ_SRC_DIR=%BUILD_DIR%%ZMQ_BASENAME%-src
set ZMQ_INSTALL_DIR=%BUILD_DIR%%ZMQ_BASENAME%zmq-install
set ZMQ_BUILD_DIR=%ZMQ_SRC_DIR%\build


if not exist %ZMQ_INSTALL_DIR% (
    echo %FILE_N% Cloning ZMQ.
    call git clone --depth 1 --branch %ZMQ_TAG% %ZMQ_REPO% %ZMQ_SRC_DIR%

    mkdir %ZMQ_BUILD_DIR%
    mkdir %ZMQ_INSTALL_DIR%

    cd "%ZMQ_BUILD_DIR%"

    echo %FILE_N% Compiling Chrono.
    echo %FILE_N% ZMQ_INSTALL_DIR: %ZMQ_INSTALL_DIR%
    echo %FILE_N% ZMQ_SRC_DIR: %ZMQ_SRC_DIR%


    cmake -DCMAKE_CONFIGURATION_TYPES=Release^
         -DBUILD_TESTS=OFF^
         -DWITH_DOCS=OFF^
         -DWITH_TLS=OFF^
         -DZMQ_BUILD_TESTS=OFF^
         -DBUILD_SHARED=ON^
         -DBUILD_STATIC=OFF^
         -DCMAKE_INSTALL_PREFIX="%ZMQ_INSTALL_DIR%"^
         ..

    echo %FILE_N% Building...
    cmake --build . --config Release --target install

) else (
    goto already_build
)

goto success

rem ============================================================================
rem -- Messages and Errors -----------------------------------------------------
rem ============================================================================

:help
    echo %FILE_N% Download and install a the zmq library.
    goto eof
:already_build
    echo %FILE_N% A LibZMQ installation already exists.
    echo %FILE_N% Delete "%ZMQ_INSTALL_DIR%" if you want to force a rebuild.
    goto good_exit
:success
    echo.
    echo %FILE_N% LibZMQ has been successfully installed in "%ZMQ_INSTALL_DIR%"!
    goto good_exit

:good_exit
    echo %FILE_N% Exiting...
    rem A return value used for checking for errors
    endlocal & set install_libzmq=%ZMQ_INSTALL_DIR%
    exit /b 0

