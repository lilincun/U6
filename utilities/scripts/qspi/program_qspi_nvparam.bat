@SETLOCAL
@set on_off=OFF
@echo %on_off%
SETLOCAL ENABLEEXTENSIONS
SETLOCAL ENABLEDELAYEDEXPANSION
@rem store this script name for help
set thisfile=%~0
set this_folder=%~dp0

:more_options
shift
if "%~0"=="" goto :no_more_options
if /i "%~0"=="--id" (set DEVICE_ID=%1
    set id_arg=--id %1
    shift
    goto :more_options)
if /i "%~0"=="/?" goto :help
if /i "%~0"=="--help" goto :help
if /i "%~0"=="-h" goto :help

:no_more_options
if "" == "%verbose%" set verbose_on=%on_off%

@rem Setup SDK root
if defined SDKROOT goto :sdk_set

pushd %this_folder%\..\..\..
set SDKROOT=%CD%
popd
:sdk_set

call :banner "PROGRAMMING PARAMETERS"

set OUTPUT_ROOT=%~0
set PROJECT_ROOT=%OUTPUT_ROOT%/..
set NVPARAM_BIN=%OUTPUT_ROOT%/nvparam.bin

set CLI_PROGRAMMER=%SDKROOT%/binaries/cli_programmer.exe
@rem Find jlink install path from registry
for /F "tokens=2*" %%i in ('%windir%\system32\reg query HKCU\Software\SEGGER\J-Link /v InstallPath') do set jlink_path=%%j

set TEMPJSCRIPT=
set GDBPID=

if defined id goto id_selected
set id=
@REM Read attached Jlink devices
set jlink_output_file=%CD%\t
echo exit | "%jlink_path%JLink.exe" > "%jlink_output_file%"
for /f "tokens=2 SKIP=2" %%f in ('%windir%\system32\FIND.exe "S/N" "%jlink_output_file%"') do set id=%%f
@del "%jlink_output_file%"

if not defined id (
@echo No Jlink device found.
exit /b 1
)
set id_arg=--id %id%
:id_selected

call :banner "NV-Parameters PROGRAMMING via JTAG"

if "%~0" == "" (
  call :error "Project output path not specified"
  echo Usage %thisfile% ^<project output path^>
  exit /b 1
)

if not exist "%~0" (
  call :error "%~0 not found!"
  echo Please select the folder which contains the binary you want to program and try again.
  exit /b 1
)

:find_gcc
if exist "arm-none-eabi-gcc.exe" goto gcc_found
if exist "%ARM_TOOLCHAIN%/arm-none-eabi-gcc.exe" goto gcc_found
if exist "%ARM_TOOLCHAIN%/bin/arm-none-eabi-gcc.exe" goto gcc_found
  set /p ARM_TOOLCHAIN="Please enter GNU ARM Toolchain path > "
goto find_gcc

:gcc_found

call "%SDKROOT%\utilities\nvparam\create_nvparam.bat" "%OUTPUT_ROOT%" "%PROJECT_ROOT%\config" "%SDKROOT%/sdk/bsp/adapters/include"
if errorlevel 1 (
  call :error "Could not create nvparam.bin"
  exit /b 1
)

if defined DEVICE_ID goto program_params
set jlink_output_file=%CD%\t
echo exit | "%jlink_path%JLink.exe" > "%jlink_output_file%"
for /f "tokens=2 SKIP=2" %%f in ('%windir%\system32\FIND.exe "S/N" "%jlink_output_file%"') do set DEVICE_ID=%%f
@del "%jlink_output_file%"

:program_params
echo "Programming flash of device %DEVICE_ID%"

%windir%\system32\CScript /nologo "%SDKROOT%\utilities\scripts\qspi\prepare_local_ini_file.vbs" %DEVICE_ID%

echo "%CLI_PROGRAMMER%" gdbserver write_qspi 0x80000 "%NVPARAM_BIN%"
"%CLI_PROGRAMMER%" gdbserver write_qspi 0x80000 "%NVPARAM_BIN%"

call :banner "FINISHED PROGRAMMING NV PARAMETERS!"

exit /b 0

:error
@echo Error: %~1
@goto:eof

:banner
@if defined quiet goto :eof
@echo .......................................................................................................................
@echo ..
@echo .. %~1
@echo ..
@echo .......................................................................................................................
@goto:eof

:help
echo Usage:
echo "%thisfile% [-h] [--id serial_number] <project output path>"
echo    where:
echo      -h                  - prints this help
echo      --id serial_number  - serial number of jlink to connect to
echo.
echo      project output path - folder which contains the binary you want to program
exit /b 0

