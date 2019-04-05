@echo off
setlocal

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 64-bit visual studio build environment"
)

:: Compile the DEBUG|x64 and RELEASE|x64 builds of libusb
pushd ..\thirdparty\libusb\msvc\
echo "Building libusb DEBUG|x64..."
MSBuild.exe libusb_2017.sln /tv:15.0 /p:configuration=DEBUG /p:Platform="x64" /t:Clean;Build 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb DEBUG|x64!"
)
echo "Building libusb RELEASE|x64..."
MSBuild.exe libusb_2017.sln /tv:15.0 /p:configuration=RELEASE /p:Platform="x64" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb RELEASE|x64!"
)
popd