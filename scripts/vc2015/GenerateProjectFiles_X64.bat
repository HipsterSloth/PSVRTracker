@echo off

IF NOT EXIST ..\..\build mkdir ..\..\build
pushd ..\..\build

echo "Rebuilding PSVRTracker x64 Project files..."
cmake .. -G "Visual Studio 14 2015 Win64"
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating PSVRTracker 64-bit project files"
)

popd
