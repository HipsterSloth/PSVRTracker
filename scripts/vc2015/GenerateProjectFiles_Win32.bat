@echo off

IF NOT EXIST ..\..\build mkdir ..\..\build
cd build

echo "Rebuilding PSVRTracker Project files..."
cmake .. -G "Visual Studio 14 2015" 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating PSVRTracker 32-bit project files"
)
