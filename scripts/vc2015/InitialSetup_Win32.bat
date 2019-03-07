@echo off
setlocal

::Clean up the old PSMoveService build folder
IF EXIST build (
del /f /s /q ..\..\build > nul
rmdir /s /q ..\..\build
)

::Clean up the old PSMoveService deps folder
IF EXIST deps (
del /f /s /q ..\..\deps > nul
rmdir /s /q ..\..\deps
)

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 32-bit visual studio build environment"
)

:: Compile the DEBUG|Win32 and RELEASE|Win32 builds of libusb
pushd ..\..\thirdparty\libusb\msvc\
echo "Building libusb DEBUG|Win32..."
MSBuild.exe libusb_2015.sln /p:configuration=DEBUG /p:Platform="Win32" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb DEBUG|Win32!"
) 
echo "Building libusb RELEASE|Win32..."
MSBuild.exe libusb_2015.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building libusb RELEASE|Win32!"
)
popd

:: Compile the RELEASE|Win32 build of SDL2
echo "Creating SDL2 project files..."
pushd ..\..\thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 14 2015" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating SDL2 project files!"
)

echo "Building SDL2 Release|Win32..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="Win32" /t:Clean
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2 Release|Win32!"
)
MSBuild.exe SDL2-static.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2-static Release|Win32!"
)
MSBuild.exe SDL2main.vcxproj /p:configuration=RELEASE /p:Platform="Win32" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2main Release|Win32!"
)
popd
popd

:: Generate the project files for PSMoveService
call GenerateProjectFiles_Win32.bat
