@echo off
setlocal

::Clean up the old PSMoveService build folder
IF EXIST ..\..\build (
del /f /s /q ..\..\build > nul
rmdir /s /q ..\..\build
)

::Clean up the old PSMoveService deps folder
IF EXIST ..\..\deps (
del /f /s /q ..\..\deps > nul
rmdir /s /q ..\..\deps
)

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 64-bit visual studio build environment"
)

:: Compile the DEBUG|x64 and RELEASE|x64 builds of libusb
pushd ..\..\thirdparty\libusb\msvc\
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

:: Compile the RELEASE|x64 build of SDL2
echo "Creating SDL2 project files..."
pushd ..\..\thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 15 2017 Win64" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_STATIC=ON -DSDL_SHARED=OFF -DFORCE_STATIC_VCRT=ON -DEXTRA_CFLAGS="-MT -Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS -DHAVE_LIBC -D_USE_MATH_DEFINES
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating SDL2 project files!"
)

echo "Building SDL2 Release|x64..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="x64" /t:Clean
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2 Release|x64!"
)
MSBuild.exe SDL2-static.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build 
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2-static Release|x64!"
)
MSBuild.exe SDL2main.vcxproj /p:configuration=RELEASE /p:Platform="x64" /t:Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2main Release|x64!"
)
popd
popd

:: Generate the project files for PSMoveService
call GenerateProjectFiles_X64.bat