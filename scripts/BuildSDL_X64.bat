@echo off
setlocal

:: Add MSVC build tools to the path
call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64
IF %ERRORLEVEL% NEQ 0 (
  echo "Unable to initialize 64-bit visual studio build environment"
)

:: Compile the RELEASE|x64 build of SDL2
echo "Creating SDL2 project files..."
pushd ..\thirdparty\SDL2
del /f /s /q build > nul
rmdir /s /q build
mkdir build
pushd build
cmake .. -G "Visual Studio 15 2017 Win64" -DDIRECTX=OFF -DDIRECTX=OFF -DSDL_SHARED=ON -DSDL_STATIC=OFF -DFORCE_STATIC_VCRT=OFF -DEXTRA_CFLAGS="-Z7 -DSDL_MAIN_HANDLED -DWIN32 -DNDEBUG -D_CRT_SECURE_NO_WARNINGS
IF %ERRORLEVEL% NEQ 0 (
  echo "Error generating SDL2 project files!"
)

echo "Building SDL2 Release|x64..."
MSBuild.exe SDL2.sln /p:configuration=RELEASE /p:Platform="x64" /t:Clean;Build
IF %ERRORLEVEL% NEQ 0 (
  echo "Error building SDL2 Release|x64!"
)
popd
popd