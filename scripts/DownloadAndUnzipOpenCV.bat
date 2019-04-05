if not exist "..\deps\" mkdir ..\deps
pushd ..\deps
powershell -Command "Invoke-WebRequest https://astuteinternet.dl.sourceforge.net/project/opencvlibrary/opencv-win/3.4.3/opencv-3.4.3-vc14_vc15.exe -OutFile opencv-3.4.3-vc14_vc15.exe"
opencv-3.4.3-vc14_vc15.exe -y -gm2 -InstallPath="opencv"
del opencv-3.4.3-vc14_vc15.exe
popd