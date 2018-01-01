where /r "%LOCALAPPDATA%\Temp" *.ino.bin > file.locations
cat file.locations
for /f "tokens=*" %%a in (file.locations) DO xcopy "%%a" "images\%%~pa\"

where /r C:\Users\MikeD\AppData\Local\Temp\ *.ino.bin > file.locations
cp --target-directory=images

find C:\\Users\\MikeD\\AppData\\Local\\Temp\\ -name "*.ino.bin" -print | cp --target-directory=images