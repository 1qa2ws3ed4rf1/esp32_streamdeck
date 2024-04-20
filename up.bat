git remote add origin https://github.com/1qa2ws3ed4rf1/esp32_streamdeck.git
set /p mode = mode: 1=add all thing 2=input your command(like be git add streamdeck.ino)
git add .
set /p comm=Please input commit:
git commit -m "%comm%"
git pull
git push -u origin main
pause