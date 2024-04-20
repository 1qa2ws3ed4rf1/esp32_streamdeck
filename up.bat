git remote add origin https://github.com/1qa2ws3ed4rf1/esp32_streamdeck.git
git add .
set /p comm=Please input commit:
git commit -m "%comm%"
git pull
git push -u origin main
pause