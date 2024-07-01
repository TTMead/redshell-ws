# Redshell Vision
Provides vision processing nodes for the redshell droid.

## Build
Install opencv with,
```
sudo apt update
sudo apt install libopencv-dev python3-opencv
```

Ensure the user is added to the "video" group with,
```
id -a
```

If you do not see a 'video' group, run,
```
sudo usermod -a -G video $LOGNAME
```