# Redshell Bringup
Provides launch scripts and core interfacing nodes for redshell droid.

## Common Issues
If you encounter a "Permission denied" error when attempting to access the serial port, ensure that the current user is a part of the dialout group with,

```
sudo adduser $USER dialout
```
