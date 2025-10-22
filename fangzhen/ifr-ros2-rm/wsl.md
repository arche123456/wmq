# USB
https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb

## Windows 
```bash
usbipd list
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

## Ubuntu
```bash
lsusb
```

# PORT
https://blog.csdn.net/weixin_40466317/article/details/124355844
## Start
```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

## listen
```bash
cat < /dev/pts/xxxx
```