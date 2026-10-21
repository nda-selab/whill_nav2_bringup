# Remote Control

## WHILL PC

### X11 VNC

```bash
x11vnc -display "$DISPLAY"   -rfbauth "$HOME/.vnc/passwd"   -localhost   -rfbport 5900   -forever   -shared
```


## Remote PC

### SSH

```bash
ssh -L 5901:127.0.0.1:5900 selab@192.168.179.12
```

### Remmina

```bash
remmina
```

- VNC
- 127.0.0.1:59001
