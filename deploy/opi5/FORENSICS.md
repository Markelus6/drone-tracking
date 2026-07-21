# SD forensics → reference snapshot

Полный разбор старой Raspberry Pi SD перенесён в:

**[`reference/rpi_interceptor/`](../../reference/rpi_interceptor/)**

Там plaintext полёта (`flight/app`, `config.ini`, `start.sh`), boot UART overlays, crypto/unlock, сеть.

## Bootfs (кратко)

```
dtoverlay=uart2   # ELRS CRSF → /dev/ttyAMA2
dtoverlay=uart4   # FC MSP   → /dev/ttyAMA4
enable_tvout=1
```

На OPi5 см. [WIRING.md](WIRING.md): uart1 / uart3 / pwm3.
