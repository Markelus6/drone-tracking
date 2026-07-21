# Orange Pi 5 wiring (образ ~1.2.x / 1.3.x)

Роли как в PDF «Гипертех» + bootfs старого RPi (`uart2`=ELRS, `uart4`=FC).  
На **Orange Pi 5** разъём **26-pin**.

## Важно про видео / OSD

**Видео с Orange Pi на VTX не идёт.**  
Аналог: камера → FC (OSD) → VTX → очки.

Рамка трека рисуется в **OSD полётника** через тот же UART MSP (`MSP DisplayPort`): Pi шлёт координаты рамки, FC мешает их в картинку до VTX. Оператор видит рамку в очках.

Поток прицеливания:
1. В очках видна центральная рамка-прицел (aim).
2. Оператор заводит цель в рамку.
3. CH8 → **2000** (переключатель) → захват LightTrack → серво «в нос» → chase PID.
4. CH8 низко → сброс, серво ~30°.

## Overlays (`/boot/orangepiEnv.txt`)

```
overlays=uart1-m1 uart3-m0 pwm3-m0
```

```bash
sudo bash deploy/opi5/install_overlays.sh
# ВАЖНО: нужен полный reboot (не systemctl без пароля/PTY):
sudo shutdown -r now
# После загрузки:
ls /dev/ttyS1 /dev/ttyS3   # должны появиться
```

На образе 1.2.2 проверено: `overlays=uart1-m1 uart3-m0 pwm3-m0` → `/dev/ttyS1`, `/dev/ttyS3`, `pwmchip3`.

## Распиновка 26-pin

| Роль | OPi5 pin | Сигнал | Устройство |
|------|----------|--------|------------|
| ELRS 5V | 2 | 5V | ELRS VCC |
| ELRS GND | 6 | GND | ELRS GND |
| ELRS CRSF → Pi | 3 | UART1 RX | `/dev/ttyS1` |
| ELRS Pi → RX (опц.) | 5 | UART1 TX | `/dev/ttyS1` |
| FC MSP Pi TX | 19 | UART3 TX | `/dev/ttyS3` |
| FC MSP Pi RX | 21 | UART3 RX | `/dev/ttyS3` |
| Серво PWM | 15 | PWM3_M0 | sysfs pwmchip |
| AV in (трек) | USB AV | UVC | `/dev/video*` → orch |

UART **3.3V**. CRSF **420000**, MSP **115200**.

## Betaflight

- UART к Pi: **MSP** on (и DisplayPort / OSD canvas для внешней отрисовки, если требуется в версии BF).
- Все **Serial RX** off.
- Receiver = **MSP**.
