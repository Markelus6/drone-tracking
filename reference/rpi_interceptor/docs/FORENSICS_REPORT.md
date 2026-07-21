# Полный отчёт: forensics старой SD / Raspberry Pi (Гипертех → OPi5)

Дата: 2026-07-22  
Цель: вытащить код управления полётом (CRSF→MSP, PID, servo, OSD) со старой microSD и перенести логику на Orange Pi 5.

---

## 1. Вердикт

| Вопрос | Ответ |
|--------|--------|
| Есть ли на карте код управления полётом? | **Да**, в `/opt/data/` |
| Можем ли прочитать его сейчас? | **Нет** — fscrypt (Adiantum), нужен ключ |
| Нашли ли половину ключа? | **Да** — `sk` в `/etc/fscrypt.key` |
| SoC serial (`sn`) | **Есть** — Data Matrix: `0000011051063167` (`rpi_serial.txt`) |
| keyfile fscrypt | **Собран** — `fscrypt.keyfile` = interleave(sn[:16], sk[:16]) |
| Что делать дальше без расшифровки? | Продолжать на PDF + уже написанный `chase_fc` |

Фото, которое прислали: **Raspberry Pi 4 Model B** (не Orange Pi). Маркировки KCC `R-C-P2R-RPI4B`, Anatel, CMIIT и т.п. — это **сертификаты**, не серийник чипа. Нужный `sn` живёт в OTP SoC и читается только так:

```bash
# на этой же живой Raspberry Pi 4:
cat /sys/firmware/devicetree/base/serial-number
# или:
cat /proc/cpuinfo | grep Serial
```

QR-наклейка `04/08` рядом с microSD могла бы содержать серийник — с фото автоматически не декодировалась; можно просканировать телефоном.

---

## 2. Железо и проводка (из bootfs)

Снято с FAT `boot/` (`legacy_rpi_sd/boot/`):

**`config.txt` (важное):**
- `dtoverlay=uart2` — ELRS CRSF (GPIO0/1 ≈ pins 27/28)
- `dtoverlay=uart4` — FC MSP (GPIO8/9 ≈ pins 24/21)
- `enable_uart=1`
- `enable_tvout=1`, `sdtv_mode=2` — композит на Jack (видео оператору через FC/VTX путь на RPi)
- `arm_64bit=1`

**`cmdline.txt`:** root ext4 `PARTUUID=9e020a5c-02`, hostname-образ Raspberry Pi reference **2025-11-24**.

Это совпадает со схемой PDF «Гипертех»: ELRS→SBC→MSP→Betaflight, аналоговое видео не через Ethernet.

---

## 3. Rootfs (образ и выгрузки)

| Артефакт | Путь | Размер / статус |
|----------|------|-----------------|
| Raw dump раздела 2 | `C:\fpv4win\sd_rootfs.img` | ~5.5 GiB, ext4, volume `rootfs` |
| Читаемое дерево | `legacy_rpi_sd/full_extract/` | ~170 MB |
| Ранее | `rootfs_extract/`, `wsl_extract/` | дубли/черновики |
| WSL | Ubuntu-24.04 **WSL1** | без BIOS virt; `wsl --mount` недоступен |

**Файловая система:** ext4 с флагом `encrypt` (fscrypt).  
**Hostname:** `raspberrypi`.  
**machine-id:** `5fe1696c197b4a39b6668d9b6ea8962d`.

---

## 4. Где лежит код полёта (`/opt/data`)

Зашифрованный каталог (имена тоже Adiantum):

| inode | размер | mode | вероятная роль |
|------:|-------:|------|----------------|
| 20511 | 498 B | 755 | мелкий файл (скрипт/мета) |
| 20179 | 2 366 128 B | 755 | основной бинарь |
| 20180 | 598 B | 644 | конфиг |

После успешного unlock бинарь `unlock` ожидает и запускает **`/opt/data/start.sh`**.

Ciphertext сохранён:  
`legacy_rpi_sd/full_extract/opt/data_CIPHERTEXT/`  
(и копии в `wsl_extract/opt/data_raw/`).

Снаружи `/opt/data` полезного автопилота **нет**:
- `/home/user`, `/root` — только shell-доты
- `/opt/opencv` — библиотеки OpenCV 4.13 (не chase)
- `/opt/pigpio` — почти пусто
- отдельных unit’ов `chase.service` / исходников MSP в открытом виде нет

---

## 5. Шифрование и ключи

### 5.1. Конфиг
`/etc/fscrypt.conf`:
- `source: raw_key`
- contents + filenames: **Adiantum**
- policy v2

Protector name в метаданных: `data` (id `901d29317cad5cb7`).

### 5.2. Сервис
`unlock.service` → `ExecStart=/usr/local/bin/unlock` (oneshot, до сети).

### 5.3. Бинарь `unlock`
- ELF **aarch64**, static, **not stripped**, ~2.3 MB  
- Путь: `full_extract/usr/local/bin/unlock`  
- Дизассембл `main` показал алгоритм:

1. Читает **sn** из  
   `/sys/firmware/devicetree/base/serial-number`  
   (длина ≥ 16)
2. Читает **sk** из  
   `/etc/fscrypt.key`  
   (длина ≥ 16)
3. Пишет keyfile: **чередование байт** `sn[i], sk[i], …` (32 байта) → `/tmp/fscrypt.key`
4. Вызывает `fscrypt unlock /opt/data --key=…`
5. Дальше пути к `/opt/data/start.sh` (и при ошибках — wipe/mkdir ветки)

### 5.4. Что уже есть
**sk** (файл на SD, 32 hex-символа):

```
fd128c4509f91ea4d40ed74ed66eb758
```

Сохранён: `legacy_rpi_sd/full_extract/etc/fscrypt.key`

**sn** — **не найден** ни на SD, ни на шелкографии платы на фото.

---

## 6. Сеть / прочее с карты

Wi‑Fi AP (NetworkManager), файл  
`full_extract/etc/NetworkManager/system-connections/swc.nmconnection`:
- mode: **AP**, hidden SSID
- SSID: `b1052352G`
- IP: `10.12.14.1/24`
- PSK: в том же файле (не дублируем здесь лишний раз)

Логи: установка пакета `fscrypt` (фев 2026), без PID/UART чисел автопилота.

---

## 7. Окружение Windows / WSL

- DISM: WSL + VirtualMachinePlatform включены  
- BIOS virt у ноутбука (Insyde / ICL P1511) **выключена** → только **WSL1**  
- Ubuntu 24.04 импортирован как WSL1  
- Выгрузка rootfs: `debugfs` / Python `ext4` (fuse2fs/loop на WSL1 не работают)

---

## 8. Связь с текущим проектом OPi5

Уже в репозитории (ветка interceptor), без старого кода:
- `control/chase_fc` — CRSF→MSP, engage CH8, PID, servo PWM, MSP DisplayPort OSD  
- `deploy/chase.json`, `deploy/opi5/WIRING.md` (uart1/uart3/pwm3 на 26-pin)  
- `start_tracking.sh` + `DRON_ENABLE_CHASE=1`

Схема контракта совпадает с PDF/bootfs RPi (ELRS→SBC→MSP→BF; видео оператору через FC OSD, не Pi→VTX).

Числа PID/углов серво со старой SD **ещё не сверены** — они внутри `/opt/data`.

---

## 9. Как открыть `/opt/data` (когда будет sn)

На Linux с поддержкой fscrypt (лучше Live USB / настоящий Linux; WSL2 с virt):

```bash
# 1) смонтировать sd_rootfs.img (loop) или SD p2
# 2) sn с платы:
SN=$(tr -d '\0' </sys/firmware/devicetree/base/serial-number)  # на живой Pi
SK=$(cat /path/to/full_extract/etc/fscrypt.key)
# 3) собрать keyfile чередованием (как unlock):
python3 - <<PY
sn, sk = "$SN", "$SK"
# взять по 16 символов (как в unlock: len>=16, буфер 32)
sn, sk = sn[:16], sk[:16]
key = "".join(a+b for a,b in zip(sn, sk))
open("/tmp/fscrypt.key","w").write(key)
print(key, len(key))
PY
# 4)
sudo fscrypt unlock /mnt/sdroot/opt/data --key=/tmp/fscrypt.key
# 5) скопировать /mnt/sdroot/opt/data → legacy_rpi_sd/opt_data_PLAIN/
```

Точная длина/нормализация sn (нулевой терминатор, leading zeros) может чуть отличаться — при неудаче подобрать padding по `error: sn < 16` / длине строки с живой Pi.

---

## 10. Следующие шаги (практика)

1. **Просканировать QR** на нижней стороне Pi телефоном → прислать текст.  
2. Или **включить эту Pi** с любой SD / той же картой и выполнить:
   `cat /sys/firmware/devicetree/base/serial-number`  
3. Прислать sn → расшифруем `/opt/data` и вытащим PID/UART/углы в `chase.json`.  
4. Пока sn нет — продолжать отладку стека на OPi5 по текущему коду и PDF.

---

## 11. Карта артефактов в репо

```
legacy_rpi_sd/
  README.md              — краткий статус
  REPORT.md              — этот отчёт
  boot/                  — FAT bootfs (uart2/uart4, tvout)
  full_extract/          — полный читаемый dump + sk + ciphertext
  rootfs_extract/        — более ранняя частичная выгрузка
  wsl_extract/           — выгрузка через WSL/debugfs
C:\fpv4win\
  sd_rootfs.img          — полный raw rootfs
```
