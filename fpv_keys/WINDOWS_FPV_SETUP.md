# Windows FPV приём (fpv4win / свой порт) — рабочая схема

Скопируй этот файл и папки на второй ноут. Ниже только то, что реально заработало.

## Что нужно на втором ноуте

| Компонент | Путь / значение |
|-----------|-----------------|
| Программа | `fpv4win` 0.0.5-beta (вся папка целиком, не один `.exe`) |
| Zadig | `zadig-2.9.exe` |
| USB-адаптер | Realtek **RTL8812AU**, VID:PID **`0bda:8812`** |
| Ключ GS | `gs.key` (64 байта) |
| Канал | **161** |
| Ширина канала | **20** MHz |
| Кодек в приёмнике | **H264** (не H265) |

Рабочая папка на первом ноуте:
`C:\Users\User\Downloads\fpv-setup\fpv4win\`

Скопируй на второй ноут всю папку:
- `fpv-setup\fpv4win\` (exe + все dll + `gs.key` + `config.ini`)
- `fpv-setup\zadig-2.9.exe`

Ключ также лежит в репозитории: `drone-tracking\fpv_keys\gs.key`  
MD5 заводского ключа: `24767056dc165963fe6db7794aee12cd`  
Он **должен совпадать** с `/etc/drone.key` на камере.

---

## Настройка Windows (один раз)

### 1. Zadig — драйвер для FPV (не обычный Wi‑Fi)

1. Вставь RTL8812AU в USB (лучше USB 3).
2. Запусти **Zadig** от администратора.
3. Options → **List All Devices**.
4. Выбери адаптер (**RTL8812AU** / `0bda:8812` / 802.11n NIC).
5. Справа выбери драйвер **libusbK** (как на рабочем ноуте).
6. **Install Driver** / Replace Driver.
7. Не ставь драйвер на мышь/клавиатуру и не ставь обычный Realtek Wi‑Fi драйвер для FPV — нужен именно WinUSB/libusbK для monitor/rx.

После Zadig адаптер **не будет** обычным Wi‑Fi в Windows — это нормально для FPV.

### 2. Запуск приёмника

1. Открой папку `fpv4win`.
2. Запусти **`fpv4win.exe`** из этой папки.
3. Настройки:
   - **RTL8812AU VID:PID:** `0bda:8812`
   - **Channel:** `161`
   - **Codec:** `H264`
   - **Channel Width:** `20`
   - **Key:** путь к `gs.key` из этой же папки
4. **Start** (кнопка станет STOP).
5. **Не нажимай MP4** — в 0.0.5-beta программа падает.

Ожидаемый результат: растёт `Packet(RTP/WFB/802.11)`, битрейт > 0, есть картинка.

Ручной путь запуска:
`C:\Users\User\Downloads\fpv-setup\fpv4win\fpv4win.exe`
(на втором ноуте — твой путь к скопированной папке).

---

## Настройки камеры OpenIPC (уже выставлены на рабочей камере)

Камера: `192.168.1.10`, SSH `root` / `12345`.

### `/etc/majestic.yaml` (важное)

```yaml
video0:
  enabled: true
  codec: h264          # обязательно h264 для fpv4win
  fps: 30
  bitrate: 1000
  size: 1280x720
  ...

outgoing:
  enabled: true
  naluSize: 1200       # КРИТИЧНО: не 3894
  wfb: true
```

После правок:
```sh
/etc/init.d/S95majestic restart
```

### `/etc/wfb.yaml` (важное)

```yaml
wireless:
  channel: 161
  width: 20
  ...
broadcast:
  mcs_index: 0         # устойчивее для ноутбука
  fec_k: 8
  fec_n: 12
  link_id: 7669206
```

Применить радио:
```sh
/etc/init.d/S98wifibroadcast stop
/etc/init.d/S98wifibroadcast start
```

Проверка TX (должно быть `-M 0` у video):
```sh
ps | grep wfb_tx
# wfb_tx ... -M 0 -B 20 -k 8 -n 12 ... -i 7669206 ...
```

Ключ на камере:
```sh
md5sum /etc/drone.key
# 24767056dc165963fe6db7794aee12cd
```

---

## Проблемы и решения (только рабочие)

### 1. Чёрный экран, `0bps`, в логе куча `Long packet (fec payload)`
**Проблема:** пакеты больше лимита приёмника → fpv4win их **отбрасывает** (это ошибка, не «шум»).  
**Причина:** `outgoing.naluSize: 3894` (или другой большой размер).  
**Решение:** `naluSize: 1200` в `/etc/majestic.yaml`, перезапуск majestic. После этого `Long packet` почти пропадает, появляется картинка.

### 2. H265: `Multi-layer HEVC coding is not implemented`
**Проблема:** FFmpeg внутри fpv4win не декодирует multi-layer HEVC с этой камеры.  
**Решение:** на камере `video0.codec: h264`, в программе **Codec = H264**. H265 в fpv4win не использовать.

### 3. Пакеты есть (`Packet` растёт), но `0bps` / `RTP: missed` / `max delay`
**Проблема:** кадры не собираются из‑за потерь или слишком больших UDP/NALU.  
**Решение:** `naluSize: 1200`, битрейт ~1000, `mcs_index: 0`, адаптер близко к камере, порт USB 3. Кодек H264.

### 4. Нет расшифровки / нет RTP (раньше было `0/xxxx/xxxx`)
**Проблема:** неверный `gs.key`.  
**Решение:** заводской `gs.key` из zip fpv4win = `/etc/drone.key` на камере (MD5 выше). Не подставлять кастомный `wfb_keygen`, пока не сгенерируешь пару и не пропишешь оба ключа.

### 5. Программа закрывается при записи
**Проблема:** баг кнопки **MP4** в fpv4win 0.0.5-beta.  
**Решение:** не нажимать MP4.

### 6. В логе `SwChnl false, _setChannelBw false`
**Проблема:** повторная установка канала драйвером; при живых пакетах обычно не мешает.  
**Решение:** если пакеты идут и есть картинка — игнорировать. Если пакетов нет — переткнуть USB, проверить Zadig/libusbK, канал 161 / width 20.

### 7. Адаптер не виден в fpv4win
**Проблема:** стоит обычный Realtek Wi‑Fi драйвер, а не libusbK/WinUSB.  
**Решение:** снова Zadig → libusbK на `0bda:8812`.

---

## Что встраивать в свою программу (тот же принцип)

Чтобы «как тут» работало в другом приложении с кодом fpv4win / wfb-ng:

1. **Тот же USB backend:** адаптер в режиме, который видит libusb (Zadig libusbK/WinUSB), VID:PID `0bda:8812`.
2. **Тот же ключ:** 64-байтный `gs.key`, MD5 `24767056...`, путь настраиваемый.
3. **Радио:** channel **161**, bandwidth **20**, link_id **7669206** (как на камере).
4. **FEC:** k=8, n=12 (как TX: `-k 8 -n 12`).
5. **Видео:** ожидать **H264 over RTP** после WFB decrypt+FEC; не H265.
6. **Лимит размера пакета:** RX отбрасывает oversized (`Long packet (fec payload)`). На TX `naluSize` ≤ ~1200…1400 под старый лимит forwarder в fpv4win.
7. **Не полагаться на MP4-запись** из старого UI, если это тот же код.

Минимальный чеклист «как на первом ноуте»:
- [ ] Zadig libusbK на `0bda:8812`
- [ ] Скопирована вся папка fpv4win + тот же `gs.key`
- [ ] Channel 161, Width 20, Codec H264
- [ ] На камере h264 + `naluSize: 1200` + mcs 0
- [ ] Start → есть RTP и bitrate, без лавины `Long packet`

---

## Файлы для копирования на второй ноут

```
fpv-setup/
  zadig-2.9.exe
  fpv4win/          ← целиком
    fpv4win.exe
    gs.key
    config.ini
    *.dll
    ...
```

`config.ini` (эталон):
```ini
[config]
channel=161
channelWidth=0
codec=H264
key=C:/Users/User/Downloads/fpv-setup/fpv4win/gs.key
```
На втором ноуте поправь только путь `key=` на локальный.

Камеру трогать не нужно, если это та же камера с уже выставленными настройками.
