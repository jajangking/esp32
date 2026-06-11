# Wiring Diagram — KEI Robot

## Pin Mapping ESP32 → Hardware

```
┌─────────────────────────────────────────────────────────┐
│                      ESP32 DOIT                         │
│                                                         │
│  GPIO 25 ─── PWMA  (TB6612 — Motor Kiri)               │
│  GPIO 26 ─── AIN1  (TB6612 — Motor Kiri)               │
│  GPIO 27 ─── AIN2  (TB6612 — Motor Kiri)               │
│  GPIO 13 ─── PWMB  (TB6612 — Motor Kanan)              │
│  GPIO 14 ─── BIN1  (TB6612 — Motor Kanan)              │
│  GPIO 33 ─── BIN2  (TB6612 — Motor Kanan)              │
│  GPIO 32 ─── STBY  (TB6612 — Standby)                  │
│                                                         │
│  GPIO  5 ─── Servo (signal)                             │
│  GPIO  4 ─── Buzzer Passive (signal)                    │
│  GPIO  2 ─── LED Status Indikator                       │
│                                                         │
│  GPIO 21 ─── SDA  (VL53L0X + MPU6050)                  │
│  GPIO 22 ─── SCL  (VL53L0X + MPU6050)                  │
│                                                         │
│  GPIO 15 ─── VL53L0X XSHUT (sensor enable)             │
│                                                         │
│  3.3V    ─── VCC TB6612, Servo, VL53L0X, MPU6050       │
│  GND     ─── GND semua modul                            │
│  VIN / 5V ─── (optional) input dari baterai via regulator│
└─────────────────────────────────────────────────────────┘
```

---

## TB6612FNG (Dual Motor Driver)

```
TB6612FNG
┌─────────────────────────────┐
│         TB6612              │
│  ┌─────────────────────┐    │
│  │ PWMA  ← GPIO 25     │    │
│  │ AIN1  ← GPIO 26     │    │
│  │ AIN2  ← GPIO 27     │    │
│  │ AO1   → Motor Kiri +│    │
│  │ AO2   → Motor Kiri -│    │
│  ├─────────────────────┤    │
│  │ PWMB  ← GPIO 13     │    │
│  │ BIN1  ← GPIO 14     │    │
│  │ BIN2  ← GPIO 33     │    │
│  │ BO1   → Motor Kanan +│    │
│  │ BO2   → Motor Kanan -│    │
│  ├─────────────────────┤    │
│  │ VM    ← Baterai (3–13V)   │
│  │ VCC   ← 3.3V (ESP32) │    │
│  │ GND   → GND          │    │
│  │ STBY  ← GPIO 32      │    │
│  └─────────────────────┘    │
└─────────────────────────────┘
```

**Catatan:**
- **STBY** dikontrol via GPIO 32 (HIGH = enable, LOW = disable). Firmware otomatis mengelola STBY.
- **VM** = tegangan motor (baterai). Range 3–13V. Contoh: 2× Li-ion (7.4V) atau 4× AA (6V).
- **VCC** = logik level (3.3V dari ESP32).
- Ground semua modul wajib disatukan (common ground).

### Tabel kontrol motor TB6612

| IN1 | IN2 | PWM | Motor |
|-----|-----|-----|-------|
| H   | L   | PWM | Maju  |
| L   | H   | PWM | Mundur|
| L   | L   | X   | Free  |
| H   | H   | X   | Brake |

---

## Passive Buzzer

```
Buzzer (passive)
┌──────────────────┐
│  (+)  ─── GPIO 4 │
│  (-)  ─── GND    │
└──────────────────┘
```

Ciri buzzer **passive**: suara bisa diubah frekuensinya via PWM.

---

## Servo (SG90 / MG90S / MG995)

```
Servo
┌──────────────┐
│ Coklat  ─ GND │
│ Merah   ─ 3.3V│  (atau 5V kalau dari VIN eksternal)
│ Kuning  ─  5  │
└──────────────┘
```

**⚠️** Untuk servo besar (MG995), jangan ambil daya dari 3.3V ESP32. Pakai VIN atau regulator 5V eksternal. Ground tetap disatukan.

---

## LED Status

```
LED
┌──────────────┐
│ (+)  ─── GPIO 2 │
│ (-)  ─── GND    │  (pakai resistor 220Ω–1kΩ)
└──────────────┘
```

Indikasi:
- **Nyala solid**: normal, WiFi connected
- **Kedip pelan** (600ms): mode explore
- **Kedip cepat** (120ms): emergency
- **Kedip** (200ms): WiFi tidak terhubung

---

## I2C — VL53L0X + MPU6050

```
3.3V ─── VCC (VL53L0X) ─── VIN (MPU6050)
GND  ─── GND (keduanya)
GPIO 21 ─── SDA (keduanya)
GPIO 22 ─── SCL (keduanya)
GPIO 15 ─── XSHUT (VL53L0X)
```

Keduanya memakai I2C yang sama secara paralel. Alamat default:
- VL53L0X: `0x29`
- MPU6050: `0x68`

**VL53L0X** — pin XSHUT dihubungkan ke GPIO 15 dan dikontrol firmware (HIGH = enable).

---

## Baterai

Rekomendasi:
- **Motor + TB6612**: 2S Li-ion (7.4V) atau 4× AA (6V) → VM
- **ESP32 + sensor + servo**: via regulator AMS1117 3.3V atau langsung pakai USB power bank
- **⚠️ Jangan sambung** baterai langsung ke pin 3.3V ESP32 — bisa mati.

Skema sederhana:

```
Baterai 7.4V ─┬─ VM TB6612
               └─ AMS1117 5V (opsional) ── VIN ESP32 + Servo (5V)
```

---

## Ringkasan Koneksi

| Modul      | ESP32 Pin |
|------------|-----------|
| TB6612 PWMA | 25       |
| TB6612 AIN1 | 26       |
| TB6612 AIN2 | 27       |
| TB6612 PWMB | 13       |
| TB6612 BIN1 | 14       |
| TB6612 BIN2 | 33       |
| TB6612 STBY | 32       |
| TB6612 VCC  | 3.3V     |
| TB6612 VM   | Baterai (7.4V) |
| TB6612 GND  | GND      |
| Servo signal| 5        |
| Servo VCC   | 3.3V / 5V |
| Servo GND   | GND      |
| Buzzer (+)  | 4        |
| Buzzer (-)  | GND      |
| VL53L0X SDA | 21       |
| VL53L0X SCL | 22       |
| VL53L0X XSHUT | 15     |
| VL53L0X VCC | 3.3V     |
| MPU6050 SDA | 21       |
| MPU6050 SCL | 22       |
| MPU6050 VCC | 3.3V     |
| LED (+)     | 2        |
| LED (-)     | GND      |
