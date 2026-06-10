# Wiring Diagram — KEI Robot

## Pin Mapping ESP32 → Hardware

```
┌─────────────────────────────────────────────────────────┐
│                      ESP32 DOIT                         │
│                                                         │
│  GPIO 32 ─── PWMA  (TB6612 — Motor Kiri)               │
│  GPIO 33 ─── AIN1  (TB6612 — Motor Kiri)               │
│  GPIO 25 ─── AIN2  (TB6612 — Motor Kiri)               │
│  GPIO 26 ─── PWMB  (TB6612 — Motor Kanan)              │
│  GPIO 27 ─── BIN1  (TB6612 — Motor Kanan)              │
│  GPIO 14 ─── BIN2  (TB6612 — Motor Kanan)              │
│                                                         │
│  GPIO 13 ─── Servo (signal)                             │
│  GPIO  4 ─── Buzzer Passive (signal)                    │
│                                                         │
│  GPIO 21 ─── SDA  (VL53L0X + MPU6050)                  │
│  GPIO 22 ─── SCL  (VL53L0X + MPU6050)                  │
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
│  │ PWMA  ← GPIO 32     │    │
│  │ AIN1  ← GPIO 33     │    │
│  │ AIN2  ← GPIO 25     │    │
│  │ AO1   → Motor Kiri +│    │
│  │ AO2   → Motor Kiri -│    │
│  ├─────────────────────┤    │
│  │ PWMB  ← GPIO 26     │    │
│  │ BIN1  ← GPIO 27     │    │
│  │ BIN2  ← GPIO 14     │    │
│  │ BO1   → Motor Kanan +│    │
│  │ BO2   → Motor Kanan -│    │
│  ├─────────────────────┤    │
│  │ VM    ← Bateri (3–13V)   │
│  │ VCC   ← 3.3V (ESP32) │    │
│  │ GND   → GND          │    │
│  │ STBY  ← 3.3V (enable)│    │
│  └─────────────────────┘    │
└─────────────────────────────┘
```

**Catatan:**
- **STBY** harus di-*pull up* ke 3.3V biar driver aktif. Bisa langsung ke 3.3V atau ke GPIO 12 (kalau mau kontrol standby via software).
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

Ciri buzzer **passive**: suara bisa diubah frekuensinya via PWM. Jangan pake buzzer *active* (yang ada osilator internal) karena cuma bisa 1 nada.

---

## Servo (SG90 / MG90S / MG995)

```
Servo
┌──────────────┐
│ Coklat  ─ GND │
│ Merah   ─ 3.3V│  (atau 5V kalau dari VIN eksternal)
│ Kuning  ─ 13  │
└──────────────┘
```

**⚠️** Untuk servo besar (MG995), jangan ambil daya dari 3.3V ESP32. Pakai VIN atau regulator 5V eksternal. Ground tetap disatukan.

---

## I2C — VL53L0X + MPU6050

```
3.3V ─── VCC (VL53L0X) ─── VIN (MPU6050)
GND  ─── GND (keduanya)
GPIO 21 ─── SDA (keduanya)
GPIO 22 ─── SCL (keduanya)
```

Keduanya memakai I2C yang sama secara paralel. Alamat default:
- VL53L0X: `0x29`
- MPU6050: `0x68`

**VL53L0X** biasanya punya pin `XSHUT` — kalau cuma 1 sensor, bisa dibiarkan *floating* atau di-pull-up ke 3.3V.

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

| Modul | ESP32 Pin |
|-------|-----------|
| TB6612 PWMA | 32 |
| TB6612 AIN1 | 33 |
| TB6612 AIN2 | 25 |
| TB6612 PWMB | 26 |
| TB6612 BIN1 | 27 |
| TB6612 BIN2 | 14 |
| TB6612 VCC | 3.3V |
| TB6612 VM | Baterai (7.4V) |
| TB6612 STBY | 3.3V |
| TB6612 GND | GND |
| Servo signal | 13 |
| Servo VCC | 3.3V / 5V |
| Servo GND | GND |
| Buzzer (+) | 4 |
| Buzzer (-) | GND |
| VL53L0X SDA | 21 |
| VL53L0X SCL | 22 |
| VL53L0X VCC | 3.3V |
| MPU6050 SDA | 21 |
| MPU6050 SCL | 22 |
| MPU6050 VCC | 3.3V |
