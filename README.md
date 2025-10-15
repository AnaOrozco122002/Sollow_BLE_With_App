# XIAO ESP32‑S3 Line Follower (BLE) — README

> Firmware para un carro seguidor de línea con **Seeed XIAO ESP32‑S3**, arreglo **QTR x16** con **MUX**, control **PID** y **turbina** (servo o PWM). Expone un **servicio BLE** para sintonía en tiempo real (Kp/Ti/Td/Vmax/offset/umbrales) y telemetría básica.

---

## 📦 Características principales
- Lectura **QTR de 16 canales** vía `QTRSensorsMux` (posición 0..15000).
- **Robustez anti‑pérdida de línea** con histéresis y “last valid position”.
- **Control PID** (offset ± U) con anti‑windup, zona muerta y saturación.
- **Turbina** seleccionable por variante: **servo** (grados) o **PWM** (0..900).
- **BLE** (NimBLE del core ESP32):
  - **CH1** (READ/WRITE): Kp, Ti, Td, Vmax, ValTurb, (KTurb opcional).
  - **CH2** (READ/WRITE): comandos `ESTADO`, `OFFSET`, `TH_ON`, `TH_OFF`; lectura de `salida`, `raw`, umbrales.
- **PWM 10‑bit @ 5 kHz** para motores.

---

## 🧰 Hardware usado (pines)
| Señal | Pin XIAO | Notas |
|---|---|---|
| MInit (start/stop) | `D3` | Se puede controlar también por BLE (`ESTADO`). |
| Turbina | `D4` | Servo (50 Hz) o PWM según variante. |
| PWM Motor Izq | `D5` | 10‑bit @ 5 kHz. |
| Dir Izq | `D6` | Dirección motor izquierdo. |
| Dir Der | `D7` | Dirección motor derecho. |
| PWM Motor Der | `D8` | 10‑bit @ 5 kHz. |
| MUX SL0..SL3 | `D9, D10, D0, D2` | Líneas de selección del MUX QTR. |
| Entrada MUX | `A1` | Señal analógica desde MUX a MCU. |

> Ajusta los pines si tu shield/PCB es diferente.

---

## 🧬 Variantes del firmware
Selecciona **una** al inicio del archivo:
```cpp
//#define ROBOT_VARIANT_ZENIT
#define ROBOT_VARIANT_SOLLOW_OLD
```

- **ZENIT (nuevo)**
  - Nombre BLE: `ZENIT`
  - `SERVICE_UUID`: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
  - `CH1 UUID`: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
  - `CH2 UUID`: `ceb5483e-36e1-4688-b7f5-ea07361b26a8`
  - Turbina **PWM** (`is_Servo = false`, rango 0..900).

- **SOLLOW_OLD (viejo)**
  - Nombre BLE: `SOLLOW`
  - `SERVICE_UUID`: `4fafc201-1fb5-120e-8fcc-c5c9c331914b`
  - `CH1 UUID`: `beb5483e-36e1-120e-b7f5-ea07361b26a8`
  - `CH2 UUID`: `ceb5483e-36e1-120e-b7f5-ea07361b26a8`
  - Turbina **SERVO** (`is_Servo = true`, rango 0..180°).

---

## 📚 Dependencias (Arduino IDE)
- **ESP32 core** para Arduino (XIAO ESP32‑S3).
- **ESP32Servo** (servo a 50 Hz en `D4`).
- **QTRSensors**/`QTRSensorsMux`.
- `BLEDevice/BLEServer/BLEUtils` incluidos en el core ESP32 (NimBLE).

---

## 🛠️ Compilación y carga
1. Arduino IDE 3.3.x → **Board**: *Seeed XIAO ESP32‑S3*.
2. Selecciona la **variante** deseada (ver arriba).
3. Instala librerías mencionadas.
4. Conecta por USB‑C, compila y sube.

---

## 🎚️ Calibración de sensores QTR
En `setup()` se llaman 300 iteraciones de `qtra.calibrate()` (~25 ms c/u). Durante ese tiempo **mueve el robot** sobre la línea para registrar blancos/negros. Los valores de cada sensor quedan normalizados a **0..1000**.

**Umbrales configurables por BLE**:
- `TH_ON` (default `570`): ganancia para **detectar aparición** de línea.
- `TH_OFF` (default `330`): ganancia para **mantener** presencia con histéresis.

> El firmware fuerza `TH_OFF < TH_ON`. Si configuras `TH_OFF >= TH_ON`, se ajusta automáticamente a `TH_ON - 10` (no negativo).

---

## 📡 Protocolo BLE
**MTU:** 185. El dispositivo reanuda *advertising* tras desconexión.

### Servicio y características
- **Service UUID**: cambia según variante (ver tabla de variantes).
- **CH1 (READ/WRITE)** — *Sintonía PID/turbina*  
  **Formato WRITE** (todo en una sola línea):
  
  ```
  *Kp,Ti,Td,Vmax,ValTurb[,KTurb]\n
  ```
  - `Kp,Ti,Td` → `float`
  - `Vmax` → `0..1023` (10‑bit PWM)
  - `ValTurb` → servo: `0..180` | PWM: `0..900`
  - `KTurb` (opcional) → `0.0..2.0`

  **Ejemplo:**
  ```
  *3.2,0,0.02,900,120,0.6\n
  ```
  > Nota: el firmware sustituye `-` por `.` como “compat decimal”. Usa **punto** para decimales.

  **READ** devuelve snapshot:
  ```
  kp=3.200,ti=0.000,td=0.020,vmax=900,valturb=120,offset=1.000,ktur=0.600
  ```

- **CH2 (READ/WRITE)** — *Estado/offset/umbrales & telemetría*  
  **Comandos WRITE** (clave=valor, sin espacios):
  - `ESTADO=0|1` → habilita/deshabilita lazo.
  - `OFFSET=±x.x`
  - `TH_ON=0..1000`
  - `TH_OFF=0..1000` (si ≥ `TH_ON` se ajusta a `TH_ON-10`).
  
  Si envías **solo un número** (p.ej. `0.8`), se interpreta como `OFFSET`.

  **READ** expone telemetría:
  ```
  salida=−0.035,raw=7420,th_on=570,th_off=330
  ```
  - `salida` → posición normalizada en **[−1, +1]**.
  - `raw` → posición ponderada **0..15000** (con retención si se pierde la línea).

---

## 🕹️ Operación
1. Conecta por BLE (app o script). Nombre: **ZENIT** o **SOLLOW** según variante.
2. Ajusta PID, Vmax y turbina por **CH1**.
3. Configura `OFFSET` (velocidad base) y umbrales por **CH2**.
4. Habilita el control con `ESTADO=1` (o usa el pin `MInit`).
5. Monitorea `salida/raw` leyendo **CH2**.

> En `loop()`, cuando `ESTADO=0`, los **motores y turbina quedan en 0** por seguridad.

---

## ⚙️ Parámetros y rangos recomendados
- `Kp` ≈ 2.0–5.0, `Ti` desde 0 (integ. desactivada) y `Td` ≈ 0.01–0.05.
- `OFFSET` en 0.6–1.0 (según relación PWM‑velocidad de tu chasis).
- `Vmax` 600–1023 (cuidado con saturación y derrapes).
- `TH_ON` 500–700 / `TH_OFF` 250–450 (ajusta según iluminación/suelo).
- `KTurb` 0.3–0.8 (si usas `Esfuerzo_Turbina()`).

---

## 🔧 Implementación destacada
- **Histéresis**: `hadLine` y umbrales duales evitan oscilaciones ante ruido.
- **“Last valid pos”**: si se pierde la línea, se mantiene la última posición válida evitando saltos a 0/15000.
- **PID estándar**: integración trapezoidal, derivada discreta y anti‑windup por `constrain` de integral y salida.
- **PWM por‑pin**: resolución 10‑bit y 5 kHz para motores; 50 Hz para servo.
- **BLE snapshot**: tras `WRITE` en CH1/CH2, se actualiza el estado leíble.

---

## 🩺 Troubleshooting
- **“A veces no hace caso a la app”**: verifica que envíes **todo el comando CH1 en una sola línea** iniciando con `*` y terminando con `\n`. Evita fragmentar por MTU en tu app.
- **No cambia `TH_OFF`**: si está ≥ `TH_ON`, el firmware lo ajusta. Re‑lee CH2 para confirmar.
- **Saltos de `raw` a 0/15000**: indica pérdida de línea o saturación; ajusta `TH_ON/TH_OFF` y mejora calibración.
- **Servo no se mueve**: asegúrate de estar en variante **SOLLOW_OLD** y que `ValTurb` esté en 0..180.
- **PWM sin efecto**: en **ZENIT**, `ValTurb` rango 0..900 y `analogWrite(Tur, ValTurb)` activo.


