// ======================= INCLUDES =======================
#include <Arduino.h>
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>   // por fabs()

#define ROBOT_VARIANT_ZENIT
//#define ROBOT_VARIANT_SOLLOW_OLD

#if defined(ROBOT_VARIANT_ZENIT)
// ---------- ZENIT (nuevo) ----------
#define FW_BLE_NAME         "ZENIT"  // <— nombre visible en el escaneo
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-4688-b7f5-ea07361b26a8"
static bool is_Servo = false; // PWM turbina (nuevo)

#elif defined(ROBOT_VARIANT_SOLLOW_OLD)
// ---------- SOLLOW (viejo) ----------
#define FW_BLE_NAME         "SOLLOW" // <— nombre visible en el escaneo
#define SERVICE_UUID        "4fafc201-1fb5-120e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-120e-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-120e-b7f5-ea07361b26a8"
static bool is_Servo = true;  // servo turbina (viejo)

#else
#error "Define ROBOT_VARIANT_ZENIT o ROBOT_VARIANT_SOLLOW_OLD"
#endif

BLECharacteristic *pCharacteristic;    // CH1
BLECharacteristic *pCharacteristic_2;  // CH2

// ======================= MÓDULO DE INICIO =======================
const byte MInit = D3;
int   Estado = 0;   // 0/1 (general)

// ======================= TURBINA =======================
Servo myTurbina;
const int Tur = D4; // PIN PWM o Servo

int   ValTurb = 0;
int minvaltur = 0;
int maxvaltur = 0;

float KTurb = 0.6f; // (si usas Esfuerzo_Turbina)

// ======================= SENSORES QTR =======================
#define NUM_SENSORS 16
#define NUM_SAMPLES_PER_SENSOR 3
#define IN_PIN A1 // PIN de entrada del MUX hacia el micro

QTRSensorsMux qtra(
  (unsigned char[]){0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
  NUM_SENSORS,
  NUM_SAMPLES_PER_SENSOR,
  (unsigned char)IN_PIN,
  (unsigned char[]){ D9, D10, D0, D2 }
);
unsigned int sensorValues[NUM_SENSORS];

const uint16_t QTR_MAX_POS   = (NUM_SENSORS - 1) * 1000; // 15000
const bool     QTR_INVERT_LR = false; // invierte IZQ/DER si tu arreglo quedó espejado

// --------- UMBRALES ---------
const uint16_t QTR_TH_ON_DEFAULT  = 570; // fijos iniciales de respaldo
const uint16_t QTR_TH_OFF_DEFAULT = 330;

// Elegir por código: true = calcular desde muestreo en setup; false = usar fijos
const bool USE_AUTO_THRESHOLDS = true;

// Umbrales vigentes (0..1000)
uint16_t qtr_th_on  = QTR_TH_ON_DEFAULT;   // "apareció" línea
uint16_t qtr_th_off = QTR_TH_OFF_DEFAULT;  // "desapareció" línea (TH_OFF < TH_ON)

static uint16_t lastPosRaw = QTR_MAX_POS / 2; // última pos válida (centro=7500)
static bool     hadLine    = true;            // estado "hay línea" con histéresis

// ======================= CONTROL =======================
float Tm = 4.0f; // ms (medido por loop)
float Referencia = 0.0f, Control = 0.0f, Kp = 3.2f, Ti = 0.0f, Td = 0.02f;
float Salida = 0.0f, Error = 0.0f, Error_ant = 0.0f;
float offset = 1.0f, Vmax = 0.0f, E_integral = 0.0f;

// Cache de lecturas para BLE
float lastSalidaNorm = 0.0f;  // normalizada [-1, +1]
float lastRawLine    = 0.0f;  // ponderada 0..~15000

// ======================= BLUETOOTH (parse) =======================
String datos;     // buffer CH1
String S_offset;  // buffer CH2

// ======================= PWM Motores =======================
const uint16_t Frecuencia = 5000;   // Hz
const uint8_t  Resolucion = 10;     // bits (0..1023)

const int PWMI = D5; // Motor Izquierdo PWM
const int PWMD = D8; // Motor Derecho  PWM
const int DirI = D6; // Dirección Izquierdo
const int DirD = D7; // Dirección Derecho

// ======================= ESTADO RUNTIME =======================
unsigned long Tinicio = 0;
bool conect = false;

// ======================= BLE CALLBACKS =======================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    conect = true;
    Serial.println("Cliente conectado.");
  }
  void onDisconnect(BLEServer* pServer) override {
    conect = false;
    Serial.println("Cliente desconectado.");
    pServer->getAdvertising()->start();
    Serial.println("Publicidad BLE reiniciada.");
  }
};

static String rtrim(const String& s) {
  int end = s.length() - 1;
  while (end >= 0 && (s[end] == '\r' || s[end] == '\n' || s[end] == ' ' || s[end] == '\t')) end--;
  if (end < 0) return "";
  return s.substring(0, end + 1);
}

// ======================= PROTOTIPOS =======================
void Inicializacion_Pines();
void Inicializacion_turbina();
void Inicializacion_Sensores();
void CrearPWM();
void ActualizarLecturasCache();
float Lectura_Sensor();
float Controlador(float Referencia, float Salida);
void  Esfuerzo_Control(float Control);
void  Esfuerzo_Turbina();
unsigned long Tiempo_Muestreo(unsigned long Tinicio);
void Inicializacion_Bluetooth();
void  InitThresholdsOnce();
void  AutoAjustarUmbrales(uint16_t dur_ms);

// ======================= CH1: PID/Vmax/ValTurb/(KTurb) =======================
class MyCallbacks_1 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc) override {
    String value = pc->getValue();
    if (value.length() == 0) return;

    if (value[0] == '*') datos = ""; // nuevo comando

    datos += value;
    datos.replace("-", ".");   // compat decimal
    int nl = datos.indexOf('\n');
    if (nl >= 0) datos = datos.substring(0, nl);
    datos = rtrim(datos);

    if (datos.length() < 2 || datos[0] != '*') return;

    int d1 = datos.indexOf(',', 1);              if (d1 < 0) return;
    int d2 = datos.indexOf(',', d1 + 1);         if (d2 < 0) return;
    int d3 = datos.indexOf(',', d2 + 1);         if (d3 < 0) return;
    int d4 = datos.indexOf(',', d3 + 1);         if (d4 < 0) return;
    int d5 = datos.indexOf(',', d4 + 1);         // opcional KTurb

    String S_Kp      = (d1>0) ? datos.substring(1, d1) : "";
    String S_Ti      = (d2>0) ? datos.substring(d1 + 1, d2) : "";
    String S_Td      = (d3>0) ? datos.substring(d2 + 1, d3) : "";
    String S_Vmax    = (d4>0) ? datos.substring(d3 + 1, d4) : "";
    String S_ValTurb = (d5>=0) ? datos.substring(d4 + 1, d5) : datos.substring(d4 + 1);
    String S_KTurb   = (d5>=0) ? datos.substring(d5 + 1)     : ""; // opcional

    datos = "";

    Kp      = S_Kp.toFloat();
    Ti      = S_Ti.toFloat();
    Td      = S_Td.toFloat();
    Vmax    = constrain(S_Vmax.toFloat(), 0, 1023);
    ValTurb = constrain((int)S_ValTurb.toFloat(), minvaltur, maxvaltur);
    if (S_KTurb.length() > 0) {
      KTurb = S_KTurb.toFloat();
      KTurb = constrain(KTurb, 0.0f, 2.0f);
    }

    Serial.printf("[BLE CH1] kp=%.3f ti=%.3f td=%.3f vmax=%.0f turb=%d kturb=%.3f\n",
                  Kp, Ti, Td, Vmax, ValTurb, KTurb);

    char pbuf[180];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }

  void onRead(BLECharacteristic *pc) override {
    char pbuf[180];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }
};

// ======================= CH2: OFFSET/ESTADO/THs + lecturas =======================
class MyCallbacks_2 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc2) override {
    String value = pc2->getValue();
    if (value.length() == 0) return;

    S_offset = value;
    S_offset.replace("-", ".");
    S_offset.trim();

    Serial.print("[BLE CH2] RX: '"); Serial.println(S_offset);

    String up = S_offset; up.toUpperCase();
    int eq = up.indexOf('=');
    String key = up, val = "";
    if (eq >= 0) {
      key = up.substring(0, eq);
      val = S_offset.substring(eq + 1);
      val.trim();
    } else {
      key = up; val = S_offset;
    }
    key.trim();

    if (key == "ESTADO") {
      int iv = val.toInt();
      Estado = (iv != 0) ? 1 : 0;
      Serial.printf("[BLE CH2] ESTADO=%d\n", Estado);

    } else if (key == "OFFSET") {
      offset = val.toFloat();
      Serial.println("[BLE CH2] OFFSET=" + String(offset));

    } else if (key == "TH_ON") {
      int v = val.toInt();
      v = constrain(v, 0, 1000);
      qtr_th_on = v;
      if (qtr_th_off >= qtr_th_on) qtr_th_off = (qtr_th_on > 10) ? (qtr_th_on - 10) : 0;
      Serial.printf("[BLE CH2] TH_ON=%u (ajustada TH_OFF=%u si era >=)\n", qtr_th_on, qtr_th_off);

    } else if (key == "TH_OFF") {
      int v = val.toInt();
      v = constrain(v, 0, 1000);
      if (v >= (int)qtr_th_on) v = (qtr_th_on > 10) ? (qtr_th_on - 10) : 0;
      qtr_th_off = v;
      Serial.printf("[BLE CH2] TH_OFF=%u\n", qtr_th_off);

    } else {
      // Si solo vino un número, asume OFFSET
      bool soloNumero = true;
      for (size_t i = 0; i < S_offset.length(); ++i) {
        char c = S_offset[i];
        if (!((c >= '0' && c <= '9') || c == '.' || c == '+' || c == '-' )) { soloNumero = false; break; }
      }
      if (soloNumero) {
        offset = S_offset.toFloat();
        Serial.println("[BLE CH2] OFFSET=" + String(offset));
      } else {
        Serial.println("[BLE CH2] Comando no reconocido (use OFFSET=..., ESTADO=0/1, TH_ON=..., TH_OFF=...).");
      }
    }

    // Actualiza snapshot de parámetros en CH1
    if (pCharacteristic) {
      char pbuf[180];
      snprintf(pbuf, sizeof(pbuf),
        "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
        (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
        (double)offset, (double)KTurb);
      pCharacteristic->setValue((uint8_t*)pbuf, strlen(pbuf));
    }
  }

  void onRead(BLECharacteristic *pc2) override {
    char sbuf[112];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
             (double)lastSalidaNorm, (double)lastRawLine,
             (unsigned)qtr_th_on, (unsigned)qtr_th_off);
    pc2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }
};

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);

  if (is_Servo) { minvaltur = 0; maxvaltur = 180; }
  else          { minvaltur = 0; maxvaltur = 900; }

  Inicializacion_Pines();
  if (is_Servo) Inicializacion_turbina();

  Inicializacion_Sensores();     // <-- Calibración SOLO aquí
  InitThresholdsOnce();          // <-- Auto (si USE_AUTO_THRESHOLDS) o fijos

  CrearPWM();
  Inicializacion_Bluetooth();

  delay(300);

  if (is_Servo) myTurbina.write(ValTurb);
  else          analogWrite(Tur, ValTurb);
}

// ======================= LOOP =======================
void loop() {
  while (Estado) {
    Tinicio  = millis();
    Salida   = Lectura_Sensor();            // actualiza lastRawLine/lastSalidaNorm
    Control  = Controlador(Referencia, Salida);
    Esfuerzo_Control(Control);
    Tm       = Tiempo_Muestreo(Tinicio);

    if (is_Servo) myTurbina.write(ValTurb);
    else          analogWrite(Tur, ValTurb);

    ActualizarLecturasCache();
  }

  // Al salir del while => APAGA motores y turbina
  analogWrite(PWMI, 0);
  analogWrite(PWMD, 0);
  if (is_Servo) myTurbina.write(0);
  else          analogWrite(Tur, 0);

  ActualizarLecturasCache();
}

// ======================= FUNCIONES =======================
float Lectura_Sensor(void) {
  uint16_t pos = qtra.readLine(sensorValues);

  uint16_t maxv = 0;
  uint32_t sumv = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = sensorValues[i]; // 0..1000 (normalizado si hubo calibración)
    if (v > maxv) maxv = v;
    sumv += v;
  }

  bool lineNow = hadLine
                 ? (maxv > qtr_th_off)   // mantener con menos señal
                 : (maxv > qtr_th_on);   // aparecer con más señal
  hadLine = lineNow;

  uint16_t posStable = lineNow ? pos : lastPosRaw;

  if (QTR_INVERT_LR) posStable = QTR_MAX_POS - posStable;

  lastRawLine    = (float)posStable;                                 // 0..15000
  Salida         = ((float)posStable / (QTR_MAX_POS / 2.0f)) - 1.0f; // ~[-1,+1]
  lastSalidaNorm = Salida;

  if (lineNow) lastPosRaw = posStable;

  return Salida;
}

void ActualizarLecturasCache() {
  if (!pCharacteristic_2) return;
  char sbuf[112];
  snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
           (double)lastSalidaNorm, (double)lastRawLine,
           (unsigned)qtr_th_on, (unsigned)qtr_th_off);
  pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
}

float Controlador(float Ref, float Y) {
  float U;
  Error = Ref - Y;

  if ((Error > -0.15f && Error < 0.0f) || (Error > 0.0f && Error < 0.15f)) Error = 0.0f;

  float Ts = Tm / 1000.0f;
  E_integral += ((Error * Ts) + (Ts * (Error - Error_ant)) / 2.0f);
  E_integral = constrain(E_integral, -100.0f, 100.0f);

  float E_derivativo = (Error - Error_ant) / max(Ts, 1e-3f);

  U = Kp * (Error + Ti * E_integral + Td * E_derivativo);
  Error_ant = Error;

  U = constrain(U, -2.5f, 2.5f);
  return U;
}

void Esfuerzo_Control(float U) {
  float s1 = (offset - U); // Derecho
  float s2 = (offset + U); // Izquierdo

  int pwm1 = floor(constrain(fabs(s1), 0.0f, 1.0f) * Vmax);
  int pwm2 = floor(constrain(fabs(s2), 0.0f, 1.0f) * Vmax);

  analogWrite(PWMD, pwm1);
  analogWrite(PWMI, pwm2);

  digitalWrite(DirD, (s1 <= 0.0f) ? LOW  : HIGH);
  digitalWrite(DirI, (s2 <= 0.0f) ? LOW  : HIGH);
}

void Esfuerzo_Turbina() {
  float estur = constrain(round(minvaltur + ((KTurb * fabs(Error)) * (maxvaltur - minvaltur))),
                          (float)minvaltur, (float)maxvaltur);
  if (is_Servo) myTurbina.write((int)estur);
  else          analogWrite(Tur, (int)estur);
}

unsigned long Tiempo_Muestreo(unsigned long TinicioRef) {
  return millis() - TinicioRef;
}

void CrearPWM() {
  analogWriteResolution(PWMD, Resolucion);
  analogWriteResolution(PWMI, Resolucion);
  if (!is_Servo) analogWriteResolution(Tur, Resolucion);

  analogWriteFrequency(PWMD, Frecuencia);
  analogWriteFrequency(PWMI, Frecuencia);
  if (!is_Servo) analogWriteFrequency(Tur, Frecuencia);

  analogWrite(PWMD, 0);
  analogWrite(PWMI, 0);
  if (!is_Servo) analogWrite(Tur, 0);
}

void Inicializacion_turbina() {
  ESP32PWM::allocateTimer(2);
  myTurbina.setPeriodHertz(50);
  myTurbina.attach(Tur, 1000, 2000);
  myTurbina.write(0);
}

void Inicializacion_Sensores() {
  // Calibración solo en setup: mueve el robot para ver blanco y línea
  for (int i = 0; i < 300; i++) {
    qtra.calibrate(); // ~25ms por llamada
  }
}

void Inicializacion_Pines() {
  pinMode(PWMD, OUTPUT);
  pinMode(PWMI, OUTPUT);
  pinMode(DirI, OUTPUT);
  pinMode(DirD, OUTPUT);
  pinMode(MInit, INPUT);
  if (!is_Servo) pinMode(Tur, OUTPUT);
  digitalWrite(DirI, LOW);
  digitalWrite(DirD, LOW);
}

void Inicializacion_Bluetooth() {
  BLEDevice::init(FW_BLE_NAME);
  BLEDevice::setMTU(185);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // CH1: READ/WRITE
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setCallbacks(new MyCallbacks_1());
  {
    char pbuf[180];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pCharacteristic->setValue((uint8_t*)pbuf, strlen(pbuf));
  }

  // CH2: READ/WRITE
  pCharacteristic_2 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_2,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic_2->setCallbacks(new MyCallbacks_2());
  {
    char sbuf[112];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
             (double)lastSalidaNorm, (double)lastRawLine,
             (unsigned)qtr_th_on, (unsigned)qtr_th_off);
    pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.print("BLE Advertising iniciado (");
  Serial.print(FW_BLE_NAME);
  Serial.println(").");
}

// ======================= AUTO-THRESHOLDS (solo setup, por percentiles) =======================
void InitThresholdsOnce() {
  if (USE_AUTO_THRESHOLDS) {
    // 1.2–2.0 s suele bastar; muestrea mientras mueves el robot por línea y fondo
    AutoAjustarUmbrales(1400);
  } else {
    qtr_th_on  = QTR_TH_ON_DEFAULT;
    qtr_th_off = (QTR_TH_OFF_DEFAULT >= QTR_TH_ON_DEFAULT)
                   ? ((QTR_TH_ON_DEFAULT > 10) ? (QTR_TH_ON_DEFAULT - 10) : 0)
                   : QTR_TH_OFF_DEFAULT;
    Serial.printf("[QTR] Modo fijos: TH_ON=%u, TH_OFF=%u\n", qtr_th_on, qtr_th_off);

    // Publica enseguida a la app (sin esperar al loop)
    if (pCharacteristic_2) {
      char sbuf[112];
      snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
               (double)lastSalidaNorm, (double)lastRawLine,
               (unsigned)qtr_th_on, (unsigned)qtr_th_off);
      pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
    }
  }
}

void AutoAjustarUmbrales(uint16_t dur_ms) {
  // -------- CONFIG Y BUFFERS --------
  const uint16_t max_samples = 240;   // tope de muestras
  uint16_t buf[max_samples];          // almacena el "max por lectura" (0..1000)
  uint16_t n = 0;

  // -------- ADQUISICIÓN --------
  unsigned long t0 = millis();
  while ((millis() - t0) < dur_ms && n < max_samples) {
    qtra.readLine(sensorValues);  // llena sensorValues (0..1000 calibrado)
    uint16_t maxv = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > maxv) maxv = sensorValues[i];
    }
    buf[n++] = maxv;
    delay(8); // ~125 Hz (no bloquea BLE de forma crítica)
  }

  // -------- FALLBACK POR POCAS MUESTRAS --------
  if (n < 20) {
    qtr_th_on  = QTR_TH_ON_DEFAULT;
    qtr_th_off = (QTR_TH_OFF_DEFAULT < qtr_th_on)
                  ? QTR_TH_OFF_DEFAULT
                  : (qtr_th_on > 10 ? qtr_th_on - 10 : 0);
    Serial.printf("[AUTO-TH] pocas muestras (%u) -> fallback: ON=%u OFF=%u\n",
                  n, qtr_th_on, qtr_th_off);
    // Publica inmediato a BLE
    if (pCharacteristic_2) {
      char sbuf[112];
      snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
               (double)lastSalidaNorm, (double)lastRawLine,
               (unsigned)qtr_th_on, (unsigned)qtr_th_off);
      pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
    }
    return;
  }

  // -------- ORDENAR PARA PERCENTILES --------
  for (uint16_t i = 1; i < n; ++i) {
    uint16_t key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j+1] = buf[j]; j--; }
    buf[j+1] = key;
  }

  auto pctIndex = [&](float p) -> uint16_t {
    int idx = (int)roundf(p * (n - 1));
    if (idx < 0) idx = 0; if (idx >= (int)n) idx = n - 1;
    return buf[idx];
  };

  const uint16_t p20 = pctIndex(0.20f);
  const uint16_t p50 = pctIndex(0.50f);
  const uint16_t p80 = pctIndex(0.80f);
  const uint16_t span = (p80 > p20) ? (p80 - p20) : 0;

  // -------- HEURÍSTICAS DE SANIDAD (tu caso: línea >= ~800, fondo <= ~250) --------
  const uint16_t SAFE_TH_ON  = 700;
  const uint16_t SAFE_TH_OFF = 400;

  bool bad = false;
  if (span < 80)          bad = true;   // separación débil
  if (p80 < 650)          bad = true;   // línea demasiado baja
  if (p20 > 350)          bad = true;   // fondo demasiado alto
  if ((p50 >= 900) && (p80 >= 950)) bad = true; // saturación
  if ((p80 - p20) <= 30)  bad = true;

  if (bad) {
    qtr_th_on  = SAFE_TH_ON;
    qtr_th_off = (SAFE_TH_OFF >= SAFE_TH_ON) ? ((SAFE_TH_ON>10)? SAFE_TH_ON-10 : 0) : SAFE_TH_OFF;
    Serial.printf("[AUTO-TH] cal inválida (p20=%u p50=%u p80=%u span=%u) -> Fallback: ON=%u OFF=%u\n",
                  p20, p50, p80, span, qtr_th_on, qtr_th_off);
  } else {
    // -------- ESTIMACIÓN CON HISTÉRESIS --------
    // OFF = p20 + 25% del span, ON = p20 + 65% del span
    uint16_t off_est = p20 + (uint16_t)(0.25f * span);
    uint16_t on_est  = p20 + (uint16_t)(0.65f * span);

    // Sanitiza e histéresis mínima
    off_est = constrain(off_est, (uint16_t)0, (uint16_t)1000);
    on_est  = constrain(on_est,  (uint16_t)0, (uint16_t)1000);
    if (on_est <= off_est) on_est = (off_est + 10 <= 1000) ? (off_est + 10) : 1000;

    // Clamps finales para tu perfil de pista
    on_est  = constrain(on_est,  600, 900);
    off_est = constrain(off_est, 250, (uint16_t)(on_est - 10));

    qtr_th_off = off_est;
    qtr_th_on  = on_est;

    Serial.printf("[AUTO-TH] p20=%u p50=%u p80=%u span=%u -> ON=%u OFF=%u\n",
                  p20, p50, p80, span, qtr_th_on, qtr_th_off);
  }

  // -------- PUBLICAR A LA APP AL INSTANTE --------
  if (pCharacteristic_2) {
    char sbuf[112];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
             (double)lastSalidaNorm, (double)lastRawLine,
             (unsigned)qtr_th_on, (unsigned)qtr_th_off);
    pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }
}
