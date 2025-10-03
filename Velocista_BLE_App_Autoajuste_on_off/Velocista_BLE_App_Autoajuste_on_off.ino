// ======================= INCLUDES =======================
#include <Arduino.h>
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>   // por fabs()

// ======================= BLE UUIDs =======================
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8" // CH1: PID/Vmax/ValTurb/(KTurb opcional) (READ/WRITE)
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-4688-b7f5-ea07361b26a8" // CH2: Offset/Estado/THs (WRITE) + Lecturas (READ)

BLECharacteristic *pCharacteristic;    // CH1
BLECharacteristic *pCharacteristic_2;  // CH2

// ======================= MÓDULO DE INICIO =======================
const byte MInit = D3;
int   Estado = 0;   // 0/1 (general)

// ======================= TURBINA =======================
Servo myTurbina;
const int Tur = D4; // PIN PWM o Servo

int   ValTurb = 150;

bool is_Servo = false; // false=PWM (Nuevo Sollow), true=servo (viejo sollow)

int minvaltur=0;
int maxvaltur=0;

float KTurb = 0.6f; // (si usas Esfuerzo_Turbina)

// ======================= SENSORES QTR =======================
#define NUM_SENSORS 16
#define NUM_SAMPLES_PER_SENSOR 3
#define IN_PIN A1 // PIN de entrada del MUX hacia el micro

// Secuencia 0..15 y select lines del MUX: SL0,SL1,SL2,SL3
QTRSensorsMux qtra(
  (unsigned char[]){0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
  NUM_SENSORS,
  NUM_SAMPLES_PER_SENSOR,
  (unsigned char)IN_PIN,
  (unsigned char[]){ D9, D10, D0, D2 }
);
unsigned int sensorValues[NUM_SENSORS];

// ======= Robustez ponderado (anti-saltos a 0/15000 cuando se pierde la línea) =======
const uint16_t QTR_MAX_POS   = (NUM_SENSORS - 1) * 1000; // 15*1000 = 15000
const bool     QTR_INVERT_LR = false; // true si tu arreglo quedó espejado (invierte IZQ/DER)

// Umbrales AJUSTABLES por BLE (rango 0..1000 tras calibración)
uint16_t qtr_th_on  = 520;  // "apareció" línea
uint16_t qtr_th_off = 320;  // "desapareció" línea (histeresis) -> debe ser < qtr_th_on

// ======= Auto-Ajuste de Umbrales (configurable) =======
bool     auto_thresholds = true;    // <<--- pon en false para desactivar el autoajuste
uint16_t th_on_default  = 520;      // respaldo si falla el autoajuste
uint16_t th_off_default = 320;      // respaldo si falla el autoajuste

static uint16_t lastPosRaw = QTR_MAX_POS / 2; // última pos válida (centro=7500)
static bool     hadLine    = true;            // estado "hay línea" con histéresis

// ======================= CONTROL =======================
float Tm = 4.0f; // ms (medido por loop)
float Referencia = 0.0f, Control = 0.0f, Kp = 2.0f, Ti = 0.0f, Td = 0.02f;
float Salida = 0.0f, Error = 0.0f, Error_ant = 0.0f;
float offset = 1.0f, Vmax = 100.0f, E_integral = 0.0f;

// Cache de lecturas para BLE (evita recomputos en READ)
float lastSalidaNorm = 0.0f;  // normalizada [-1, +1]
float lastRawLine    = 0.0f;  // ponderada 0..~15000

// ======================= BLUETOOTH (parse) =======================
String datos;     // buffer para sintonización CH1
String S_offset;  // buffer para offset

// ======================= PWM Motores =======================
const uint16_t Frecuencia = 5000;
const byte     Canales[]  = { 0, 1 , 2};
const byte     Resolucion = 10;

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
    pServer->getAdvertising()->start(); // Reinicia advertising
    Serial.println("Publicidad BLE reiniciada.");
  }
};

// ---- Utilidad: recorta CR/LF/espacios al final ----
static String rtrim(const String& s) {
  int end = s.length() - 1;
  while (end >= 0 && (s[end] == '\r' || s[end] == '\n' || s[end] == ' ' || s[end] == '\t')) end--;
  if (end < 0) return "";
  return s.substring(0, end + 1);
}

// Característica 1: Kp,Ti,Td,Vmax,ValTurb,(KTurb opcional)
class MyCallbacks_1 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc) override {
    std::string value = pc->getValue();
    if (value.empty()) return;

    if (value[0] == '*') datos = ""; // nuevo comando

    datos += String(value.c_str());
    datos.replace("-", ".");   // compat decimal
    int nl = datos.indexOf('\n');
    if (nl >= 0) datos = datos.substring(0, nl);
    datos = rtrim(datos);

    // Espera "*a,b,c,d,e" (y opcional ",f" para KTurb)
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

    datos = ""; // limpia

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

    // Snapshot legible
    char pbuf[160];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }

  void onRead(BLECharacteristic *pc) override {
    char pbuf[160];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }
};

// Característica 2: WRITE para offset/estado/umbrales; READ para lecturas de sensor + umbrales
class MyCallbacks_2 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc2) override {
    std::string value = pc2->getValue();
    if (value.empty()) return;

    S_offset = String(value.c_str());
    S_offset.replace("-", "."); // compat
    S_offset.trim();

    Serial.print("[BLE CH2] RX: '"); Serial.println(S_offset);

    String up = S_offset; up.toUpperCase();
    int eq = up.indexOf('=');
    String key = up, val = "";
    if (eq >= 0) {
      key = up.substring(0, eq);
      val = S_offset.substring(eq + 1); // usa original para decimales
      val.trim();
    } else {
      key = up; val = S_offset;
    }
    key.trim();

    if (key == "ESTADO") {
      int iv = val.toInt();
      Estado = (iv != 0) ? 1 : 0;
      Serial.printf("[BLE CH2] ESTADO=%d (control BLE)\n", Estado);

    } else if (key == "OFFSET") {
      offset = val.toFloat();
      Serial.println("[BLE CH2] OFFSET=" + String(offset));

    } else if (key == "TH_ON") {
      int v = val.toInt();
      v = constrain(v, 0, 1000);
      qtr_th_on = v;
      // Garantiza histéresis: off < on
      if (qtr_th_off >= qtr_th_on) {
        qtr_th_off = (qtr_th_on > 10) ? (qtr_th_on - 10) : 0;
      }
      Serial.printf("[BLE CH2] TH_ON=%u (ajustada TH_OFF=%u si era >=)\n", qtr_th_on, qtr_th_off);

    } else if (key == "TH_OFF") {
      int v = val.toInt();
      v = constrain(v, 0, 1000);
      // Asegura off < on
      if (v >= (int)qtr_th_on) {
        v = (qtr_th_on > 10) ? (qtr_th_on - 10) : 0;
      }
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
      char pbuf[160];
      snprintf(pbuf, sizeof(pbuf),
        "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
        (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
        (double)offset, (double)KTurb);
      pCharacteristic->setValue((uint8_t*)pbuf, strlen(pbuf));
    }
  }

  void onRead(BLECharacteristic *pc2) override {
    char sbuf[96];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
             (double)lastSalidaNorm, (double)lastRawLine,
             (unsigned)qtr_th_on, (unsigned)qtr_th_off);
    pc2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }
};

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
void AutoAjustarUmbrales(uint16_t dur_ms = 1800);

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  
  if(is_Servo){
    minvaltur = 50;
    maxvaltur = 180;
  }else{
    minvaltur = 0;
    maxvaltur = 900;
  }

  Inicializacion_Pines();
  if(is_Servo){
    Inicializacion_turbina();
  }
  Inicializacion_Sensores();

  // === Autoajuste de TH_ON / TH_OFF tras calibración ===
  if (auto_thresholds) {
    AutoAjustarUmbrales(1800); // ~1.8 s de muestreo
  } else {
    // Mantén valores por defecto o los que tengas.
    qtr_th_on = th_on_default;
    // Asegura histéresis (off < on)
    qtr_th_off = (th_off_default < qtr_th_on) ? th_off_default : (qtr_th_on > 10 ? qtr_th_on - 10 : 0);
    Serial.printf("[AUTO-TH DISABLED] ON=%u OFF=%u\n", qtr_th_on, qtr_th_off);
  }

  CrearPWM();
  Inicializacion_Bluetooth();

  delay(300);

  if(is_Servo){
    myTurbina.write(ValTurb); // posición inicial
  }else{
    ledcWrite(Canales[2], ValTurb);
  }
}

// ======================= LOOP =======================
void loop() {
  //Estado = digitalRead(MInit);  // Si deseas control SOLO por BLE, deja comentada esta línea

  while (Estado) {
    //Estado = digitalRead(MInit); // idem comentario anterior

    Tinicio  = millis();
    Salida   = Lectura_Sensor();            // actualiza lastRawLine/lastSalidaNorm
    Control  = Controlador(Referencia, Salida);
    Esfuerzo_Control(Control);
    Tm       = Tiempo_Muestreo(Tinicio);

    if(is_Servo){
      myTurbina.write(ValTurb);
    }else{
      ledcWrite(Canales[2], ValTurb);
    }
    ActualizarLecturasCache();
  }

  // Al salir del while => APAGA motores y turbina
  ledcWrite(Canales[0], 0);
  ledcWrite(Canales[1], 0);
  if(is_Servo){
    myTurbina.write(0);
  }else{
    ledcWrite(Canales[2], 0);
  }
  
  ActualizarLecturasCache();
}

// ======================= FUNCIONES =======================

// Lee posición de la línea (ponderado) con robustez de "línea perdida"
float Lectura_Sensor(void) {
  // Lee ponderado (0..15000) y llena sensorValues (calibrados 0..1000)
  uint16_t pos = qtra.readLine(sensorValues);
  //pos = 15000 - pos;

  // Señal por sensor para decidir si hay línea (máximo y suma)
  uint16_t maxv = 0;
  uint32_t sumv = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = sensorValues[i];
    if (v > maxv) maxv = v;
    sumv += v;
  }

  // Histéresis para "línea presente"
  bool lineNow = hadLine
                 ? (maxv > qtr_th_off)   // mantener con menos señal
                 : (maxv > qtr_th_on);   // aparecer con más señal
  hadLine = lineNow;

  // Si no hay línea, mantenemos última válida (evita salto a 0/15000 del readLine)
  uint16_t posStable = lineNow ? pos : lastPosRaw;

  // Inversión Izq/Der 
  if (QTR_INVERT_LR) {
    posStable = QTR_MAX_POS - posStable;
  }

  // Exporta ambas lecturas
  lastRawLine = (float)posStable;                                      // 0..15000 estable
  Salida      = ((float)posStable / (QTR_MAX_POS / 2.0f)) - 1.0f;      // ~[-1,+1]
  lastSalidaNorm = Salida;

  // Guarda la última válida para la próxima vuelta
  if (lineNow) lastPosRaw = posStable;

  return Salida;
}

void ActualizarLecturasCache() {
  if (!pCharacteristic_2) return;
  char sbuf[96];
  snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u",
           (double)lastSalidaNorm, (double)lastRawLine,
           (unsigned)qtr_th_on, (unsigned)qtr_th_off);
  pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
}

// Controlador PID (forma estándar con anti-windup y zona muerta pequeña)
float Controlador(float Ref, float Y) {
  float E_derivativo;
  float U;

  Error = Ref - Y;

  // Zona muerta +/-0.1
  if ((Error > -0.15f && Error < 0.0f) || (Error > 0.0f && Error < 0.15f)) {
    Error = 0.0f;
  }

  // Integración trapezoidal
  float Ts = Tm / 1000.0f; // seg
  E_integral += ((Error * Ts) + (Ts * (Error - Error_ant)) / 2.0f);
  E_integral = constrain(E_integral, -100.0f, 100.0f);

  // Derivada
  E_derivativo = (Error - Error_ant) / max(Ts, 1e-3f);

  U = Kp * (Error + Ti * E_integral + Td * E_derivativo);
  Error_ant = Error;

  // Saturación salida control
  U = constrain(U, -2.5f, 2.5f);
  return U;
}

// Envía PWM a motores según offset +/- Control
void Esfuerzo_Control(float U) {
  float s1 = (offset - U); // Derecho
  float s2 = (offset + U); // Izquierdo

  // Magnitud PWM
  int pwm1 = floor(constrain(fabs(s1), 0.0f, 1.0f) * Vmax);
  int pwm2 = floor(constrain(fabs(s2), 0.0f, 1.0f) * Vmax);

  ledcWrite(Canales[0], pwm1);
  ledcWrite(Canales[1], pwm2);

  // Direcciones (ajustadas a tu driver)
  digitalWrite(DirD, (s1 <= 0.0f) ? LOW  : HIGH);
  digitalWrite(DirI, (s2 <= 0.0f) ? LOW  : HIGH);
}

// Turbina proporcional al |Error| (opcional)
void Esfuerzo_Turbina(){
  float estur = constrain(round(minvaltur + ((KTurb * fabs(Error)) * (maxvaltur - minvaltur))),
                          (float)minvaltur, (float)maxvaltur);
  if(is_Servo){
    myTurbina.write((int)estur);
  }else{
    ledcWrite(Canales[2], (int)estur);
  }
}

unsigned long Tiempo_Muestreo(unsigned long TinicioRef) {
  return millis() - TinicioRef;
}

void CrearPWM() {
  ledcSetup(Canales[0], Frecuencia, Resolucion);
  ledcSetup(Canales[1], Frecuencia, Resolucion);
  ledcSetup(Canales[2], Frecuencia, Resolucion);
  ledcAttachPin(PWMD, Canales[0]); // CH0 -> Motor Derecho
  ledcAttachPin(PWMI, Canales[1]); // CH1 -> Motor Izquierdo
  if(!is_Servo){
    ledcAttachPin(Tur, Canales[2]);
  }
}

//PARA TURBINA COMO SERVO
void Inicializacion_turbina() {
  ESP32PWM::allocateTimer(2);
  myTurbina.setPeriodHertz(50);
  myTurbina.attach(Tur, 1000, 2000);
  myTurbina.write(0);
}

void Inicializacion_Sensores() {
  // Calibración simple de QTR
  for (int i = 0; i < 300; i++) {
    qtra.calibrate(); // ~25ms por llamada
  }
}

void Inicializacion_Pines() {
  pinMode(PWMD, OUTPUT);
  pinMode(PWMI, OUTPUT);
  pinMode(DirI, OUTPUT);
  pinMode(DirD, OUTPUT);
  pinMode(MInit, INPUT); // << como lo tenías
  if(!is_Servo){
    pinMode(Tur, OUTPUT);
    ledcWrite(Canales[2], 0);
  }
  // Estados iniciales seguros
  digitalWrite(DirI, LOW);
  digitalWrite(DirD, LOW);
  ledcWrite(Canales[0], 0);
  ledcWrite(Canales[1], 0);
}

void Inicializacion_Bluetooth() {
  BLEDevice::init("SOLLOW");
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
    char pbuf[160];
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
    char sbuf[96];
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

  Serial.println("BLE Advertising iniciado (SOLLOW).");
}

// ======================= Autoajuste de Umbrales =======================
void AutoAjustarUmbrales(uint16_t dur_ms) {
  // Recolecta muestras del valor máximo por lectura (0..1000, ya calibrados)
  const uint16_t max_samples = 240;             // tope de muestras
  uint16_t buf[max_samples];
  uint16_t n = 0;

  unsigned long t0 = millis();
  while ((millis() - t0) < dur_ms && n < max_samples) {
    qtra.readLine(sensorValues);  // llena sensorValues (0..1000)
    uint16_t maxv = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > maxv) maxv = sensorValues[i];
    }
    buf[n++] = maxv;
    delay(8); // ~125 Hz de muestreo
  }

  if (n < 20) {
    // Demasiado pocas muestras -> fallback
    qtr_th_on  = th_on_default;
    qtr_th_off = (th_off_default < qtr_th_on) ? th_off_default : (qtr_th_on > 10 ? qtr_th_on - 10 : 0);
    Serial.printf("[AUTO-TH] pocas muestras (%u) -> fallback: ON=%u OFF=%u\n",
                  n, qtr_th_on, qtr_th_off);
    return;
  }

  // Ordena (inserción) para percentiles simples
  for (uint16_t i = 1; i < n; ++i) {
    uint16_t key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j+1] = buf[j]; j--; }
    buf[j+1] = key;
  }

  auto pctIndex = [&](float p) -> uint16_t {
    // p en [0,1], índice redondeado
    int idx = (int)roundf(p * (n - 1));
    if (idx < 0) idx = 0; if (idx >= (int)n) idx = n - 1;
    return buf[idx];
  };

  uint16_t p20 = pctIndex(0.20f);
  uint16_t p50 = pctIndex(0.50f);
  uint16_t p80 = pctIndex(0.80f);

  // Heurística de separación
  uint16_t span = (p80 > p20) ? (p80 - p20) : 0;

  if (span < 80) {
    // No hay separación clara fondo/línea -> fallback
    qtr_th_on  = th_on_default;
    qtr_th_off = (th_off_default < qtr_th_on) ? th_off_default : (qtr_th_on > 10 ? qtr_th_on - 10 : 0);
    Serial.printf("[AUTO-TH] separación débil (span=%u) -> fallback: ON=%u OFF=%u\n",
                  span, qtr_th_on, qtr_th_off);
    return;
  }

  // Coloca TH_OFF cerca del "fondo" y TH_ON bien por encima, con histéresis
  // OFF = p20 + 25% del span, ON = p20 + 65% del span
  uint16_t off_est = p20 + (uint16_t)(0.25f * span);
  uint16_t on_est  = p20 + (uint16_t)(0.65f * span);

  // Sanitiza y fuerza histéresis mínima de 10
  off_est = constrain(off_est, (uint16_t)0, (uint16_t)1000);
  on_est  = constrain(on_est,  (uint16_t)0, (uint16_t)1000);
  if (on_est <= off_est) on_est = (off_est + 10 <= 1000) ? (off_est + 10) : 1000;

  qtr_th_off = off_est;
  qtr_th_on  = on_est;

  Serial.printf("[AUTO-TH] p20=%u p50=%u p80=%u span=%u -> ON=%u OFF=%u\n",
                p20, p50, p80, span, qtr_th_on, qtr_th_off);
}
