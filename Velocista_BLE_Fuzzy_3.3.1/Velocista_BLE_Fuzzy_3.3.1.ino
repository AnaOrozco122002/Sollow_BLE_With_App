// ======================= INCLUDES =======================
#include <Arduino.h>
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>   // por fabs()

//#define ROBOT_VARIANT_ZENIT
#define ROBOT_VARIANT_SOLLOW_OLD

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

// ======= Robustez ponderado (anti-saltos a 0/15000 cuando se pierde la línea) =======
const uint16_t QTR_MAX_POS   = (NUM_SENSORS - 1) * 1000; // 15*1000 = 15000
const bool     QTR_INVERT_LR = false; // true si espejado

// Umbrales AJUSTABLES por BLE (rango 0..1000 tras calibración)
uint16_t qtr_th_on  = 570;  // "apareció" línea
uint16_t qtr_th_off = 330;  // "desapareció" línea (histéresis)
static uint16_t lastPosRaw = QTR_MAX_POS / 2; // última pos válida (centro=7500)
static bool     hadLine    = true;            // estado "hay línea" con histéresis

// ======================= CONTROL =======================
float Tm = 4.0f; // ms (medido por loop)
float Referencia = 0.0f, Control = 0.0f, Kp = 3.2f, Ti = 0.0f, Td = 0.02f;
float Salida = 0.0f, Error = 0.0f, Error_ant = 0.0f;
float offset = 1.0f, Vmax = 0.0f, E_integral = 0.0f;

// ---- Selector de modo de control (PID o FUZZY) ----
enum ControlMode : uint8_t { MODE_PID=0, MODE_FUZZY=1 };
volatile ControlMode gMode = MODE_PID;

// Para dE (derivada del error) en FUZZY
static float prevE = 0.0f;
static unsigned long prevE_ms = 0;

// ======================= CACHÉ BLE =======================
float lastSalidaNorm = 0.0f;  // normalizada [-1, +1]
float lastRawLine    = 0.0f;  // ponderada 0..~15000

// ======================= BLUETOOTH (parse) =======================
String datos;     // buffer para sintonización CH1
String S_offset;  // buffer para offset

// ======================= PWM Motores (Arduino PWM API) =======================
const uint16_t Frecuencia = 5000;   // Hz
const uint8_t  Resolucion = 10;     // bits (0..1023)
const int PWMI = D5; // Motor Izquierdo PWM
const int PWMD = D8; // Motor Derecho  PWM
const int DirI = D6; // Dirección Izquierdo
const int DirD = D7; // Dirección Derecho

// ======================= ESTADO RUNTIME =======================
unsigned long Tinicio = 0;
bool conect = false;

// ======================= FUZZY (parametrizable por BLE) =======================
// Error e ~[-1,1] con margen
float FZ_E_SPAN  = 1.5f;   // extremo absoluto
float FZ_E_SMALL = 0.5f;   // frontera “pequeño”
float FZ_E_Z     = 0.2f;   // media-anchura de zona cero
// dError ~[-3,3]
float FZ_DE_SPAN  = 3.0f;
float FZ_DE_SMALL = 1.0f;
float FZ_DE_Z     = 0.3f;
// Singletons de salida (NG, NP, Z, PP, PG) en [-1..1] (se permiten ±1.5 por flex)
float FZ_OUT_NG = -1.0f;
float FZ_OUT_NP = -0.5f;
float FZ_OUT_Z  =  0.0f;
float FZ_OUT_PP =  0.5f;
float FZ_OUT_PG =  1.0f;
// Escala final (mapea U_fuzzy -> actuador)
float FZ_SCALE = 2.5f;

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

// ====== Utilidades y Fuzzy ======
static inline float clampf(float x, float a, float b){ return x<a?a:(x>b?b:x); }
static float tri(float x, float a, float b, float c){
  if (x<=a || x>=c) return 0.0f;
  if (x==b) return 1.0f;
  return (x<b) ? (x-a)/(b-a) : (c-x)/(c-b);
}
// genéricas con parámetros ajustables
static void fuzzify_generic(float x, float SPAN, float SMALL, float Z,
                            float &NG, float &NP, float &ZC, float &PP, float &PG) {
  x = clampf(x, -SPAN, SPAN);
  NG = tri(x, -SPAN, -SMALL, -Z);
  NP = tri(x, -SMALL, -Z,    0);
  ZC = tri(x, -Z,      0,    +Z);
  PP = tri(x,  0,     +Z,   +SMALL);
  PG = tri(x, +Z,    +SMALL,+SPAN);
}
static void fuzzifyErr(float e, float &NG, float &NP, float &ZC, float &PP, float &PG){
  fuzzify_generic(e, FZ_E_SPAN, FZ_E_SMALL, FZ_E_Z, NG, NP, ZC, PP, PG);
}
static void fuzzifyDE(float de, float &NG, float &NP, float &ZC, float &PP, float &PG){
  fuzzify_generic(de, FZ_DE_SPAN, FZ_DE_SMALL, FZ_DE_Z, NG, NP, ZC, PP, PG);
}
static float defuzzySingleton(float wNG,float wNP,float wZ,float wPP,float wPG){
  float num = wNG*FZ_OUT_NG + wNP*FZ_OUT_NP + wZ*FZ_OUT_Z + wPP*FZ_OUT_PP + wPG*FZ_OUT_PG;
  float den = wNG + wNP + wZ + wPP + wPG + 1e-6f;
  return clampf(num/den, -1.0f, 1.0f);
}
float ControladorFuzzyPD(float Ref, float Y, float dE){
  float e = Ref - Y;
  float eNG,eNP,eZ,ePP,ePG, dNG,dNP,dZ,dPP,dPG;
  fuzzifyErr(e,  eNG,eNP,eZ,ePP,ePG);
  fuzzifyDE(dE,  dNG,dNP,dZ,dPP,dPG);

  float wNG=0, wNP=0, wZ=0, wPP=0, wPG=0;

  // Reglas simétricas base
  wPG += fmaxf(ePG* dZ, fmaxf(ePG* dPP, ePG* dPG));
  wPP += ePP * dZ;    wPG += ePP * dPP;    wZ  += ePP * dNG;

  wNG += fmaxf(eNG* dZ, fmaxf(eNG* dNP, eNG* dNG));
  wNP += eNP * dZ;    wNG += eNP * dNP;    wZ  += eNP * dPP;

  wZ  += eZ * dZ;     wNP += eZ * dNP;     wPP += eZ * dPP;

  float U = defuzzySingleton(wNG,wNP,wZ,wPP,wPG);  // [-1..1]
  // Escala a tu rango de actuador (igual que PID ~±2.5)
  return clampf(U * FZ_SCALE, -2.5f, 2.5f);
}

// ======================= BLE CALLBACKS DE CARACTERÍSTICAS =======================

// Característica 1: Kp,Ti,Td,Vmax,ValTurb,(KTurb opcional)
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

// Característica 2: WRITE para offset/estado/umbrales/modo/fuzzy; READ para lecturas + snapshot fuzzy
class MyCallbacks_2 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc2) override {
    String value = pc2->getValue();
    if (value.length() == 0) return;

    S_offset = value;
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
      if (qtr_th_off >= qtr_th_on) {
        qtr_th_off = (qtr_th_on > 10) ? (qtr_th_on - 10) : 0;
      }
      Serial.printf("[BLE CH2] TH_ON=%u (ajustada TH_OFF=%u si era >=)\n", qtr_th_on, qtr_th_off);

    } else if (key == "TH_OFF") {
      int v = val.toInt();
      v = constrain(v, 0, 1000);
      if (v >= (int)qtr_th_on) {
        v = (qtr_th_on > 10) ? (qtr_th_on - 10) : 0;
      }
      qtr_th_off = v;
      Serial.printf("[BLE CH2] TH_OFF=%u\n", qtr_th_off);

    } else if (key == "CTRL") {
      String vup = val; vup.toUpperCase();
      if (vup.indexOf("PID") >= 0) gMode = MODE_PID;
      else if (vup.indexOf("FUZ") >= 0 || vup.indexOf("FUZZY") >= 0) gMode = MODE_FUZZY;
      Serial.printf("[BLE CH2] CTRL=%s\n", (gMode==MODE_PID?"PID":"FUZZY"));

    // ------- Ajustes Fuzzy -------
    } else if (key == "FZ_E") {
      int c1 = val.indexOf(',');
      int c2 = (c1>=0) ? val.indexOf(',', c1+1) : -1;
      if (c1>0 && c2>c1) {
        FZ_E_SPAN  = constrain(val.substring(0, c1).toFloat(), 0.5f, 5.0f);
        FZ_E_SMALL = constrain(val.substring(c1+1, c2).toFloat(), 0.05f, FZ_E_SPAN);
        FZ_E_Z     = constrain(val.substring(c2+1).toFloat(), 0.01f, FZ_E_SMALL);
        Serial.printf("[FZ_E] SPAN=%.2f SMALL=%.2f Z=%.2f\n", FZ_E_SPAN, FZ_E_SMALL, FZ_E_Z);
      }

    } else if (key == "FZ_DE") {
      int c1 = val.indexOf(',');
      int c2 = (c1>=0) ? val.indexOf(',', c1+1) : -1;
      if (c1>0 && c2>c1) {
        FZ_DE_SPAN  = constrain(val.substring(0, c1).toFloat(), 0.5f, 10.0f);
        FZ_DE_SMALL = constrain(val.substring(c1+1, c2).toFloat(), 0.05f, FZ_DE_SPAN);
        FZ_DE_Z     = constrain(val.substring(c2+1).toFloat(), 0.01f, FZ_DE_SMALL);
        Serial.printf("[FZ_DE] SPAN=%.2f SMALL=%.2f Z=%.2f\n", FZ_DE_SPAN, FZ_DE_SMALL, FZ_DE_Z);
      }

    } else if (key == "FZ_OUT") {
      float ng,np,zz,pp,pg;
      int c1 = val.indexOf(',');
      int c2 = (c1>=0) ? val.indexOf(',', c1+1) : -1;
      int c3 = (c2>=0) ? val.indexOf(',', c2+1) : -1;
      int c4 = (c3>=0) ? val.indexOf(',', c3+1) : -1;
      if (c4>c3 && c3>c2 && c2>c1 && c1>0) {
        ng = val.substring(0, c1).toFloat();
        np = val.substring(c1+1, c2).toFloat();
        zz = val.substring(c2+1, c3).toFloat();
        pp = val.substring(c3+1, c4).toFloat();
        pg = val.substring(c4+1).toFloat();
        FZ_OUT_NG = clampf(ng, -1.5f, 1.5f);
        FZ_OUT_NP = clampf(np, -1.5f, 1.5f);
        FZ_OUT_Z  = clampf(zz, -1.5f, 1.5f);
        FZ_OUT_PP = clampf(pp, -1.5f, 1.5f);
        FZ_OUT_PG = clampf(pg, -1.5f, 1.5f);
        Serial.printf("[FZ_OUT] NG=%.2f NP=%.2f Z=%.2f PP=%.2f PG=%.2f\n",
          FZ_OUT_NG,FZ_OUT_NP,FZ_OUT_Z,FZ_OUT_PP,FZ_OUT_PG);
      }

    } else if (key == "FZ_SCALE") {
      float s = val.toFloat();
      FZ_SCALE = clampf(s, 0.5f, 5.0f);
      Serial.printf("[FZ_SCALE] SCALE=%.2f\n", FZ_SCALE);

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
        Serial.println("[BLE CH2] Comando no reconocido (use OFFSET=..., ESTADO=0/1, TH_ON=..., TH_OFF=..., CTRL=PID/FUZ, FZ_...).");
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
    char sbuf[240];
    snprintf(sbuf, sizeof(sbuf),
      "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u,mode=%s,"
      "fz_e=%.2f/%.2f/%.2f,fz_de=%.2f/%.2f/%.2f,fz_out=%.2f/%.2f/%.2f/%.2f/%.2f,fz_scale=%.2f",
      (double)lastSalidaNorm, (double)lastRawLine,
      (unsigned)qtr_th_on, (unsigned)qtr_th_off,
      (gMode==MODE_PID?"PID":"FUZZY"),
      (double)FZ_E_SPAN,(double)FZ_E_SMALL,(double)FZ_E_Z,
      (double)FZ_DE_SPAN,(double)FZ_DE_SMALL,(double)FZ_DE_Z,
      (double)FZ_OUT_NG,(double)FZ_OUT_NP,(double)FZ_OUT_Z,(double)FZ_OUT_PP,(double)FZ_OUT_PG,
      (double)FZ_SCALE
    );
    pc2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }
};

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);

  if (is_Servo) { minvaltur = 0; maxvaltur = 180; }
  else          { minvaltur = 0; maxvaltur = 900; }

  Inicializacion_Pines();
  if (is_Servo) { Inicializacion_turbina(); }
  Inicializacion_Sensores();
  CrearPWM();
  Inicializacion_Bluetooth();
  delay(300);

  if (is_Servo) myTurbina.write(ValTurb);
  else          analogWrite(Tur, ValTurb);

  prevE_ms = millis();
}

// ======================= LOOP =======================
void loop() {
  //Estado = digitalRead(MInit);  // Si deseas control SOLO por BLE, deja comentada esta línea

  while (Estado) {
    //Estado = digitalRead(MInit); // idem

    Tinicio  = millis();
    Salida   = Lectura_Sensor(); // actualiza lastRawLine/lastSalidaNorm

    // === DERIVADA DEL ERROR para FUZZY ===
    unsigned long now = millis();
    float Ts = (now - prevE_ms) / 1000.0f; if (Ts < 1e-3f) Ts = 1e-3f;
    float e = Referencia - Salida;
    float dE = (e - prevE) / Ts;
    prevE = e; prevE_ms = now;

    // === CONTROL ===
    if (gMode == MODE_FUZZY) {
      Control = ControladorFuzzyPD(Referencia, Salida, dE); // ya escalado y saturado
    } else {
      Control  = Controlador(Referencia, Salida); // PID
    }

    // === ACTUAR ===
    Esfuerzo_Control(Control);
    Tm = Tiempo_Muestreo(Tinicio);

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

// Lee posición de la línea (ponderado) con robustez de "línea perdida"
float Lectura_Sensor(void) {
  uint16_t pos = qtra.readLine(sensorValues);
  uint16_t maxv = 0;
  uint32_t sumv = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = sensorValues[i];
    if (v > maxv) maxv = v;
    sumv += v;
  }

  bool lineNow = hadLine ? (maxv > qtr_th_off) : (maxv > qtr_th_on);
  hadLine = lineNow;

  uint16_t posStable = lineNow ? pos : lastPosRaw;
  if (QTR_INVERT_LR) posStable = QTR_MAX_POS - posStable;

  lastRawLine = (float)posStable;                                      // 0..15000
  Salida      = ((float)posStable / (QTR_MAX_POS / 2.0f)) - 1.0f;      // ~[-1,+1]
  lastSalidaNorm = Salida;

  if (lineNow) lastPosRaw = posStable;
  return Salida;
}

void ActualizarLecturasCache() {
  if (!pCharacteristic_2) return;
  char sbuf[240];
  snprintf(sbuf, sizeof(sbuf),
    "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u,mode=%s,"
    "fz_e=%.2f/%.2f/%.2f,fz_de=%.2f/%.2f/%.2f,fz_out=%.2f/%.2f/%.2f/%.2f/%.2f,fz_scale=%.2f",
    (double)lastSalidaNorm, (double)lastRawLine,
    (unsigned)qtr_th_on, (unsigned)qtr_th_off,
    (gMode==MODE_PID?"PID":"FUZZY"),
    (double)FZ_E_SPAN,(double)FZ_E_SMALL,(double)FZ_E_Z,
    (double)FZ_DE_SPAN,(double)FZ_DE_SMALL,(double)FZ_DE_Z,
    (double)FZ_OUT_NG,(double)FZ_OUT_NP,(double)FZ_OUT_Z,(double)FZ_OUT_PP,(double)FZ_OUT_PG,
    (double)FZ_SCALE
  );
  pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
}

// Controlador PID (forma estándar con anti-windup y zona muerta pequeña)
float Controlador(float Ref, float Y) {
  float E_derivativo;
  float U;
  Error = Ref - Y;

  // Zona muerta +/-0.15
  if ((Error > -0.15f && Error < 0.0f) || (Error > 0.0f && Error < 0.15f)) {
    Error = 0.0f;
  }

  float Ts = Tm / 1000.0f; // seg
  E_integral += ((Error * Ts) + (Ts * (Error - Error_ant)) / 2.0f);
  E_integral = constrain(E_integral, -100.0f, 100.0f);

  E_derivativo = (Error - Error_ant) / max(Ts, 1e-3f);
  U = Kp * (Error + Ti * E_integral + Td * E_derivativo);
  Error_ant = Error;

  U = constrain(U, -2.5f, 2.5f);
  return U;
}

// Envía PWM a motores según offset +/- Control
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

// Turbina proporcional al |Error| (opcional)
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

// PARA TURBINA COMO SERVO
void Inicializacion_turbina() {
  ESP32PWM::allocateTimer(2);
  myTurbina.setPeriodHertz(50);
  myTurbina.attach(Tur, 1000, 2000);
  myTurbina.write(0);
}

void Inicializacion_Sensores() {
  // Calibración simple de QTR (aprox ~7.5 s)
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
  if (!is_Servo) {
    pinMode(Tur, OUTPUT);
  }
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
    char sbuf[240];
    snprintf(sbuf, sizeof(sbuf),
      "salida=%.3f,raw=%.0f,th_on=%u,th_off=%u,mode=%s,"
      "fz_e=%.2f/%.2f/%.2f,fz_de=%.2f/%.2f/%.2f,fz_out=%.2f/%.2f/%.2f/%.2f/%.2f,fz_scale=%.2f",
      (double)lastSalidaNorm, (double)lastRawLine,
      (unsigned)qtr_th_on, (unsigned)qtr_th_off,
      (gMode==MODE_PID?"PID":"FUZZY"),
      (double)FZ_E_SPAN,(double)FZ_E_SMALL,(double)FZ_E_Z,
      (double)FZ_DE_SPAN,(double)FZ_DE_SMALL,(double)FZ_DE_Z,
      (double)FZ_OUT_NG,(double)FZ_OUT_NP,(double)FZ_OUT_Z,(double)FZ_OUT_PP,(double)FZ_OUT_PG,
      (double)FZ_SCALE
    );
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
