// ======================= INCLUDES =======================
#include <Arduino.h>
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>   // por fabsf()

// ======================= BLE UUIDs =======================
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8" // CH1: PID/Vmax/ValTurb/(KTurb opcional) (READ/WRITE)
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-4688-b7f5-ea07361b26a8" // CH2: Offset (WRITE) + Lecturas (READ)

BLECharacteristic *pCharacteristic;    // CH1
BLECharacteristic *pCharacteristic_2;  // CH2

// ======================= MÓDULO DE INICIO =======================
const byte MInit = D3;
int   Estado = 0;   // 0/1 (general)

// ======================= TURBINA =======================
Servo myTurbina;
const int Tur = D4; // PIN PWM o Servo
int   ValTurb = 150;

bool is_Servo = false; // false: PWM (nuevo Sollow), true: servo (viejo Sollow)
int minvaltur=0;
int maxvaltur=0;
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
const uint16_t QTR_MAX_POS      = (NUM_SENSORS - 1) * 1000; // 15*1000 = 15000
const uint16_t QTR_DETECT_TH_ON = 250;   // umbral para "apareció" línea (0..1000 calibrado)
const uint16_t QTR_DETECT_TH_OFF= 180;   // umbral para "desapareció" (histéresis)
const bool     QTR_INVERT_LR    = false; // true si tu arreglo quedó espejado (invierte IZQ/DER)

static uint16_t lastPosRaw = QTR_MAX_POS / 2; // última pos válida (centro=7500)
static bool     hadLine    = true;            // estado "hay línea" con histéresis

// ======================= CONTROL =======================
float Tm = 9.0f; // ms (medido por loop)
float Referencia = 0.0f, Control = 0.0f, Kp = 2.0f, Ti = 0.0f, Td = 0.02f;
float Salida = 0.0f, Error = 0.0f, Error_ant = 0.0f, dError = 0.0f;
float offset = 1.0f, Vmax = 100.0f, E_integral = 0.0f;

// ======= FRENADO ADAPTATIVO (CURVA) =======
// Límite inferior de velocidad (piso) y rampa
float Vmin = 40.0f;          // PWM mínimo en rectas muy lentas / salida segura
float V_aplicada = 0.0f;     // PWM realmente aplicado tras la política

// Sensores de curva (ajusta a gusto)
float TH_ERR   = 0.6f;       // error normalizado a partir del cual considero "curva"
float TH_DERR  = 4.0f;       // |dE/dt| en 1/s (derivada) que indica giro brusco
int   ACTIVE_LO = 2;         // sensores sobre umbral para "piso" de ancho
int   ACTIVE_HI = 6;         // sensores sobre umbral para "techo" de ancho

// Pesos del score de curva (deben sumar ~1, pero no es obligatorio)
float W_ERR  = 0.55f;        // peso de |E|
float W_DERR = 0.35f;        // peso de |dE/dt|
float W_WID  = 0.10f;        // peso de ancho (nº de sensores activos)

// Forma de la reducción y rampas
float GAMMA_SHAPE = 1.0f;   // >1 reduce más agresivo al acercarse a 1
float RAMP_UP_PPS = 1200.0f; // subida máx (PWM por segundo)
float RAMP_DN_PPS = 2000.0f; // bajada máx (frenado rápido)

// Cache de lecturas para BLE (evita recomputos en READ)
float lastSalidaNorm = 0.0f;  // normalizada [-1, +1]
float lastRawLine    = 0.0f;  // ponderada 0..~15000

// ======================= BLUETOOTH (parse) =======================
String datos;     // buffer para sintonización CH1
String S_offset;  // buffer para offset

// ======================= PWM Motores =======================
const uint16_t Frecuencia = 5000;
const byte     Canales[]  = { 0, 1 , 2 };
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
    Vmax    = constrain(S_Vmax.toFloat(), 0, 1023); // techo de velocidad
    ValTurb = constrain((int)S_ValTurb.toFloat(), minvaltur, maxvaltur);
    if (S_KTurb.length() > 0) {
      KTurb = S_KTurb.toFloat();
      KTurb = constrain(KTurb, 0.0f, 2.0f);
    }

    Serial.printf("[BLE CH1] kp=%.3f ti=%.3f td=%.3f vmax=%.0f turb=%d kturb=%.3f\n",
                  Kp, Ti, Td, Vmax, ValTurb, KTurb);

    char pbuf[140];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }

  void onRead(BLECharacteristic *pc) override {
    char pbuf[140];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
             (double)offset, (double)KTurb);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }
};

// Característica 2: WRITE para offset; READ para lecturas de sensor
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
      val = S_offset.substring(eq + 1);
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
    } else {
      bool soloNumero = true;
      for (size_t i = 0; i < S_offset.length(); ++i) {
        char c = S_offset[i];
        if (!((c >= '0' && c <= '9') || c == '.' || c == '+' || c == '-' )) { soloNumero = false; break; }
      }
      if (soloNumero) {
        offset = S_offset.toFloat();
        Serial.println("[BLE CH2] OFFSET=" + String(offset));
      } else {
        Serial.println("[BLE CH2] Comando no reconocido (use OFFSET=... o ESTADO=0/1).");
      }
    }

    if (pCharacteristic) {
      char pbuf[140];
      snprintf(pbuf, sizeof(pbuf),
        "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f,ktur=%.3f",
        (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb,
        (double)offset, (double)KTurb);
      pCharacteristic->setValue((uint8_t*)pbuf, strlen(pbuf));
    }
  }

  void onRead(BLECharacteristic *pc2) override {
    char sbuf[80];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f",
             (double)lastSalidaNorm, (double)lastRawLine);
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

// --- Frenado adaptativo ---
float   ComputeCurveScore(float err, float derr);
void    ActualizarVelocidad();  // ajusta V_aplicada con rampas

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);
  
  if(is_Servo){
    minvaltur = 50;
    maxvaltur = 180;
  }else{
    minvaltur = 10;
    maxvaltur = 900;
  }

  Inicializacion_Pines();
  if(is_Servo){
    Inicializacion_turbina();
  }
  Inicializacion_Sensores();
  CrearPWM();
  Inicializacion_Bluetooth();

  delay(300);

  // Estado inicial seguro
  V_aplicada = Vmin; // empezamos lento
  if(is_Servo){
    myTurbina.write(ValTurb);
  }else{
    ledcWrite(Canales[2], ValTurb);
  }
}

// ======================= LOOP =======================
void loop() {
  //Estado = digitalRead(MInit); // << si quieres controlar por BLE, deja comentado

  while (Estado) {
    //Estado = digitalRead(MInit);

    Tinicio  = millis();
    Salida   = Lectura_Sensor();            // actualiza lastRawLine/lastSalidaNorm y Error
    Control  = Controlador(Referencia, Salida); // también rellena dError
    ActualizarVelocidad();                  // <-- calcula V_aplicada con frenado adaptativo
    Esfuerzo_Control(Control);

    // Turbina fija (o ligada al frenado si deseas, ver abajo)
    if(is_Servo){
      // myTurbina.write(ValTurb);
      // Si quieres que acompañe la reducción en curvas:
      /*int tcmd = map((int)V_aplicada, 0, (int)max(1.0f, Vmax), minvaltur, maxvaltur);
      myTurbina.write(constrain(tcmd, minvaltur, maxvaltur));*/
      myTurbina.write(ValTurb);
      // Esfuerzo_Turbina(); // alternativo proporcional a |Error|
    }else{
      // ledcWrite(Canales[2], ValTurb);
      /*int tcmd = map((int)V_aplicada, 0, (int)max(1.0f, Vmax), minvaltur, maxvaltur);
      ledcWrite(Canales[2], constrain(tcmd, minvaltur, maxvaltur));*/
      ledcWrite(Canales[2], ValTurb);
      // Esfuerzo_Turbina(); // alternativo proporcional a |Error|
    }

    Tm       = Tiempo_Muestreo(Tinicio);
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
  uint16_t pos = qtra.readLine(sensorValues);
  pos = 15000 - pos;

  uint16_t maxv = 0;
  uint32_t sumv = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = sensorValues[i];
    if (v > maxv) maxv = v;
    sumv += v;
  }

  bool lineNow = hadLine
                 ? (maxv > QTR_DETECT_TH_OFF)
                 : (maxv > QTR_DETECT_TH_ON);
  hadLine = lineNow;

  uint16_t posStable = lineNow ? pos : lastPosRaw;

  if (QTR_INVERT_LR) {
    posStable = QTR_MAX_POS - posStable;
  }

  lastRawLine    = (float)posStable;                                 // 0..15000
  Salida         = ((float)posStable / (QTR_MAX_POS / 2.0f)) - 1.0f; // ~[-1,+1]
  lastSalidaNorm = Salida;

  if (lineNow) lastPosRaw = posStable;

  return Salida;
}

void ActualizarLecturasCache() {
  if (!pCharacteristic_2) return;
  char sbuf[64];
  snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f",
           (double)lastSalidaNorm, (double)lastRawLine);
  pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
}

// Controlador PID (forma estándar con anti-windup)
float Controlador(float Ref, float Y) {
  float E_derivativo;
  float U;

  Error = Ref - Y;

    // Zona muerta +/-0.1
  if ((Error > -0.1f && Error < 0.0f) || (Error > 0.0f && Error < 0.1f)) {
    Error = 0.0f;
  }

  float Ts = max(Tm / 1000.0f, 1e-3f); // seg
  E_integral += ((Error * Ts) + (Ts * (Error - Error_ant)) / 2.0f);
  E_integral = constrain(E_integral, -100.0f, 100.0f);

  E_derivativo = (Error - Error_ant) / Ts;
  dError = E_derivativo; // <-- exponemos dE/dt para la política de frenado

  U = Kp * (Error + Ti * E_integral + Td * E_derivativo);
  Error_ant = Error;

  U = constrain(U, -2.5f, 2.5f);
  return U;
}

// Envía PWM a motores según offset +/- Control, usando V_aplicada (no Vmax)
void Esfuerzo_Control(float U) {
  float s1 = (offset - U); // Derecho
  float s2 = (offset + U); // Izquierdo

  int pwm1 = floor(constrain(fabsf(s1), 0.0f, 1.0f) * V_aplicada);
  int pwm2 = floor(constrain(fabsf(s2), 0.0f, 1.0f) * V_aplicada);

  ledcWrite(Canales[0], pwm1);
  ledcWrite(Canales[1], pwm2);

  digitalWrite(DirD, (s1 <= 0.0f) ? LOW  : HIGH);
  digitalWrite(DirI, (s2 <= 0.0f) ? LOW  : HIGH);
}

// Turbina proporcional al |Error| (alternativo, si lo prefieres)
void Esfuerzo_Turbina(){
  float estur = constrain(round(minvaltur + ((KTurb * fabsf(Error)) * (maxvaltur - minvaltur))),
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
  if(!is_Servo){
    pinMode(Tur, OUTPUT);
    ledcWrite(Canales[2], 0);
  }
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
    char pbuf[140];
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
    char sbuf[80];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f",
             (double)lastSalidaNorm, (double)lastRawLine);
    pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE Advertising iniciado (SOLLOW).");
}

// ======================= FRENADO ADAPTATIVO =======================

// Calcula un score de curva ∈ [0,1] usando |E|, |dE/dt| y ancho de línea
float ComputeCurveScore(float err, float derr) {
  // 1) componente por |E|
  float a = err / TH_ERR;
  a = fabsf(a);
  a = constrain(a, 0.0f, 1.0f);

  // 2) componente por |dE/dt|
  float b = fabsf(derr) / TH_DERR;
  b = constrain(b, 0.0f, 1.0f);

  // 3) componente por ancho (número de sensores por encima de umbral OFF)
  int active = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if ((int)sensorValues[i] > (int)QTR_DETECT_TH_OFF) active++;
  }
  float c = 0.0f;
  if (ACTIVE_HI > ACTIVE_LO) {
    c = (float)(active - ACTIVE_LO) / (float)(ACTIVE_HI - ACTIVE_LO);
  }
  c = constrain(c, 0.0f, 1.0f);

  // combinación ponderada
  float score = (W_ERR * a) + (W_DERR * b) + (W_WID * c);
  score = constrain(score, 0.0f, 1.0f);
  return score;
}

// Actualiza V_aplicada a partir del score, con rampas de subida/bajada
void ActualizarVelocidad() {
  float score = ComputeCurveScore(Error, dError);

  // target = Vmin + (Vmax - 0) * (1 - score)^gamma, pero dejamos Vmax como techo directo
  float reduct = powf(constrain(1.0f - score, 0.0f, 1.0f), GAMMA_SHAPE);
  float target = Vmin + (Vmax - Vmin) * reduct;

  // ramp limiter (en unidades PWM)
  float dt = max(Tm / 1000.0f, 1e-3f);
  float up = RAMP_UP_PPS * dt;
  float dn = RAMP_DN_PPS * dt;

  if (target > V_aplicada) {
    V_aplicada = min(target, V_aplicada + up);
  } else {
    V_aplicada = max(target, V_aplicada - dn);
  }

  // seguridad
  V_aplicada = constrain(V_aplicada, Vmin, Vmax);
}
