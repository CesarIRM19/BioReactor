/*
 *  SENSORES:
 *    - Turbidez : LED IR (GPIO 15) + Fotodiodo (GPIO 34)
 *    - Temperatura : DS18B20 (GPIO 4)
 *    - Luz ambiente : LDR (GPIO 33)
 *
 *  ACTUADORES (via relé, activo en LOW):
 *    - Bomba de aire   : GPIO 18
 *    - Resistencia PTC : GPIO 5
 *    - Foco LED        : GPIO 17
 *
 *  THINGSPEAK MQTT — Canales de campo:
 *    field1 → Turbidez (NTU)
 *    field2 → TB_norm (TBt/TB0)
 *    field3 → Temperatura (°C)
 *    field4 → Luz LDR (%)
 *    field5 → Foco LED (1=ON, 0=OFF)
 *    field6 → Resistencia PTC (1=ON, 0=OFF)
 *    field7 → Bomba de aire (1=ON, 0=OFF)
 *    field8 → Fase de crecimiento (0=LAG,1=EXP,2=STAT,3=MUERTE)
 * ============================================================
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <time.h>
#include <PubSubClient.h> 


const char* WIFI_SSID     = "";
const char* WIFI_PASSWORD = "";

const char* TS_MQTT_SERVER   = "mqtt3.thingspeak.com";  
const int   TS_MQTT_PORT     = 1883;                     
const char* TS_CLIENT_ID     = "";     
const char* TS_MQTT_USER     = "";       
const char* TS_MQTT_PASSWORD = "";       
const long  TS_CHANNEL_ID    = 123456;                   

char TS_TOPIC[50];

//  NTP
const char* NTP_SERVER          = "pool.ntp.org";
const long  GMT_OFFSET_SEC      = -21600;   
const int   DAYLIGHT_OFFSET_SEC = 0;

//  PINES
const int PIN_LED_IR      = 15;
const int PIN_FOTODIODO   = 34;
const int PIN_DS18B20     = 4;
const int PIN_LDR         = 33;
const int PIN_RELE_BOMBA  = 18;
const int PIN_RELE_PTC    = 5;
const int PIN_RELE_LED    = 17;


//  PARÁMETROS DE CONTROL
const float TEMP_MIN      = 25.0;
const float TEMP_MAX      = 30.0;
const float TEMP_OPTIMO   = 27.5;

const int HORA_LUZ_INICIO = 18;
const int HORA_LUZ_FIN    = 6;

const float NTU_LAG_MAX   = 20.0;
const float NTU_EXP_MAX   = 70.0;
const float NTU_STAT_MAX  = 100.0;

// Intervalo de muestreo
const unsigned long INTERVALO_MS = 300000UL;

const int LDR_UMBRAL_OSCURO = 500;

const unsigned long BOMBA_ON_MS  = 270000UL;
const unsigned long BOMBA_OFF_MS =  30000UL;

const unsigned long MQTT_RECONNECT_TIMEOUT = 5000UL;

//  OBJETOS
OneWire           oneWireBus(PIN_DS18B20);
DallasTemperature ds18b20(&oneWireBus);

WiFiClient    wifiClient;           
PubSubClient  mqttClient(wifiClient); 

// VARIABLES
float TB0              = -1.0;
float lastTBnorm       = 1.0;
unsigned long lastMillis     = 0;
unsigned long lastBombaMs    = 0;
bool  bombaEncendida         = false;
bool  wifiConectado          = false;
bool  ntpSincronizado        = false;
bool  ldrDetectaLuz          = false;
unsigned long lastLDRcheckMs = 0;


//  FUNCIONES
float   leerTurbidezNTU();
float   mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float   calcularNTU(float v);
float   leerTemperatura();
float   leerLDR_relativo();
String  estimarFase(float ntu, float tb_norm);
int     faseANumero(String fase);
void    controlarTemperatura(float tempC);
void    controlarLuz();
void    controlarBomba();
void    conectarWiFi();
void    sincronizarNTP();
bool    conectarMQTT();            
void    publicarThingSpeak(float ntu, float tb_norm, float tempC, float lux_rel,
                            bool ledOn, bool ptcOn, bool bombaOn, String fase); 
void    imprimirEstado(float ntu, float tb0, float tb_norm, float tempC,
                       float lux_rel, bool ledOn, bool ptcOn, bool bombaOn,
                       String fase, String timestamp);
String  obtenerTimestamp();

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║   FOTOBIORREACTOR IoT — ESP32 + MQTT     ║");
  Serial.println("║   Iniciando sistema...                   ║");
  Serial.println("╚══════════════════════════════════════════╝\n");

  snprintf(TS_TOPIC, sizeof(TS_TOPIC), "channels/%ld/publish", TS_CHANNEL_ID);
  Serial.print("[MQTT]    Topic: "); Serial.println(TS_TOPIC);

  // Pines actuadores
  pinMode(PIN_RELE_BOMBA, OUTPUT); digitalWrite(PIN_RELE_BOMBA, HIGH);
  pinMode(PIN_RELE_PTC,   OUTPUT); digitalWrite(PIN_RELE_PTC,   HIGH);
  pinMode(PIN_RELE_LED,   OUTPUT); digitalWrite(PIN_RELE_LED,   HIGH);

  // Pines sensores
  pinMode(PIN_LED_IR,    OUTPUT); digitalWrite(PIN_LED_IR, LOW);
  pinMode(PIN_FOTODIODO, INPUT);
  pinMode(PIN_LDR,       INPUT);
  analogSetAttenuation(ADC_11db);

  // DS18B20
  ds18b20.begin();
  Serial.println("[DS18B20] Inicializado.");

  // WiFi
  conectarWiFi();
  if (wifiConectado) {
    sincronizarNTP();

    // MQTT
    mqttClient.setServer(TS_MQTT_SERVER, TS_MQTT_PORT);
    mqttClient.setKeepAlive(60)
    conectarMQTT();
  }

  // Bomba: inicia encendida
  digitalWrite(PIN_RELE_BOMBA, LOW);
  bombaEncendida = true;
  lastBombaMs    = millis();
  Serial.println("[BOMBA]   Aireación iniciada.");

  // TB0 — lectura inicial
  Serial.println("\n[TURBIDEZ] Midiendo TB0...");
  delay(2000);
  TB0 = leerTurbidezNTU();
  if (TB0 <= 0.0) TB0 = 1.0;
  Serial.print("[TURBIDEZ] TB0 = "); Serial.print(TB0, 2); Serial.println(" NTU");

  // Lectura inicial completa
  Serial.println("\n[INICIO]  Tomando lectura inicial...");
  controlarLuz();
  {
    float ntu_0     = leerTurbidezNTU();
    float tb_norm_0 = ntu_0 / TB0;
    float tempC_0   = leerTemperatura();
    float lux_rel_0 = leerLDR_relativo();
    String fase_0   = estimarFase(ntu_0, tb_norm_0);
    bool ledOn_0    = (digitalRead(PIN_RELE_LED) == LOW);
    bool ptcOn_0    = (digitalRead(PIN_RELE_PTC) == LOW);

    imprimirEstado(ntu_0, TB0, tb_norm_0, tempC_0, lux_rel_0,
                   ledOn_0, ptcOn_0, bombaEncendida, fase_0, obtenerTimestamp());

    // Publicar lectura inicial
    publicarThingSpeak(ntu_0, tb_norm_0, tempC_0, lux_rel_0,
                       ledOn_0, ptcOn_0, bombaEncendida, fase_0);
  }

  lastMillis = millis();
  Serial.println("[SISTEMA] Listo. Próxima lectura en 5 min.\n");
}

void loop() {
  unsigned long ahora = millis();

  // Mantener conexión viva
  if (wifiConectado) {
    if (!mqttClient.connected()) {
      conectarMQTT();
    }
    mqttClient.loop();
  }

  // Ciclo de la bomba
  controlarBomba();

  // Ciclo de muestreo cada 5 min
  if (ahora - lastMillis >= INTERVALO_MS) {
    lastMillis = ahora;

    float ntu     = leerTurbidezNTU();
    float tb_norm = (TB0 > 0.0) ? (ntu / TB0) : 1.0;
    float tempC   = leerTemperatura();
    float lux_rel = leerLDR_relativo();
    String fase   = estimarFase(ntu, tb_norm);

    controlarTemperatura(tempC);
    controlarLuz();

    bool ledOn   = (digitalRead(PIN_RELE_LED) == LOW);
    bool ptcOn   = (digitalRead(PIN_RELE_PTC) == LOW);
    bool bombaOn = bombaEncendida;

    imprimirEstado(ntu, TB0, tb_norm, tempC, lux_rel,
                   ledOn, ptcOn, bombaOn, fase, obtenerTimestamp());

    // Publicar a ThingSpeak
    publicarThingSpeak(ntu, tb_norm, tempC, lux_rel, ledOn, ptcOn, bombaOn, fase);

    lastTBnorm = tb_norm;
  }

  delay(100);
}

// CONECTAR AL BROKER THINGSPEAK
bool conectarMQTT() {
  if (mqttClient.connected()) return true;

  Serial.print("[MQTT]    Conectando a ThingSpeak...");

  String clientId = String(TS_CLIENT_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  unsigned long inicio = millis();
  while (!mqttClient.connected()) {
    if (millis() - inicio > MQTT_RECONNECT_TIMEOUT) {
      Serial.println("\n[MQTT]    TIMEOUT. Se reintentará en el próximo ciclo.");
      return false;
    }

    if (mqttClient.connect(clientId.c_str(), TS_MQTT_USER, TS_MQTT_PASSWORD)) {
      Serial.println(" ¡Conectado!");
      return true;
    } else {
      Serial.print(".");
      delay(500);
    }
  }
  return false;
}

// PUBLICAR DATOS A THINGSPEAK
void publicarThingSpeak(float ntu, float tb_norm, float tempC, float lux_rel,
                         bool ledOn, bool ptcOn, bool bombaOn, String fase) {

  if (!wifiConectado) {
    Serial.println("[MQTT]    Sin WiFi — publicación omitida.");
    return;
  }

  if (!mqttClient.connected()) {
    Serial.println("[MQTT]    Sin conexión — reintentando antes de publicar...");
    if (!conectarMQTT()) return;
  }

  // Construir payload
  char payload[256];
  snprintf(payload, sizeof(payload),
    "field1=%.2f&field2=%.4f&field3=%.2f&field4=%.1f&field5=%d&field6=%d&field7=%d&field8=%d",
    ntu,
    tb_norm,
    tempC,
    lux_rel,
    ledOn   ? 1 : 0,
    ptcOn   ? 1 : 0,
    bombaOn ? 1 : 0,
    faseANumero(fase)
  );

  Serial.print("[MQTT]    Publicando → "); Serial.println(payload);

  bool ok = mqttClient.publish(TS_TOPIC, payload);
  if (ok) {
    Serial.println("[MQTT]    ✓ Publicado correctamente.");
  } else {
    Serial.println("[MQTT]    ✗ Error al publicar. Estado: " + String(mqttClient.state()));
  }
}

//  Convierte nombre de fase a entero para ThingSpeak
int faseANumero(String fase) {
  if (fase == "LAG")          return 0;
  if (fase == "EXPONENCIAL")  return 1;
  if (fase == "ESTACIONARIA") return 2;
  return 3;  // MUERTE
}

// SENSOR DE TURBIDEZ
float leerTurbidezNTU() {
  digitalWrite(PIN_LED_IR, HIGH);
  delay(20);
  long suma = 0;
  for (int i = 0; i < 64; i++) {
    suma += analogRead(PIN_FOTODIODO);
    delayMicroseconds(200);
  }
  float voltaje = (suma / 64.0) * 3.3 / 4095.0;
  digitalWrite(PIN_LED_IR, LOW);
  return calcularNTU(voltaje);
}

float calcularNTU(float v) {
  if (v >= 3.0) return 0.0;
  if (v >= 2.5) return mapFloat(v, 3.0, 2.5, 0.0, 20.0);
  if (v >= 1.8) return mapFloat(v, 2.5, 1.8, 20.0, 100.0);
  if (v >= 0.9) return mapFloat(v, 1.8, 0.9, 100.0, 400.0);
  return 400.0 + mapFloat(v, 0.9, 0.0, 0.0, 1000.0);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// SENSOR DE TEMPERATURA
float leerTemperatura() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) {
    Serial.println("[DS18B20] ERROR: sensor desconectado.");
    return -999.0;
  }
  return t;
}

// SENSOR DE LUZ (LDR)
float leerLDR_relativo() {
  int raw = analogRead(PIN_LDR);
  return (raw / 4095.0) * 100.0;
}

// ESTIMACIÓN DE FASE
String estimarFase(float ntu, float tb_norm) {
  if (ntu <= NTU_LAG_MAX && tb_norm < 1.2)          return "LAG";
  if (ntu <= NTU_EXP_MAX && tb_norm >= 1.2)         return "EXPONENCIAL";
  if (ntu <= NTU_STAT_MAX)                           return "ESTACIONARIA";
  return "MUERTE";
}

// CONTROL DE TEMPERATURA
void controlarTemperatura(float tempC) {
  if (tempC == -999.0) return;
  bool ptcActivo = (digitalRead(PIN_RELE_PTC) == LOW);
  if (!ptcActivo && tempC < (TEMP_MIN - 0.5)) {
    digitalWrite(PIN_RELE_PTC, LOW);
    Serial.println("[PTC]     Resistencia ENCENDIDA.");
  } else if (ptcActivo && tempC >= TEMP_MAX) {
    digitalWrite(PIN_RELE_PTC, HIGH);
    Serial.println("[PTC]     Resistencia APAGADA.");
  }
}

// CONTROL DE LUZ (L/D)
void controlarLuz() {
  if (ntpSincronizado) {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) goto fallback_ldr;

    int horaActual = timeinfo.tm_hour;
    bool dentroDeIntervalo = (HORA_LUZ_INICIO > HORA_LUZ_FIN)
      ? (horaActual >= HORA_LUZ_INICIO || horaActual < HORA_LUZ_FIN)
      : (horaActual >= HORA_LUZ_INICIO && horaActual < HORA_LUZ_FIN);

    if (dentroDeIntervalo) {
      digitalWrite(PIN_RELE_LED, LOW);
      Serial.println("[LUZ]     Foco ON (NTP).");
    } else {
      digitalWrite(PIN_RELE_LED, HIGH);
      unsigned long ahora = millis();
      if (ahora - lastLDRcheckMs >= 3600000UL) {
        lastLDRcheckMs = ahora;
        ldrDetectaLuz = (analogRead(PIN_LDR) > LDR_UMBRAL_OSCURO);
        Serial.print("[LUZ]     Foco OFF. Luz ambiente: ");
        Serial.println(ldrDetectaLuz ? "SI" : "NO");
      }
    }
    return;
  }

  fallback_ldr:
  {
    unsigned long ahora = millis();
    if (ahora - lastLDRcheckMs >= 3600000UL) {
      lastLDRcheckMs = ahora;
      ldrDetectaLuz = (analogRead(PIN_LDR) > LDR_UMBRAL_OSCURO);
      Serial.print("[LUZ]     Sin NTP. Luz ambiente: ");
      Serial.println(ldrDetectaLuz ? "SI (foco OFF)" : "NO (foco ON)");
    }
    digitalWrite(PIN_RELE_LED, ldrDetectaLuz ? HIGH : LOW);
  }
}

// CONTROL DE BOMBA
void controlarBomba() {
  unsigned long ahora = millis();
  unsigned long tiempoEnEstado = ahora - lastBombaMs;
  if (bombaEncendida) {
    if (tiempoEnEstado >= BOMBA_ON_MS) {
      digitalWrite(PIN_RELE_BOMBA, HIGH);
      bombaEncendida = false;
      lastBombaMs    = ahora;
      Serial.println("[BOMBA]   Pausa.");
    }
  } else {
    if (tiempoEnEstado >= BOMBA_OFF_MS) {
      digitalWrite(PIN_RELE_BOMBA, LOW);
      bombaEncendida = true;
      lastBombaMs    = ahora;
      Serial.println("[BOMBA]   Reanudada.");
    }
  }
}

//  WiFi
void conectarWiFi() {
  Serial.print("[WiFi]    Conectando a: "); Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500); Serial.print("."); intentos++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiConectado = true;
    Serial.println("\n[WiFi]    ¡Conectado! IP: " + WiFi.localIP().toString());
  } else {
    wifiConectado = false;
    Serial.println("\n[WiFi]    FALLO.");
  }
}

//  NTP
void sincronizarNTP() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  Serial.print("[NTP]     Sincronizando");
  struct tm timeinfo;
  int intentos = 0;
  while (!getLocalTime(&timeinfo) && intentos < 10) {
    delay(500); Serial.print("."); intentos++;
  }
  if (getLocalTime(&timeinfo)) {
    ntpSincronizado = true;
    Serial.println("\n[NTP]     Hora: " + obtenerTimestamp());
  } else {
    ntpSincronizado = false;
    Serial.println("\n[NTP]     FALLO.");
  }
}

//  TIMESTAMP ISO 8601
String obtenerTimestamp() {
  if (!ntpSincronizado) return "1970-01-01T00:00:00Z";
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "ERROR";
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buffer);
}

//  IMPRIMIR ESTADO
void imprimirEstado(float ntu, float tb0, float tb_norm, float tempC,
                    float lux_rel, bool ledOn, bool ptcOn, bool bombaOn,
                    String fase, String timestamp) {
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print  ("  Timestamp  : "); Serial.println(timestamp);
  Serial.println("──────────────────── SENSORES ──────────────");
  Serial.print  ("  Turbidez   : "); Serial.print(ntu, 2);     Serial.println(" NTU");
  Serial.print  ("  TB0        : "); Serial.print(tb0, 2);     Serial.println(" NTU");
  Serial.print  ("  TB_norm    : "); Serial.print(tb_norm, 4); Serial.println(" (TBt/TB0)");
  Serial.print  ("  Temperatura: "); Serial.print(tempC, 2);   Serial.println(" °C");
  Serial.print  ("  Luz (LDR)  : "); Serial.print(lux_rel, 1);Serial.println(" %");
  Serial.println("──────────────────── FASE ───────────────────");
  Serial.print  ("  Estimada   : "); Serial.println(fase);
  Serial.println("──────────────────── ACTUADORES ─────────────");
  Serial.print  ("  Foco LED   : "); Serial.println(ledOn   ? "ON  ✓" : "OFF");
  Serial.print  ("  Resist PTC : "); Serial.println(ptcOn   ? "ON  ✓" : "OFF");
  Serial.print  ("  Bomba aire : "); Serial.println(bombaOn ? "ON  ✓" : "OFF");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}
