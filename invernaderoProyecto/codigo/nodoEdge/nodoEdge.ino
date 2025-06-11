/**
  @file nodoEdge.ino
 * @brief Sistema ESP32 para recepción de datos de sensores, almacenamiento en SD,
 *        envío de resúmenes vía Telegram y control de actuadores via ESP-NOW.
 *
 * @author Briyith Guacas y Karol Palechor
 * @date 2025-06-11
 */

// Librerías
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <float.h>
#include <SD.h>
#include "time.h"
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

// Configuración NTP y Wi-Fi
static const char* ssid = "";                          /**< SSID de la red Wi-Fi */
static const char* pass = "";                          /**< Contraseña de la red Wi-Fi */
static const char* ntpServer = "time.google.com";      /**< Servidor NTP */
static const long gmtOffset = -18000;                  /**< Offset GMT en segundos (Bogotá: -5h) */
static const int dstOffset = 0;                        /**< Offset horario de verano */

// Telegram Bot
#define BOTtoken ""                                    /**< Token del bot Telegram */
#define CHAT_ID  ""                                    /**< Chat ID destino */
WiFiClientSecure client;                               /**< Cliente TLS para Telegram */
UniversalTelegramBot bot(BOTtoken, client);            /**< Instancia del bot Telegram */

// Pines y rutas de almacenamiento
#define CS_PIN      4                                  /**< Pin CS para tarjeta SD */
#define ARCH_CONFIG "/config.json"                     /**< Ruta de configuración en LittleFS */
File root;                                             /**< Directorio raíz abierto */

// JSON
DynamicJsonDocument jsonConfig(2048);                  /**< Buffer JSON global */

// Variables globales de sensores
float temperatura_val = 0.0f;                          /**< Temperatura (sensor primario) */
float humedad_val = 0.0f;                              /**< Humedad relativa */
float temperaturaS_val = 0.0f;                         /**< Temperatura DS18B20 */
int luz_val = 0;                                       /**< Nivel de luz */
int RSSI = 0;                                          /**< Intensidad de señal Wi-Fi */

// Semáforos y flags de estado
SemaphoreHandle_t mutexHora;                           /**< Semáforo para acceso a hora */
SemaphoreHandle_t printMutex;                          /**< Semáforo para impresión serie */
bool wifiConectado = false;                            /**< Flag de conexión Wi-Fi */
bool tiempoObtenido = false;                           /**< Flag de sincronización NTP */
bool sdInicializada = false;                           /**< Flag de inicialización SD */
struct tm timeinfo;                                    /**< Estructura de tiempo obtenida */
String alertMsg = "";                                  /**< Mensajes de alerta de condiciones */

// Buffers de recepción ESP-NOW
uint8_t bufferDatos[64];                               /**< Buffer de datos entrantes */
size_t tamEsperado = 0;                                /**< Tamaño esperado de datos sensores */

// Estructura para mensajes a actuadores
typedef struct {
  bool activateMotorA;                                 /**< Activar Motor A */
  bool activateMotorB;                                 /**< Activar Motor B */
  bool activateLED;                                    /**< Activar LED */
  int daySent;                                         /**< Día de envío para evitar duplicados */
} struct_message;

// Datos persistentes en Deep Sleep
RTC_DATA_ATTR int lastSentA = -1, lastSentB = -1, lastSentLED = -1;                         /**< Días de última activación */
RTC_DATA_ATTR unsigned long lastTryA = 0, lastTryB = 0, lastTryLED = 0;                     /**< Tiempo última tentativa */
RTC_DATA_ATTR bool motorAConfirmed = false, motorBConfirmed = false, ledConfirmed = false;  /**< Confirmaciones */

static const unsigned long RETRY_INTERVAL = 1000;      /**< Intervalo de reintento en ms */
uint8_t actuadorAddr[6];                               /**< Dirección MAC del actuador */

// Horarios programados de actuadores: inicio/fin horas y minutos
int mA_sh, mA_sm, mA_eh, mA_em;                        /**< Motor A */
int mB_sh, mB_sm, mB_eh, mB_em;                        /**< Motor B */
int l_sh, l_sm, l_eh, l_em;                            /**< LED */

// Prototipos de funciones
void createDir(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, const char *message);
void appendFile(fs::FS &fs, const char *path, const char *message);
bool cargarConfiguracion();
bool cargarEstructuraPorMAC(const String &macStr);
void interpretarDatos(const uint8_t *datos, int len, const String &macStr);
String macToString(const uint8_t *mac);
void entrarDeepSleep();
void OnDataRecv(const esp_now_recv_info_t *, const uint8_t *, int);
void onSent(const uint8_t *mac, esp_now_send_status_t status);
void onRecvConf(const esp_now_recv_info_t *, const uint8_t *, int);
void sendMessage(bool a, bool b, bool led, int day);
void parseHourMinute(const char *str, int &h, int &m);
bool loadConfigActu();
bool isTimeInRange(int h, int mi, int sh, int sm, int eh, int em);
void ensureDirCreated(fs::FS &fs, const char *path);
void writeOrAppendFile(fs::FS &fs, const char *path, const char *message);

// FreeRTOS Tasks
/**
 * @brief Tarea que muestra datos periódicamente (placeholder).
 * @param pvParameters Parámetros (no usados).
 */
void tareaMostrarDatos(void *pvParameters) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

/**
 * @brief Tarea para conectar ESP32 a Wi-Fi.
 * @param param Parámetro (no usado).
 */
void tareaWiFi(void *param) {
  WiFi.begin(ssid, pass);
  Serial.printf("Conectando a %s", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  RSSI = WiFi.RSSI();
  wifiConectado = true;

  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  bot.sendMessage(CHAT_ID,
                  "🤖 ESP32 listo. Arrancó y sincronizó Wi-Fi correctamente.",
                  "");
  vTaskDelete(NULL);
}

/**
 * @brief Tarea para sincronizar hora mediante NTP.
 * @param param Parámetro (no usado).
 */
void tareaTiempoNTP(void *param) {
  while (!wifiConectado) vTaskDelay(pdMS_TO_TICKS(100));
  configTime(gmtOffset, dstOffset, ntpServer);
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (!getLocalTime(&timeinfo)) {
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  tiempoObtenido = true;
  vTaskDelete(NULL);
}

/**
 * @brief Tarea para inicializar tarjeta SD.
 * @param param Parámetro (no usado).
 */
void tareaSD(void *param) {
  Serial.print("Inicializando tarjeta SD... ");
  if (!SD.begin(CS_PIN)) {
    vTaskDelete(NULL);
  } else {
    sdInicializada = true;
    vTaskDelete(NULL);
  }
}

/**
 * @brief Tarea principal: crea archivo CSV, envía Telegram y controla actuadores.
 * @param param Parámetro (no usado).
 */
void tareaCrearArchivo(void *param) {
  // Esperar inicialización SD y hora NTP
  while (!sdInicializada || !tiempoObtenido) {
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  // Leer hora con mutex
  xSemaphoreTake(mutexHora, portMAX_DELAY);
  getLocalTime(&timeinfo, 1000);
  xSemaphoreGive(mutexHora);

  // Crear ruta: /YYYYMMDD/HH.csv
  char dirName[16], fileName[16];
  strftime(dirName, sizeof(dirName), "/%Y%m%d", &timeinfo);
  strftime(fileName, sizeof(fileName), "%H.csv", &timeinfo);
  ensureDirCreated(SD, dirName);

  // Ensamblar contenido de la línea
  char filePath[48];
  snprintf(filePath, sizeof(filePath), "%s/%s", dirName, fileName);
  String linea;
  {
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d%H%M%S", &timeinfo);
    linea = String(ts) + ",";
    linea += "1," + String(RSSI) + ",";
    linea += String(temperatura_val, 2) + ",";
    linea += String(humedad_val, 2) + ",";
    linea += String(temperaturaS_val, 2) + ",";
    linea += String(luz_val) + ",Estado\n";
  }

  // Escribir o anexar y listar
  xSemaphoreTake(printMutex, portMAX_DELAY);
  writeOrAppendFile(SD, filePath, linea.c_str());
  root = SD.open(dirName);
  root.close();
  xSemaphoreGive(printMutex);

  // Enviar resumen Telegram
  String msg = "📊 Nuevos datos:\n";
  msg += "Temperatura: " + String(temperatura_val, 2) + " °C\n";
  msg += "Humedad:      " + String(humedad_val, 2) + " %\n";
  msg += "Temp DS18B20: " + String(temperaturaS_val, 2) + " °C\n";
  msg += "Luz:          " + String(luz_val) + "\n";
  bot.sendMessage(CHAT_ID, msg, "");
  
  if (alertMsg.length() > 0) {
    String fullAlert = "🚨 Alarmas de umbral\n" + alertMsg;
    bot.sendMessage(CHAT_ID, fullAlert, "Markdown");
  }

  // Configurar actuadores y enviar según horarios
  loadConfigActu();
  if (getLocalTime(&timeinfo)) {
    int h = timeinfo.tm_hour, m = timeinfo.tm_min, d = timeinfo.tm_mday;
    unsigned long now = millis();
    // Motor A
    if (isTimeInRange(h, m, mA_sh, mA_sm, mA_eh, mA_em)
        && lastSentA != d && (!motorAConfirmed)
        && now - lastTryA >= RETRY_INTERVAL) {
      sendMessage(true, false, false, d);
      lastTryA = now;
      lastSentA = d;
    }
    // Motor B
    if (isTimeInRange(h, m, mB_sh, mB_sm, mB_eh, mB_em)
        && lastSentB != d && (!motorBConfirmed)
        && now - lastTryB >= RETRY_INTERVAL) {
      sendMessage(false, true, false, d);
      lastTryB = now;
      lastSentB = d;
    }
    // LED
    if (isTimeInRange(h, m, l_sh, l_sm, l_eh, l_em)
        && lastSentLED != d && (!ledConfirmed)
        && now - lastTryLED >= RETRY_INTERVAL) {
      sendMessage(false, false, true, d);
      lastTryLED = now;
      lastSentLED = d;
    }
  }
  entrarDeepSleep();
}

/**
 * @brief Configuración inicial.
 */
void setup() {
  Serial.begin(115200);
  printMutex = xSemaphoreCreateMutex();
  mutexHora = xSemaphoreCreateMutex();

  // FS inicial
  LittleFS.begin();

  // Leer config sensores/SD
  cargarConfiguracion();

  // ESP-NOW init para recibir sensores
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    while (true) {}
  }
  
  esp_now_register_recv_cb(onGeneralRecv);

  // ESP-NOW init para actuadores (callbacks comunes)
  esp_now_register_send_cb(onSent);

  // Tareas
  xTaskCreate(tareaMostrarDatos, "MostrarDatos", 2048, NULL, 3, NULL);
  xTaskCreate(tareaWiFi, "WiFi", 8192, NULL, 3, NULL);
  xTaskCreate(tareaTiempoNTP, "TiempoNTP", 2048, NULL, 2, NULL);
  xTaskCreate(tareaSD, "SD", 2048, NULL, 1, NULL);
  xTaskCreate(tareaCrearArchivo, "CrearFile", 8192, NULL, 1, NULL);
}

/**
 * @brief Bucle principal (no usado, vacío).
 */
void loop() {
  // Vacío
}

/**
 * @brief Convierte cadena "HH:MM" a horas y minutos.
 * @param str Cadena de entrada formateada "HH:MM".
 * @param h  Variable de salida para hora.
 * @param m  Variable de salida para minuto.
 */
void parseHourMinute(const char *str, int &h, int &m) {
  sscanf(str, "%d:%d", &h, &m);
}

/**
 * @brief Verifica si la hora actual está dentro de un rango dado.
 * @param h    Hora actual.
 * @param mi   Minuto actual.
 * @param sh   Hora inicio.
 * @param sm   Minuto inicio.
 * @param eh   Hora fin.
 * @param em   Minuto fin.
 * @return true si dentro del rango, false en caso contrario.
 */
 bool isTimeInRange(int h, int mi, int sh, int sm, int eh, int em) {
  int nowM = h * 60 + mi;
  int startM = sh * 60 + sm;
  int endM = eh * 60 + em;
  return (nowM >= startM && nowM <= endM);
}

/**
 * @brief Carga configuración de actuadores desde JSON en LittleFS.
 * @return true si carga exitosa, false en error.
 */
bool loadConfigActu() {
  File f = LittleFS.open(ARCH_CONFIG, "r");
  if (!f) return false;
  StaticJsonDocument<512> doc;
  auto err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  // MAC
  const char *macStr = doc["actuadores"]["direccion_mac"];
  int vals[6];
  if (sscanf(macStr, "%x:%x:%x:%x:%x:%x",
             &vals[0], &vals[1], &vals[2],
             &vals[3], &vals[4], &vals[5])
      != 6) return false;
  for (int i = 0; i < 6; i++) actuadorAddr[i] = (uint8_t)vals[i];

  // Horarios
  parseHourMinute(doc["actuadores"]["limites_hora"]["motorA"]["start"],
                  mA_sh, mA_sm);
  parseHourMinute(doc["actuadores"]["limites_hora"]["motorA"]["end"],
                  mA_eh, mA_em);
  parseHourMinute(doc["actuadores"]["limites_hora"]["motorB"]["start"],
                  mB_sh, mB_sm);
  parseHourMinute(doc["actuadores"]["limites_hora"]["motorB"]["end"],
                  mB_eh, mB_em);
  parseHourMinute(doc["actuadores"]["limites_hora"]["led"]["start"],
                  l_sh, l_sm);
  parseHourMinute(doc["actuadores"]["limites_hora"]["led"]["end"],
                  l_eh, l_em);

  // Registrar peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, actuadorAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  return true;
}

/**
 * @brief Envía mensaje de control a actuadores via ESP-NOW.
 * @param a   Activar motor A.
 * @param b   Activar motor B.
 * @param led Activar LED.
 * @param day Día actual para evitar repetición.
 */
void sendMessage(bool a, bool b, bool led, int day) {
  struct_message msg = { a, b, led, day };
  esp_now_send(actuadorAddr, (uint8_t *)&msg, sizeof(msg));
}
/**
 * @brief Callback de confirmación de envío ESP-NOW.
 * @param mac    MAC destino.
 * @param status Estado del envío.
 */
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.printf("Envio a %02X:%02X…: %s\n",
                mac[0], mac[1], mac[2],
                (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief Callback al recibir confirmación desde actuadores.
 *        Actualiza flags de confirmación.
 */
void onRecvConf(const esp_now_recv_info_t *info,
                const uint8_t *data, int len) {
  struct_message cm;
  memcpy(&cm, data, sizeof(cm));
  if (cm.activateMotorA) {
    motorAConfirmed = true;
    Serial.println("Motor A confirmado");
  }
  if (cm.activateMotorB) {
    motorBConfirmed = true;
    Serial.println("Motor B confirmado");
  }
  if (cm.activateLED) {
    ledConfirmed = true;
    Serial.println("LED confirmado");
  }
}

/**
 * @brief Callback general para recepción ESP-NOW.
 *        Distingue entre datos sensores y confirmaciones.
 */
void onGeneralRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  String macStr = macToString(info->src_addr);

  if (len == sizeof(struct_message)) {
    onRecvConf(info, data, len);
  } else {
    OnDataRecv(info, data, len);
  }
}

/**
 * @brief Carga configuración de sensores desde JSON en LittleFS.
 * @return true siempre (asume existencia y parseo correcto).
 */
bool cargarConfiguracion() {
  File file = LittleFS.open(ARCH_CONFIG, "r");

  DeserializationError err = deserializeJson(jsonConfig, file);
  file.close();

  return true;
}

/**
 * @brief Determina la estructura de datos según MAC y calcula tamEsperado.
 * @param macStr MAC del dispositivo sensores.
 * @return true si existe la configuración, false en error.
 */
bool cargarEstructuraPorMAC(const String &macStr) {
  if (!jsonConfig.containsKey("devices")) {
    return false;
  }
  JsonObject devices = jsonConfig["devices"];

  if (!devices.containsKey(macStr)) {
    return false;
  }

  jsonConfig["estructura"] = devices[macStr];

  tamEsperado = 0;
  JsonObject estructura = jsonConfig["estructura"];
  for (JsonPair kv : estructura) {
    // Ignorar la key "conditions" porque no es tipo de dato
    if (String(kv.key().c_str()) == "conditions") continue;

    String tipo = kv.value().as<String>();
    if (tipo == "float") tamEsperado += sizeof(float);
    else if (tipo == "int") tamEsperado += sizeof(int);
    else if (tipo == "uint8") tamEsperado += sizeof(uint8_t);
    else {
      return false;
    }
  }

  return true;
}

/**
 * @brief Interpreta datos binarios recibidos según estructura y genera alertas.
 */
void interpretarDatos(const uint8_t *datos, int len, const String &macStr) {
  if (len != tamEsperado) {
    return;
  }

  size_t offset = 0;
  JsonObject estructura = jsonConfig["estructura"];
  JsonObject devices = jsonConfig["devices"];
  JsonObject device = devices[macStr];
  JsonObject conditions = device["conditions"] | JsonObject();  // Puede no existir

  alertMsg = "";

  if (xSemaphoreTake(printMutex, portMAX_DELAY)) {
    for (JsonPair kv : estructura) {
      String campo = kv.key().c_str();
      if (campo == "conditions") continue;  // Saltar

      String tipo = kv.value().as<String>();

      if (tipo == "float") {
        float valor;
        memcpy(&valor, datos + offset, sizeof(float));

        offset += sizeof(float);
        if (campo == "temperatura") {
          temperatura_val = valor;
        } else if (campo == "humedad") {
          humedad_val = valor;
        } else if (campo == "temperaturaS") {
          temperaturaS_val = valor;
        }

        if (!conditions.isNull() && conditions.containsKey(campo)) {
          float minVal = conditions[campo]["min"] | -FLT_MAX;
          float maxVal = conditions[campo]["max"] | FLT_MAX;
          if (valor < minVal || valor > maxVal) {
            alertMsg += "⚠️ " + campo + " fuera de rango: " + String(valor, 2) + " (min " + String(minVal, 2) + " / max " + String(maxVal, 2) + ")\n";
          }
        }
      } else if (tipo == "int") {
        int valor;
        memcpy(&valor, datos + offset, sizeof(int));
        offset += sizeof(int);
        if (campo == "luz") {
          luz_val = valor;
        }
        if (!conditions.isNull() && conditions.containsKey(campo)) {
          int minVal = conditions[campo]["min"] | INT_MIN;
          int maxVal = conditions[campo]["max"] | INT_MAX;
          if (valor < minVal || valor > maxVal) {
            alertMsg += "⚠️ " + campo + " fuera de rango: " + String(valor) + " (min " + String(minVal) + " / max " + String(maxVal) + ")\n";
          }
        }
      } else if (tipo == "uint8") {
        uint8_t valor;
        memcpy(&valor, datos + offset, sizeof(uint8_t));
        offset += sizeof(uint8_t);

        if (!conditions.isNull() && conditions.containsKey(campo)) {
          uint8_t minVal = conditions[campo]["min"] | 0;
          uint8_t maxVal = conditions[campo]["max"] | 255;
        }
      }
    }
    xSemaphoreGive(printMutex);
  }
}

/**
 * @brief Procesa datos recibidos y llama a interpretarDatos.
 */
void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *incomingData, int len) {
  String macStr = macToString(info->src_addr);
  if (!cargarEstructuraPorMAC(macStr)) {
    return;
  }
  memcpy(bufferDatos, incomingData, len);
  interpretarDatos(bufferDatos, len, macStr);
}

/**
 * @brief Crea directorio.
 */
void createDir(fs::FS &fs, const char *path) {
  fs.mkdir(path);
}

/**
 * @brief Escribe archivo nuevo.
 */
void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    return;
  }
  file.close();
}

/**
 * @brief Anexa texto a archivo existente.
 */
void appendFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    return;
  }
  file.close();
}

/**
 * @brief Asegura creación de directorio si no existe.
 */
void ensureDirCreated(fs::FS &fs, const char *path) {
  if (!fs.exists(path)) {
    createDir(fs, path);
  }
}

/**
 * @brief Escribe o anexa mensaje al archivo.
 */
void writeOrAppendFile(fs::FS &fs, const char *path, const char *message) {
  if (!fs.exists(path)) {
    // Si el archivo no existe, lo creamos con writeFile
    writeFile(fs, path, message);
  } else {
    // Si ya existe, solo agregamos la línea
    appendFile(fs, path, message);
  }
}

/**
 * @brief Convierte MAC a String formateado.
 * @param mac Puntero a 6 bytes de MAC.
 * @return String representando la MAC ("AA:BB:CC:DD:EE:FF").
 */
String macToString(const uint8_t *mac) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

/**
 * @brief Entra en modo Deep Sleep tras flush serie.
 */
void entrarDeepSleep() {
  if (xSemaphoreTake(printMutex, portMAX_DELAY)) {
    Serial.flush();
    xSemaphoreGive(printMutex);
  }
  esp_sleep_enable_timer_wakeup(36 * 1000000);
  esp_deep_sleep_start();
}

