/**
 * @file sensores.ino
 * @brief Lectura de sensores y envío de datos mediante ESP-NOW.
 * 
 * Este programa lee datos de temperatura (DHT11 y DS18B20), humedad y luz,
 * y los envía a otro dispositivo usando ESP-NOW. Las lecturas se ejecutan en
 * tareas separadas usando FreeRTOS. La comunicación se sincroniza mediante un mutex.
 * 
 * @author Briyith Guacas y Karol Palechor
 * @date 2025-06-11
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"

#define ONE_WIRE_BUS 4    ///< Pin del bus OneWire
#define DHTPIN 5          ///< Pin del sensor DHT11
#define DHTTYPE DHT11     ///< Tipo de sensor DHT
#define PIN_LUZ 34        ///< Pin para lectura de luz

/// Dirección MAC del receptor
uint8_t receptorMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
DHT dht(DHTPIN, DHTTYPE

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
DHT dht(DHTPIN, DHTTYPE);

/**
 * @brief Datos de sensores con protección por mutex.
 */
typedef struct {
  float temperaturaS;         ///< Temperatura del DS18B20
  float temperatura;          ///< Temperatura del DHT11
  float humedad;              ///< Humedad del DHT11
  int luz;                    ///< Nivel de luz
  SemaphoreHandle_t mutex;    ///< Mutex para acceso concurrente
} SensorData;

SensorData datos;

/**
 * @brief Datos que se envían por ESP-NOW.
 */
typedef struct {
  float temperaturaS;   ///< Temperatura del DS18B20
  float temperatura;    ///< Temperatura del DHT11
  float humedad;        ///< Humedad del DHT11
  int luz;              ///< Nivel de luz
} struct_message;

struct_message datosSensor;

/**
 * @brief Lee la temperatura desde el sensor DS18B20.
 * @return Temperatura válida o NAN si hay error.
 */
float leerTemperaturaDS18B20() {
  float t = NAN;
  for (int i = 0; i < 3; i++) {
    ds18b20.requestTemperatures();
    t = ds18b20.getTempCByIndex(0);
    if (!isnan(t) && t != 0.0) break;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  return (!isnan(t) && t != 0.0) ? t : NAN;
}

/**
 * @brief Lee la temperatura desde el sensor DHT11.
 * @return Temperatura o NAN si no es válida.
 */
float leerTemperaturaDHT() {
  float t = dht.readTemperature();
  return (!isnan(t)) ? t : NAN;
}

/**
 * @brief Lee la humedad desde el sensor DHT11.
 * @return Humedad o NAN si no es válida.
 */
float leerHumedadDHT() {
  float h = dht.readHumidity();
  return (!isnan(h)) ? h : NAN;
}

/**
 * @brief Tarea FreeRTOS para lectura del sensor DS18B20.
 */
void tarea_temperatura_ds18b20(void *pvParameters) {
  while (true) {
    float t = leerTemperaturaDS18B20();
    if (xSemaphoreTake(datos.mutex, portMAX_DELAY)) {
      datos.temperaturaS = t;
      xSemaphoreGive(datos.mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(9500));
  }
}

/**
 * @brief Tarea FreeRTOS para lectura del sensor DHT11.
 */
void tarea_dht(void *pvParameters) {
  while (true) {
    float t = leerTemperaturaDHT();
    float h = leerHumedadDHT();
    if (xSemaphoreTake(datos.mutex, portMAX_DELAY)) {
      datos.temperatura = t;
      datos.humedad = h;
      xSemaphoreGive(datos.mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(9500));
  }
}

/**
 * @brief Tarea FreeRTOS para lectura del nivel de luz.
 */
void tarea_luz(void *pvParameters) {
  while (true) {
    int valor = analogRead(PIN_LUZ);
    if (xSemaphoreTake(datos.mutex, portMAX_DELAY)) {
      datos.luz = valor;
      xSemaphoreGive(datos.mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(9500));
  }
}

/**
 * @brief Tarea que envía los datos por ESP-NOW y entra en modo deep sleep.
 */
void enviar_datos_task(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(datos.mutex, portMAX_DELAY)) {
      if (!isnan(datos.temperaturaS) &&
          !isnan(datos.temperatura) &&
          !isnan(datos.humedad)) {
        datosSensor.temperaturaS = datos.temperaturaS;
        datosSensor.temperatura = datos.temperatura;
        datosSensor.humedad = datos.humedad;
        datosSensor.luz = datos.luz;

        esp_now_send(receptorMAC, (uint8_t *)&datosSensor, sizeof(datosSensor));
      }
      xSemaphoreGive(datos.mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(5000));  
    esp_sleep_enable_timer_wakeup(3 * 1000000ULL); 
    esp_deep_sleep_start();

    vTaskDelete(NULL);
  }
}

/**
 * @brief Inicializa sensores, WiFi, ESP-NOW y tareas.
 */
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));

  ds18b20.begin();
  dht.begin();
  pinMode(PIN_LUZ, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_now_init();

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receptorMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(receptorMAC)) {
    esp_now_add_peer(&peerInfo);
  }

  datos.mutex = xSemaphoreCreateMutex();

  xTaskCreate(tarea_temperatura_ds18b20, "TempDS18B20", 2048, NULL, 1, NULL);
  xTaskCreate(tarea_dht, "DHT11", 2048, NULL, 1, NULL);
  xTaskCreate(tarea_luz, "Luz", 2048, NULL, 1, NULL);
  xTaskCreate(enviar_datos_task, "Enviar", 4096, NULL, 1, NULL);
}

/**
 * @brief Loop principal.
 */
void loop() {
  // Nada aquí
}