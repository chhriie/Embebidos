/**
 * @file actuadores.ino
 * @brief Control de actuadores (motores y LED) mediante ESP-NOW en un ESP32 usando FreeRTOS.
 * 
 * Este programa recibe comandos mediante ESP-NOW y ejecuta tareas
 * para activar motores y un LED, evitando activaciones múltiples el mismo día.
 * 
 * @author Briyith Guacas y Karol Palechor
 * @date 2025-06-11
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>  // Librería para FreeRTOS

// Pines de los motores y LED
/** @brief Pines del ventilador */
int motor1Pin1 = 27;
int motor1Pin2 = 26;
int enable1Pin = 14;

/** @brief Pines de la bomba de agua */
int motor2Pin1 = 33;
int motor2Pin2 = 32;
int enable2Pin = 15;

/** @brief Pin del LED */
#define LED_GPIO 23

/** @brief Ciclo de trabajo para el control de velocidad PWM */
int dutyCycle = 200;

/**
 * @brief Estructura del mensaje recibido o enviado.
 */
typedef struct struct_message {
  bool activateMotorA; ///< Indica si se debe activar el motor A
  bool activateMotorB; ///< Indica si se debe activar el motor B
  bool activateLED;    ///< Indica si se debe activar el LED
  int daySent;         ///< Día en que se envió el mensaje
} struct_message;

struct_message incomingMessage;
struct_message confirmationMessage;

/** @brief Dirección MAC del transmisor */
uint8_t transmitterMac[6];

// Variables para evitar múltiples activaciones por día (almacenadas en RTC)
RTC_DATA_ATTR int lastDayMotorA = -1;
RTC_DATA_ATTR int lastDayMotorB = -1;
RTC_DATA_ATTR int lastDayLED = -1;
RTC_DATA_ATTR TickType_t lastAttemptMotorA = 0;
RTC_DATA_ATTR TickType_t lastAttemptMotorB = 0;
RTC_DATA_ATTR TickType_t lastAttemptLED = 0;

/** @brief Intervalo mínimo entre intentos (2 segundos) */
const TickType_t RETRY_INTERVAL_TICKS = pdMS_TO_TICKS(2 * 1000);

/**
 * @brief Tarea para mover el motor 1.
 */
void moveMotor1(void *pvParameters) {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  analogWrite(enable1Pin, dutyCycle);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(enable1Pin, 0);
  
  confirmationMessage = {true, false, false, lastDayMotorA};
  esp_now_send(transmitterMac, (uint8_t*)&confirmationMessage, sizeof(confirmationMessage));
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  esp_sleep_enable_timer_wakeup(5 * 1000000);
  esp_deep_sleep_start();
  vTaskDelete(NULL);
}

/**
 * @brief Tarea para mover el motor 2.
 */
void moveMotor2(void *pvParameters) {
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(enable2Pin, dutyCycle);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(enable2Pin, 0);
  
  confirmationMessage = {false, true, false, lastDayMotorB};
  esp_now_send(transmitterMac, (uint8_t*)&confirmationMessage, sizeof(confirmationMessage));
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  esp_sleep_enable_timer_wakeup(5 * 1000000);
  esp_deep_sleep_start();
  vTaskDelete(NULL);
}

/**
 * @brief Tarea para parpadear el LED.
 */
void blinkLED(void *pvParameters) {
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_GPIO, HIGH);
    Serial.println("Mensaje procesado.");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  confirmationMessage = {false, false, true, lastDayLED};
  esp_now_send(transmitterMac, (uint8_t*)&confirmationMessage, sizeof(confirmationMessage));
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  esp_sleep_enable_timer_wakeup(5 * 1000000);
  esp_deep_sleep_start();
  vTaskDelete(NULL);
}

/**
 * @brief Callback que se ejecuta al recibir datos por ESP-NOW.
 * 
 * @param info Información del emisor.
 * @param incomingData Datos recibidos.
 * @param len Longitud de los datos.
 */
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  memcpy(&incomingMessage, incomingData, sizeof(incomingMessage));
  memcpy(transmitterMac, info->des_addr, sizeof(transmitterMac));

  int today = incomingMessage.daySent;
  TickType_t now = xTaskGetTickCount();

  if (incomingMessage.activateMotorA && lastDayMotorA != today && (now - lastAttemptMotorA >= RETRY_INTERVAL_TICKS)) {
    xTaskCreate(moveMotor1, "MotorA", 2048, NULL, 1, NULL);
    lastDayMotorA = today;
    lastAttemptMotorA = now;
  }

  if (incomingMessage.activateMotorB && lastDayMotorB != today && (now - lastAttemptMotorB >= RETRY_INTERVAL_TICKS)) {
    xTaskCreate(moveMotor2, "MotorB", 2048, NULL, 1, NULL);
    lastDayMotorB = today;
    lastAttemptMotorB = now;
  }

  if (incomingMessage.activateLED && lastDayLED != today && (now - lastAttemptLED >= RETRY_INTERVAL_TICKS)) {
    xTaskCreate(blinkLED, "LED", 2048, NULL, 1, NULL);
    lastDayLED = today;
    lastAttemptLED = now;
  }
}

/**
 * @brief Función de inicialización del ESP32.
 */
void setup() {
  Serial.begin(115200);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  pinMode(LED_GPIO, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error iniciando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

/**
 * @brief Bucle principal (no se utiliza, las tareas hacen el trabajo).
 */
void loop() {
  // Todo se ejecuta en las tareas
}
