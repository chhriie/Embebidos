#include "StateMachineLib.h"
#include "DHT.h"
#include <AsyncTaskLib.h>

#define LED_BLUE 27     // Pin para el LED azul del LED RGB
#define DHTPIN 4        // Pin del sensor DHT11
#define LDRPIN 34       // Pin analógico para el sensor de luz (LDR)
#define LED_RED 25

// Selección del tipo de sensor DHT
#define DHTTYPE DHT11  

// Inicializar DHT11
DHT dht(DHTPIN, DHTTYPE);

float ValueTemperatura;
float ValueHumedad;
int ValueLuz;
unsigned long previousMillis = 0;
bool ledState = false;

void readtempfunct (void);
void readhumedfunct (void);
void readlightfunct (void);
void toggleLED (void);
void DisplayData (void);
void Timeout (void);
void Func_AMB_Init (void);
void Func_AMB_Fin (void);
void Func_LUZ_Init (void);
void Func_LUZ_Fin (void);
void Func_ALARM_Init (void);
void Func_ALARM_Fin (void);

// Crear tareas asíncronas
AsyncTask readTempTask(2500, true, readtempfunct);    // Leer temperatura cada 2500ms
AsyncTask timeOutTask(5000, false, Timeout);
AsyncTask readHumedTask(2500, true, readhumedfunct);  // Leer humedad cada 3200ms
AsyncTask readLightTask(1500, true, readlightfunct);  // Leer luz cada 1600ms
AsyncTask displayDataTask(2000, true, DisplayData);   // Mostrar datos cada 2000ms
AsyncTask LEDTask(1100, true, toggleLED);     // Encender/apagar LED azul cada 1100ms (700ms ON, 400ms OFF)

// State Alias
enum State //enumera, tipo de datos definido por el usuario
{
	MONIT_AMB = 0,
	MONIT_LUZ = 1,
	ALARMA = 2,
};

// Input Alias
enum Input  //enumersción de entrada
{
	Sign_T = 0,
	Sign_L = 1,
	Sign_H = 2,
	Unknown = 3,
};

// Create new StateMachine
StateMachine stateMachine(3, 5); 

// Stores last user input
Input input;  

// Setup the State Machine
void setupStateMachine()
{ 
  //A: Ambiental B: Luz C: Alarma
	// Se crean todas las transisiones
	stateMachine.AddTransition(MONIT_AMB, MONIT_LUZ, []() { return input == Sign_T; }); 
	stateMachine.AddTransition(MONIT_LUZ, MONIT_AMB, []() { return input ==Sign_T; });
	stateMachine.AddTransition(MONIT_LUZ, ALARMA, []() { return input == Sign_L; });
	stateMachine.AddTransition(ALARMA, MONIT_AMB, []() { return input == Sign_T; });
  stateMachine.AddTransition(MONIT_AMB, ALARMA, []() { return input == Sign_H; });

	// Add actions
	stateMachine.SetOnEntering(MONIT_AMB, Func_AMB_Init);
	stateMachine.SetOnEntering(MONIT_LUZ, Func_LUZ_Init);
	stateMachine.SetOnEntering(ALARMA, Func_ALARM_Init);

	stateMachine.SetOnLeaving(MONIT_AMB, Func_AMB_Fin);
	stateMachine.SetOnLeaving(MONIT_LUZ, Func_LUZ_Fin);
	stateMachine.SetOnLeaving(ALARMA, Func_ALARM_Fin);
}

void Func_AMB_Init (void){
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  Serial.println("Func_AMB_Init");
  readTempTask.Start();
  readHumedTask.Start();
  timeOutTask.Start();
}

void Func_AMB_Fin (void){
  Serial.println("Func_AMB_Fin");
  readTempTask.Stop();
  readHumedTask.Stop();
  timeOutTask.Stop();
}

void Func_LUZ_Init (void){
  Serial.println("Func_LUZ_Init");
  readLightTask.Start();
  timeOutTask.SetIntervalMillis(3000);
  timeOutTask.Start();
}
void Func_LUZ_Fin (void){
  Serial.println("Func_LUZ_Fin");
  readLightTask.Stop();
  timeOutTask.Stop();
}

void Func_ALARM_Init (void){
  timeOutTask.Stop();
  Serial.println("Func_ALARM_Init");
  LEDTask.Start();
  timeOutTask.SetIntervalMillis(6000);
  timeOutTask.Start();
}

void Func_ALARM_Fin (void){
  Serial.println("Func_ALARM_Fin");
  LEDTask.Stop();
  timeOutTask.Stop();
}

void setup() 
{
	Serial.begin(9600);
  //delay(1000);
	Serial.println("Starting State Machine...");
	setupStateMachine();
  dht.begin();
	Serial.println("Start Machine Started");
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

	// Initial state
	stateMachine.SetState(MONIT_AMB, false, true);
  displayDataTask.Start();
}

void loop() 
{
	// Read user input
	input = static_cast<Input>(readInput());
  readTempTask.Update();   
  timeOutTask.Update();
  readHumedTask.Update();  
  readLightTask.Update();  
  displayDataTask.Update();
  LEDTask.Update();     

	// Update State Machine
	stateMachine.Update();
  input = Unknown; 
}

// Auxiliar function that reads the user input
int readInput()
{
	Input currentInput = Input::Unknown;
	if (Serial.available())
	{
		char incomingChar = Serial.read();
    Serial.print("Recibido: "); Serial.println(incomingChar);

		switch (incomingChar)
		{
			case 'R': currentInput = Input::Sign_T; break;
			case 'A': currentInput = Input::Sign_L; break;
			case 'D': currentInput = Input::Sign_H; break;
			default: break;
		}
	}

	return currentInput;
}

// Leer temperatura
void readhumedfunct (void) {
  ValueHumedad = dht.readHumidity();
  if(ValueHumedad > 70){
    input = Sign_H;
    Serial.println("Cambio a estado Sign_H");
  }
}

void readtempfunct (void) {
  ValueTemperatura = dht.readTemperature();
  if((ValueTemperatura < 24 || ValueTemperatura > 35)){
    input = Sign_H;
    Serial.println("Cambio a estado Sign_H");
  }
}
// Leer luz (sensor LDR)
void readlightfunct (void) {
  ValueLuz = analogRead(LDRPIN);
  if ((ValueLuz>500)){
    input = Sign_L;
    Serial.println("Cambio a estado Sign_L");
  }
}

void toggleLED (void) {
  unsigned long currentMillis = millis();
   // Apagar todos los LEDs antes de encender el correcto
   digitalWrite(LED_BLUE, LOW);
   digitalWrite(LED_RED, LOW);

   if (ValueLuz > 500) {
     if (ledState && (currentMillis - previousMillis >= 700)) {
        digitalWrite(LED_BLUE, LOW);  // Apagar LED después de 700ms encendido
        ledState = false;
        previousMillis = currentMillis;
    } 
    else if (!ledState && (currentMillis - previousMillis >= 400)) {
        digitalWrite(LED_BLUE, HIGH); // Encender LED después de 400ms apagado
        ledState = true;
        previousMillis = currentMillis;
    }
   } 
    else if((ValueTemperatura < 24 || ValueTemperatura > 35) || (ValueHumedad > 70)){
     if (ledState && (currentMillis - previousMillis >= 700)) {
        digitalWrite(LED_RED, LOW);  // Apagar LED después de 700ms encendido
        ledState = false;
        previousMillis = currentMillis;
    } 
    else if (!ledState && (currentMillis - previousMillis >= 400)) {
        digitalWrite(LED_RED, HIGH); // Encender LED después de 400ms apagado
        ledState = true;
        previousMillis = currentMillis;
    }
   } 
}

void DisplayData (void) {
  Serial.print(F("Humedad: "));
  Serial.print(ValueHumedad);
  Serial.print(F("% \t Temperatura: "));
  Serial.print(ValueTemperatura);
  Serial.print(F(" *C \t Luz: "));
  Serial.println(ValueLuz);
}

void Timeout (void){
  input = Sign_T;
}
