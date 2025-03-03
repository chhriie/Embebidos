#include "StateMachineLib.h"
#include "DHT.h"
#include <AsyncTaskLib.h>

#define LED_BLUE 27     // Pin para el LED azul del LED RGB
#define DHTPIN 4        // Pin del sensor DHT11
#define LDRPIN 34       // Pin analógico para el sensor de luz (LDR)
#define LED_RED 25
#define LED_GREEN 26

// Selección del tipo de sensor DHT
#define DHTTYPE DHT11  

// Inicializar DHT11
DHT dht(DHTPIN, DHTTYPE);

// Variables para almacenar los valores
float ValueTemperatura;
float ValueHumedad;
int ValueLuz;

void readtemphumfunct (void);
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
AsyncTask readTempTask(2500, true, readtemphumfunct);    // Leer temperatura cada 2500ms
AsyncTask timeOutTask(5000, false, Timeout);
//AsyncTask readHumedTask(3200, true, readhumedfunct);  // Leer humedad cada 3200ms
AsyncTask readLightTask(1500, true, readlightfunct);  // Leer luz cada 1600ms
AsyncTask displayDataTask(2000, true, DisplayData);   // Mostrar datos cada 2000ms
AsyncTask LEDTask(1100, true, toggleLED);     // Encender/apagar LED cada 1100ms (700ms ON, 400ms OFF)


// State Alias
enum State //enumera, tipo de datos definido por el usuario
{
	PosicionA = 0,
	PosicionB = 1,
	PosicionC = 2,
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
StateMachine stateMachine(3, 5); //instancia, recibe instancias (numero de estados, numero de transisiones)

// Stores last user input
Input input;  

// Setup the State Machine
void setupStateMachine()
{ 
  //A: Ambiental B: Luz C: Alarma
	// Se crean todas las transiciones
	stateMachine.AddTransition(PosicionA, PosicionB, []() { return input == Sign_T; });  //visionar transición
	stateMachine.AddTransition(PosicionB, PosicionA, []() { return input ==Sign_T; });
	stateMachine.AddTransition(PosicionB, PosicionC, []() { return input == Sign_L; });
	stateMachine.AddTransition(PosicionC, PosicionA, []() { return input == Sign_T; });
  stateMachine.AddTransition(PosicionA, PosicionC, []() { return input == Sign_H; });



	// Add actions
	stateMachine.SetOnEntering(PosicionA, Func_AMB_Init);
	stateMachine.SetOnEntering(PosicionB, Func_LUZ_Init);
	stateMachine.SetOnEntering(PosicionC, Func_ALARM_Init);

	stateMachine.SetOnLeaving(PosicionA, Func_AMB_Fin);
	stateMachine.SetOnLeaving(PosicionB, Func_LUZ_Fin);
	stateMachine.SetOnLeaving(PosicionC, Func_ALARM_Fin);
}

void Func_AMB_Init (void){
  timeOutTask.Stop();
  Serial.println("Func_AMB_Init");
  readTempTask.Start();
  timeOutTask.Start();
}

void Func_AMB_Fin (void){
  Serial.println("Func_AMB_Fin");
  readTempTask.Stop();
  timeOutTask.Stop();
}

void Func_LUZ_Init (void){
  timeOutTask.Stop();
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
	Serial.println("Start Machine Started");
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

	// Initial state
  // Quien se activa
	stateMachine.SetState(PosicionA, false, true);
  displayDataTask.Start();
}

void loop() 
{
	// Read user input
	input = static_cast<Input>(readInput());

	// Update State Machine
	stateMachine.Update();
  displayDataTask.Update();
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
      //case 'P': currentInput = Input::Sign__T; break;
			default: break;
		}
	}

	return currentInput;
}



// Leer temperatura
void readtemphumfunct (void) {
  ValueTemperatura = dht.readTemperature();
  ValueHumedad = dht.readHumidity();

  if((ValueTemperatura>24) && (ValueHumedad>70)){
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
   // Apagar todos los LEDs antes de encender el correcto
   digitalWrite(LED_BLUE, LOW);
   digitalWrite(LED_RED, LOW);
   digitalWrite(LED_GREEN, LOW);

   if (ValueLuz > 500) {
     digitalWrite(LED_BLUE, HIGH);  // Luz mayor a 500 → LED azul
   } 
   else if (ValueTemperatura > 24) {
     digitalWrite(LED_GREEN, HIGH); // Temp > 24 → LED verde
   } 
   else if (ValueHumedad > 70) {
     digitalWrite(LED_RED, HIGH);   // Humedad > 70 → LED rojo
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

// Auxiliar output functions that show the state debug
void outputA()
{
	Serial.println("A   B   C");
	Serial.println("X        ");
	Serial.println();
}

void outputB()
{
	Serial.println("A   B   C");
	Serial.println("    X    ");
	Serial.println();
}

void outputC()
{
	Serial.println("A   B   C");
	Serial.println("        X");
	Serial.println();
}


