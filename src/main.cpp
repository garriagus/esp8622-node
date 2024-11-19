#include <MD_Parola.h>
#include <MD_MAX72XX.h>
#include <SPI.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include "config.h"

// Definición del hardware para la matriz LED
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW // Tipo de hardware
#define MAX_DEVICES 4                     // Número de módulos 8x8 (8x32 tiene 4 módulos)
#define DATA_PIN D7                       // Pin de datos
#define CS_PIN D8                         // Pin Chip Select
#define CLK_PIN D5                        // Pin de reloj

// Definición del DHT22
#define DHT_PIN D6          // Pin conectado al DHT22
#define DHT_TYPE DHT22      // Tipo de sensor
DHT dht(DHT_PIN, DHT_TYPE); // Inicializa el DHT22

// Definición del Joystick
#define JOYSTICK_X_PIN A0       // Conectar el eje X al pin A0
#define JOYSTICK_THRESHOLD 100  // Umbral de cambio significativo para el eje X
#define JOYSTICK_BUTTON_PIN D2  // Pin del botón del joystick

int joystickXValue = 0;          // Valor actual del eje X del joystick
int lastJoystickXValue = 0;      // Último valor registrado del eje X
bool lastButtonState = HIGH;     // Último estado del botón (HIGH = no presionado)

// Instancia de MD_Parola
MD_Parola matrix = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Variables para temporizador
unsigned long lastUpdate = 0;         // Tiempo de la última actualización
const unsigned long updateInterval = 2000; // Intervalo de 2 segundos (en ms)

void connectToWiFi()
{
  matrix.displayText("Conectando...", PA_CENTER, 100, 2000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  matrix.displayReset();

  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConexión establecida");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // Mostrar mensaje de éxito en la matriz
  matrix.displayText("WiFi Conectado", PA_CENTER, 100, 2000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  matrix.displayReset();
}

void setup()
{
  // Inicialización de la matriz, sensor, Wi-Fi y joystick
  matrix.begin();         // Inicializa la matriz
  Serial.begin(9600);     // Inicializa el puerto serial
  matrix.setIntensity(5); // Ajusta el brillo (0 a 15)
  dht.begin();            // Inicializa el sensor DHT22

  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP); // Configura el botón como entrada con resistencia pull-up

  // Conexión Wi-Fi
  connectToWiFi();
}

void loop()
{
  // Leer el valor del eje X del joystick (rango de 0 a 1023)
  joystickXValue = analogRead(JOYSTICK_X_PIN);

  // Detectar cambios significativos en el eje X
  if (abs(joystickXValue - lastJoystickXValue) > JOYSTICK_THRESHOLD)
  {
    Serial.printf("Movimiento detectado en el eje X: %d\n", joystickXValue);
    lastJoystickXValue = joystickXValue; // Actualizar el último valor
  }

  // Leer el estado del botón del joystick
  bool buttonState = digitalRead(JOYSTICK_BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH)
  {
    Serial.println("Botón del joystick presionado.");
  }
  lastButtonState = buttonState;

  // Animación de la matriz LED
  if (matrix.displayAnimate())
  {
    unsigned long currentMillis = millis();

    // Verificar si ha pasado el intervalo de actualización
    if (currentMillis - lastUpdate >= updateInterval)
    {
      static bool showTemp = true; // Alterna entre temperatura y humedad
      lastUpdate = currentMillis; // Actualizar el temporizador

      // Leer datos del sensor
      float temperature = dht.readTemperature(); // Temperatura en °C
      float humidity = dht.readHumidity();       // Humedad relativa

      // Verifica errores en la lectura
      if (isnan(temperature) || isnan(humidity))
      {
        matrix.displayText("Error sensor", PA_CENTER, 100, 3000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
        Serial.println("Error al leer el sensor DHT22.");
      }
      else
      {
        char displayMessage[32]; // Buffer para los mensajes
        if (showTemp)
        {
          snprintf(displayMessage, sizeof(displayMessage), "Temp: %.1f C", temperature);
          Serial.printf("Temperatura: %.1f°C\n", temperature);
        }
        else
        {
          snprintf(displayMessage, sizeof(displayMessage), "Humedad: %.1f%%", humidity);
          Serial.printf("Humedad: %.1f%%\n", humidity);
        }

        // Mostrar el mensaje en la matriz LED
        matrix.displayText(displayMessage, PA_CENTER, 100, 2000, PA_SCROLL_LEFT, PA_SCROLL_LEFT);

        // Alterna entre temperatura y humedad
        showTemp = !showTemp;
      }

      // Reinicia la animación
      matrix.displayReset();
    }
  }
}
