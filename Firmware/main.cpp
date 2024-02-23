#include <Arduino.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <AsyncElegantOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>
#include <ESPAsyncWiFiManager.h>   
#include <Preferences.h>



#include "dimmer.hpp"
#include "secretos.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#define serial_activado
#define usar_led
#ifdef usar_led
const int led = 3; // Pin GPIO8
#endif

#define TEMPERATURA_VENTILADOR  40.0
#define HISTERESIS               3.0
#define TEMPERATURA_MAXIMA      65.0

// Data wire is plugged into port 2 on the Arduino
const int oneWireBus = 6;
const int ventilador = 7;

Preferences prefs;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void mqtt_recibido(char* topic, byte* payload, unsigned int length);

// Configuración Wifi y MQTT
#ifdef WiFi_activado  
IPAddress servidor_mqtt(192, 168, 0, 192);

DNSServer dns;
AsyncWebServer server(80);
WiFiClient espClient;
PubSubClient client(servidor_mqtt, 1883, mqtt_recibido, espClient);
#endif

// Variables globales
uint32_t tiempo1=0, tiempo2=0;                  // para sincronizar envíos de mensajes en el bucle principal

dimmer Gestor_excedentes;

float tempC;
float parametros[3];

#ifdef WiFi_activado  

void reconectar_mqtt() {
  // Bucle hasta conseguir conexión MQTT
  while (!client.connected()) {
    #ifdef serial_activado
    Serial.print("Conectando MQTT...");
    #endif
    // Intento de conexión
    if (client.connect("ESP32-Client",mqtt_usuario, mqtt_contrase)) {
      #ifdef serial_activado
      Serial.println("MQTT Conectado");
      #endif
      // Once connected, publish an announcement...
      client.publish(topic_estado,"Arranca Gestor de Excedentes");
      // ... and resubscribe
      client.subscribe(topic_objetivo);
      client.subscribe(topic_config);
    } else {
      #ifdef serial_activado
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" nuevo intento en 5 segundos");
      #endif
      delay(5000);
    }
  }
}

// Procesamiento de mensaje recibido por MQTT
void mqtt_recibido(char* topic, byte* payload, unsigned int length) {
  uint16_t objetivo_mqtt = 0;

  #ifdef serial_activado
  Serial.print("Recibido [");
  Serial.print(topic);
  Serial.print("] ");
  #endif

  if (String(topic) == topic_objetivo)  {

    // extraemos dígito a dígito la cadena recibida y lo almacenamos en una variable numérica
    for (int i=0;i<length;i++) {
      #ifdef serial_activado
      Serial.print((char)payload[i]);
      #endif
      objetivo_mqtt = objetivo_mqtt*10 + payload[i]-'0';
    }
    #ifdef serial_activado
    Serial.println();
    Serial.print (objetivo_mqtt);
    #endif

    // información de estado: objetivo recibido
    client.publish(topic_estado,"Objetivo recibido");

    // Si recibimos el objetivo mínimo o máximo desactivamos el triac dejándolo en ON o en OFF
    // Cambiamos el estado del dimmer a uno de los dos estados estáticos Apagado o Encendido
    if (tempC>TEMPERATURA_MAXIMA) {
      objetivo_mqtt = 0;
      client.publish(topic_estado,"Alerta temperatura");
    }

    Gestor_excedentes.nuevo_objetivo(objetivo_mqtt);
  };

  if (String(topic) == topic_config)  {    
    int i=0,j=0,k=0;
    char param[10];
    
    char c,configuracion[20];
    Serial.printf("%s",payload);
    Serial.println();
    for (i=0;i<=length;i++) {
      c = (char)payload[i];
      if (c==',' || i==length) {
        param[k]=0;
        parametros[j] = atof(param);
        k=0;
        param[k]=0;
        j++;
      } else {
        param[k] = payload[i];
        k++;
      }
    };
    snprintf (configuracion, sizeof(configuracion), "%5.2f,%5.2f,%5.2f", parametros[0], parametros[1], parametros[2]);
    client.publish(topic_estado,configuracion);

    prefs.putFloat("param0", parametros[0]);
    prefs.putFloat("param1", parametros[1]);
    prefs.putFloat("param2", parametros[2]);
    
    Gestor_excedentes.configuracion(parametros[0],parametros[1],parametros[2]);
    
  }
}
#endif

// Configuración
void setup() {
  uint8_t i;
 
  #ifdef WiFi_activado

  AsyncWiFiManager wifiManager(&server,&dns);
  // wifiManager.resetSettings(); // descomentar cuando quiera probar el portal captive

  // Conexión con IP fija
  wifiManager.setSTAStaticIPConfig(IPAddress(192,168,0,198), IPAddress(192,168,0,1), IPAddress(255,255,255,0)); 
  wifiManager.autoConnect("Gestor Excedentes MicroPIC");

  #ifdef usar_led
  // 5 parpadeos del led
  pinMode(led, OUTPUT);
  pinMode(ventilador, OUTPUT);
  for (i=0;i<5;i++) {
    digitalWrite(led,1);
    delay(100);
    digitalWrite(led,0);
    delay(100);
  }
  #endif

  prefs.begin("dimmer", false);
  parametros[0]=prefs.getFloat("param0");
  parametros[1]=prefs.getFloat("param1");
  parametros[2]=prefs.getFloat("param2");

  // Texto que se muestra al entrar a la IP del dispositivo con un navegador web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Gestor de Excedentes - MicroPIC 2024");
  });
  #ifdef serial_activado
  Serial.println("Servidor HTTP arrancado");
  #endif 
  #endif

  #ifdef WiFi_activado
  // Para poder programar el dispositivo vía OTA
  
  AsyncElegantOTA.begin(&server);    
  server.begin();
  #endif

  Gestor_excedentes.begin(1800,0,1,2);
  Gestor_excedentes.configuracion(parametros[0],parametros[1],parametros[2]);

  sensors.begin();
  // Almacenamos la hora actual
  tiempo1 = millis();
}

void loop() {
  uint16_t valorIngresado;    
  uint16_t potencia;
  char potencia_media[10];
  char temperatura[10];

  Gestor_excedentes.loop();

#ifdef WiFi_activado  
  if (!client.connected()) {                    // Si no hubiera conexión MQTT intentamos reconectar
    reconectar_mqtt();
  }
  client.loop();
#endif

  tiempo2 = millis();
  if (tiempo2 > tiempo1 + 1000) {               // una vez cada segundo enviamos la potencia consumida por MQTT
    tiempo1 = millis();
    snprintf (potencia_media, sizeof(potencia_media), "%u", Gestor_excedentes.pot_consumida());
    client.publish(topic_potencia,potencia_media);
    sensors.requestTemperatures(); // Send the command to get temperatures
    tempC = sensors.getTempCByIndex(0);
    // Si la lectura de temperatura es correcta
    if (tempC != DEVICE_DISCONNECTED_C) {
      snprintf (temperatura, sizeof(temperatura), "%5.2f", tempC);
      client.publish(topic_temperatura,temperatura);
    }
    if ((tempC < TEMPERATURA_VENTILADOR - HISTERESIS) && (digitalRead (ventilador)==1)) {
      digitalWrite(ventilador,0);
      client.publish(topic_ventilador,"0");
    }
        
    if ((tempC > TEMPERATURA_VENTILADOR) && (digitalRead (ventilador)==0)) {
      digitalWrite(ventilador,1);
      client.publish(topic_ventilador,"1");
    }
    
    if (tempC > TEMPERATURA_MAXIMA)
      mqtt_recibido((char *)topic_objetivo, (byte *)"0", 1);
  }
}