// Librerías necesarias para el código
#include <SD.h>
#include <SPI.h>
#include <WiFiEsp.h>
#include <ThingSpeak.h>
#include "secrets.h"

// Configuración de pines y variables
const int potPin1 = A0; // Pin analógico A0
const int potPin2 = A1; // Pin analógico A1
const int chipSelect = BUILTIN_SDCARD; // Selección del pin del adaptador microSD integrado en el Teensy 4.1

unsigned long startTime = 0; // Tiempo de inicio de la medición de datos
//const int measureDuration = 30000;  // Duración de 30 segundos
//const int measureInterval = 25;  // 40Hz -> 25ms
unsigned long lastMeasureTime = 0;
unsigned long currentTime = 0;

unsigned long lastConnectionTime = 0;               // Track the last connection time
unsigned long previousUpdate = 0;                   // Track the last update time
const unsigned long postingInterval = 20L * 1000L;  // Recording data time
const unsigned long updateInterval = 0.1L * 1000L;  // Update JSON once every updateInterval ms


float m1 = -0.275, b1 = 134.58; // Ecuación y = mx + b para potenciómetro 1
float m2 = -0.263, b2 = 128.77; // Ecuación y = mx + b para potenciómetro 2
float angle1Max = 0; // Inicializamos con el menor valor posible
float angle1Min = 0;  // Inicializamos con el mayor valor posible
float angle2Max = 0;
float angle2Min = 0;
float angle2MinLimit = -10.0;  // Límite inferior de angle2 para contar una repetición
float angle2MaxLimit = 10.0;   // Límite superior de angle2 para contar una repetición
bool isMovingUp = true;        // Indica si se está moviendo hacia el extremo superior
int repetitionCount = 0;       // Contador de repeticiones
bool reachedMin = false;       // Indica si alcanzó el mínimo
bool reachedMax = false;       // Indica si alcanzó el máximo

long ejercicio = 0;
long resistencia = 0;
float fuerza = 0;
float torque = 0;

#define ESP_BAUDRATE  115200 // Baudrate del ESP01
#define DEG_TO_RAD 0.017453292519943295769236907684886
// Variables para WiFi y Thingspeak
char ssid[] = SECRET_SSID;    // Nombre de la red WiFi proveniente de la librería secrets.h
char pass[] = SECRET_PASS;    // Contraseña de la red WiFi proveniente de la librería secrets.h
char server[] = "api.thingspeak.com"; // ThingSpeak Server
WiFiEspClient client; //Inicio de cliente WiFi
unsigned long myChannelNumber = SECRET_CH_ID; // ID del canal de ThingSpeak proveniente de la librería secrets.h
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;
const char * myReadAPIKey = SECRET_READ_APIKEY; // Llave de escritura del canal de ThingSpeak proveniente de la librería secrets.h
char jsonBuffer[30000] = "["; // Buffer para almacenamiento de JSON

// Para SD
File dataFile;

// Setup

void setup() {


  Serial.begin(115200);
  Serial.println("Inicio del programa");
  // Inicializar el módulo SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Error al inicializar la tarjeta SD");
    return; } else {Serial.println("Tarjeta SD inicializada");}

  // Inicializar ESP01
  setEspBaudRate(ESP_BAUDRATE); 

  // Inicializar WiFi
  WiFi.init(&Serial1);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(5000);
  }
  
  ThingSpeak.begin(client); // Iniciar ThingSpeak con el cliente WiFi anterior
  startTime = millis(); // Iniciar tiempo de medición
  lastConnectionTime = millis(); // Iniciar tiempo de última conexión

  readThingSpeakFields();

}

void loop() {
  currentTime = millis();
  if (currentTime - startTime >=  updateInterval) { // Si ya ha transcurrido un tiempo de muestreo
    updatesJson(jsonBuffer);
  }
}


void setEspBaudRate(unsigned long baudrate){
  long rates[6] = {115200,74880,57600,38400,19200,9600};
  Serial.print("Seteando el baudrate del ESPO01 a: ");
  Serial.print(baudrate);
  Serial.println("...");
  for(int i = 0; i < 6; i++){
    Serial1.begin(rates[i]);
    delay(100);
    Serial1.print("AT+UART_DEF=");
    Serial1.print(baudrate);
    Serial1.print(",8,1,0,0\r\n");
    delay(100);  
  }
  Serial1.begin(baudrate);
}

void updatesJson(char* jsonBuffer) {
  int val1 = analogRead(potPin1);
  float angle1 = val1 * m1+b1; // Convertir valor analógico a grados
  int val2 = analogRead(potPin2);
  float angle2 = val2 * m2+b2; // Convertir valor analógico a grados

  if (angle1 > angle1Max) {
    angle1Max = angle1;
  }
  if (angle1 < angle1Min) {
    angle1Min = angle1;
  }

  // Actualizar valores máximos y mínimos de angle2
  if (angle2 > angle2Max) {
    angle2Max = angle2;
  }
  if (angle2 < angle2Min) {
    angle2Min = angle2;
  }

  if (isMovingUp) {
    if (angle2 >= angle2MaxLimit && !reachedMax) {
      // Ha alcanzado el límite superior
      reachedMax = true;
      isMovingUp = false;
      Serial.println("Alcanzó el límite superior, espera el límite inferior para completar la repetición.");
      reachedMin = false;
    }
  } else {
    if (angle2 <= angle2MinLimit && !reachedMin) {
      // Ha alcanzado el límite inferior
      reachedMin = true;
      isMovingUp = true;
      repetitionCount++;  // Cuenta una repetición completa
      reachedMax = false; // Reiniciar para la próxima repetición
      Serial.print("Repetición completada. Total de repeticiones: ");
      Serial.println(repetitionCount);
    }
  }

  Serial.print("Ángulo 1: "); Serial.print(angle1);
  Serial.print(" | Ángulo 2: "); Serial.println(angle2);
  strcat(jsonBuffer, "{\"delta_t\":"); // Relative timestamp
  unsigned long deltaT = (millis() - previousUpdate) / 1000;
  size_t lengthT = String(deltaT).length(); // Returns length of deltaT string
  char temp[lengthT];
  String(deltaT).toCharArray(temp, lengthT + 1); // Copies deltaT string characters to the temp buffer.
  strcat(jsonBuffer, temp); // Concatenate 'temp' buffer to json buffer
  strcat(jsonBuffer, ",");  // Concatenate ', ' to json buffer
  strcat(jsonBuffer, "\"field1\":");  // Concatenate '\"field1\":' to json buffer
  lengthT = String(angle1).length(); // Returns length of angle 1 string
  char temp1[lengthT];
  String(angle1).toCharArray(temp1, lengthT + 1);  // Data uploaded to ThinkSpeak channel field 1
  strcat(jsonBuffer, temp1); // Concatenate 'temp' buffer to json buffer
  strcat(jsonBuffer, ",");  // Concatenate ', ' to json buffer
  strcat(jsonBuffer, "\"field2\":"); // Concatenate '\"field2\":' to json buffer
  lengthT = String(angle2).length();
  char temp2[lengthT];
  String(angle2).toCharArray(temp2, lengthT + 1);   // Data uploaded to ThinkSpeak channel field 2
  strcat(jsonBuffer, temp2); // Concatenate 'temp' buffer to json buffer
  strcat(jsonBuffer, "},"); // Concatenate '},' buffer to json buffer
  Serial.println("Json update done");
  size_t lennn = strlen(jsonBuffer);  // Returns length of jsonBuffer
  Serial.print("Tamaño de json: "); Serial.println(lennn); //Prints the length of jsonbuffer
  currentTime = millis();

  if (currentTime - lastConnectionTime >=  postingInterval) {
    Force();
    Serial.print("Ángulo 1 máximo: "); Serial.print(angle1Max);
    Serial.print(" | Ángulo 1 mínimo: "); Serial.print(angle1Min);
    Serial.print(" | Ángulo 2 máximo: "); Serial.print(angle2Max);
    Serial.print(" | Ángulo 2 mínimo: "); Serial.print(angle2Min);
    Serial.print(" | Total de repeticiones: "); Serial.print(repetitionCount);
    Serial.print(" | Fuerza medida: "); Serial.print(fuerza);
    size_t len = strlen(jsonBuffer);
    jsonBuffer[len - 1] = ']';
    Serial.println(jsonBuffer);
    saveJsonToSD(jsonBuffer);
    updateThingSpeakFields();
    
    
    
    //Serial.println("Iniciando función HTTP request");
    //httpRequest(jsonBuffer);  // Calls the 'httpRequest' subroutine
    resetRepetitionLogic();
  }
  startTime = millis(); // Update the last update time
}

void httpRequest(char* jsonBuffer) {
  Serial.println("Haciendo request");
  char data[30000] = "{\"write_api_key\":\"";
  strcat(data, myWriteAPIKey); // Usa la variable global myWriteAPIKey
  strcat(data, "\",\"updates\":");
  strcat(data, jsonBuffer);
  strcat(data, "}");
  
  // Close any connection before sending a new request
  client.stop();
  String data_length = String(strlen(data) + 1); // Returns length of 'data' string
  Serial.print("Longitud del envío: "); Serial.println(data_length);
  
  // POST data to ThingSpeak
  if (client.connect(server, 443)) {
    Serial.println("CONECTADO A THINGSPEAK");
    
    // Usa la variable global myChannelNumber
    client.print("POST /channels/");
    client.print(myChannelNumber);
    client.println("/bulk_update.json HTTP/1.1");
    
    client.println("Host: api.thingspeak.com");
    client.println("User-Agent: mw.doc.bulk-update (Arduino ESP8266)");
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + data_length);
    client.println();
    
    Serial.println("A punto de intentar enviar data");
    client.println(data);
    Serial.println("Envio intentado");
    delay(1000); // Wait to receive the response
  } else {
    Serial.println("Failure: Failed to connect to ThingSpeak");
  }
  
  delay(500); // Wait to receive the response
  client.parseFloat();
  String resp = String(client.parseInt());
  Serial.println("Response code:" + resp);
  
  // Reinitialize the jsonBuffer for next batch of data
  jsonBuffer[0] = '[';             
  jsonBuffer[1] = '\0';
  
  lastConnectionTime = millis();   // Update the last connection time
}

void saveJsonToSD(char* jsonBuffer) {
  dataFile = SD.open("data.json", FILE_WRITE);
  
  if (dataFile) {
    Serial.println("Guardando JSON en la tarjeta SD...");
    dataFile.println(jsonBuffer);  // Escribe el JSON en el archivo
    dataFile.close();  // Cierra el archivo
    Serial.println("JSON guardado exitosamente.");
  } else {
    Serial.println("Error al abrir el archivo data.json para escritura.");
  }
}

void resetRepetitionLogic() {
  reachedMax = false;
  reachedMin = false;
  isMovingUp = true;  // El ángulo comenzará subiendo
  Serial.println("Lógica de repetición reseteada.");
}

void updateThingSpeakFields() {
  Serial.println("Enviando actualización de máximos, mínimos y repeticiones a ThingSpeak...");
  //char data[30000] = "{\"write_api_key\":\"JY2YI0RF1F1O1EEY\",\"updates\":";
  char data[30000] =   "{\"write_api_key\":\"";
  strcat(data, myWriteAPIKey); // Usa la variable global myWriteAPIKey
  strcat(data, "\",\"updates\":");
  // Calcular deltaT

  // Crear el buffer para el JSON de actualización
  char updateJsonBuffer[10000] = "["; // Asegúrate de que este tamaño sea suficiente para tus datos

  // Inicializar el updateJsonBuffer
  strcat(updateJsonBuffer, "{\"delta_t\":"); // Relative timestamp
  unsigned long deltaT = (millis() - previousUpdate) / 1000;
  size_t lengthT = String(deltaT).length(); // Returns length of deltaT string
  char temp[lengthT];
  String(deltaT).toCharArray(temp, lengthT + 1); // Copies deltaT string characters to the temp buffer.
  strcat(updateJsonBuffer, temp);
  strcat(updateJsonBuffer, ","); 

  // Field1: angle1Max
  strcat(updateJsonBuffer, "\"field1\":");
  lengthT = String(angle1Max).length();
  char temp1[lengthT + 1];
  String(angle1Max).toCharArray(temp1, lengthT + 1);
  strcat(updateJsonBuffer, temp1);
  strcat(updateJsonBuffer, ",");  // Concatenate ', ' to json buffer

  // Field2: angle1Min
  strcat(updateJsonBuffer, "\"field2\":");
  lengthT = String(angle1Min).length();
  char temp2[lengthT + 1];
  String(angle1Min).toCharArray(temp2, lengthT + 1);
  strcat(updateJsonBuffer, temp2);
  strcat(updateJsonBuffer, ",");  // Concatenate ', ' to json buffer

  // Field3: angle2Max
  strcat(updateJsonBuffer, "\"field3\":");
  lengthT = String(angle2Max).length();
  char temp3[lengthT + 1];
  String(angle2Max).toCharArray(temp3, lengthT + 1);
  strcat(updateJsonBuffer, temp3);
  strcat(updateJsonBuffer, ",");  // Concatenate ', ' to json buffer

  // Field4: angle2Min
  strcat(updateJsonBuffer, "\"field4\":");
  lengthT = String(angle2Min).length();
  char temp4[lengthT + 1];
  String(angle2Min).toCharArray(temp4, lengthT + 1);
  strcat(updateJsonBuffer, temp4);
  strcat(updateJsonBuffer, ",");  // Concatenate ', ' to json buffer

  // Field5: repetitionCount
  strcat(updateJsonBuffer, "\"field5\":");
  lengthT = String(repetitionCount).length();
  char temp5[lengthT + 1];
  String(repetitionCount).toCharArray(temp5, lengthT + 1);
  strcat(updateJsonBuffer, temp5);
  strcat(updateJsonBuffer, ",");  // Concatenate ', ' to json buffer

  strcat(updateJsonBuffer, "\"field6\":");
  lengthT = String(fuerza).length();
  char temp6[lengthT + 1];
  String(fuerza).toCharArray(temp6, lengthT + 1);
  strcat(updateJsonBuffer, temp6);

  // Finaliza el JSON
  strcat(updateJsonBuffer, "}]");
  strcat(data, updateJsonBuffer);
  strcat(data, "}");
  // Close any previous connection
  client.stop();
  Serial.println(updateJsonBuffer);
  // Establecer la longitud de los datos
  String data_length = String(strlen(data) + 1); // Returns length of 'updateJsonBuffer'
  Serial.print("Longitud del envío: "); Serial.println(data_length);
  
  // POST data to ThingSpeak
  if (client.connect(server, 80)) {
    Serial.println("CONECTADO A THINGSPEAK");
    //client.println("POST /channels/2522041/bulk_update.json HTTP/1.1"); // TODO - replace CHANNEL-ID with your ThingSpeak channel ID
    client.print("POST /channels/");
    client.print(myChannelNumber);
    client.println("/bulk_update.json HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("User-Agent: mw.doc.bulk-update (Arduino ESP8266)");
    client.println("Connection: close");
    client.println("Content-Type: application/json");
    client.println("Content-Length: " + data_length);
    client.println();
    Serial.println("A punto de intentar enviar data");
    client.println(data);
    Serial.println("Envio intentado");
    // Esperar para recibir la respuesta
    delay(500); // Esperar para recibir la respuesta
  } else {
    Serial.println("Failure: Failed to connect to ThingSpeak");
  }

  // Leer la respuesta del servidor
  client.parseFloat();
  String resp = String(client.parseInt());
  Serial.println("Response code:" + resp);
  updateJsonBuffer[0] = '[';             
  updateJsonBuffer[1] = '\0';
  jsonBuffer[0] = '[';             
  jsonBuffer[1] = '\0';
  readThingSpeakFields();
  Serial.println("Delay");
  delay(15000);
  lastConnectionTime = millis();
}

void readThingSpeakFields(){
    int statusCode=0 ;
    ejercicio = ThingSpeak.readLongField(myChannelNumber, 7, myReadAPIKey); 
    statusCode = ThingSpeak.getLastReadStatus();
    if (statusCode == 200)
    {
      Serial.println("Ejercicio: " + String(ejercicio));
    }
    else
    {
      Serial.println(F("Problem reading channel"));
    }

    statusCode = 0;

    resistencia= ThingSpeak.readLongField(myChannelNumber, 8, myReadAPIKey); 
    statusCode = ThingSpeak.getLastReadStatus();
    if (statusCode == 200)
    {
      Serial.println("Resistencia: " + String(resistencia));
    }
    else
    {
      Serial.println(F("Problem reading channel"));
    } delay(1000);
}

// void Force(){
//     float px1 = 181.17
//     int resis = int(resistencia);
//     // Selección de ecuación de fuerza según el valor de resistencia
//     switch (resis) {
//         case 1: // Primera ecuación para resistencia 1
//             fuerza = 2.5 * angle2Max;  // Ejemplo de ecuación
//             break;
//         case 2: // Segunda ecuación para resistencia 2
//             fuerza = 3.0 * angle2Max + 5.0;  // Ejemplo de ecuación
//             break;
//         case 3: // Tercera ecuación para resistencia 3
//             fuerza = 4.0 * angle2Max - 10.0;  // Ejemplo de ecuación
//             break;
//         default:
//             Serial.println("Resistencia no válida");
//             fuerza = 0; // Valor de error en caso de resistencia no válida
//     }
// }

void Force() {
    float px1 = 181.17;
    float theta = (-39.4 + angle2Max) * DEG_TO_RAD;  // Conversión de grados a radianes
    float p1x = px1 * cos(theta);
    float p1z = px1 * sin(theta);
    float p1y = 0.0;

    float p2x = 166.0;
    float p2y = -74.0;
    float p2z = -336.0;

    // Cálculo de la distancia entre p1 y p2
    float distancia = sqrt(pow(p1x - p2x, 2) + pow(p1y - p2y, 2) + pow(p1z - p2z, 2));

    // Parámetros y cálculo del delta de distancia
    float k = 2.0;
    float delta_distancia = distancia - 74.0;

    // Cálculo de fuerzas con las ecuaciones dadas
    float F_resorte = k * (-0.0003 * pow(delta_distancia, 2) + 0.167 * delta_distancia + 1.0758);
    float F_verde = k * (-0.00007 * pow(delta_distancia, 2) + 0.0832 * delta_distancia + 0.1277);
    float F_amarilla = k * (-0.0001 * pow(delta_distancia, 2) + 0.1204 * delta_distancia + 0.665);
    float F_azul = k * (-0.0001 * pow(delta_distancia, 2) + 0.1104 * delta_distancia + 0.266);

    // Cálculo de componentes unitarios para el vector p2-p1
    float vec_p2_p1_x = p1x - p2x;
    float vec_p2_p1_y = p1y - p2y;
    float vec_p2_p1_z = p1z - p2z;
    float magnitud = sqrt(pow(vec_p2_p1_x, 2) + pow(vec_p2_p1_y, 2) + pow(vec_p2_p1_z, 2));
    float unitario_p2_p1_z = vec_p2_p1_z / magnitud;

    // Componentes en Z de las fuerzas
    float F_resorte_z = F_resorte * unitario_p2_p1_z;
    float F_verde_z = F_verde * unitario_p2_p1_z;
    float F_amarilla_z = F_amarilla * unitario_p2_p1_z;
    float F_azul_z = F_azul * unitario_p2_p1_z;

    // Cálculo de torques
    float torque = p1x * F_resorte_z / 1000.0;
    float torque_azul = p1x * F_azul_z / 1000.0;
    float torque_amarillo = p1x * F_amarilla_z / 1000.0;
    float torque_verde = p1x * F_verde_z / 1000.0;

    // Selección del torque a retornar según el valor de resistencia
    int resis = int(resistencia);
    switch (resis) {
        case 1:
            fuerza = torque_verde;
            break;
        case 2:
            fuerza = torque_azul;
            break;
        case 3:
            fuerza = torque_amarillo;
            break;
        case 4:
            fuerza = torque;
            break;
        default:
            Serial.println("Resistencia no válida");
            fuerza = 0;  // Valor de error en caso de resistencia no válida
    }
}

