#include<TinyGPS++.h>
#include<SoftwareSerial.h>
#include<ESP8266WiFi.h>
#include<DHT.h>
#include<SDS011.h>

#define DHT_TYPE DHT22
uint8_t DHTPin = D2;
DHT dht(DHTPin, DHT_TYPE);

SDS011 sds011;
uint8_t sds011Response;

uint8_t MQ9PowerPin = D0;
uint8_t MQ9Pin = A0;

WiFiClient mClient;
const char* ssid     = "Malibongwe";
const char* password = "Hephzibah";
const char* server = "https://iot-air-quality-backend.herokuapp.com";

TinyGPSPlus gps;
SoftwareSerial gpsSerial(D5, D6); //(Rx, Tx)

// Data to send to server
float temperature, humidity, gasConcentration, pm25, pm10,latitude, longitude;

// Counter to determine when data will be sent to the server
int intervalCounter = 0;

// Function to send captured sensor data to server
void sendEnvironmentData(float temperature, float humidity, float gasConcentration, float pm25, float pm10, float latitude, float longitude){

  String endPoint = "/api/v1/environment-properties/" + 
                      String(temperature) + "/" + 
                      String(humidity) + "/" + 
                      String(gasConcentration) + "/" + 
                      String(pm25) + "/" + 
                      String(pm10) + "/" + 
                      String(latitude) + "/" + 
                      String(longitude);
  
  if (mClient.connect(server, 5000))
  { 
    Serial.println(F("Connected to remote server on port 80"));

    Serial.println(F("Attempting to send captured data to remote server..."));

    const String request =  "GET " + endPoint + " HTTP/1.1\r\n" +
                            "Host: https://iot-air-quality-backend.herokuapp.com\r\n" +
                            "Connection: close\r\n" +
                            "\r\n";
                            
    mClient.print(request);
    mClient.flush();

    delay(3000);

    Serial.println("Response: ");
    
    while(mClient.available()){
      char c = mClient.read();
      Serial.print(c);
    }
    Serial.println(F("Data sent to remote server"));
  }else{
    Serial.println(F("Could not connect to the server. Check your internet connection."));
  }
  mClient.stop();
}

void handlePmSensor(){
  sds011Response = sds011.read(&pm25, &pm10);
  if (!sds011Response) {
    Serial.print(F("P2.5: "));
    Serial.println(pm25);
    Serial.print(F("P10:  "));
    Serial.println(pm10);
  }else{
    Serial.println(F("SDS011 Error"));
  }
}

void handleDhtSensor(){
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  Serial.print(F("Temperature: "));
  Serial.println(temperature);
  Serial.print(F("Humidity: "));
  Serial.println(humidity);
}

void handleMQ9Sensor(){
  int rawValue = analogRead(MQ9Pin);
  float sensorVoltage = rawValue / 1024 * 5.0;
  float RsGas = (5.0 - sensorVoltage)/sensorVoltage;
  gasConcentration = RsGas/3.78;
  Serial.print(F("Gas concentration: "));
  Serial.println(gasConcentration);
}

void setup() {
  // Set up the serial monitor
  Serial.begin(9600);

  //Set up the Wi-Fi module
  Serial.println();
  Serial.print(F("Trying to connect to ")); 
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  delay(1000);
  Serial.print(F("WiFi connected - IP address: "));
  Serial.println(WiFi.localIP());

  // Set up DHT22 sensor
  pinMode(DHTPin, INPUT);
  dht.begin();

  // Set up the PM sensor
   sds011.begin(D3, D4);

  // Set up the MQ9 sensor
  pinMode(MQ9PowerPin, OUTPUT);
  digitalWrite(MQ9PowerPin, HIGH); // Power MQ9 via D0
  pinMode(MQ9Pin, INPUT);

  // Set up the GPS module
  gpsSerial.begin(9600);
  
}

void loop() {
  while (gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read())){
      if (gps.location.isValid()){
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        Serial.print(F("Latitude: "));
        Serial.println(latitude);
        Serial.print(F("Longitude: "));
        Serial.println(longitude);

        handlePmSensor();
        handleDhtSensor();
        handleMQ9Sensor();

        if(intervalCounter%200 == 0){
          if(temperature != 0 && humidity != 0 && gasConcentration != 0 && pm25 != 0 && pm10 != 0 && latitude != 0 && longitude != 0){
            sendEnvironmentData(temperature, humidity, gasConcentration, pm25, pm10, latitude, longitude);
            Serial.println(F("........................................................................."));
          }
        }

        intervalCounter++;
      }
    }
  }
}
