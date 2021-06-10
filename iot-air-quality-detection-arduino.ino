#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include<DHT.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#include <SDS011.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Variables for Wi-Fi connection
#ifndef STASSID
  #define STASSID "Malibongwe"
  #define STAPSK  "Hephzibah"
#endif

#define DHT_TYPE DHT22
uint8_t DHTPin = D2;
DHT dht(DHTPin, DHT_TYPE);

float p10, p25;
int error;
SDS011 sds011;

uint8_t MQ5Pin = A0;

WiFiClient mClient;
const char* ssid     = STASSID;
const char* password = STAPSK;
const char* server = "192.168.43.7";

TinyGPSPlus GPS;
SoftwareSerial gpsSerial(D5, D6); //(Rx, Tx)

float fLatitude, fLatitude, fLatitude, fLatitude
// Data to send to server
String temperature = "25";
String humidity = "80";
String gasConcentration = "45";
String pm25 = "10.5";
String pm10 = "7.5";
String latitude = "4.5";
String longitude = "10.2";

// Counter to determine when data will be sent to the server
int intervalCounter = 0;

// Function to send captured sensor data to server
void sendEnvironmentData(String temperature, String humidity, String gasConcentration, String pm25, String pm10, String latitude, String longitude){

  String endPoint = "/api/v1/environment-properties/" + 
                      temperature + "/" + 
                      humidity + "/" + 
                      gasConcentration + "/" + 
                      pm25 + "/" + 
                      pm10 + "/" + 
                      latitude + "/" + 
                      longitude;
  
  if (mClient.connect(server, 5000))
  { 
    Serial.println(F("Connected to remote server on port 80"));

    Serial.println(F("Attempting to send captured data to remote server..."));

    const String request =  "GET " + endPoint + " HTTP/1.1\r\n" +
                            "Host: 192.168.43.7\r\n" +
                            "Connection: close\r\n" +
                            "\r\n";
                            
    mClient.print(request);
    mClient.flush();

    delay(5000);

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

void setup() {
  // Set up the serial monitor
  Serial.begin(9600);

  // Set up DHT22 sensor
  pinMode(DHTPin, INPUT);
  dht.begin();

  // Set up the PM sensor
  sds011.begin(D3, D4);

  pinMode(MQ5Pin, INPUT);

  //Set up the GPS module
  gpsSerial.begin(9600);

  //Set up the Wi-Fi module
  Serial.println();
  Serial.print(F("Trying to connect to ")); 
  Serial.println(String(ssid));
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
  
}

void loop() {
  // Read temperature and humidity from DHT22 sensor
  temperature = String(dht.readTemperature());
  humidity = String(dht.readHumidity());
  Serial.print(F("Temperature: "));
  Serial.println(temperature);
  Serial.print(F("Humidity: "));
  Serial.println(humidity);

  // Read PM2.5 and PM10 from PM sensor
  error = sds011.read(&p25, &p10);
  if (!error) {
    pm25 = String(p25);
    pm10 = String(p10);
    Serial.println("P2.5: "+String(p25));
    Serial.println("P10:  "+String(p10));
  }else{
    Serial.println("SDS011 Error:  "+String(error));
  }

  // Read gas concentration from MQ5 sensor
  int MQ5Output = analogRead(MQ5Pin);
  //TODO: Compute actual gas concentration from MQ5Output
  gasConcentration = String(MQ5Output);
  Serial.println("MQ5:  "+gasConcentration);
  
  // Read location data from GPS sensor
  while(gpsSerial.available() > 0){
    if (GPS.encode(gpsSerial.read())){
      if(GPS.location.isUpdated()){
        double lat = GPS.location.lat();
        double lng = GPS.location.lng();
        latitude = String(lat);
        longitude = String(lng));
        Serial.println("Latitude: "+latitude);
        Serial.println("Longitude:  "+longitude);
      }
    }
  }
  
  if(intervalCounter%5 == 0){
    sendEnvironmentData(temperature, humidity, gasConcentration, pm25, pm10, latitude, longitude);
    Serial.println(F("........................................................................."));
  }

  Serial.println();
  
  delay(5000);

  intervalCounter++;
}
