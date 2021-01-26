
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <Wire.h>

//const char *coverAddress = "192.168.178.100";
//IPAddress coverIP(192,168,178,100);
const char *coverAddress = "141.58.8.5";
IPAddress coverIP(141,58,8,5);
const int coverPort = 31319;
WiFiUDP toCOVER;

struct messageBuffer
{
  float fl;
  float fr;
  float rl;
  float rr;
  unsigned long state;
};

struct messageBuffer mb;

// Replace with your network credentials
//#include "../../../wifiPasswd.h"
#include "../../../wifiPasswdHLRS.h"
#include <HX711.h>
#include "debounceButton.h"

// HX711 circuit wiring
const int S0_DOUT_PIN = 32;
const int S0_SCK_PIN = 33;
const int S1_DOUT_PIN = 25;
const int S1_SCK_PIN = 26;
const int S2_DOUT_PIN = 17;
const int S2_SCK_PIN = 16;
const int S3_DOUT_PIN = 19;
const int S3_SCK_PIN = 18;

const int numSensors = 4;

long zeroOffset[numSensors];
const int ZeroPin = 22;
debounceButton zeroButton(ZeroPin);


HX711 sensor[numSensors];

void calcZeroOffset()
{
  Serial.println("Calibrating Zero");
  for (int i = 0; i < 10; i++)
  {
    for (int n = 0; n < numSensors; n++)
    {
      if (sensor[n].wait_ready_timeout(1000))
      {
        long reading = sensor[n].read();
        if (i == 0)
        {
          zeroOffset[n] = reading;
        }
        else
        {
          zeroOffset[n] = (float)(i - 1) / float(i) * zeroOffset[n] + 1.0 / float(i) * zeroOffset[n];
        }
      }
      else
      {
        Serial.println("HX711 not found.");
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  toCOVER.begin(coverPort);

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Skateboard");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  sensor[0].begin(S0_DOUT_PIN, S0_SCK_PIN);
  sensor[1].begin(S1_DOUT_PIN, S1_SCK_PIN);
  sensor[2].begin(S2_DOUT_PIN, S2_SCK_PIN);
  sensor[3].begin(S3_DOUT_PIN, S3_SCK_PIN);
  zeroButton.init(true);

  calcZeroOffset();
}
float weight=0.0;
int numValues = 0;
void loop()
{
  ArduinoOTA.handle();
  debounceButton::update();

  long reading[numSensors];
  float weight[numSensors];
  for (int n = 0; n < numSensors; n++)
  {
    if (sensor[n].wait_ready_timeout(1000))
    {
      reading[n] = sensor[n].read();
      weight[n] = (float)(reading[n] - zeroOffset[n])/23.22;
    }
    else
    {
      Serial.printf("HX711 %d not found.\n",n);
    }
  }
  toCOVER.beginPacket(coverIP,coverPort);
  mb.fl = weight[0]/1000.0;
  mb.fr = weight[1]/1000.0;
  mb.rl = weight[2]/1000.0;
  mb.rr = weight[3]/1000.0;
  mb.state = 0;
  int received = toCOVER.parsePacket();
  if(received>0)
  {
      unsigned char buffer[100];
      int numRead = toCOVER.read(buffer, 100);
      if(numRead > 0)
      {
          Serial.printf("Read %d bytes\n",numRead);
      }
  }
  //Serial.printf("weight %f %f\n",weight[0],weight[1]);
  
  if(zeroButton.wasKlicked())
  { 
    Serial.printf("button Klicked\n");
    mb.state = 1;
  }
  if(zeroButton.isPressed())
  { 
    Serial.printf("button isPressed\n");
    mb.state = 2;
  }
  if(zeroButton.wasDoubleKlicked())
  {
    calcZeroOffset();
    Serial.printf("button Double Klicked\n");
    mb.state = 4;
  }
  toCOVER.write((const uint8_t *)&mb,sizeof(mb));
  toCOVER.endPacket();

}