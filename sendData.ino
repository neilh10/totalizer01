/**
 * \file
 *
 * \brief Module to Send Meter Counts to Internet.
  Expects a Wiznet Ethernet shield. This can be 
  - Arduino Ethernet shield, or
  - Adafruit Ethernet shield, 
  
 This uses version 2.0 of the Cosm.com API, 
 and specifies two meter readings.

 http://arduino.cc/en/Tutorial/CosmClientString
 This code is in the public domain. 
 
 User Configuration required for 
  - api.cosm.com values 
  - Cosm.com Destination Address, IP# Address (DHCP client strings not working)
  - MAC address (MAC should be on Ethernet board, but hw doesn't support this right now)
 */
 
#include <SPI.h>
#include <Ethernet.h>
//#include "component_adc12b.h"
#define _WaterMeterDef_
#ifdef _WaterMeterDef_
//Datastreams ID meter1 meter2
#define APIKEY         "3WhMGqIciR-MotAOMKhOxC_t0tiSAKxKRHA1WkUzRUJDST0g" // cosm api key
#define FEEDID         71185 // your feed ID
#define USERAGENT      "WaterMeter Test" // user agent is the project name

#else
//Datastreams ID meter1 meter2 depth1 depth2 tick
#define APIKEY         "V4oIWxvVzSDFLWagBJZZrRnYDqGSAKxEdEZHSkVWMGwyWT0g" // your cosm api key
#define FEEDID         80470 // your feed ID
#define USERAGENT      "WaterLevel Test" // user agent is the project name
#endif  //
// assign a MAC address for the ethernet controller.
// fill in your address here: 
  byte mac[] = { 
  0x90, 0xA2, 0xDA, 0x0D, 0x4B, 0x53 };    
  
// fill in an available IP address on your network here,
// for manual configuration:
IPAddress ip(10,0,0,177);

// initialize the library instance:
EthernetClient client;

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress cosmServerIp(216,52,233,121);      // numeric IP for api.cosm.com
char cosmServerIp[] = "api.cosm.com";   // name address for Cosm API

unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
boolean lastConnected = false;                 // state of the connection last time through the main loop


/**
 * \brief Configure the Internet to be able to send data
 *
 */
void setupSendData() {

  // give the ethernet module time to boot up:
  // start the Ethernet connection:
   Serial.print("Starting connection ");
   Serial.println(FEEDID,DEC);

   delay(100);
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP, using default");
    // DHCP failed, so use a fixed IP address:
    Ethernet.begin(mac, ip);
  }
  // print IP address:
  Serial.print("IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print("."); 
  }
  Serial.println();
}

/**
 * \brief Send pulse meter readings to the internet
 *
 */
void sendPulseMeters() {
  // read the analog sensor:
  //adc_enable_ts(ADC12B);
  //int sensorReading = analogRead(A11);  //Temperature 
  // convert the data to a String to send it:

  String dataString = "meter1,";
  dataString += meter1cntIncr;

  // Append Meter2:
  //int otherSensorReading = analogRead(A11);
  dataString += "\nmeter2,";
  dataString += meter3cntIncr;
#ifndef _WaterMeterDef_ 
  dataString += "\ndepth1,";
  dataString += waterLevel1_mm;
  dataString += "\ndepth2,";
  dataString += waterLevel2_mm;
  dataString += "\ntick,";
  dataString += testTick;
#endif// _WaterMeterDef_
  if (3 < ++testTick) testTick=0;
  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  if (client.available()) {
    char c = client.read();
    Serial.print(c);
  }

  // if there's no net connection, but there was one last time
  // through the loop, then stop the client:
  if (!client.connected() && lastConnected) {
    Serial.println();
    Serial.println("spm:forcing disconnect.");
    client.stop();
  }

  // if you're not connected, and ten seconds have passed since
  // your last connection, then connect again and send data: 
  //if(!client.connected() && (millis() - lastConnectionTime > postingInterval)) 
  {
    sendHttpRequest(dataString);
  }
  delay(2);
  if (client.connected() ) {
    Serial.println("spm:completed...");
    client.stop();
  }

  // Keep for error checking
  lastConnected = client.connected();
}

// this method makes a HTTP connection to the server:
void sendHttpRequest(String thisData) {
  Serial.println("\n\rsendData-shr: ");
  Serial.println(thisData);
  delay(10);
  // if there's a successful connection:
  Serial.print("connecting...");
  if (client.connect(cosmServerIp, 80)) {
    Serial.print("connected...");
    delay(10);
    // send the HTTP PUT request:
    client.print("PUT /v2/feeds/");
    client.print(FEEDID);
    Serial.print(FEEDID);
    client.println(".csv HTTP/1.1");
    client.println("Host: api.cosm.com");
    client.print("X-ApiKey: ");
    client.println(APIKEY);
    client.print("User-Agent: ");
    client.println(USERAGENT);
    client.print("Content-Length: ");
    client.println(thisData.length());

    // last pieces of the HTTP PUT request:
    client.println("Content-Type: text/csv");
    client.println("Connection: close");
    client.println();

    // here's the actual content of the PUT request:
    client.println(thisData);
    //Serial.println(thisData);
    Serial.println("..delivered ");
    LedActivityUpdateSlow();
  } else {
    // if you couldn't make a connection:
    LedActivityUpdateFast();
    Serial.println("shr:connection failed- disconnecting");
    client.stop();
  }
  // note the time that the connection was made or attempted:
  lastConnectionTime = millis();
}
