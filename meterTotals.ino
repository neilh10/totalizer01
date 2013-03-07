/*
  Meter Reader
 
 Capture pulses from an inline Meter and then send them to the cloud.

 User needs to set update value

 This uses the String library,requires Arduino core version 0019.  
 
 Circuit:
 * Digital realy closures attached to TCLK6,7,8
 * Ethernet shield attached to pins IPCS & 10, 11, 12, 13
 
 updated 14 August 2012 with Due/SAM3X 
 created 15 March 2010  by Tom Igoe with input from Usman Haque and Joe Saavedra
 
 */

//const unsigned long postingInterval = 5*60*1000;  //delay between updates to Webserver

void setup() {
  int waitLp=0; 
  // start serial port:
  Serial.begin(57600);
  while (!Serial && waitLp<15) {
    // wait for serial port to connect. Needed for USB ports
    delay(1000);
    waitLp++;
  }
  Serial.println("PulseMonitoring verXx starting");
  setupPulseCollection();
  
  setupSendData(); //Including ethernet declaration
  //setupNtpClient(); //After Ethernet declaration
  sendPulseMeters(); //To the internet server

}////setup


void loop() {
  //loopOutPulseTest();
  loopPulseCollection();
  
}////loop




