/**
 * \file \brief Module to setup and read Meter Pulses.

 This uses the Timer Counter (TC) to count pulses.
 The AT91SAM3X has 3 TC named TC0, TC1, TC2. 
 Each TCx has 3 channels with a 32 bit counter.
 The hardware is capable of counting a total of 9 channels.
 TC0 is used as the harware pins are convenient
 
 The pins are specified to have pullups and also debounce the incoming relay signal.
 Typically contacts of mechanical switches, relays bounce - anywhere up to 5mS. 
 A fast counter will count these if they are not smoothed out. 

 Expects meter contact closures on specified pins. 
 Meter flow is specified per open/close - 1G
 The meter used has a closure time representative of 0.1G and the open time of 0.9G

 The input pins TC0 for SAM3X8E
 Due#  SAM3X Desc 
 22     PB26/Pin1  TCLK0/XC0 input to Channel2 - Call this Meter1
 58/AD2 PA4 /Pin83 TCLK1/XC1 input to Channel0   Call this Meter2
 31     PA7 /Pin26 TCLK2/XC2 input to Channel1 - call this Meter3


Arduino 2560 pins - needs some tracking down, maybe someone familiar with it wants to
translate for the mega#
\hardware\arduino\avr\variants\mega\pins_arduino.h
mega#
22 - PA0 (physical pin#78 of Atmeag1280-16AU)
31 - PC6 (physical pin#59 of Atmega1280-16AU)

Other options for SAM3X using TC
 The input pins TC1 for SAM3X8E
 Due# SAM3X Desc 
 A3   PA22/Pin81 TCLK3   
 A2   PA23/Pin80 TCLK4 
 A13  PB16/Pin77 TCLK5
 
 The input pins TC2 for SAM3X8E
 Due# SAM3X Desc 
 --   PC27/Pin138 TCLK6   
      PC30/Pin103 TCLK7 -led Rx
 30   PD9 /Pin22  TCLK8 
 */
//#include <assert.h>
#include "TotalizerPrj.h"
#define TC_CHANNEL_0 0
#define TC_CHANNEL_1 1
#define TC_CHANNEL_2 2
#define PIN_TC0_TCLK0_ARD (22u)
#define PIN_TC0_TCLK1_ARD (58u)
#define PIN_TC0_TCLK2_ARD (31u)
//Analog Input pins A0-A15 - this is hardwired and referenced in
//SAM3X: hardware/sam/variants/arduino_due_x pins_arduino.h variant.cpp
//SAM3X hardware/sam/variants/arduinio_due_x/variants/variant.h defines default collection as 12bits
//SAM3X: A8[62] is PB17 A9 is PB18 and variant.cpp:[row63] 
//DUEX: pins A14/A15 not available 
//#define Adc4_20mA_1  A8
//#define Adc4_20mA_2  A9

  //Output
  uint8_t testTick =0;
  uint32_t meter1cntIncr = 0;
  uint32_t meter2cntIncr = 0;
  uint32_t meter3cntIncr = 0;
  uint16_t waterLevel1_mm = 0;
  uint16_t waterLevel2_mm = 0;
  //Historical internal private
  uint32_t meter1cntLstTime = 0;
  uint32_t meter2cntLstTime = 0;
  uint32_t meter3cntLstTime = 0;

unsigned long lastPollTime_mS = 0;          // last time PollTime (mS)
const unsigned long postingInterval_mS = 
  15*60*1000; // polling time (mS)
  //  30*1000; // polling time (mS)
unsigned long LedFlashInterval_mS = LED_FLASH_FAST_MS;
unsigned long lastLedTime_mS;

uint16_t ActivityLedTmr_mS=0; //Down counter so activity is visible
unsigned long lastActivityLed_mS=5000;
uint8_t ActivityLed_Sema=0;
uint8_t lastState1=HIGH,lastState2=HIGH,lastState3=HIGH;
uint8_t ledCntr=0; //Simple method of controlling LED
  
 /**
 * \brief Configure the Pulse Meter Ports
 *
 */
 void setupPulseCollection() {
  Serial.print("Publish every "); 
  Serial.print(postingInterval_mS/1000,DEC); 
  Serial.println(" secs, for pins 22 58 31"); 

//Num of SLCK to make 10mS
//doc11057b:Sect 32.7.28  2*(DIV+1)*SLCK
//#define PIN_DEBOUNCE_PERIOD_10mS    128
#define PIN_DEBOUNCE_CUTOFF_FREQ_HZ 100
  //pmc_enable_periph_clk(ID_TC0); - done in pinMode()
  //hardware\arduino\sam\cores\arduino\wiring_digital.c/h:pinMode-->PIO_Configure( DIGITAL);
  //hardware\arduino\sam\system\libsam\source\pio.c:PIO_Configure()-->PIO_SetINput
  //std Arduino - which does everything except PIO_DEBOUNCE 
  pinMode(PIN_TC0_TCLK2_ARD,INPUT_PULLUP);
  pinMode(PIN_TC0_TCLK0_ARD,INPUT_PULLUP);
  pinMode(PIN_TC0_TCLK1_ARD,INPUT_PULLUP);
  PIO_SetDebounceFilter(g_APinDescription[PIN_TC0_TCLK0_ARD].pPort,
      g_APinDescription[PIN_TC0_TCLK0_ARD].ulPin  | g_APinDescription[PIN_TC0_TCLK2_ARD].ulPin,
      PIN_DEBOUNCE_CUTOFF_FREQ_HZ);
     
  //TCx capture initialize
  //hardware\arduino\sam\system\cmsis\Device\atmel\sam3xa\include\sam3x8e.h
  //hardware/arduino/sam\system\libsam\include\pmc.h
  //harware\arduino\system\libsam\source\pmc.c
  pmc_enable_periph_clk(ID_TC2);
  pmc_enable_periph_clk(ID_TC0);
  pmc_enable_periph_clk(ID_TC1);

  //hardware\arduino\sam\system\libsam\source\tc.c
  TC_Configure(TC0, TC_CHANNEL_2, TC_CMR_TCCLKS_XC0); // Use XC0/TCLK0 on PB26 
  TC_Configure(TC0, TC_CHANNEL_0, TC_CMR_TCCLKS_XC1); // Use XC1/TCLK1 on PA5 
  TC_Configure(TC0, TC_CHANNEL_1, TC_CMR_TCCLKS_XC2); // Use XC2/TCLK1 on PA4 
  
  TC_Start(TC0, TC_CHANNEL_2);  
  TC_Start(TC0, TC_CHANNEL_0);
  TC_Start(TC0, TC_CHANNEL_1);  

  doStatePrint(); // Indicate state
  pinMode(PIN_LED, OUTPUT);   
  pinMode(PIN_LED_ACTIVITY, OUTPUT);  
  
  //ADC

  analogReadResolution(12);//Set to resolve to app readings as 12bits forever
#ifdef Adc4_20mA_1  
 Serial.print("Adc4_20mA_1=");
 Serial.print(Adc4_20mA_1);
#endif //Adc4_20mA_1
#ifdef Adc4_20mA_2
 Serial.print(" Adc4_20mA_2=");
 Serial.print((Adc4_20mA_2);
#endif //Adc4_20mA_2
 Serial.println();
 }/////setupPulseCollection
 
/**
 * \brief Entrance loop for the Pulse Collection 
 *
 */
void loopPulseCollection() {
  unsigned long time_mS = 0;

 time_mS = millis(); 
  //Timer List test.
 if((time_mS - lastPollTime_mS > postingInterval_mS)) {
   lastPollTime_mS=time_mS;
   digitalWrite(PIN_LED_TXL,LOW);
   ActivityLed_Sema=1;
   lastActivityLed_mS=time_mS;
   pollPulseCollection();
   pollAnalogCollection();
   //NtpRequest();
   sendPulseMeters(); //To the internet server
   meter1cntIncr=meter2cntIncr=meter3cntIncr=0;
    
    //Print state of pins as helpful confidence indication  
    doStatePrint();

    
    //When the meters change it is indicated as '1' or '2' or '3'
    Serial.print("MeterChnge=");
    
  }
  pollHwPins();
  
  //Turn off ActivityLed after some period
  /* needs work
  if ((time_mS - lastActivityLed_mS>0) && (1==ActivityLed_Sema)) {
    ActivityLed_Sema=0;
    Serial.print("LED_TCXL OFF"); 
    digitalWrite(PIN_LED_ACTIVITY,HIGH);
  } /* */
 //Toggle status led based on WebServer access
 if((time_mS - lastLedTime_mS > LedFlashInterval_mS)) {
   lastLedTime_mS=time_mS;
   //Serial.println("LED_STAT TGL"); 
   digitalWrite(PIN_LED,(ledCntr++ & 0x01));
 }
}////loopPulseCollection

/**
 * \brief snapshot the Analog registers
 * 
 * Hardware is Pulse/4-20mA board, R=49.9ohms, Vref=2500mV
 * Using 12bit - each bit is 2500/4096mV or wmV/bit */
#define c_mvBit 0.6103515625
 /* V=IR where I is 4 to 20mA
 * the output in mA is 4096/(2500x49.9)
 *
 **** Metric
 * For a scale of 0-3m for 4mA to 20mA
 * the conversion is 3/16 m/mA or 375/2 mm/mA
 * For output in mm
 * 4096*375/(2500x49.9x2) = 6.15631 = 1536/(5*49.9)
 *
 *** Feet
 * For a scale of 0-9.84252ft (0-3m) for 4mA to 20mA
 * the conversion is 9.84252/16 ft/mA or 984.252/16 0.01ft/mA
 * 
 *
 */
 void pollAnalogCollection() {
   uint32_t sensorRawAdc, sensorRaw_mV;
   //Setup ADC12bit which calls 
   //--> hardware\arduino\sam\cores\arduino\wiring_analog.c which calls
   //--> hardware\arduino\sam\system\libsam\source\adc.c
 #ifdef Adc4_20mA_1
   sensorRawAdc = sensorRawAdc=analogRead(Adc4_20mA_1);
   sensorRaw_mV =(uint32_t)((float)sensorRawAdc *  c_mvBit);
   waterLevel1_mm = sensorRaw_mV;
       Serial.print("depth1(raw)=");
       Serial.print(sensorRawAdc);
       Serial.print(" mV=");
       Serial.println(sensorRaw_mV);
#endif //#ifdef Adc4_20mA_2
#ifdef Adc4_20mA_2       
   sensorRawAdc = sensorRawAdc=analogRead(Adc4_20mA_2);
   sensorRaw_mV =(uint32_t)((float)sensorRawAdc *  c_mvBit);

   waterLevel2_mm = sensorRaw_mV;
          Serial.print("depth2(raw)=");
       Serial.print(sensorRawAdc);
       Serial.print(" mV=");
       Serial.println(sensorRaw_mV);
#endif // Adc4_20mA_2
 }
/**
 * \brief Poll the HwPins to collect pin changes
 *  This is expected to be called regularly and could miss
 *  pin level changes if there is a holdup somewhere else
 */
void pollHwPins() {
  uint8_t pinState1, pinState2,pinState3;
  
  pinState1 = digitalRead(PIN_TC0_TCLK0_ARD);
  pinState2 = digitalRead(PIN_TC0_TCLK1_ARD);
  pinState3 = digitalRead(PIN_TC0_TCLK2_ARD);

  if (pinState1 != lastState1){
    lastState1=pinState1;
    meter1cntIncr++;
    Serial.print("1"); 

  } 
  
  if (pinState2 != lastState2){
    lastState2=pinState2;
    meter2cntIncr++;
    Serial.print("2"); 
  } 

  if (pinState3 != lastState3){
    lastState3=pinState3;
    meter3cntIncr++;
    Serial.print("3"); 
  }
}////pollHwPins

/**
 * \brief snapshot the Pulse Counter registers
 *
 */
 void pollPulseCollection() {

  uint32_t meter1cntThisTime = TC_ReadCV(TC0,TC_CHANNEL_2);
  uint32_t meter2cntThisTime = TC_ReadCV(TC0,TC_CHANNEL_0);
  uint32_t meter3cntThisTime = TC_ReadCV(TC0,TC_CHANNEL_1);
 // uint32_t meter1regStatus = TC_GetStatus(TC0,TC_CHANNEL_2);
 // uint32_t meter2regStatus = TC_GetStatus(TC0,TC_CHANNEL_0);
 // uint32_t meter3regStatus = TC_GetStatus(TC0,TC_CHANNEL_1);

  Serial.println(""); 
  Serial.print(millis()/1000, DEC); 


  Serial.print(" rppc: "); 
 // Serial.print(" {"); 
 // Serial.print(meter1regStatus, HEX);
 // Serial.print("/"); 
 // Serial.print(meter2regStatus, HEX);
 // Serial.print("/"); 
 // Serial.print(meter3regStatus, HEX);
 // Serial.print("} "); 

 //------------------------------------------------
  Serial.print(" meter1="); 
  Serial.print(meter1cntIncr/2, DEC); 
  //chkRounding(meter1cntIncr); 
  Serial.print("/"); 

  if (meter1cntThisTime >= meter1cntLstTime) {
     meter1cntIncr = meter1cntThisTime -meter1cntLstTime;
  } else {
     meter1cntIncr = meter1cntLstTime-meter1cntThisTime;
  }  
  meter1cntLstTime = meter1cntThisTime;
 Serial.print(meter1cntIncr, DEC); 
//------------------------------------------------
  Serial.print(" meter2="); 
  Serial.print((meter2cntIncr/2), DEC); 
  //chkRounding(meter2cntIncr); 
  Serial.print("/"); 

  if (meter2cntThisTime >= meter2cntLstTime) {
     meter2cntIncr = meter2cntThisTime -meter2cntLstTime;
  } else {
     meter2cntIncr = meter2cntLstTime-meter2cntThisTime;
  }  
  meter2cntLstTime = meter2cntThisTime;
 Serial.print(meter2cntIncr, DEC); 

//------------------------------------------------
  Serial.print(" meter3="); 
  Serial.print((meter3cntIncr/2), DEC);
  Serial.print("/"); 
  
  if (meter3cntThisTime >= meter3cntLstTime) {
     meter3cntIncr = meter3cntThisTime -meter3cntLstTime;
  } else {
     meter3cntIncr = meter3cntLstTime-meter3cntThisTime;
  }  
  meter3cntLstTime = meter3cntThisTime;
  Serial.print(meter3cntIncr, DEC);


//------------------------------------------------
  Serial.print(" ["); 
  Serial.print(meter1cntThisTime, DEC); 
  Serial.print(",");
  Serial.print(meter2cntThisTime, DEC); 
  Serial.print(",");
  Serial.print(meter3cntThisTime, DEC); 
  Serial.print("]");

 }////pollPulseCollection

void doStatePrint() {
    Serial.print(" state{");
    doStatePrintClk(PIN_TC0_TCLK0_ARD);
    doStatePrintClk(PIN_TC0_TCLK1_ARD);
    doStatePrintClk(PIN_TC0_TCLK2_ARD);
    Serial.print("}");
}////

void doStatePrintClk(uint32_t pinArd) {
   if (HIGH == digitalRead(pinArd)) {
       Serial.print("H");
   } else {
       Serial.print("L");
   }
 }////doStatePrintClk
 
/**
 * \brief loop Output Pulse Test 
 *  Manual test to verify the hardware pin the signal is output on
 */
void loopOutPulseTest() {
  #define PIN_TC0_TCLKX_ARD PIN_TC0_TCLK2_ARD
     pinMode(PIN_TC0_TCLKX_ARD,OUTPUT);
    //pinMode(PIN_TC0_TCLK2_ARD,OUTPUT); 
    
  
  digitalWrite(PIN_TC0_TCLKX_ARD,1);
  delay(10);
  digitalWrite(PIN_TC0_TCLKX_ARD,0);
  delay(10);
}////loopOutPulseTest
