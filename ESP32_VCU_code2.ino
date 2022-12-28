/*
OPERATORS
and             &&
and_eq          &=
bitand          &
bitor           |
not             !
not_eq          !=
or              ||
or_eq           |=
xor             ^
xor_eq          ^=

PWM
https://www.baldengineer.com/millis-ind-on-off-times.html

Mitsubishi Outlander Charger CAN 0377 05 87 00 53 30 30 30 22
0x377h 8bytes DC-DC converter status
  B0+B1 = 12V Battery voltage (h04E2=12,50V -> 0,01V/bit)
  B2+B3 = 12V Supply current  (H53=8,3A -> 0,1A/bit)
  B4 =  Temperature 1   (starts at -40degC, +1degC/bit)
  B5 =  Temperature 2   (starts at -40degC, +1degC/bit)
  B6 =  Temperature 3   (starts at -40degC, +1degC/bit)
  B7 =  Statusbyte    (h20=standby, h21=error, h22=in operation)
    bit0(LSB) = Error 
`   bit1    = In Operation
    bit3      =
    bit4      =
    bit5      = Ready
    bit6    = 
    bit7(MSB) = 
  
  
0x389h 8bytes Charger status  0389 A0 E2 5A 30 30 02 91 5A
  B0 = HV-Batteryvoltage    (hAB=342V -> 2V/bit)
  B1 = AC-Mains voltage     (hE2=226V -> 1V/bit)
  B2 = DC-charge current (1?) (h5B=9,1A -> 0,1A/bit
  B3 = Temperature 1    (starts at -40degC, +1degC/bit)
  B4 = Temperature 2    (starts at -40degC, +1degC/bit)
  B5 = Statusbyte     (CA while charging)
    bit0(LSB) = 
`   bit1    = Mains voltage present
    bit2      = 
    bit3      = Charging
    bit4      = Error (no CAN messages received)
    bit5      =
    bit6    = DC-DC converter request active
    bit7(MSB) = 1KHz pilot present    
  B6 = AC-Mains current     (h91=14,5A -> 0,1A/bit)
  B7 = DC-charge current (2?) (same as B3, sometimes differs by 0,1A)


0x38Ah 8bytes Charger status
  B0 = Temperature    Raises together with other temperatures, but with +5deg offset
  B1 = Temperature    (starts at -40degC, +1degC/bit)
  B2 = DC-bus voltage     (9B=310V -> 2V/bit)
  B3 = PWM-signal from EVSE   (h19=25% -> 1%/bit)
  B4 = Status
    bit0(LSB) = 
`   bit1    =
    bit2    = Wait for mains voltage 
    bit3      = Ready for charging
    bit4      =
    bit5      =
    bit6    = 
    bit7(MSB) = 
  B5 = 00
  B6 = 00
  B7 = 00

SEND
0x285 B3  = hB6 enables charging, other bytes can stay 00
0x286 B1+B2   = voltage setpoint    (0E74=370.0V, 0,1V/bit) -> How to enable CV mode ??
      B3  = current setpoint DC-side  (78=12A -> 0,1A/bit)

ID 0x388 at 100ms
b0 status, 7C start, 02 no power, 01 power present.
b1 00
b2 RPM report high byte ((ACrpm1 * 256) + ACrpm2)
b3 RPM report low byte
b4 seems to be amp flow [current=dec value/10]
b5 00
b6 00
b7 CAN status 00-No CAN, 01-CAN error, 02-heartbeat 0x285 present.

Eltek charger status report
ID 0x3X5 8bytes
  B0 = Charger status    (1-idle, 2-charge, 3-error, 4-errorx)
  B1 = AC-Mains current LSB (hex/10 A)
  B2 = AC-Mains current HSB (hex/10 A)
  B3 = DC-charge current LSB (hex/10 A)
  B4 = DC-charge current HSB (hex/10 A)
  B5 = DC-charge voltage LSB (hex/10 V)
  B6 = DC-charge voltage HSB (hex/10 V)
  B7 = AC-Mains freq (hex hz)

ID 0x3X6 8bytes
  B0 = Charger primary temp (Hex °C)
  B1 = Charger secondary temp (Hex °C)
  B2 = AC-Mains voltage LSB (Hex V)
  B3 = AC-Mains voltage HSB (Hex V)
  B4 = Max Power LSB (hex W)
  B5 = Max Power HSB (hex W)
  B6 = Available Power (hex/2 %)
  B7 = 00


Input pins
PP_pin
CP_pin
Enable_pin
BMS_pin
Heater_pin
AC_pin
button
GPIO_NUM_5 // CAN tx


Output pins
CP_relay
DCDC_active
pwm.pin
Disable_pin
LED1_pin
LED2_pin
LED3_pin
BUZZER_PIN
GPIO_NUM_4 // CAN rx


Functions
initializeEEPROMPage() // innitialize EEPROM variables
checkButton() // check for button pressed
Chargertemp() // charger temperature reaction
chargertempctrl() // charger temperature control
PWMsignalDCDC() // signal for DCDC to start or stop
PWMsignalDCDCoff() //  signal for DCDC to fully stop
sendCANframeX() // send different CAN frames
sendCANframeURGENT() // send CAN frame urgent
sendCANfremeBMSX () // send CAN frame for BMS command
listenCANframeBMS ()
Rotate BMS CAN frame // case for rotating different BMS commands
serialEventRun() //serial interrupt
buzzer() // buzzer signal function
checkforinput() // Checks for input from Serial Port 1
getInterval() // Enter the interval in ms between each CAN frame transmission 
getRate() // Enter the Data Rate in Kbps you want for CAN
initializeCAN() // Initialize CAN bus 0 or 1 and set filters to capture incoming CAN frames and route to interrupt service routines in our program.
printMenu() // print selectable menu
printstatus() // This function prints an ASCII statement out the SerialUSB port summarizing various data from program variables.  Typically, these variables are updated by received CAN messages that operate from interrupt routines. This routine also time stamps the moment at which it prints out.

https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/can.html

Web serial
https://randomnerdtutorials.com/esp32-webserial-library/

*/

#include <Arduino.h>
#include <esp32_can.h>
#include <ESP32_PWM.h>
#include <esp_task_wdt.h> //watchdog header

//3 seconds WDT https://iotassistant.io/esp32/enable-hardware-watchdog-timer-esp32-arduino-ide/
#define WDT_TIMEOUT 3

// define the number of bytes you want to access
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;

hw_timer_t * timer = NULL; 
volatile bool interrupt = false;

/*Interrupt routine for Timer overflow event*/
void IRAM_ATTR onTimer() {
   interrupt = true; //Indicates that the interrupt has been entered since the last time its value was changed to false 
}

// Input pins
int PP_pin = 35; // PP input connected to pin 34
int CP_pin = 23; // CP signal PWM sense at pin 23
int Enable_pin = 36; // +12V Enable input connected to pin 36
int Heater_pin = 33; // heater input connected to pin 33
int AC_pin = 14; // AC input connected to pin 14

// Digital Output pins
int DCDC_active = 4; // relay output connected to pin 4
int CP_relay = 19; // relay output connected to pin 19
int Disable_pin = 2; // Output for Disable pump relay 4 
int Buzzer_pin = 13; // Buzzer pin

//PWM output pins
int PWM3_pin = 22;
int PWM2_pin = 5;
int PWM1_pin = 25;

//Power pins
int Out1_pin = 26;
int Out2_pin = 27;
int Out3_pin = 12;

//Analog pins
int Throttle1_pin = 39;
//int Throttle2_pin = 36; 


//*********GENERAL VARIABLE   DATA ******************

//Other ordinary variables
float Version=1.0;
uint16_t page=300;    //EEPROM page to hold variable data.  We save time by setting up a structure and saving a large block
int i;
unsigned long elapsedtime,time228,timestamp,startime,last,interval,intervalm1,intervalm2;//Variables to compare millis for timers
const long interval1 = 100;
const long interval2 = 500;
const unsigned int ontime = 120000; //2min for on time
boolean debug=false;
uint8_t logcycle=0;
uint8_t voltage = 0;
uint16_t transmitime;
uint16_t alarmtime1;
uint16_t alarmtime2;
byte buzzCount = 0;
byte state = 0;
uint8_t Ctemp = 0;
uint8_t aux1 = 0;
uint8_t aux2 = 0;
uint8_t auxc1 = 0;
uint8_t auxc2 = 0;
uint8_t DCDCstatus = 0;
float auxvoltage = 0; 
float auxcurrent = 0;
uint8_t Htemp = 0;
uint8_t Heatertemp = 0;
uint8_t ACrpm1 = 0;
uint8_t ACrpm2 = 0;
uint16_t ACrpm = 0;
uint16_t Throttle1_val = 0;
uint16_t Throttle2_val = 0;
uint16_t PWM1DutyCycle;
uint16_t PWM2DutyCycle;
uint16_t PWM3DutyCycle;
static byte frameRotate;

// PWM setup
const int PWM1Freq = 100; /* 100Hz */
const int PWM1Channel = 0;
const int PWM1Resolution = 10;
const int PWM1_MAX_DUTY_CYCLE = (int)(pow(2, PWM1Resolution) - 1);

const int PWM2Freq = 120; /* 120Hz */
const int PWM2Channel = 0;
const int PWM2Resolution = 10;
const int PWM2_MAX_DUTY_CYCLE = (int)(pow(2, PWM2Resolution) - 1);

const int PWM3Freq = 1000; /* 1kHz */
const int PWM3Channel = 2;
const int PWM3Resolution = 13;
const int PWM3_MAX_DUTY_CYCLE = (int)(pow(2, PWM3Resolution) - 1);

// VCU process flags
bool Heater_flag = false; // Flag for heater operation
bool AC_flag = false; // Flag for AC operation
bool PP_flag = false; //Evse connection
bool Enable_flag = false; // car is started
bool chargerV1 = false; //charger voltage threshold is not reached
bool chargerV2 = false; //charger voltage threshold is not reached
bool auxState = false; //create + name flag for aux voltage

//******* END OF GENERAL VARIABLE DATA***********


//*********EEPROM  DATA ******************


//*********EEPROM  DATA ******************


//********************SETUP FUNCTION*******I*********
/*
 * The SETUP function in the Arduino IDE simply lists items that must be performed prior to entering the main program loop.  In this case we initialize Serial 
 * communications via the USB port, set an interrupt timer for urgent outbound frames, zero our other timers, and load our EEPROM configuration data saved during the 
 * previous session.  If no EEPROM data is found, we initialize EEPROM data to default values
 * 
 */
void setup() 
  {
//Timer3.attachInterrupt(sendCANframeURGENT).start(30000); // This sets an interrupt time to send an URGENT CAN frame every 30ms.  The function can be changed and the time can be changed.  
//https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/

    last=startime=time228=timestamp=interval=intervalm1=intervalm2=millis();  //Zero our other timers
    elapsedtime=millis();          
                                                            
Serial.begin(115200);
Serial.println("Initializing ...");
CAN0.begin(500000);
transmitime=500;
alarmtime1=200;
alarmtime2=500;
Serial.println("Ready ...!");  

  timer = timerBegin(0, 80, true);                //Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, &onTimer, true);   //Attach the interrupt to Timer1
  timerAlarmWrite(timer, 30000, true);      //Initialize the timer
  timerAlarmEnable(timer); 
  
// Initialize PWM Channels with Frequency and Resolution 
ledcSetup(PWM1Channel, PWM1Freq, PWM1Resolution);
ledcSetup(PWM2Channel, PWM2Freq, PWM2Resolution);
ledcSetup(PWM3Channel, PWM3Freq, PWM3Resolution);

// Attach the LED PWM Channel to the GPIO Pin 
ledcAttachPin(PWM1_pin, PWM1Channel);
ledcAttachPin(PWM2_pin, PWM2Channel);
//ledcAttachPin(PWM3_pin, PWM3Channel);
     
 //Load/validate EEPROM variable

  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
 
 //Print welcome screen and menu  
    Serial<<"\n\n Startup successful. VCU started "<<Version<<"\n\n";
 //   printMenu(); 

pinMode(PP_pin,INPUT_PULLUP); // set PP pin to input with using built in pull up resistor
pinMode(CP_pin,INPUT); // sensing CP duty
pinMode(Enable_pin,INPUT); // 
pinMode(Heater_pin,INPUT); // set Heater pin to input with using built in pull up resistor
pinMode(AC_pin,INPUT); // set AC compressor pin to input with using built in pull up resistor
pinMode(CP_relay,OUTPUT); // 
pinMode(DCDC_active,OUTPUT); 
pinMode(Disable_pin,OUTPUT); 
pinMode(Buzzer_pin, LOW);
pinMode(Throttle1_pin,INPUT); // Sense pin 12V
//pinMode(Throttle2_pin,INPUT); // Sense pin 
//pinMode(PWM1_pin, OUTPUT);
//pinMode(PWM2_pin, OUTPUT);
//pinMode(PWM3_pin, OUTPUT);
pinMode(Out1_pin, OUTPUT);
pinMode(Out2_pin, OUTPUT);
pinMode(Out3_pin, OUTPUT);

CAN0.watchFor(0x100, 0xF00); //setup a special filter
CAN0.watchFor(); //then let everything else through anyway
CAN0.setCallback(0, gotHundred); //callback on that first special filter

}  

int lasti = millis();
//********************END SETUP FUNCTION*******I*********

//******************** CAN ROUTINES ****************************************
//* This section contains CAN routines to send and receive messages over the CAN bus
// *  INITIALIZATION routines set up CAN and are called from program SETUP to establish CAN communications.
// *  These initialization routines allow you to set filters and interrupts.  On RECEIPT of a CAN frame, an interrupt stops execution of the main program and 
// *  sends the frame to the specific routine used to process that frame by Message ID. Once processed, the main program is resumed.
// *  
// *  Frames can also be sent, either from the main control loop, or by a Timer interrupt allowing specific frames to be sent on a regular interrupt interval.
// *  
// *  For example a frame that MUST go out every 10 ms would use the Timer Interrupt in SETUP to cause a CAN function here to send a frame every 10 ms.  Other frames 
// *  could be sent normally from the main program loop or conditionally based on some received or calculated data.
// *  

void printFrame(CAN_FRAME *message)
{
  Serial.print(message->id, HEX);
  if (message->extended) Serial.print(" X ");
  else Serial.print(" S ");   
  Serial.print(message->length, DEC);
  for (int i = 0; i < message->length; i++) {
    Serial.print(message->data.byte[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void gotHundred(CAN_FRAME *frame)
{
  Serial.print("Got special frame!  ");
  printFrame(frame);
}
void sendCANframeA() { // Tesla DCDC 14V signal 0x3D8 MSG DD 06 00
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x3D8; 
    txFrame.extended = false;
    txFrame.length = 3; // Data payload 8 bytes
    txFrame.data.uint8[0] = 0xDD;
    txFrame.data.uint8[1] = 0x06;
    txFrame.data.uint8[2] = 0x00;  
    CAN0.sendFrame(txFrame);
}

void sendCANframeB() { // Outlander charger
 CAN_FRAME txFrame;
    txFrame.rtr = 0; // Extended addresses - 0=11-bit 1=29bit
    txFrame.id = 0x286; // Set our transmission address ID
    txFrame.extended = false;
    txFrame.length = 8; // Data payload 8 bytes 0x286 0C 80 78 37 00 00 0A 00
    txFrame.data.uint8[0] = 0x80;
    txFrame.data.uint8[1] = 0x0C; // 0F3C=3900, 0DDE=3550, 0,1V/bit
    if(chargerV1 == true) { // if Charger senses less than 320V
    txFrame.data.uint8[2] = 0x1E; // 78=120 12A, 50=80 8A, 32=50 5A, 1E=30, 3A 14=20 2A at 0,1A/bit
    }
    else if(chargerV2 == true) { // if Charger senses less than 322V
    txFrame.data.uint8[2] = 0x00; // 78=120 12A, 50=80 8A, 32=50 5A, 1E=30, 3A 14=20 2A at 0,1A/bit
    }
    else {  //any other case
    txFrame.data.uint8[2] = 0x78; // 78=120 12A, 50=80 8A, 32=50 5A, 1E=30, 3A 14=20 2A at 0,1A/bit 
    }   
    txFrame.data.uint8[3] = 0x37; // why 37?
    txFrame.data.uint8[4] = 0x00;
    txFrame.data.uint8[5] = 0x00;
    txFrame.data.uint8[6] = 0x0A;
    txFrame.data.uint8[7] = 0x00;
    CAN0.sendFrame(txFrame);
}

void sendCANframeC() {
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x320; 
    txFrame.extended = false;
    txFrame.length = 8; // Data payload 8 bytes
    txFrame.data.uint8[0] = 0x01;
    txFrame.data.uint8[1] = 0xE8;
    txFrame.data.uint8[2] = 0x03;  
    txFrame.data.uint8[3] = 0x54; // 418V
    txFrame.data.uint8[4] = 0x10;
    txFrame.data.uint8[5] = 0x78; // 78=12A -> 0,1A/bit
    txFrame.data.uint8[6] = 0x00;
    txFrame.data.uint8[7] = 0x00;
    CAN0.sendFrame(txFrame);
}

void sendCANframeD() { // Heater
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x188; // // 0x188 03 50 A2 40 00 00 00 00 
    txFrame.extended = false;
    txFrame.length = 8; // Data payload 8 bytes
        if(Heater_flag == true) { // if heater pin is ON   
    txFrame.data.uint8[0] = 0x03;
        }   
        else {
    txFrame.data.uint8[0] = 0x00;             
        }
    txFrame.data.uint8[1] = 0x50;
    if(Heatertemp > 55) { // if temp is higher than 55deg  
    txFrame.data.uint8[2] = 0x00; // when 55deg power goes to 0A
        } 
        else if(Heatertemp > 50) { // if temp is higher than 50deg
    txFrame.data.uint8[2] = 0x32; // seems to be current command (dec/10)
        }         
        else {
    txFrame.data.uint8[2] = 0xA2; 
        }
    txFrame.data.uint8[3] = 0x40;
    txFrame.data.uint8[4] = 0x00;
    txFrame.data.uint8[5] = 0x00;         
    txFrame.data.uint8[6] = 0x00;
    txFrame.data.uint8[7] = 0x0;
    CAN0.sendFrame(txFrame);
}

void sendCANframeE() { //AC compressor
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x185; // 0x185 0B 00 1D 00 00 08 00 03 
    txFrame.extended = false;
    txFrame.length = 8; // Data payload 8 bytes
    if (AC_flag == true) { //AC pin is active   
    txFrame.data.uint8[0] = 0x0B;
        }   
        else {
    txFrame.data.uint8[0] = 0x08;              
        }
    txFrame.data.uint8[1] = 0x00;
    txFrame.data.uint8[2] = 0x1D;    
    txFrame.data.uint8[3] = 0x00;
    txFrame.data.uint8[4] = 0x00;
       if(AC_flag == true) { // AC is ON    
        if (ACrpm <= 4000) { // Full power for spinup 
    txFrame.data.uint8[5] = 0x35; // RPM command 08 to 54
        }
        else if ((ACrpm > 3800) && (ACrpm <= 4500)) { // power is reduced 
    txFrame.data.uint8[5] = 0x25; // RPM command 08 to 54        
        } 
        else if (ACrpm > 4800) { // power is reduced 
    txFrame.data.uint8[5] = 0x20; // RPM command 08 to 54        
        }
        }
        else {
    txFrame.data.uint8[5]=0x00; //AC pin is inactive
        }            
    txFrame.data.uint8[6] = 0x00;
    txFrame.data.uint8[7] = 0x03;
    CAN0.sendFrame(txFrame);
}

void sendCANframeURGENT() { // 0x285 00,00,14,39,91,FE,0C,10 Driving
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x285; 
    txFrame.extended = false;
    txFrame.length = 8; // Data payload 8 bytes
    txFrame.data.uint8[0] = 0x00;
    txFrame.data.uint8[1] = 0x00;
    if(PP_flag == false) { // we are driving, no EVSE connected
    txFrame.data.uint8[2] = 0x14; 
        }   
        else {
    txFrame.data.uint8[2] = 0xB6; //if we sense PP signal we are charging
        }      
    txFrame.data.uint8[3] = 0x39;
    txFrame.data.uint8[4] = 0x91;
    txFrame.data.uint8[5] = 0xFE;
    txFrame.data.uint8[6] = 0x0E;
    txFrame.data.uint8[7] = 0x10;
    CAN0.sendFrame(txFrame);
}

void sendCANframeDCDC() { // Tesla DCDC CAN ID 0x3D8 MSG DD 06 00 
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x3D8;
    txFrame.extended = false;
    txFrame.length = 3; // Data payload 8 bytes
    txFrame.data.uint8[0] = 0xDD;
    txFrame.data.uint8[1] = 0x06;
    txFrame.data.uint8[2] = 0x00;                 
    CAN0.sendFrame(txFrame);
}


//******************** END CAN ROUTINES ****************

void DCDCauxcharge()
{ 
if ((auxState == true) && (digitalRead(Enable_pin) == LOW)) { // auxvoltage went below 12.6V
digitalWrite(DCDC_active,HIGH);   
if (millis() - elapsedtime >= ontime) { // if aux voltage is low and for 5min 
digitalWrite(DCDC_active,LOW); // turn off DCDC_active relay 
elapsedtime=millis();
auxState = false;
}}
else { // if auxvoltage is OK 
auxState = false; // turn off DCDC_active relay
}
}

void Pump()
{
if ((PP_flag == true) || (Enable_flag == true)) // If DCDC or charger is in operation 
{
digitalWrite(Out1_pin,HIGH); // turn on Pump relay 
digitalWrite(DCDC_active,HIGH); // turn on DCDC_active relay 
} 
else { 
digitalWrite(Out1_pin,LOW); // turn on Pump relay    
digitalWrite(DCDC_active,LOW); // turn off DCDC_active relay
}}

//********************MAIN PROGRAM LOOP*******I*********
/*
 * This is the main program loop. Lacking interrupts, this loop is simply repeated endlessly.  Show is an example of code that only executes after a certain
 * amount of time has passed. myVars.transmitime.  It also uses a counter to perform a very low priority function every so many loops.
 * Any other entries will be performed with each loop and on Arduino Due, this can be very fast.
 */

 

void loop()
{ 
  // resetting WDT every 2s, programmed to run indefinitely or untill stuck
  if (millis() - lasti >= 2000 ) {
      Serial.println("Resetting WDT...");
      esp_task_wdt_reset();
      lasti = millis();
  }
  
if(interrupt)
{
  sendCANframeURGENT();  
      interrupt = false;
}

 CAN_FRAME incoming; // reading incoming traffic
    Can0.read(incoming); 
 if(incoming.id==0x398){
      Htemp = incoming.data.bytes[3] ;
      Heatertemp = Htemp-40;
    }
 if(incoming.id==0x389){
      voltage=incoming.data.bytes[0] ;
      Ctemp=incoming.data.bytes[4] ;      
    } 
 if(incoming.id==0x377){
      aux1=incoming.data.bytes[0] ;
      aux2=incoming.data.bytes[1] ;   
      auxc1=incoming.data.bytes[2] ;
      auxc2=incoming.data.bytes[3] ;
      DCDCstatus=incoming.data.bytes[7] ;
      auxvoltage=((aux1 * 256) + aux2); //recalculate two bit voltage value
      auxcurrent=((auxc1 * 256) + auxc2); //recalculate two bit current value      
    }    
if(incoming.id==0x388){
      ACrpm1 = incoming.data.bytes[2] ;
      ACrpm2 = incoming.data.bytes[3] ;      
      ACrpm = ((ACrpm1 * 256) + ACrpm2); //recalculate two bit voltage value
    Serial.println(ACrpm);
    }   


if(digitalRead(Heater_pin) == LOW) { // if heater pin is ON
Serial.println("Heater ON");  
Heater_flag = true;
}
else { // if heater pin is OFF 
Heater_flag = false;
}

if(digitalRead(AC_pin) == LOW) { //AC is ON
Serial.println("AC running");    
AC_flag = true;  
}
else { // if heater pin is OFF 
AC_flag = false;
}

if(digitalRead(PP_pin) == LOW) { // if PP_pin senses EVSE
PP_flag = true;
Serial.println("PP pin "); 
  if(millis()-last > transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
    {   
     last=millis();        //Zero our timer
       sendCANframeB();          
       }         
digitalWrite(CP_relay,HIGH); // turn on Cp_relay
digitalWrite(Disable_pin,HIGH); // turn on relay to disable driving     
}      
             
else {
PP_flag = false;
digitalWrite(CP_relay,LOW ); // turn off Cp_relay
digitalWrite(Disable_pin,LOW); // turn off relay to disable driving 
 } 

if(digitalRead(Enable_pin) == HIGH) { // if heater pin is ON
Serial.println("Car is running");    
Enable_flag = true;
}
else { // if heater pin is OFF 
Enable_flag = false;
}

if((voltage >= 160) && (voltage < 162)) { // if Charger senses greater than 320V
Serial.println("HV charger reduce"); 
chargerV1 = true;   
 }
else {
chargerV1 = false;    
} 

if(voltage >= 162) { // if Charger senses less than or equal 322V
Serial.println("HV charger stop ");  
chargerV2 = true;   
 }
else {
chargerV2 = false;    
} 

if (auxvoltage < 1260) { // if aux voltage is low and DCDC is off
Serial.println("Aux voltage too low");  
auxState = true; // set the flag to true   
elapsedtime=millis();  
}
DCDCauxcharge();
Pump();
  
}
//********************END MAIN PROGRAM LOOP*******I*********






  
 //********************USB SERIAL INPUT FROM KEYBOARD *******I*********
 /* These routines use an automatic interrupt generated by SERIALEVENT to process keyboard input any time a string terminated with carriage return is received 
  *  from an ASCII terminal program via the USB port.  The normal program execution is interrupted and the checkforinput routine is used to examine the incoming
  *  string and act on any recognized text.
  * 
  */

//********************END USB SERIAL INPUT FROM KEYBOARD ****************









//******************** USB SERIAL OUTPUT TO SCREEN ****************
/*  These functions are used to send data out the USB port for display on an ASCII terminal screen.  Menus, received frames, or variable status for example
 *   
 */

//void printMenu()
//{
//  Serial<<"\f\n=========== VCU Controller  Version "<<Version<<" ==============\n************ List of Available Commands ************\n\n";
//  Serial<<"  ? or h  - Print this menu\n";
//  Serial<<"  d - toggles debug DISPLAY FRAMES to print CAN data traffic\n";
//  Serial<<"  i - set interval in ms between CAN frames i550 \n";
//  Serial<<"  k - set data rate in kbps ie k500 \n";
// 
// Serial<<"**************************************************************\n==============================================================\n\n"; 
//}

int milliseconds(void)
{
  int milliseconds = (int) (micros()/100) %10000 ;
  return milliseconds;
}


 int seconds(void)
{
    int seconds = (int) (micros() / 1000000) % 60 ;
    return seconds;
}


int minutes(void)
{
    int minutes = (int) ((micros() / (1000000*60)) % 60);
    return minutes;
}

    
int hours(void)
{    
    int hours = (int) ((micros() / (1000000*60*60)) % 24);
    return hours;
}  



//******************** END USB SERIAL OUTPUT TO SCREEN ****************
