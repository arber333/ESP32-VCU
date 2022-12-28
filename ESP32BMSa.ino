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



Input pins
PP_pin
CP_pin
Enable_pin
BMS_pin
Heater_pin
AC_pin
button

Output pins
CP_relay
DCDC_active
PWM_pin
Cooling_pin
OUT1_pin
OUT2_pin
OUT3_pin
BUZZER_PIN




Functions
initializeEEPROMPage() // innitialize EEPROM variables
checkButton() // check for button pressed
PWMsignalDCDC() // signal for DCDC to start or stop
PWMsignalDCDCoff() //  signal for DCDC to fully stop
sendCANframe() // send different CAN frames
sendCANframeURGENT() // send CAN frame urgent
sendBMSquerry () // send CAN frame for BMS command
SendBMSbalancingON () // Turn ON/OFF balancing
SendBMSbalancingON () // Turn ON/OFF balancing 
listenCANframeBMS ()
Rotate BMS CAN frame // case for rotating different BMS commands
serialEventRun() //serial interrupt
buzzer() // buzzer signal function
checkforinput() // Checks for input from Serial Port 1
getInterval() // Enter the interval in ms between each CAN frame transmission 
initializeCAN() // Initialize CAN bus 0 or 1 and set filters to capture incoming CAN frames and route to interrupt service routines in our program.
printMenu() // print selectable menu
printstatus() // This function prints an ASCII statement out the SerialUSB port summarizing various data from program variables.  Typically, these variables are updated by received CAN messages that operate from interrupt routines. This routine also time stamps the moment at which it prints out.

https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/can.html
*/

#include <Arduino.h>
#include <esp32_can.h>
#include <ESP32_PWM.h>
#include <esp_task_wdt.h> //watchdog header


//3 seconds WDT
#define WDT_TIMEOUT 10
//#define WDT_KEY (0xA5)

// define the number of bytes you want to access
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Sets up serial streaming Serial<<someshit;

hw_timer_t * timer = NULL; 
volatile bool interrupt = false;

/*Interrupt routine for Timer overflow event*/
void IRAM_ATTR onTimer() {
   interrupt = true; //Indicates that the interrupt has been entered since the last time its value was changed to false 
}

// Input pins
int PP_pin = 34; // PP input connected to pin 35
int Enable_pin = 36; // Enable input connected to pin 36, Throttle2 pin
int Balval_pin = 39; // Enable input connected to pin 39, Throttle1 pin

// Output pins
int BMS_pin = 2; // BMS output connected to pin 2
int Cooling_pin = 4; // Output for BMS cooling 4 
int Buzzer_pin = 13; // buzzer pin
int BuzzerState = LOW;

//Power pins
int Out1_pin = 26;
int Out2_pin = 27;
int Out3_pin = 12;

//*********GENERAL VARIABLE   DATA ******************

//Other ordinary variables
float Version=1.0;
uint16_t page=300;    //EEPROM page to hold variable data.  We save time by setting up a structure and saving a large block
int i = 0;
unsigned long elapsedtime,time228,timestamp,last,startime,interval,intervalm1,intervalm2;//Variables to compare millis for timers
const long interval1 = 100
const long interval2 = 500
boolean debug=false;
uint8_t logcycle=0;
uint16_t voltage = 0;
uint16_t transmitime;
uint16_t alarmtime1;
uint16_t alarmtime2;
byte buzzCount = 0;
byte state = 0;

//...................first
uint16_t temp1 = 0;
uint8_t avgv1_h = 0;
uint8_t avgv1_l = 0;
uint16_t avgv1 = 0;
uint8_t cell1_h = 0;
uint8_t cell1_l = 0;
uint8_t alarm1 = 0;
uint8_t maxdiff1_h = 0;
uint8_t maxdiff1_l = 0;
uint16_t maxdiff1 = 0;
uint8_t balval1 = 0;
//.....................second
uint16_t temp2 = 0;
uint8_t avgv2_h = 0;
uint8_t avgv2_l = 0;
uint16_t avgv2 = 0;
uint8_t cell2_h = 0;
uint8_t cell2_l = 0;
uint8_t alarm2 = 0;
uint8_t maxdiff2_h = 0;
uint8_t maxdiff2_l = 0;
uint16_t maxdiff2 = 0;
uint8_t balval2 = 0;
//.....................third
uint16_t temp3 = 0;
uint8_t avgv3_h = 0;
uint8_t avgv3_l = 0;
uint16_t avgv3 = 0;
uint8_t cell3_h = 0;
uint8_t cell3_l = 0;
uint8_t alarm3 = 0;
uint8_t maxdiff3_h = 0;
uint8_t maxdiff3_l = 0;
uint16_t maxdiff3 = 0;
uint8_t balval3 = 0;
//.....................fourth
uint16_t temp4 = 0;
uint8_t avgv4_h = 0;
uint8_t avgv4_l = 0;
uint16_t avgv4 = 0;
uint8_t cell4_h = 0;
uint8_t cell4_l = 0;
uint8_t alarm4 = 0;
uint8_t maxdiff4_h = 0;
uint8_t maxdiff4_l = 0;
uint16_t maxdiff4 = 0;
uint8_t balval4 = 0;
//.....................fifth
uint16_t temp5 = 0;
uint8_t avgv5_h = 0;
uint8_t avgv5_l = 0;
uint16_t avgv5 = 0;
uint8_t cell5_h = 0;
uint8_t cell5_l = 0;
uint8_t alarm5 = 0;
uint8_t maxdiff5_h = 0;
uint8_t maxdiff5_l = 0;
uint16_t maxdiff5 = 0;
uint8_t balval5 = 0;
//.....................

uint16_t Throttle1_val = 0;
uint16_t Throttle2_val = 0;
static byte frameRotate;

int CellV1[24]; //setup an array for N cells
int CellV2[24]; //setup an array for N cells
int CellV3[24]; //setup an array for N cells
int CellV4[24]; //setup an array for N cells
int CellV5[24]; //setup an array for N cells
int CellV[96]; //setup complete array for all cells
int Cellcount1=24;
int CellAvg[5] = {avgv1, avgv2, avgv3, avgv4, avgv5};
int v; //voltage value index  

// BMS process flags
bool BMS_DIFF = false; // Cell difference is too high,
bool BMS_VOL = false; // One of the cells is over 3.90V,
bool BMS_BAL = false; // start balancing !
bool BALVAL = false;// balancing state!
bool BMS_OV = false; // one of the cells is too high
bool BMS_UV = false; // one of the cells is too low
bool BMS_T = false; // cell temp too high, colling active
bool BMS_t = false; // cell temp too low, heating active 
bool BMS_0 = false; // signal is not 0

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

// Timer3.attachInterrupt(request).start(500000); // This sets an interrupt to send BMS CAN request every 500ms.  The function can be changed and the time can be changed.  
// https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
// https://circuitdigest.com/microcontroller-projects/esp32-timers-and-timer-interrupts
// https://github.com/khoih-prog/ESP32TimerInterrupt
// https://iotespresso.com/timer-interrupts-with-esp32/

    last=startime=time228=timestamp=interval=intervalm1=intervalm2=millis();  //Zero our other timers
    elapsedtime=millis();     
                                                            
Serial.begin(115200);
Serial.println("Initializing ...");
CAN0.begin(250000);
transmitime=100;
alarmtime1=100;
alarmtime2=500;
Serial.println("Ready ...!");  

  timer = timerBegin(0, 80, true);                //Begin timer with 1 MHz frequency (80MHz/80)
  timerAttachInterrupt(timer, &onTimer, true);   //Attach the interrupt to Timer1
  timerAlarmWrite(timer, 500000, true);      //Initialize the timer
  timerAlarmEnable(timer); 
    
 //Load/validate EEPROM variable

  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
 
 //Print welcome screen and menu  
    Serial<<"\n\n Startup successful. BMS started "<<Version<<"\n\n";
 //   printMenu(); 

pinMode(PP_pin,INPUT); // set PP pin to input with using built in pull up resistor
pinMode(Enable_pin,INPUT); // Balval_pin
pinMode(Balval_pin,INPUT);
pinMode(BMS_pin, OUTPUT); 
pinMode(Cooling_pin, OUTPUT); 
pinMode(Buzzer_pin, OUTPUT);
pinMode(Out1_pin, OUTPUT);
pinMode(Out2_pin, OUTPUT);
pinMode(Out3_pin, OUTPUT);

digitalWrite(Buzzer_pin, LOW)


CAN0.watchFor(0x01, 0x05); //setup a special filter
CAN0.watchFor(); //then let everything else through anyway
//CAN0.setCallback(0, gotHundred); //callback on that first special filter

}

int lasti = millis();
//********************END SETUP FUNCTION*******I*********

//********************ALARMS****************************

//void buzzeralarm() {
//if(currentMillis - intervalm1 >= interval1) {
//  intervalm1 = currentMillis;
//  digitalWrite(Buzzer_pin, LOW);
//}
//
//if(currentMillis - intervalm2 >= interval2) {
//  intervalm1 = currentMillis;
//  digitalWrite(Buzzer_pin, HIGH);
//}  
//}


void buzzeralarm_1() {
switch (state)
  {
    case 0:  //buzzer short 2 times
  if(millis()-elapsedtime > alarmtime1)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
      {
  digitalWrite(Buzzer_pin, !digitalRead(Buzzer_pin)); // turn on buzzer
     elapsedtime = millis();
        buzzCount++;
        if (buzzCount == 4)
        {
          state = 1;
          buzzCount = 0;
        }
      }
      break;
  
    case 1:  //buzzer long 1 times
  if(millis()-elapsedtime > alarmtime2)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
      {
  digitalWrite(Buzzer_pin, !digitalRead(Buzzer_pin)); // turn on buzzer
     elapsedtime = millis();
        buzzCount++;
        if (buzzCount == 2)
        {
          state = 2;
 digitalWrite(Buzzer_pin,HIGH);
        }
      }
      break;
 }
 }

void buzzeralarm_2() {
switch (state)
  {
    case 0:  //buzzer short 3 times
  if(millis()-elapsedtime > alarmtime1)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
      {
  digitalWrite(Buzzer_pin, !digitalRead(Buzzer_pin)); // turn on buzzer
     elapsedtime = millis();
        buzzCount++;
        if (buzzCount == 6)
        {
          state = 1;
          buzzCount = 0;
        }
      }
      break;
  
    case 1:  //buzzer long 1 times
  if(millis()-elapsedtime > alarmtime2)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
      {
  digitalWrite(Buzzer_pin, !digitalRead(Buzzer_pin)); // turn on buzzer
     elapsedtime = millis();
        buzzCount++;
        if (buzzCount == 2)
        {
          state = 2;
 digitalWrite(Buzzer_pin,HIGH);
        }
      }
      break;
 }
 }

 void buzzeralarm_3() {
switch (state)
  {
    case 0:  //buzzer short 6 times
  if(millis()-elapsedtime > alarmtime1)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
      {
  digitalWrite(Buzzer_pin, !digitalRead(Buzzer_pin)); // turn on buzzer
     elapsedtime = millis();
        buzzCount++;
        if (buzzCount == 12)
        {
          state = 1;
          buzzCount = 0;
        }
      }
      break;
  
    case 1:  //buzzer long 1 times
  if(millis()-elapsedtime > alarmtime2)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
      {
  digitalWrite(Buzzer_pin, !digitalRead(Buzzer_pin)); // turn on buzzer
     elapsedtime = millis();
        buzzCount++;
        if (buzzCount == 1)
        {
          state = 2;
 digitalWrite(Buzzer_pin,HIGH);
        }
      }
      break;
 }
 }
 
//********************END OF ALARMS****************************


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


void sendBMSquerry_1() // send CAN frame for BMS command 
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x01; // Rotate ID for each report
    txFrame.extended = false;
    txFrame.length = 1; // Data payload 1 byte
    txFrame.data.uint8[0] = 0xFF;
    CAN0.sendFrame(txFrame);
}

void sendBMSquerry_2() // send CAN frame for BMS command 
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x02; // Rotate ID for each report
    txFrame.extended = false;
    txFrame.length = 1; // Data payload 1 byte
    txFrame.data.uint8[0] = 0xFF;
    CAN0.sendFrame(txFrame);
}

void sendBMSquerry_3() // send CAN frame for BMS command 
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x03; // Rotate ID for each report
    txFrame.extended = false;
    txFrame.length = 1; // Data payload 1 byte
    txFrame.data.uint8[0] = 0xFF;
    CAN0.sendFrame(txFrame);
}

void sendBMSquerry_4()// send CAN frame for BMS command 
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x04; // Rotate ID for each report
    txFrame.extended = false;
    txFrame.length = 1; // Data payload 1 byte
    txFrame.data.uint8[0] = 0xFF;
    CAN0.sendFrame(txFrame);
}

void sendBMSquerry_5()// send CAN frame for BMS command 
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x05; // Rotate ID for each report
    txFrame.extended = false;
    txFrame.length = 1; // Data payload 1 byte
    txFrame.data.uint8[0] = 0xFF;
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingON_1() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x01; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x01; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingON_2() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x02; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x01; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingON_3() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x03; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x01; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingON_4() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x04; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x01; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingON_5() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x05; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x01; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingOFF_1() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x01; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x00; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingOFF_2() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x02; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x00; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingOFF_3() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x03; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x00; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingOFF_4() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x04; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x00; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void SendBMSbalancingOFF_5() // Turn ON/OFF balancing when cells are within 3.9V
{
 CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = 0x05; // Rotating frame max no. of modules 
    txFrame.extended = false;
    txFrame.length = 2; // Data payload 2 bytes
    txFrame.data.uint8[0] = 0xF6;
    txFrame.data.uint8[1] = 0x00; // Use BMS balancing flag to switch on/off
    CAN0.sendFrame(txFrame);
}

void requestDATA () 
{
          Serial.println("Sending data");     
          sendBMSquerry_1 ();
          sendBMSquerry_2();
          sendBMSquerry_3();
          sendBMSquerry_4(); 
          sendBMSquerry_5();           
}

void requestBMS_ON () {
if (BALVAL == false) {
Serial.println("Sending balancing command ON");  
     SendBMSbalancingON_1();
     SendBMSbalancingON_2();
     SendBMSbalancingON_3();
     SendBMSbalancingON_4();
     SendBMSbalancingON_5();
BALVAL = true;  
Serial.println("BALVAL is ON");                  
      }
else {
Serial.println("BALVAL is allready ON");       
}}

void requestBMS_OFF () {
 if (BALVAL == true) {
Serial.println("Sending balancing command OFF");  
          SendBMSbalancingOFF_1();
          SendBMSbalancingOFF_2();
          SendBMSbalancingOFF_3();
          SendBMSbalancingOFF_4(); 
          SendBMSbalancingOFF_5();
BALVAL = false;
Serial.println("BALVAL is OFF");                             
      }
else {
Serial.println("BALVAL is allready OFF");   
}}
//******************** END CAN ROUTINES ****************



 //********************USB SERIAL INPUT FROM KEYBOARD *******I*********
 /* These routines use an automatic interrupt generated by SERIALEVENT to process keyboard input any time a string terminated with carriage return is received 
  *  from an ASCII terminal program via the USB port.  The normal program execution is interrupted and the checkforinput routine is used to examine the incoming
  *  string and act on any recognized text.
  * 
  */
   
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


//********************END USB SERIAL INPUT FROM KEYBOARD ****************



//********************MAIN PROGRAM LOOP*******I*********
/*
 * This is the main program loop. Lacking interrupts, this loop is simply repeated endlessly.  Show is an example of code that only executes after a certain
 * amount of time has passed. myVars.transmitime.  It also uses a counter to perform a very low priority function every so many loops.
 * Any other entries will be performed with each loop and on Arduino Due, this can be very fast.
 */


void loop()
{ 
  // resetting WDT every 2s, 5 times only
  if (millis() - lasti >= 3000 && i < 10) {
      Serial.println("Resetting WDT...");
      esp_task_wdt_reset();
      lasti = millis();
      i++;
      if (i == 10) {
        Serial.println("Stopping WDT reset. CPU should reboot in 3s");
      }
  }

if(interrupt)
{
  requestDATA (); 
      interrupt = false;
}
  
 CAN_FRAME incoming; // reading incoming traffic
    Can0.read(incoming); 
    if(incoming.id == 0x01)
    {
    if (incoming.data.bytes[0] == 0x01)
    {
      temp1 = incoming.data.bytes[3] ;
      avgv1_h = incoming.data.bytes[5] ;
      avgv1_l = incoming.data.bytes[6] ;
      avgv1 = ((avgv1_h * 256) + avgv1_l) ; //recalculate two bit voltage value   
    }
    if (incoming.data.bytes[0] == 0x02) 
    {
      cell1_h = incoming.data.bytes[1] ;
      cell1_l = incoming.data.bytes[2] ;      
      alarm1 = incoming.data.bytes[3];  // bit0 charging, bit1 discharge, bit4, not correct cell no, bit5 resistance,  
      maxdiff1_h = incoming.data.bytes[4] ;
      maxdiff1_l = incoming.data.bytes[5] ;  
      maxdiff1 = ((maxdiff1_h * 256) + maxdiff1_l) ; //recalculate two bit voltage value 
     }
    if (incoming.data.bytes[0] == 0x03)
    { 
      balval1 = incoming.data.bytes[5];
    }
    if (incoming.data.bytes[0] == 0x04)
    { 
      CellV1[incoming.data.bytes[1]] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV1[incoming.data.bytes[1]+1] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV1[incoming.data.bytes[1]+2] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
    }       
    }
    if(incoming.id == 0x02){
    if (incoming.data.bytes[0] == 0x01)
    {
      temp2 = incoming.data.bytes[3] ;
      avgv2_h = incoming.data.bytes[5] ;
      avgv2_l = incoming.data.bytes[6] ;
      avgv2 = ((avgv2_h * 256) + avgv2_l) ; //recalculate two bit voltage value    
    }
    if (incoming.data.bytes[0] == 0x02) 
    {
      cell2_h = incoming.data.bytes[1] ;
      cell2_l = incoming.data.bytes[2] ;
      alarm2 = incoming.data.bytes[3];     // bit0 charging, bit1 discharge, bit4, not correct cell no, bit5 resistance,          
      maxdiff2_h = incoming.data.bytes[4] ;
      maxdiff2_l = incoming.data.bytes[5] ;  
      maxdiff2 = ((maxdiff2_h * 256) + maxdiff2_l) ; //recalculate two bit voltage value       
  } 
    if (incoming.data.bytes[0] == 0x03)
    { 
      balval2 = incoming.data.bytes[5];
    }
        if (incoming.data.bytes[0] == 0x04)
    { 
      CellV2[incoming.data.bytes[1]] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV2[incoming.data.bytes[1]+1] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV2[incoming.data.bytes[1]+2] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
    }  
    }
    if(incoming.id == 0x03){
    if (incoming.data.bytes[0] == 0x01)
    {
      temp3 = incoming.data.bytes[3] ;
      avgv3_h = incoming.data.bytes[5] ;
      avgv3_l = incoming.data.bytes[6] ;
      avgv3 = ((avgv3_h * 256) + avgv3_l) ; //recalculate two bit voltage value
    }
    if (incoming.data.bytes[0] = 0x02) 
    {
      cell3_h = incoming.data.bytes[1] ;
      cell3_l = incoming.data.bytes[2] ;
      alarm3 = incoming.data.bytes[3];      // bit0 charging, bit1 discharge, bit4, not correct cell no, bit5 resistance,   
      maxdiff3_h = incoming.data.bytes[4] ;
      maxdiff3_l = incoming.data.bytes[5] ;  
      maxdiff3 = ((maxdiff3_h * 256) + maxdiff3_l) ; //recalculate two bit voltage value       
    } 
    if (incoming.data.bytes[0] == 0x03)
    { 
      balval3 = incoming.data.bytes[5];
    }
        if (incoming.data.bytes[0] == 0x04)
    { 
      CellV3[incoming.data.bytes[1]] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV3[incoming.data.bytes[1]+1] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV3[incoming.data.bytes[1]+2] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
    } 
    }           
    if(incoming.id == 0x04){
    if (incoming.data.bytes[0] == 0x01)
    {
      temp4 = incoming.data.bytes[3] ;
      avgv4_h = incoming.data.bytes[5] ;
      avgv4_l = incoming.data.bytes[6] ;
      avgv4 = ((avgv4_h * 256) + avgv4_l) ; //recalculate two bit voltage value  
    }
    if (incoming.data.bytes[0] == 0x02) 
    {
      cell4_h = incoming.data.bytes[1] ;
      cell4_l = incoming.data.bytes[2] ;     
      alarm4 = incoming.data.bytes[3];       // bit0 charging, bit1 discharge, bit4, not correct cell no, bit5 resistance,  
      maxdiff4_h = incoming.data.bytes[4] ;
      maxdiff4_l = incoming.data.bytes[5] ;  
      maxdiff4 = ((maxdiff4_h * 256) + maxdiff4_l) ; //recalculate two bit voltage value       
  }
    if (incoming.data.bytes[0] == 0x03)
    { 
      balval4 = incoming.data.bytes[5];
    }
        if (incoming.data.bytes[0] == 0x04)
    { 
      CellV4[incoming.data.bytes[1]] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV4[incoming.data.bytes[1]+1] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV4[incoming.data.bytes[1]+2] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
    }  
    } 
    if(incoming.id == 0x05){
    if (incoming.data.bytes[0] == 0x01)
    {
      temp5 = incoming.data.bytes[3] ;
      avgv5_h = incoming.data.bytes[5] ;
      avgv5_l = incoming.data.bytes[6] ;
      avgv5 = ((avgv5_h * 256) + avgv5_l) ; //recalculate two bit voltage value    
    }
    if (incoming.data.bytes[0] == 0x02) 
    {
      cell5_h = incoming.data.bytes[1] ;
      cell5_l = incoming.data.bytes[2] ;      
      alarm5 = incoming.data.bytes[3];      // bit0 charging, bit1 discharge, bit4, not correct cell no, bit5 resistance,  
      maxdiff5_h = incoming.data.bytes[4] ;
      maxdiff5_l = incoming.data.bytes[5] ;  
      maxdiff5 = ((maxdiff5_h * 256) + maxdiff5_l) ; //recalculate two bit voltage value       
    } 
    if (incoming.data.bytes[0] == 0x03)
    { 
      balval5 = incoming.data.bytes[5];
    }
        if (incoming.data.bytes[0] == 0x04)
    { 
      CellV5[incoming.data.bytes[1]] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV5[incoming.data.bytes[1]+1] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
      CellV5[incoming.data.bytes[1]+2] = incoming.data.bytes[1]*256+incoming.data.bytes[3];
    }   
    }



//Balancing flag conditionals
     
if(avgv1 >= 3900 || avgv2 >= 3900 || avgv3 >= 3900 || avgv4 >= 3900 || avgv5 >= 3900) { // conditional to raise voltage flag when any cell group goes higher than 3.95V per cell
  BMS_VOL = true;
}
else  { // conditional to lower voltage flag when any cell group goes lower than 3.85V per cell
  BMS_VOL = false;
  }

if(maxdiff1 > 100 || maxdiff2 > 100 || maxdiff3 > 100 || maxdiff4 > 100 || maxdiff5 > 100) { // conditional to raise difference flag when cell difference is higher than 0.1V
  BMS_DIFF = true;
}
if (maxdiff1 < 50 || maxdiff2 < 50 || maxdiff3 < 50 || maxdiff4 < 50 || maxdiff5 < 50) { // conditional to lower difference flag when cell difference is lower than 0.05V
  BMS_DIFF = false;
}

if (BMS_VOL == true || BMS_DIFF == true) { // if either of flags are true we raise the balancing flag 
 BMS_BAL = true;
}
 
else if (BMS_VOL == false || BMS_DIFF == false) { 
 BMS_BAL = false;  
  }  

      
/*
Balancing is started on condition: 
+ any average cell value is higher than 3.95V, BMS_VOL flag = true
+ average difference between cells is more than 200mV, BMS_DIFF flag = true 
+ car is charging, PP is ON
 
 * once we request start balancing master changes the BALVAL flag to true 
 
Balancing stops on conditions:
+ Car IS driving, Enable is on
+ any average cell value is lower than 3.85V, BMS_VOL flag = false
+ average difference between cells is less than 150mV, BMS_DIFF flag = false 
  
 * once we request stop balancing master changes the BALVAL flag to false 
 
*/

// 
  
//// balancing command performs one time on condition   
//if (BMS_BAL == true) { // if balancing flag is true or cell difference is high 
//  if(millis()-last > transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
//    {   
//     last=millis();        //Zero our timer
//       requestBMS_ON (); 
//Serial.println("BMS balancing ON");
// }}

if(digitalRead(PP_pin) == LOW) { // if PP_pin senses EVSE
  if(millis()-last > transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
    {   
     last=millis();        //Zero our timer
       requestBMS_ON ();
Serial.println("BMS is in charge mode");
    } }      

if(digitalRead(Enable_pin) == HIGH) { // if PP_pin senses EVSE
  if(millis()-last > transmitime)  //Nominally set for 120ms - do stuff on 120 ms non-interrupt clock
    {   
     last=millis();        //Zero our timer
       requestBMS_OFF ();
Serial.println("BMS in drive mode"); 
 } }  



 
 }
//********************END MAIN PROGRAM LOOP*******I*********
