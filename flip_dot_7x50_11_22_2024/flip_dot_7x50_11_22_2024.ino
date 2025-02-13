
// Program for controller that operates Luminator Mark IV Slave display

// Template ID, Device Name and Auth Token are provided by Blynk
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "XXXXXXXX"
#define BLYNK_TEMPLATE_NAME "Split Flap"
#define BLYNK_AUTH_TOKEN "XXXXXXXX"
#define BLYNK_FIRMWARE_VERSION "1.0.1"
char auth[] = BLYNK_AUTH_TOKEN;
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[20] = "XXXXXXXX";
char pass[20] = "XXXXXXXX";
char k_GMT_str[20] = "-7"; //-8 for California ST and -7 for California DST
char activeTime_str[20] = "10";  // 10 minutes for production
char enableStartHours_str[20] = "7";  // start motion function at 7 am
char enableEndHours_str[20] = "20";  // end motion function at 8 pm
char randomPhraseDisplayFlag_str[20] = "1";  // on in production
int randomPhraseDisplayFlag;
int k_GMT;
unsigned long activeTime;
int enableStartHours;
int enableEndHours;
int motionWindowFlag;

char version_str[32] = "V.060724";  // month/day/year

// for eeprom emulation in nano flash memory
// #define FLASH_DEBUG       0  // Use 0-2. Larger for more debugging messages
#include <FlashStorage_SAMD.h>
const int WRITTEN_SIGNATURE = 0xabcdabcd;
// eeprom data for refernece
  //char ssid[20] = "XXX";
  //char pass[20] = "XXX";
  //int k_GMT = -7; //-8 for California ST and -7 for California DST
  //unsigned long activeTime = 2;  // 2 minutes for testing
  //int enableStartHours = 7;  // start motion function at 7 am
  //int enableEndHours = 20; // end motion function at 8 pm
  //int randomPhraseDisplayFlag = 1;  / on in production
typedef struct
{
  char eeprom_ssid[20];
  char eeprom_pass[20]; 
  char eeprom_k_GMT[20];
  char eeprom_enableStartHours[20];
  char eeprom_enableEndHours[20];
  char eeprom_activeTime[20];
  char eeprom_randomPhraseDisplayFlag[20];
} Device;
// define controller as a "Device"
Device controller;
// If the EEPROM is written, then there is a written signature 
// at address 0
uint16_t storedAddress = 0;
int signature;

// for wifi, Blynk and rtc
#include <SPI.h>
#include <WiFiNINA.h> // for mkr 1010
#include <BlynkSimpleWiFiNINA.h>  // needed for Blynk to run

// for real time clock
#include <RTCZero.h>
RTCZero rtc;
//#include "arduino_secrets.h" // for putting the ssid and pwd on a separate tab
//  can enter sensitive data in the Secret tab/arduino_secrets.h
// cont char ssid = ; // network SSID (name)
// const char pass = ; // network password (use for WPA, or use as key for WEP)
//int key_index = 0; // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
unsigned long epoch = 0;

long rssi;
char rssi_str[16];

// for date and time functions
char seconds_str[8];
int seconds_int;
char minutes_str[8];
int minutes_int;
char hours_str[8];
int hours_int;
char days_str[8];
int days_int;
char months_str[8];
int months_int;
char years_str[8];
int years_int;
char date_str[16]; // date string
char time_str[16]; // time string
int prior_days_int = 0;
int secondsDisplayOn = 1; // blinking : - 0 is off, 1 is on

// for error logging and resets
char error_type_str[32];
char error_type_display_str[32];
// set the reset flag on error and clear the reset flag when restarting
int mkr_reset_flag_int = 0;

// for watch dog timer
// Note - for below, must delete lines 58 to 66 of WDTZero.cpp 
// to avoid clock configuation conflict with RTCZero in order to use  
// the external crystal for accuracy, instead of the internal oscillator
// for power saving
#include <WDTZero.h>  
WDTZero WDTimer; // Define watch dog timer

//for Blynk
// BlynkTimer timer; // Do not use BlynkTimer (based on SimpleTimer) as it may 
// have been causing resets
// Attach virtual serial terminal to Virtual Pin V3
WidgetTerminal terminal(V3);
// for reading a second input line in the Blynk terminal
int terminal_second_line_flag_int = 0;

// for manual timers
unsigned long currentMillis;
unsigned long previousMillis_Blynk = 0;
unsigned long previousMillis_blinkOnBoardLED = 0; 
unsigned long previousMillis_mkrReset = 0; 
unsigned long previousMillis_updateTimeDisplay = 0;
//unsigned long previousMillis_updateLeds = 0;
unsigned long previousMillis_displayActive = 0;
unsigned long previousMillis_updateMotion = 0;
unsigned long lastCommandMillis = 0;
unsigned long lastMotionMillis = 0;

// for push buttons and mkr led (if top shield used)
#include <utility/wifi_drv.h>
int value_sw0_int = 0;
int value_prior_sw0_int = 0;
int state_sw0_int = 0;
int state_prior_sw0_int = 0;

int value_sw1_int = 0;
int value_prior_sw1_int = 0;
int state_sw1_int = 0;
int state_prior_sw1_int = 0;

int onBoardLEDValue = 0; // for blinking on board led

// for notifications
char notification_str[256]; 

// for USB Serial commands
char cmd_str[640];
int cmd_length_int = 0;
int cmd_flag_int = 0;
int serial_second_line_flag_int = 0;

// for splitflap display commands --****** update this as much code not needed
int currentDisplayMonths_int; // will cause time to be set
int currentDisplayDays_int;
int currentDisplayHours_int = 999; // will cause time to be set
int currentDisplayMinutes_int = 999;
//char flipCharsDisplay_str[128] = " 1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ-./"; 
// above for reference normal characters
char flipCharsHoursDisplay_str[128] = "U1234567890ABCDEFGHIJKLM";  
// above U is 0 - time characters
char flipCharsMinutesDisplay_str[128] = " 601A2B3C4D5E";  
// above 6 is 0, E is 55 - time characters
char hours_address_str[4] = "91";
char minutes_address_str[4] = "92";
char currentCharacter_str[4] = "?"; // with single character, [1] is always '\0'
int timeDisplayFlag = 1;  // default is time display is on
// command in last 5 minutes or motion on last 5 minutes
int displayActiveFlag = 1; 
int phrase_num_old = 0; // to avoid random repeating of phrases
int blankFlag = 0; // to avoid repeat blanking
int cmdDelay = 0; // set delay before command runs (for video recording)


// for leds
// configure leds
// led state 0-off, 1-on, 2-blinking(1s)
//int redLedPin = A0; // active low (D15)
//int redLedState;
//int greenLedPin = A1;  // active low (D16)
//int greenLedState;
//int blueLedPin = A2;  // active low (D17)
//int blueLedState;
//int switchLedPin = A3;  // active low (D18)
//int switchLedState;

// for display dimensions
int row_length = 10; // using two slave display units - one unit woudl be 5
char input_str[640];
char input_str2[640];
int row_display_number = 0;
char row_display_array[32][20];
char word_str[128][20];  // up to 128 words of 19 characters with '\0'
int word_str_length[40];
int word_cnt = 0;
int row_number = 0;
char display_str[640];
int phrase_num = 0;

// for motion configure motion controls
int motionPin = 8; // D8 active high
int motionState; // state 0-off, 1-on
int prior_motionState;
int motionReportFlag = 1; // default is on

// for cell control 
// analog pins continue numbering after digital pins
int RLDpin = 15; // A0
int RHDpin = 16;
int RSApin = 17;
int RSBpin = 18;
int RSCpin = 19;
int CLDpin = 20;
int CHDpin = 21;  // A6
int CSApin = 0;  // D0
int CSBpin = 1;
int CSCpin = 2;
int CS0pin = 3;
int CS1pin = 4;
int CS2pin = 5;
// Note - D6 is built in LED
int CS3pin = 7;  // D7
int CNTRpin = 9;
int ENDpin = 10;

// for displaying character array
byte fd_array[5][7]; 
// up to 50 characters scrolling with blank column separating 6*50
byte scrolling_fd_array[300][7]; 
int pulseTime = 5; // 0.5ms (5 100us steps) for disk solenoid pulses
int pulsePostDelay = 0; // 0.5ms is 5 * 100us steps) post pulse delay in us
int pulseNumber = 1; // 0 no pulse, 1 normal pulse, 2 double pulse
int scrollDelay = 0; // scroll delay in ms
// for input switches
int SW0pin = 12;  // consider other inputs as 13, 14 rx tx
int SW1pin = 11;

void setup()
{
// immediately configure drivers to off states
// 2801A 3.3 to 12V level shifters invert all of these outputs

// CD4051 Inhibits are active high - level shifter inverts these outputs
pinMode(CS0pin, OUTPUT);  // CS0
digitalWrite(CS0pin, LOW); // turn off at startup
delay(1);
pinMode(CS1pin, OUTPUT); // CS1
digitalWrite(CS1pin, LOW); // turn off at startup
delay(1);
pinMode(CS2pin, OUTPUT);  // CS2
digitalWrite(CS2pin, LOW); // turn off at startup
delay(1);
pinMode(CS3pin, OUTPUT); // CS3
digitalWrite(CS3pin, LOW); // turn off at startup
delay(1);
pinMode(CNTRpin, OUTPUT);  // CNTR
digitalWrite(CNTRpin, LOW); // turn on at startup
delay(1);
pinMode(ENDpin, OUTPUT); // END
digitalWrite(ENDpin, LOW); // turn off at startup
delay(1);

// row driver
pinMode(RLDpin, OUTPUT);  // RLD
digitalWrite(RLDpin, HIGH); // turn off at startup
delay(1);
pinMode(RHDpin, OUTPUT); // RHD
digitalWrite(RHDpin, LOW); // turn off at startup
delay(1);
pinMode(RSApin, OUTPUT);  // RSA  select unused output 7
digitalWrite(RSApin, LOW);
delay(1);
pinMode(RSBpin, OUTPUT);  // RSB
digitalWrite(RSBpin, LOW);
delay(1);
pinMode(RSCpin, OUTPUT);  // RSC
digitalWrite(RSCpin, LOW);
delay(1);

// column driver
pinMode(CLDpin, OUTPUT);  // CLD
digitalWrite(CLDpin, HIGH); // turn off at startup
delay(1);
pinMode(CHDpin, OUTPUT); // CHD
digitalWrite(CHDpin, LOW); // turn off at startup
delay(1);
pinMode(CSApin, OUTPUT);  // CSA  // select 0
digitalWrite(CSApin, HIGH);
delay(1);
pinMode(CSBpin, OUTPUT);  // CSB
digitalWrite(CSBpin, HIGH);
delay(1);
pinMode(CSCpin, OUTPUT);  // CSC
digitalWrite(CSCpin, HIGH);
delay(1);

// for Switches
pinMode(SW0pin, INPUT_PULLUP); // SW0
pinMode(SW1pin, INPUT_PULLUP); // SW1

// blank screen
blankDisplay();

Serial.begin(38400); // for serial monitor
//while (!Serial) {}; // wait for serial port to connect.

// Program would not compile when code below was dropped into a function
// write eeprom data or if reset, obtain eeprom data
// Check signature at address 0
EEPROM.get(storedAddress, signature);
// If the EEPROM is written, then there is a written signature
if (signature == WRITTEN_SIGNATURE){
  EEPROM.get(storedAddress + sizeof(signature), controller);
  /*Serial.println("Returning data previsouly written");
  Serial.println(controller.eeprom_ssid); 
  Serial.println(controller.eeprom_pass); 
  Serial.println(controller.eeprom_k_GMT);
  Serial.println(controller.eeprom_enableStartHours);
  Serial.println(controller.eeprom_enableEndHours);
  Serial.println(controller.eeprom_activeTime);
  Serial.println(controller.eeprom_randomPhraseDisplayFlag);*/
  }
else { // eeprom is not written and needs to be written
  //Serial.println("EEPROM not written.  Writing data to EEPROM");
  // EPROMWrite();  // Does not work here
  EEPROM.put(storedAddress, WRITTEN_SIGNATURE);
  strcpy(controller.eeprom_ssid, ssid);
  strcpy(controller.eeprom_pass, pass);
  strcpy(controller.eeprom_k_GMT, k_GMT_str);
  strcpy(controller.eeprom_enableStartHours, enableStartHours_str);
  strcpy(controller.eeprom_enableEndHours, enableEndHours_str);
  strcpy(controller.eeprom_activeTime, activeTime_str);
  strcpy(controller.eeprom_randomPhraseDisplayFlag, randomPhraseDisplayFlag_str);
  // Save everything into EEPROM
  EEPROM.put(storedAddress + sizeof(signature), controller);
  if (!EEPROM.getCommitASAP()){
    //Serial.println("CommitASAP not set. Need commit()");
    EEPROM.commit();
    }
  }
  // in each case convert data to strings and numbers used in the program
  strcpy(ssid, controller.eeprom_ssid);
  strcpy(pass, controller.eeprom_pass);
  k_GMT = atoi(controller.eeprom_k_GMT);
  enableStartHours = atoi(controller.eeprom_enableStartHours);
  enableEndHours = atoi(controller.eeprom_enableEndHours);
  int activeTime_int = atoi(controller.eeprom_activeTime); 
  activeTime = (long)activeTime_int; // and then to unsigned long
  randomPhraseDisplayFlag = atoi(controller.eeprom_randomPhraseDisplayFlag); 

//  for real time clock 
//  Note - This selects the external crystal and confiures the clock.
//  This must be in front of the wdt setup as the modified version
//  of the wdt code has the clock configuration deleted.
rtc.begin();
// for watch dog timer
// Note - Modified WDTZero header file.  16 seconds avail too.
WDTimer.setup(WDT_HARDCYCLE8S);  // initialize WDT counter refesh cycle on 8 sec
// when time runs out, processor does a hardware reset
// for serial monitor

// attempt to connect to WiFi networks :
int number_of_tries_ssid = 0;
char number_of_tries_ssid_str[4]; 
while ( status != WL_CONNECTED) {
  // update display for connection attempts
  WDTimer.clear();  // refresh watch dog timer
  number_of_tries_ssid++;
  Serial.print("Attempting to connect to SSID: ");
  Serial.print(ssid);
  Serial.print("  ");
  Serial.println(number_of_tries_ssid);
  sprintf(number_of_tries_ssid_str, "%i", number_of_tries_ssid);
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  status = WiFi.begin(ssid, pass);
  // wait 10 seconds for connection:
  WDTimer.clear();  // refresh watch dog timer
  delay(5000);
  WDTimer.clear();  // refresh watch dog timer
  delay(5000);
  }
WDTimer.clear();
// you're connected now, so print out the status on the serial port
Serial.print("SSID: ");
Serial.println(WiFi.SSID());
// print your WiFi shield's IP address:
IPAddress ip = WiFi.localIP();
Serial.print("IP Address: ");
Serial.println(ip);
// print and display the received signal strength
rssi = WiFi.RSSI();
Serial.print("Signal strength (RSSI):");
Serial.print(rssi);
Serial.println(" dBm");

// for obtaining epoch and starting rtc
int number_of_tries_NTP = 0;
do {
  WDTimer.clear();  // refresh watch dog timer
  epoch = WiFi.getTime();
  number_of_tries_NTP++;
  Serial.print("NTP tries:  ");
  Serial.println(number_of_tries_NTP);
  delay(1000);
  }
while ((epoch == 0) && (number_of_tries_NTP < 10));
if (number_of_tries_NTP >= 10) {
  strcpy(error_type_str, "NTP-0");
  mkrError();
  }
else {
  WDTimer.clear();  // refresh watch dog timer
  Serial.print("Epoch received: ");
  // print out epoch on serial port
  Serial.println(epoch);
  epoch = epoch + (k_GMT * 60 * 60);  // adjsut for GMT standard time
  rtc.setEpoch(epoch);
  Serial.println();
  }
delay(500);

// attempt to connect to Blynk
Blynk.begin(auth, ssid, pass);
// Note - Blynk.begin is blocking
WDTimer.clear();  // refresh watch dog timer
updateDate();
updateTime();
Serial.println();
Serial.print(date_str);
Serial.print("  ");
Serial.println(time_str);
Serial.println("Flip Disk Controller is online.");
Serial.print(ssid);
Serial.print("  ");
rssi = WiFi.RSSI();
Serial.print(rssi);
Serial.println(" dBm");
Serial.println(); // add line feed
Serial.println("Type cmd in Blynk App for list of commands.");
Serial.println(); // add line feed
terminal.clear();
terminal.print(date_str);
terminal.print("  ");
terminal.println(time_str);
terminal.println("Flip Disk controller is online.");
terminal.print(ssid);
terminal.print("  ");
terminal.print(rssi);
terminal.println(" dBm");
terminal.println();  // add line feed
terminal.println("Type cmd for list of commands.");
terminal.println(); // add line feed
terminal.flush();

WDTimer.clear();  // refresh watch dog timer
Blynk.logEvent("FDC_restarted"); // log event to timeline
// create visual indicator for push buttons
WiFiDrv::pinMode(25, OUTPUT); //GREEN
WiFiDrv::pinMode(26, OUTPUT); //RED
WiFiDrv::pinMode(27, OUTPUT); //BLUE
  
//pinMode(LED_BUILTIN, OUTPUT); // built in yellow LED  Note - this is controlled by pin 6
//digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

// configure leds
//pinMode(redLedPin, OUTPUT); // indicates motion sensor output
//redLedState = 0;
//digitalWrite(redLedPin, HIGH); // turn led off - active low

//pinMode(greenLedPin, OUTPUT); // indicate relay on-off ***** no relay
//greenLedState = 1;
//digitalWrite(greenLedPin, LOW); // turn led on - active low
//pinMode(blueLedPin, OUTPUT);  
//blueLedState = 1; // indicates if power on and wifi connected
                  // flashes when relay is off   ****** no relay
//digitalWrite(blueLedPin, LOW); // turn led on - active low
//pinMode(switchLedPin, OUTPUT);  // indicates same as blue led
//switchLedState = 1;
//digitalWrite(switchLedPin, LOW); // turn led on - active low

// configure motion controls
pinMode(motionPin, INPUT_PULLDOWN);  // remains low or off when not connected
motionState = digitalRead(motionPin);
prior_motionState = motionState;

// Print a confirmation of the EEPROM data
/*terminal.println("Saved Data: ");
terminal.println(controller.eeprom_ssid); 
terminal.println(controller.eeprom_pass); 
terminal.println(controller.eeprom_k_GMT);
terminal.println(controller.eeprom_enableStartHours);
terminal.println(controller.eeprom_enableEndHours);
terminal.println(controller.eeprom_activeTime);
terminal.println(controller.eeprom_randomPhraseDisplayFlag);
terminal.flush();*/

// end setup
}

void loop()
{
WDTimer.clear();  // refresh watch dog timer
currentMillis = millis();
// Check for Blynk command every 0.5 second
if (currentMillis - previousMillis_Blynk >= 500) { 
  previousMillis_Blynk = currentMillis;  // Remember the time
  Blynk.run();
  }
// Check for motion every 0.5 seconds
if (currentMillis - previousMillis_updateMotion >= 500){ 
  previousMillis_updateMotion = currentMillis;  // Remember the time
  motionState = digitalRead(motionPin);  // active high
  if (motionState == HIGH){
    //redLedState = 1; // led on
    lastMotionMillis = currentMillis;  // Remember the time
    }
  else{
    //redLedState = 0; // led off
    }
  if (motionState != prior_motionState){
    prior_motionState = motionState; 
    if (motionReportFlag == 1){
      if (motionState == HIGH){
        terminal.println("motion detected  ");
        }
      else{
        terminal.println("no motion  ");
        }
      //updateTime();
      //terminal.println(time_str); 
      terminal.flush();
      Blynk.run();
      }
    }
  }  

// check if display active every 0.5 seconds
if (currentMillis - previousMillis_displayActive >= 500) { 
  previousMillis_displayActive = currentMillis;  // Remember the time
  // check if motion is in enabled time
  updateTime();
  if (hours_int >= enableStartHours && hours_int < enableEndHours) motionWindowFlag = 1; 
  else motionWindowFlag = 0;
  // check if motion in enabled time in last 5 minutes or command in last 5 minutes
  // activeTime is in minutes
  if ( ((((currentMillis - lastMotionMillis) <= (activeTime * 60000)) 
    && (motionWindowFlag == 1)) && (lastMotionMillis != 0)) 
    || ((currentMillis - lastCommandMillis) <= (activeTime * 60000)) ){     
    displayActiveFlag = 1;
    }
  else {
    displayActiveFlag = 0;
    } 
  }   
// Leds updated every 0.5 seconds
//if (currentMillis - previousMillis_updateLeds >= 500) { 
//  previousMillis_updateLeds = currentMillis;  // Remember the time
//  updateLeds();
//  }

// Time display and random phrase display updated every second
if (currentMillis - previousMillis_updateTimeDisplay >= 1000) {
  previousMillis_updateTimeDisplay = currentMillis;  // Remember the time
  // update the time display if time is turned on and display active
  if ((timeDisplayFlag == 1) && (displayActiveFlag == 1)){
    updateTimeDisplay();
    }
  if ((randomPhraseDisplayFlag == 1) && (displayActiveFlag == 1)){ // not used
    // check if new day
    days_int = rtc.getDay(); // update day
    if (prior_days_int != days_int){
      // WDTimer.clear();  // refresh watch dog timer
      // randomPhraseDisplay();
      prior_days_int = days_int; // only one display per day
      }
    }  
}  
// blink on board led every 1 second
if(currentMillis - previousMillis_blinkOnBoardLED >= 1000) {
  previousMillis_blinkOnBoardLED = currentMillis;  // Remember the time
  blinkOnBoardLED();
  }
// Error after 5 seconds
// error routine sets previousMillis_mkrReset
if((mkr_reset_flag_int == 1) && (currentMillis - previousMillis_mkrReset >= 5000)) {  
  mkrReset();
  }
}  // end of loop


void updateRTC()
{
// get epoch and update RTC
int number_of_tries_NTP = 0;
do {
  epoch = WiFi.getTime();
  number_of_tries_NTP++;
  delay500ms();
  }
while ((epoch == 0) && (number_of_tries_NTP < 6));
  if (number_of_tries_NTP >= 6) {
    strcpy(error_type_str, "NTP-1");
    mkrError();
    }
  else {
    epoch = epoch + (k_GMT * 60 * 60);  // adjsut for GMT standard time
    rtc.setEpoch(epoch);
    } 
}

void updateDate()
{
  // get date data
  years_int = rtc.getYear(); // Note - year is just two digits - 00 to 99
  sprintf(years_str, "%02d", years_int); 
  // above converts to 2 character decimal base - pads leading 0s by adding the 0
  months_int = rtc.getMonth();
  sprintf(months_str, "%02d", months_int);
  days_int = rtc.getDay();
  sprintf(days_str, "%02d", days_int);
  strcpy(date_str, years_str);
  strcat(date_str, "-");
  strcat(date_str, months_str);
  strcat(date_str, "-");
  strcat(date_str, days_str);
}  

void updateTime()
{
  //get time data
  hours_int = rtc.getHours();
  sprintf(hours_str, "%02d", hours_int); 
  // above converts to 2 character decimal base - pads leading 0s by adding the 0
  minutes_int = rtc.getMinutes();
  sprintf(minutes_str, "%02d", minutes_int); 
  seconds_int = rtc.getSeconds();
  sprintf(seconds_str, "%02d", seconds_int); 
  strcpy(time_str, hours_str);
  strcat(time_str, ":");
  strcat(time_str, minutes_str);
  strcat(time_str, ":");
  strcat(time_str, seconds_str);
}  

void mkrError()
{
  // do not log new errors if one has been reported 
  //   and now in prcess of logging and resetting
  if (mkr_reset_flag_int == 1) {return;}
  // general case errors
  strcpy(error_type_display_str, error_type_str);
  strcat(error_type_display_str, " Error");
  // special case error
  if (strcmp(error_type_str, "BT") == 0) {
    strcpy(error_type_display_str, "BT restart");
    }  
  if (strcmp(error_type_str, "ST") == 0) {
    strcpy(error_type_display_str, "ST restart");
    }  
  // set flag for error reporting and shutdown
  mkr_reset_flag_int = 1;
}

// processor software reset 
void mkrReset()  // runs at end of 5 second mkrReset timer
{
  // send notification
  strcpy(notification_str, error_type_display_str);
  Blynk.logEvent("7x50 flip dot restarted", String(notification_str)); 
  // above log restart in timeline nnnnn
  Serial.println(notification_str);
  NVIC_SystemReset();
}

void setCell(int cellColumn, int cellRow, int cellValue)
{ 
  // solenoid controls
  byte RHDOut; // 1 is on
  byte RLDOut; // 0 is on 
  byte CHDOut; // 1 is on 
  byte CLDOut; // 0 is on
  // row drivers -  8 rows max - active low due to level shifter
  // invert with exclusive or
  byte rowOut = byte(cellRow); 
  byte RSAOut = (rowOut & 0x01) ^ 0x01; // bitwise and
  byte RSBOut = ((rowOut >> 1) & 0x01) ^ 0x01;
  byte RSCOut = ((rowOut >> 2) & 0x01) ^ 0x01;
  // column drivers - active low due to level shifter
  byte columnOut = byte(cellColumn);
  byte CSAOut = 0; // active low
  byte CSBOut = 0; // active low
  byte CSCOut = 0; // active low
  byte CS0Out = 0; // active low - for below, 0 causes high inhibit which turns on inhibit
  byte CS1Out = 0; // active low 
  byte CS2Out = 0; // active low
  byte CS3Out = 0; // active low
  byte CNTROut = 0; // active low
  byte ENDOut = 0; // active low
  if (columnOut >=  0 && columnOut <= 24) CNTROut = 1; // these active the coils on each board
  if (columnOut >= 25 && columnOut <= 49) ENDOut = 1;
  if (CNTROut == 1){  // 1 to 25 columns
    if ((columnOut >=  0) && (columnOut <=  7)) CS0Out = 1;  // 1 causes a low inhibit which turns off inhibit
    if ((columnOut >=  8) && (columnOut <= 15)) CS1Out = 1;
    if ((columnOut >= 16) && (columnOut <= 23)) CS2Out = 1;
    if (columnOut == 24) CS3Out = 1; // 25 columns max
    // invert with exclusive or
    CSAOut = (columnOut & 0x01) ^ 0x01;
    CSBOut = ((columnOut >> 1) & 0x01) ^ 0x01;  
    CSCOut = ((columnOut >> 2) & 0x01) ^ 0x01;   
    }
  if (ENDOut == 1){ // 26 to 50 columns
    if ((columnOut >= 25) && (columnOut <= 32)) CS0Out = 1;
    if ((columnOut >= 33) && (columnOut <= 40)) CS1Out = 1;
    if ((columnOut >= 41) && (columnOut <= 48)) CS2Out = 1;
    if (columnOut == 49) CS3Out = 1; // 50 columns max
    columnOut --; // ** adjust to allow proper calculation
    // invert with exclusive or
    CSAOut = (columnOut & 0x01) ^ 0x01;
    CSBOut = ((columnOut >> 1) & 0x01) ^ 0x01;  
    CSCOut = ((columnOut >> 2) & 0x01) ^ 0x01;   
    }
  // cell value driver
  // inverting level shifter and 
  // inverting active low 2580 driver
  // active high 2801 driver
  // all paths have two inversions from mkr
  //
  //  CHD mkr H  2801 level shift L 4051 NC 2580 active low  on 
  //  CHD mkr L  2801 level shift H 4051 NC 2580 active low  off
  //  CLD mkr H  2801 level shift L 4051 NC 2801 active high  off
  //  CLD mkr L  2801 level shift H 4051 NC 2801 active high  on
  //
  //  RHD mkr H  2801 level shift L 4051 NC 2580 active low  on 
  //  RHD mkr L  2801 level shift H 4051 NC 2580 active low  off
  //  RLD mkr H  2801 level shift L 4051 NC 2801 active high  off
  //  RLD mkr L  2801 level shift H 4051 NC 2801 active high  on
  //
  //  high drives - on if 1
  //  low drives - on if 0
  //  protection diodes after level shifter should 
  //    prevent high low driver concurrent state from happening 
  //
  if (cellValue == 1){ // set yellow side
    RHDOut = 0; // 0 1 0 active high - off
    RLDOut = 0; // 0 1 0 active low - on
    CHDOut = 1; // 1 0 1 active high - on
    CLDOut = 1; // 1 0 1 active low - off
    }
  else { // set black side
    RHDOut = 1; // 1 0 1 active high - on
    RLDOut = 1; // 1 0 1 active low - off
    CHDOut = 0; // 0 1 0 active high - off
    CLDOut = 0; // 0 1 0 active low - on 
    }
  
  // for protection, turn off row and column drivers and coils through
  // cd4051 inhibits (active high)  
  // below inverted by 2801A level shifter
  digitalWrite(CNTRpin, LOW);
  digitalWrite(ENDpin, LOW);
  digitalWrite(CS0pin, LOW); 
  digitalWrite(CS1pin, LOW); 
  digitalWrite(CS2pin, LOW); 
  digitalWrite(CS3pin, LOW); 
  delayMicroseconds(5);
  // set up row driver
  digitalWrite(RLDpin, RLDOut);
  digitalWrite(RHDpin, RHDOut);
  digitalWrite(RSApin, RSAOut);
  digitalWrite(RSBpin, RSBOut);
  digitalWrite(RSCpin, RSCOut);
  delayMicroseconds(5);
  // set up column driver
  digitalWrite(CLDpin, CLDOut);
  digitalWrite(CHDpin, CHDOut);
  digitalWrite(CSApin, CSAOut); 
  digitalWrite(CSBpin, CSBOut); 
  digitalWrite(CSCpin, CSCOut); 
  delayMicroseconds(5);

  if (pulseNumber == 0){  
    // pulseNumber 0 - no pulse, 1 - normal pulse, 2 doulbe pulse
    // rows and columns set up, but no pulse at cd4051 row drivers
    // useful for debugging
    }
    
  if ((pulseNumber == 1) || (pulseNumber == 2)) {
    if (cellValue ==1){ // set yellow side
      // engage column driver first as high side
      digitalWrite(CS0pin, CS0Out); 
      digitalWrite(CS1pin, CS1Out); 
      digitalWrite(CS2pin, CS2Out); 
      digitalWrite(CS3pin, CS3Out); 
      // then engage row driver as low side 
      // to complete circuit
      digitalWrite(CNTRpin, CNTROut);
      digitalWrite(ENDpin, ENDOut);
      delayMicroseconds(5);
      }
    else{ // set black side
      // engage row driver first as high side 
      digitalWrite(CNTRpin, CNTROut);
      digitalWrite(ENDpin, ENDOut);
      // then engage column driver as low side 
      // to complete circuit
      digitalWrite(CS0pin, CS0Out); 
      digitalWrite(CS1pin, CS1Out); 
      digitalWrite(CS2pin, CS2Out); 
      digitalWrite(CS3pin, CS3Out); 
      delayMicroseconds(5);
      }
    
    delayMicroseconds(pulseTime*100); // delay = pulseTime * 100us - 1 ms pulse default
    // after turn off, it takes 60 ms to complete the entire flip
    
    // turn off row and column drivers and coils through cd4051 inhibits (active high)
    // all below inverted by 2801A level shifter
    if (cellValue ==1){ // set yellow side
      // turn off row driver as low side first
      digitalWrite(CNTRpin, LOW);
      digitalWrite(ENDpin, LOW);
      digitalWrite(CS0pin, LOW); 
      digitalWrite(CS1pin, LOW); 
      digitalWrite(CS2pin, LOW); 
      digitalWrite(CS3pin, LOW); 
      }
    else{ // set black side
      // turn off column driver as low side first
      digitalWrite(CS0pin, LOW); 
      digitalWrite(CS1pin, LOW); 
      digitalWrite(CS2pin, LOW); 
      digitalWrite(CS3pin, LOW); 
      digitalWrite(CNTRpin, LOW); 
      digitalWrite(ENDpin, LOW);
      }
    }   
  if (pulseNumber == 2) {  // double pulse
      // repeat above again after post pulse delay
      delayMicroseconds(pulsePostDelay * 100);  // delay between pulses
      
      if (cellValue ==1){ // set yellow side
      // engage column driver first as high side
      digitalWrite(CS0pin, CS0Out); 
      digitalWrite(CS1pin, CS1Out); 
      digitalWrite(CS2pin, CS2Out); 
      digitalWrite(CS3pin, CS3Out); 
      // then engage row driver as low side 
      // to complete circuit
      digitalWrite(CNTRpin, CNTROut);
      digitalWrite(ENDpin, ENDOut);
      }
    else{ // set black side
      // engage row driver first as high side 
      digitalWrite(CNTRpin, CNTROut);
      digitalWrite(ENDpin, ENDOut);
      // then engage column driver as low side 
      // to complete circuit
      digitalWrite(CS0pin, CS0Out); 
      digitalWrite(CS1pin, CS1Out); 
      digitalWrite(CS2pin, CS2Out); 
      digitalWrite(CS3pin, CS3Out); 
      }
    
    delayMicroseconds(pulseTime*100); // delay = pulseTime * 100us - 1 ms pulse default
    // after turn off, it takes 60 ms to complete the entire flip
    
    // turn off row and column drivers and coils through cd4051 inhibits (active high)
    // Note - could test driving through turning on and off 4051 inputs instead of inhibits
    // all below inverted by 2801A level shifter
    if (cellValue == 1){ // set yellow side
      // turn off row driver as low side first
      digitalWrite(CNTRpin, LOW);
      digitalWrite(ENDpin, LOW);
      digitalWrite(CS0pin, LOW); 
      digitalWrite(CS1pin, LOW); 
      digitalWrite(CS2pin, LOW); 
      digitalWrite(CS3pin, LOW); 
      }
    else{ // set black side
      // turn off column driver as low side first
      digitalWrite(CS0pin, LOW); 
      digitalWrite(CS1pin, LOW); 
      digitalWrite(CS2pin, LOW); 
      digitalWrite(CS3pin, LOW); 
      digitalWrite(CNTRpin, LOW); 
      digitalWrite(ENDpin, LOW);
      }
    }  

  // turn off inputs to row and column coil drivers
  digitalWrite(RHDpin, LOW);  // turn off high drives
  digitalWrite(CHDpin, LOW);
  digitalWrite(RLDpin, HIGH);  // turn off low drives
  digitalWrite(CLDpin, HIGH);
    
  delayMicroseconds(pulsePostDelay * 100);  // post pulse delay between pulses
  
  // for testing - very slow flipping however as runs after each cell
  /*terminal.println();
  terminal.println("columnOut, cellColumn");
  terminal.print(columnOut);
  terminal.print(", ");
  terminal.println(cellColumn);
  terminal.println("CS3Out, CS2Out, CS1Out, CS0Out"); 
  terminal.print(CS3Out); // only one variable per print statement
  terminal.print(CS2Out);
  terminal.print(CS1Out);
  terminal.println(CS0Out);
  terminal.println("CSCOut, CSBOut, CSAOut");
  terminal.print(int(CSCOut));
  terminal.print(int(CSBOut));
  terminal.println(int(CSAOut));
  terminal.println("RSCOut, RSBOut, RSAOut"); 
  terminal.print(int(RSCOut));
  terminal.print(int(RSBOut));
  terminal.println(int(RSAOut)); 
  terminal.println("CNTROut, ENDOut");
  terminal.print(int(CNTROut));
  terminal.println(int(ENDOut));
  terminal.println("RLDOut, RHDOut, CLDOut, CHDOut");
  terminal.print(int(RLDOut));
  terminal.print(int(RHDOut));
  terminal.print(int(CLDOut));
  terminal.println(int(CHDOut));
  terminal.println();
  terminal.flush();
  Blynk.run();
  */
}
  
// For Blynk terminal commands
BLYNK_WRITE(V3)
{
  strcpy(cmd_str, param.asStr());  
  cmd_length_int = strlen(cmd_str);  // note - does not include '\0'.
  //terminal.println(cmd_str); // for testing
  //terminal.flush();
  //Blynk.run();
  
  // if not phrase command, clear blankFlag
  if (cmd_str[0] != '@' && strlen(cmd_str) < 3){blankFlag == 0;}
  
  // delay if delay set
  // useful to record split flaps on video
  if (cmdDelay != 0){
    for (int k = 0; k < cmdDelay; k++){delay1s();} 
    }
  
  // page two commands start first
  // for second line date and time
  if ((terminal_second_line_flag_int == 1) 
     && (strcmp(param.asStr(), "") != 0)) { // returns 0 if equal
     int c_day_int; 
     int c_month_int; 
     int c_year_int; 
     int c_hours_int; 
     int c_minutes_int; 
     int c_seconds_int;
     char c_day_str[4]; 
     char c_month_str[4]; 
     char c_year_str[4]; 
     char c_hours_str[4]; 
     char c_minutes_str[4]; 
     char c_seconds_str[4];
     if (strlen(param.asStr()) != 17) { // note- null character ‘\0’ not counted
       terminal.println();  // add line feed
       terminal.println ("Invalid entry");
       terminal.println();  // add line feed
       terminal_second_line_flag_int = 0; // reset file read flag 
       terminal.flush();
       return;
       }
     sscanf(param.asStr(), "%d%*c%d%*c%d%*c%d%*c%d%*c%d", &c_year_int, 
       &c_month_int, &c_day_int, &c_hours_int, &c_minutes_int, &c_seconds_int);
     sscanf(param.asStr(), "%2s%*c%2s%*c%2s%*c%2s%*c%2s%*c%2s", c_year_str, 
       c_month_str, c_day_str, c_hours_str, c_minutes_str, c_seconds_str);
     terminal.println();  // add line feed
     terminal.print("Date and time changed to: ");
     terminal.print(c_year_str);
     terminal.print("/");
     terminal.print(c_month_str);
     terminal.print("/");
     terminal.print(c_day_str);
     terminal.print(" ");
     terminal.print(c_hours_str);
     terminal.print(":");
     terminal.print(c_minutes_str);
     terminal.print(":");
     terminal.println(c_seconds_str);
     terminal_second_line_flag_int = 0; // reset file read flag 
     // format rtc.setDate(byte day, byte month, byte year)
     // format rtc.setTime(byte hours, byte minutes, byte seconds)
     rtc.setDate(c_day_int, c_month_int, c_year_int);
     rtc.setTime(c_hours_int, c_minutes_int, c_seconds_int);
     //sscanf(param.asStr(), "%2s%*c%2s%*c%2s", 
     //   prior_years_str, prior_months_str, prior_days_str);    
     //terminal.print("prior years: ");  // for testing
     //terminal.println(prior_years_str);
     //terminal.print("prior months: ");
     //terminal.println(prior_months_str);
     //terminal.print("prior days: ");
     //terminal.println(prior_days_str);    
     terminal.println();  // add line feed
     // Ensure everything is sent
     terminal.flush();
     Blynk.run();
     return;
  }
  // for second line Wifi ssid
    if ((serial_second_line_flag_int == 2) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
     if (strlen(cmd_str) > 15) { // note- null character ‘\0’ not counted
       terminal.println ("Invalid entry");
       terminal.println();  // add line feed
       terminal.flush();
       Blynk.run();
       serial_second_line_flag_int = 0; // reset file read flag 
       return;
       }
     strcpy(ssid, cmd_str);
     terminal.print("Wifi SSID changed to: ");
     terminal.println(ssid); 
     terminal.println();  // add line feed
     terminal.flush();
     Blynk.run();
     EEPROMWrite();
     serial_second_line_flag_int = 0; // reset file read flag 
     return;
     }
  // for second line Wifi password
    if ((serial_second_line_flag_int == 3) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
     if (strlen(cmd_str) > 15) { // note- null character ‘\0’ not counted
       terminal.println ("Invalid entry");
       terminal.println();  // add line feed
       terminal.flush();
       Blynk.run();
       serial_second_line_flag_int = 0; // reset file read flag 
       return;
       }
     strcpy(pass, cmd_str);
     terminal.print("Wifi password changed to: ");
     terminal.println(pass); 
     terminal.println();  // add line feed
     terminal.flush();
     Blynk.run();
     EEPROMWrite();
     serial_second_line_flag_int = 0; // reset file read flag 
     return;
     }
  // for second line GMT offset
    if ((serial_second_line_flag_int == 4) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
      int l = atoi(cmd_str);
      if ((l > 12) || (l < -12)) {
        terminal.println ("Invalid entry");
        terminal.println();  // add line feed
        terminal.flush();
        Blynk.run();
        serial_second_line_flag_int = 0; // reset file read flag 
        return;
        }
      strcpy(k_GMT_str, cmd_str);
      k_GMT = atoi(k_GMT_str);
      EEPROMWrite();
      serial_second_line_flag_int = 0; // reset file read flag
      updateRTC();
      // read new time
      updateTime();
      terminal.print("GMT offset changed to: ");
      terminal.println(k_GMT_str); 
      terminal.println("Real time clock updated"); 
      terminal.println(); // add line feed
      terminal.flush();
      Blynk.run();
      return;
      }
  // for second line enable start hours
    if ((serial_second_line_flag_int == 5) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
      int l = atoi(cmd_str);
      if ((l < 0) || (l > 24)) {
        terminal.println ("Invalid entry");
        terminal.println();  // add line feed
        terminal.flush();
        Blynk.run();
        serial_second_line_flag_int = 0; // reset file read flag 
        return;
        }
      strcpy(enableStartHours_str, cmd_str);
      terminal.print("Enable start hour changed to: ");
      terminal.println(enableStartHours_str); 
      enableStartHours = atoi(enableStartHours_str);
      terminal.print("Integer is: ");
      terminal.println(enableStartHours);  
      terminal.println();  // add line feed
      terminal.flush();
      Blynk.run();
      EEPROMWrite();
      serial_second_line_flag_int = 0; // reset file read flag
      return;
      }
  // for second line enable end hours
    if ((serial_second_line_flag_int == 6) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
      int l = atoi(cmd_str);
      if ((l < 0) || (l > 24)) {
        terminal.println ("Invalid entry");
        terminal.println();  // add line feed
        serial_second_line_flag_int = 0; // reset file read flag 
        return;
        }
      strcpy(enableEndHours_str, cmd_str);
      terminal.print("Enable end hour changed to: ");
      terminal.println(enableEndHours_str); 
      enableEndHours = atoi(enableEndHours_str);
      terminal.print("Integer is: ");
      terminal.println(enableEndHours);  
      terminal.println();  // add line feed
      terminal.flush();
      Blynk.run();
      EEPROMWrite();
      serial_second_line_flag_int = 0; // reset file read flag
      return;
      }  
  // for second line active time
    if ((serial_second_line_flag_int == 7) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
      int l = atoi(cmd_str);
      if ((l < 1) || (l > 1440)) {  // time is in minutes
        terminal.println ("Invalid entry");
        terminal.println();  // add line feed
        serial_second_line_flag_int = 0; // reset file read flag 
        return;
        }
      strcpy(activeTime_str, cmd_str);
      terminal.print("Active time (minutes) changed to: ");
      terminal.println(activeTime_str); 
      int activeTime_int = atoi(activeTime_str); // convert string to integer 
      activeTime = (long)activeTime_int; // and then to unsigned long
      // activeTime = (long)atoi(read_activeTime);
      terminal.print("Integer is: ");
      terminal.println(activeTime);  
      terminal.println();  // add line feed
      terminal.flush();
      Blynk.run();
      EEPROMWrite();
      serial_second_line_flag_int = 0; // reset file read flag
      return;
      }
  // for second line random phrase display
    if ((serial_second_line_flag_int == 8) && (strcmp(cmd_str, "") != 0)) { // returns 0 if equal
      int l = atoi(cmd_str);
      if ((l != 0) && (l != 1)) {  // flag is 0 off or 1 on
        terminal.println ("Invalid entry");
        terminal.println();  // add line feed
        serial_second_line_flag_int = 0; // reset file read flag 
        return;
        }
      strcpy(randomPhraseDisplayFlag_str, cmd_str);
      randomPhraseDisplayFlag = atoi(randomPhraseDisplayFlag_str);
      terminal.print("Random phrase display flag set to: ");
      terminal.print(randomPhraseDisplayFlag_str); 
      if (randomPhraseDisplayFlag == 1) {terminal.println (" on");}
      else {terminal.println(" off");}  
      terminal.print("Integer is: ");
      terminal.println(randomPhraseDisplayFlag);  
      terminal.println();  // add line feed
      terminal.flush();
      Blynk.run();
      EEPROMWrite();
      serial_second_line_flag_int = 0; // reset file read flag
      return;
      }  
  WDTimer.clear();  // refresh watch dog timer
  // end page two commands 
  // use caution defining 4 letter commands so as not 
  // to interfere with decoding of remote graphics direct commands  
  if (strcmp(cmd_str, "cmd") == 0) { // list commands
    terminal.println(",__    - display 10 char letter string"); 
    terminal.println(";__    - display letter string split"); 
    terminal.println("dxc    - display ascii c on x (0-9)"); 
    terminal.println("((     - set all characters yellow"); 
    terminal.println("))     - set all characters black");
    terminal.println("((x    - set character x yellow (0-9)"); 
    terminal.println("))x    - set character x black (0-9)");
    terminal.println("(ccr   - set cell yellow"); 
    terminal.println(")ccr   - set cell black"); 
    terminal.println("(cc    - set column yellow"); 
    terminal.println(")cc    - set column black"); 
    terminal.println("(r     - set row yellow"); 
    terminal.println(")r     - set row black");
    terminal.println("ya     - test alpha display");
    terminal.println("yb     - test alpha display again");
    terminal.println("yc     - test scrolling alpha display (char 0)");
    terminal.println("yd     - test scrolling alpha display (char 0-9)");
    terminal.println("ye     - test checker pattern display");
    terminal.println("yf     - test columns and rows display");
    terminal.println("yg     - test characters by displaying 0-9");
    terminal.println("yhx    - set all characters to x");
    terminal.println("yi     - random dot display");
    terminal.println("rst    - reset controller");
    terminal.println("sig    - report WiFi signal strength"); 
    terminal.println("v      - report version of code");
    terminal.println("rtc    - report rtc time");
    terminal.println("c      - Blynk terminal clear");
    terminal.println("clr    - local terminal clear");
    terminal.println("cmd    - list available commands");
    terminal.println("cmdm   - list more commands");
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    } // note 50 max terminal width
 if (strcmp(cmd_str, "cmdm") == 0) { // list more commands
    terminal.println("ptxx   - set pulse time (1-50 100us steps)");
    terminal.println("ppxx   - set post pulse delay (1-50 100us steps)");
    terminal.println("ssxxx  - set column scroll speed (1-999 ms)");
    terminal.println("pnx    - set pulse number (0, 1 or 2)");
    terminal.println("trs    - report/syncs rtc/WiFi times");
    terminal.println("tdon   - time display on");
    terminal.println("tdoff  - time display off");
    terminal.println("tdclr  - clear time display");
    terminal.println("tdset  - set time display");
    terminal.println("tdsec  - time display toggle seconds on/off");
    terminal.println("cdt    - change date and time");  // second page flag 1
    terminal.println("rms    - report motion states (15s)");
    terminal.println("rmot   - report motion detection (toggle)");
    terminal.println("cat    - cancel active time");
    terminal.println("st     - report op status");
    terminal.println("delxx  - set command delay 0-15 seconds");
    terminal.println("cssid  - change Wifi SSID (eeprom)");  // second page flag 2
    terminal.println("cpass  - change Wifi password (eeprom)");   // second page flag 3
    terminal.println("cgmto  - change GMT offset (eeprom)");  // second page flag 4
    terminal.println("csthr  - change enable start hour (eeprom)");  // second page flag 5
    terminal.println("cenhr  - change enable end hour (eeprom)");  // second page flag 6
    terminal.println("cactm  - change active time minutes (eeprom)");  // second page flag 7
    terminal.println("crpf   - change random phrase flag (eeprom)");  // second page flag 8
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (cmd_str[0] == ','){  // if comma, letter string command - up to 10 letters
    int strLength = strlen(cmd_str);  // includes '\0'
    lastCommandMillis = currentMillis; // update time out  *** add to other commands???
    clearTimeDisplay();  // turn time display off
    for (int charNum = 0; charNum <= 9; charNum ++){
      if (charNum <= (strLength - 2)){  // [0] is ',' and subtract out '\0' too
        displayCharacter(charNum, cmd_str[charNum + 1]); // 0 is ','
        }
      else{
        displayCharacter(charNum, ' '); // fill blanks
        }
      delay25ms();
      }
    delay5s();
    setTimeDisplay(); // turn time display on
    return;
    }  
  if (cmd_str[0] == ';'){  // if semicolon, letter string command 
    lastCommandMillis = currentMillis; // update time out  *** add to other commands???
    clearTimeDisplay();  // turn time display off
    createDisplayStrings(); // split words into lines and display
    delay2s(); // total of 5 with line delay of 3
    setTimeDisplay();  // turn time display on    
    return;
    }  
  if (strcmp(cmd_str, "rst") == 0) {  // reset mkr
    terminal.println(); // add line feed
    terminal.println("device reset through Blynk terminal");
    // report type of error
    strcpy(error_type_str, "BT");
    mkrError();
    return;
    }
  if (strcmp(cmd_str, "ya") == 0){ 
    // test alpha display
    char example_str[128] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz-./"; // 66 char
    int valueInput = 0; // 1 - yellow,  0 - black
    for (int k = 0; k < 66; k++){
      // get font array
      get_char_fd_array(example_str[k]); 
      // display font
      // display font
      for (int charNum = 0; charNum <= 9; charNum ++){
        for (int columnInput = (charNum * 5); columnInput <= ((charNum * 5) + 4); columnInput++){
          for (int rowInput = 0; rowInput <= 6; rowInput++){
            valueInput = fd_array[columnInput - (charNum * 5)][rowInput];
            setCell(columnInput, rowInput, valueInput);
            }    
          }
        }
      delay1s(); // delay between characters
      }
    return;
    }
  if (strcmp(cmd_str, "yb") == 0){ 
    // test alpha display again
    char example_str[128] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ !#$%&'()*+,-./:;<=>?@[\]^_`{|}~"; // 68 char
    int valueInput = 0; // 1 - yellow,  0 - black
    for (int k = 0; k < 68; k++){
      // get font array
      get_char_fd_array(example_str[k]); 
      // display font
      for (int charNum = 0; charNum <= 9; charNum ++){
        for (int columnInput = charNum * 5; columnInput <= (charNum * 5) + 4; columnInput++){
          for (int rowInput = 0; rowInput <=6; rowInput++){
            valueInput = fd_array[columnInput - (charNum * 5)][rowInput];
            setCell(columnInput, rowInput, valueInput);
            }    
          }
        }
      delay1s(); // delay between characters
      }
    return;
    }
  if (strcmp(cmd_str, "yc") == 0){ 
    // test scrolling alpha display
    char example_str[128] = " 0123456789 "; // 1 char 12 numbers/letters
    int valueInput = 0; // 1 - yellow,  0 - black
    // load scrolling font array with blank column between each character
    for (int k = 0; k < 12; k++){
      // get font array
      get_char_fd_array(example_str[k]); 
      // load scrolling array 
      for (int columnInput = 0; columnInput <= 5; columnInput++){ // 6 columns to include blank column
        for (int rowInput = 0; rowInput <=6; rowInput++){
          // scrolling_fd_array holds up to 50 characters [50*6][7]
          if (columnInput !=5){
            scrolling_fd_array[(k*6)+columnInput][rowInput] = fd_array[columnInput][rowInput];
            }
          else{ // blank column after each character
            scrolling_fd_array[(k*6)+columnInput][rowInput] = 0x0;
            }  
          }
        }
      }  
    // display scrolling font array
    for (int base = 0; base < 12*6; base ++){ // 6 columns to include blank column
      for (int columnInput = 49; columnInput >= 0; columnInput--){ // 6 columns to include blank column
        for (int rowInput = 0; rowInput <=6; rowInput++){
          valueInput = scrolling_fd_array[base+columnInput][rowInput];
          setCell(columnInput, rowInput, valueInput);
          }  
        }
      //delay25ms(); // reset wdt and run Blynk
      WDTimer.clear();
      delay(scrollDelay); // 0-999 ms delay between column updates
      }
    return;
    }
  if (strcmp(cmd_str, "yd") == 0){ 
    // test scrolling number display
    char example_str[128] = "-0123456789-"; // 12 char 12 letters/numbers
    int valueInput = 0; // 1 - yellow,  0 - black
    // load scrolling font array with blank column between each character
    // load 10 blank characters of 6 columns each
    for (int columnInput = 0; columnInput <= 60; columnInput++){
        for (int rowInput = 0; rowInput <= 6; rowInput++){
           // scrolling_fd_array holds up to 50 characters [50*6][7]
          scrolling_fd_array[columnInput][rowInput] = 0x0;
          // jw*** in final version will need to add start and end pointers for displaying array
        }
      }
    for (int k = 0; k < 12; k++){
      // get font array
      get_char_fd_array(example_str[k]); 
      // load scrolling array 
      for (int charNum = 0; charNum <= 12; charNum ++){ // 12 characters in example
        // each character has 6 columns including a blank column
        for (int charColumnNum = 0; charColumnNum <= 5; charColumnNum ++){  
          for (int rowInput = 0; rowInput <= 6; rowInput++){
            // scrolling_fd_array holds up to 50 characters [50*6][7]
            // starting with 10 blanks or at 60 with 0-59 filled
            scrolling_fd_array[60 + (charNum * 6) + charColumnNum][rowInput] = fd_array[charColumnNum][rowInput];
            if ((charColumnNum + 1) % 6 == 0){ //override as blank column after each character
              scrolling_fd_array[60 + (charNum * 6) + charColumnNum][rowInput] = 0x0;
              }  
            } 
          }
        }
      }  
   // load 10 blank characters of 6 columns each
   // for (int columnInput = (10 + 12) * 6; columnInput <= (10 + 12 + 10) * 6; columnInput++){
   //     for (int rowInput = 0; rowInput <= 6; rowInput++){
   //        // scrolling_fd_array holds up to 50 characters [50*6][7]
   //       scrolling_fd_array[columnInput][rowInput] = 0x0;
   //       // jw*** in final version will need to add start and end pointers for displaying array
   //       }
   //   } 
    // display scrolling font array
    for (int charColumnNum = 0; charColumnNum < (10 + 12 + 10); charColumnNum ++){ // 32 characters with 6 columns to include blank column
      for (int displayColumn = 0; displayColumn <= 49; displayColumn ++){ 
          for (int rowInput = 0; rowInput <=6; rowInput++){
          valueInput = scrolling_fd_array[charColumnNum + displayColumn][rowInput];
          setCell(displayColumn, rowInput, valueInput);
          }  
        }
      delay25ms(); // reset wdt and run Blynk
      delay(scrollDelay); // 0-999 ms delay between column updates
      }
    return;
    }
  if (strcmp(cmd_str, "ye") == 0){ 
    // test checker fonts 0x00 0x01
    int valueInput = 0; // 1 - yellow,  0 - black
    byte checker = 0x00;
    for (int k = 0; k < 20; k++){
      // get font array
      get_char_fd_array(checker); 
      // display font
      for (int charNum = 0; charNum <= 9; charNum ++){ // 10 characters in example
        for (int columnInput = 0; columnInput <= 4; columnInput++){
          for (int rowInput = 0; rowInput <= 6; rowInput++){
            valueInput = fd_array[columnInput][rowInput]; // same display each character 
            setCell(columnInput + (charNum * 5), rowInput, valueInput);
            }  
          }
        }
      delay25ms(); // reset wdt and run Blynk
      delay(scrollDelay); // 0-999 ms delay between character updates
      checker = checker ^ 0x01; // toggle checker
      }
    return;
    }
  if (strcmp(cmd_str, "yf") == 0){ 
    // test columns and then rows
    int valueInput = 1; // 1 - yellow,  0 - black
    for (int columnInput = 0; columnInput <= 50; columnInput++){
      for (int rowInput = 0; rowInput <=6; rowInput++){
        setCell(columnInput, rowInput, valueInput);
        }  
      delay25ms(); // reset wdt and run Blynk
      delay(scrollDelay); // 0-999 ms delay between character updates
      }
    valueInput = 0; // 1 - yellow,  0 - black
    for (int columnInput = 0; columnInput <= 49; columnInput++){ // 50 columns
      for (int rowInput = 0; rowInput <=6; rowInput++){
        setCell(columnInput, rowInput, valueInput);
        }  
      delay25ms(); // reset wdt and run Blynk
      delay(scrollDelay); // 0-999 ms delay between character updates
      }
    valueInput = 1; // 1 - yellow,  0 - black
    for (int rowInput = 0; rowInput <=6; rowInput++){
      for (int columnInput = 0; columnInput <= 49; columnInput++){ // 50 columns
        setCell(columnInput, rowInput, valueInput);
        }
      delay25ms(); // reset wdt and run Blynk
      delay(scrollDelay); // 0-999 ms delay between character updates
      }
    valueInput = 0; // 1 - yellow,  0 - black
    for (int rowInput = 0; rowInput <=6; rowInput++){
      for (int columnInput = 0; columnInput <= 49; columnInput++){ // 50 columns
        setCell(columnInput, rowInput, valueInput);
        }  
      delay25ms(); // reset wdt and run Blynk
      delay(scrollDelay); // 0-999 ms delay between character updates
      }
    return;
    }
  if (strcmp(cmd_str, "yg") == 0){ 
    char example_str[128] = "0123456789"; // 10 numbers
    for (int charNum = 0; charNum <= 9; charNum ++){
      displayCharacter(charNum, example_str[charNum]);
      }
    return;
    }
  if ((cmd_str[0] == 'y') && (cmd_str[1] == 'h') && (cmd_length_int == 3)){ 
    // yhx - set all characters to x
    for (int charNum = 0; charNum <= 9; charNum ++){
      displayCharacter(charNum, cmd_str[2]);
      }
    return;
    }
  if ((cmd_str[0] == 'y') && (cmd_str[1] == 'i') && (cmd_length_int == 2)){ 
    // yh - random dot display
    // set all characters black
    timeDisplayFlag = 0;  // time display off
    blankDisplay();
    delay1s();
    // seed random number gernerator with somewhat random number
    updateTime();
    int seed = (minutes_int * 60) + seconds_int;
    srand(seed); 
    for (int i = 0; i < 30000; i++){ // run random display
      int rRow = rand() % 7;  // create random numbers
      int rColumn = rand() % 50;
      int rCellValue = (rand() % 2) & 0x01; // bitwise and
      setCell(rColumn, rRow, rCellValue); // display random cell
      if (i % 5000 == 0) {WDTimer.clear();}  // clear wdt as needed could be 10000
      }
    blankDisplay();
    delay1s();
    setTimeDisplay(); // time display back on
    return;
    }
  if ((cmd_str[0] == '(') && (cmd_str[1] == '(') && (cmd_length_int == 2)){ 
    // )) - set all characters yellow
    int valueInput = 1; // 1 - yellow,  0 - black
    for (int columnInput = 0; columnInput <= 49; columnInput++){ // 50 columns
      for (int rowInput = 0; rowInput <=6; rowInput++){
        setCell(columnInput, rowInput, valueInput);
        }
      }
    return;
    }
  if ((cmd_str[0] == ')') && (cmd_str[1] == ')') && (cmd_length_int == 2)){ 
    // )) - set all characters black
    int valueInput = 0; // 1 - yellow,  0 - black
    for (int columnInput = 0; columnInput <= 49; columnInput++){ // 50 columns
      for (int rowInput = 0; rowInput <=6; rowInput++){
        setCell(columnInput, rowInput, valueInput);
        }
      }
    return;
    }
  if ((cmd_str[0] == '(') && (cmd_str[1] == '(') && (cmd_length_int == 3)){ 
    // ))x - set character x yellow
    int valueInput = 1; // 1 - yellow,  0 - black
    int columnStart = 0;
    char sc[3];
    sc[0] = cmd_str[2];
    sc[1] = cmd_str[3];
    sc[2] = '\0';
    int charInput = atoi(sc);
    switch(charInput){
      case 0: columnStart = 0; break;
      case 1: columnStart = 5; break;
      case 2: columnStart = 10; break;
      case 3: columnStart = 15; break;
      case 4: columnStart = 20; break;
      case 5: columnStart = 25; break;
      case 6: columnStart = 30; break;
      case 7: columnStart = 35; break;
      case 8: columnStart = 40; break;
      case 9: columnStart = 45; break;
      }
    for (int columnInput = columnStart; columnInput <= columnStart + 4; columnInput++){
      for (int rowInput = 0; rowInput <=6; rowInput++){
        setCell(columnInput, rowInput, valueInput);
        }
      }
    return;
    }
  if ((cmd_str[0] == ')') && (cmd_str[1] == ')') && (cmd_length_int == 3)){ 
    // ))x - set character x black
    int valueInput = 0; // 1 - yellow,  0 - black
    int columnStart = 0;
    char sc[3];
    sc[0] = cmd_str[2];
    sc[1] = cmd_str[3];
    sc[2] = '\0';
    int charInput = atoi(sc);
    switch(charInput){
      case 0: columnStart = 0; break;
      case 1: columnStart = 5; break;
      case 2: columnStart = 10; break;
      case 3: columnStart = 15; break;
      case 4: columnStart = 20; break;
      case 5: columnStart = 25; break;
      case 6: columnStart = 30; break;
      case 7: columnStart = 35; break;
      case 8: columnStart = 40; break;
      case 9: columnStart = 45; break;
      }
    for (int columnInput = columnStart; columnInput <= columnStart + 4; columnInput++){
      for (int rowInput = 0; rowInput <=6; rowInput++){
        setCell(columnInput, rowInput, valueInput);
        }
      }
    return;
    }
  if ((cmd_str[0] == 'd') && (cmd_str[1] == 'a') && (cmd_length_int == 3)){ 
    // dac - display ascii character on all 0-9
    char charOut = cmd_str[2];
    get_char_fd_array(charOut); // populates fd_array with charOut
    // display font
    for (int charNum = 0; charNum <= 9; charNum ++){ // 10 display characters
      for (int columnInput = 0; columnInput <= 4; columnInput++){
        for (int rowInput = 0; rowInput <= 6; rowInput++){
          int valueInput = fd_array[columnInput][rowInput]; // get dot value
          setCell(columnInput + (charNum * 5), rowInput, valueInput);
          } 
        }
      }
    return;
    }
  if ((cmd_str[0] == 'd') && (cmd_length_int == 3)){ 
    // dxc - display ascii c on x (0-9)
    char sch[2];
    sch[0] = cmd_str[1];
    sch[1] = '\0';
    int charNum = atoi(sch);
    char charOut = cmd_str[2];
    get_char_fd_array(charOut); // populates fd_array with charOut
    // display font
    for (int columnInput = 0; columnInput <= 4; columnInput++){
      for (int rowInput = 0; rowInput <= 6; rowInput++){
        int valueInput = fd_array[columnInput][rowInput]; // get dot value
        setCell(columnInput + (charNum * 5), rowInput, valueInput); // adjust columns by charNum
        }  
      }
      return;
    }
  if ((cmd_str[0] == '(') && (cmd_str[1] != '(') && (cmd_length_int == 4)){ 
    // (ccr - set cell yellow
    char sc[3];
    sc[0] = cmd_str[1];
    sc[1] = cmd_str[2];
    sc[2] = '\0';
    int columnInput = atoi(sc);
    sc[0] = cmd_str[3];
    sc[1] = '\0';
    int rowInput = atoi(sc);
    int valueInput = 1; // 1 - yellow,  0 - black
    setCell(columnInput, rowInput, valueInput);
    return;
    }
  if ((cmd_str[0] == ')') && (cmd_str[1] != ')') && (cmd_length_int == 4)){ 
    // )ccr - set cell black
    char sc[3];
    sc[0] = cmd_str[1];
    sc[1] = cmd_str[2];
    sc[2] = '\0';
    int columnInput = atoi(sc);
    sc[0] = cmd_str[3];
    sc[1] = '\0';
    int rowInput = atoi(sc);
    int valueInput = 0; // 1 - yellow,  0 - black
    setCell(columnInput, rowInput, valueInput);
    return;
    }
  if ((cmd_str[0] == '(') && (cmd_str[1] != '(') && (cmd_length_int == 3)){ 
    // (cc - set column yellow
    char sc[3]; 
    sc[0] = cmd_str[1];
    sc[1] = cmd_str[2];
    sc[2] = '\0';
    int columnInput = atoi(sc);
    int valueInput = 1; // 1 - yellow,  0 - black
    for (int rowInput = 0; rowInput <=6; rowInput++){
      setCell(columnInput, rowInput, valueInput);
      }
    return;
    }
  if ((cmd_str[0] == ')') && (cmd_str[1] != ')') && (cmd_length_int == 3)){ 
    // )cc - set column black
    char sc[3]; 
    sc[0] = cmd_str[1];
    sc[1] = cmd_str[2];
    sc[2] = '\0';
    int columnInput = atoi(sc);
    int valueInput = 0; // 1 - yellow,  0 - black
    for (int rowInput = 0; rowInput <=6; rowInput++){
      setCell(columnInput, rowInput, valueInput);
      }
    return;
    }
  if ((cmd_str[0] == '(') && (cmd_length_int == 2)){ 
    // (r - set row yellow
    char sc[3];
    sc[0] = cmd_str[1];
    sc[1] = '\0';
    int rowInput = atoi(sc);
    int valueInput = 1; // 1 - yellow,  0 - black
    for (int columnInput = 0; columnInput <=49; columnInput++){ 
      setCell(columnInput, rowInput, valueInput);
      }
    return;
    }
  if ((cmd_str[0] == ')') && (cmd_length_int == 2)){ 
    // )r - set row black
    char sc[3]; 
    sc[0] = cmd_str[1];
    sc[1] = '\0';
    int rowInput = atoi(sc);
    int valueInput = 0; // 1 - yellow,  0 - black
    for (int columnInput = 0; columnInput <= 49; columnInput++){ 
      setCell(columnInput, rowInput, valueInput);
      }
    return;
    }
  if (cmd_str[0] == 'p' && cmd_str[1] == 't'){
    // pulseTime - set solenoid pulsetime 100us to 9ms ptxx
    char sd[4];
    sd[0] = cmd_str[2];
    sd[1] = cmd_str[3];  // one x causes this to be '/0'
    sd[2] = '\0';
    pulseTime = atoi(sd); // set pulseTime to xx sent
    if ((cmd_length_int < 3 || cmd_length_int > 4) // check for invald entry
      || (atoi(sd) < 1 || atoi(sd) > 50)) { // limit delay to 5ms
      pulseTime = 10; // default to 1ms 10 times 100us
      }
    terminal.println(); // add line feed
    terminal.print("pulseTime set to: ");
    terminal.print(pulseTime * 100);
    terminal.print(" us");
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (cmd_str[0] == 'p' && cmd_str[1] == 'n'){
    // pulseNumber - set number of pulses to 0, 1 or 2
    char sd[4];
    sd[0] = cmd_str[2];
    sd[1] = '\0';
    pulseNumber = atoi(sd); // set pulseNumber to x sent
    if ((cmd_length_int != 3) // check for invald entry
      || (atoi(sd) < 0 || atoi(sd) > 2)) { // check range 0-2
      pulseNumber = 1; // default to 1 pulse
      }
    terminal.println(); // add line feed
    terminal.print("pulseNumber set to: ");
    terminal.print(pulseNumber);
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (cmd_str[0] == 'p' && cmd_str[1] == 'p'){
    // pulsePostDelay - set pulsePostDelay from 0us to 5000us pppxx
    char sd[4];
    sd[0] = cmd_str[2];
    sd[1] = cmd_str[3]; // one x causes this to be '/0'
    sd[2] = '\0';
    pulsePostDelay = atoi(sd); // set pulsePostDelay to xx sent
    if ((cmd_length_int < 3 || cmd_length_int > 4) // check for invald entry
      || (atoi(sd) < 0 || atoi(sd) > 50)) { // limit delay to 5ms
      pulsePostDelay = 5; // default to 500us
      }
    terminal.println(); // add line feed
    terminal.print("pulsePostDelay set to: ");
    terminal.print(pulsePostDelay * 100);
    terminal.print(" us");
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (cmd_str[0] == 's' && cmd_str[1] == 's'){
    // scrollDelay 0 to 999ms ssxxx
    char ss[6];
    ss[0] = cmd_str[2];
    ss[1] = cmd_str[3]; // one x causes this to be '/0'
    ss[2] = cmd_str[4]; // two x causes this to be '/0'
    ss[3] = '\0';
    scrollDelay = atoi(ss); // set scrollDelay to xxx sent
    if ((cmd_length_int < 3 || cmd_length_int > 5) // check for invald entry
      || (atoi(ss) < 0 || atoi(ss) > 999)) { // limit speed to 0 to 999ms
      scrollDelay = 100; // default to 100ms
      }
    terminal.println(); // add line feed
    terminal.print("scrollDelay set to: ");
    terminal.print(scrollDelay);
    terminal.print(" ms");
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (cmd_str[0] == 'd' && cmd_str[1] == 'e' 
    && cmd_str[2] == 'l'){ // delxx - set command delay 0-15 seconds 
    char sd[4];
    sd[0] = cmd_str[3];
    sd[1] = cmd_str[4];  // one x causes this to be '/0'
    sd[2] = '\0';
    cmdDelay = atoi(sd); // set delay to xx sent
    if ((cmd_length_int < 4 || cmd_length_int > 5) // check for invald entry
      || (atoi(sd) < 0 || atoi(sd) > 15)) { // limit delay to 15 seconds
      cmdDelay = 0;
      }
    terminal.println(); // add line feed
    terminal.print("Command delay set to: ");
    terminal.print(cmdDelay);
    terminal.print(" seconds");
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "tdon") == 0) { // time display on
    timeDisplayFlag = 1;
    terminal.println("time display on");
    setTimeDisplay();
    terminal.println();
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "tdoff") == 0) { // time display off
    timeDisplayFlag = 0;
    terminal.println("time display off");
    clearTimeDisplay();
    terminal.println();
    terminal.flush();
    Blynk.run();
    return;
    } 
  if (strcmp(cmd_str, "tdset") == 0) { // set time display - note - likely not neded by itself
    setTimeDisplay();
    delay500ms();
    return;
    } 
  if (strcmp(cmd_str, "tdsec") == 0) {  // toggle seconds on flag
    if (secondsDisplayOn == 1) {secondsDisplayOn = 0;}
    else {secondsDisplayOn = 1;}
    terminal.print("secondsDisplayOn flag = ");
    terminal.println(secondsDisplayOn);
    terminal.println();
    terminal.flush();
    Blynk.run();
    return;
  }
    if (strcmp(cmd_str, "tdclr") == 0) { // clear time display - note - likely not needed by itself
    clearTimeDisplay();
    delay500ms();
    return;
    } 
  if (strcmp(param.asStr(), "cdt") == 0) { // change date and time
     terminal.println(); // add line feed
     terminal.println("Enter date and time (yy/mm/dd hh:mm:ss)");
     terminal_second_line_flag_int = 1;  // set flag for next line read
     terminal.println(); // add line feed
     terminal.flush();
     Blynk.run();
     return;
    }
  if (strcmp(cmd_str, "rtc") == 0) {  // report rtc time
    //get time data
    updateTime();
    terminal.println(); // add line feed
    terminal.print("RTC time: ");
    terminal.println(time_str);
    terminal.println();
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "c") == 0) { // Clear the terminal content    // note - returns 0 if equal
    terminal.clear();  // this is the remote clear.  type clr for a local clear.
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "cssid") == 0) { // change Wifi SSID
    terminal.println("Enter new Wifi SSID: ");
    serial_second_line_flag_int = 2;  // set flag for next WifI SSID line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "cpass") == 0) { // change Wifi password
    terminal.println("Enter new Wifi password: ");
    serial_second_line_flag_int = 3;  // set flag for next WifI password line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "cgmto") == 0) { // change GMT offset
    terminal.println("Enter new GMT offset: ");
    serial_second_line_flag_int = 4;  // set flag for next GMT offset line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "csthr") == 0) { // change enable start hour
    terminal.println("Enter new enable start hour: ");
    serial_second_line_flag_int = 5;  // set flag for next enable start hour line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "cenhr") == 0) { // change enable end hour
    terminal.println("Enter new enable end hour: ");
    serial_second_line_flag_int = 6;  // set flag for next enable end hour line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
 if (strcmp(cmd_str, "cactm") == 0) { // change active time
    terminal.println("Enter new active time (minutes): ");
    serial_second_line_flag_int = 7;  // set flag for next active time line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
 if (strcmp(cmd_str, "crpf") == 0) { // change random phrase display flag
    terminal.println("Set random phrase flag - 0 off 1 on: ");
    serial_second_line_flag_int = 8;  // set flag for next active time line read
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "v") == 0) {  // report version
    terminal.println(); // add line feed
    terminal.print("Version of Controller Code is: ");
    terminal.println(version_str);
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "sig") == 0) {  // report wifi signal strength
    rssi = WiFi.RSSI();
    terminal.println(); // add line feed
    terminal.print("Signal strength (RSSI) is ");
    terminal.print(rssi);
    terminal.println(" dBm");
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "trs") == 0) {  // report and sync rtc and current wifi times
    //get time data
    terminal.println(); // add line feed
    updateTime();
    terminal.print("RTC time is ");
    terminal.println(time_str);
    // update RTC
    updateRTC();
    // read new time
    updateTime();
    terminal.print("WiFi time is ");
    terminal.println(time_str);
    terminal.println(); // add line feed
    terminal.flush();
    Blynk.run();
    return;
    }
  if (strcmp(cmd_str, "wdt") == 0) {  // check wdt function
    unsigned int t;
    terminal.println("\nWatchdog Test - run 18 seconds with a WDTimer.clear()\n");
    Serial.println("\nWatchdog Test - run 18 seconds with a WDT.clear()\n");
    for (t = 1; t <= 18; ++t) {
      WDTimer.clear();  // refresh wdt - before it loops
      delay(950);
      terminal.print(t);
      terminal.print(".");
      terminal.flush();
      Blynk.run(); 
      }
    terminal.println("\n\nWatchdog Test - free run wait for reset at 8 seconds\n");
    for (t = 1; t >= 1; ++t) {
      delay(950);
      terminal.print(t);
      terminal.print(".");
      terminal.flush();
      Blynk.run();
      }
    return;
    }   
  if (strcmp(cmd_str, "cat") == 0) {  //  cancel active time in 10 seconds
    lastMotionMillis = currentMillis - (activeTime * 60000) + 10000;
    lastCommandMillis = currentMillis - (activeTime * 60000) + 10000;
    return;
    }


 
  if (strcmp(cmd_str, "rms") == 0) {  //  report motion state (15s)
    terminal.println("Motion State for 15 seconds:");
    for (int j = 0; j < 15; j++){
      motionState = digitalRead(motionPin);  // active high
      terminal.print("  ");
      terminal.println(motionState);
      terminal.flush();  // output to terminal immediately
      //Blynk.run();      
      delay1s();
      //WDTimer.clear();  // refresh watch dog timer
      }
    return;
    }
    
  if (strcmp(cmd_str, "rmot") == 0) {  //  report motion status
    if (motionReportFlag == 0){motionReportFlag = 1;}
    else motionReportFlag = 0;
    return;
    }
    
  if (strcmp(cmd_str, "st") == 0) {
    terminal.print("uptime = ");
    terminal.print(millis() / 60000);
    terminal.println(" minutes");
    terminal.print("motionState = ");
    terminal.println(motionState);
    terminal.print("motionWindowFlag = ");
    terminal.println(motionWindowFlag);
    //terminal.print("redLedState = ");
    //terminal.println(redLedState);
    terminal.print("cmdDelay = ");
    terminal.print(cmdDelay);
    terminal.println(" seconds");
    terminal.flush();  // output to terminal immediately
    terminal.print("hours_int = ");
    terminal.println(hours_int);
    terminal.print("minutes_int = ");
    terminal.println(minutes_int);
    terminal.print("ssid = ");
    terminal.println(ssid);
    terminal.print("pass = ");
    terminal.println(pass);
    terminal.print("GMT offset = ");
    terminal.println(k_GMT);
    terminal.print("enableStartHours = ");
    terminal.println(enableStartHours);
    terminal.print("enableEndHours = ");
    terminal.println(enableEndHours);
    terminal.print("activeTime = ");
    terminal.print(activeTime);
    terminal.println(" minutes");
    currentMillis = millis();
    terminal.print("remaining time motion(sec) = ");
    // activeTime is in minutes
    if (lastMotionMillis != 0){ // note - if no motion, then lastMotionMillis will be 0
      // Note - min used to avoid negative number
      terminal.println(((activeTime * 60000)- min((currentMillis - lastMotionMillis), (activeTime * 60000)))/1000);
      }
    else {terminal.println(0);}
    terminal.print("remaining time command(sec) = "); // above calcuation returns seconds due to /1000
    terminal.println(((activeTime * 60000)- min((currentMillis - lastCommandMillis), (activeTime * 60000)))/1000);
    terminal.print("timeDisplayFlag = ");
    terminal.println(timeDisplayFlag);
    terminal.print("randomPhraseDisplayFlag = ");
    terminal.println(randomPhraseDisplayFlag);
    terminal.println(); // add line
    terminal.flush();
    Blynk.run();
    return;
    }
   
// end of command waterfall
}

// for blinking MKR1010 built in LED 
void blinkOnBoardLED() 
{
if (onBoardLEDValue == 1) {
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  onBoardLEDValue = 0;
  }
else {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  onBoardLEDValue = 1;
  }
}

void blankDisplay()
{
int valueInput = 0; // 1 - yellow,  0 - black
for (int columnInput = 0; columnInput <= 49; columnInput++){ // 50 columns
  for (int rowInput = 0; rowInput <=6; rowInput++){
    setCell(columnInput, rowInput, valueInput);
    }
  }
}


void updateTimeDisplay()
{
// updates time display only if hours or minutes has changed
updateTime();
updateDate();
// check if new hour or mintues and update display if so  
if ((currentDisplayHours_int != hours_int) ||
  (currentDisplayMinutes_int != minutes_int)){
  // update months
  displayCharacter(0, months_str[0]);
  displayCharacter(1, months_str[1]);
  currentDisplayMonths_int = months_int;
  // update days
  displayCharacter(2, days_str[0]);
  displayCharacter(3, days_str[1]);
  currentDisplayDays_int = days_int;
  displayCharacter(4, ' '); // blank space
  // update hours
  currentDisplayHours_int = hours_int;
  // display hours
  displayCharacter(5, hours_str[0]);
  displayCharacter(6, hours_str[1]);
  // display separator at 7 - see below
  // update minutes
  currentDisplayMinutes_int = minutes_int;
  // display minutes
  displayCharacter(8, minutes_str[0]);
  displayCharacter(9, minutes_str[1]);
  }
if (secondsDisplayOn == 1){
  if (seconds_int % 2 != 0){
    displayCharacter(7, 0x02);  // left :
    }
  else {
    displayCharacter(7, 0x03);  // right :
    }
  }
else {
  displayCharacter(7, ':');  // center :
  }
}

void setTimeDisplay()
{
// updates time display to current time and turns time display on
currentDisplayHours_int = 999;  // cause update by using incorrect time 
currentDisplayMinutes_int = 999; 
timeDisplayFlag = 1; // turn on time display if not already on
updateTimeDisplay();
}

void clearTimeDisplay()
{
// clears time display and turns time display off
for (int displayNum = 0; displayNum <= 9; displayNum++){
  displayCharacter(displayNum, ' ');
  }
timeDisplayFlag = 0; // turn off time display to keep clear
}

void EEPROMWrite()
{
EEPROM.put(storedAddress, WRITTEN_SIGNATURE);
strcpy(controller.eeprom_ssid, ssid);
strcpy(controller.eeprom_pass, pass);
strcpy(controller.eeprom_k_GMT, k_GMT_str);
strcpy(controller.eeprom_enableStartHours, enableStartHours_str);
strcpy(controller.eeprom_enableEndHours, enableEndHours_str);
strcpy(controller.eeprom_activeTime, activeTime_str);
strcpy(controller.eeprom_randomPhraseDisplayFlag, randomPhraseDisplayFlag_str);
// Save everything into EEPROM
EEPROM.put(storedAddress + sizeof(signature), controller);
if (!EEPROM.getCommitASAP()){
  //Serial.println("CommitASAP not set. Need commit()");
  EEPROM.commit();
  }
// Print a confirmation of the data written.
terminal.println("Updated Data: ");
terminal.print("ssid ");
terminal.println(controller.eeprom_ssid); 
terminal.print("pass: ");
terminal.println(controller.eeprom_pass); 
terminal.print("k_GMT: ");
terminal.println(controller.eeprom_k_GMT);
terminal.print("enableStartHours: ");
terminal.println(controller.eeprom_enableStartHours);
terminal.print("enableEndHours: ");
terminal.println(controller.eeprom_enableEndHours);
terminal.print("activeTime: ");
terminal.println(controller.eeprom_activeTime);
terminal.print("randomPhraseDisplayFlag: ");
terminal.println(controller.eeprom_randomPhraseDisplayFlag);
terminal.println();
terminal.flush();
Blynk.run();
}

void displayCharacter(int charNum, char charOut)
{
get_char_fd_array(charOut); // populates fd_array with charOut
for (int columnInput = 0; columnInput <= 4; columnInput++){
  for (int rowInput = 0; rowInput <= 6; rowInput++){
    int valueInput = fd_array[columnInput][rowInput]; // get dot value
    setCell(columnInput + (charNum * 5), rowInput, valueInput); // adjust columns by charNum
    }  
  }
return;
}

void createDisplayStrings()
{
// set up and initialize variables
char *pch; 
word_cnt = 0;
char* cmd_chopped = cmd_str + 1; // eliminate leading ';' 
strcpy(input_str, cmd_chopped); 
//"0123t 4567t 890t 1234t 567890123t 45678t 901234t"); // for testing
// split the string on these delimiters into "tokens"
strcpy(input_str2, input_str); // new string needed as strtok is destructive
pch = strtok (input_str2," ,");
sprintf(word_str[word_cnt], "%s", pch);
//Serial.println(word_str[word_cnt]);  // for testing
while (1)
  {
  pch = strtok (NULL, " ,"); // continue to split the string on these delimiters
  if (pch == NULL) {break;}  // this configuration was needed for while loop to work
  word_cnt++;
  sprintf(word_str[word_cnt], "%s", pch);
  }
// print out words  for testing
//Serial.println(input_str);
//Serial.println(word_cnt+1);
//Serial.println();
for (int i = 0; i <= word_cnt; i++) {  // note - first word is [0]
  word_str_length[i] = strlen(word_str[i]);
  if (word_str_length[i] >= row_length - 2) { 
    // above subtract two to add '-' and due to starting row posiiton at 0 not 1
    // make room for additional word
    for (int j = word_cnt; j > i; j--) {
      strcpy(word_str[j+1], word_str[j]);
      }
    // trim end and hyphenate first part
    strcpy(word_str[i+1], word_str[i]);
    word_str[i][11] = '-';
    word_str[i][12] = '\0';
    // copy and trim front of second part 
    for (int m = 0; m <= (word_str_length[i] - (row_length - 1)); m++){
      word_str[i+1][m] = word_str[i+1][m + (row_length - 1)]; // capture null character
      //Serial.println(word_str[i+1][m]);
      }
    // update new string lengths
    word_str_length[i] = strlen(word_str[i]);
    word_str_length[i+1] = strlen(word_str[i+1]);
    // update loop variables for added word
    i++;;
    word_cnt++;
    //Serial.println(word_str_length[i]);
    }
  }
// for testing
//for (int i = 0; i <= word_cnt; i++) { 
//  Serial.print(word_str[i]);
//  Serial.print("  ");
//  Serial.println(word_str_length[i]);
//  }
//Serial.println();
// assemble row lines
int new_row_flag = 1;
row_number = 0;
for (int i = 0; i <= word_cnt; i++) {
  if (new_row_flag == 1){ // this really is only for the first row
    strcpy(row_display_array[row_number], word_str[i]);
    //Serial.println(row_display_array[row_number]);
    new_row_flag = 0;
    }
  else { 
    if ((strlen(row_display_array[row_number]) + word_str_length[i]) <= (row_length - 1)) { 
      // row length includes '\0'
      // two '\0' make up for space and last '\0'
      strcat(row_display_array[row_number], " ");  // add space between words
      strcat(row_display_array[row_number], word_str[i]);
      //Serial.println(row_display_array[row_number]);
      }
    else {  // make new row
      row_number++; 
      strcpy(row_display_array[row_number], word_str[i]);
      //Serial.println(row_display_array[row_number]);
      new_row_flag = 0;
      }
    }
  }
// add spaces to fill each row
for (int i = 0; i <= row_number; i++){
  int k;
  int j = row_length - 1 - strlen(row_display_array[i]);
  for (k = 0; k <= j; k++) {
    strcat(row_display_array[i], " ");  // add space after word(s)
    }
  //Serial.print(row_display_array[i]);
  //Serial.println("end");
  }

// display each row with delay
for (int i = 0; i <= row_number; i++){
  for (int charNum = 0; charNum <=9; charNum++){ 
    displayCharacter(charNum, row_display_array[i][charNum]); // [32][20]
    }
  delay3s();
  }
}

// delays that maintain led connection 
//   and WDT clearing
void delay25ms(){ 
  currentMillis = millis();
  if(currentMillis - previousMillis_blinkOnBoardLED >= 1000) {
    previousMillis_blinkOnBoardLED = currentMillis;  // Remember the time
    blinkOnBoardLED();
    }
  delay(25);
  WDTimer.clear();
  }
  
void delay30ms(){delay(5); delay25ms();}
void delay50ms(){delay(25); delay25ms();}
void delay75ms(){delay(50); delay25ms();}
void delay100ms(){delay(75); delay25ms();}
void delay150ms(){delay(100); delay50ms();}
void delay200ms(){delay(175); delay25ms();}
void delay250ms(){delay(225); delay25ms();}
void delay300ms(){delay(275); delay25ms();}
void delay400ms(){delay(375); delay25ms();}
void delay500ms(){delay250ms(); delay250ms();}
void delay750s(){delay500ms(); delay250ms();}
void delay1s(){delay500ms(); delay500ms();}
void delay2s(){delay1s(); delay1s();}
void delay3s(){delay2s(); delay1s();}
void delay4s(){delay2s(); delay2s();}
void delay5s(){delay3s(); delay2s();}
void delay6s(){delay4s(); delay2s();}
void delay7s(){delay4s(); delay3s();}
void delay8s(){delay4s(); delay4s();}
void delay9s(){delay5s(); delay4s();}
void delay10s(){delay5s(); delay5s();}
