/*
 * Grillotron 0.1.4

 * probe, output and PID arrays created
* next up: data logging, simple PID algorithm, lid open output
* functions to update temperatures outside mainloop, functions to update / activate outputs
 */
//include i2c, include i2c lcd, include max6675 thermocouples, include ESP8266 wifi for server availability

#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD
#include <max6675.h> // library for thermocouples
#include <ESP8266WiFi.h> //wifi library to serve data to apps

/*
 * create SPI pin standards
 */
const int CLKpin = 14;
const int DOpin = 13;
const String version = "0.1.4";
/*
 * define several arrays relating to probes and outputs
 * maxprobes and maxoutputs define max no of probes and outputs
 * probe and output indices are P1 = 0, etc. so just relate to elements
 * probeTypes is a bool : 0 = grate, 1=meat
 * probeTemps has three columns per probe; setTemp, currentTemp, pastTemp (last temperature measured) for purposes of PID / pid-like algorithm.
 * probetemps = float for precision
 * probePins has the CS pin for the MAX6675 associated CS pin (see above ints CLKpin and DOpin)
 * probeOutput allows us to associate output to probe, entailing that a probe stores associated output index number to allow update.
 * outPuttypes defined as int to allow more options. 0 (standard) = on/off fan
 * outputFreqs has two fields, currentFreq and pastFreq (to allow verification against last version)
 * outputFreqs = float for precision. Current assumptions: freq = speed/60 (PWM?)
 * indices should match as much as possible
 *
 */
const int maxprobes = 6; //easily used in for n = 0 to maxprobes-1
const int maxoutputs = 6; // easily used in for n = 0 to maxoutputs-1

/*
 * PROBES ARRAYS
 *
 */
//probe types (meat or grate) idea is to have target temps associated with type (i.e. 0 gives range of 100C to 400 C (smoker - grill) while 1 gives range of 48-95)
boolean probeTypes [maxprobes]; //defaults to all meat probes (0)
double probeTemps [maxprobes][3]; // settemp, currenttemp, pasttemp

int probePins [maxprobes]= {15}; //CS pins
int probeOutput [maxprobes]; //associated outputs

double probes [maxprobes][7]; // idea for probes array which has maxprobes rows and 6 columns: [n][CSpin, probeType,probeOutput, settemp, currenttemp, pasttemp, PID]
/*
 * OUTPUT ARRAYS
 * outputs still need to be mapped. need to examine PID ranges and map these to proper way to operate types of outputs
 * types of outputs: air pump, fan, others imaginable
 * outputFreqs currently has no specific function but could contain mapped values from PID
 */
int outputTypes [maxoutputs];
double outputFreqs [maxoutputs][2]; // allows particular factors to be applied
int outputPins [maxoutputs];

/*
 * PID associated with probes - i.e. a PID uses a probe and can then be mapped to an output
 * array used to allow multiple PIDs
 * PID[maxprobes number of rows][kp,ki,kd,p,i,d,output]
 */

double PID [maxprobes][7] = {1,0.1,0.05,0,0,0,0};
double correctionF = 0.0001; //correction factor to account for continuous input of fuel - increments slowly
unsigned long previousTime;

//LCD
int displayPos[maxprobes/2][3] = {{0,6,12},{1,2,3}};
//{{0,1},{0,2},{0,3}{6,1},{6,2},{6,3}}; //column and line positions per probe for display (should only use lines 1-3 and reserve 0 for system status / scroll
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4);

/*
 * lidOpen boolean and timeSincelidOpen exist to ensure outputs are not turned on in case temperature spikes / drops occur within 5 min from lid opening and closing
 *
 */
boolean lidOpen; //false / true, if true update output should be 0.
unsigned long timeSincelidOpen[2]; // timer for lid open. wait at least 5 min. before restarting outputs following lid open. Value [0]= actual time value [1] = delta

/*
 *  LCD supports custom characters
 */
//custom characters for lcd
//degrees c symbol
byte degC[] = {
  B01000,
  B10100,
  B01000,
  B00000,
  B00011,
  B00100,
  B00100,
  B00011
};
byte degF[] = {
  B01000,
  B10100,
  B01000,
  B00000,
  B00111,
  B00100,
  B00110,
  B00100
};
// probe identifiers (max 6 probes currently but theoretically unlimited)
byte p1[] = {
  B00001,
  B00001,
  B00001,
  B00001,
  B11001,
  B11000,
  B10000,
  B10000
};
byte p2[] = {
  B00011,
  B00001,
  B00011,
  B00010,
  B11011,
  B11000,
  B10000,
  B10000
};
byte p3[] = {
  B00011,
  B00001,
  B00011,
  B00001,
  B11011,
  B11000,
  B10000,
  B10000
};
byte p4[] = {
  B00010,
  B00010,
  B00011,
  B00001,
  B11001,
  B11000,
  B10000,
  B10000
};
byte p5[] = {
  B00011,
  B00010,
  B00011,
  B00001,
  B11011,
  B11000,
  B10000,
  B10000
};

byte p6[] = {
  B00011,
  B00010,
  B00011,
  B00011,
  B11011,
  B11000,
  B10000,
  B10000
};
/*
 * Main declarations etc. and instantiating classes
 */

WiFiServer server(80);

// create MAX6675 objects
//int probes[maxprobes];
//*MAX6675 probe[maxprobes]={MAX6675(CLKpin, probePins[0],DOpin)},{MAX6675(CLKpin, probePins[0],DOpin)},{MAX6675(CLKpin, probePins[0],DOpin)},{MAX6675(CLKpin, probePins[0],DOpin)},{MAX6675(CLKpin, probePins[0],DOpin)},{MAX6675(CLKpin, probePins[0],DOpin)};
//MAX6675 TCa= MAX6675(CLKpin, probePins[0], DOpin);
MAX6675 probe[]= {MAX6675(CLKpin, probePins[0], DOpin),MAX6675(CLKpin, probePins[0], DOpin)};//manually declare them all.

void setup() {
  // Initiate the LCD:
  lcd.init();
  lcd.backlight();
  delay(500);
//init probes at default
  probeTemps[0][0]=21.5;


//create custom chars
//degreeC icon, fan icon (show fan status), probes 1-5 (subscript p, superscript number) and flame icon

  lcd.createChar(0, p1);
  lcd.createChar(1, p2);
  lcd.createChar(2, p3);
  lcd.createChar(3, p4);
  lcd.createChar(4, p5);
  lcd.createChar(5, p6);
  lcd.createChar(6, degF);
  lcd.createChar(7, degC);

//setup wifi connection
  WiFi.hostname("Grillotron");
  WiFi.begin("ZiggoA7725", "ncf3WspTLqEH");


}

void loop() {

  //call updates for temperature, PID and output updates
  tempUpdate();
  PIDupdate();
  //call screen update
  displayUpdate();
  //delay updates
  delay(400);
}

void tempUpdate(){

  for (int n = 0; n<maxprobes; n++){
    // update pasttemp array space to last known currenttemp
    probeTemps[n][2]=probeTemps[n][1];
    //update currenttemp
    probeTemps[n][1]=probe[n].readCelsius();
  }
}
void outputUpdate(){
  if (lidOpen=0){
    for (int n=0; n<maxoutputs; n++){
    }

  }
}
void PIDupdate(){
 unsigned long currentTime = millis();
 unsigned long elapsedTime = currentTime-previousTime;
 previousTime = currentTime;

    for (int n=0; n<maxprobes; n++){
    //P
    PID[0][3] = probeTemps[0][0]-probeTemps[0][1];
    //I
    PID[0][4] += PID[0][3] * elapsedTime;
    //D
    PID[0][5] = (PID[0][3]- (probeTemps[0][0]-probeTemps[0][2]))/elapsedTime;

    //compute PID output factor
    PID[0][6]= PID[0][0]*PID[0][3] + PID[0][1]*PID[0][4] + PID[0][2]*PID[0][5];
    //n]=op;//*(1-correctionF);
    //correctionF=correctionF+0.0001;

    }
}

void displayUpdate(){

  lcd.setCursor(0,0);
  lcd.print (version);
  lcd.write (7);
  // print probe temps
  //probe 1
  lcd.setCursor(0, 1); // Set the cursor on the first column and first row.
  lcd.write (0);
   lcd.print(probeTemps[0][1]);
  lcd.write(7);
  lcd.setCursor(0,2);
  lcd.print ("PT");
  lcd.print (probeTemps[0][2]);
  lcd.write(7);
  lcd.setCursor (0,3);
  lcd.print ("ST");
  lcd.print (probeTemps[0][0]);
  lcd.write(7);
  lcd.setCursor (10,1);
  lcd.print (PID[0][6]);
  //targettemp
}
