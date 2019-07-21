#include <EEPROM.h>

#include <NMEA2000.h>
#include <N2kMsg.h>
#include <N2kMessages.h>
#include <FlexCAN.h>
#include <NMEA2000_teensy.h> // https://github.com/sarfata/NMEA2000_teensy>

tNMEA2000_teensy NMEA2000;


#define LED_PIN 13
#define CALIBRATE_PIN 14
#define WIND_ANGLE_PIN 7






#define ANGLE_FROM_COMPASS 1
#define ANGLE_FROM_RESISTOR 2
#define ANGLE_SOURCE ANGLE_FROM_RESISTOR

#define ENABLE_WIND_INPUT 1
#define ENABLE_TEMP 1

#define DEBUG_WIND 1
#define DEBUG_ANGLE 1
#define DEBUG_NMEA2000 0
#define DEBUG_TEMP 0

#define TEMP_SEND_EVERY_NTH 8

#define CALIBRATE_PIN_NEEDS_ROUNDS 100 // every round ~ 50ms

#define NUMBER_OF_AVG_MAST 10

float OHM_OTHER_RESISTOR = 82;
float ANGLE_VOLTAGE = 5.0;

float OHM_MIN_ANGLE = 0;
float OHM_MAX_ANGLE = 190;
float OHM_CABLE = 5.5;
float ANGLE_FOR_MIN= -45;
float ANGLE_FOR_MAX = 45;



// for averaging the mast-rotation over the last few seconds
double mastvalues[NUMBER_OF_AVG_MAST];
int mastIdx = 0;

// buffers to collect complete lines from the different connected ttys
String windInputBuffer = "";
String solarInputBuffer = "";
String batteryInputBuffer = "";
String bluetoothInputBuffer = "";

float lastWindAngleUncorrected;


// ID of the settings block
#define CONFIG_VERSION "ws3"
#define CONFIG_START 32

#define SERIAL_WAIT_MILLIS 40
#define SERIAL_WAIT_STEP_MILLIS 1


/*
 * 
 * data structure for configuration storage
 * 
 */
 
struct ConfigurationStoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  double offsetMastSensors;
} storage = {
  CONFIG_VERSION,
  0
};

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;



void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  Serial.println("Loading Config2");
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {
    for (unsigned int t = 0; t < sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
    Serial.println("Loaded");
  }

}

void saveConfig() {
  for (unsigned int t = 0; t < sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}






/*
 * TSP
 */


#define TWS_SIZE 11
#define TWD_SIZE 12
float twsBase[TWS_SIZE] = { 0, 4, 6, 8, 10, 12, 14, 16, 20, 25, 30 } ;
//float twdBase[TWD_SIZE] = { 0, 5, 10, 15, 20, 25, 32, 36, 40, 45, 52, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180 } ;
float twdBase[TWD_SIZE] = { 0, 5, 10, 15, 20, 25, 32, 36, 40, 45, 52, 60 } ;
float polars[TWD_SIZE][TWS_SIZE] = { 
{ 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00 },
{ 0.00, 0.10, 0.20, 0.30, 0.40, 0.40, 0.50, 0.50, 0.50, 0.50, 0.40 },
{ 0.00, 0.30, 0.40, 0.60, 0.70, 0.90, 0.90, 0.90, 0.90, 0.90, 0.70 },
{ 0.00, 0.40, 0.70, 0.90, 1.10, 1.30, 1.40, 1.40, 1.40, 1.40, 1.10 },
{ 0.00, 0.50, 0.80, 1.00, 1.30, 1.50, 1.60, 1.60, 1.60, 1.60, 1.30 },
{ 0.00, 0.60, 0.90, 1.20, 1.50, 1.80, 1.90, 1.90, 1.90, 1.90, 1.50 },
{ 0.00, 1.00, 1.50, 2.00, 2.50, 3.00, 3.10, 3.10, 3.10, 3.10, 2.50 },
{ 0.00, 1.50, 2.30, 3.00, 3.80, 4.60, 4.60, 4.60, 4.70, 4.70, 3.70 },
{ 0.00, 2.00, 3.00, 4.10, 5.10, 6.10, 6.10, 6.20, 6.20, 6.20, 5.00 },
{ 0.00, 2.10, 3.20, 4.20, 5.30, 6.40, 6.40, 6.40, 6.50, 6.50, 5.20 },
{ 0.00, 2.70, 4.10, 5.50, 6.80, 8.20, 10.00, 11.80, 14.40, 14.40, 11.60 },
{ 0.00, 3.00, 4.50, 6.00, 7.50, 9.00, 11.00, 13.00, 16.00, 16.00, 12.80 },
/*
 * { 0.00, 3.50, 5.30, 7.00, 8.80, 10.50, 12.50, 14.40, 17.30, 17.30, 13.90 },
{ 0.00, 4.40, 6.60, 8.80, 11.00, 13.20, 14.70, 16.10, 18.20, 18.20, 14.60 },
{ 0.00, 5.00, 7.50, 10.00, 12.50, 15.00, 16.00, 17.00, 18.50, 18.50, 14.80 },
{ 0.00, 4.90, 7.40, 9.90, 12.30, 14.80, 15.70, 16.60, 17.90, 17.90, 14.30 }
,
{ 0.00, 4.60, 7.00, 9.30, 11.60, 13.90, 14.90, 16.00, 17.50, 17.50, 14.00 },
{ 0.00, 4.30, 6.50, 8.70, 10.80, 13.00, 14.40, 15.90, 18.00, 18.00, 14.40 },
{ 0.00, 4.30, 6.50, 8.60, 10.80, 13.00, 15.00, 17.00, 20.00, 20.00, 16.00 },
{ 0.00, 3.60, 5.40, 7.30, 9.10, 10.90, 13.40, 15.80, 19.50, 19.50, 15.60 },
{ 0.00, 3.30, 4.90, 6.60, 8.20, 9.90, 12.10, 14.40, 17.70, 17.70, 14.20 },
{ 0.00, 3.00, 4.50, 6.00, 7.50, 9.00, 11.00, 13.00, 16.00, 16.00, 12.80 },
{ 0.00, 2.80, 4.20, 5.60, 7.00, 8.40, 10.30, 12.10, 14.80, 14.80, 11.80 },
{ 0.00, 2.70, 4.00, 5.30, 6.70, 8.00, 9.70, 11.40, 14.00, 14.00, 11.20 }
*/
};
float getPolarValue(int tws, int twd ) {
  return polars[twd][tws];
}
float getPolarWeight(int twsIndex, int twdIndex, float twsReal, float twdReal) {
  float tws = twsBase[twsIndex];
  float twd = twdBase[twdIndex];
  return 1 / ( abs(twd-twdReal) + abs(tws-twsReal) ); 
}

float findTargetSpeed(float tws, float twd) {
   
   if ( twd > 180 ) { twd = twd-360; }
   twd = abs(twd);
   
   int twsIndex = 0;
   while ( twsBase[twsIndex] < tws && twsIndex < TWD_SIZE-1 ) { twsIndex++; }
   int twsIndexLeft = twsBase[twsIndex] == tws ? twsIndex : twsIndex-1;
   int twsIndexRight = twsIndex;
  
   int twdIndex = 0;
   while ( twdBase[twdIndex] < twd && twdIndex < TWD_SIZE-1 ) { twdIndex++; }
   int twdIndexTop = twdBase[twdIndex] == twd ? twdIndex : twdIndex-1;
   int twdIndexBottom = twdIndex;

   float sum = 0;
   float sumWeight = 0;
   
   sum += getPolarValue(twsIndexLeft, twdIndexTop) * getPolarWeight(twsIndexLeft, twdIndexTop, tws, twd);
   sumWeight += getPolarWeight(twsIndexLeft, twdIndexTop, tws, twd);

   sum += getPolarValue(twsIndexLeft, twdIndexBottom) * getPolarWeight(twsIndexLeft, twdIndexBottom, tws, twd);
   sumWeight += getPolarWeight(twsIndexLeft, twdIndexBottom, tws, twd);

   sum += getPolarValue(twsIndexRight, twdIndexTop) *  getPolarWeight(twsIndexRight, twdIndexTop, tws, twd);
   sumWeight += getPolarWeight(twsIndexRight, twdIndexTop, tws, twd);
   
   sum += getPolarValue(twsIndexRight, twdIndexBottom)*getPolarWeight(twsIndexRight, twdIndexBottom, tws, twd);
   sumWeight += getPolarWeight(twsIndexRight, twdIndexBottom, tws, twd);

   float avg = sum / sumWeight;
   
   return avg;
}


/*
 * LOGGING
 */
 
void llog(String s) {
  //Serial.print(s);h
}

void llog(double s) {
  //Serial.print(s);
}


void llogln(String s) {
  //Serial.println(s);
}






/***
 * Handler for updates from/to NMEA2000 network
 * I want to log some stuff to the sd card for later analysis
 * (polar creation)
 ***/

void SystemDateTimeHandler(const tN2kMsg &N2kMsg);
void WindHandler(const tN2kMsg &N2kMsg);
void BoatSpeedHandler(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {126992L,&SystemDateTimeHandler},
  {130306L,&WindHandler},
  {128259L,&BoatSpeedHandler},
  {127257L,&AttitudeHandler},   
  {0,0}
};




double LastBoatHeel = 0;

void AttitudeHandler(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double Yaw;
  double Pitch;
  double Roll;
  if ( ParseN2kAttitude(N2kMsg, SID, Yaw, Pitch, Roll) ) {
    LastBoatHeel = Roll;
    debug("Heel ");
    debugln(LastBoatHeel);
  }
  
}


void SystemDateTimeHandler(const tN2kMsg &N2kMsg) {
  
  uint16_t SystemDate;
  unsigned char SID;
  double SystemTime;
  tN2kTimeSource TimeSource;
                     
  if (ParseN2kPGN126992(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) {
      llog("PGN126992");
      if (SystemDate>0) {
        llog("|SystemDate:");
        llog(SystemDate);
      }
      if (SystemTime>0) {
        llog("|SystemTime:");
        llog(SystemTime);
      }
      if (TimeSource>0) {
        llog("|TimeSource:");
        llog(TimeSource);
      }
      llogln("");
    }
}

void WindHandler(const tN2kMsg &N2kMsg) {
  
  unsigned char SID;
  double WindSpeed;
  double WindAngle;
  tN2kWindReference WindReference;
                     
  if ( ParseN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference) ) {
      llog("PGN130306");
     
        llog("|WindSpeed:");
        llog(WindSpeed);
   
        llog("|WindAngle:");
        llog(WindAngle);
   
        llog("|WindReference:");
        llog(WindReference);
     
      llogln("");
    }
}


void BoatSpeedHandler(const tN2kMsg &N2kMsg) {
  
  unsigned char SID;
  double WaterRefereced;
  double GroundReferenced;
  tN2kSpeedWaterReferenceType SWRT;

  if ( ParseN2kPGN128259(N2kMsg, SID, WaterRefereced, GroundReferenced, SWRT) ) {
      llog("PGN130306");
      if (WaterRefereced>0) {
        llog("|WaterReferenced:");

        if ( WaterRefereced>0 ) {
          reprocessBoatSpeed(WaterRefereced);
        }
        llog(WaterRefereced);
      }
      if (GroundReferenced>0) {
        llog("|GroundReferenced:");
        llog(GroundReferenced);
      }
      if (SWRT>0) {
        llog("|SWRT:");
        llog(SWRT);
      }
      llogln("");
    }
}
 

void reprocessBoatSpeed(double WaterReferenced) {
  double heelAngleDegree = RadToDeg(LastBoatHeel);
  double realSpeed = WaterReferenced;
  if ( heelAngleDegree >  5 && heelAngleDegree <  20 ) {
    realSpeed = realSpeed * (1 + (heelAngleDegree-5)/(20-5) * 0.3 );
  }

  tN2kMsg N2kMsg;
  SetN2kBoatSpeed(N2kMsg, 1, realSpeed, N2kDoubleNA, N2kSWRT_Paddle_wheel);
  NMEA2000.SendMsg(N2kMsg);
 
}



void WindHandler2(const tN2kMsg &N2kMsg) {
  
  unsigned char SID;
  double WindSpeed;
  double WindAngle;
  tN2kWindReference WindReference;
                     
  if ( ParseN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference) ) {
      llog("PGN130306");
        llog("|WindSpeed:");
        llog(WindSpeed);
   
        llog("|WindAngle:");
        llog(WindAngle);
   
        llog("|WindReference:");
        llog(WindReference);


        if ( 1 || WindReference == N2kWind_True_boat ) {

          WindSpeed = MetersPerSecondToKnots(WindSpeed);
  //7Serial.println("debug");
          WindAngle = RadToDeg(WindAngle);
 // Serial.println(WindAngle);
//  Serial.println(WindSpeed);
  
          double targetSpeed = KnotsToMetersPerSecond(findTargetSpeed(WindSpeed,  WindAngle)); 
//Serial.println(targetSpeed);
          tN2kMsg N2kMsg;
          NMEA2000.SendMsg(N2kMsg);
          SetN2kPGN128259(N2kMsg, 1, targetSpeed, targetSpeed, N2kSWRT_Paddle_wheel);
           NMEA2000.SendMsg(N2kMsg);
        }
      llogln("");
    }
}



//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}






void setup() {

  // Console
  Serial.begin(115200);
  loadConfig();

  // Set Product information
  NMEA2000.SetProductInformation("00000004", // Manufacturer's Model serial code
                                 105, // Manufacturer's product code
                                 "Wind System 2",  // Manufacturer's Model ID
                                 "1.0.0.11 (2016-02-10)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2016-02-10)" // Manufacturer's Model version
                                );
  NMEA2000.SetDeviceInformation(1231, // Unique number. Use e.g. Serial number.
                                130, // Device function=Temperature. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                85, // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                1976 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  // NMEA0183 Wind
  Serial3.begin(4800, SERIAL_8N1);
  windInputBuffer.reserve(200);

  // NMEA2000 CAN-Bus
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  NMEA2000.SetForwardOwnMessages();
  if ( DEBUG_NMEA2000 ) {
    NMEA2000.EnableForward(true);
  } else {
    NMEA2000.EnableForward(false);
  }
  
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 24);//NORMAL
  //NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 24);
  
  Serial.println("debug");
  NMEA2000.Open();

  Serial.println("opened");

  // Info-LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

 Serial.println("card initialized.");
 
    pinMode(A0, INPUT);

}


int resendInitializationEveryMillis = 10000;
int resendInformationCounter = resendInitializationEveryMillis;

int calibratePinCounter = CALIBRATE_PIN_NEEDS_ROUNDS;

void loop() {
  NMEA2000.ParseMessages();

  int calibrationButtonValue = digitalRead(CALIBRATE_PIN);

  resendInformationCounter--;
  
  if ( resendInformationCounter <= 0 ) {
    
    NMEA2000.SendIsoAddressClaim(0xff,0);
    NMEA2000.SendProductInformation(0);
    NMEA2000.SendConfigurationInformation(0);
    resendInformationCounter = resendInitializationEveryMillis;
    NMEA2000.Open();  
    
    
  } 
  
  else if ( calibrationButtonValue == HIGH && 0) {

    delay(50);
    if ( calibratePinCounter-- == 0 ) {
      calibrateAngles();
    }    
    
 } else {
    calibratePinCounter = CALIBRATE_PIN_NEEDS_ROUNDS;
    resendInformationCounter -= 50;
    delay(50);
  }
  


}





// Blinking lights 


void signalLong() {
  Serial.println("LONG");
  digitalWrite(LED_PIN, HIGH);
  delay(5000);
  digitalWrite(LED_PIN, LOW);
}

void signalStart() {
  Serial.println("START");
  for ( int i = 0; i < 10; i++ ) {
    Serial.println(i);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void signalBlinkShort(int millis = 200) {
  for ( int i = 0; i < millis; i += 100 ) {
    Serial.println(i);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
}






// Calibration

int calibrateAnglesMinus() {

  storage.offsetMastSensors =  storage.offsetMastSensors - 1;
  Serial.print("Update Mast-Correction to ");
  Serial.print(storage.offsetMastSensors);
  Serial.println(".");
  saveConfig();
  
  
}

int calibrateAnglesPlus() {

  storage.offsetMastSensors =  storage.offsetMastSensors + 1;
  Serial.print("Update Mast-Correction to ");
  Serial.print(storage.offsetMastSensors);
  Serial.println(".");
  saveConfig();
  
}

int calibrateAngles() {

  signalStart();
  Serial.println("Start calibration");

  float offsetSum = 0;
  int offsetCount = 0;

  for ( int i = 0; i < 10; i++ ) {

    float mastAngle = readMastAngle();
    debug("got mastAngle ");
    debugln(mastAngle);

    float windAngle  = lastWindAngleUncorrected;
    debug(" and windAngle ");
    debugln(windAngle);

    float offset = - windAngle - mastAngle;
    debug(" = offset ");
    debugln(offset);

    offsetCount++;
    offsetSum += offset;

    signalBlinkShort(500);
  }

  float offsetAverage = offsetSum / offsetCount;
  debug(" = offsetAverage ");
  debugln(offsetAverage);

  
  storage.offsetMastSensors = offsetAverage;

  saveConfig();
  debugln("Stored");

  signalLong();

}





/*
   Rotation Sensing
*/


double readMastAngle() {
   if ( ANGLE_SOURCE == ANGLE_FROM_RESISTOR ) {
    int    val = analogRead(A1);    // read the input pin

float volt_over_sensor = (float)val / 1024.0 * ANGLE_VOLTAGE;
float volt_other = ANGLE_VOLTAGE - volt_over_sensor;
float ohm_sensor  = OHM_OTHER_RESISTOR / volt_other * volt_over_sensor - OHM_CABLE; 



float angle = ( ohm_sensor - OHM_MIN_ANGLE ) / ( OHM_MAX_ANGLE - OHM_MIN_ANGLE ) * ( ANGLE_FOR_MAX - ANGLE_FOR_MIN) + ANGLE_FOR_MIN;
if ( DEBUG_ANGLE ) {
  Serial.println("Angle");
  Serial.println(val);             // debug value
  Serial.println(volt_over_sensor);             // debug value
    Serial.println(ohm_sensor);             // debug value
}
 return angle;
   }

}





/*
    Input from the main tty
*/

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();

    if ( inChar == 'c' ) {
      calibrateAngles();
    }
    else if ( inChar == 'p' ) {
      calibrateAnglesPlus();
    }
    else if ( inChar == 'm' ) {
      calibrateAnglesMinus();
    }
  }
}


/*
    Wind Input
*/

void serialEvent3() {

  while (Serial3.available()) {
    // get the new byte:
    char inChar = (char)Serial3.read();
       
    processWindCharacter(inChar);
  }
}

void processWindCharacter(char inChar) {

  // add it to the windInputBuffer:
  windInputBuffer += inChar;
  
  // if the incoming character is a newline, set a flag
  // so the main loop can do something about it:

  int ascii = inChar;

  if (inChar == '\n' || inChar == '\r' ) {
    processNMEA0183Line(windInputBuffer);
    windInputBuffer  = "";
  }

}


double avgMastvalues(double newVal) {

  mastvalues[mastIdx++] = newVal;
  if ( mastIdx == NUMBER_OF_AVG_MAST ) {
      mastIdx = 0;
  }
  double sum = 0;
  for (int i=0; i<NUMBER_OF_AVG_MAST; i++ ) {
    sum += mastvalues[i];
    if (DEBUG_ANGLE) {
      if(i == mastIdx-1) { debug("*"); }
      debug("mv ");
      debug(i);
      debug(" ");
      debug(mastvalues[i]);
      debugln("");
    }
  }
  return sum / NUMBER_OF_AVG_MAST;
}


int tempReceiveCounter = 0;

void processNMEA0183Line(String s) {

  if ( ENABLE_TEMP && getValue(s, ',', 0).equals("$WIXDR") && tempReceiveCounter-- == 0 ) { // for temp-lines

    tempReceiveCounter = TEMP_SEND_EVERY_NTH;
    
    String tempString = getValue(s, ',', 2);
    double temp = StringToDouble(tempString);
    
    if ( DEBUG_TEMP ) {
      debug("got temp ");
      debugln(temp);
    }  

    if ( DEBUG_TEMP || DEBUG_NMEA2000 ) {
        debug("nmea-send temp, temp ");
        debugln(temp);
    }
    
    tN2kMsg N2kMsg;
    SetN2kTemperature(N2kMsg, 1, 1, N2kts_OutsideTemperature, CToKelvin(temp));
    NMEA2000.SendMsg(N2kMsg);

  }
  
  else if ( 
    ( ENABLE_WIND_INPUT && getValue(s, ',', 0).equals("$IIMWV")  ) ||   
    ( ENABLE_WIND_INPUT && getValue(s, ',', 0).equals("$WIMWV")  ) 
  ) { // for wind-lines

    String windAngleString = getValue(s, ',', 1);
    String windSpeedString = getValue(s, ',', 3);

    double windAngle = StringToDouble(windAngleString);
    lastWindAngleUncorrected = windAngle;

    double windSpeed = StringToDouble(windSpeedString);

    double mastAngle = avgMastvalues(readMastAngle());


    if ( DEBUG_WIND ) {
      debug("got wind, speed ");
      debug(windSpeed);
      debug(", angle uncorrected ");
      debug(windAngle);
      debug(", mastAngle ");
      debug(mastAngle);
      debug(", offset ");
      debug(storage.offsetMastSensors); 
    }

    windAngle = windAngle + mastAngle + storage.offsetMastSensors;
    windAngle = fmod(windAngle, 360.0);

    windAngle = round(windAngle);
    if (windAngle < 0 ) {
      windAngle = 360 + windAngle;
    }

    if ( DEBUG_WIND ) {
      debug(", calculated ");
      debugln(windAngle); 
    }

    tN2kMsg N2kMsg;

    double windAngleInRad = DegToRad(windAngle);
    if  (  windSpeed > -1 ) {
     
      if ( DEBUG_WIND || DEBUG_NMEA2000 ) {
        debug("nmea-send wind, speed ");
        debug(windSpeed);
        debug(", angle ");
        debugln(windAngle);
      }
 
      SetN2kWindSpeed(N2kMsg, 1, KnotsToMetersPerSecond(windSpeed), windAngleInRad, N2kWind_Apprent);
      NMEA2000.SendMsg(N2kMsg);
    
      WindHandler(N2kMsg);
      
    }

  }

}


double KnotsToMetersPerSecond(double knots) {
  return knots * 0.514444;
}

double MetersPerSecondToKnots(double mps) {
  return mps / 0.514444;
}




/*
    Helper
*/



double StringToDouble (String s) {
  char buf[15]; //make this the size of the String
  s.toCharArray(buf, 15);

  char *endptr;
  float num;
  num = strtod(buf, &endptr);
 // if (*endptr != '\0')
  //  return -1;
  return num;
}


String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1  };
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void debugln(String s) {
  Serial.println(s);
}


void debugln(double s) {
  Serial.println(s);
}


void debug(String s) {
  Serial.print(s);
}


void debug(double s) {
  Serial.print(s);

}
