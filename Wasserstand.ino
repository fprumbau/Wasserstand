#include <AdjustableButtonConfig.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <ButtonConfig.h>
#include <ArduinoJson.h>
#include <SimpleDHT.h>
#include <AceButton.h>
#include <TimerOne.h> 
#include <EEPROM.h>
#include <Wire.h>

using namespace ace_button;

#define TRIG_PIN 6
#define ECHO_PIN 5
#define ARDUINO_RX 10
#define ARDUINO_TX 11
#define PHYST 3  //dann schaltet Relais wieder (z.B. bei Limit 67 wäre dies 67 + 3 = 70cm)
#define THYST 2 //ist die Temperatur um 2Grad unter die Grenze gefallen, wird wieder eingeschaltet
#define pinDHT22 7
#define BUTTON1_PIN 3 
#define BUTTON2_PIN 2 
#define relay 4 //relay turns trigger signal - active high
#define EEPROMaddress_PEGEL 130
#define EEPROMaddress_TEMP 140
#define MAX_LINE_LENGTH 100
#define LINE_1 0
#define LINE_2 1

int NIVEAU_UEBER_BODEN=154; //korrigiert von 350 (Wasserstand gemessen: 71cm)

//EEPROM is good 100.000 write /erase cycles
// 3,3ms per write; Uno == 1024 bytes, Mega == 4096 bytes

int PLIM = 66; //cm Wasser, wenn der Sensor 350cm über Grund aufgehängt wurde
int TLIM = 40; //Grad Celsius

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display  
unsigned long lastHell = -1;
bool displayHell = true;

String s1="";
String s2="";

String oldLine1 = "";
String oldLine2 = "";

//Temp. und Feuchtigkeit
SimpleDHT22 dht22(pinDHT22);


//fuer Standardsensor
//#include <NewPing.h>
//NewPing sonar(TRIG_PIN, ECHO_PIN, NIVEAU_UEBER_BODEN);

//Fuer DYP-ME007Y
SoftwareSerial dypSerial = SoftwareSerial(ECHO_PIN, TRIG_PIN);
byte read_buffer[4];
byte crcCalc;


bool debug = false;
bool debug2 = false;

ButtonConfig buttonConfig1;
AceButton button1(&buttonConfig1);

ButtonConfig buttonConfig2;
AceButton button2(&buttonConfig2);

unsigned long lastCheckValues=0;
unsigned long checkInterval = 1000;

//Konfigurationszeitraum in Millis
unsigned long KONFIG_TIME=10000;

//Pegellimitkonfiguration ist aktiv
bool pegelSet = false;
//Zeitraum der Pegellimitkonfiguration aktiv seit n millis
unsigned long setPegelLimit = 0;
//Eingestellter Wert des neuen Pegellimits, der nach der Konfigzeit ggfls übernommen wird
uint8_t tmpPegelLimit = 0;
bool pegelOk = true;
int buff=50;

//Temperaturlimitkonfiguration ist aktiv
bool tempSet = false;
//Zeitraum der Temperaturlimitkonfiguration aktiv seit n millis
unsigned long setTempLimit = 0;
//Eingestellter Wert des neuen Temperaturlimits, der nach der Konfigzeit ggfls übernommen wird
uint8_t tmpTempLimit = 0;
bool tempOk = true;

//ok-message, die beim Wiedereinschalten des Release gesendet wird
String okMessage="-";

uint8_t relayStatus;
//das Pumpenrelais darf höchstens alle 30s umgeschaltet werden
unsigned long lastRelaisAction;
unsigned long minRelaisActionInterval=30000;

//D8(13)[RX] -> Arduino D11[TX]
//D7(15)[TX] -> Arduino D10[RX]
SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX);

void handleEvent1(AceButton*, uint8_t, uint8_t);
void handleEvent2(AceButton*, uint8_t, uint8_t);

const char* minRelActionMessage = "@ Es sind noch keine 30s seit der letzten Relaisumschaltung vergangen, warte...";

volatile int pegelIndex=-1;
volatile float pegelBuffer[50] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 };
volatile float pegel = -1;

void setup() {

  initInterruptTimer1();

  Serial.begin(115200); //USB
  mySerial.begin(19200); //NodeMCU8266
  dypSerial.begin(9600); //DYP-ME007Y

  for (byte loopstep = 0; loopstep <= 3; loopstep++) {
    read_buffer[loopstep] = 0;
  }

  Serial.println(F("Version 1.0.3"));

  Serial.print(F("Pegellimit vorher: "));
  Serial.println(PLIM);
  PLIM = EEPROM.read(EEPROMaddress_PEGEL);
  Serial.print(F("Pegellimit nachher: "));
  Serial.println(PLIM); 
 
  Serial.print(F("Temperaturlimit vorher: "));
  Serial.println(TLIM);
  TLIM = EEPROM.read(EEPROMaddress_TEMP);
  Serial.print(F("Temperaturlimit nachher: "));
  Serial.println(TLIM); 

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  buttonConfig1.setEventHandler(handleEvent1);
  buttonConfig1.setFeature(ButtonConfig::kFeatureLongPress);

  buttonConfig2.setEventHandler(handleEvent2);
  buttonConfig2.setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig2.setFeature(ButtonConfig::kFeatureLongPress);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  button1.init(BUTTON1_PIN, HIGH, 0 /* id */);

  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  button2.init(BUTTON2_PIN, HIGH, 0 /* id */);
 
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  
  // Richtet die Anzahl der Spalten und Zeilen der LCD ein:
  lcd.begin(16, 2);

  //Relay
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);

  lastHell = millis();
  lastCheckValues = lastHell;
  lastRelaisAction = lastHell;
}
  
void loop() {

  button1.check();
  button2.check();

  if(displayHell) {
    lcd.backlight();
  }
  unsigned long now = millis();
  unsigned long time = now - lastHell;
  
  if(time >= 30000 && !pegelSet && !tempSet && displayHell) {
    displayHell = false;
    lcd.noBacklight();
    Serial.println(F("Schalte Hintergrundbeleuchtung aus."));
  } 
  time = now - lastCheckValues;
  if(time >= checkInterval) {
    lastCheckValues = now;
    checkValues();
  }

  // By default, the last intialized port is listening.
  // when you want to listen on a port, explicitly select it:
  // mySerial.listen();
  if(mySerial.available() > 0){
    byte cmd = mySerial.read();
    char c = (char) mySerial.read();

    if(c == 'd') { //toggle debug
      delay(4); //Ohne diese wartezeit wird auch bei 9600Baud immer die False-Variante durchlaufen!!!
      char onOff = (char) mySerial.read();
      if(onOff == '1') {
        Serial.println(F("Set debug to true"));
        debug = true;
      } else {
        Serial.println(F("Set debug to false"));
        debug = false;
      }
    }
    if(c == '@') {
      Serial.println(F("Neuer Client fordert Datenrefresh, an"));
      //@datarequest, neuer Client hat sich connected, es sollten neue Daten gesendet werden
      oldLine1=""; //Zurücksetzen, um neues Refresh zu erwirken.
      oldLine2=""; //Zurücksetzen, um neues Refresh zu erwirken.
    }
  }
}

void checkValues() { 

  StaticJsonDocument<300> doc; //letzte Zaehlung: 114

  if(debug) {
    Serial.print("+");
  }

  //Standardsensor
  //int cm = sonar.ping_cm();

  //DYP-ME007Y  
  float rcm = NIVEAU_UEBER_BODEN - pegel;

  if(debug && pegel > 0) {
    Serial.print("NIVEAU_UEBER_BODEN-dst = ");
    Serial.println(rcm);
  }
  
  byte wasserstand = (byte) rcm;

  doc["p"]=wasserstand;
  doc["PL"]=PLIM;

  bool send = false;
  bool skipCheckTemperature = false;

  String message = "";

  if(rcm>0) { //negative Werte NICHT beachten
    if(rcm < PLIM) {
      relayStatus=HIGH;
      if(pegelOk) {
        message=F("*Pegellimit unterschritten ( ");
        message+=rcm;
        message+=F(" cm ; Limit: ");
        message+=PLIM;
        message+=F(" cm)");
        Serial.println(message);
        pegelOk = false;
      }
      lastRelaisAction=millis();
      digitalWrite(relay, relayStatus);
    } else if(rcm > (PLIM + PHYST)) {
      if(!pegelOk) {
        okMessage=F("*Pegel wieder OK ( ");
        okMessage+=rcm;
        okMessage+=F("cm ; Limit: ");
        okMessage+=PLIM;
        okMessage+=F("cm)");
        Serial.println(okMessage);
        pegelOk = true;
      }
      if(relayStatus != LOW) {
        if((millis()-lastRelaisAction) < minRelaisActionInterval) {
          message = minRelActionMessage;
          if(!debug) Serial.println(message);
        } else {
          relayStatus=LOW;
          digitalWrite(relay, relayStatus);
          lastRelaisAction=millis();
          message = okMessage;
          send = true;
          skipCheckTemperature = true; //damit message nicht ueberschrieben wird!
        }
      }
    }
  } else {
    if(debug) {
      Serial.print("Pegelmesswert negativ, skipping...");
      Serial.println(rcm);
    }
  }

  /* 4 is mininum width, 1 is precision; float value is copied onto str_temp*/
  char str_temp[4];
  dtostrf(rcm, 3, 1, str_temp);

  if(rcm>0) {
    s1 = "" ;
    s1 += String(str_temp);
    int sl = s1.length();
    if(sl==3) {
      s1 += "cm  ";
    } else if(sl==4) {
      s1 += "cm ";
    } else if(sl==5) {
      s1 += "cm";
    }
    float pi = 3.1415;
    float fl = pi * 0.75 * 0.75;
    float height = ((float)rcm) / 100.0;
    float vol = fl * height;
    doc["v"]=String(vol,2);
    
    s1 += "| ";
    s1 += vol;
    s1 += "qm";    
  }

  // read without samples.
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;
  int temp;
  if ((err = dht22.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    if(debug) {
      Serial.print(F("Read DHT22 failed, err=")); Serial.println(err);
    }
    temp = 0;
  } else {
    temp = (int) temperature;
  }
  
  if(temp > 0) {
    s2="";
    s2 += (int) humidity;
    
    if(s2.length()==1) {
      s2 += "%     | ";
      s2 += temp;
    } else {
      s2 += "%    | ";
      s2 += temp;
    }
    s2 += "C";
  } 
  
  doc["t"]=temperature;
  doc["TL"]=TLIM;
  doc["h"]=humidity;

  //Die Temperatur wird nur dann ueberwacht, wenn 
  // das Pump Strom bekommt. Wenn eh ein Fehler vorliegt,
  // kann die Temperatur ignoriert werden.
  if((relayStatus == LOW || !tempOk) && !skipCheckTemperature) {
    if(temp > TLIM) {
      relayStatus=HIGH;
      if(tempOk) {
        message=F("*Temperaturlimit ueberschritten ( ");
        message+=temp;
        message+=F(" °C ; Limit: ");
        message+=TLIM;
        message+=F(" °C)");
        Serial.println(message);
        tempOk=false;
      }
      lastRelaisAction=millis();
      digitalWrite(relay, relayStatus);   
    } else if(temp < (TLIM - THYST)) {
      relayStatus=LOW;
      if(!tempOk) {
        okMessage=F("*Temperatur wieder OK ( ");
        okMessage+=temp;
        okMessage+=F("°C ; Limit: ");
        okMessage+=TLIM;
        okMessage+=F("°C)");
        Serial.println(okMessage);
        tempOk=true;
      }
      if(relayStatus != LOW) {
        if((millis()-lastRelaisAction) < minRelaisActionInterval) {
          //Nachrichten, die mit '@' beginnen, werden nicht gelogged, sondern unten als Statusmeldung angezeigt
          message = minRelActionMessage;
          Serial.println(message);
        } else {
          relayStatus=LOW;
          digitalWrite(relay, relayStatus);
          lastRelaisAction=millis();
          message=okMessage;
          send = true;
        }    
      }
    }
  }

  delay(10); //seltsam, sonst wird 'message = okMessage' nicht korrekt ausgeführt
  
  doc["rs"]=relayStatus;
  doc["d"]=debug;

  //Konfigzeit pruefen, bei Ablauf Wert prüfen und ggfls. speichern
  if(pegelSet && (millis() - setPegelLimit) >= KONFIG_TIME) {
      if(tmpPegelLimit > 0 && tmpPegelLimit != PLIM) {
        PLIM = tmpPegelLimit;
        EEPROM.write(EEPROMaddress_PEGEL, PLIM); 
        Serial.print(F("Neues Pegellimit: "));
        Serial.println(PLIM);
      } 
      tmpPegelLimit = 0;
      Serial.println(F("Resetting pegelSet"));
      pegelSet = false;
      oldLine1=""; //Zurücksetzen, um neues Refresh zu erwirken.
      oldLine2=""; //Zurücksetzen, um neues Refresh zu erwirken.
  }

  //Konfigzeit pruefen, bei Ablauf Wert prüfen und ggfls. speichern
  if(tempSet && (millis() - setTempLimit) >= KONFIG_TIME) {
      if(tmpTempLimit > 0 && tmpTempLimit != TLIM) {
        TLIM = tmpTempLimit;
        EEPROM.write(EEPROMaddress_TEMP, TLIM); 
        Serial.print(F("Neues Temperaturlimit: "));
        Serial.println(TLIM);
      } 
      tmpTempLimit = 0;
      Serial.println(F("Resetting tempSet"));
      tempSet = false;
      oldLine1=""; //Zurücksetzen, um neues Refresh zu erwirken.
      oldLine2=""; //Zurücksetzen, um neues Refresh zu erwirken.
  }

  if(!tempSet && !pegelSet) {
    //Pegel schreiben (int rcm)
    if(s1 != oldLine1) {
      lcdStr(LINE_1, s1);
      send=true;
    }  
    //Temperatur schreiben ( byte temperature )
    if(s2 != oldLine2) {
      lcdStr(LINE_2, s2);
      send=true;
    }
    if(message.length()>MAX_LINE_LENGTH) {
      message=F("Nachricht zu lang, entfernt");
    }
    doc["ht"]=THYST;
    doc["hp"]=PHYST;
    
    doc["m"]=message;
    if(send) {
      if(debug) {
        serializeJsonPretty(doc, Serial);
        //root.prettyPrintTo(Serial);
        Serial.println(F("\n______________________"));
      }
      serializeJson(doc, mySerial);
      //root.printTo(mySerial);
    }
  }
  
}

void handleEvent1(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
     displayHell = true;
     lcd.backlight();
     lastHell = millis();
     long passed;
     passed = lastHell - setPegelLimit;
      if(pegelSet && (passed < KONFIG_TIME)) {
        tmpPegelLimit++;    
        Serial.print(F("Raising tmpPegelLimit to "));
        Serial.println(tmpPegelLimit);             
        lcdStr(LINE_2, tmpPegelLimit);   
      }
      passed = lastHell - setTempLimit;
      if(tempSet && (passed < KONFIG_TIME)) {
        tmpTempLimit++;
        Serial.print(F("Raising tmpTempLimit to "));
        Serial.println(tmpTempLimit); 
        lcdStr(LINE_2, tmpTempLimit);    
      }
      break;    
    case AceButton::kEventLongPressed:
      displayHell = true;
      lcd.backlight();
      if(!tempSet) {
        tmpPegelLimit = PLIM;
        lcdStr(LINE_1, F("Pegellimit:")); 
        lcdStr(LINE_2, PLIM);
        Serial.println(F("Setting pegelSet"));
        pegelSet = true;
        setPegelLimit = millis();
      }
      break;        
  }
}

void handleEvent2(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      displayHell = true;
      lcd.backlight();
      lastHell = millis();
      long passed;
      passed = lastHell - setPegelLimit;
      if(pegelSet && (passed < KONFIG_TIME)) {
        tmpPegelLimit--;
        Serial.print(F("Lowering tmpPegelLimit to "));
        Serial.println(tmpPegelLimit);     
        lcdStr(LINE_2, tmpPegelLimit);   
      }
      passed = lastHell - setTempLimit;
      if(tempSet && (passed < KONFIG_TIME)) {
        tmpTempLimit--;
        Serial.print(F("Lowering tmpTempLimit to "));
        Serial.println(tmpTempLimit);        
        lcdStr(LINE_2, tmpTempLimit);   
      }
      break;   
    case AceButton::kEventLongPressed:
      displayHell = true;
      lcd.backlight();
      if(!pegelSet) {
        tmpTempLimit = TLIM;
        lcdStr(LINE_1, "Temperaturlimit:");
        lcdStr(LINE_2, TLIM);    
        Serial.println(F("Setting tempSet"));
        tempSet = true;
        setTempLimit = millis();
      }
      break;   
  }
}

void initInterruptTimer1() {
  cli(); //disable interrupts
  TCCR1A = 0; //set TCCR1A register to 0
  TCCR1B = 0; //set TCCR1B register to 0
  TCNT1 = 0; //reset counter value
  //62499 == 4s
  OCR1A = 7750; //compare match register, ca. 0.5s
  TCCR1B |= (1 << CS12) | (1 << CS10); //set prescaler
  TCCR1B |= (1 << WGM12); //turn on CTC mode
  TIMSK1 |= (1 << OCIE1A);//enable timer compare input
   sei(); //allow inputs
}

//function wird aufgerufen wenn ein interrupt auf timer 1 passiert
ISR(TIMER1_COMPA_vect) { 

  /**
   * Every time an output module containing four eight data frame format is : 
    0XFF + H_DATA + L_DATA + SUM
    1. 0XFF: for a start of data for judgment.
    2. H_DATA: distance data of eight high .
    3. L_DATA: low distance data of eight .
    4. SUM: data and for efficacy . Its 0XFF + H_DATA + L_DATA = SUM ( only 
    low 8 )
    5. H_DATA and L_DATA synthetic 16 data , the distance value in 
    millimeters .
   */
   if (dypSerial.available() < 1) {
    return;
  }
  
  //Es werden immer 4 Byte uebertragen, beginnend mit 0xff
  while (dypSerial.available() > 0) {
    read_buffer[00] = dypSerial.read();
    if (read_buffer[00] != 0xff) {
      continue; 
    };
    if(dypSerial.available() > 0) {
      read_buffer[01] = dypSerial.read(); 
    }
    if(dypSerial.available() > 0) {
      read_buffer[02] = dypSerial.read(); 
    }
    if(dypSerial.available() > 0) {
      read_buffer[03] = dypSerial.read(); 
    }
  }
  
  crcCalc = read_buffer[00] + read_buffer[01] + read_buffer[02];
  if (read_buffer[03] != crcCalc) {
    return; 
  };

  word distance = (read_buffer[01] * 0xff) + read_buffer[02];

  float newVal = distance/10.0;

  //pegel = newVal;
  //return;

//Serial.print("Frisch Gelesener Wert: ");
  //  Serial.println(newVal);
  
  if(debug2) {
    Serial.println("\n________________________________________________");
    Serial.print("Frisch Gelesener Wert: ");
    Serial.println(newVal);
    Serial.print("Letzter Pegel: ");
    Serial.println(pegel);    
    Serial.print("\nIndexedValues: ");
    for(int k=0; k<buff; k++) {
      float v = pegelBuffer[k];
      Serial.print(v);
      Serial.print(", ");   
    }    
  }
  if(debug2) {
      Serial.print("\nPegelindex ");
      Serial.println(pegelIndex);
  }
  //first init
  if(pegel < 0){
     if(debug2) {
        Serial.print("\nInitializing Pegel to ");
        Serial.println(newVal);
     }
     pegel = newVal;  
  }

  if(newVal > (pegel + 1)) {
    newVal = pegel + 1;
    if(debug2) {
      Serial.print("\nLimiting max Pegel change to ");
      Serial.println(newVal);
    }
  }
  if(newVal < (pegel - 1)) {
    newVal = pegel - 1;
    if(debug2) {
      Serial.print("\nLimiting min Pegel change to ");
      Serial.println(newVal);
    }
  }

  //float pegel ist volatile und wird aus loop() gelesen, Set von 50 Werten bilden und nur Mittelwert verarbeiten
  pegelIndex++;
  if(pegelIndex>=buff) pegelIndex=0;
  pegelBuffer[pegelIndex] = newVal;

  float total = 0.0;
  for(int i = 0; i<buff; i++) {
    float val = pegelBuffer[i];
    if(val > 0) {
      total += val;
    } else {
      total += pegelBuffer[0];
    }
  }
  //Mittelwert bilden
  pegel = total / buff;

  if(debug2) {
    Serial.print("\nNeuer Pegel: ");
    Serial.print(pegel,1);
  }

  //alle inzwischen wieder aufgelaufenen Messungen verwerfen
  while (dypSerial.available() > 0) {
    dypSerial.read();
  }
}

void lcdStr(int line, uint8_t val) {
  String str = String(val);
  lcd.setCursor(0, line);
  if(line==0) {    
    oldLine1=str;
  } else {
    oldLine2=str;
  }
  lcd.print(str);
  for(int i = str.length(); i<16;i++) lcd.print(' ');
}

void lcdStr(int line, String str) {
  //Serial.print(str);
  //Serial.println(line);
  
  lcd.setCursor(0, line);
  if(line==0) {    
    oldLine1=str;
  } else {
    oldLine2=str;
  }
  lcd.print(str);
  for(int i = str.length(); i<16;i++) lcd.print(' ');
}
