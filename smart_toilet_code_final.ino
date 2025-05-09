#include "DHT.h" 
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Wiring: SDA pin is connected to A4 and SCL pin to A5
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "NewPing.h"
#include <DFRobot_DF1201S.h>

#define TRIGGER_PIN 8
#define ECHO_PIN 9
#define TRIGGER_PIN2 12
#define ECHO_PIN2 13
#define MAX_DISTANCE 200

int motion_sensor = 4;
int fanPin = 5;
int LED1 = 10;
int LED2 = 11;
int pump = 7;
int distance1;
int distance2;
bool in_toilet = false;

byte Heart[8] = {
0b00000,
0b01010,
0b11111,
0b11111,
0b01110,
0b00100,
0b00000,
0b00000
};
byte smile[8] = {
0b00000,
0b01010,
0b00000,
0b00000,
0b10001,
0b01110,
0b00000,
0b00000
};

DHT dht2(6, DHT11);
LiquidCrystal_I2C lcd(0x27, 16, 2);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

SoftwareSerial mySerial(2, 3); // RX, TX
DFRobot_DF1201S mp3player;

void setup() {
  dht2.begin();

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(pump, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(motion_sensor, INPUT);
  
  Serial.begin(115200);
  mySerial.begin(115200);
  while (!mp3player.begin(mySerial)) {
    Serial.println("Init failed, please check the wire connection!");
    delay(1000);
  }
  mp3player.setVol(/*VOL = */8);
  Serial.print("VOL:");
  
  // Initiate the LCD:
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.createChar(0, Heart);
  lcd.createChar(1, smile);
}

void loop(){

distance1 = sonar.ping_cm();
if ((in_toilet == false) && (distance1 > 10)){
      //Serial.println(in_toilet);
      //Serial.println(distance1);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write(byte(0));
      lcd.setCursor(1,0); 
      lcd.print(" Welcome "); 
      lcd.setCursor(15,0);
      lcd.write(byte(0));
      delay(1000);//Delay used to give a dynamic effect
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1); 
      lcd.print(" AVAILABLE ");
      lcd.setCursor(15,1);
      lcd.write(byte(1));
      in_toilet = false;
      // stop music
      Serial.println("Stop playing");
      mp3player.pause();
      // turn off lights
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
}
if ((in_toilet == true) && (distance1 < 10)){
      //Serial.println(in_toilet);
      //Serial.println(distance1);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write(byte(0));
      lcd.setCursor(1,0); 
      lcd.print(" Welcome "); 
      lcd.setCursor(15,0);
      lcd.write(byte(0));
      delay(1000);//Delay used to give a dynamic effect
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1); 
      lcd.print(" AVAILABLE ");
      lcd.setCursor(15,1);
      lcd.write(byte(1));
      in_toilet = false;
      // stop music
      Serial.println("Stop playing");
      mp3player.pause();
      // turn off lights
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
}
if ((in_toilet == false) && (distance1 < 10))  {
    Serial.println(in_toilet);
    Serial.println(distance1);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write(byte(0));
      lcd.setCursor(1,0); 
      lcd.print(" Welcome "); 
      lcd.setCursor(15,0);
      lcd.write(byte(0));
      delay(1000);//Delay used to give a dynamic effect
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1); 
      lcd.print(" OCCUPIED ");
      lcd.setCursor(15,1);
      lcd.write(byte(1));
      in_toilet = true;
      // play music
      //Serial.println("Start playing");
      mp3player.start();
      // turn on lights
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      //check for water pump
      
    }
  if  ((in_toilet == true) && (distance1 > 10)) {
    //Serial.println(in_toilet);
    //Serial.println(distance1);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.write(byte(0));
      lcd.setCursor(1,0); 
      lcd.print(" Welcome "); 
      lcd.setCursor(15,0);
      lcd.write(byte(0));
      delay(1000);//Delay used to give a dynamic effect
      lcd.setCursor(0,1);
      lcd.write(byte(1));
      lcd.setCursor(1,1); 
      lcd.print(" OCCUPIED ");
      lcd.setCursor(15,1);
      lcd.write(byte(1));
      in_toilet = true;
      // play music
      //Serial.println("Start playing");
      mp3player.start();
      // turn on lights
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      //check for water pump
      
  }

  distance2 = sonar2.ping_cm();
      //Serial.println("distance2");
     // Serial.println(distance2);
      if ((distance2 > 0) && (distance2 < 5)) {
        digitalWrite(pump, HIGH);
        Serial.println("water pump ON");
        delay(2000);
        digitalWrite(pump, LOW);
        Serial.println("water pump OFF");
      }

  float temperature = dht2.readTemperature( ); 
 
  //Serial.print("temp Value: ");
  //Serial.println(temperature);
  
  if (temperature < 30)
  {
    digitalWrite(fanPin, HIGH);
    //Serial.print("Fan On ");
  }
  else
  {
  digitalWrite(fanPin, LOW);
  //Serial.print("Fan Off ");
  }
  delay(500);
}