#include<Wire.h>

int count = 0;                                          // count = 0
char input[12];                                         // RF-ID character array of size 12 
boolean flag = 0;                                       // flag =0
int check = 0;

const int MPU = 0x68;
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int OldAcX, OldAcY, OldAcZ, oldTmp, OldGyX, OldGyY, OldGyZ = 0; 

const int AcSensitivity = 800;
boolean moved = false;

const int maxMovementTime = 5000;
const int maxStillnessThreshold = 20;
const int readMPUinterval = 250;

const int maxMovementTimeArrayDifference = maxMovementTime / readMPUinterval;
const int movedRecordLength = maxMovementTimeArrayDifference * 2;
boolean movedRecord[movedRecordLength];

unsigned long previousMPUmillis = 0;

const int ledPin=13;

const int PIEZO_PIN = A1; // vibration- Piezo output
const int LED_PIN = 12;

const int threshold = 500;
char theftMode = '0';  //theft mode to be read from app

int reedPin = 3;

int emergencyPin = 5;
int prevTime = -10000;

bool watersensed = false;
bool stolenreported = false;

char mqttdata = '0';


void setup() {
  //setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(emergencyPin, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(4, OUTPUT);  // for indicating bag being stolen
  pinMode(ledPin,OUTPUT);
  pinMode(reedPin,INPUT_PULLUP);
  
  beginMPUcommunications();
  initializeMovedRecord();
  Serial.print("MovedRecordLength: ");
  Serial.println(movedRecordLength);
  pinMode(3, INPUT_PULLUP);

  pinMode(2, INPUT);

//  digitalWrite(emergencyPin, LOW);
}

void loop() {
  Serial.print("theif value - >>");
  Serial.println(theftMode);
//  Serial1.print("MQTT");
//  Serial1.print(Serial.read());
  if(Serial.available() > 0) {  //reading mqtt data
//    mqttdata = Serial.read();
if(Serial1.available()>0){
    Serial1.print("MQTT");
    Serial1.print(Serial.read());
  } 
  }
 
        //  ########EMEGENCY-PIN#########
  int curTime = millis();
//  Serial.println("##############Time#############");
//  Serial.println(curTime);
  int emergency = digitalRead(emergencyPin);
  Serial.println();
  Serial.print("emergency");
  Serial.println(emergency);
  if(emergency == LOW && curTime-prevTime < 5000) //if push-button is switched twice within 5 seconds
    reportEmergency();
  prevTime = curTime;  

  //    #####################################

    
  if(Serial1.available()>0)
    {
      Serial.println();
      Serial.println("#########theftmode###########");
      theftMode = Serial1.read();  //read whether on theft mode or not using bluetooth module from app
      delay(500);
      Serial.println(theftMode);
      Serial.println("#######################");
    }
  
    
  Serial.print("water reading value - > ");
  int water_data = analogRead(2);
  Serial.println(water_data);
  if(water_data < 200 && watersensed == false) 
    {
      reportWater();  //water sensed in the bag
      watersensed = true;
    }
    
  int reed = digitalRead(reedPin);  //reed switch digittal data
    delay(50);
    if(reed == LOW){
      Serial.println("BAG Closed");
      reportBagclosed(); //data updated on app
    }
    else{
      Serial.println("BAG open");
      reportBagopen();
    }
  
//  Serial.println("start");
   if(Serial2.available())
   {   //Rf-Id TAG character read 
      count = 0;
      while(Serial2.available() && count < 12)          // Read 12 characters and store them in input array
      {
         input[count] = Serial2.read();
//         Serial.println("TAG FOUND .....");
//         Serial.print(input[count]);
         count++;
         delay(5);
      }
      delay(100);
      check = (check+1)%2; 
      if(check ==0){  //checking validity of RF-ID tags 
        if((input[0] ^ input[2] ^ input[4] ^ input[6] ^ input[8] == input[10]) && (input[1] ^ input[3] ^ input[5] ^ input[7] ^ input[9] == input[11])){
            Serial.println("Notebook is outside"); 
            sendRfidtags();
         }
      }
      else{
         if((input[0] ^ input[2] ^ input[4] ^ input[6] ^ input[8] == input[10]) && (input[1] ^ input[3] ^ input[5] ^ input[7] ^ input[9] == input[11])){
            Serial.println("Notebook is inside");
            sendRfidtags();
         }
      }
      Serial.println(input);
  }

  
  // Read Piezo ADC value in, and convert it to a voltage
  int piezoADC = analogRead(PIEZO_PIN);
  float piezoV = piezoADC;  //vibration analog data 
  if(piezoV >= threshold)   //if greater than threshold value
  {
    Serial.println("SOMEONE IS CALLING");    
    digitalWrite(LED_PIN, HIGH);
    delay(5000);
    digitalWrite(LED_PIN, LOW);
  }
  Serial.println(piezoV); // Print the voltage.
  delay(50);


  //  int sensorState = digitalRead(6);
//  Serial.println(sensorState);
////  delay(100);
//  if(sensorState == HIGH)
//  {
//    digitalWrite(ledPin,HIGH);
//    delay(1000);
//  }
//  else
//  {
//    digitalWrite(ledPin,LOW);
//  }

  
  // To run repeatedly:
  unsigned long currentMillis = millis();
//  Serial.println("reading");
  boolean isStolen = false;
//  Serial.println("dummy stolen value - " );
//  Serial.println(isStolen);

  if((currentMillis - previousMPUmillis) > readMPUinterval){
    checkPositionChangeAndShiftRecord();
    printMovedRecord();
    isStolen = checkIfStolen();
    if(isStolen == true && mqttdata == '0' && stolenreported==false){  
      Serial.print("Stolen acc to criteria only");
      if(theftMode != '0')  //theft mode criteria enabled from app
        {
          Serial.print("STOLEN(theft mode on"); reportStolen(); stolenreported=true; digitalWrite(4, HIGH); //buzzer alarm
        }
    }
    else{
      digitalWrite(4, LOW);
    }
    previousMPUmillis = currentMillis;
  }
  delay(500);
}

void initializeMovedRecord(){
  for(int x = 0;x < movedRecordLength; x++){
    movedRecord[x] = false;
  }
}

void printMovedRecord(){
  for(int x = 0;x < movedRecordLength; x++){
    Serial.print(movedRecord[x]);
  }
  Serial.println();
}

void beginMPUcommunications(){  //MPU communication with arduino
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPUrawvalues(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 12, true);
  AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
//  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void checkPositionChangeAndShiftRecord(){  // sensitivity check for accelerometer values
  readMPUrawvalues();
  if(abs(OldAcX - AcX) > AcSensitivity){  //sensitivity difference between accelerometer values
    moved = true;
  }
  if(abs(OldAcY - AcY) > AcSensitivity){
    moved = true;
  }
  if(abs(OldAcZ - AcZ) > AcSensitivity){
    moved = true;
  }
  shiftMovedRecord(moved);  //update old and new accelerometer values
  OldAcX = AcX;
  OldAcY = AcY;
  OldAcZ = AcZ;
  moved = false;
}

void shiftMovedRecord(boolean movedValue){  //left shifting moved value for next cycle check
  for(int x = 0;x < movedRecordLength; x++){
    if(x == (movedRecordLength - 1)){
      movedRecord[x] = movedValue;
    }
    else{
    movedRecord[x] = movedRecord[x+1];
  }

  }
}

boolean checkIfStolen(){  //boolean check for stolen
  int firstMoved = movedRecordLength - 1;
  int lastMoved = 0;
  boolean firstMovedSet = false;
  boolean lastMovedSet = false;
  boolean movedEnoughTime = false;

  for(int x = 0;x < movedRecordLength; x++){    //boolean array of moved value. 1 shows sensitivized accelerometer value
    if(movedRecord[x] == true && firstMovedSet == false){
      firstMoved = x;
      firstMovedSet = true;
    }
  }

  for(int x = (movedRecordLength - 1);x >= 0;x--){
    if(movedRecord[x] == true && lastMovedSet == false){
      lastMoved = x;
      lastMovedSet = true;
    }
  }  //Last moved value for max range of stolen data

  if((lastMoved - firstMoved) > maxMovementTimeArrayDifference){  //if the two points differ by a max 5 seconds
    movedEnoughTime = true;
  }

  if(movedEnoughTime == true){  // if the movement difference is for at max 5 seconds counts no. of 1's in the array
    int trueCounter = 0;
    for(int x = firstMoved; x <= lastMoved; x++){
      if(movedRecord[x] == true){
        trueCounter++;  //counting no. of movements in between last and first movedRecord
      }
    }

    int movementPercent = map(trueCounter, 0, lastMoved - firstMoved + 1, 0, 100); //percentage of movement between two 1's 
    if(movementPercent > maxStillnessThreshold){  //if upto threshold movement
      return true;
    }
    else{
      return false;
    }

  }
  return false;
}

//Sends an alert to the phone that someone is moving the bag while in theft mode
void reportStolen()
{
  Serial1.print('#');
  Serial1.println();
}
//Sends the status of the bag zip(open) to the phone through bluetooth
void reportBagopen()
{
  Serial1.print('~');
  Serial1.println();
}
//Sends the status of the bag zip(closed) to the phone through bluetooth
void reportBagclosed()
{
  Serial1.print('+');
  Serial1.println();
}
//Sends the rfid tag numbers to the phone through bluetooth
void sendRfidtags()
{
  Serial1.print('*');
  Serial1.print(input);
  Serial1.println();
}
//Sends an alert to the phone when water is detected in bag through bluetooth
void reportWater()
{
  Serial1.print('$');
  Serial1.println();
}
//Sends the emergency signal to the phone to send sms with location to contacts
void reportEmergency()
{
  Serial.print("emergency push button");
  Serial1.print(1);
  delay(10);
  Serial.println();
}
