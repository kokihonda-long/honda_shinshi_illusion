//EDDITED THE FINAL VERSION FOR USE AS OF 8/25

#include <TimerThree.h>
#include <IcsHardSerialClass.h>

#define vibThresh 2
#define ENC_A1 20
#define ENC_B1 21
#define KeisouAmp 8 

const int SIZE = 400;//400;//x0.05で継続秒数
//int TWARI = 50000;
float angleLog[SIZE];
double forceLog[SIZE];
double motorLog[SIZE];
int timeLog[SIZE];
int intervalLog[SIZE];
int count_record = 0;
int i;

int posMotor; //モータの角度情報（変換前）
double motorAngle_init; //モータの初期角度 
double motorAngle; //モータの角度情報（変換後そのまま）
double motorAngleTreated; //モータの角度情報（変換後0基準）
int motorGoal; //モータの目標角度
int motorSpeed;//モータの速度
const byte EN_PIN = 8; 
const long BAUDRATE = 115200;
const int TIMEOUT = 1000;
volatile bool flagRecord = false;
IcsHardSerialClass krs(&Serial3,EN_PIN,BAUDRATE,TIMEOUT);

float angle; //2021_6_24_本田
volatile byte pos;
volatile int  enc_count;
double forceSensorOutput;
double force; //2021_7_20_本田
double slope = 0.0453; //2024_8_和泉
double intercept = -32.003;// -32.663; //2024_8_和泉

double dt;
int time;
int current_time;
int previous_time;

void servoRotate(){
  krs.setSpd(0, motorSpeed);
  krs.setPos(0, motorGoal);
}

void flagRecordISR(){
  flagRecord = true;
}

void ENC_READ() { //エンコーダの読み込み、変えない
  byte cur = (!digitalRead(ENC_B1) << 1) + !digitalRead(ENC_A1);
  byte old = pos & B00000011;
  byte dir = (pos & B00110000) >> 4;
  if (cur == 3) cur = 2;
  else if (cur == 2) cur = 3;
  if (cur != old)
  {
    if (dir == 0)
    {
      if (cur == 1 || cur == 3) dir = cur;
    } else {
      if (cur == 0)
      {
        if (dir == 1 && old == 3) enc_count++;
        else if (dir == 3 && old == 1) enc_count--;
        dir = 0;
      }
    }
    bool rote = 0;
    if (cur == 3 && old == 0) rote = 0;
    else if (cur == 0 && old == 3) rote = 1;
    else if (cur > old) rote = 1;
    pos = (dir << 4) + (old << 2) + cur;
  }
}

void dataRecord(){
  //左エンコーダの角度情報
  angle = float(enc_count) * 360.00 / 1000.00;
  angleLog[count_record] = angle;

  //力センサの情報
  forceSensorOutput = analogRead(KeisouAmp);
  force = slope*forceSensorOutput + intercept;
  forceLog[count_record] = force;

  //モータの角度情報
  posMotor = krs.getPos(0);
  //motorAngle = map(posMotor, 3500, 11500, -135, 135);
  motorAngle = 0.03375*posMotor-253.125;
  motorAngleTreated = motorAngle - motorAngle_init;
  motorLog[count_record] = motorAngleTreated;

  //時間間隔の情報
  current_time = millis();
  dt = current_time - previous_time;
  timeLog[count_record] = current_time;
  intervalLog[count_record] = dt;
  previous_time = current_time;
}

void dataPrint(){
  Serial.println("data below");
  Serial.println("time / wrist_ang / force / motor_ang / dt");
  for(int i=0; i<SIZE; i++){
    //Serial.print(i);
    //Serial.print(": ");
    Serial.print(timeLog[i]);
    Serial.print(", ");
    Serial.print(angleLog[i], 3);
    Serial.print(", ");
    Serial.print(forceLog[i], 1);
    Serial.print(", ");
    Serial.print(motorLog[i], 2);
    Serial.print(", ");
    Serial.println(intervalLog[i]);
  }
}

void setup() {
  current_time = 0;
  previous_time = 0;

  for(i=0; i<SIZE; i++){
    timeLog[i] = 0;
  }

  for(i=0; i<SIZE; i++){
    angleLog[i] = 0;
  }

  for(i=0; i<SIZE; i++){
    forceLog[i] = 0;
  }

  for(i=0; i<SIZE; i++){
    motorLog[i] = 0;
  }

  for(i=0; i<SIZE; i++){
    intervalLog[i] = 0;
  }

  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(vibThresh, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), ENC_READ, CHANGE); //外部割込み
  attachInterrupt(digitalPinToInterrupt(ENC_B1), ENC_READ, CHANGE);

  Timer3.initialize(50000); //単位はマイクロ秒
  Timer3.stop();
  Timer3.attachInterrupt(flagRecordISR);

  krs.begin();
  krs.setSpd(0, 10);  
  krs.setPos(0,6250);
  delay(1000);
  posMotor = krs.getPos(0); 
  //motorAngle_init = map(posMotor, 3500, 11500, -135, 135);
  motorAngle_init = 0.03375*posMotor-253.125;

  Serial.begin(9600);
  Serial.println("ready");
  forceSensorOutput = analogRead(KeisouAmp);
  force = slope*forceSensorOutput + intercept;
  Serial.print("force is: ");
  Serial.println(force);
}

void loop(){
  if(Serial.available()>0){
    char command = Serial.read();
    if(command == 'a'){
      motorGoal = 6694;
      motorSpeed = 10;
      Serial.println("set to A");
    }

    if(command == 'b'){
      motorGoal = 6694;
      motorSpeed = 57;
      Serial.println("set to B");
    }

    if(command == 'c'){
      motorGoal = 6694;
      motorSpeed = 127;
      Serial.println("set to C");
    }

    if(command == 'd'){
      motorGoal = 7586;
      motorSpeed = 10;
      Serial.println("set to D");
    }

    if(command == 'e'){
      motorGoal = 7586;
      motorSpeed = 57;
      Serial.println("set to E");
    }

    if(command == 'f'){
      motorGoal = 7586;
      motorSpeed = 127;
      Serial.println("set to F");
    }

    if(command == 'g'){
      motorGoal = 8962;
      motorSpeed = 10;
      Serial.println("set to G");
    }

    if(command == 'h'){
      motorGoal = 8962;
      motorSpeed = 57;
      Serial.println("set to H");
    }

    if(command == 'i'){
      motorGoal = 8962;
      motorSpeed = 127;
      Serial.println("set to I");
    }

    if(command == 'j'){
      //motorGoal = 6520;
      motorSpeed = 0;
      Serial.println("set to J");
    }
    
    if(command == 'z'){
      Serial.println("Z = motor only");
      Timer3.start();
      servoRotate(); //krs.setPos(0, 8000);
        
      while(count_record<SIZE){ //最後に付け足したこいつがカギか？これがないと異常なスピードで記録されていく
        if(flagRecord){
         flagRecord = false;
         dataRecord();
         count_record++;
        } 
      } 

      if(count_record == SIZE){ 
        Serial.println("Logs full");
        count_record++;
      }
    }

    if(command == 's'){
      dataPrint();
    }
  }

  if(digitalRead(vibThresh) == LOW){
    Timer3.start();
    servoRotate();
        
    while(count_record<SIZE){ //最後に付け足したこいつがカギか？これがないと異常なスピードで記録されていく
      if(flagRecord){
        flagRecord = false;
        dataRecord();
        count_record++;
      } 
    } 

    if(count_record == SIZE){ 
      Serial.println("Logs full");
      count_record++;
    }
  }
}