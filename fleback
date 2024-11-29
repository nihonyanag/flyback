//共用************
#include <Wire.h>
  unsigned long keepTime = 0;  
  unsigned long currentTime = 0;
  unsigned long fallTime = 0;  
  int interval = 100;//データを取得する間隔
  bool RO = true;
  const int button_pin = 12;
  const int LED_pin = 14;
  int value = 0;
  RTC_DATA_ATTR int bootCount = 0;
  RTC_DATA_ATTR bool a = true;
  RTC_DATA_ATTR bool b = true;
//共用************

//BME用********************************
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
  float Altitude = 0;
  float maxAltitude = 0;
  float keepAltitude = 0;
  float PLA = 0;
  float LA = 0;
  float s = 3;//高度差(m)
//BME用********************************

//ADXL用************************************************************
#include <ADXL345.h>
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
  double xyz[3];
  double ax,ay,az;
  double G;
//ADXL用************************************************************

//GPS用********************
#include <TinyGPS++.h>
#define OLED_RESET -1
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);
TinyGPSPlus gps;
  double LatA = 42.907647;  //目的地_緯度
  double LongA = 141.574028;//目的地_経度
  double laT;
  double lonG;
  double Direction = 0;//方角
  double Distance = 0;//距離
//GPS用********************

//SD用********************************************************************************
#include <SPI.h>
#include <SD.h>

  const int CS_PIN = 2;
  const String file_path = "/data/AfterTheFall.csv";//作成するファイルとパスの名前を指定、作成
File f;
//SD用********************************************************************************

//サーボ****************
#include <ESP32Servo.h>
#define SERVO_PIN 4
Servo myServo;
//サーボ****************

//ブザー***************************************
#define BUZZER_PIN 26// ブザーのピン番号
#define BUZZER_CHANEL 1// ブザーのチャネル番号
//ブザー***************************************

void setup() {
  pinMode( button_pin , INPUT_PULLUP );
  pinMode( LED_pin , OUTPUT );
  Serial.begin(115200);
//BME用********************************
  bool status;
  status = bme.begin(0x76);
  if (!status)
  {
    Serial.println("BME280が使えない");
    while (10);
  }
//BME用********************************

//ADXL用************************************************************
  adxl.powerOn();
//ADXL用************************************************************

//GPS用***************************************
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
//GPS用***************************************

//SD用********************************************************************************
  if(SD.begin(CS_PIN)){//SD初期化
  Serial.println("SD OK");
  }else{
    Serial.println("SD Failed");
    while(1);
    }
 //フォルダの確認
 if(!SD.exists("/data")){
  Serial.println("directory none");
  SD.mkdir("/data");
  }

 //ファイルの確認
if(SD.exists(file_path)){
    Serial.println("file exist");
   // ReadFile(file_path);
  }else{
    Serial.println("file none");
    f = SD.open(file_path, FILE_APPEND);
    f.println("Time,Altitude,latitude,longitude,acceleration_X,acceleration_Y,acceleration_Z,impact");
    f.close();
  }
//SD用********************************************************************************

//サーボ用************************
  pinMode(SERVO_PIN, OUTPUT);     // サーボピンのモードを設定
  myServo.attach( SERVO_PIN );

//サーボ用************************

delay(1000);
}

void loop() {
  while(  a == true ){
    GPS();
    if ( digitalRead( button_pin ) == LOW ){
      delay( 200 );
      bootCount += 1;
      Serial.println( bootCount );
      while (digitalRead(button_pin) == LOW) {
      // 何もしない
      }
    }
    if ( bootCount == 1 ){
      a = false;
      myServo.write( 0 );//原点合わせ
      f = SD.open(file_path, FILE_APPEND);
      f.println("Power Saving Mode");
      f.close();
      digitalWrite(LED_pin , HIGH);
      delay(500);
      digitalWrite(LED_pin , LOW);
      DeepSleep();
    }
  }
  while( a == false ){
    if ( b == true ){
      Buzzer();
      delay(500);
      ledcWriteTone(BUZZER_CHANEL, 0);    // 消音
      PLA = bme.readAltitude(SEALEVELPRESSURE_HPA);
      f = SD.open(file_path, FILE_APPEND);
      f.println("Weke UP");
      f.print("Pre-launch altitude");
      f.println( PLA );
      f.close();
      b = false;
    }

    lead();
  }
}

void lead(){
  currentTime = millis();//interval秒で現在時刻を更新
  Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);//interval秒で現在高度を更新
  Altitude = Altitude +200;
  if (currentTime - keepTime >= interval ){ //interval秒でデータを取得
    keepTime = currentTime;

    if ( Altitude > maxAltitude ){//最高高度の入れ替え
      maxAltitude = Altitude;
    }
    if ( Altitude - keepAltitude != 0 ){//現在高度の入れ替え
      keepAltitude = Altitude;
    }
    
    if ( maxAltitude - Altitude >= s && RO == true  ){//パラシュート展開
      loofOpen();
      RO = false;
      fallTime = currentTime;
    }
    GPS();
    impact();
    SDcard();
    if ( currentTime - fallTime >= 10000 ){
      if( RO == false ){//着地確認
        f = SD.open(file_path, FILE_APPEND);
        f.println( "landing" );
        f.close();
        while(1){//ブザーを鳴らし続ける
          Buzzer();
          delay(5000);
          ledcWriteTone(BUZZER_CHANEL, 0);    // 消音
          delay(1000);
          
        }
      }
    }
  }
}

void GPS(){
  boolean newData = false; //新しいデータが取得されたかどうか
  while ( neogps.available() ){ //新しいデータの有無
    if ( gps.encode( neogps.read() ) ){ //GPSデータを解析し、緯度と経度を取得
      newData = true;
      double laT = (gps.location.lat());
      double lonG = (gps.location.lng());
       
      Serial.print("latitude: ");//緯度
      Serial.print( String(laT,7) );
      Serial.print("/");
      Serial.print("longitude: ");//経度
      Serial.print( String(lonG,7) );
      Serial.print("/");
      
      Direction = (atan2((lonG - LongA) * 1.23, (laT - LatA)) * 57.3 + 180);   //目的地までの方角
      Distance = (sqrt(pow(LongA - lonG, 2) + pow(LatA - laT, 2)) * 99096.44); //目的地までの距離
    }
  }
}

void loofOpen(){
  pinMode(SERVO_PIN, OUTPUT);     // サーボピンのモードを設定
  myServo.attach( SERVO_PIN );
  myServo.write( 120 );
  delay(500);
  digitalWrite(LED_pin , HIGH);
  delay(500);
  digitalWrite(LED_pin , LOW);
  f = SD.open(file_path, FILE_APPEND);
  if (f){
    f.println( "************************************************************" );
    f.println( "RoofOpen" );
    f.println( "************************************************************" );
    f.close();
  }
  else{
    Serial.println("Error opening file");
  }
}

void SDcard(){
  char buffer[128]; // 書き込むデータを保持するバッファ
  sprintf(buffer, "%ld,%f,%0.7f,%0.7f,%f,%f,%f,%f", currentTime, Altitude, laT, lonG, ax, ay, az, G);

  f = SD.open(file_path, FILE_APPEND);
  if (f) {
    f.println(buffer); // まとめたデータを一気に書き込む
    f.close();
  }
  else{
    Serial.println("Error opening file");
  }
}

void impact(){

  adxl.getAcceleration(xyz);
  ax = xyz[0];
  ay = xyz[1];
  az = xyz[2];
  Serial.print("X=");
  Serial.print(ax);
  Serial.println(" g");
  Serial.print("Y=");
  Serial.print(ay);
  Serial.println(" g");
  Serial.print("Z=");
  Serial.print(az);
  Serial.println(" g");
  G = sqrt(pow(ax,2) + pow(ay,2) + pow(az,2));
  Serial.print("G:");
  Serial.println(G);
//  return G;
}

void DeepSleep(){
//  esp_sleep_enable_timer_wakeup(10 * 1000000);  //スリープ時間を５秒に設定
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, LOW);
  esp_deep_sleep_start();                      //スリープモードへ移行
}

void Buzzer(){
  ledcSetup(BUZZER_CHANEL, 12000, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANEL);
  ledcWriteTone(BUZZER_CHANEL, 440); // ラ
}
