//共用************
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ADXL345.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>
#include <ESP32Servo.h>
#define button_pin 12
#define LED_pin 14
  unsigned long keepTime = 0;   
  unsigned long currentTime = 0;
  unsigned long fallTime = 0;   //パラシュート展開からの時間経過用変数
  const int interval = 100;//データを取得する間隔
  const int par_to_land = 10000;//パラシュート展開から着地まで
  bool fallDet_flag = true;//落下検知フラグ(Fall Detection)

  RTC_DATA_ATTR int bootCount = 0;
  RTC_DATA_ATTR bool BeforeDS_flag = true;//ディープスリープに入る前フラグ
  RTC_DATA_ATTR bool RturnDS_flag = true;//ディープスリープから復帰後フラグ
//共用************

//BME用********************************
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
  const int AddAltitude = 200;//高度が負の値にならないように
  float Altitude = 0;
  float maxAltitude = 0;
  float keepAltitude = 0;
  float PLA = 0;//Pre-launch altitude(打上げ前高度)
  float AltitudeDif = 1.5;//Altitude difference高度差(m)
//BME用********************************

//ADXL用************************************************************
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
  double xyz[3];
  double ax,ay,az;
  double G;
//ADXL用************************************************************

//GPS用********************

#define OLED_RESET -1
#define RXD2 16
#define TXD2 17
HardwareSerial neogps(1);
TinyGPSPlus gps;
  float laT;
  float lonG;
//GPS用********************

//SD用********************************************************************************
  const int CS_PIN = 2;
  const String file_path = "/data/AfterTheFall.csv";//作成するファイルとパスの名前を指定、作成
File f;
//SD用********************************************************************************

//サーボ****************
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
      //LED点滅
      digitalWrite(LED_pin , HIGH);
      delay(200);
      digitalWrite(LED_pin , LOW);
      delay(200);
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
    while(1){
      //LED点滅
      digitalWrite(LED_pin , HIGH);
      delay(1000);
      digitalWrite(LED_pin , LOW);
      delay(1000);
    }
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
    f.println("Time,Altitude,latitude,longitude,X_acceleration,Y_acceleration,Z_acceleration,impact");
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
  while(  BeforeDS_flag == true ){//ボタンが押されてDSに入るまでGPSの取得のみ行う、DS復帰後はこの処理は行わない
    GPS();
    if ( digitalRead( button_pin ) == LOW ){//カウント処理
      delay( 200 );//チャタリング
      bootCount += 1;
      Serial.println( bootCount );
      while (digitalRead(button_pin) == LOW) {
      // 何もしない
      }
    }
    if ( bootCount == 1 ){
      BeforeDS_flag = false;
      myServo.write( 0 );//原点合わせ
      f = SD.open(file_path, FILE_APPEND);
      f.println("Power Saving Mode");
      f.close();
      digitalWrite(LED_pin , HIGH);
      delay(500);
      digitalWrite(LED_pin , LOW);
      DeepSleep();//DS突入
    }
  }
  while( BeforeDS_flag == false ){
    if ( RturnDS_flag == true ){//復帰後に処理を開始
      Buzzer();//復帰をブザーで確認
      delay(500);
      ledcWriteTone(BUZZER_CHANEL, 0);// 消音
      PLA = bme.readAltitude(SEALEVELPRESSURE_HPA);//ロケット発射前高度の記録
      f = SD.open(file_path, FILE_APPEND);
      f.println("Weke UP");//DS復帰のログ
      f.print("Pre-launch altitude");
      f.println( PLA );
      f.close();
      RturnDS_flag = false;
    }

    lead();
  }
}//loop end

void lead(){
  currentTime = millis();//interval秒で現在時刻を更新
  Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);//interval秒で現在高度を更新
  Altitude = Altitude + AddAltitude;
  if (currentTime - keepTime >= interval ){ //interval秒でデータを取得
    keepTime = currentTime;

    if ( Altitude > maxAltitude ){//最高高度の入れ替え
      maxAltitude = Altitude;
    }
    if ( Altitude - keepAltitude != 0 ){//現在高度の入れ替え
      keepAltitude = Altitude;
    }
    
    if ( maxAltitude - Altitude >= AltitudeDif && fallDet_flag == true  ){//パラシュート展開
      loofOpen();
      fallDet_flag = false;
      fallTime = currentTime;
    }
    GPS();
    Calc_impact();
    SDcard();
    if ( currentTime - fallTime >= par_to_land ){
      if( fallDet_flag == false ){//着地確認
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
      laT = (gps.location.lat());
      lonG = (gps.location.lng());

      
      Serial.print("latitude: ");//緯度
      Serial.print(laT);
      Serial.print("/");
      Serial.print("longitude: ");//経度
      Serial.println( lonG );
    }
  }
}

void loofOpen(){
  pinMode(SERVO_PIN, OUTPUT);//サーボピンのモードを設定
  myServo.attach( SERVO_PIN );
  myServo.write( 120 );//ロックが解除され、パラシュート展開
  f = SD.open(file_path, FILE_APPEND);
  f.println( "************************************************************" );
  f.println( "RoofOpen" );
  f.println( "************************************************************" );
  f.close();
}

void SDcard(){
String data = String(currentTime) + "," + 
              String(Altitude) + "," + 
              String(laT, 7) + "," + 
              String(lonG, 7) + "," + 
              String(ax) + "," + 
              String(ay) + "," + 
              String(az) + "," + 
              String(G);
  f = SD.open(file_path, FILE_APPEND);
  f.println(data);
  f.close();
}

void Calc_impact(){
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
}

void DeepSleep(){
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, LOW);
  esp_deep_sleep_start();                      //スリープモードへ移行
}

void Buzzer(){
  ledcSetup(BUZZER_CHANEL, 12000, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANEL);
  ledcWriteTone(BUZZER_CHANEL, 440); // ラ
}
