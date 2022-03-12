/*  Kode Sumber Perangkat Directional Finder
 *  Nama  : Muhammad Ihsan Al Hafiz
 *  Teknik Fisika Angkatan 2013
 *  Departemen Teknik Nuklir dan Teknik Fisika
 *  Fakultas Teknik Universitas Gadjah Mada
 *  24 Mei 2017
 */

#include <math.h>
#include <Servo.h>
#include <Wire.h>
#include <HMC5883L.h>
//#include <TinyGPS.h>
#include <TinyGPS++.h>
#include <Scheduler.h>

int PWM_pan;
float LatDF ;
float LatMRS ;
float LongDF ;
float LongMRS ;
float latMRS_last ; 
float longMRS_last ; 
float AltitudeMRS = 0.0 ;
float AltitudeDF = 0.0 ;
float Sudut_MRS;
float Sudut_DF;
float Sudut_MRSPred;
float Error_Pan;
float Error_Tilt;
float Error_PanPred;
float deltaTimePan;
float deltaTimeTilt;
float prosesTerakhirPan;
float prosesTerakhirTilt;
float KecSudutMRS;
float SudutPrediksi;
float JarakMRS;
int SudutTilt;
int SudutTiltLast = 0;
int SudutAwalTilt = 0;
int SudutAkhirTilt;
unsigned long waktuservopan;
unsigned long waktuservotilt;
unsigned long waktuprosespan;
unsigned long WaktuSekarang_Kec;
unsigned long WaktuLampau_Kec;

String dataIn;
String dt[100];
boolean parsing = false;
int i;

float outMax = 200;
float outMin = 0;

double 
  kp_pan = 5;

double P_Pan;

//TinyGPS gpsmrs;
TinyGPSPlus gpsdf;
HMC5883L compass;
Servo servo_pan;
Servo servo_tilt;

void setup() {
  Serial.begin(57600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  inisialisasi_kompas();  
  //ParsingGPS_DF();
  LatDF = -7.7662762;
  LongDF = 110.3727803;
    
  servo_pan.attach(5);
  servo_pan.writeMicroseconds(1500);
  servo_tilt.attach(8);
  servo_tilt.write(135);
  delay(2000);

  Scheduler.startLoop(loop2);
  LatMRS = -7.7678920;
  LongMRS = 110.3719836;
}

void loop() {    
  ParsingGPS_MRS();
  hitung_sudut_pesawat(LatDF, LongDF, LatMRS, LongMRS);
  sudutDF();
  Error_Pan = abs(Sudut_MRS - Sudut_DF);
  HitungKecepatan(LatMRS, LongMRS);
  PrediksiPan(KecSudutMRS, Error_Pan);
  Error_PanPred = Error_Pan + SudutPrediksi;
  P_Pan = (kp_pan*Error_PanPred);
  if(Sudut_MRS > Sudut_DF) PWM_pan = map(P_Pan,0,200,1500,1000);
  else if(Sudut_MRS < Sudut_DF) PWM_pan = map(P_Pan,0,200,1500,2000);
  else if(abs(Sudut_MRS - Sudut_DF) <=1) PWM_pan=1500;
  servo_pan.writeMicroseconds(PWM_pan);
  yield();
}

void loop2() {
  ParsingGPS_MRS();
  haversine(LatMRS, LongMRS, LatDF, LongDF);
  sudutTiltMRS(JarakMRS,AltitudeMRS);
  GerakServoTilt(SudutTiltLast, SudutTilt);
  SudutTiltLast = SudutTilt;
  yield();
}

/////////////////////////////
//Parsing Data Simulasi
////////////////////////////



////////////////////////////////////////////////////////
// Fungsi Parsing Data GPS Mobile Remote System (MRS) //
////////////////////////////////////////////////////////
void ParsingGPS_MRS(){
  if (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    dataIn += inChar;
    if (inChar == '\n') {
      parsing = true;
    }
  }

  if (parsing) {
    parsingData();
    parsing = false;
    dataIn = "";
  }
}

void parsingData()
{
  int j = 0;

  //kirim data yang telah diterima sebelumnya
  Serial.print("data masuk : ");
  Serial.print(dataIn);
  Serial.print("\n");

  //inisialisasi variabel, (reset isi variabel)
  dt[j] = "";
  //proses parsing data
  for (i = 1; i < dataIn.length(); i++) {
    //pengecekan tiap karakter dengan karakter (#) dan (,)
    if ((dataIn[i] == '#') || (dataIn[i] == ','))
    {
      //increment variabel j, digunakan untuk merubah index array penampung
      j++;
      dt[j] = "";     //inisialisasi variabel array dt[j]
    }
    else
    {
      //proses tampung data saat pengecekan karakter selesai.
      dt[j] = dt[j] + dataIn[i];
    }
  }
  LatMRS = dt[0].toFloat();
  LongMRS = dt[1].toFloat();
  AltitudeMRS = dt[2].toFloat();
}
/////////////////////////////////////////////////////
// Fungsi Parsing Data GPS Directional Finder (DF) //
/////////////////////////////////////////////////////
void ParsingGPS_DF(){
    while (Serial1.available() > 0)
    if (gpsdf.encode(Serial1.read())){
      LatDF = gpsdf.location.lat();
      LongDF = gpsdf.location.lng();
    }
}

/////////////////////////////////////
// Fungsi inisialisasi awal kompas //
/////////////////////////////////////
void inisialisasi_kompas(){
  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(-107, -426); // nilai offset diambil dari nilai kalibrasi
}

/////////////////////////////////////////////////////////
// Fungsi Akuisisi data kompas pada directional finder //
/////////////////////////////////////////////////////////
float sudutDF(){
  Vector norm = compass.readNormalize();

  // menghitung heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // menghitung dan mempertimbangkan magnetic declination
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (0.0 + (56.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // mengubah ke sudut
  float headingDegrees = heading * 180/M_PI; 

  //int smoothHeadingDegrees = round(headingDegrees);
  Sudut_DF = headingDegrees;
}

//////////////////////////////
// menghitung sudut pan MRS //
//////////////////////////////
double hitung_sudut_pesawat(float latT, float longT, float latP, float longP) {
  float x = cos(latP * 0.01745329252) * sin(abs(longT - longP) * 0.01745329252);
  float y = cos(latT * 0.01745329252) * sin(latP * 0.01745329252) - sin(latT * 0.01745329252) * cos(latP * 0.01745329252) * cos(abs(longT - longP) * 0.01745329252);
  Sudut_MRS = atan2(x, y);
  Sudut_MRS = (Sudut_MRS / PI) * 180.0;

  if (longP < longT) {
    float Sudut_pesawat_cor = 180.0 - Sudut_MRS;
    Sudut_MRS = 180.0 + Sudut_pesawat_cor;
  }
  else
  {
    Sudut_MRS = Sudut_MRS;
  }
}

///////////////////////
// Formula Haversine //
///////////////////////
float haversine(float latMRS, float longMRS, float latDF, float longDF){
  const float conRad = PI/180.0;
  const float R = 6371;
  float dlat = abs(latMRS-latDF)*conRad;
  float dlong = abs(longMRS-longDF)*conRad;

  float a = sin(dlat/2)*sin(dlat/2) + cos(latMRS*conRad)*cos(latDF*conRad)*sin(dlong/2)*sin(dlong/2);
  float c = 2*atan2(sqrt(a),sqrt(1-a));
  float d = (R*c)*1000.0;
  JarakMRS = d;
}

///////////////////////////////
// menghitung sudut tilt MRS //
///////////////////////////////
float sudutTiltMRS(float d, float alt){
  const float conRad = PI/180.0;
  float e = sqrt((d*d)+(alt*alt));
  float cosB = (d/e);
  float B = (acos(cosB))/conRad;
  SudutTilt = round(B);
}

//////////////////////////
// hitung kecepatan MRS //
//////////////////////////
float HitungKecepatan(float latMRS, float longMRS){
  WaktuSekarang_Kec = micros();
  if(latMRS_last==0 && longMRS_last==0){
    latMRS_last = latMRS;
    longMRS_last = longMRS;
    return 0;
  }
  
  float n = cos(latMRS * 0.01745329252) * sin(abs(LongDF - longMRS) * 0.01745329252);
  float m = cos(LatDF * 0.01745329252) * sin(latMRS * 0.01745329252) - sin(LatDF * 0.01745329252) * cos(latMRS * 0.01745329252) * cos(abs(LongDF - longMRS) * 0.01745329252);
  float SudutSekarang = atan2(n, m);
  SudutSekarang  = (SudutSekarang / PI) * 180.0;

  if (latMRS < LongDF) {
    float Sudut_pesawat_cor = 180.0 - SudutSekarang;
    SudutSekarang = 180.0 + Sudut_pesawat_cor;
  }
  else
  {
    SudutSekarang = SudutSekarang;
  }
  
  float q = cos(latMRS_last * 0.01745329252) * sin(abs(LongDF - longMRS_last) * 0.01745329252);
  float w = cos(LatDF * 0.01745329252) * sin(latMRS_last * 0.01745329252) - sin(LatDF * 0.01745329252) * cos(latMRS_last * 0.01745329252) * cos(abs(LongDF - longMRS_last) * 0.01745329252);
  float SudutLampau = atan2(q, w);
  SudutLampau  = (SudutLampau / PI) * 180.0;

  if (longMRS_last < LongDF) {
    float Sudut_pesawat_cor1 = 180.0 - SudutLampau;
    SudutLampau = 180.0 + Sudut_pesawat_cor1;
  }
  else
  {
    SudutLampau = SudutLampau;
  }

  float KecSudut = (abs(SudutSekarang-SudutLampau)) / (WaktuSekarang_Kec-WaktuLampau_Kec);
  WaktuLampau_Kec = WaktuSekarang_Kec;
  latMRS_last = latMRS;
  longMRS_last = longMRS;
  KecSudutMRS = KecSudut;
}

/////////////////////
// prediksi target //
/////////////////////
float PrediksiPan(float kecsudut, float error){
  float prediksi = kecsudut * (error/50.0);
  SudutPrediksi = prediksi;
}

/////////////////////////
// Gerak Sudut Elevasi //
/////////////////////////
float GerakServoTilt(int pwmlast, int pwm){
  pwmlast = map(pwmlast,0,90,135,50);
  pwm = map(pwm,0,90,135,50);
  int pos;
  if(pwmlast < pwm){
    for (pos = pwmlast; pos <= pwm; pos += 1) { 
    // in steps of 1 degree
      servo_tilt.write(pos);      
      delay(40);                       
    }
  }
  else{
    for (pos = pwmlast; pos >= pwm; pos -= 1) { 
      servo_tilt.write(pos);       
      delay(40);                       
    }
  }
}


