#include <Wire.h>
#include <Servo.h>

#define SET_POIN 0.00
#define MIN 1220
#define MAX 1500
#define P 1.4329
#define I 0.000821
#define D 603.98901

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
unsigned long waktuSekarang,waktuAkhir,waktuSelisih;
float erorAwal, koreksiP, koreksiI,erorAkhir, koreksiD, koreksiTotal,sig1;
Servo esc;

void setup_mpu_6050_registers(){
//Activate the MPU-6050
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
//Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
//Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void setup(){
  Wire.begin();
  Serial.begin(115200);
  pinMode(9,OUTPUT);
  pinMode (10, OUTPUT);
  esc.attach (9);
  esc.attach(10);
  esc.writeMicroseconds(1000);
  setup_mpu_6050_registers();
//Serial.print("tunggu" );
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++)
  {
//Run this code 2000 times
    if(cal_int % 125 ==0)
    //Serial.print(".");
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3) ;
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  loop_timer = micros();
//delay (50) ;
  esc.writeMicroseconds(2000);
}

void read_mpu_6050_data(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,14);
  while(Wire.available() < 14);
  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  temperature = Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}
void hitung(){
  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
//Gyro angle calculations
//0.0000611=1/(250Hz/65.5)
  angle_pitch += gyro_x * 0.0000611;
  angle_roll += gyro_y * 0.0000611;
//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin fu
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
//Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
//Place the MPU-6050 spirit level and note the values in the foll
  angle_pitch_acc -= 0.0;
  angle_roll_acc -= 0.0;
  
  if(set_gyro_angles){
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  }
  else{
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }
  
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
  while(micros() - loop_timer < 4000);
  loop_timer = micros();
}
float hitung_PID(float x)
{
  waktuSekarang = millis();
  waktuSelisih = (waktuSekarang - waktuAkhir);
  erorAwal = SET_POIN - x;
  koreksiP = P*erorAwal;
  koreksiI += (erorAwal-erorAkhir)*waktuSelisih;
  koreksiD = (erorAwal-erorAkhir)/waktuSelisih;

  koreksiTotal = koreksiP + I*koreksiI + D*koreksiD;

  if(koreksiTotal <= -700) {koreksiTotal = -700;}
  if(koreksiTotal >= 700) {koreksiTotal = 700;}
  return koreksiTotal;
}
void loop(){
  hitung();
  Serial.print(angle_roll_output);
  Serial.print("|");
  Serial.println(angle_pitch_output);
  hitung_PID(angle_pitch_output);
  sig1 = MIN + koreksiTotal;
  if(sig1 <= MIN) {sig1 = MIN;}
  if(sig1 >= MAX) {sig1 = MAX;}
  esc.writeMicroseconds(sig1);
  erorAkhir = erorAwal;
  waktuAkhir = waktuSekarang;
}
