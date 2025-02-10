#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <FlexCAN_T4.h>
#include <stdio.h>
#include <math.h>
#include <bmi323.h>
#include <common.h>
#include <SdFat.h>
#include <RingBuf.h>
#include <string.h>
#include <EKF.h>

#define SD_CONFIG SdioConfig(FIFO_SDIO)
//I2C takes way too fucking long, hence 6ms 
//this also means that we are doing 125Hz
//which is half the speed I wanted but it is what it is

#define LOG_INTERVAL 8000
/*
The idea is to log data at 250 hertz
For the wheel sensors, we have 3 sensors
IMU can produce 7 16 bit values
MLX can produce 2 16 bit values
The wheelspeed sensor can be stored in a 1 bit value
The time ping from the main IMU is stored as 1 bit value
The micros are stored as an unsigned long 32 bits
The millis are stored as an unsigned long 32 bits
The total number of bits is 7*16 + 2*16 + 1 + 1 + 32 + 32 = 160 bits
We include a comma character between every value to make it easier to parse
The total number of characters is 160 + 6 = 166 bits per line 
Round this up to get 168/2 = 84
^^^ this math is all worng
from testing its ~65 chars per line
65 * 8 * 125 per second
60 seconds in a minute at most 1 hour run
fuck me 
65 * 8 * 125 * 60 * 60 
*/
//60 minute log files
// divided by 10 for testing purposes
#define LOG_FILE_SIZE_COMP 75*8*125*60*60 //270MB
#define LOG_FILE_SIZE_TEST 75*8*125*60*20 //90MB
#define LOG_FILE_SIZE_SMALL 75*8*125*60*5 //22.5MB
#define LOG_FILE_SIZE_SUPER_SMALL 75*8*125*60 //4.5MB
//want to store 2s of data in RB
#define RINGBUF_SIZE 75*8*125 //75kB

uint8_t flush_ctr;

String file_type = ".csv";
String file_start = "0";

char file_name[8] = {0x00};
bool file_full = false;

uint32_t time_ref_mil = 0;
uint32_t time_ref_mic = 0;

SdFs sd;
FsFile file;

RingBuf<FsFile, RINGBUF_SIZE> ringbuf;

//LED
const int ledPin = LED_BUILTIN;
//MLX sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//BMI sensor
bmi323 bmi323_dev;


//for KALmna
int16_t acc[3] = {0};
int16_t gyro[3] = {0};

EKF efk;

#define KALMAN_P 0.01f
#define KALMAN_Q 0.0001f
#define KALMAN_R 0.011f

#define PREDICT 10
#define UPDATE 100

float KALMANP[2] = {KALMAN_P, KALMAN_P};
float KALMANQ[2] = {KALMAN_Q, KALMAN_Q};
float KALMANR[3] = {KALMAN_R, KALMAN_R, KALMAN_R};

//error func
void error();

void logger(char *file_name, RingBuf<FsFile, RINGBUF_SIZE> *ringbuf, FsFile& file);

void setup() {
  //Serial intialization
  Serial.begin(115200);
  delay(1000);
  //I2C initialization
  //fast I2c init
  Wire.setClock(1000000);
  Wire.begin();
  // while(true){
  //   Serial.println("HI");
  //   delay(1);
  // }
  //BMI323 initialization
  if(bmi323_init(&bmi323_dev, &Wire)){
    D_println("Error connecting to BMI sensor.");
    error();
  }
  //MLX90614 initialization
  if(!mlx.begin()){
    D_println("MLX sensor failed to initialize.");
    error();
  }
  //SD card initialization
  // if(!sd.begin(SD_CONFIG)){
  //   D_println("SD card failed to initialize.");
  //   error();
  // }
  // //check if the file exists
  // D_println(file_start + file_type);
  // if(sd.exists(file_start + file_type)){
  //   D_println("File exists.");
  //   //if the file exists we want to create a new file with a new name
  //   for(int i = 0; i < 1000; i++){
  //     if(!sd.exists(file_start + file_type)){
  //       break;
  //     }
  //     file_start = String(i);
  //   }
  // }else{
  //   D_println("File does not exist.");
  // }
  // //create the file

  // D_println(file_start + file_type);
  // (file_start + file_type).toCharArray(file_name, 8);
  // D_println(file_name);

  // if(!file.open(file_name, O_RDWR | O_CREAT | O_TRUNC)){
  //   D_println("Error opening file.");
  //   file.close();
  //   error();
  // }
  // //preallocate the
  // //dont need to do this i think?
  // // if(!file.preAllocate(LOG_FILE_SIZE_SUPER_SMALL/10)){
  // //   D_println("Error preallocating file.");
  // //   file.close();
  // //   error();
  // // }
  // file.close();
  //initialize the ring buffer
  


  //the idea is to enable the imu in high performance, in 6400kHz, divide by 64 to get 1000kHz availible samples
  //then we just poll that at 125Hz to get every ~8th sample.
  bmi323_enable_acc(&bmi323_dev, HIGH_PERF, AVG_64, ODR_DIV_2, ACC_RANGE_4G, ODR_6400);
  bmi323_enable_gyro(&bmi323_dev, HIGH_PERF, AVG_64, ODR_DIV_2, 0x1, ODR_6400);
  uint32_t time_ref = millis();
  bmi323_calib(&bmi323_dev);
  D_println(millis() - time_ref);
  time_ref_mil = millis();
  time_ref_mic = micros();
  flush_ctr = 0;
  //make sure everything is actually setup and give it some breathing time
  EKF_Init(&efk, KALMANP, KALMANQ, KALMANR);
  delay(500);

}

void logger(char *file_name, RingBuf<FsFile, RINGBUF_SIZE> *ringbuf, FsFile& file){
  if(file_full){
    return;
  }
  if(!file.open(file_name, O_RDWR)){
    D_println("Error opening file.");
    file.close();
    error();
  }
  ringbuf->begin(&file);
  while(!file_full){
    size_t n = ringbuf->bytesUsed();
    
    if ((n + file.curPosition()) > (LOG_FILE_SIZE_TEST - 100)) {
        D_println(n);
        D_println(file.curPosition());
        D_println(LOG_FILE_SIZE_TEST);
        D_println("File full - quitting.");
        
        break;
    }
    if(n >4096 && !file.isBusy()){
      if(4096 != ringbuf->writeOut(4096)){
        
        D_println("Error writing to file.");
        break;
      }else{
        if(flush_ctr >= 4){
          file.flush();
          flush_ctr=0;
        }else{
          flush_ctr++;
        }
      }
    }
    if(micros() - time_ref_mic >= LOG_INTERVAL){
      time_ref_mic = micros();
      mlx.readAmbientTempC();
      mlx.readObjectTempC();
      ringbuf->print(short(bmi323_read_acc_x(&bmi323_dev)));
      ringbuf->write(',');
      ringbuf->print(short(bmi323_read_acc_y(&bmi323_dev)));
      ringbuf->write(',');
      ringbuf->print(short(bmi323_read_acc_z(&bmi323_dev)));
      ringbuf->write(',');
      ringbuf->print(short(bmi323_read_gyr_x(&bmi323_dev)));
      ringbuf->write(',');
      ringbuf->print(short(bmi323_read_gyr_y(&bmi323_dev)));
      ringbuf->write(',');
      ringbuf->print(short(bmi323_read_gyr_z(&bmi323_dev)));
      ringbuf->write(',');
      ringbuf->print(short(mlx.readAmbientTempC()));
      ringbuf->write(',');
      ringbuf->print(mlx.readObjectTempC());
      ringbuf->write(',');
      ringbuf->print(0); //this is for the wheel speed sensor
      ringbuf->write(',');
      ringbuf->print(0); //this is for the time ping from the main IMU
      ringbuf->write(',');
      ringbuf->print(micros());
      ringbuf->write(',');
      ringbuf->print(millis());
      ringbuf->write('\n');
    }
  }
  D_println("file filled");
  file_full = true; 
  ringbuf->sync();
  file.truncate();
  file.close();
}

void loop() {
  // logger(file_name, &ringbuf, file);
  // D_println("Logger BAD");
  // delay(1000); 
  //delay(6);

  // accelerations 
  acc[0] = bmi323_read_acc_x(&bmi323_dev);
  acc[1] = -bmi323_read_acc_y(&bmi323_dev);
  acc[2] = -bmi323_read_acc_z(&bmi323_dev);

  // gyro
  gyro[0] = bmi323_read_gyr_x(&bmi323_dev);
  gyro[1] = -bmi323_read_gyr_y(&bmi323_dev);
  gyro[2] = -bmi323_read_gyr_z(&bmi323_dev);


  if(millis() - time_ref_mil >  PREDICT){
    time_ref_mil = millis();
    // Serial.println("GOOD");
    EKF_Predict(&efk, gyro[0]/131.072f * PI/180.0f, gyro[1]/131.072f * PI/180.0f, gyro[2]/131.072f * PI/180.0f, PREDICT * 0.001f);
  }
  if(millis() - time_ref_mic > UPDATE){
    time_ref_mic = millis();
    // Serial.println("BAD ");
    EKF_Update(&efk, acc[0]/8192.0f * 9.8066, acc[1]/8192.0f * 9.8066, acc[2]/8192.0f * 9.8066);
    Serial.print("phi: ");
    Serial.print(efk.phi_r * 180/PI);
    Serial.print(" theta: ");
    Serial.print(efk.theta_r * 180/PI);
    Serial.print(" acc: ");
    Serial.print(acc[0]/8192.0);
    Serial.print(", ");
    Serial.print(acc[1]/8192.0);
    Serial.print(", ");
    Serial.print(acc[2]/8192.0);
    Serial.print(" gyro: ");
    Serial.print(gyro[0]/500.0);
    Serial.print(", ");
    Serial.print(gyro[1]/500.0);
    Serial.print(", ");
    Serial.print(gyro[2]/500.0);
    Serial.print(" P: ");
    Serial.print(efk.P[0][0]);
    Serial.print(", ");
    Serial.print(efk.P[0][1]);
    Serial.print(", ");
    Serial.print(efk.P[1][0]);
    Serial.print(", ");
    Serial.print(efk.P[1][1]);
    Serial.print("\n");
  }

  

  
  // fxu[0] = 1 * gyro[0] + sin(x_n[0]* tan(x_n[1])) * gyro[1] + cos(x_n[0]* tan(x_n[1])) * gyro[2];
  // fxu[1] = cos(x_n[0]) * gyro[1] - sin(x_n[0]) * gyro[2];

  //prediction 

  // x_n[0] = x_n[0] + T * fxu[0];
  // x_n[1] = x_n[1] + T * fxu[1];




  /*
  EKF stuff
  1. calculate the \x hat + 1 = \x hat + T * f(\x hat, u)
  2. Update P where P is the error covariance matrix  P = (E\x hat * \x hat Transpose)
  3. Use measurements from teh sensor to correct the predictions: \x hat = \x hat + K * (y - h(\x hat, u))
  4. Update P where P should decrease 
  */
}


void error(){
  while(1){
    //SOS
    //S
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    //O
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(200);
    //S
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
    delay(2000);
  }
}



 