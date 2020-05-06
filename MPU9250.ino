#include "I2CHandler.h"
#include "MPU9250.h"
#include "AttitudeEstimator.h"
#include "EEPROM.h"

//#define CALIBRATE

I2CHandler i2c;
MPU9250 mpu;
AttitudeEstimator est;
unsigned long last = 0;
float ax,ay,az,gx,gy,gz,mx,my,mz;
float e[3];

void setup() {
  Serial.begin(115200);
  if(!mpu.Initialize(&i2c)){
    while(1){
      Serial.println("Init Failed");
      delay(300);
    }
  }
  
  #ifdef CALIBRATE
  for(int i = 0; i < 100; i++){
    float x,y,z;
    while(!mpu.ReadGyr(&x,&y,&z));
    e[0] += x;
    e[1] += y;
    e[2] += z;
    delay(5);
  }
  e[0] /= 100;
  e[1] /= 100;
  e[2] /= 100;
  
  for(int i = 0; i < 3; i++){
      uint8_t *temp = (uint8_t*)&e[i];
      for(int j = 0; j < sizeof(float); j++){
        EEPROM.write( i * sizeof(float) + j, temp[j]);
    }
  }
  #else
  for(int i = 0; i < 3; i++){
    uint8_t temp[4];
    for(int j = 0; j < sizeof(float); j++){
      temp[j] = EEPROM.read( i * sizeof(float) + j);
    }
    e[i] = *(float*)temp;
  }
  #endif
}

void loop() {
  if(mpu.ReadAccGyr(&ax,&ay,&az,&gx,&gy,&gz)){
    if(mpu.ReadMag(&mx,&my,&mz)){
      unsigned long now = micros();
      est.Update(gx - e[0],gy - e[1],gz - e[2],ax,ay,az,mx,my,mz,(now - last) * 0.000001);
      last = now;
    }else{
      unsigned long now = micros();
      est.Update(gx - e[0],gy - e[1],gz - e[2],ax,ay,az,(now - last) * 0.000001);
      last = now;
    }
    
    float x,y,z,w;
    est.GetQuaternion(&x,&y,&z,&w);
    float wax = ax,way = ay,waz = az;
    est.vectorRotateToWorld(&wax,&way,&waz);
    float wmx = mx,wmy = my,wmz = mz,mcx,mcy,mcz;
    est.GetMagCenter(&mcx,&mcy,&mcz);
    wmx -= mcx;
    wmy -= mcy;
    wmz -= mcz;
    est.vectorRotateToWorld(&wmx,&wmy,&wmz);

    float val[] = {x,y,z,w,wax,way,waz,wmx,wmy,wmz};
    for(int i = 0;i < sizeof(val) / sizeof(float);i++){
      Serial.print(val[i],6);
      Serial.print(' ');
    }
    Serial.println();
  }
  delay(0);
}
