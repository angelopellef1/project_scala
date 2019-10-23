
#include <Wire.h>
#include <Math.h>
#include <MPU6050.h>

#define MOD_THRESOLD_U  250
#define MOD_THRESOLD_D  100
#define CALIBRATION_CYCLES  20

const int MPU_addr= 0x68;
int32_t aX,aY,aZ,tE;
float modu;


MPU6050 mpu ;

typedef struct
{
  int32_t x0,y0,z0;
  float mod0;
  int32_t mod_thresold_down;  //threasold going down ( higher)
  int32_t mod_thresold_up;  //threasold goind up ( lower)
}CLB_Type;


typedef enum 
{
    CONNE = 0x00,
    CALIB,
    READ_OP,
    EVALUATING,
    NET_SEND,
}fsm_st;



CLB_Type calib;
int32_t calib_cycles = 0;

/*Samples variables*/
float MEA_Array[4]; //evaluate direction on 4 samples
bool acceptable_data = false;
int32_t s = 0;

/*machine var*/
fsm_st fsm = CALIB;



void setup() 
{
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  
  checkSettings();
  calib.mod_thresold_up = MOD_THRESOLD_U;
  calib.mod_thresold_down = MOD_THRESOLD_D;
  fsm = CALIB;
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}





void Tsk_MPU()
{
   
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  aX = rawAccel.XAxis;
  aY = rawAccel.YAxis;
  aZ = rawAccel.ZAxis;
  modu = sqrt(aX^2 + aY^2 +aZ^2);
//  Serial.print("X: ");Serial.print(aX);
//  Serial.print("| Y: ");Serial.print(aY);
//  Serial.print("| Z: ");Serial.print(aZ);
//  Serial.print("| MOD: ");Serial.println(modu);
  delay(100);    
}

void Tsk_ResetQueue()
{
  s=0;
  memset(MEA_Array,0x00,sizeof(MEA_Array));
  acceptable_data = false;
}

void loop() {
  // put your main code here, to run repeatedly:
  //PrintGets();

  switch(fsm)
  {
    case CONNE:

    break;

    case CALIB:
      Tsk_MPU();
      calib.x0 += aX;
      calib.y0 += aY;
      calib.z0 += aZ;
      calib.mod0 += modu;
      calib_cycles++;
      if(calib_cycles == CALIBRATION_CYCLES)
      {
        calib_cycles = 0;
          calib.x0 /=  CALIBRATION_CYCLES;
          calib.y0 /=  CALIBRATION_CYCLES;
          calib.z0 /=  CALIBRATION_CYCLES;
          calib.mod0 /= CALIBRATION_CYCLES; //base on this
           Serial.println("DBG calibration ended");
          fsm = READ_OP;
      }

    break;

    case READ_OP:
      Tsk_MPU();
      aX -= calib.x0;
      aY -= calib.y0;
      aZ -= calib.z0;
      modu -= calib.mod0;
      //Serial.print("|Z: ");Serial.println(aZ);
      if((aZ > MOD_THRESOLD_U) || (aZ < -MOD_THRESOLD_U))
      {
          Serial.println("DBG Collect");
          if (aZ <0)
          {
            aZ *= -1;
          }
          Serial.print("|Z: ");Serial.println(aZ);
          MEA_Array[s] = aZ;
          s++;
          if(s>=3)
          {
            acceptable_data = true;
            Serial.println("DBG Acceptable data");
            fsm = EVALUATING;
          }
      }
       fsm = EVALUATING;
    break;

    case EVALUATING:
      if(acceptable_data)
      { 
        if(MEA_Array[0] > MEA_Array[1]  )
        {
          if(MEA_Array[1] > MEA_Array[2]  )
          {
            Serial.println("DBG SCENDOOOOO");
          }
        }
        if(MEA_Array[0] < MEA_Array[1]  )
        {
          if(MEA_Array[1] < MEA_Array[2]  )
          {
            Serial.println("DBG SALGOOO");
          }
        }
        
         fsm = NET_SEND;
         Tsk_ResetQueue();
         
      }
      else
      {
         fsm = READ_OP;
      }
    break;

    case NET_SEND:
       Serial.println("DBG Sending something to the net");
        calib.x0 = 0;
           calib.y0 = 0;
            calib.z0 = 0;
      fsm = CALIB;
    break;

  }
  
  Tsk_MPU();

}
