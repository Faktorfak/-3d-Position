#include <Arduino_LSM9DS1.h>
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float xR, yR, zR;
float rX, rY, rZ; 
float gravity = 1;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float posX = 0, posY = 0, posZ = 0;
float lastPosX = 0, lastPosY = 0, lastPosZ = 0;
float accelerationX = 0, accelerationY = 0, accelerationZ = 0;
float lastAccelerationX = 0, lastAccelerationY = 0, lastAccelerationZ = 0;
float lastVelocityX = 0, lastVelocityY = 0, lastVelocityZ = 0;
float outX, outY, outZ;

int countx, county, countz ;

float Gscale = (M_PI / 180.0) * 0.00875; 

float G_offset[3] = {-0.111176, -0.781616, -0.580872};

float A_B[3]
{ 0.0049 , -0.04 ,-0.002};

float A_Ainv[3][3]
{ {  1.04036, -0.01699,  0.01123},
  { -0.01699,  0.96608,  0.01616},
  {  0.01123,  0.01616,  0.97912}
};

//Mag scale 
float M_B[3]
{  15.97,   28.38,    2.82};

float M_Ainv[3][3]
{ {  1.38541,  0.02445,  0.01669},
  {  0.02445,  1.40471, -0.00056},
  {  0.01669, -0.00056,  1.46882}
};


float declination = +4.44;


#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; 
float deltat = 0; 

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; 


void movment_end_check(){

if (accelerationX==0) //we count the number of acceleration samples that equals cero
 { countx++;}
 else { countx =0;}

 if (countx>=46) //if this number exceeds 25, we can assume that velocity is cero
  {
    lastVelocityX=0;
    velocityX=0;
   }
//////////////////////////////////////////////////////////////////////////////////////
if (accelerationY==0) //we count the number of acceleration samples that equals cero
 { county++;}
 else { county =0;}

 if (county>=46) //if this number exceeds 25, we can assume that velocity is cero
  {
    lastVelocityY=0;
    velocityY=0;
   }
/////////////////////////////////////////////////////////////////////////////////////////////////
if (accelerationZ==0) //we count the number of acceleration samples that equals cero
 { countz++;}
 else { countz =0;}

 if (countz=46) //if this number exceeds 25, we can assume that velocity is cero
  {
    lastVelocityZ=0;
    velocityZ=0;
   }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial); 
  Serial.println();
  Serial.println("LSM9DS1 AHRS starting");

  if (!IMU.begin()) 
  {
    Serial.println(("LSM9DS1 not detected"));
    while (1);
  }

   IMU.setAccelFS(3);
   IMU.setAccelODR(5);
   IMU.setAccelOffset(0.003079, -0.023211, -0.043487);
   IMU.setAccelSlope (0.995680, 1.003019, 1.008170);

   IMU.setGyroFS(2);
   IMU.setGyroODR(5);
   IMU.setGyroOffset (-0.097168, -0.535614, -0.396851);
   IMU.setGyroSlope (1.178305, 1.129912, 1.117383);

}

void loop()
{

  static float Gxyz[3], Axyz[3], Mxyz[3]; 

  
  if ( IMU.accelerationAvailable() && IMU.magneticFieldAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readMagneticField(mx, my, mz);
    IMU.readGyroscope(gx, gy, gz);
  }
    get_scaled_IMU(Gxyz, Axyz, Mxyz);


    Axyz[0] = -Axyz[0]; 
    Gxyz[0] = -Gxyz[0]; 
   
    now = micros();
    deltat = (now - last) * 1.0e-6; 
    last = now;

    MagdwickQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);

      
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      
      yaw = -(yaw + declination);


      xR = gravity * sin(PI*pitch/180);
      yR = gravity * sin(PI*roll/180);
      zR = gravity * cos(PI*roll/180) * cos(PI*pitch/180);
     
    if(pitch > 0){
      rX = ax - abs(xR);
    }
    if(pitch < 0){
      rX = ax + abs(xR);
    }
    if(roll > 0){
      rY = ay - abs(yR);
    }
    if(roll < 0){
      rY = ay + abs(yR);
    }

    rZ = az - zR;

    accelerationX = rX;
    accelerationY = rY + 0.05;
    accelerationZ = rZ;

   if ((accelerationX <= 0.05)&&(accelerationX >= -0.05)) //Discrimination window applied to
   {accelerationX = 0;} // the X axis acceleration variable

   if ((accelerationY <= 0.05)&&(accelerationY >= -0.05)) //Discrimination window applied to
   {accelerationY = 0;} // the X axis acceleration variable

   if ((accelerationZ <= 0.1)&&(accelerationZ >= -0.1)) //Discrimination window applied to
   {accelerationZ = 0;} // the X axis acceleration variable

    // First integration (Velocity)
    velocityX = lastVelocityX + lastAccelerationX + ((accelerationX - lastAccelerationX) / 2);
    velocityY = lastVelocityY + lastAccelerationY + ((accelerationY - lastAccelerationY) / 2);
    velocityZ = lastVelocityZ + lastAccelerationZ + ((accelerationZ - lastAccelerationZ) / 2);

    // Second integration (Position)
    posX = lastPosX + lastVelocityX + ((velocityX - lastVelocityX) / 2);
    posY = lastPosY + lastVelocityY + ((velocityY - lastVelocityY) / 2);
    posZ = lastPosZ + lastVelocityZ + ((velocityZ - lastVelocityZ) / 2);

    // Update last values for next iteration
    lastAccelerationX = accelerationX;
    lastAccelerationY = accelerationY;
    lastAccelerationZ = accelerationZ;

    lastVelocityX = velocityX;
    lastVelocityY = velocityY;
    lastVelocityZ = velocityZ;

    movment_end_check();

    lastPosX = posX;
    lastPosY = posY;
    lastPosZ = posZ;
    
    outX = posX/100;
    outY = posY/100;
    outZ = posZ/10;

    
    Serial.print(outX);
    Serial.print(",");
    Serial.print(outY);
    Serial.print(",");
    Serial.println(outZ); 
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}



void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Gxyz[0] = Gscale * (gx - G_offset[0]);
  Gxyz[1] = Gscale * (gy - G_offset[1]);
  Gxyz[2] = Gscale * (gz - G_offset[2]);

  Axyz[0] = ax;
  Axyz[1] = ay;
  Axyz[2] = az;
  Mxyz[0] = mx;
  Mxyz[1] = my;
  Mxyz[2] = mz;

 

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  

  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}


void MagdwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  
  static float eInt[3] = {0.0, 0.0, 0.0};
   
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  
  float ux, uy, uz, wx, wy, wz; 
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
 
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; 

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

 

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);
 
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      
    eInt[1] += ey;
    eInt[2] += ez;
   
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }

 
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

 
 
  gx = gx * (0.5*deltat); 
  gy = gy * (0.5*deltat);
  gz = gz * (0.5*deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}