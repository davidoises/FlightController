#include "IMU.h"

int16_t gyrFiltered[3];

typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;
    
    // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;

    cosx = cosf(delta[0]);
    sinx = sinf(delta[0]);
    cosy = cosf(delta[1]);
    siny = sinf(delta[1]);
    cosz = cosf(delta[2]);
    sinz = sinf(delta[2]);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}


//void getEstimatedAttitude(){
void getEstimatedAttitude(int16_t* acc, int16_t* gyr)
{
  int32_t axis;
  int32_t accMag = 0;
  float deltaGyroAngle[3];

  static t_fp_vector EstG = {0, 0, ACC_1G};

  static uint32_t previousT;
  uint32_t currentT = micros();
  uint32_t deltaT = currentT - previousT;
  float scale = deltaT * GYRO_SCALE;
  previousT = currentT;
  
  // Initialization
  for (axis = 0; axis < 3; axis++) {
    gyrSmooth[axis] = 0.7*gyrSmooth[axis] + 0.3*gyr[axis];
    gyrFiltered[axis] = 0.95*gyrFiltered[axis] + 0.05*gyr[axis];
    accSmooth[axis]  = 0.99*accSmooth[axis] + 0.01*acc[axis];
    
    accMag += (int32_t)accSmooth[axis] * accSmooth[axis]; 

    // This could/shoule be done using filtered gyro
    //deltaGyroAngle[axis] = gyr[axis] * scale;
    deltaGyroAngle[axis] = gyrFiltered[axis] * scale;
  }
  
  /*
  Serial.print(accSmooth[0]);
  Serial.print(" ");
  Serial.print(accSmooth[1]);
  Serial.print(" ");
  Serial.println(accSmooth[2]);
  */
  
  accMag = accMag * 100 / ((int32_t)ACC_1G * ACC_1G);

  rotateV(&EstG.V, deltaGyroAngle);

  if (72 < (uint16_t)accMag && (uint16_t)accMag < 133) {
    for (axis = 0; axis < 3; axis++)
      EstG.A[axis] = EstG.A[axis]*0.99  + accSmooth[axis]*0.01;
  }

  float anglerad[2] = { 0.0f, 0.0f };    // absolute angle inclination in radians

  // Attitude of the estimated vector
  anglerad[0] = atan2f(EstG.V.Y, EstG.V.Z);
  anglerad[1] = atan2f(-EstG.V.X, sqrtf(EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z));
  angle[0] = lrintf(anglerad[0] * (1800.0f / M_PI));
  angle[1] = lrintf(anglerad[1] * (1800.0f / M_PI));
}
