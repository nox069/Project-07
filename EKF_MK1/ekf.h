#ifndef EKF_H
#define EKF_H

#include "arm_math.h"   // CMSIS-DSP for matrix/quat support

typedef struct {
    float32_t q[4];        // quaternion [w,x,y,z]
    float32_t P[16];       // covariance matrix (4x4 simplified)
    float32_t gyro_bias[3];
} EKF_t;

void EKF_Init(EKF_t *ekf);
void EKF_Predict(EKF_t *ekf, const float gyro[3], float dt);
void EKF_UpdateAccelMag(EKF_t *ekf, const float accel[3], const float mag[3]);
void EKF_GetEuler(const EKF_t *ekf, float *roll, float *pitch, float *yaw);

#endif
