#include <ekf.h>
#include <math.h>

static void quat_normalize(float q[4]) {
    float norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if(norm > 0.0f) {
        q[0]/=norm; q[1]/=norm; q[2]/=norm; q[3]/=norm;
    }
}

void EKF_Init(EKF_t *ekf) {
    ekf->q[0] = 1; ekf->q[1] = ekf->q[2] = ekf->q[3] = 0;
    for(int i=0;i<16;i++) ekf->P[i] = 0.0f;
    ekf->P[0] = ekf->P[5] = ekf->P[10] = ekf->P[15] = 1.0f;
    ekf->gyro_bias[0]=ekf->gyro_bias[1]=ekf->gyro_bias[2]=0.0f;
}

void EKF_Predict(EKF_t *ekf, const float gyro[3], float dt) {
    // remove bias
    float gx = gyro[0] - ekf->gyro_bias[0];
    float gy = gyro[1] - ekf->gyro_bias[1];
    float gz = gyro[2] - ekf->gyro_bias[2];

    // quaternion derivative
    float q0=ekf->q[0], q1=ekf->q[1], q2=ekf->q[2], q3=ekf->q[3];
    float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    ekf->q[0] += dq0*dt;
    ekf->q[1] += dq1*dt;
    ekf->q[2] += dq2*dt;
    ekf->q[3] += dq3*dt;
    quat_normalize(ekf->q);
}

void EKF_UpdateAccelMag(EKF_t *ekf, const float accel[3], const float mag[3]) {
    // normalize
    float ax=accel[0], ay=accel[1], az=accel[2];
    float norm = sqrtf(ax*ax+ay*ay+az*az);
    if(norm>0) { ax/=norm; ay/=norm; az/=norm; }

    float mx=mag[0], my=mag[1], mz=mag[2];
    norm = sqrtf(mx*mx+my*my+mz*mz);
    if(norm>0) { mx/=norm; my/=norm; mz/=norm; }

    // quick correction: align accel with gravity and mag with heading
    float roll  = atan2f(ay, az);
    float pitch = -asinf(ax);
    float yaw   = atan2f(-my*cosf(roll)+mz*sinf(roll),
                         mx*cosf(pitch)+my*sinf(pitch)*sinf(roll)+mz*sinf(pitch)*cosf(roll));

    // rebuild quaternion from RPY
    float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f), sy = sinf(yaw*0.5f);

    ekf->q[0] = cr*cp*cy + sr*sp*sy;
    ekf->q[1] = sr*cp*cy - cr*sp*sy;
    ekf->q[2] = cr*sp*cy + sr*cp*sy;
    ekf->q[3] = cr*cp*sy - sr*sp*cy;
    quat_normalize(ekf->q);
}

void EKF_GetEuler(const EKF_t *ekf, float *roll, float *pitch, float *yaw) {
    float q0=ekf->q[0], q1=ekf->q[1], q2=ekf->q[2], q3=ekf->q[3];

    *roll  = atan2f(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2)) *57.3f;
    *pitch = asinf(2*(q0*q2 - q3*q1)) *57.3f;
    *yaw   = atan2f(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) *57.3f;
}


