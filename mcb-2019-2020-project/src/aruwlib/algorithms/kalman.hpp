/**
  * Robomaster source code code  
  */

#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

typedef struct {
    float X_last;
    float X_mid;
    float X_now;
    float P_mid;
    float P_now;
    float P_last;
    float kg;
    float A;
    float B;
    float Q;
    float R;
    float H;
} ExtKalman;

void KalmanCreate(ExtKalman *p, float T_Q, float T_R);
float KalmanFilter(ExtKalman* p, float dat);

#endif
