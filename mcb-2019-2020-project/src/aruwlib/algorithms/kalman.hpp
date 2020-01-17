/**
  * Robomaster source code code  
  */

#ifndef __KALMAN_HPP__
#define __KALMAN_HPP__

typedef struct {
    float X_last;  // last optimal prediction
    float X_mid;   // forcast optimal prediction
    float X_now;   // current optimal prediction
    float P_mid;   // predicted covariance
    float P_now;   // current covariance
    float P_last;  // previous covariance
    float kg;  // kalman gain
    float A;   // system parameters
    float B;
    float Q;
    float R;
    float H;
} ExtKalman;

void KalmanCreate(ExtKalman *p, float T_Q, float T_R);
float KalmanFilter(ExtKalman* p, float dat);

#endif
