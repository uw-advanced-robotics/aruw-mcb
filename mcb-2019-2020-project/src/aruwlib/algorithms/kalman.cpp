/**    
  * Example source:
  * 
  * extKalman p;
  * float SersorData;
  * KalmanCreate(&p,20,200);
  * while(1)
  * {
  *     SersorData = sersor();
  *     SersorData = KalmanFilter(&p,SersorData);
  * }
  */

#include "kalman.hpp"

void KalmanCreate(ExtKalman *p, float T_Q, float T_R)
{
    p->X_last = 0.0f;
    p->P_last = 0.0f;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1.0f;
    p->B = 0.0f;
    p->H = 1.0f;
    p->X_mid = p->X_last;
}

float KalmanFilter(ExtKalman* p, float dat)
{
    p->X_mid = p->A * p->X_last;
    p->P_mid = p->A * p->P_last + p->Q;
    p->kg = p->P_mid / (p->P_mid + p->R);
    p->X_now = p->X_mid + p->kg * (dat-p->X_mid);
    p->P_now = (1 - p->kg) * p->P_mid;
    p->P_last = p->P_now;
    p->X_last = p->X_now;
    return p->X_now;
}
