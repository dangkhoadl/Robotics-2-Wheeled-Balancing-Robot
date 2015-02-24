#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
 extern "C" {
#endif 

float LPF(float x, float CUTOFF,float SAMPLE_RATE);
float HPF(float x, float CUTOFF,float SAMPLE_RATE);
float kalman_single(float z, float measure_noise, float process_noise);
void kalman(float* in,float* out, float measure_noise, float process_noise);
void Smooth_filter(float rawDATA, float *filteredARRAY, float LPFgain);


#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */


