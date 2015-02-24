#ifndef __MATRIX_H
#define __MATRIX_H

#ifdef __cplusplus
 extern "C" {
#endif 


void matrix_multiply(float* A, float* B, unsigned char m, unsigned char p, unsigned char n, float* C);
void matrix_addition(float* A, float* B, unsigned char m, unsigned char n, float* C);
void matrix_subtraction(float* A, float* B, unsigned char m, unsigned char n, float* C);
void matrix_transpose(float* A, unsigned char m, unsigned char n, float* C);
void matrix_copy(int m, int n, float* A, float* B);
int matrix_inversion(float* A, int n, float* AInverse);


#ifdef __cplusplus
}
#endif

#endif /* __MATRIX_H */


