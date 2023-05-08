#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H 

#include <math.h> 

double norm(double* arr, size_t num_elements) {
    double out = 0.0; 
    for (int i=0; i < num_elements; i++) out += arr[i] * arr[i]; 
    return sqrt(out); 
}


#endif 

