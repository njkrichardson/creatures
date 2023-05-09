#include <stdio.h> 

#include "utils.h" 

void initialize_to_zeros(double* vector, size_t size) {
    for (int i=0; i < size; i++) vector[i] = 0.0; 
}

void print_double_vector(double* vector, size_t size) {
    printf("("); 
    for (int i=0; i < size; i++) {
        printf("%f ", vector[i]); 
    }
    printf(")\n"); 
}
