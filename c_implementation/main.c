#include <stdlib.h>
#include <stdio.h> 

#include "controller.h" 
#include "utils.h"

int main(void) {
    Controller controller; 
    initialize_controller_default(&controller); 
    printf("controller num sensors: %d\n", controller.num_sensors); 

    double* distances = malloc(4 * sizeof(double)); 

    distances[0] = 1.0; 
    distances[1] = 1.0; 
    distances[2] = 1.0; 
    distances[3] = 1.0; 

    double* desired_velocity = call(&controller, distances, 0.0); 

    printf("Desired velocity: "); 
    print_double_vector(desired_velocity, 2); 

    free(distances); 
    free(desired_velocity); 
    return 0; 
}
