#include <stdlib.h>
#include <stdio.h> 

#include "controller.h" 

int main(void) {
    Controller controller; 
    initialize_controller_default(&controller); 
    printf("controller num sensors: %d\n", controller.num_sensors); 
    return 0; 
}
