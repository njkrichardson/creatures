#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include <stdbool.h> 

typedef struct {
    // parameters 
    double collide_distance_threshold; 
    double runaway_force_threshold; 
    double significant_force_threshold; 
    double avoid_supress_time; 

    // behavioral state (historical)
    double previous_wander_time; 
    double* previous_avoid_heading; 
    double* previous_heading; 
    double* previous_wander; 
    double previous_time; 

    // scheduling 
    int wander_period; 

    // sensors
    int num_sensors; 
    int* sonar_radian_offsets; 
    double** sonar_basis_vectors; 
} Controller; 

// Controller private methods 
double* feel_force(Controller*, double*); 
bool collide(Controller*, double*); 
double* runaway(Controller*, double*); 
double* wander(Controller*); 
double* avoid(Controller*, double*, double*); 

// Public API 
void initialize_controller_default(Controller*); 
void free_controller(Controller*); 
void reset(Controller*); 
double* call(Controller*, double*, double); 

#endif
