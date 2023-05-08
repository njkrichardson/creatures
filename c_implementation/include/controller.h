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
    double previous_avoid_heading[2]; 
    double previous_heading[2]; 
    double previous_wander[2]; 
    double previous_time; 

    // scheduling 
    int wander_period; 

    // sensors
    int num_sensors; 
    int sonar_radian_offsets[4]; 
    double sonar_basis_vectors[4][2]; 
} Controller; 

// Controller private methods 
double* feel_force(Controller*, double*); 
bool collide(Controller*, double*); 
double* runaway(Controller*, double*); 
double* wander(Controller*); 
double* avoid(Controller*, double*, double*); 

// Public API 
void initialize_controller_default(Controller*); 
void reset(Controller*); 
double* call(Controller*, double*, double); 

#endif
