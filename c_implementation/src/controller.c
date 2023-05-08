#include <stdbool.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <math.h> 

#include "controller.h"
#include "linear_algebra.h"
#include "utils.h"

void initialize_controller_default(Controller* controller) {
    controller->collide_distance_threshold = 0.1; 
    controller->runaway_force_threshold = 0.1; 
    controller->significant_force_threshold = 0.0; 
    controller->avoid_supress_time = 0.5; 
    controller->num_sensors = 4; 

    controller->sonar_radian_offsets = malloc(controller->num_sensors * sizeof(int)); 
    controller->sonar_radian_offsets[0] = 0; 
    controller->sonar_radian_offsets[1] = 90; 
    controller->sonar_radian_offsets[2] = 180; 
    controller->sonar_radian_offsets[3] = 270; 

    controller->sonar_basis_vectors = (double **) malloc(4 * sizeof(double *));
    for (int i = 0; i < 4; i++) {
        controller->sonar_basis_vectors[i] = (double *) malloc(2 * sizeof(double));
    }
    controller->sonar_basis_vectors[0][0] = 0;
    controller->sonar_basis_vectors[0][1] = 1;

    controller->sonar_basis_vectors[1][0] = 1;
    controller->sonar_basis_vectors[1][1] = 0;

    controller->sonar_basis_vectors[2][0] = 0;
    controller->sonar_basis_vectors[2][1] = -1;

    controller->sonar_basis_vectors[2][0] = -1;
    controller->sonar_basis_vectors[2][1] = 0;

    controller->previous_wander = malloc(2 * sizeof(double)); 
    initialize_to_zeros(controller->previous_wander, 2); 

    controller->previous_avoid_heading = malloc(2 * sizeof(double)); 
    initialize_to_zeros(controller->previous_avoid_heading, 2); 

    controller->previous_time = 0.0f; 
    controller->previous_wander_time = -10.0f; 
    controller->wander_period = 6; 
}

void free_controller(Controller* controller){
    for (int i = 0; i < 4; i++) {
        free(controller->sonar_basis_vectors[i]);
    }
    free(controller->sonar_basis_vectors);
    free(controller->sonar_radian_offsets); 
    free(controller->previous_wander); 
    free(controller->previous_avoid_heading); 
}

double* feel_force(Controller* controller, double* distances) { 
    /* Computes the (repulsive) force from the measured sensor distances. 

    Parameters 
    ----------
    double* distances 
        array of distance measurements (assumed to have length equal to this->num_sensors). 
    */
    size_t num_sensors = controller->num_sensors ;

    double* overall_force = malloc(2 * sizeof(double)); 
    initialize_to_zeros(overall_force, 2); 
    double force_per_sensor[num_sensors]; 

    for (int i=0; i < num_sensors; ++i) force_per_sensor[i] = -1.0 / pow((10.0 * distances[i] + 0.001), 5); 
    for (int i=0; i < num_sensors; ++i) {
        for (int j=0; j < 2; ++j) overall_force[j] += controller->sonar_basis_vectors[i][j] * force_per_sensor[i]; 
    }
    return overall_force; 
}

bool collide(Controller* controller, double* distances) {
    // Note: assumes that the first index of distances holds the sensor with 0 offset 
    int front_facing_index = 0; 
    return distances[front_facing_index] < controller->collide_distance_threshold; 
}

double* runaway(Controller* controller, double* force) {
    if (norm(force, 2) > controller->runaway_force_threshold) return force; 
    double* zeros = malloc(2 * sizeof(double)); 
    initialize_to_zeros(zeros, 2); 
    return zeros;
}

double* wander(Controller* controller) {
    double* wander_heading = malloc(2 * sizeof(double)); 
    double* random_direction = malloc(2 * sizeof(double)); 

    initialize_to_zeros(wander_heading, 2); 
    initialize_to_zeros(random_direction, 2); 

    for (int j=0; j < 2; ++j) random_direction[j] = (((double)rand()/(double)RAND_MAX) * 2.0) - 1.0; 

    for (int i=0; i < 2; ++i) {
        wander_heading[i] = controller->previous_wander[i] + random_direction[i]; 
    }

    double wander_heading_norm = norm(wander_heading, 2); 

    for (int i=0; i < 2; ++i) wander_heading[i] = wander_heading[i] / wander_heading_norm; 
    free(random_direction); 
    return wander_heading; 
}

double* avoid(Controller* controller, double* force, double* heading) {
    double* force_plus_heading = malloc(2 * sizeof(double)); 
    initialize_to_zeros(force_plus_heading, 2); 

    for (int i=0; i < 2; ++i) force_plus_heading[i] = force[i] + heading[i]; 
    if (norm(force_plus_heading, 2) > controller->significant_force_threshold) return force_plus_heading; 

    double* zeros = malloc(2 * sizeof(double)); 
    initialize_to_zeros(zeros, 2); 
    return zeros;
}
void reset(Controller* controller) {
    controller->previous_heading[0] = 0.0; 
    controller->previous_heading[1] = 0.0; 
}

double* call(Controller* controller, double* distances, double time) {
    // get raw repulsive force (sum over sensors)
    double* avoid_force = feel_force(controller, distances); 
    double* wander_force; 

    // generate new wander force (normalized) every wander period 
    if ((time - controller->previous_wander_time) >= controller->wander_period) {
        wander_force = wander(controller); 
        controller->previous_wander_time = time; 
    } else {
        wander_force = malloc(2 * sizeof(double)); 
        initialize_to_zeros(wander_force, 2); 
    }

    // combine wander and avoid forces 
    double* desired_velocity = avoid(controller, avoid_force, wander_force); 

    free(wander_force); 

    controller->previous_heading[0] = desired_velocity[0]; 
    controller->previous_heading[1] = desired_velocity[1]; 
    controller->previous_time = time; 
    free(avoid_force); 
    return desired_velocity; 
}
