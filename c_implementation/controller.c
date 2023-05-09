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

    controller->previous_heading = malloc(2 * sizeof(double)); 
    controller->previous_heading[0] = 0.0; 
    controller->previous_heading[1] = 1.0; 

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
    printf("freed sonar_basis_vectors rows"); 
    free(controller->sonar_basis_vectors);
    printf("freed sonar_basis_vectors entirely"); 
    free(controller->sonar_radian_offsets); 
    printf("freed sonar_radian_offsets"); 
    free(controller->previous_wander); 
    printf("freed previous_wander"); 
    free(controller->previous_avoid_heading); 
    printf("freed previous_avoid_heading"); 
    free(controller->previous_heading); 
}

double* feel_force(Controller* controller, double* distances) { 
    /* Computes the (repulsive) force from the measured sensor distances. 

    Parameters 
    ----------
    double* distances 
        array of distance measurements (assumed to have length equal to this->num_sensors). 
    */
    size_t num_sensors = controller->num_sensors;

    double* overall_force = malloc(2 * sizeof(double)); 
    initialize_to_zeros(overall_force, 2); 
    double force_per_sensor[num_sensors]; 

    for (int i=0; i < num_sensors; ++i) force_per_sensor[i] = -0.001 / pow((distances[i] + 0.001), 5); 
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
    double* wander_force_normalized = malloc(2 * sizeof(double)); 
    double* wander_force = malloc(2 * sizeof(double)); 

    initialize_to_zeros(wander_force_normalized, 2); 
    initialize_to_zeros(wander_force, 2); 

    for (int j=0; j < 2; ++j) wander_force[j] = (((double)rand()/(double)RAND_MAX) * 2.0) - 1.0; 

    // for (int i=0; i < 2; ++i) {
    //     wander_heading[i] = controller->previous_wander[i] + random_direction[i]; 
    // }

    // double wander_heading_norm = norm(wander_heading, 2); 

    double wander_force_norm = norm(wander_force, 2); 

    for (int i=0; i < 2; ++i) wander_force_normalized[i] = wander_force[i] / wander_force_norm; 

    controller->previous_wander = wander_force_normalized; 

    free(wander_force); 
    return wander_force_normalized; 
}

double* avoid(Controller* controller, double* avoid_force, double* wander_force) {
    double* combined = malloc(2 * sizeof(double)); 
    initialize_to_zeros(combined, 2); 

    for (int i=0; i < 2; ++i) combined[i] = avoid_force[i] + wander_force[i]; 

    double combined_norm = norm(combined, 2); 

    if (combined_norm > controller->significant_force_threshold) {
        combined[0] = combined[0] / combined_norm; 
        combined[1] = combined[1] / combined_norm; 
        return combined; 
    }

    double* zeros = malloc(2 * sizeof(double)); 
    initialize_to_zeros(zeros, 2); 
    return zeros;
}
void reset(Controller* controller) {
    controller->previous_heading[0] = 0.0; 
    controller->previous_heading[1] = 0.0; 
}

void discretize(double* direction, size_t size) {
    /* Discretizes direction into directions with angle (against x-axis, in radians) of [0 * pi/4, 1 * pi / 4, 2 * pi /4, 3 * pi/4, ..., 7pi/4]*/
    normalize(direction, size); 

    // TODO don't recompute
    double pi = 3.1415926; 
    // double pi_over_4 = M_PI / 4.0; 
    // double pi_over_8 = M_PI / 8.0; 
    double pi_over_4 = pi / 4.0; 
    double pi_over_8 = pi / 8.0; 

    if ((direction[0] >= 0) && (direction[0] < pi_over_8)) direction[0] = 0.;
    else if ((direction[0] >= pi_over_8) && (direction[0] <= (1. - pi_over_8))) direction[0] = pi_over_4;
    else if (direction[0] >= (1. - pi_over_8)) direction[0] = 1.;

    if ((direction[0] < 0.) && (direction[0] > -pi_over_8)) direction[0] = 0.; 
    else if ((direction[0] < -pi_over_8) && (direction[0] > (-1. + pi_over_8))) direction[0] = -pi_over_4; 
    else if (direction[0] < (-1. + pi_over_8)) direction[0] = -1.; 

    if ((direction[1] >= 0) && (direction[1] < pi_over_8)) direction[1] = 0.; 
    else if ((direction[1] >= pi_over_8) && (direction[1] <= (1. - pi_over_8))) direction[1] = pi_over_4; 
    else if (direction[1] >= (1. - pi_over_8)) direction[1] = 1.; 

    if ((direction[1] < 0.) && (direction[1] > -pi_over_8)) direction[1] = 0.; 
    else if ((direction[1] < -pi_over_8) && (direction[1] > (-1. + pi_over_8))) direction[1] = -pi_over_4; 
    else if (direction[1] < (-1. + pi_over_8)) direction[1] = -1.; 
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
