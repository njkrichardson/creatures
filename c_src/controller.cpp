#include <math.h> 
#include <iostream> 
#include <vector> 

#include "controller.h"

double norm(std::vector<double> arr) {
    double out = 0.0; 
    for (auto x : arr) out += x * x; 
    return sqrt(out); 
}

Controller::Controller(
        float collide_distance_threshold, 
        float runaway_force_threshold, 
        float significant_force_threshold, 
        float avoid_supress_time, 
        int num_sensors
) : 
    collide_distance_threshold(collide_distance_threshold), 
    runaway_force_threshold(runaway_force_threshold), 
    significant_force_threshold(significant_force_threshold), 
    avoid_supress_time(avoid_supress_time), 
    num_sensors(num_sensors)
{
    sonar_radian_offsets = {0, 90, 180, 270}; 
    sonar_basis_vectors = {{0, 1}}; 
    previous_wander = {0, 0}; 
}

Controller::~Controller() {}; 


std::vector<double> Controller::feel_force(std::vector<double> distances) { 
    size_t num_distances = distances.size(); 
    std::vector<std::vector<double>> directional_forces(sonar_basis_vectors.size(), std::vector<double>(2)); 
    std::vector<double> overall_force(2, 0); 
    std::vector<double> force_per_sensor(num_distances, 0); 

    for (int i=0; i < num_distances; ++i) force_per_sensor[i] = -1.0 / pow((10.0 * distances[i] + 0.001), 5); 
    for (int i=0; i < num_distances; ++i) {
        for (int j=0; j < 2; ++j) {
            overall_force[j] += sonar_basis_vectors[i][j] * force_per_sensor[i]; 
        }
    }
    return overall_force; 
}

bool Controller::collide(std::vector<double> distances) {
    // Note: assumes that the first index of distances holds the sensor with 0 offset 
    return distances[0] < collide_distance_threshold; 
}

std::vector<double> Controller::runaway(std::vector<double> force) {
    if (norm(force) > runaway_force_threshold) return force; 
    std::vector<double> zeros(2, 0); 
    return zeros; 
}

std::vector<double> Controller::wander() {
    size_t wander_size = previous_wander.size(); 
    std::vector<double> wander_heading(wander_size, 0); 

    std::vector<double> random_direction(2, 0); 
    for (int j=0; j < 2; ++j) random_direction[j] = (((double)rand()/(double)RAND_MAX) * 2.0) - 1.0; 

    for (int i=0; i < wander_size; ++i) {
        wander_heading[i] = previous_wander[i] + random_direction[i]; 
    }

    double wander_heading_norm = norm(wander_heading); 

    for (int i=0; i < wander_size; ++i) wander_heading[i] = wander_heading[i] / wander_heading_norm; 
    return wander_heading; 
}

std::vector<double> Controller::avoid(std::vector<double>, std::vector<double>) {}
void Controller::reset() {} 
std::vector<double> Controller::call(std::vector<double>) {} 