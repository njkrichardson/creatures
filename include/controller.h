#ifndef CONTROLLER_H
#define CONTROLLER_H 

#include <vector> 

double norm(std::vector<double> arr); 

struct Controller {
    // parameters 
    float collide_distance_threshold; 
    float runaway_force_threshold; 
    float significant_force_threshold; 
    float avoid_supress_time; 

    // behavioral state (historical)
    float previous_wander_time; 
    std::vector<double> previous_avoid_heading; 
    std::vector<double> previous_heading; 
    std::vector<double> previous_wander; 
    float previous_time; 

    // sensors
    int num_sensors; 
    std::vector<int> sonar_radian_offsets; 
    std::vector<std::vector<double>> sonar_basis_vectors; 

    // rule of three (two)
    Controller(
        float collide_distance_threshold=0.1, 
        float runaway_force_threshold=0.1, 
        float significant_force_threshold=0.0, 
        float avoid_supress_time=0.5, 
        int num_sensors=2
    ); 
    ~Controller(); 

    // private (from Python's point of view)
    std::vector<double> feel_force(std::vector<double>); 
    bool collide(std::vector<double>); 
    std::vector<double> runaway(std::vector<double>); 
    std::vector<double> wander(); 
    std::vector<double> avoid(std::vector<double>, std::vector<double>); 
    
    // API 
    public: 
        void reset(); 
        std::vector<double> call(std::vector<double>); 
}; 

#endif 