#include <pybind11/pybind11.h> 
#include <pybind11/stl.h> 
#include "controller.h"

namespace cpython = pybind11; 

PYBIND11_MODULE(control_c, m) { 
    cpython::class_<Controller>(m, "Controller")
    .def(cpython::init<
        float, 
        float, 
        float, 
        float, 
        int
    >(),
        cpython::arg("collide_distance_threshold")=0.1, 
        cpython::arg("runaway_force_threshold")=0.1, 
        cpython::arg("significant_force_threshold")=0.0, 
        cpython::arg("avoid_supress_time")=0.5, 
        cpython::arg("num_sensors")=2
    )
    .def("_feel_force", &Controller::feel_force)
    .def("_collide", &Controller::collide)
    .def("_runaway", &Controller::runaway)
    .def("_wander", &Controller::wander)
    .def("_avoid", &Controller::avoid)
    .def("reset", &Controller::reset)
    .def("__call__", &Controller::call); 
    m.def("norm", &norm); 
}