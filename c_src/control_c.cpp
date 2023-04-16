#include <pybind11/pybind11.h> 
#include "controller.h"

namespace cpython = pybind11; 

PYBIND11_MODULE(control_c, m) { 
    cpython::class_<Controller>(m, "Controller")
    .def(cpython::init<>()); 
}