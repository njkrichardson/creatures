#include <Arduino.h> 

#include "controller.h" 

void setup(); 
void loop(); 

void setup() {
  // construct controller
  Controller controller; 
}

void loop() {
  // collect distances 

  // execute controller 

  // apply motor control 
}

int main() {
  init(); 
  setup(); 
  while (true) { 
    loop();
  }
  return 0;
}