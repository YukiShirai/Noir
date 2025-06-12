#include "sensor.h"

#include <iostream>

using namespace noir;

void ForceSensor::read() const {
    std::cout << "[ForceSensor] Reading force data from " << m_name << std::endl;
}

void VisionSensor::read() const {
    std::cout << "[VisionSensor] Reading force data from " << m_name << std::endl;
}
