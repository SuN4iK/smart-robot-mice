// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

Robot *robot = new Robot();
int timeStep = (int)robot->getBasicTimeStep();
Motor *left_motor = robot->getMotor("left wheel motor");
Motor *right_motor = robot->getMotor("right wheel motor");
DistanceSensor *left_sensor = robot->getDistanceSensor ("gs0");
DistanceSensor *center_sensor = robot->getDistanceSensor ("gs1");
DistanceSensor *right_sensor = robot->getDistanceSensor ("gs2");
DistanceSensor *ps0 = robot->getDistanceSensor("ps0");
DistanceSensor *ps1 = robot->getDistanceSensor("ps1");
DistanceSensor *ps2 = robot->getDistanceSensor("ps2");
DistanceSensor *ps3 = robot->getDistanceSensor("ps3");
DistanceSensor *ps4 = robot->getDistanceSensor("ps4");
DistanceSensor *ps5 = robot->getDistanceSensor("ps5");
DistanceSensor *ps6 = robot->getDistanceSensor("ps6");
DistanceSensor *ps7 = robot->getDistanceSensor("ps7");

void initAll();
void Forward();
void Stop();
void Left();
void Right();

int main() {
  initAll();
  bool start = false;
  while (robot->step(timeStep) != -1) {
    bool stena = false;
    bool stenaSleva = false;
    bool stenaSprava = false;
    if(ps0->getValue() > 100 or ps7->getValue() > 100)
      stena = true;
    if(ps5->getValue() > 100 or ps6->getValue() > 100)
      stenaSleva = true;
    if(ps2->getValue() > 100 or ps1->getValue() > 100)
      stenaSprava = true;
    std::cout << stena << stenaSleva << stenaSprava << std::endl;

    if (!stena)
      Forward();
    else
      start = true;

    if (start) {
      if (stenaSprava == true) {
        if (stena == true) {
          Left();
        }
        else
          Forward();
      }
      else
        Right();
    }

  };
  delete robot;
  return 0;
}

void Left() {
    left_motor->setVelocity(-5);
    right_motor->setVelocity(5);
}
void Right() {
    left_motor->setVelocity(5);
    right_motor->setVelocity(-5);
}
void Stop() {
    left_motor->setVelocity(0);
    right_motor->setVelocity(0);
}
void Forward() {
    left_motor->setVelocity(5);
    right_motor->setVelocity(5);
}

void initAll() {
  left_sensor->enable(timeStep);
  right_sensor->enable(timeStep);
  ps0->enable(timeStep);
  ps1->enable(timeStep);
  ps2->enable(timeStep);
  ps3->enable(timeStep);
  ps4->enable(timeStep);
  ps5->enable(timeStep);
  ps6->enable(timeStep);
  ps7->enable(timeStep);
  center_sensor->enable(timeStep);
  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);
  left_motor->setVelocity(5);
  right_motor->setVelocity(5);
}
