// File: my_controller.cpp
// Description: e-puck controller with selectable algorithms via enum

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace webots;

enum Algorithm {
  RANDOM_MOUSE,
  RIGHT_HAND_RULE
};

// --- Choose algorithm here ---
// Change this line before running the simulation:
//Algorithm currentAlgorithm = RANDOM_MOUSE;
Algorithm currentAlgorithm = RIGHT_HAND_RULE;

Robot *robot = new Robot();
int timeStep = (int)robot->getBasicTimeStep();

Motor *left_motor = robot->getMotor("left wheel motor");
Motor *right_motor = robot->getMotor("right wheel motor");

DistanceSensor *ps0 = robot->getDistanceSensor("ps0");
DistanceSensor *ps1 = robot->getDistanceSensor("ps1");
DistanceSensor *ps2 = robot->getDistanceSensor("ps2");
DistanceSensor *ps3 = robot->getDistanceSensor("ps3");
DistanceSensor *ps4 = robot->getDistanceSensor("ps4");
DistanceSensor *ps5 = robot->getDistanceSensor("ps5");
DistanceSensor *ps6 = robot->getDistanceSensor("ps6");
DistanceSensor *ps7 = robot->getDistanceSensor("ps7");

const double SPEED = 5.0;
const double THRESHOLD = 100.0;

// Small delays so the robot does not change direction every step.
const int FORWARD_HOLD = 2;
const int TURN_HOLD = 6;
const int BACK_HOLD = 8;

void initAll();

void Forward();
void Stop();
void Left();
void Right();
void Back();

bool frontBlocked();
bool leftBlocked();
bool rightBlocked();

void runRandomMouse();
void runRightHandRule();

void applyRandomAction();
void applyRightHandAction();

enum Action {
  ACT_FORWARD,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

Action currentAction = ACT_FORWARD;
int actionStepsLeft = 0;

void applyAction(Action action) {
  currentAction = action;

  switch (action) {
    case ACT_FORWARD:
      actionStepsLeft = FORWARD_HOLD;
      Forward();
      break;
    case ACT_LEFT:
      actionStepsLeft = TURN_HOLD;
      Left();
      break;
    case ACT_RIGHT:
      actionStepsLeft = TURN_HOLD;
      Right();
      break;
    case ACT_BACK:
      actionStepsLeft = BACK_HOLD;
      Back();
      break;
  }
}

bool frontBlocked() {
  return (ps0->getValue() > THRESHOLD || ps7->getValue() > THRESHOLD);
}

bool leftBlocked() {
  return (ps5->getValue() > THRESHOLD || ps6->getValue() > THRESHOLD);
}

bool rightBlocked() {
  return (ps1->getValue() > THRESHOLD || ps2->getValue() > THRESHOLD);
}

// ===========================
// RANDOM MOUSE
// ===========================
void runRandomMouse() {
  bool front = frontBlocked();
  bool left = leftBlocked();
  bool right = rightBlocked();

  std::cout << "[RANDOM] F:" << front << " L:" << left << " R:" << right << std::endl;

  // Continue previous action for a few steps
  if (actionStepsLeft > 0) {
    actionStepsLeft--;

    switch (currentAction) {
      case ACT_FORWARD: Forward(); break;
      case ACT_LEFT:    Left();    break;
      case ACT_RIGHT:   Right();   break;
      case ACT_BACK:    Back();    break;
    }
    return;
  }

  // Dead end
  if (front && left && right) {
    applyAction(ACT_BACK);
    return;
  }

  // Collect available directions
  Action options[3];
  int count = 0;

  if (!front) options[count++] = ACT_FORWARD;
  if (!left)  options[count++] = ACT_LEFT;
  if (!right) options[count++] = ACT_RIGHT;

  if (count == 0) {
    applyAction(ACT_BACK);
    return;
  }

  // Random choice among available options
  int choiceIndex = std::rand() % count;
  applyAction(options[choiceIndex]);
}

// ===========================
// RIGHT HAND RULE
// ===========================
void runRightHandRule() {
  bool front = frontBlocked();
  bool left = leftBlocked();
  bool right = rightBlocked();

  std::cout << "[RIGHT] F:" << front << " L:" << left << " R:" << right << std::endl;

  // Continue previous action for a few steps
  if (actionStepsLeft > 0) {
    actionStepsLeft--;

    switch (currentAction) {
      case ACT_FORWARD: Forward(); break;
      case ACT_LEFT:    Left();    break;
      case ACT_RIGHT:   Right();   break;
      case ACT_BACK:    Back();    break;
    }
    return;
  }

  // Right hand priority:
  // 1) turn right if possible
  // 2) go forward if possible
  // 3) turn left if possible
  // 4) go back if trapped
  if (!right) {
    applyAction(ACT_RIGHT);
  } else if (!front) {
    applyAction(ACT_FORWARD);
  } else if (!left) {
    applyAction(ACT_LEFT);
  } else {
    applyAction(ACT_BACK);
  }
}

int main() {
  initAll();
  std::srand((unsigned)std::time(nullptr));

  std::cout << "Controller started. Algorithm = "
            << (currentAlgorithm == RANDOM_MOUSE ? "RANDOM_MOUSE" : "RIGHT_HAND_RULE")
            << std::endl;

  while (robot->step(timeStep) != -1) {
    switch (currentAlgorithm) {
      case RANDOM_MOUSE:
        runRandomMouse();
        break;
      case RIGHT_HAND_RULE:
        runRightHandRule();
        break;
    }
  }

  delete robot;
  return 0;
}

void Left() {
  left_motor->setVelocity(-SPEED);
  right_motor->setVelocity(SPEED);
}

void Right() {
  left_motor->setVelocity(SPEED);
  right_motor->setVelocity(-SPEED);
}

void Back() {
  left_motor->setVelocity(-SPEED);
  right_motor->setVelocity(-SPEED);
}

void Stop() {
  left_motor->setVelocity(0);
  right_motor->setVelocity(0);
}

void Forward() {
  left_motor->setVelocity(SPEED);
  right_motor->setVelocity(SPEED);
}

void initAll() {
  ps0->enable(timeStep);
  ps1->enable(timeStep);
  ps2->enable(timeStep);
  ps3->enable(timeStep);
  ps4->enable(timeStep);
  ps5->enable(timeStep);
  ps6->enable(timeStep);
  ps7->enable(timeStep);

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

  left_motor->setVelocity(0);
  right_motor->setVelocity(0);
}