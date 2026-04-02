// File: my_controller.cpp
// Description: e-puck controller with selectable algorithms by robot name

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>

using namespace webots;

enum Algorithm {
  RANDOM_MOUSE,
  BRUTEFORCE,
  RIGHT_HAND_RULE
};

enum Action {
  ACT_FORWARD,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

struct SensorsState {
  bool front;
  bool left;
  bool right;
};

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
const int FORWARD_HOLD = 4;
const int TURN_HOLD = 7;
const int BACK_HOLD = 10;

Algorithm selectAlgorithmByName(const std::string &robotName);

void initAll();
void Forward();
void Stop();
void Left();
void Right();
void Back();

SensorsState readSensors();
bool continueCurrentAction();
void applyAction(Action action);

void runRandomMouse();
void runBruteforce();
void runRightHandRule();

Algorithm currentAlgorithm = RIGHT_HAND_RULE;
Action currentAction = ACT_FORWARD;
int actionStepsLeft = 0;

std::string algorithmName(Algorithm a) {
  switch (a) {
    case RANDOM_MOUSE: return "RANDOM_MOUSE";
    case BRUTEFORCE: return "BRUTEFORCE";
    case RIGHT_HAND_RULE: return "RIGHT_HAND_RULE";
    default: return "UNKNOWN";
  }
}

Algorithm selectAlgorithmByName(const std::string &robotName) {
  if (robotName == "epuck_random") return RANDOM_MOUSE;
  if (robotName == "epuck_bruteforce") return BRUTEFORCE;
  if (robotName == "epuck_wall") return RIGHT_HAND_RULE;

  // Пока Tremaux и AI не реализованы, оставляем рабочую заглушку.
  if (robotName == "epuck_researcher") return RIGHT_HAND_RULE;
  if (robotName == "epuck_ai") return RIGHT_HAND_RULE;

  return RIGHT_HAND_RULE;
}

SensorsState readSensors() {
  SensorsState s;
  s.front = (ps0->getValue() > THRESHOLD || ps7->getValue() > THRESHOLD);
  s.left  = (ps5->getValue() > THRESHOLD || ps6->getValue() > THRESHOLD);
  s.right = (ps1->getValue() > THRESHOLD || ps2->getValue() > THRESHOLD);
  return s;
}

bool continueCurrentAction() {
  if (actionStepsLeft <= 0)
    return false;

  actionStepsLeft--;

  switch (currentAction) {
    case ACT_FORWARD: Forward(); break;
    case ACT_LEFT:    Left();    break;
    case ACT_RIGHT:   Right();   break;
    case ACT_BACK:    Back();    break;
  }

  return true;
}

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

// ===========================
// RANDOM MOUSE
// ===========================
// Случайное поведение с небольшим "удержанием" направления,
// чтобы робот не дёргался слишком часто.
void runRandomMouse() {
  SensorsState s = readSensors();

  std::cout << "[RANDOM] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  if (continueCurrentAction())
    return;

  // Тупик
  if (s.front && s.left && s.right) {
    applyAction(ACT_BACK);
    return;
  }

  Action options[3];
  int count = 0;

  if (!s.front) options[count++] = ACT_FORWARD;
  if (!s.left)  options[count++] = ACT_LEFT;
  if (!s.right) options[count++] = ACT_RIGHT;

  if (count == 0) {
    applyAction(ACT_BACK);
    return;
  }

  // Небольшой приоритет вперёд, чтобы движение было менее "дёрганым".
  // Если вперёд свободно, чаще продолжаем ехать туда.
  if (!s.front && (std::rand() % 100 < 60)) {
    applyAction(ACT_FORWARD);
    return;
  }

  int choiceIndex = std::rand() % count;
  applyAction(options[choiceIndex]);
}

// ===========================
// BRUTEFORCE
// ===========================
// Практический "полный перебор" для локального управления:
// робот систематически проверяет направления в фиксированном порядке.
// Порядок здесь: forward -> left -> right -> back.
// Это проще и более предсказуемо, чем random, но всё ещё не очень эффективно.
void runBruteforce() {
  SensorsState s = readSensors();

  std::cout << "[BRUTE] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  if (continueCurrentAction())
    return;

  if (s.front && s.left && s.right) {
    applyAction(ACT_BACK);
    return;
  }

  // Систематический порядок проверки
  if (!s.front) {
    applyAction(ACT_FORWARD);
  } else if (!s.left) {
    applyAction(ACT_LEFT);
  } else if (!s.right) {
    applyAction(ACT_RIGHT);
  } else {
    applyAction(ACT_BACK);
  }
}

// ===========================
// RIGHT HAND RULE
// ===========================
void runRightHandRule() {
  SensorsState s = readSensors();

  std::cout << "[RIGHT] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  if (continueCurrentAction())
    return;

  // Right hand priority:
  // 1) right
  // 2) forward
  // 3) left
  // 4) back
  if (!s.right) {
    applyAction(ACT_RIGHT);
  } else if (!s.front) {
    applyAction(ACT_FORWARD);
  } else if (!s.left) {
    applyAction(ACT_LEFT);
  } else {
    applyAction(ACT_BACK);
  }
}

int main() {
  initAll();
  std::srand((unsigned)std::time(nullptr));

  std::string robotName = robot->getName();
  currentAlgorithm = selectAlgorithmByName(robotName);

  std::cout << "Robot name: " << robotName << std::endl;
  std::cout << "Selected algorithm: " << algorithmName(currentAlgorithm) << std::endl;

  if (robotName == "epuck_researcher" || robotName == "epuck_ai") {
    std::cout << "Warning: this robot name is mapped to RIGHT_HAND_RULE for now." << std::endl;
  }

  while (robot->step(timeStep) != -1) {
    switch (currentAlgorithm) {
      case RANDOM_MOUSE:
        runRandomMouse();
        break;
      case BRUTEFORCE:
        runBruteforce();
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