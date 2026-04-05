// File: my_controller.cpp
// Description: e-puck controller with selectable algorithms

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace webots;

// Перечисление алгоритмов
enum Algorithm {
  RANDOM_MOUSE,
  BRUTEFORCE,
  RIGHT_HAND_RULE,
  TREMAUX
};

// Выбор алгоритма определяется здесь, расскомментируйте нужную строку
// Algorithm currentAlgorithm = RANDOM_MOUSE;
// Algorithm currentAlgorithm = RIGHT_HAND_RULE;
// Algorithm currentAlgorithm = BRUTEFORCE;
Algorithm currentAlgorithm = TREMAUX;

// Перечисление действий робота
enum Action {
  ACT_FORWARD,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

enum Heading { 
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};

// Состояние датчиков (имеется ли препятствие с какой-то стороны)
struct SensorsState {
  bool front;
  bool left;
  bool right;
};

// Структура для описания узлов лабиринта
struct Node {
  int marks[4]; // 0: вперед, 1: направо, 2: назад, 3: налево (относительно входа)
  Node() { 
    for (int i = 0; i < 4; i++) marks[i] = 0; 
  }
};

// Создание Webots объектов
Robot *robot = new Robot();
int timeStep = (int)robot->getBasicTimeStep();

Motor *left_motor = robot->getMotor("left wheel motor");
Motor *right_motor = robot->getMotor("right wheel motor");

// Создание датчиков
DistanceSensor *ps0 = robot->getDistanceSensor("ps0");
DistanceSensor *ps1 = robot->getDistanceSensor("ps1");
DistanceSensor *ps2 = robot->getDistanceSensor("ps2");
DistanceSensor *ps3 = robot->getDistanceSensor("ps3");
DistanceSensor *ps4 = robot->getDistanceSensor("ps4");
DistanceSensor *ps5 = robot->getDistanceSensor("ps5");
DistanceSensor *ps6 = robot->getDistanceSensor("ps6");
DistanceSensor *ps7 = robot->getDistanceSensor("ps7");

// Константы
const double THRESHOLD = 90.0;  // Порог для активации датчиков

// Скорости ниже maxVelocity (= 6.28), адаптированные под работу алгоритмов
const double RANDOM_SPEED = 6.2;
const double BRUTE_SPEED = 5.9;
const double WALL_SPEED = 4.4;
const double TREMAUX_SPEED = 4.7;  // Используемая скорость для Тремо

// Удержание действий
const int RANDOM_FORWARD_HOLD = 8;
const int RANDOM_TURN_HOLD = 6;
const int RANDOM_BACK_HOLD = 10;

const int BRUTE_FORWARD_HOLD = 6;
const int BRUTE_TURN_HOLD = 9;
const int BRUTE_BACK_HOLD = 12;

const int WALL_FORWARD_HOLD = 5;
const int WALL_TURN_HOLD = 7;
const int WALL_BACK_HOLD = 10;

// Константы конкретно под Тремо
const double DIST_THRESHOLD = 120.0; // Порог обнаружения стены
const int TIME_STEP = 32;

// Если робот недоворачивает — увеличь TURN_90_STEPS, если перелетает — уменьши.
const int TURN_90_STEPS = 8;   // Шагов для поворота на 90 градусов
const int CELL_STEPS = 24;      // Шагов для проезда одной клетки (узла)

// Текущее действие
Action currentAction = ACT_FORWARD;
int actionStepsLeft = 0;
double currentLeftSpeed = 0.0;
double currentRightSpeed = 0.0;

// Память для bruteforce
int bruteforcePhase = 0;
int currentNodeId = 0;
int stepCounter = 0;

// Состояние мира для Тремо
int currentX = 0, currentY = 0;
Heading currentHeading = NORTH;
std::map<std::pair<int, int>, Node> mazeMap;

// Прототипы
void initAll();
void setMotors(double leftSpeed, double rightSpeed);
void Forward(double speed);
void Left(double speed);
void Right(double speed);
void Back(double speed);
void Stop();
void Vel_stop();

SensorsState readSensors();
bool continueCurrentAction();
void applyAction(Action action, int holdSteps, double leftSpeed, double rightSpeed);

bool frontBlocked();
bool leftBlocked();
bool rightBlocked();

// Логика выбора алгоритма
const char *algorithmName(Algorithm a);
Action chooseBruteforceAction(const SensorsState &s);
Action chooseTremauxAction(const SensorsState &s);

void runRandomMouse();
void runBruteforce();
void runRightHandRule();
void runTremaux();

void moveTremaux(Action a);
int getNextNodeId();

// Логика выбора алгоритма в зависимости от заданного значения
const char *algorithmName(Algorithm a) {
  switch (a) {
    case RANDOM_MOUSE: return "RANDOM_MOUSE";
    case BRUTEFORCE: return "BRUTEFORCE";
    case RIGHT_HAND_RULE: return "RIGHT_HAND_RULE";
    case TREMAUX: return "TREMAUX";
    default: return "UNKNOWN";
  }
}

// Запись скорости на моторы
void setMotors(double leftSpeed, double rightSpeed) {
  left_motor->setVelocity(leftSpeed);
  right_motor->setVelocity(rightSpeed);
}

// Логика движений
void Forward(double speed) { setMotors(speed, speed); }
void Left(double speed)    { setMotors(-speed, speed); }
void Right(double speed)   { setMotors(speed, -speed); }
void Back(double speed)    { setMotors(-speed, -speed); }
void Stop()                { setMotors(0.0, 0.0); }
void Vel_stop() {
  left_motor->setVelocity(0);
  right_motor->setVelocity(0);
}

// Универсальная функция шага - позволяет двигшаться прямолинейно
void execute(double leftS, double rightS, int steps) {
    left_motor->setVelocity(leftS);
    right_motor->setVelocity(rightS);
    for (int i = 0; i < steps; i++) {
        if (robot->step(TIME_STEP) == -1) exit(0);
    }
    Vel_stop();
    // Короткая пауза (1 шаг), чтобы погасить инерцию перед следующим действием
    robot->step(TIME_STEP); 
}

// Функции поворотов под Tremaux
void turn90(bool right) {
    if (right) {
        execute(TREMAUX_SPEED, -TREMAUX_SPEED, TURN_90_STEPS);
        currentHeading = static_cast<Heading>((currentHeading + 1) % 4);
    } else {
        execute(-TREMAUX_SPEED, TREMAUX_SPEED, TURN_90_STEPS);
        currentHeading = static_cast<Heading>((currentHeading + 3) % 4);
    }
}

void turn180() {
    // Используем TURN_90_STEPS * 2 для разворота на месте
    execute(TREMAUX_SPEED, -TREMAUX_SPEED, TURN_90_STEPS * 2);
    currentHeading = static_cast<Heading>((currentHeading + 2) % 4);
}


// Чтение датчиков
SensorsState readSensors() {
  SensorsState s;
  s.front = (ps0->getValue() > THRESHOLD || ps7->getValue() > THRESHOLD);
  s.left = (ps5->getValue() > THRESHOLD || ps6->getValue() > THRESHOLD);
  s.right = (ps1->getValue() > THRESHOLD || ps2->getValue() > THRESHOLD);
  return s;
}

// Проверка препятствий
bool frontBlocked() {
  return (ps0->getValue() > THRESHOLD || ps7->getValue() > THRESHOLD);
}

bool leftBlocked() {
  return (ps5->getValue() > THRESHOLD || ps6->getValue() > THRESHOLD);
}

bool rightBlocked() {
  return (ps1->getValue() > THRESHOLD || ps2->getValue() > THRESHOLD);
}

// Выполнить текущее действие еще один шаг
bool continueCurrentAction() {
  if (actionStepsLeft <= 0)
    return false;

  actionStepsLeft--;
  setMotors(currentLeftSpeed, currentRightSpeed);
  return true;
}

// Назначить действие и задать длительность.
void applyAction(Action action, int holdSteps, double leftSpeed, double rightSpeed) {
  currentAction = action;
  actionStepsLeft = holdSteps;
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  setMotors(leftSpeed, rightSpeed);
}

// RANDOM MOUSE
void runRandomMouse() {
  SensorsState s = readSensors();
  
  std::cout << "[RANDOM] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  if (continueCurrentAction())
    return;

  if (s.front && s.left && s.right) {
    applyAction(ACT_BACK, RANDOM_BACK_HOLD, -RANDOM_SPEED, -RANDOM_SPEED);
    return;
  }

  Action options[3];
  int count = 0;

  if (!s.front) options[count++] = ACT_FORWARD;
  if (!s.left)  options[count++] = ACT_LEFT;
  if (!s.right) options[count++] = ACT_RIGHT;

  if (count == 0) {
    applyAction(ACT_BACK, RANDOM_BACK_HOLD, -RANDOM_SPEED, -RANDOM_SPEED);
    return;
  }

  if (!s.front && (std::rand() % 100 < 70)) {
    applyAction(ACT_FORWARD, RANDOM_FORWARD_HOLD + 2, RANDOM_SPEED, RANDOM_SPEED);
    return;
  }

  int choiceIndex = std::rand() % count;
  Action chosen = options[choiceIndex];

  if (chosen == ACT_FORWARD)
    applyAction(ACT_FORWARD, RANDOM_FORWARD_HOLD, RANDOM_SPEED, RANDOM_SPEED);
  else if (chosen == ACT_LEFT)
    applyAction(ACT_LEFT, RANDOM_TURN_HOLD, -RANDOM_SPEED, RANDOM_SPEED);
  else if (chosen == ACT_RIGHT)
    applyAction(ACT_RIGHT, RANDOM_TURN_HOLD, RANDOM_SPEED, -RANDOM_SPEED);
  else
    applyAction(ACT_BACK, RANDOM_BACK_HOLD, -RANDOM_SPEED, -RANDOM_SPEED);
}

// BRUTEFORCE
Action chooseBruteforceAction(const SensorsState &s) {
  Action order0[4] = {ACT_FORWARD, ACT_LEFT, ACT_RIGHT, ACT_BACK};
  Action order1[4] = {ACT_LEFT, ACT_RIGHT, ACT_FORWARD, ACT_BACK};
  Action order2[4] = {ACT_RIGHT, ACT_FORWARD, ACT_LEFT, ACT_BACK};

  Action *order = order0;
  if (bruteforcePhase % 3 == 1)
    order = order1;
  else if (bruteforcePhase % 3 == 2)
    order = order2;

  bruteforcePhase++;

  for (int i = 0; i < 4; i++) {
    Action a = order[i];

    if (a == ACT_FORWARD && !s.front) return ACT_FORWARD;
    if (a == ACT_LEFT && !s.left) return ACT_LEFT;
    if (a == ACT_RIGHT && !s.right) return ACT_RIGHT;

    if (a == ACT_BACK && s.front && s.left && s.right) return ACT_BACK;
  }

  if (!s.front) return ACT_FORWARD;
  if (!s.left) return ACT_LEFT;
  if (!s.right) return ACT_RIGHT;

  return ACT_BACK;
}

// Запуск метода брутфорс
void runBruteforce() {
  SensorsState s = readSensors();
  
  std::cout << "[BRUTE] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  if (continueCurrentAction())
    return;

  if (s.front && s.left && s.right) {
    applyAction(ACT_BACK, BRUTE_BACK_HOLD, -BRUTE_SPEED, -BRUTE_SPEED);
    return;
  }

  Action chosen = chooseBruteforceAction(s);

  if (chosen == ACT_FORWARD) {
    applyAction(ACT_FORWARD, BRUTE_FORWARD_HOLD, BRUTE_SPEED, BRUTE_SPEED);
  } else if (chosen == ACT_LEFT) {
    applyAction(ACT_LEFT, BRUTE_TURN_HOLD, -BRUTE_SPEED, BRUTE_SPEED);
  } else if (chosen == ACT_RIGHT) {
    applyAction(ACT_RIGHT, BRUTE_TURN_HOLD, BRUTE_SPEED, -BRUTE_SPEED);
  } else {
    applyAction(ACT_BACK, BRUTE_BACK_HOLD, -BRUTE_SPEED, -BRUTE_SPEED);
  }
}

// RIGHT HAND RULE
void runRightHandRule() {
  SensorsState s = readSensors();
  
  std::cout << "[RIGHT] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  if (continueCurrentAction())
    return;

  if (!s.right) {
    applyAction(ACT_RIGHT, 7, WALL_SPEED, -WALL_SPEED);
  } else if (!s.front) {
    applyAction(ACT_FORWARD, 4, WALL_SPEED, WALL_SPEED);
  } else if (!s.left) {
    applyAction(ACT_LEFT, 7, -WALL_SPEED, WALL_SPEED);
  } else {
    applyAction(ACT_BACK, 10, -WALL_SPEED, -WALL_SPEED);
  }
}

void runTremaux() {
    SensorsState s = readSensors();
    
    // Считаем точкой принятия решения любую развилку или тупик
    bool isJunction = (!s.left || !s.right || s.front);

    if (isJunction) {
          Vel_stop();
        std::pair<int, int> pos = {currentX, currentY};
        Node &node = mazeMap[pos];

        // 1. Отмечаем направление, откуда пришли
        int backDir = (currentHeading + 2) % 4;
        node.marks[backDir]++;

        // 2. Ищем лучший путь (минимальное кол-во меток)
        // 0-Forward, 1-Right, 2-Back, 3-Left
        bool canGo[4] = {!s.front, !s.right, true, !s.left};
        int bestRel = 2; 
        int minMarks = 999;

        for (int i = 0; i < 4; i++) {
            if (canGo[i]) {
                int absDir = (currentHeading + i) % 4;
                if (node.marks[absDir] < minMarks) {
                    minMarks = node.marks[absDir];
                    bestRel = i;
                }
            }
        }

        // 3. Выполняем маневр
        if (bestRel == 1) turn90(true);
        else if (bestRel == 3) turn90(false);
        else if (bestRel == 2) turn180();
        
        int chosenAbs = currentHeading;
        node.marks[chosenAbs]++;


        // Финальный проезд в выбранном направлении
        execute(TREMAUX_SPEED, TREMAUX_SPEED, CELL_STEPS);
        
        // Обновляем координаты в сетке
        if (currentHeading == NORTH) currentY++;
        else if (currentHeading == EAST) currentX++;
        else if (currentHeading == SOUTH) currentY--;
        else if (currentHeading == WEST) currentX--;

    } else {
        // Обычный коридор без развилок
        execute(TREMAUX_SPEED, TREMAUX_SPEED, 5); 
    }
}

int main() {
  initAll();
  std::srand((unsigned)std::time(nullptr));

  std::cout << "Controller started. Algorithm = " << algorithmName(currentAlgorithm) << std::endl;

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
      case TREMAUX:
        runTremaux();
        break;
    }
  }

  delete robot;
  return 0;
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

  Vel_stop();
}