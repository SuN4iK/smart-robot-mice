// File: my_controller.cpp
// Description: e-puck controller with selectable algorithms

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <cstdlib>
#include <ctime>
#include <iostream>

using namespace webots;

// Перечень алгоритмов.
enum Algorithm {
  RANDOM_MOUSE,
  BRUTEFORCE,
  RIGHT_HAND_RULE
};

// Выбор алгоритма меняется только здесь.
// Например:
//Algorithm currentAlgorithm = RANDOM_MOUSE;
//Algorithm currentAlgorithm = BRUTEFORCE;
Algorithm currentAlgorithm = RIGHT_HAND_RULE;

// Состояние робота и устройств Webots.
Robot *robot = new Robot();
int timeStep = (int)robot->getBasicTimeStep();

Motor *left_motor = robot->getMotor("left wheel motor");
Motor *right_motor = robot->getMotor("right wheel motor");

// Датчики приближения e-puck.
DistanceSensor *ps0 = robot->getDistanceSensor("ps0");
DistanceSensor *ps1 = robot->getDistanceSensor("ps1");
DistanceSensor *ps2 = robot->getDistanceSensor("ps2");
DistanceSensor *ps3 = robot->getDistanceSensor("ps3");
DistanceSensor *ps4 = robot->getDistanceSensor("ps4");
DistanceSensor *ps5 = robot->getDistanceSensor("ps5");
DistanceSensor *ps6 = robot->getDistanceSensor("ps6");
DistanceSensor *ps7 = robot->getDistanceSensor("ps7");

// Скорость движения.
const double SPEED = 5.0;

// Порог срабатывания датчиков.
const double THRESHOLD = 100.0;

// Сколько шагов робот продолжает текущее действие.
// Это нужно, чтобы робот не менял направление каждый такт симуляции.
const int FORWARD_HOLD = 4;
const int TURN_HOLD = 7;
const int BACK_HOLD = 10;

// Возможные действия.
enum Action {
  ACT_FORWARD,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

// Структура для состояния датчиков.
struct SensorsState {
  bool front;
  bool left;
  bool right;
};

// Текущее действие и число оставшихся шагов.
Action currentAction = ACT_FORWARD;
int actionStepsLeft = 0;

// Счетчик для систематического перебора направлений.
int bruteforcePhase = 0;

// Инициализация.
void initAll();

// Движение.
void Forward();
void Stop();
void Left();
void Right();
void Back();

// Считывание датчиков.
SensorsState readSensors();

// Выполнение текущего действия.
bool continueCurrentAction();

// Назначение действия с удержанием.
void applyAction(Action action, int holdSteps);

// Проверка препятствий.
bool frontBlocked();
bool leftBlocked();
bool rightBlocked();

// Реализация алгоритмов.
void runRandomMouse();
void runBruteforce();
void runRightHandRule();

// Выбор действия для bruteforce.
Action chooseBruteforceAction(const SensorsState &s);

// Возвращает строку с названием алгоритма.
const char *algorithmName(Algorithm a) {
  switch (a) {
    case RANDOM_MOUSE: return "RANDOM_MOUSE";
    case BRUTEFORCE: return "BRUTEFORCE";
    case RIGHT_HAND_RULE: return "RIGHT_HAND_RULE";
    default: return "UNKNOWN";
  }
}

// Считывание состояния датчиков.
SensorsState readSensors() {
  SensorsState s;
  s.front = frontBlocked();
  s.left = leftBlocked();
  s.right = rightBlocked();
  return s;
}

// Проверка переда.
bool frontBlocked() {
  return (ps0->getValue() > THRESHOLD || ps7->getValue() > THRESHOLD);
}

// Проверка левой стороны.
bool leftBlocked() {
  return (ps5->getValue() > THRESHOLD || ps6->getValue() > THRESHOLD);
}

// Проверка правой стороны.
bool rightBlocked() {
  return (ps1->getValue() > THRESHOLD || ps2->getValue() > THRESHOLD);
}

// Выполнить текущее действие еще один шаг.
bool continueCurrentAction() {
  if (actionStepsLeft <= 0)
    return false;

  actionStepsLeft--;

  switch (currentAction) {
    case ACT_FORWARD:
      Forward();
      break;
    case ACT_LEFT:
      Left();
      break;
    case ACT_RIGHT:
      Right();
      break;
    case ACT_BACK:
      Back();
      break;
  }

  return true;
}

// Назначить действие и задать длительность.
void applyAction(Action action, int holdSteps) {
  currentAction = action;
  actionStepsLeft = holdSteps;

  switch (action) {
    case ACT_FORWARD:
      Forward();
      break;
    case ACT_LEFT:
      Left();
      break;
    case ACT_RIGHT:
      Right();
      break;
    case ACT_BACK:
      Back();
      break;
  }
}

// Случайное поведение.
// Это базовый алгоритм: на развилке выбирает направление случайно.
// Чтобы робот не дергался слишком сильно, действие удерживается несколько шагов.
// Также есть небольшой приоритет движения вперед, если путь свободен.
void runRandomMouse() {
  SensorsState s = readSensors();

  std::cout << "[RANDOM] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  // Если робот уже выполняет действие, продолжаем его.
  if (continueCurrentAction())
    return;

  // Тупик.
  if (s.front && s.left && s.right) {
    applyAction(ACT_BACK, BACK_HOLD);
    return;
  }

  // Список доступных направлений.
  Action options[3];
  int count = 0;

  if (!s.front) options[count++] = ACT_FORWARD;
  if (!s.left)  options[count++] = ACT_LEFT;
  if (!s.right) options[count++] = ACT_RIGHT;

  // Если вдруг все закрыто, отъезжаем назад.
  if (count == 0) {
    applyAction(ACT_BACK, BACK_HOLD);
    return;
  }

  // Если впереди свободно, чаще продолжаем ехать вперед.
  // Это делает поведение менее рваным.
  if (!s.front && (std::rand() % 100 < 60)) {
    applyAction(ACT_FORWARD, FORWARD_HOLD + 2);
    return;
  }

  // Иначе выбираем случайное направление из доступных.
  int choiceIndex = std::rand() % count;
  Action chosen = options[choiceIndex];

  if (chosen == ACT_FORWARD)
    applyAction(ACT_FORWARD, FORWARD_HOLD);
  else if (chosen == ACT_LEFT)
    applyAction(ACT_LEFT, TURN_HOLD);
  else if (chosen == ACT_RIGHT)
    applyAction(ACT_RIGHT, TURN_HOLD);
  else
    applyAction(ACT_BACK, BACK_HOLD);
}

// Полный перебор.
// Это не "настоящий" полный перебор всего лабиринта с картой,
// а локальный систематический перебор направлений на развилке.
// Для проекта этого достаточно как отдельной демонстрационной версии.
// Логика: робот последовательно пробует варианты в фиксированном порядке,
// но порядок немного меняется по фазам, чтобы поиск не был слишком однообразным.
Action chooseBruteforceAction(const SensorsState &s) {
  // Три фазы перебора:
  // 0: вперед -> влево -> вправо -> назад
  // 1: влево -> вправо -> вперед -> назад
  // 2: вправо -> вперед -> влево -> назад
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

    // Назад используем только если других вариантов нет.
    if (a == ACT_BACK && s.front && s.left && s.right) return ACT_BACK;
  }

  // На всякий случай.
  if (!s.front) return ACT_FORWARD;
  if (!s.left) return ACT_LEFT;
  if (!s.right) return ACT_RIGHT;

  return ACT_BACK;
}

// Алгоритм полного перебора.
void runBruteforce() {
  SensorsState s = readSensors();

  std::cout << "[BRUTE] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  // Если уже выполняем действие, продолжаем его.
  if (continueCurrentAction())
    return;

  // Если тупик, уходим назад.
  if (s.front && s.left && s.right) {
    applyAction(ACT_BACK, BACK_HOLD);
    return;
  }

  // Выбираем направление систематически.
  Action chosen = chooseBruteforceAction(s);

  switch (chosen) {
    case ACT_FORWARD:
      applyAction(ACT_FORWARD, FORWARD_HOLD);
      break;
    case ACT_LEFT:
      applyAction(ACT_LEFT, TURN_HOLD);
      break;
    case ACT_RIGHT:
      applyAction(ACT_RIGHT, TURN_HOLD);
      break;
    case ACT_BACK:
      applyAction(ACT_BACK, BACK_HOLD);
      break;
  }
}

// Правило правой руки.
// Приоритет: вправо -> вперед -> влево -> назад.
// Эта версия уже рабочая и стабильная.
void runRightHandRule() {
  SensorsState s = readSensors();

  std::cout << "[RIGHT] F:" << s.front << " L:" << s.left << " R:" << s.right << std::endl;

  // Если робот уже выполняет действие, продолжаем его.
  if (continueCurrentAction())
    return;

  // Приоритет правой руки.
  if (!s.right) {
    applyAction(ACT_RIGHT, TURN_HOLD);
  } else if (!s.front) {
    applyAction(ACT_FORWARD, FORWARD_HOLD);
  } else if (!s.left) {
    applyAction(ACT_LEFT, TURN_HOLD);
  } else {
    applyAction(ACT_BACK, BACK_HOLD);
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
    }
  }

  delete robot;
  return 0;
}

// Движение влево.
void Left() {
  left_motor->setVelocity(-SPEED);
  right_motor->setVelocity(SPEED);
}

// Движение вправо.
void Right() {
  left_motor->setVelocity(SPEED);
  right_motor->setVelocity(-SPEED);
}

// Движение назад.
void Back() {
  left_motor->setVelocity(-SPEED);
  right_motor->setVelocity(-SPEED);
}

// Остановка.
void Stop() {
  left_motor->setVelocity(0);
  right_motor->setVelocity(0);
}

// Движение вперед.
void Forward() {
  left_motor->setVelocity(SPEED);
  right_motor->setVelocity(SPEED);
}

// Инициализация датчиков и моторов.
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