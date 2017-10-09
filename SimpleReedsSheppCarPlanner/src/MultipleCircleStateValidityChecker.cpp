#include "MultipleCircleStateValidityChecker.h"

bool MultipleCircleStateValidityChecker::isValid(const ob::State *state) const {
  const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
  float x, y, theta, value;
  for (int i = 0; i < numCoords; i++) {
    theta = (float)s->getYaw();
    x = xCoords[i]*cos(theta) - yCoords[i]*sin(theta);
    y = xCoords[i]*sin(theta) + yCoords[i]*cos(theta);
    x += (float)s->getX();
    y += (float)s->getY();
    value = gridmap.computeClearance(x, y, radius);
    if (value < radius) return false;
  }
  return true;
}
