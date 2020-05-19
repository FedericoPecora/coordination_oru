#include "MultipleCircleStateValidityChecker.h"

bool MultipleCircleStateValidityChecker::isValid(const ob::State *state) const {

  if (noMap) {
    //std::cout << "No map, so no collision! " << std::endl;
    return true;
  }

  const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
  float x, y, theta, value;
  for (int i = 0; i < numCoords; i++) {
    theta = (float)s->getYaw();
    x = xCoords[i]*cos(theta) - yCoords[i]*sin(theta);
    y = xCoords[i]*sin(theta) + yCoords[i]*cos(theta);
    x += (float)s->getX();
    y += (float)s->getY();
    //value = gridmap.computeClearance(x, y, radius);

    int xMap = (int)x/mapResolution;
    int yMap = (int)y/mapResolution;
    int radiusMap = (int)radius/mapResolution;
    //std::cout << "map coords are (x,y,r) " << xMap << "," << yMap << "," << radiusMap << std::endl;

    int threshold = 130;
    for (int dx = 0; dx <= radiusMap; dx++) {
      for (int dy = 0; dy <= radiusMap; dy++) {
	
	if (xMap-dx < 0) return false;
	if (yMap-dy < 0) return false;
	if (xMap+dx > mapWidth/mapResolution) return false;
	if (yMap+dy > mapHeight/mapResolution) return false;
	
	if (map[xMap+dx][yMap+dy] < threshold) return false;
	if (map[xMap-dx][yMap+dy] < threshold) return false;
	if (map[xMap+dx][yMap-dy] < threshold) return false;
	if (map[xMap-dx][yMap-dy] < threshold) return false;
      }
    }
  }

  //std::cout << "Checked, no collision! " << std::endl;
  
  return true;
}
