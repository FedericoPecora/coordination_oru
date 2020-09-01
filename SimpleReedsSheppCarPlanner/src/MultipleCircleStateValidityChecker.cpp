#include "MultipleCircleStateValidityChecker.h"

bool MultipleCircleStateValidityChecker::isValid(const ob::State *state) const {

  const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
  float x, y, theta;

  float refPoseX = (float)s->getX();
  float refPoseY = (float)s->getY();
  float refPoseTheta = (float)s->getYaw();
  //std::cout << "Checking for collision in " << refPoseX << "," << refPoseY << "," << refPoseTheta << std::endl;

  if (noMap) {
    //std::cout << "No map, so no collision! " << std::endl;
    return true;
  }
  
  for (int i = 0; i < numCoords; i++) {
    x = xCoords[i]*cos(refPoseTheta) - yCoords[i]*sin(refPoseTheta);
    y = xCoords[i]*sin(refPoseTheta) + yCoords[i]*cos(refPoseTheta);
    x += refPoseX;
    y += refPoseY;

    int xPx = (int)((x-mapOriginX)/mapResolution);
    int yPx = mapHeight-(int)((y-mapOriginY)/mapResolution);
    int radiusPx = ceil(radius/mapResolution);

    /*
    std::cout << "(" << x << "," << y << ") -> (" << xPx << "," << yPx << ") with radius in pixels of " << radiusPx << std::endl;
    if (xPx >= 0 && yPx >= 0 && xPx < mapWidth && yPx < mapHeight) {
      std::cout << "Occupancy map value: " << (*(occupancyMap+yPx*mapWidth/8+xPx/8) & (1<<7)) << " (value < 0 occupied)" << std::endl;
    }
    */
      
    for (int dx = 0; dx <= radiusPx; dx++) {
      for (int dy = 0; dy <= radiusPx; dy++) {

	if (xPx-dx < 0) { return false; }
	if (yPx-dy < 0) { return false; }
	if (xPx+dx >= mapWidth) { return false; }
	if (yPx+dy >= mapHeight) { return false; }

	if (*(occupancyMap+(yPx+dy)*mapWidth/8+(xPx+dx)/8) & (1<<7)) { return false; }
	if (*(occupancyMap+(yPx+dy)*mapWidth/8+(xPx-dx)/8) & (1<<7)) { return false; }
	if (*(occupancyMap+(yPx-dy)*mapWidth/8+(xPx+dx)/8) & (1<<7)) { return false; }
	if (*(occupancyMap+(yPx-dy)*mapWidth/8+(xPx-dx)/8) & (1<<7)) { return false; }
	
	/*
	if ((int)occupancyMap[(yPx+dy)*mapWidth/8+(xPx+dx)/8] < 0) { return false; }
	if ((int)occupancyMap[(yPx+dy)*mapWidth/8+(xPx-dx)/8] < 0) { return false; }
	if ((int)occupancyMap[(yPx-dy)*mapWidth/8+(xPx+dx)/8] < 0) { return false; }
	if ((int)occupancyMap[(yPx-dy)*mapWidth/8+(xPx-dx)/8] < 0) { return false; }
	*/
      }
    }
  }

  //std::cout << "No collision found in " << refPoseX << "," << refPoseY << "," << refPoseTheta << std::endl;
  
  return true;
}
