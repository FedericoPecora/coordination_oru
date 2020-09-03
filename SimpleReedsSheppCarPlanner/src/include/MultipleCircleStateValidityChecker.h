#ifndef MultipleCircleStateValidityChecker_H
#define MultipleCircleStateValidityChecker_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <math.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MultipleCircleStateValidityChecker : public ob::StateValidityChecker {
 public:
  uint8_t* occupancyMap;
  double mapResolution;
  int mapWidth;
  int mapHeight;
  double mapOriginX;
  double mapOriginY;
  float radius;
  double* xCoords;
  double* yCoords;
  int numCoords;
  bool noMap;
  
  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si, uint8_t* _occupancyMap, int _mapWidth, int _mapHeight, double _mapResolution, double _mapOriginX, double _mapOriginY, double _radius, double* _xCoords, double* _yCoords, int _numCoords) : ob::StateValidityChecker(si) {
    noMap = false;
    radius = (float)_radius;
    xCoords = _xCoords;
    yCoords = _yCoords;
    numCoords = _numCoords;
    occupancyMap = _occupancyMap;
    mapResolution = _mapResolution;
    mapWidth = _mapWidth;
    mapHeight = _mapHeight;
    mapOriginX = _mapOriginX;
    mapOriginY = _mapOriginY;
  }
  
  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {
    noMap = true;
    std::cout << "(Using empty map for validity checking)" << std::endl;
  }

  virtual bool isValid(const ob::State *state) const;  

};

#endif
