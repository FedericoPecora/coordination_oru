#ifndef MultipleCircleStateValidityChecker_H
#define MultipleCircleStateValidityChecker_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MultipleCircleStateValidityChecker : public ob::StateValidityChecker {
 public:
  double* occupancyMap;
  double mapResolution;
  int mapWidth;
  int mapHeight;
  float radius;
  double* xCoords;
  double* yCoords;
  int numCoords;
  bool noMap;
  //Values less than this are occupied
  double occupiedThreshold;
  
  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si, double* _occupancyMap, int _mapWidth, int _mapHeight, double _occupiedThreshold, double _mapResolution, double _radius, double* _xCoords, double* _yCoords, int _numCoords) : ob::StateValidityChecker(si) {
    noMap = false;
    radius = (float)_radius;
    xCoords = _xCoords;
    yCoords = _yCoords;
    numCoords = _numCoords;
    occupancyMap = _occupancyMap;
    mapResolution = _mapResolution;
    mapWidth = _mapWidth;
    mapHeight = _mapHeight;
    occupiedThreshold = _occupiedThreshold;
  }
  
  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {
    noMap = true;
    std::cout << "(Using empty map for validity checking)" << std::endl;
  }

  virtual bool isValid(const ob::State *state) const;  

};

#endif
