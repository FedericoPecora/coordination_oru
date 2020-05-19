#ifndef MultipleCircleStateValidityChecker_H
#define MultipleCircleStateValidityChecker_H

#include <png.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class MultipleCircleStateValidityChecker : public ob::StateValidityChecker {
 public:
  png_bytepp map;
  double mapResolution;
  float radius;
  double* xCoords;
  double* yCoords;
  int numCoords;
  bool noMap;
  int mapWidth;
  int mapHeight;
  
  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si, const png_bytepp _map, double _mapResolution, int _mapWidth, int _mapHeight, double _radius, double* _xCoords, double* _yCoords, int _numCoords) : ob::StateValidityChecker(si) {
    noMap = false;
    radius = (float)_radius;
    xCoords = _xCoords;
    yCoords = _yCoords;
    numCoords = _numCoords;
    map = _map;
    mapResolution = _mapResolution;
    mapWidth = _mapWidth;
    mapHeight = _mapHeight;
  }
  
  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {
    noMap = true;
    std::cout << "(Using empty map for validity checking)" << std::endl;
  }

  virtual bool isValid(const ob::State *state) const;  

};

#endif
