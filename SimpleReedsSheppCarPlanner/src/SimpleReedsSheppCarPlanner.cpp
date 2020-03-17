#include "SimpleStateValidityChecker.h"

using namespace mrpt::maps;
using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

typedef struct PathPose {
  double x;
  double y;
  double theta;
} PathPose;


extern "C" void cleanupPath(PathPose* path) {
  std::cout << "Cleaning up memory.." << std::endl;
  free(path);
}

extern "C" bool plan(const char* mapFilename, double mapResolution, double robotRadius, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs) {

  double pLen = 0.0;
  int numInterpolationPoints = 0;
  ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));
  
  COccupancyGridMap2D gridmap;
  gridmap.loadFromBitmapFile( mapFilename, (float)mapResolution, 0.0f, 0.0f );
  std::cout << "Loaded map (1) " << mapFilename << std::endl;
  
  ob::ScopedState<> start(space), goal(space);
  ob::RealVectorBounds bounds(2);
  bounds.low[0] = gridmap.getXMin();
  bounds.low[1] = gridmap.getYMin();
  bounds.high[0] = gridmap.getXMax();
  bounds.high[1] = gridmap.getYMax();
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1] << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]" << std::endl;
  
  // define a simple setup class
  og::SimpleSetup ss(space);
  
  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss.getSpaceInformation());
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new SimpleStateValidityChecker(si, mapFilename, (float)mapResolution, (float)robotRadius)));
  
  //Return false if the goal is occupied.
  ompl::base::State *statePtr = space->allocState();
  statePtr->as<ompl::base::SE2StateSpace::StateType>()->setX(goalX);
  statePtr->as<ompl::base::SE2StateSpace::StateType>()->setY(goalY);
  statePtr->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goalTheta);
  bool isGoalValid = si->getStateValidityChecker()->isValid(statePtr);
  space->freeState(statePtr);
  if (!isGoalValid) {
	  std::cout << "Invalid goal." << std::endl;
	  return false;
  }

  // set the start and goal states
  start[0] = startX;
  start[1] = startY;
  start[2] = startTheta;
  goal[0] = goalX;
  goal[1] = goalY;
  goal[2] = goalTheta;
  ss.setStartAndGoalStates(start, goal);
  
  // this call is optional, but we put it in to get more output information
  ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
  ss.setup();
  ss.print();
  
  // attempt to solve the problem within planningTimeInSecs seconds of planning time
  ob::PlannerStatus solved = ss.solve(planningTimeInSecs);
  
  if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION) {
    std::cout << "Found solution:" << std::endl;
    ss.simplifySolution();
    og::PathGeometric pth = ss.getSolutionPath();
    pLen = pth.length();
    numInterpolationPoints = ((double)pLen)/distanceBetweenPathPoints;
    if (numInterpolationPoints > 0) pth.interpolate(numInterpolationPoints);
    
    std::vector<ob::State*> states = pth.getStates();
    std::vector<double> reals;
    
    *pathLength = states.size();
    *path = (PathPose*)malloc(sizeof(PathPose) * states.size());
    memset(*path, 0, sizeof(PathPose) * states.size());
    
    for (unsigned i=0; i < states.size(); i++) {
      space->copyToReals(reals, states[i]);
      (*path)[i].x = reals[0];
      (*path)[i].y = reals[1];
      (*path)[i].theta = reals[2];
    }
    return true;
  }
  std::cout << "No solution found" << std::endl;
  return false;
}
