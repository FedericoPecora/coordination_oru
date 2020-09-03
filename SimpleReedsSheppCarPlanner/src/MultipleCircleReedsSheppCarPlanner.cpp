#include "MultipleCircleStateValidityChecker.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
//#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <boost/math/constants/constants.hpp>

#include <stdio.h>

//using namespace mrpt::maps;
using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

enum PLANNING_ALGORITHM { RRTConnect, RRTstar, TRRT, SST, LBTRRT, PRMstar, SPARS, pRRT, LazyRRT };

typedef struct PathPose {
  double x;
  double y;
  double theta;
} PathPose;

extern "C" void cleanupPath(PathPose* path) {
  std::cout << "Cleaning up memory.." << std::endl;
  free(path);
}

extern "C" bool plan_multiple_circles(uint8_t* occupancyMap, int mapWidth, int mapHeight, double mapResolution, double mapOriginX, double mapOriginY, double robotRadius, double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs, PLANNING_ALGORITHM algo) {

  double pLen = 0.0;
  int numInterpolationPoints = 0;
  ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));

  std::cout << "Using " << mapWidth << "x" << mapHeight << " occupancy map for validity checking (values < 0 are not free)" << std::endl;
  
  ob::ScopedState<> start(space), goal(space);
  ob::RealVectorBounds bounds(2);
  bounds.low[0] = mapOriginX;
  bounds.low[1] = mapOriginY;
  bounds.high[0] = mapOriginX+mapWidth*mapResolution;
  bounds.high[1] = mapOriginY+mapHeight*mapResolution;
	  
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1] << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]" << std::endl;

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss.getSpaceInformation());
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new MultipleCircleStateValidityChecker(si, occupancyMap, mapWidth, mapHeight, mapResolution, mapOriginX, mapOriginY, robotRadius, xCoords, yCoords, numCoords)));
  
  //Return false if the start is occupied.  
  ompl::base::State *statePtrS = space->allocState();
  statePtrS->as<ompl::base::SE2StateSpace::StateType>()->setX(startX);
  statePtrS->as<ompl::base::SE2StateSpace::StateType>()->setY(startY);
  statePtrS->as<ompl::base::SE2StateSpace::StateType>()->setYaw(startTheta);
  std::cout << "Checking start pose (" << startX << "," << startY << "," << startTheta << ")" << std::endl;
  bool isStartValid = si->getStateValidityChecker()->isValid(statePtrS);
  space->freeState(statePtrS);
  if (!isStartValid) {
    std::cout << "Invalid start pose (" << startX << "," << startY << "," << startTheta << ") since pixel(s) around (" << (startX-mapOriginX)/mapResolution << "," << (mapHeight-(startY-mapOriginY)/mapResolution) << ") are occupied" << std::endl;
    return false;
  }
  

  //Return false if the goal is occupied.
  ompl::base::State *statePtrG = space->allocState();
  statePtrG->as<ompl::base::SE2StateSpace::StateType>()->setX(goalX);
  statePtrG->as<ompl::base::SE2StateSpace::StateType>()->setY(goalY);
  statePtrG->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goalTheta);
  std::cout << "Checking goal pose (" << goalX << "," << goalY << "," << goalTheta << ")" << std::endl;
  bool isGoalValid = si->getStateValidityChecker()->isValid(statePtrG);
  space->freeState(statePtrG);
  if (!isGoalValid) {
    std::cout << "Invalid goal pose (" << goalX << "," << goalY << "," << goalTheta << ") since pixel(s) around (" << (goalX-mapOriginX)/mapResolution << "," << (mapHeight-(goalY-mapOriginY)/mapResolution) << ") are occupied" << std::endl;
    return false;
  }
    
  if (algo == RRTConnect) {
    ob::PlannerPtr planner(new og::RRTConnect(si));
    ss.setPlanner(planner);
  }
  else if (algo == RRTstar) {
    ob::PlannerPtr planner(new og::RRTstar(si));
    ss.setPlanner(planner);
  }
  else if (algo == TRRT) {
    ob::PlannerPtr planner(new og::TRRT(si));
    ss.setPlanner(planner);
  }
  else if (algo == LBTRRT) {
    ob::PlannerPtr planner(new og::LBTRRT(si));
    ss.setPlanner(planner);
  }
  else if (algo == PRMstar) {
    ob::PlannerPtr planner(new og::PRMstar(si));
    ss.setPlanner(planner);
  }
  else if (algo == SPARS) {
    ob::PlannerPtr planner(new og::SPARS(si));
    ss.setPlanner(planner);
  }
  else if (algo == pRRT) {
    ob::PlannerPtr planner(new og::pRRT(si));
    ss.setPlanner(planner);
  }
  else if (algo == LazyRRT) {
    ob::PlannerPtr planner(new og::LazyRRT(si));
    ss.setPlanner(planner);
  }
  else {
    std::cout << "Invalid planner specified, aborting." << std::endl;
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
  std::cout << "Planning time is " << planningTimeInSecs << " secs." << std::endl;
  ob::PlannerStatus solved = ss.solve(planningTimeInSecs);

  if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION || solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION) {
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


extern "C" bool plan_multiple_circles_nomap(double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs, PLANNING_ALGORITHM algo) {

  double pLen = 0.0;
  int numInterpolationPoints = 0;
  ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));
  
  std::cout << "Using empty map" << std::endl;
  
  ob::ScopedState<> start(space), goal(space);
  ob::RealVectorBounds bounds(2);
  bounds.low[0] = -10000;
  bounds.low[1] = -10000;
  bounds.high[0] = 10000;
  bounds.high[1] = 10000;
  
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1] << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]" << std::endl;
  
  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss.getSpaceInformation());
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new MultipleCircleStateValidityChecker(si)));

  if (algo == RRTConnect) {
    ob::PlannerPtr planner(new og::RRTConnect(si));
    ss.setPlanner(planner);
  }
  else if (algo == RRTstar) {
    ob::PlannerPtr planner(new og::RRTstar(si));
    ss.setPlanner(planner);
  }
  else if (algo == TRRT) {
    ob::PlannerPtr planner(new og::TRRT(si));
    ss.setPlanner(planner);
  }
  else if (algo == LBTRRT) {
    ob::PlannerPtr planner(new og::LBTRRT(si));
    ss.setPlanner(planner);
  }
  else if (algo == PRMstar) {
    ob::PlannerPtr planner(new og::PRMstar(si));
    ss.setPlanner(planner);
  }
  else if (algo == SPARS) {
    ob::PlannerPtr planner(new og::SPARS(si));
    ss.setPlanner(planner);
  }
  else if (algo == pRRT) {
    ob::PlannerPtr planner(new og::pRRT(si));
    ss.setPlanner(planner);
  }
  else if (algo == LazyRRT) {
    ob::PlannerPtr planner(new og::LazyRRT(si));
    ss.setPlanner(planner);
  }
  else {
    std::cout << "Invalid planner specified, aborting." << std::endl;
    return false;
  }
  
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
  std::cout << "Planning time is " << planningTimeInSecs << " secs." << std::endl;
  ob::PlannerStatus solved = ss.solve(planningTimeInSecs);
  
  if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION || solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION) {
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
