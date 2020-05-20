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

typedef struct PathPose {
  double x;
  double y;
  double theta;
} PathPose;

extern "C" void cleanupPath(PathPose* path) {
  std::cout << "Cleaning up memory.." << std::endl;
  free(path);
}

extern "C" bool plan_multiple_circles(const char* mapFilename, double occupiedThreshold, double mapResolution, double robotRadius, double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs) {

  double pLen = 0.0;
  int numInterpolationPoints = 0;
  ob::StateSpacePtr space(new ob::ReedsSheppStateSpace(turningRadius));

  ///////////////////////////
  int** occ;
  png_bytepp map;
  png_structp png_ptr;
  png_infop info_ptr;
  FILE * fp;
  png_uint_32 width;
  png_uint_32 height;
  int bit_depth;
  int color_type;
  int interlace_method;
  int compression_method;
  int filter_method;
  int rowbytes;
  fp = fopen (mapFilename, "rb");
  if (! fp) {
    std::cout << "Could not load map " << mapFilename << " for validity checking" << std::endl;
  }
  png_ptr = png_create_read_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if (! png_ptr) {
    std::cout << "Could not create PNG read structure " << mapFilename << " for validity checking" << std::endl;
  }
  info_ptr = png_create_info_struct (png_ptr);
  if (! png_ptr) {
    std::cout << "Could not create PNG info structure " << mapFilename << " for validity checking" << std::endl;
  }
  png_init_io (png_ptr, fp);
  png_read_png (png_ptr, info_ptr, 0, 0);
  png_get_IHDR (png_ptr, info_ptr, & width, & height, & bit_depth,
		& color_type, & interlace_method, & compression_method,
		& filter_method);
  map = png_get_rows (png_ptr, info_ptr);
  ///////////////////////////
  
  rowbytes = png_get_rowbytes (png_ptr, info_ptr);
  int bpp = rowbytes/width;
  std::cout << "Loaded map " << mapFilename << " (" << width << "x" << height << " pixels) rowbytes = " << rowbytes << " bitdepth = " << bit_depth << " bytes-per-pixel = " << bpp << std::endl;

  int threshold = (int)((1-occupiedThreshold)*255);
  std::cout << "Occupied threshold = " << occupiedThreshold << ", pixel threshold = " << threshold << std::endl;
  
  occ = (int**)malloc(height*sizeof(int*)); 
  for (int i = 0; i < height; i++) occ[i] = (int*)malloc(width*sizeof(int)); 
  
  FILE * fpo;
  FILE * fpoV;
  fpo = fopen ("/home/fpa/tempMap.txt","w");
  fpoV = fopen ("/home/fpa/tempMapValues.txt","w");
  for (int j = 0; j < height; j++) {
    int i;
    png_bytep row;
    row = map[j];
    for (i = 0; i < rowbytes; i+=bpp) {
      png_byte pixel = 0;
      for (int k = 0; k < bpp; k++) pixel += row[i+k];
      pixel /= bpp;
      occ[j][i/bpp] = (int)pixel;

      if (pixel < threshold) {
	fprintf (fpo,"#");
      }

      else {
	fprintf (fpo,".");
      }
      fprintf (fpoV,"%d ",occ[j][i/bpp]);
      
    }
    fprintf (fpo,"\n");
    fprintf (fpoV,"\n");
  }
  fclose (fpo);
  fclose (fpoV);
  
  ob::ScopedState<> start(space), goal(space);
  ob::RealVectorBounds bounds(2);
  bounds.low[0] = 0;
  bounds.low[1] = 0;
  bounds.high[0] = width*mapResolution;
  bounds.high[1] = height*mapResolution;
	  
  space->as<ob::SE2StateSpace>()->setBounds(bounds);
  std::cout << "Bounds are [(" << bounds.low[0] << "," << bounds.low[1] << "),(" << bounds.high[0] << "," << bounds.high[1] << ")]" << std::endl;

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking for this space
  ob::SpaceInformationPtr si(ss.getSpaceInformation());
  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new MultipleCircleStateValidityChecker(si, occ, threshold, mapResolution, width*mapResolution, height*mapResolution, robotRadius, xCoords, yCoords, numCoords)));
  
  //Return false if the start is occupied.
  ompl::base::State *statePtrS = space->allocState();
  statePtrS->as<ompl::base::SE2StateSpace::StateType>()->setX(startX);
  statePtrS->as<ompl::base::SE2StateSpace::StateType>()->setY(startY);
  statePtrS->as<ompl::base::SE2StateSpace::StateType>()->setYaw(startTheta);
  std::cout << "Checking start pose (" << startX << "," << startY << "," << startTheta << ")" << std::endl;
  bool isStartValid = si->getStateValidityChecker()->isValid(statePtrS);
  space->freeState(statePtrS);
  if (!isStartValid) {
    std::cout << "Invalid start pose (" << startX << "," << startY << "," << startTheta << ") since pixel(s) around (" << startX/mapResolution << "," << (height-startY/mapResolution) << ") are occupied" << std::endl;
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
    std::cout << "Invalid goal pose (" << goalX << "," << goalY << "," << goalTheta << ") since pixel(s) around (" << goalX/mapResolution << "," << (height-goalY/mapResolution) << ") are occupied" << std::endl;
    return false;
  }

  ob::PlannerPtr planner(new og::RRTConnect(si));
  //ob::PlannerPtr planner(new og::RRTstar(si));
  //ob::PlannerPtr planner(new og::TRRT(si));
  //ob::PlannerPtr planner(new og::SST(si));
  //ob::PlannerPtr planner(new og::LBTRRT(si));
  //ob::PlannerPtr planner(new og::PRMstar(si));
  //ob::PlannerPtr planner(new og::SPARS(si));
  //ob::PlannerPtr planner(new og::pRRT(si));
  //ob::PlannerPtr planner(new og::LazyRRT(si));
  ss.setPlanner(planner);

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

extern "C" bool plan_multiple_circles_nomap(double* xCoords, double* yCoords, int numCoords, double startX, double startY, double startTheta, double goalX, double goalY, double goalTheta, PathPose** path, int* pathLength, double distanceBetweenPathPoints, double turningRadius, double planningTimeInSecs) {

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

  ob::PlannerPtr planner(new og::RRTConnect(si));
  //ob::PlannerPtr planner(new og::RRTstar(si));
  //ob::PlannerPtr planner(new og::TRRT(si));
  //ob::PlannerPtr planner(new og::SST(si));
  //ob::PlannerPtr planner(new og::LBTRRT(si));
  //ob::PlannerPtr planner(new og::PRMstar(si));
  //ob::PlannerPtr planner(new og::SPARS(si));
  //ob::PlannerPtr planner(new og::pRRT(si));
  //ob::PlannerPtr planner(new og::LazyRRT(si));
  ss.setPlanner(planner);
  
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
