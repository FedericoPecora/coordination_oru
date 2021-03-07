package se.oru.coordination.coordination_oru.tests.testsPedestrians;

import com.vividsolutions.jts.geom.Coordinate;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import se.oru.coordination.coordination_oru.*;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.PedestrianForwardModel;
import se.oru.coordination.coordination_oru.simulation2D.PedestrianTrajectory;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulationWithPedestrians;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeTrackerPedestrian;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.ColorPrint;
import se.oru.coordination.coordination_oru.util.Missions;
import org.metacsp.utility.logging.MetaCSPLogging;

import java.awt.*;
import java.io.File;
import java.io.FilenameFilter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Map;
import java.util.Vector;
import java.util.logging.Level;
import java.util.logging.Logger;

@DemoDescription(desc = "One-shot navigation of several pedestrians and a robot coordinating on static paths that overlap in a straight portion.")
public class MultiplePedestriansAndRobot {

    static final int totalAgentsToLoad = 20;

    public static void main(String[] args) throws InterruptedException {

        Logger metaCSPLogger = MetaCSPLogging.getLogger(MultiplePedestriansAndRobot.class);

        double MAX_ACCEL = 1.0;
        double MAX_VEL = 1.0;

        String timeStr = "t3";
        String scenarioType = "corridor2";


        Map<String, String> environmentVariables = System.getenv();
        if(environmentVariables.containsKey("P_TIME")) { timeStr = environmentVariables.get("P_TIME"); }
        if(environmentVariables.containsKey("S_TYPE")) { scenarioType = environmentVariables.get("S_TYPE"); }

        String scenarioStr = "atc/" + scenarioType;
        String scenarioName =  scenarioType + "-" + timeStr;
        String robotPathDir = "chitt_tests/robot/" + scenarioStr + "/" + timeStr;
        String pedestrianPathDir = "chitt_tests/pedestrians/atc/" + timeStr;

        Vector<String> robotPathFilesName = new Vector<String>();

        FilenameFilter matchingRobotPathNameFilter = new FilenameFilter() {
            @Override
            public boolean accept(File file, String name) {
                return name.contains(".path");
            }
        };

        File robotDir = new File(robotPathDir);
        for (final File f : robotDir.listFiles(matchingRobotPathNameFilter)) {
            robotPathFilesName.add(f.getName().split(".path")[0]);
        }

        ColorPrint.info("un-sorted values: " + robotPathFilesName.toString());

        robotPathFilesName.sort(new Comparator<String>() {
            @Override
            public int compare(String s, String t1) {
                //ColorPrint.warning(s + ", " + s.lastIndexOf("_") + ", " + s.length() + ", " + s.substring(s.lastIndexOf("_")+1, s.length()-1));
                Integer s_num = Integer.parseInt(s.substring(s.lastIndexOf("_")+1));
                Integer t_num = Integer.parseInt(t1.substring(t1.lastIndexOf("_")+1));
                if(t_num < s_num) return 1;
                else if (s_num < t_num) return -1;
                else return 0;
            }
        });

        ColorPrint.info("Sorted values: " + robotPathFilesName.toString());

        String pathFileName = robotPathFilesName.get(Integer.parseInt(args[0]));

        ColorPrint.positive("RUNNING TEST FOR: " + pathFileName);

        final double threshold = 2.0;

        //Instantiate a trajectory envelope coordinator.
        //The TrajectoryEnvelopeCoordinatorSimulation implementation provides
        // -- the factory method getNewTracker() which returns a trajectory envelope tracker
        // -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
        //You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
        final TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec = new TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(MAX_VEL, MAX_ACCEL);
        /*tec.addComparator(new Comparator<RobotAtCriticalSection>() {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                int returnValue = 1;
                CriticalSection cs = o1.getCriticalSection();
                RobotReport robotReport1 = o1.getRobotReport();
                RobotReport robotReport2 = o2.getRobotReport();

                Trajectory traj1 = tec.getCurrentTrajectoryEnvelope(o2.getRobotReport().getRobotID()).getTrajectory();

                if (tec.isUncontrollable(o2.getCriticalSection().getTe1().getRobotID())) {
                    double o2DistToCP = TrajectoryEnvelopeTrackerPedestrian.computeDistance(traj1, Math.max(0, robotReport2.getPathIndex()), cs.getTe2Start());
                    if (o2DistToCP < threshold) returnValue = 1;
                    else
                        returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
                } else if (tec.isUncontrollable(o1.getRobotReport().getRobotID())) {
                    double o1DistToCP = TrajectoryEnvelopeTrackerPedestrian.computeDistance(traj1, Math.max(0, robotReport1.getPathIndex()), cs.getTe1Start());
                    if (o1DistToCP < threshold) returnValue = -1;
                    else
                        returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
                }

                // If both are robots. We don't need it now, but for the sake of future prosperity...
                else {
                    returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
                }

                return returnValue;
            }
        });*/

        tec.addComparator(new Comparator<RobotAtCriticalSection> () {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                CriticalSection cs = o1.getCriticalSection();
                RobotReport robotReport1 = o1.getRobotReport();
                RobotReport robotReport2 = o2.getRobotReport();
                return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
            }
        });

        // Set up infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);

        // Start the thread that revises precedences at every period
        tec.startInference();

        // Don't solve deadlocks
        tec.setBreakDeadlocks(false, false, false);

        // Set up Finest logging
        MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.WARNING);

        // Pedestrian Footprints
        // A small circle of diameter 0.3m
        Coordinate[] pedestrianFootprint = {
                new Coordinate(0.3000, 0.0000),
                new Coordinate(0.2427, 0.1763),
                new Coordinate(0.0927, 0.2853),
                new Coordinate(-0.0927, 0.2853),
                new Coordinate(-0.2427, 0.1763),
                new Coordinate(-0.3000, 0.0000),
                new Coordinate(-0.2427, -0.1763),
                new Coordinate(-0.0927, -0.2853),
                new Coordinate(0.0927, -0.2853),
                new Coordinate(0.2427, -0.1763)
        };

        // Robot Footprints
        // [[0.5, 0.2], [0.5, -0.2], [-0.2, -0.2], [-0.2, 0.2]]
        Coordinate f1 = new Coordinate(0.5, 0.2);
        Coordinate f2 = new Coordinate(0.5, -0.2);
        Coordinate f3 = new Coordinate(-0.2, -0.2);
        Coordinate f4 = new Coordinate(-0.2, 0.2);

        final ArrayList<Integer> nums = new ArrayList<Integer>();
        // Filter names
        FilenameFilter matchingNameFilter = new FilenameFilter() {
            @Override
            public boolean accept(File file, String name) {
                return name.contains("person");
            }
        };

        File pedestrianDir = new File(pedestrianPathDir);
        for (final File f : pedestrianDir.listFiles(matchingNameFilter)) {
            nums.add(Integer.parseInt(f.getName().split("person")[1].split(".txt")[0]));
            if(nums.size() == totalAgentsToLoad) {
                ColorPrint.info("READ " + nums.size() + "pedestrians...");
                break;
            }
        }

        // Add robot
        nums.add(1729);

        //JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
        BrowserVisualization viz = new BrowserVisualization();
        //RVizVisualization viz = new RVizVisualization();
        viz.setMap("maps/atc.yaml");
        int[] nums_primitive = new int[nums.size()];
        for (int i = 0; i < nums_primitive.length; i++) {
            nums_primitive[i] = nums.get(i);
        }
        //RVizVisualization.writeRVizConfigFile(nums_primitive);
        //viz.setInitialTransform(40, 3, -10);
        viz.setInitialTransform(40, -10, 30);
        tec.setVisualization(viz);

        ArrayList<Integer> addedMissions = new ArrayList<Integer>();

        PedestrianTrajectory[] pedestrianTrajectories = new PedestrianTrajectory[nums_primitive.length];
        long startTime = 0;

        // Robot Path
        PoseSteering[] robotPath = Missions.loadPathFromFile(robotPathDir + "/" + pathFileName + ".path");;

        for (int i = 0; i < nums.size(); i++) {

            // One robot. Others behave as pedestrians.
            if (nums.get(i) != 1729) {
                pedestrianTrajectories[i] = new PedestrianTrajectory(pedestrianPathDir + "/person" + nums.get(i) + ".txt");
                tec.setFootprint(nums.get(i), pedestrianFootprint);
                tec.addUncontrollableRobots(nums.get(i));
                tec.setForwardModel(nums.get(i), new PedestrianForwardModel());
                tec.addPedestrianTrajectory(nums.get(i), pedestrianTrajectories[i]);

                Missions.enqueueMission(new Mission(nums.get(i), pedestrianTrajectories[i].getPoseSteeringAsArray()));

            } else {
                tec.setFootprint(nums.get(i), f1, f2, f3, f4);
                tec.setForwardModel(nums.get(i), new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
                tec.placeRobot(nums.get(i), robotPath[0].getPose());
                Missions.enqueueMission(new Mission(nums.get(i), robotPath));
            }
        }

        Missions.startMissionDispatchers(tec, false, 1729);

        startTime = tec.getCurrentTimeInMillis();

        while (addedMissions.size() != nums_primitive.length) {
            long timeNow = tec.getCurrentTimeInMillis();

            for (int i = 0; i < nums.size(); i++) {
                if (addedMissions.contains(nums.get(i)))
                    continue;
                PedestrianTrajectory pI = pedestrianTrajectories[i];
                // One robot. Others behave as pedestrians.
                // When the current time is greater than the start time of a pedestrian, we start that pedestrian
                if (nums.get(i) != 1729 && timeNow > startTime + pI.getStartTime() * 1000 && !addedMissions.contains(i)) {
                    tec.placeRobot(nums.get(i), pI.getPose(0));
                    ColorPrint.info("Adding mission for Pedestrian " + nums.get(i));
                    ColorPrint.info("timeNow: " + timeNow + ", startTime: " + startTime + ", first stamp: " + pI.getStartTime() * 1000);
                    Missions.startMissionDispatchers(tec, false, nums.get(i));
                    addedMissions.add(nums.get(i));
                }

                if(nums.get(i) == 1729) {
                    addedMissions.add(1729);
                }

                final int finalI = i;
                // Add a tracking callback for each ID
                long finalStartTime = startTime;
                tec.addTrackingCallback(nums.get(i), new TrackingCallback() {

                    @Override
                    public void onTrackingStart() {
                        // TODO Auto-generated method stub
                    }

                    @Override
                    public void onTrackingFinished() {

                        if(nums.get(finalI) == 1729) {
                            RobotReport thisRobotReport = tec.getRobotReport(nums.get(finalI));

                            if(thisRobotReport.getPose().distanceTo(robotPath[robotPath.length-1].getPose()) < 0.01) {
                                ColorPrint.positive("Ending Experiment: Robot has reached it's destination.");
                                for(int a = 0; a < nums.size(); a++) {
                                    ColorPrint.positive("Waiting time for robot " + nums.get(a) + ": " + tec.getRobotStoppageTime(nums.get(a)));
                                    MyTrackingCallback.writeToLogFile(String.valueOf(nums.get(a)) + ", " + tec.getRobotStoppageTime(nums.get(a)) + ", " + tec.getRobotStops(nums.get(a)) + "\n", scenarioName + "/" + pathFileName + "_waiting_times.txt");
                                }
                                System.exit(0);
                            }
                        }

                        //ColorPrint.positive("Waiting time for robot " + nums.get(finalI) + ": " + tec.getRobotStoppageTime(nums.get(finalI)));
                        //System.out.println("Does Robot" + nums.get(finalI) + " have more missions? " + Missions.hasMissions(nums.get(finalI)));
                        //MyTrackingCallback.writeToLogFile(String.valueOf(nums.get(finalI)) + ", " + tec.getRobotStoppageTime(nums.get(finalI)) + ", " + tec.getRobotStops(nums.get(finalI)) + "\n", scenarioName + "/" + pathFileName + "_waiting_times.txt");

                    }

                    @Override
                    public String[] onPositionUpdate() {
                        //int currentCriticalSections = tec.getCurrentCriticalSections().size();


                        //if (nums.get(finalI) == 1729 && currentCriticalSections != MyTrackingCallback.criticalSections) {
                        //    MyTrackingCallback.criticalSections = currentCriticalSections;
                        //    MyTrackingCallback.writeToLogFile(String.valueOf(MyTrackingCallback.criticalSections) + "\n", scenarioName + "/" + pathFileName + "_critical_sections.txt");
                        //}

                        return null;
                    }

                    @Override
                    public void onNewGroundEnvelope() {
                        // TODO Auto-generated method stub

                    }

                    @Override
                    public void beforeTrackingStart() {
                        // TODO Auto-generated method stub

                    }

                    @Override
                    public void beforeTrackingFinished() {
                        // TODO Auto-generated method stub
                    }
                });
            }
            Thread.sleep(100);
        }
        ColorPrint.info("All pedestrians added.");

        while(true) {
            Thread.sleep(50);
            if(tec.getCurrentTimeInMillis() - startTime > 600000) {
                ColorPrint.positive("Ending Experiment: Time limit exceeded.");
                System.exit(0);
            }
        }
    }
}
