package se.oru.coordination.coordination_oru.simulation2D;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.lang.Math;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.util.ColorPrint;

public class PedestrianTrajectory {

    protected ArrayList<Pose> poses = null;
    private ArrayList<Double> speeds = null;
    private ArrayList<Double> timeStamps = null;
    private ArrayList<Double> dts = null;
    private ArrayList<Double> us = null;
    private ArrayList<Double> vs = null;

    private double startTime = 0.0;

    protected int size = 0;

    public double getStartTime() {return startTime; }

    public ArrayList<Pose> getPoses() {
        return poses;
    }

    public Pose getPose(int index) {
        return poses.get(index);
    }

    public ArrayList<Double> getTimeStamps() {
        return timeStamps;
    }

    double getTimeStamp(int index) {
        return timeStamps.get(index);
    }

    public ArrayList<Double> getDTs() {
        return dts;
    }

    public double getDT(int index) {
        return dts.get(index);
    }

    public ArrayList<Double> getSpeeds() {
        return speeds;
    }

    double getSpeed(int index) {
        return speeds.get(index);
    }

    public ArrayList<Double> getUs() {
        return us;
    }

    public double getU(int index) {
        return us.get(index);
    }

    public ArrayList<Double> getVs() {
        return vs;
    }

    public double getV(int index) {
        return vs.get(index);
    }

    public PoseSteering[] getPoseSteeringAsArray() {
        PoseSteering[] poseSteering = new PoseSteering[this.poses.size()];
        for (int i = 0; i < this.poses.size(); i++) {
            poseSteering[i] = new PoseSteering(this.poses.get(i), 0.0);
        }
        return poseSteering;
    }

    public Pose[] getPosesAsArray() {
        return this.poses.toArray(new Pose[this.poses.size()]);
    }

    public double[] getSpeedsAsArray() {
        double[] speeds = new double[this.speeds.size()];
        for (int i = 0; i < this.speeds.size(); i++) {
            speeds[i] = this.speeds.get(i);
        }
        return speeds;
    }

    public double[] getTimeStampsAsArray() {
        double[] timeStamps = new double[this.timeStamps.size()];
        for (int i = 0; i < this.timeStamps.size(); i++) {
            timeStamps[i] = this.timeStamps.get(i);
        }
        return timeStamps;
    }

    public double[] getDTsAsArray() {
        double[] dts = new double[this.dts.size()];
        for (int i = 0; i < this.dts.size(); i++) {
            dts[i] = this.dts.get(i);
        }
        return dts;
    }

    public int size() {
        return size;
    }

    public PedestrianTrajectory(String fileName) {
        loadTrajectoryFromFile(fileName);
    }

    private static boolean check(List... arrays) {
        if (arrays.length < 1) {
            return true;
        }
        int expectedSize = arrays[0].size();
        for (int i = 1; i < arrays.length; i++) {
            int sizeOfThisList = arrays[i].size();
            if (sizeOfThisList != expectedSize) {
                return false;
            }
        }
        return true;
    }

    /**
     * Read a path from a file.
     *
     * @param fileName The name of the file containing the pedestrian path
     */
    private void loadTrajectoryFromFile(String fileName) {
        this.poses = new ArrayList<Pose>();
        this.speeds = new ArrayList<Double>();
        this.timeStamps = new ArrayList<Double>();
        this.dts = new ArrayList<Double>();
        this.us = new ArrayList<Double>();
        this.vs = new ArrayList<Double>();
        try {
            Scanner in = new Scanner(new FileReader(fileName));
            while (in.hasNextLine()) {
                String[] oneline = in.nextLine().trim().split(" |\t");
                if (oneline.length == 6) {
                    poses.add(new Pose(
                            Double.parseDouble(oneline[0]),
                            Double.parseDouble(oneline[1]),
                            Double.parseDouble(oneline[2])));

                    double thisU = Double.parseDouble(oneline[3]);
                    double thisV = Double.parseDouble(oneline[4]);
                    us.add(thisU);
                    vs.add(thisV);

                    speeds.add(Math.sqrt(thisU * thisU + thisV * thisV));
                    timeStamps.add(Double.parseDouble(oneline[5]));
                } else {
                    ColorPrint.error("Error reading a line from " + fileName + ". Line did not contain 5 elements. Skipping line.");
                }

            }
            in.close();

            startTime = timeStamps.get(0);

            for(int i = 0; i < timeStamps.size(); i++) {
                timeStamps.set(i, timeStamps.get(i) - timeStamps.get(0));
            }

            if (check(poses, speeds, timeStamps)) {
                size = poses.size();
            } else {
                size = Math.max(timeStamps.size(), Math.max(poses.size(), speeds.size()));
                throw new IllegalArgumentException(ColorPrint.ANSI_RED + "ERROR in PedestrianTrajectory.java. Poses, speeds and time stamps read from the file were not all the same size. Check the file." + ColorPrint.ANSI_RESET);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        // Compute deltaTs for the Trajectory.
        this.dts = new ArrayList<Double>();
        for (int i = 0; i < this.timeStamps.size() - 1; i++) {
            this.dts.add(this.timeStamps.get(i + 1) - timeStamps.get(i));
        }
        dts.add(0.0);

    }
}
