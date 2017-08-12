package se.oru.coordination.coordination_oru;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

/**
 * A critical section is defined as a quadruple (te1, te2, [start1,end1], [start2,end2]), where
 * te1 and te2 are {@link TrajectoryEnvelope}s that overlap when robot 1 is in pose
 * with index start1 and robot 2 is in pose with index start2, until the two robots reach poses
 * end1 and end2 respectively.
 * 
 * @author fpa
 *
 */
public class CriticalSection {
	
	private TrajectoryEnvelope te1;
	private TrajectoryEnvelope te2;
	private int te1Start = -1;
	private int te2Start = -1;
	private int te1End = -1;
	private int te2End = -1;
	private int te1Break = -1;
	private int te2Break = -1;

	public CriticalSection(TrajectoryEnvelope te1, TrajectoryEnvelope te2, int te1Start, int te2Start, int te1End, int te2End) {
		this.te1 = te1;
		this.te2 = te2;
		this.te1Start = te1Start;
		this.te2Start = te2Start;
		this.te1End = te1End;
		this.te2End = te2End;
	}

	public TrajectoryEnvelope getTe1() {
		return te1;
	}

	public TrajectoryEnvelope getTe2() {
		return te2;
	}

	public int getTe1Start() {
		return te1Start;
	}

	public int getTe2Start() {
		return te2Start;
	}

	public int getTe1End() {
		return te1End;
	}

	public int getTe2End() {
		return te2End;
	}

	public int getTe1Break() {
		return te1Break;
	}
	
	public int getTe2Break() {
		return te2Break;
	}
	
	public void setTe1Break(int te1Break) {
		this.te1Break = te1Break;
	}
	
	public void setTe2Break(int te2Break) {
		this.te2Break = te2Break;
	}
	
	@Override
	public String toString() {
		String ret = "";
		ret += "CriticalSection (Robot" + te1.getRobotID() + " [" + te1Start + ";" + te1End + "], Robot" + te2.getRobotID() + " [" + te2Start + ";" + te2End + "])"; //\n\t" + te1 + "\n\t" + te2; 
		return ret;
	}
}
