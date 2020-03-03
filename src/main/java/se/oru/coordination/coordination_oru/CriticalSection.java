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
	
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		CriticalSection other = (CriticalSection) obj;
		if (te1 == null || te2 == null) {
			if (other.te1 != null && other.te2 != null)
				return false;
			else if (te1 == null) 
				return te2.equals((other.te1 == null) ? other.te2 : other.te1);
		} else {
			if (!(te1.equals(other.te1) && te2.equals(other.te2) || te1.equals(other.te2) && te2.equals(other.te1)))
				return false;
			if (te1.equals(other.te1) && te2.equals(other.te2)) {
				if (te1End != other.te1End || te2End != other.te2End || te1Start != other.te1Start || te2Start != other.te2Start)
					return false;
			}
			else if (te1.equals(other.te2) && te2.equals(other.te1)) {
				if (te1End != other.te2End || te2End != other.te1End || te1Start != other.te2Start || te2Start != other.te1Start)
					return false;
			}
		}
		return true;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((te1 == null) ? 0 : te1.hashCode()) + ((te2 == null) ? 0 : te2.hashCode());
		result = prime * result + te1End + te2End;
		result = prime * result + Math.abs(te1End - te2End);
		result = prime * result + te1Start + te2Start;
		result = prime * result + Math.abs(te1Start - te2Start);
		return result;
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
		String robot1 = (te1 == null ? "null" : "Robot"+te1.getRobotID());
		String robot2 = (te2 == null ? "null" : "Robot"+te2.getRobotID());
		ret += "CriticalSection (" + robot1 + " [" + te1Start + ";" + te1End + "], " + robot2 + " [" + te2Start + ";" + te2End + "])";
		return ret;
	}
}
