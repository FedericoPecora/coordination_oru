package se.oru.coordination.coordination_oru;

/**
 * Class used to order indexed delays for a robot driving a given {@link TrajectoryEnvelope}.
 * @note The class uses a not standard compareTo() -- the results of the equals() and the compareTo() may be different,
 * so be sure that the right function will be used when adding or removing {@link IndexedDelay} from a specific data structure.
 * @author anna
 *
 */
public class IndexedDelay implements Comparable<IndexedDelay> {
	private final int teID;
	private final int csHash;
	private int index;
	private double value;

	/**
	 * Class constructor.
	 * @param teID The ID of the {@link TrajectoryEnvelope}.
	 * @param csHash  The hash code of the {@link CriticalSection} related to the given path index or 0 if the delay is related to a stopping point (never a possible code for CS).
	 * @param index The path index at which the robot will be delayed.
	 * @param value The estimated delay (in secs).
	 */
	IndexedDelay(int teID, int csHash, int index, double delay) {
		this.teID = teID;
		this.csHash = csHash;
		this.index = index;
		this.value = delay;
	}
	
	/**
	 * Returning the index of this delay.
	 * @return The path index at which the robot will be delayed.
	 */
	public int getIndex() {
		return this.index;
	}
	
	/**
	 * Returning the estimated delay.
	 * @return The estimated delay (in secs).
	 */
	public double getValue() {
		return this.value;
	}
	
	@Override
	/**Function for ordering indexed delays with increasing indices.
	 * @param other The delay to be compared.
	 * @return a negative integer, or a positive integer as this indexed delay is before or after the other one;
	 * 0 if the two delays are not comparable (related to different paths).
	 * @note While the compareTo() orders delays according to indices while assuming csHashs to be unique, the equals also considers
	 * the delay.
	 * @ATTENTION Be sure that the right function will be used when adding or removing {@link IndexedDelay} from a specific data structure.
	 */
	public int compareTo(IndexedDelay other) {
		if (teID != other.teID) return 0;
		if (this.index < other.index) return -1;
		if (this.index > other.index) return 1;
		double thisCode = Double.parseDouble(Long.toString(index) + "." + Integer.toString(csHash));
		double otherCode = Double.parseDouble(Long.toString(other.index) + "." + Integer.toString(other.csHash));
		if (thisCode < otherCode) return -1;
		if (thisCode > otherCode) return 1;
		return 0;		
	}



	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + csHash;
		result = prime * result + index;
		result = prime * result + teID;
		long temp;
		temp = Double.doubleToLongBits(value);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		IndexedDelay other = (IndexedDelay) obj;
		if (csHash != other.csHash)
			return false;
		if (index != other.index)
			return false;
		if (teID != other.teID)
			return false;
		if (Double.doubleToLongBits(value) != Double.doubleToLongBits(other.value))
			return false;
		return true;
	}

	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return super.toString();
	}
	
}
