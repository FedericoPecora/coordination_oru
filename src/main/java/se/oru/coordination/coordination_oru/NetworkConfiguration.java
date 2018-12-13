package se.oru.coordination.coordination_oru;

public class NetworkConfiguration {
	
	/**
	 * The probability of not receiving a message.
	 */
	public static double PROBABILITY_OF_PACKET_LOSS = 0.0;
		
	/**
	 * The maximum transmission delay.
	 */
	protected static int maximumTxDelay = 0;
	
	/**
	 * The minimum transmission delay.
	 */
	protected static int minimumTxDelay = 0;
	
	/**
	 * Static method to set minimum and maximum network delays.
	 */
	public static void setDelays(int minimum, int maximum) {
		int max = Math.max(0, Math.max(minimum, maximum));
		int min = Math.max(0, Math.min(minimum, maximum));
		if ( max != maximum || min != minimum)
			System.out.println("* WARNING * Wrong transmission delays. Setting max: " + max + ", min: " + min +".");
		maximumTxDelay = max;
		minimumTxDelay = min;	
	}

	/**
	 * Get the maximum transmission delay.
	 * @return The maximum transmission delay.
	 */
	public static int getMaximumTxDelay() {
		return maximumTxDelay;
	}
	
	/**
	 * Get the minimum transmission delay.
	 * @return The minimum transmission delay.
	 */
	public static int getMinimumTxDelay() {
		return minimumTxDelay;
	}
	
}
