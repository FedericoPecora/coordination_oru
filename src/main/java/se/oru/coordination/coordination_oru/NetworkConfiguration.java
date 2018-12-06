package se.oru.coordination.coordination_oru;

public class NetworkConfiguration {
	
	/**
	 * The probability of not transmitting a message to a robot (failing to set a critical point) or not receiving status messages.
	 */
	public static double PROBABILITY_OF_PACKET_LOSS = 0.0;
	
	/**
	 * The maximum probability of faults: used to find the minimum number of messages to send.
	 */
	public static double MAXIMUM_PROBABILITY_OF_FAULTS = 0.0;
	
	/**
	 * The maximum delay of messages to robots (delay in setting a critical point).
	 */
	public static int MAXIMUM_TX_DELAY = 0;

	
}
