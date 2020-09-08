/**
 * 
 */
package se.oru.coordination.coordination_oru.util;

/**
 * @author Chittaranjan Srinivas Swaminathan
 * \brief Simple coloured print out for important info.
 */
public final class ColorPrint {
	
	public static final String ANSI_BLUE = "\u001B[34m" + "\u001B[107m";
	public static final String ANSI_GREEN = "\u001B[32m" + "\u001B[107m";
	public static final String ANSI_RED = "\u001B[31m" + "\u001B[107m";
	public static final String ANSI_RESET = "\u001B[0m";
	
	/**
	 * Print an important ERROR message.
	 * @param string The message to be printed.
	 */
	public static void error(String string) {
		System.out.print(ANSI_RED + "ERROR: " + string);
		System.out.print(ANSI_RESET);
	}
	
	/**
	 * Print an important WARNING message.
	 * @param string The message to be printed.
	 */
	public static void warning(String string) {
		System.out.print(ANSI_RED + "WARNING: " + string);
		System.out.print(ANSI_RESET);
	}
	
	/**
	 * Print an important INFO message.
	 * @param string The message to be printed.
	 */
	public static void info(String string) {
		System.out.print(ANSI_BLUE + string);
		System.out.println(ANSI_RESET);
	}

	/**
	 * Print an important positive message.
	 * @param string The message to be printed.
	 */
	public static void positive(String string) {
		System.out.print(ANSI_GREEN + string);
		System.out.print(ANSI_RESET);
	}
}
