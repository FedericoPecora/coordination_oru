package se.oru.coordination.coordination_oru;

/**
 * This interface to define scenario-specific robot types at running time.
 * The interface should be implemented via public static enum which should include
 * - a value for each robot in the fleet (to be used to assign tasks to a specific robot).
 * - a value for each robots' type (e.g., manipulator, kiva like, mobile manipulator, ... ).
 * 
 * @author anmi
 *
 */
public interface RobotTypeInterface { }