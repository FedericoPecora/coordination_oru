package se.oru.coordination.coordination_oru.simulation2D;

public class Derivative {
	
	public static double EPSILON = 0.0001;
	private double velocity;
	private double acceleration;
	
	public Derivative() {
		this.velocity = 0.0;
		this.acceleration = 0.0;
	}
	
	public Derivative(double velocity, double acceleration) {
		this.velocity = velocity;
		this.acceleration = acceleration;
	}
	
	public static Derivative evaluate(State initialState, double time, double deltaTime, Derivative deriv, boolean slowDown, double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION) {
		double position = initialState.getPosition() + deriv.getVelocity()*deltaTime;
		double velocity = initialState.getVelocity() + deriv.getAcceleration()*deltaTime;
		State newState = new State(position, velocity);
		double newVelocity = newState.getVelocity();
		double newAcceleration = computeAcceleration(newState, time+deltaTime, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
		return new Derivative(newVelocity, newAcceleration);
	}
			
	public static double computeAcceleration(State state, double time, boolean slowDown, double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION) {
		if (!slowDown) {
			if (state.getVelocity() > MAX_VELOCITY_DAMPENING_FACTOR*MAX_VELOCITY) return 0.0;
			return MAX_ACCELERATION;
		}
		return -MAX_ACCELERATION;
	}

	public double getVelocity() {
		return this.velocity;
	}	
	
	public double getAcceleration() {
		return this.acceleration;
	}

}
