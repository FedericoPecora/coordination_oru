package se.oru.coordination.coordination_oru.simulation2D;

public class State {
	
	private double position;
	private double velocity;
	
	public State(double distance, double velocity) {
		this.position = distance;
		this.velocity = velocity;
	}
	
	public double getPosition() {
		return position;
	}
	
	public void setPosition(double distance) {
		this.position = distance;
	}
	
	public double getVelocity() {
		return velocity;
	}
	
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}
	
	@Override
	protected State clone() {
		return new State(this.getPosition(), this.getVelocity());
	}
	
	@Override
	public String toString() {
		return "Pos: " + this.getPosition() + " Vel: " + this.getVelocity();
	}
	

}
