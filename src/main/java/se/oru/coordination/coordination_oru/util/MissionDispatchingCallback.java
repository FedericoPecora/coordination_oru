package se.oru.coordination.coordination_oru.util;

import se.oru.coordination.coordination_oru.Mission;

public interface MissionDispatchingCallback {
	
	public void beforeMissionDispatch(Mission m);
	
	public void afterMissionDispatch(Mission m);
	
}
