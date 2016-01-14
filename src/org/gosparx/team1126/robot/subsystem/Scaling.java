package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
/**
 * Allows the robot to scale the tower
 * @author Andrew Thompson {andrewt015@gmail.com}
 */
public class Scaling extends GenericSubsystem{

	public Scaling(String name, int priority) {
		super(name, priority);
	}

	@Override
	protected boolean init() {
		return false;
	}

	@Override
	protected void liveWindow() {
		
	}

	@Override
	protected boolean execute() {
		return false;
	}

	@Override
	protected long sleepTime() {
		return 0;
	}

	@Override
	protected void writeLog() {
		
	}

}
