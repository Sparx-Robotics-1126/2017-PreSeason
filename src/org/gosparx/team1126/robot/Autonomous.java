package org.gosparx.team1126.robot;

import org.gosparx.team1126.robot.subsystem.GenericSubsystem;

public class Autonomous extends GenericSubsystem{

	public Autonomous() {
		super("Autonomous", Thread.NORM_PRIORITY);
	}

	@Override
	protected boolean init() {
		return true;
	}

	@Override
	protected boolean execute() {
		return false;
	}

	@Override
	protected void liveWindow() {

	}	

	@Override
	protected long sleepTime() {
		return 20;
	}

	@Override
	protected void writeLog() {

	}

}
