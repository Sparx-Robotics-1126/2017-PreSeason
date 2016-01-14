package org.gosparx.team1126.robot;

import org.gosparx.team1126.robot.subsystem.GenericSubsystem;

/**
 * A class for handling the autonomous functions of the robot
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class Autonomous extends GenericSubsystem{

	/**
	 * Support for singleton
	 */
	private static Autonomous auto;
	
	/**
	 * Private constructor to aid in singleton.
	 */
	private Autonomous() {
		super("Autonomous", Thread.NORM_PRIORITY);
	}
	
	/**
	 * @return The only instance of Autonomous
	 */
	public static Autonomous getInstance(){
		if(auto == null){
			auto  = new Autonomous();
		}
		return auto;
	}

	/**
	 * Performed once when the subsystems .start() method is called.
	 */
	@Override
	protected boolean init() {
		return true;
	}

	/**
	 * Loops after .start() is called.
	 */
	@Override
	protected boolean execute() {
		return false;
	}

	/**
	 * Updates the livewindow functions
	 */
	@Override
	protected void liveWindow() {

	}	

	/**
	 * @return How long to sleep between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * Writes to the log file every 5 seconds.
	 */
	@Override
	protected void writeLog() {

	}

}
