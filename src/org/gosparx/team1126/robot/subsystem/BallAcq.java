package org.gosparx.team1126.robot.subsystem;

/**
 * Purpose: to acquire/get the balls and then score them
 * @author Allison
 */

public class BallAcq extends GenericSubsystem{

	/**
	 * creates a BallAcq object 
	 */
	public BallAcq() {
		super("BallAcq", Thread.NORM_PRIORITY);
	}

	/**
	 * instantiates objects and initializes variables
	 */
	@Override
	protected boolean init() {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * to add data to objects while in test mode
	 */
	@Override
	protected void liveWindow() {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Acquires the ball and then scores/passes the ball
	 * @return false to continue loop
	 */
	@Override
	protected boolean execute() {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * the amount of time that the BallAcq class will sleep
	 * @return the amount of time between cycles, in milliseconds (ms)
	 */
	@Override
	protected long sleepTime() {
		// TODO Auto-generated method stub
		return 0;
	}

	/**
	 * writes a log to the consul every 5 seconds
	 */
	@Override
	protected void writeLog() {
		// TODO Auto-generated method stub
		
	}

}
