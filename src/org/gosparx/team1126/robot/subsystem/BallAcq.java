package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.sensors.EncoderData;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Purpose: to acquire/get the balls and then score them
 * @author Allison and Jack
 */

public class BallAcq extends GenericSubsystem{

//*****************************Constants*******************************************	

	/**
	 * the distance the blue wheel will travel per tick 
	 */
	private static final double DISTANCE_PER_TICK_BLUE = 0;
	
	/**
	 * the distance the arm will travel per tick
	 */
	private static final double DISTANCE_PER_TICK_ARM = 0;
	
//*****************************Objects*********************************************
	
	/**
	 * creates an instance of BallAcq
	 */
	private static BallAcq acq;
	
	/**
	 * the motor that rotates the larger blue wheel
	 */
	private CANTalon blueWheelMotor;
	
	/**
	 * the motor that rotates the arm
	 */
	private CANTalon armMotor;
	
	/**
	 * the motor that rotates the motor
	 */
	private CANTalon rollerMotor;
	
	/**
	 * the encoder that tracks the motion of the blue wheel
	 */
	private Encoder blueEncoder;
	
	/**
	 * the encoder that tracks the motion of the arm
	 */
	private Encoder armEncoder;
	
	/**
	 * the encoder data for the blue wheel rotation
	 */
	private EncoderData blueEncoderData;
	
	/**
	 * the encoder data for the arm rotation
	 */
	private EncoderData armEncoderData;

//*****************************Variables*******************************************
	
	/**
	 * the current state of the blue wheel
	 */
	private blueWheelState currentBlueState;
	
	/**
	 * the current state of the arm
	 */
	private armState currentArmState;
	
	/**
	 * the wanted angle of the blue wheel
	 */
	private double wantedBlueAngle;
	
	/**
	 * the wanted angle of the arm
	 */
	private double wantedArmAngle;
	
//*****************************Methods*********************************************	
	
	/**
	 * makes sure that there is only one instance of BallAcq
	 * @return a BallAcq object
	 */
	public static synchronized BallAcq getInstance(){
		if(acq == null){
			acq = new BallAcq();
		}
		return acq;
	}
	
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
		
		return false;
	}

	/**
	 * to add data to objects while in test mode
	 */
	@Override
	protected void liveWindow() {
		
	}

	/**
	 * Acquires the ball and then scores/passes the ball
	 * @return false to continue loop
	 */
	@Override
	protected boolean execute() {
		
		return false;
	}

	/**
	 * the amount of time that the BallAcq class will sleep
	 * @return the amount of time between cycles, in milliseconds (ms)
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * writes a log to the consul every 5 seconds
	 */
	@Override
	protected void writeLog() {
		
	}
	
	/**
	 * the states for the blue wheel
	 */
	public enum blueWheelState{
		STANDBY,
		ROTATING,
		ROTATE_FINDING_HOME;
		
		/**
		 * Gets the state name
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "In standby";
			case ROTATING:
				return "Rotating";
			case ROTATE_FINDING_HOME:
				return "Rotater Finding Home";
			default:
				return "Error :(";
			}
		}
	}
	
	/**
	 * the states for the arm 
	 */
	public enum armState{
		STANDBY,
		ROTATING,
		ROTATE_FINDING_HOME;
	
		/**
		 * Gets the state name
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "In Standby";
			case ROTATING:
				return "Rotating";
			case ROTATE_FINDING_HOME:
				return "Rotate Finding Home";
			default:
				return "Error :(";
			}
		}
	}
}
