package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Allows the robot to scale the tower
 * @author Andrew Thompson {andrewt015@gmail.com}
 */
public class Scaling extends GenericSubsystem{
	
	//******************************OBJECTS***********************************
	
	/**
	 * The only instance of Scaling 
	 */
	private static Scaling scaling;
	
	/**
	 * Instance of drives 
	 */
	private Drives drives;
	
	/**
	 * Controller for the right front motor and right side winch 
	 */
	private CANTalon rightFront;
	
	/*
	 * Controller for the right back motor and right side winch
	 */
	private CANTalon rightBack;
	
	/**
	 * Controller for the left front motor and right side winch
	 */
	private CANTalon leftFront;
	
	/**
	 * Controller for the left back motor and right side winch
	 */
	private CANTalon leftBack;
	
	/**
	 * Used to get the distance the robot has traveled for the right drives as well as how much line the right drum has taken in
	 */
	private Encoder encoderRight;
	
	/**
	 * Used to get the distance the robot has traveled for the left drives as well as how much line the left drum has taken in
	 */
	private Encoder encoderLeft;
	
	/**
	 * Calculates how far the robot traveled on the right side and how much line has been taken in on the right side
	 */
	private EncoderData encoderDataRight;
	
	/**
	 * Calculates how far the robot traveled on the left side and how much line has been taken in on the left side
	 */
	private EncoderData encoderDataLeft;
	
	//******************************CONSTANTS***********************************
	
	//******************************VARIABLES***********************************

	/**
	 * Returns the only instance of scaling
	 */
	public static synchronized Scaling getInstance(){
		if(scaling == null){
			scaling = new Scaling();
		}
		return scaling;
	}
	
	/**
	 * Creates a new scaling 
	 */
	public Scaling() {
		super("Scaling", Thread.NORM_PRIORITY);
	}

	/**
	 * Initializes things 
	 */
	@Override
	protected boolean init() {
		return false;
	}

	/**
	 * Adds items to the live window, overrides genericSubsystems 
	 */
	@Override
	protected void liveWindow() {
		
	}
	
	/**
	 * Loops
	 */
	@Override
	protected boolean execute() {
		return false;
	}
	
	/**
	 * Sleep time between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}
	
	/**
	 * Writes info about the subsystem to the log 
	 */
	@Override
	protected void writeLog() {
		
	}
	/**
	 *Makes the states for scaling
	 */
	public enum State{
		STANDBY,
		EXTENDING,
		EXTENDED,
		SCALING,
		SCALED,
		OVERRIDE;

		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "Standby";
			case EXTENDING:
				return "Extending";
			case EXTENDED:
				return "Extended";
			case SCALING:
				return "Scaling";
			case SCALED:
				return "Scaled";
			case OVERRIDE:
				return "Overriding";
			default:
				return "Unknown state";
			}
		}
	}
}
