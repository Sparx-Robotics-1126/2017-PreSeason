package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

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
	 * Instance of drives //TODO do i need this?
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
	 * Solenoid to extend the right arms
	 */
	private Solenoid rightSolenoid;
	
	/**
	 * Solenoid to extend the left arms
	 */
	private Solenoid leftSolenoid;
	
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
	
	/**
	 * How much line the winch intakes per tick
	 */
	private final double DISTANCE_PER_TICK = 1; //TODO find distance per tick
	
	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = false;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;
	
	//******************************VARIABLES***********************************

	/**
	 * Wanted speed for the right motors
	 */
	private double wantedRightPower;
	
	/**
	 * Wanted speed for the left motors
	 */
	private double wantedLeftPower;
	
	/**
	 * The current state that scaling is in
	 */
	private State currentScalingState;
	
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
		
		//Right 
		rightFront = new CANTalon(0); //TODO figure out these numbers 
		rightBack = new CANTalon(0);
		rightSolenoid = new Solenoid(0);
		encoderRight = new Encoder(0,0);
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK);
		wantedRightPower = 0;
		
		//Left
		leftFront = new CANTalon(0); 
		leftBack = new CANTalon(0);
		leftSolenoid = new Solenoid(0);
		encoderLeft = new Encoder(0,0);
		encoderDataLeft = new EncoderData(encoderRight,DISTANCE_PER_TICK);
		wantedLeftPower = 0;
		
		//Other
		currentScalingState = State.STANDBY;
		drives = Drives.getInstance();  //TODO do i need this 
		return true;
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
