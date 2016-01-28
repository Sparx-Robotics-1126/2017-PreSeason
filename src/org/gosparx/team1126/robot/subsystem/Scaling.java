package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
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
	 * Controller for the right bottom motor and right side winch
	 */
	private CANTalon rightBottom;
	
	/**
	 * Controller for the left front motor and right side winch
	 */
	private CANTalon leftFront;
	
	/**
	 * Controller for the left back motor and right side winch
	 */
	private CANTalon leftBack;
	
	/**
	 * Controller for the left bottom motor and left side winch
	 */
	private CANTalon leftBottom;
	
	/**
	 * Right hook sensor 
	 */
	private DigitalInput rightHook;
	
	/**
	 * Left hook sensor 
	 */
	private DigitalInput leftHook;
	
	/**
	 * Solenoid to extend first half of right arms
	 */
	private Solenoid rightSolenoidFirst;
	
	/**
	 * Solenoid to extend second half of right arms 
	 */
	private Solenoid rightSolenoidSecond;
	
	/**
	 * Solenoid to extend first half left arms
	 */
	private Solenoid leftSolenoidFirst;
	
	/**
	 * Solenoid to extend second half of left arms 
	 */
	private Solenoid leftSolenoidSecond;
	
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
	 * How much line the winch intakes per tick in inches
	 */
	private final double DISTANCE_PER_TICK_INCHES = 1; //TODO find distance per tick
	
	/**
	 * The distance to the bar and how much line the winch must take in 
	 */
	private final double DISTANCE_TO_BAR_INCHES = 16; //TODO find actual height
	
	/**
	 * Where the winch must be when scaled
	 */
	private final double WINCH_IN = 0;
	
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
		rightBottom = new CANTalon(0);
		rightHook = new DigitalInput(0);
		rightSolenoidFirst = new Solenoid(0);
		rightSolenoidSecond = new Solenoid(0);
		encoderRight = new Encoder(0,0); //TODO ask about the two values
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK_INCHES);
		wantedRightPower = 0;
		
		//Left
		leftFront = new CANTalon(0); 
		leftBack = new CANTalon(0);
		leftBottom = new CANTalon(0);
		leftSolenoidFirst = new Solenoid(0);
		leftSolenoidSecond = new Solenoid(0);
		leftHook = new DigitalInput(0);
		encoderLeft = new Encoder(0,0); //TODO ask about the two values
		encoderDataLeft = new EncoderData(encoderLeft,DISTANCE_PER_TICK_INCHES);
		wantedLeftPower = 0;
		
		//Other
		currentScalingState = State.STANDBY;
		drives = Drives.getInstance(); 
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
		switch(currentScalingState) {
		case STANDBY: 
		{
			setArms(ARMS_DOWN,true);	
			wantedRightPower = 0; //TODO figure out wanted value to work with pto 
			wantedLeftPower = 0;
			break;
		}
		case EXTEND_FULL:
		{
			encoderDataRight.reset();
			encoderDataLeft.reset();
			setArms(ARMS_UP,true);
			wantedRightPower = 0.5;
			wantedRightPower = 0.5;		
			currentScalingState = State.EXTENDING_FULL;
			break;
		}
		case EXTENDING_FULL:
		{
			if(encoderDataRight.getDistance() > DISTANCE_TO_BAR_INCHES)
			{
				wantedRightPower = 0;
			}
			if(encoderDataLeft.getDistance() > DISTANCE_TO_BAR_INCHES)
			{
				wantedLeftPower = 0;
			}
			if(encoderDataRight.getDistance() > DISTANCE_TO_BAR_INCHES && encoderDataLeft.getDistance() > DISTANCE_TO_BAR_INCHES)
			{
				currentScalingState = State.EXTENDED_FULL;
			}
			break;
		}
		case EXTENDED_FULL:
		{
			setArms(ARMS_UP,true);
			wantedRightPower = 0;
			wantedLeftPower = 0;
			break;
		}
		case SCALING:
		{
			if(rightHook.get() && leftHook.get())
			{
			setArms(ARMS_DOWN,true);
			wantedRightPower = -0.5;
			wantedLeftPower = -0.5;
			currentScalingState = State.SCALED;
			}
			break;
		}
		case EXTENDED_HALF:
			break;
		case EXTENDING_HALF:
			break;
		case EXTEND_HALF:
			break;
		case SCALED:
		{
			if(encoderDataRight.getDistance() < WINCH_IN)
			{
				wantedRightPower = 0;
			}
			if(encoderDataLeft.getDistance() < WINCH_IN)
			{
				wantedLeftPower = 0;
			}
			break;
		}
		case OVERRIDE:
			break;	
		}
		rightFront.set(wantedRightPower);
		rightBack.set(wantedRightPower);
		rightBottom.set(wantedRightPower);
		leftFront.set(wantedLeftPower);
		leftBack.set(wantedLeftPower);
		leftBottom.set(wantedLeftPower);
		return false;
	}
	
	/**
	 * @param solenoidValue is the value to send to both solenoids
	 * @param extendFull if true arms will fully extend, if false, arms will extend half way
	 */
	private void setArms(boolean solenoidValue,boolean extendFull)
	{
		if (rightSolenoidFirst.get() != solenoidValue)
		{
			rightSolenoidFirst.set(solenoidValue);
		}
		if (leftSolenoidFirst.get() != solenoidValue)
		{
			leftSolenoidFirst.set(solenoidValue);
		}
		if (extendFull)
		{
			if (rightSolenoidSecond.get() != solenoidValue)
			{
				rightSolenoidSecond.set(solenoidValue);
			}
			if (leftSolenoidSecond.get() != solenoidValue)
			{
				leftSolenoidSecond.set(solenoidValue);
			}
		}
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
		EXTEND_FULL,
		EXTENDING_FULL,
		EXTENDED_FULL,
		EXTEND_HALF,
		EXTENDING_HALF,
		EXTENDED_HALF,
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
			case EXTEND_FULL:
				return "Extend full";
			case EXTENDING_FULL:
				return "Extending full";
			case EXTENDED_FULL:
				return "Extended full";
			case EXTEND_HALF:
				return "Extend half";
			case EXTENDING_HALF:
				return "Extending half";
			case EXTENDED_HALF:
				return "Extended half";
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
