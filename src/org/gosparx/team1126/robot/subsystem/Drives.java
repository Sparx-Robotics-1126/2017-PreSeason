package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.sensors.EncoderData;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * This class is intended to drive the robot
 * @author Meekly
 */
public class Drives extends GenericSubsystem{
	
	//*********************INSTANCES**********************
	
	/**
	 * Makes a drives object that will be called to use the drives class
	 */
	private static Drives drives;
	
	
	//*********************MOTOR CONTROLLERS**************
	
	/**
	 * the controller to the right front motor
	 */
	private Talon rightFront;
	
	/**
	 * the controller to the right back motor
	 */
	private Talon rightBack;
	
	/**
	 * the controller to the left front motor
	 */
	private Talon leftFront;
	
	/**
	 * the controller to the left back motor
	 */
	private Talon leftBack;
	
	//*********************PNU****************************
	
	private Solenoid shiftingSol;
	
	//*********************SENSORS************************
	
	/**
	 * Used to get the distance the robot has traveled for the left drives 
	 */
	private Encoder encoderLeft;

	/**
	 * Used to get the distance the robot has traveled for the right drives
	 */
	private Encoder encoderRight;

	/**
	 * makes the left encoder data which calculates how far the robot traveled in inches
	 */
	private EncoderData encoderDataLeft;

	/**
	 * makes the right encoder data which calculates how far the robot traveled in inches
	 */
	private EncoderData encoderDataRight;
	
	/**
	 * Allows us to know the direction of the robot
	 */
	private Gyro gyro;

	
	//*********************CONSTANTS**********************
	
	/**
	 * the amount of distance the shortbot will make per tick
	 */
	private final double DISTANCE_PER_TICK = 0.04908738;

	/**
	 * the speed required to shift down, not accurate yet
	 */
	private static final double LOWER_SHIFTING_SPEED = 60;

	/**
	 * the speed required to shift up, not accurate yet
	 */
	private static final double UPPER_SHIFTING_SPEED = 80;

	/**
	 * the time required to shift, not accurate yet, in seconds
	 */
	private static final double SHIFTING_TIME = 0.15;

	/**
	 * the speed required to shift
	 */
	private static final double SHIFTING_SPEED = 0.35;
	
	/**
	 * determines if it's in high or low gear
	 */
	private static final boolean LOW_GEAR = false;

	
	//*********************VARIABLES**********************
	
	/**
	 * the wanted speed for the left motors
	 */
	private double wantedLeftPower;

	/**
	 * the wanted speed for the right motors
	 */
	private double wantedRightPower;

	/**
	 * the current average speed between the left and right motors
	 */
	private double currentSpeed;

	/**
	 * actual time it took to shift
	 */
	private double shiftTime;
	
	/**
	 * The current state that drives is in
	 */
	private State currentDriveState;
	
	/**
	 * the wanted distance to travel during autonomous
	 */
	private double wantedAutoDist;
	
	/**
	 * This creates a drives object with a name and its priority
	 */
	private Drives(String name, int priority) {
		super(name, priority);
	}

	/**
	 * Instantiates all of the objects and gives data to the variables
	 * return: return true it runs once, false continues, should be true
	 */
	@Override
	protected boolean init() {
		
		//RIGHT
		rightFront = new Talon(5);
		//rightBack = new Talon(1);
		encoderRight = new Encoder(0,1);
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK);
		
		
		//LEFT
		//leftBack = new Talon(1);
		leftFront = new Talon(1);
		encoderLeft = new Encoder(2,3);
		encoderDataLeft = new EncoderData(encoderLeft,DISTANCE_PER_TICK);
		
		//OTHER
		
		wantedLeftPower = 0;
		wantedRightPower = 0;
		//gyro = new Gyro(1);
		currentDriveState = State.IN_LOW_GEAR;
		shiftingSol = new Solenoid(0);
		wantedAutoDist = 0;
		
		return true;
	}
	
	/**
	 * Used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		
	}
	
	/**
	 * it runs on a loop until returned false, don't return false
	 * is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		encoderDataLeft.calculateSpeed();
		encoderDataRight.calculateSpeed();
		currentSpeed = ((encoderDataLeft.getSpeed() + encoderDataRight.getSpeed())/2);
		switch(currentDriveState){
		
		case IN_LOW_GEAR:
			if(Math.abs(currentSpeed)>= UPPER_SHIFTING_SPEED){
				System.out.println("SHIFTING HIGH!");
				shiftingSol.set(LOW_GEAR);
				shiftTime = Timer.getFPGATimestamp();
				currentDriveState = State.SHIFTING_HIGH;
				if(currentSpeed < 0){
					wantedLeftPower = (SHIFTING_SPEED * -1);
					wantedRightPower = (SHIFTING_SPEED * -1);
				}else{
					wantedLeftPower = (SHIFTING_SPEED);
					wantedRightPower =(SHIFTING_SPEED);
				}
			}
			break;
			
		case SHIFTING_HIGH:
			if(Timer.getFPGATimestamp() >= shiftTime + SHIFTING_TIME){
				currentDriveState = State.IN_HIGH_GEAR;
			}
			
			break;
			
		case IN_HIGH_GEAR:
			if(Math.abs(currentSpeed) <= LOWER_SHIFTING_SPEED){
				System.out.println("SHIFTING LOW!");
				shiftingSol.set(!LOW_GEAR);
				shiftTime = Timer.getFPGATimestamp();
				currentDriveState = State.SHIFTING_LOW;
				if(currentSpeed < 0){
					wantedLeftPower = (SHIFTING_SPEED * -1);
					wantedRightPower = (SHIFTING_SPEED * -1);
				}else{
					wantedLeftPower = (SHIFTING_SPEED);
					wantedRightPower = (SHIFTING_SPEED);
				}
			}
			
			break;
			
		case SHIFTING_LOW:
			if(Timer.getFPGATimestamp() >= shiftTime + SHIFTING_SPEED){
				 currentDriveState = State.IN_LOW_GEAR;
			}
		}
		
		leftFront.set(wantedLeftPower);
		leftBack.set(wantedLeftPower);
		rightFront.set(wantedRightPower);
		rightBack.set(wantedRightPower);
		
		return false;
	}

	/**
	 * how long the class "rests" until it is called again
	 * return: how long it rests in milliseconds
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}
	
	/**
	 * writes a log to the console every 5 seconds
	 */
	@Override
	protected void writeLog() {
		System.out.println("The wanted powers are (left, right): " + wantedLeftPower + ", " + wantedRightPower);
		System.out.println("The speeds are (left, right): " + encoderDataLeft +", " + encoderDataRight);
	}
	
	/**
	 * is used to get the power from the joysticks 
	 * @param left the left joystick input from -1 to 1
	 * @param right the right joystick input from -1 to 1
	 */
	public void setPower(double left, double right) {
		wantedLeftPower = left;
		wantedRightPower = right;
	}
	/**
	 *Makes the states for drives
	 */
	public enum State{
		IN_LOW_GEAR,
		SHIFTING_LOW,
		IN_HIGH_GEAR,
		SHIFTING_HIGH;

		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case IN_LOW_GEAR:
				return "In low gear";
			case SHIFTING_LOW:
				return "Shifting Low";
			case IN_HIGH_GEAR:
				return "In high gear";
			case SHIFTING_HIGH:
				return "Shifting high";
			default:
				return "Error :(";
			}
		}
	}
	
	/**
	 * drives the robot to a certain distance
	 * @param length: the length you want it to go
	 * @param speed: the speed you want it to go
	 */
	public void driveWantedDistance(double length, double speed){
		leftFront.set(speed);
		rightFront.set(speed);
		while(((encoderDataLeft.getDistance() + encoderDataRight.getDistance())/2) < length){
		}
		leftFront.stopMotor();
		rightFront.stopMotor();
	}

}
