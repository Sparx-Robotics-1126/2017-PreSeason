package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.sensors.EncoderData;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


/**
 * This class is intended to drive the robot in tank drive
 * @author Meekly
 */
public class Drives extends GenericSubsystem{
	
	int currentTime;

	//*********************INSTANCES**********************

	/**
	 * Makes THE drives instance that will be called to use the drives
	 */
	private static Drives drives;


	//*********************MOTOR CONTROLLERS**************

	/**
	 * the controller to the right front motor
	 */
	private CANTalon rightFront;

	/**
	 * the controller to the right back motor
	 */
	private CANTalon rightBack;

	/**
	 * the controller to the left front motor
	 */
	private CANTalon leftFront;

	/**
	 * the controller to the left back motor
	 */
	private CANTalon leftBack;

	//*********************PNEUMATICS****************************

	/**
	 * the solenoid to shift between high and low gear
	 */
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

	//*********************CONSTANTS**********************

	/**
	 * the amount of distance the shortbot will make per tick 
	 * equation: Circumference/256(distance per tick)
	 */
	private final double DISTANCE_PER_TICK = 0.0490873852;

	/**
	 * the speed required to shift down, not accurate yet
	 */
	private static final double LOWER_SHIFTING_SPEED = 15;

	/**
	 * the speed required to shift up, not accurate yet
	 */
	private static final double UPPER_SHIFTING_SPEED = 40;

	/**
	 * the time required to shift, not accurate yet, in seconds
	 */
	private static final double SHIFTING_TIME = 0.25;

	/**
	 * the speed required to shift
	 */
	private static final double SHIFTING_SPEED = 0.7;

	/**
	 * determines if it's in high or low gear
	 */
	private static final boolean LOW_GEAR = false;

	/**
	 * The value of 0 which will stop the motor
	 */
	private static final double STOP_MOTOR = 0;


	//*********************VARIABLES**********************

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
	 * The speed of which you want to go during autonomous
	 */
	private double wantedAutoSpeed;

	/**
	 * The current distance traveled in autonomous
	 */
	private double currentAutoDist;

	/**
	 * The current state that autonomous is in
	 */
	private State autoState;
	
	/**
	 * 
	 */
	private double distanceLeft;
	
	/**
	 * 
	 */
	private double distanceRight;
	
	/**
	 * 
	 */
	private double wantedLeftPower;
	
	/**
	 * 
	 */
	private double wantedRightPower;
	
	/**
	 * 
	 */
	private double leftCurrentAutoSpeed;
	
	/**
	 * 
	 */
	private double rightCurrentAutoSpeed;
	

	/**
	 * Creates a drives with normal priority
	 */
	private Drives(){
		super("Drives", Thread.NORM_PRIORITY);
	}

	/**
	 * This creates a drives object with a name and its priority
	 */
	public static synchronized Drives getInstance() {
		if(drives == null){
			drives = new Drives();
		}
		return drives;
	}

	/**
	 * Instantiates all of the objects and gives data to the variables
	 * return: return true it runs once, false continues, should be true
	 */
	@Override
	protected boolean init() {

		//RIGHT
		rightFront = new CANTalon(1);
		//rightBack = new Talon(1);
		encoderRight = new Encoder(1,0);
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK);


		//LEFT
		//leftBack = new Talon(1);
		leftFront = new CANTalon(0);
		encoderLeft = new Encoder(3,2);
		encoderDataLeft = new EncoderData(encoderLeft,DISTANCE_PER_TICK);

		//OTHER

		wantedLeftPower = 0;
		wantedRightPower = 0;
		//gyro = new Gyro(1);
		currentDriveState = State.IN_LOW_GEAR;
		shiftingSol = new Solenoid(0);
		wantedAutoDist = 60;
		autoState = State.AUTO_DRIVE;

		return true;
	}

	/**
	 * Used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		String subsytemName = "Drives";
		LiveWindow.addActuator(subsytemName, "Shifting", shiftingSol);
		LiveWindow.addActuator(subsytemName, "Right Encoder", encoderRight);
		LiveWindow.addActuator(subsytemName, "Right Front Motor", rightFront);
		//LiveWindow.addActuator(subsytemName, "Right Rear Motor", rightBack);
		LiveWindow.addActuator(subsytemName, "Left Front Motor", leftFront);
		//LiveWindow.addActuator(subsytemName, "Left Front Motor", leftBack);
		LiveWindow.addActuator(subsytemName, "Left Encoder", encoderLeft);
	}

	/**
	 * it runs on a loop until returned false, don't return false
	 * is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		encoderDataLeft.calculateSpeed();
		encoderDataRight.calculateSpeed();
		currentSpeed = ((Math.abs(encoderDataLeft.getSpeed()) + Math.abs(encoderDataRight.getSpeed()))/2);
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
			break;
		default: System.out.println("Error, current drives state is: " + currentDriveState);
		}
		switch(autoState){
		case AUTO_STANDBY:
			break;
		case AUTO_DRIVE:
			distanceLeft = -1 * encoderDataLeft.getDistance();
			distanceRight = encoderDataRight.getDistance();
			currentAutoDist = ((distanceLeft + distanceRight)/2);
			wantedAutoSpeed = (.85/10)*(Math.sqrt(Math.abs(wantedAutoDist - currentAutoDist)));
			wantedAutoSpeed = wantedAutoSpeed < Math.PI/18 ? Math.PI/16: wantedAutoSpeed;
			leftCurrentAutoSpeed = encoderDataLeft.getSpeed();
			rightCurrentAutoSpeed = encoderDataLeft.getSpeed();
			if(Math.abs(leftCurrentAutoSpeed-rightCurrentAutoSpeed)<.4){
				wantedLeftPower = wantedAutoSpeed;
				wantedRightPower = wantedAutoSpeed;
			}else if(leftCurrentAutoSpeed > rightCurrentAutoSpeed){
				wantedLeftPower = wantedAutoSpeed * (9/8) < 1 ? wantedAutoSpeed *(9/8): 1;
				wantedRightPower = wantedAutoSpeed;
			}else {
				wantedRightPower = wantedAutoSpeed * (9/8) < 1 ? wantedAutoSpeed *(9/8): 1;
				wantedLeftPower = wantedAutoSpeed;
			}
			
			if(wantedAutoDist > 0){
				wantedLeftPower = wantedAutoSpeed;
				wantedRightPower =-wantedAutoSpeed;
			}else{
				wantedLeftPower = -wantedAutoSpeed;
				wantedRightPower = wantedAutoSpeed;
			}
			if(Math.abs(currentAutoDist) >= wantedAutoDist){
				wantedLeftPower = STOP_MOTOR;
				wantedRightPower = STOP_MOTOR;
				encoderDataLeft.reset();
				encoderDataRight.reset();
				autoState = State.AUTO_STANDBY;
				System.out.println("WE'RE DONE I HOPE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}
			break;
		default: System.out.println("Error, auto state is: " + autoState);
		}

		leftFront.set(wantedLeftPower);
		//leftBack.set(leftPower);
		rightFront.set(wantedRightPower);
		//rightBack.set(rightPower);

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
				System.out.println("The speeds are (left, right): " + encoderDataLeft.getSpeed() +", " + encoderDataRight.getSpeed());
				System.out.println("We are currently in this state-------- " + currentDriveState);
				System.out.println("We have gone this far!! " + (encoderDataLeft.getDistance() + encoderDataRight.getDistance())/2);
				System.out.println("The current auto distance left is " + (wantedAutoDist - currentAutoDist));
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
		SHIFTING_HIGH,
		AUTO_DRIVE,
		AUTO_STANDBY;

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
			case AUTO_DRIVE:
				return "In Auto Drive";
			case AUTO_STANDBY:
				return "In Auto Standby";
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
	public void driveWantedDistance(double length){
		wantedAutoDist = length;
		autoState = State.AUTO_DRIVE;
	}

}