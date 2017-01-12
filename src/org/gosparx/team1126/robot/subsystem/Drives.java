package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;
import org.gosparx.team1126.robot.sensors.PID;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is intended to drive the robot in tank drive
 * @author Michael
 */
public class Drives extends GenericSubsystem{

	//*********************INSTANCES**********************

	/**
	 * Makes THE drives instance that will be called to use the drives
	 */
	private static Drives drives;
	private Controls control;

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

	/**
	 * the solenoid used to engage and disengage the pto
	 */
	//private Solenoid ptoSol;

	//*********************SENSORS************************

	/**
	 * 
	 */
	private DigitalInput leftA;

	/**
	 * 
	 */
	private DigitalInput leftB;

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
	 * gyro used to keep ourselves align and to turn in Auto
	 */
	private AnalogGyro angleGyro;
	
	private PID PIDRight;
	
	private PID PIDLeft;

	//*********************CONSTANTS**********************

	/**
	 * the amount of distance the robot will travel per tick 
	 * equation: Circumference/256(distance per tick)
	 */
	//private final double DISTANCE_PER_TICK = ((1/10)*Math.PI*6)/256;
	//private final double DISTANCE_PER_TICK = 0.007363108;
	private final double DISTANCE_PER_TICK = 0.00689;

	/**
	 * the speed required to shift down in inches per sec, not accurate yet
	 */
	private static final double LOWER_SHIFTING_SPEED = 20;

	/**
	 * the speed required to shift up in inches per sec, not accurate yet
	 */
	private static final double UPPER_SHIFTING_SPEED = 45;

	/**
	 * the time required to pause for shifting in seconds, not accurate yet, in seconds
	 */
	private static final double SHIFTING_TIME = 0.125;

	//*********************VARIABLES**********************
	
	private static double leftSpeed;
	private static double rightSpeed;
	private static double kp, ki;
	private static double rightSetPoint;
	private static double leftSetPoint;
	private static boolean driveReset;
	private static double initialHeading;
	
	
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
		kp = (1.0/50);
		ki = 0.005 * 50;
		//RIGHT
		rightFront = new CANTalon(IO.CAN_DRIVES_RIGHT_FRONT);
		rightBack = new CANTalon(IO.CAN_DRIVES_RIGHT_BACK);
		rightFront.setInverted(true);
		rightBack.setInverted(true);
		encoderRight = new Encoder(IO.DIO_RIGHT_DRIVES_ENC_A,IO.DIO_RIGHT_DRIVES_ENC_B);
		encoderDataRight = new EncoderData(encoderRight,DISTANCE_PER_TICK);
		
		//LEFT
		leftBack = new CANTalon(IO.CAN_DRIVES_LEFT_BACK);
		leftFront = new CANTalon(IO.CAN_DRIVES_LEFT_FRONT);
		leftA = new DigitalInput(IO.DIO_LEFT_DRIVES_ENC_A);
		leftB = new DigitalInput(IO.DIO_LEFT_DRIVES_ENC_B);
		encoderLeft = new Encoder(leftA, leftB);
		encoderDataLeft = new EncoderData(encoderLeft,-DISTANCE_PER_TICK);

		//OTHER
		angleGyro = new AnalogGyro(IO.ANALOG_IN_ANGLE_GYRO);
		angleGyro.calibrate();
		shiftingSol = new Solenoid(IO.PNU_SHIFTER);
		PIDRight = new PID(ki, kp);
		PIDRight.breakMode(true);
		PIDLeft = new PID(ki, kp);
		PIDLeft.breakMode(true);

		control = Controls.getInstance();
		return true;
	} 

	/**
	 * Used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemMotorName = "DrivesMotors";
		String subsystemSensorName = "DrivesSensors";
		LiveWindow.addSensor(subsystemSensorName, "RightEncoder", encoderRight);
		LiveWindow.addSensor(subsystemSensorName, "LeftEncoder", encoderLeft);
		LiveWindow.addSensor(subsystemSensorName, "angleGyro", angleGyro);
		LiveWindow.addActuator(subsystemMotorName, "Shifting", shiftingSol);
		//	LiveWindow.addActuator(subsystemMotorName, "ptoSol", ptoSol);
		LiveWindow.addActuator(subsystemMotorName, "RightFrontMotor", rightFront);
		LiveWindow.addActuator(subsystemMotorName, "RightBackMotor", rightBack);
		LiveWindow.addActuator(subsystemMotorName, "LeftFrontMotor", leftFront);
		LiveWindow.addActuator(subsystemMotorName, "LeftBackMotor", leftBack);
	}

	/**
	 * it runs on a loop until returned false, don't return true
	 * is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		double rValue, lValue;
		double correction;
		encoderDataLeft.calculateSpeed();
		encoderDataRight.calculateSpeed();
		leftSpeed = encoderDataLeft.getSpeed();
		rightSpeed = encoderDataRight.getSpeed();
		
		if (control.opJoy.joy.getRawButton(6))
		{
			rightSetPoint = 45.0;
			leftSetPoint = 45.0;
			LOG.logMessage("Left Speed: " + leftSpeed + " ");
			LOG.logMessage("Right Speed: " + rightSpeed + " ");
//			LOG.logMessage("PIDRight: " + rValue + " ");
//			LOG.logMessage("PIDLeft: " + lValue + " ");				
		}
		else if(control.opJoy.joy.getRawButton(8)){
			//driveDistance(50,36, driveReset);
			turn(90,10,driveReset);
//			LOG.logMessage("SP" + setPoint);
		}else{
			driveReset = true;
			rightSetPoint = 0;
			leftSetPoint = 0;
		}

		rValue = PIDRight.loop(rightSpeed, rightSetPoint);
		lValue = PIDLeft.loop(leftSpeed, leftSetPoint);
//		correction = .05 * (angleGyro.getAngle() - initialHeading);
//		if(rightSetPoint != 0){
//			rValue += correction;
//		}
//		if(leftSetPoint != 0){
//			lValue -= correction;
//		}
		rightFront.set(rValue);
		rightBack.set(rValue);
		leftFront.set(lValue);
		leftBack.set(lValue);

//			LOG.logMessage("Button Value: " + control.opJoy.joy.getRawButton(6) + " ");
		
		return false;
	}

	public boolean driveDistance(double dis, double speed, boolean reset){
		double avgDis = 0;
		if(reset){
			initialHeading = angleGyro.getAngle();
			encoderDataRight.reset();
			encoderDataLeft.reset();
			driveReset = false;
			angleGyro.reset();
//			LOG.logMessage("Reset");
			return false;
		}
//		LOG.logMessage("Main Loop");
		rightSetPoint = speed;
		leftSetPoint = speed;
		avgDis = (Math.abs(encoderDataRight.getDistance()) + Math.abs(encoderDataLeft.getDistance()))/2;
		if(avgDis >= Math.abs(dis - (.05*speed)) - .5){
			rightSetPoint = 0;
			leftSetPoint = 0;
			LOG.logMessage("Distance Traveled: " + avgDis);
			LOG.logMessage("Gryo Angle: " + angleGyro.getAngle());
			return true;
		}
		return false;
	}
	
	public boolean turn(double angle, double speed, boolean reset){
		double currentAngle = (angleGyro.getAngle())%360;
		double offset = angle-currentAngle;
		if(offset<Math.abs(-3)){
			return true;
		}
		if((offset>=0 && offset<=180) || (offset<=-180 && offset>=-360)){
			leftSetPoint = speed;
			rightSetPoint = -speed;
		}else{
			leftSetPoint = -speed;
			rightSetPoint = speed;
		}
		return false;
	}
		
	public void toggleSetPoint() {
		
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
		

	}

	/**
	 * is used to get the power from the joysticks 
	 * @param left the left 
	 * joystick input from -1 to 1
	 * @param right the right joystick input from -1 to 1
	 */
	public void setPower(double left, double right) {
		
	}
	
	

		
	
	/**
	 * drives the robot to a certain distance
	 * @param length: the length you want it to go
	 * @param speed: the speed you want it to go
	 */
	public void driveWantedDistance(double length){
		
	}

	/**
	 * Called to turn during autonomous
	 * @param angle the angle you want to be (negative for left turn, positive right turn)
	 */
	public void turn(double angle){
		
	}

	/**
	 * return true if the auto function finished
	 */
	public boolean autoFunctionDone(){
		return true;
	}

	/**
	 * stops everything in drives and puts auto in standby
	 */
	public void autoEStop(){
		
	}

	/**
	 * called to set the auto state to auto defense
	 */
	public void startAutoDef(){
	
	}


	/**
	 * if called, lets the driver manually shift
	 */
	public void driverShifting(){
	
	}

	/**
	 * called to manually shift up or down
	 */
	public void toggleShifting(){
	
	}

	/**
	 * If called, will either engage or disengage the pto depending on it's previous state, toggle on off
	 */
	public void manualPtoEngage(){
		
	}

	public void holdFirst(boolean newVal){

	}

	public void killAutoDrive(){
	}

	public void returnToZero(){
		
	
	}

}