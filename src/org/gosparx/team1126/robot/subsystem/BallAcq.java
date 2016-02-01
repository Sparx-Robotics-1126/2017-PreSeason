package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.AbsoluteEncoderData;
import org.gosparx.team1126.robot.sensors.EncoderData;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Purpose: to acquire/get the balls and then score them
 * @author Allison and Jack
 */

public class BallAcq extends GenericSubsystem{

//*****************************Constants*******************************************	
	
	/**
	 * the distance the arm will travel per tick
	 */
	private static final double DEGREE_PER_VOLT = 0;
	
	/**
	 * the distance the roller will travel per tick
	 */
	private static final double DISTANCE_PER_TICK_ROLLER = 0;
	
	/**
	 * The amount of time we want the flipper to stay up after firing (in seconds)
	 */
	private static final double WAIT_FIRE_TIME = 0.25;
	
	/**
	 * The power of the roller motor while centering the ball
	 */
	private static final double CENTERING_POWER = .2;
	
	/**
	 * The amount of time we want the roller to run while centering the ball (in seconds) 
	 */
	private static final double WAIT_CENTERING_TIME = 0.75;
	
	/**
	 * The power to use when kicking the ball out of the robot
	 */
	private static final double KICK_POWER = 0.8;
	
	/**
	 * The power to use when dropping the ball to a teammate
	 */
	private static final double DROP_POWER = 0.3;
	
//*****************************Objects*********************************************
	
	/**
	 * creates an instance of BallAcq
	 */
	private static BallAcq acq;
	
	/**
	 * The motor that rotates the arm
	 */
	private CANTalon armMotor;
	
	/**
	 * the motor that rotates the right roller motor
	 */
	private CANTalon rollerMotorR;
	
	/**
	 * the motor that rotates the left roller motor
	 */
	private CANTalon rollerMotorL;
	
	/**
	 * the encoder that tracks the motion of the arm
	 */
	private AbsoluteEncoderData armEncoder;
	
	/**
	 * the encoder that tracks the motion of the motor on the roller
	 */
	private Encoder rollerEncoder;
	
	/**
	 * the encoder data for the roller encoder
	 */
	private EncoderData rEncoderData;
	
	/**
	 * Magnetic sensor for the arm's home position
	 */
	private DigitalInput armHome;

//*****************************Variables*******************************************
	
	/**
	 * the current state of the arm
	 */
	private armState currentArmState;
	
	/**
	 * the current state of the flipper
	 */
	private flipperState currentFlipperState;
	
	/**
	 * the current state of the roller
	 */
	private rollerState currentRollerState;
	
	/**
	 * the current power of the roller
	 */
	private rollerPower currentRollerPower;
	
	/**
	 * the wanted angle of the blue wheel
	 */
	// FIXME:: This may become constants
	private double wantedBlueAngle;
	
	/**
	 * the wanted angle of the arm
	 */
	// FIXME:: This may become constants
	private double wantedArmAngle;
	
	/**
	 * the average speed of the right and left motors on the blue wheel
	 */
	// FIXME:: This may become constants
	private double currentBlueSpeed;
	
	/**
	 * the average speed of the right and left motors on the arms
	 */
	// FIXME:: This may become constants
	private double currentArmSpeed;
	
	/**
	 * the pnu that controls the flipper
	 */
	private Solenoid flipper; 
	
	/**
	 * the pnu that controls the circular pivot 
	 */
	private Solenoid circPivot;
	
	/**
	 * the time we fired the ball during the match (in seconds)
	 */
	private double timeFired;
	
	/**
	 * Whether we are firing or not
	 */
	private boolean firing;
	
	/**
	 * the time we started centering the ball (in seconds)
	 */
	private double timeCentered;
	
	/**
	 * Are we centering the boulder?
	 */
	private boolean centering;
	
	/**
	 * Is the roller on?
	 */
	private boolean rollerOn;
	
	/**
	 * The wanted power of the right roller motor.
	 */
	private double wantedPowerRR;
	
	/**
	 * The wanted power of the left roller motor
	 */
	private double wantedPowerRL;
	
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
	private BallAcq() {
		super("BallAcq", Thread.NORM_PRIORITY);
	}

	/**
	 * instantiates objects and initializes variables
	 */
	// FIXME:: Use the IO even if it is wrong
	@Override
	protected boolean init() {
		armMotor = new CANTalon(IO.CAN_ACQ_SHOULDER);
		rollerMotorR = new CANTalon(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorL = new CANTalon(IO.CAN_ACQ_ROLLERS_L);
		armEncoder = new AbsoluteEncoderData(IO.ANALOG_IN_ABS_ENC_SHOULDER_L, DEGREE_PER_VOLT);
		rollerEncoder = new Encoder(IO.DIO_ROLLER_ENC_A,IO.DIO_ROLLER_ENC_B);
		rEncoderData = new EncoderData(rollerEncoder, DISTANCE_PER_TICK_ROLLER);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivot = new Solenoid(IO.PNU_CIRCLE_POS);
		currentArmState = armState.STANDBY;
		currentFlipperState = flipperState.STANDBY;
		currentRollerState = rollerState.STANDBY;
		//FIXME:: DO LATER
		wantedBlueAngle = 0;
		wantedArmAngle = 0;
		currentBlueSpeed = 0;
		currentArmSpeed = 0;
		timeFired = 0;
		firing = false;
		timeCentered = 0;
		centering = false;
		rollerOn = false;
		wantedPowerRR = 0;
		wantedPowerRL = 0;
		currentRollerPower = rollerPower.STANDBY;
		return false;
	}

	/**
	 * to add data to objects while in test mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "BallAcq";
		LiveWindow.addActuator(subsystemName, "Arm Motor", armMotor);
		LiveWindow.addActuator(subsystemName, "Right Roller Motor", rollerMotorR);
		LiveWindow.addActuator(subsystemName, "Left Roller Motor", rollerMotorL);
		//LiveWindow.addActuator(subsystemName, "Arm Absolute Encoder", armEncoder);
		LiveWindow.addActuator(subsystemName, "Roller Encoder", rollerEncoder);
		LiveWindow.addActuator(subsystemName, "Flipper", flipper);
		LiveWindow.addActuator(subsystemName, "Circular Pivot", circPivot);
	}

	/**
	 * Acquires the ball and then scores/passes the ball
	 * @return false to continue loop
	 */
	@Override
	protected boolean execute() {
		switch(currentFlipperState){
		case STANDBY:
			break;
		case FIRING:
			if(!firing){
				flipper.set(true);
				timeFired = Timer.getFPGATimestamp();
				firing = true;
			}
			if(Timer.getFPGATimestamp() >= timeFired + WAIT_FIRE_TIME && firing){
				flipper.set(false);
				firing = false;
				currentFlipperState = flipperState.STANDBY;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentFlipperState);
			break;
		}
		switch(currentRollerState){
		case STANDBY:
			wantedPowerRR = 0;
			wantedPowerRL = 0;
			break;
		case CENTERING:
			if(!centering){
				wantedPowerRR = CENTERING_POWER;
				wantedPowerRL = CENTERING_POWER;
				timeCentered = Timer.getFPGATimestamp();
				centering = true;
			}
			if(Timer.getFPGATimestamp() >= timeCentered + WAIT_CENTERING_TIME && centering){
				wantedPowerRR = 0;
				wantedPowerRL = 0;
				currentRollerState = rollerState.STANDBY;
				centering = false;
			}
			break;
		case ROLLER_ON:
			if( currentRollerPower == rollerPower.DROP){
				wantedPowerRR = DROP_POWER;
				wantedPowerRL = DROP_POWER;
			}
			if(currentRollerPower == rollerPower.KICK){
				wantedPowerRR = KICK_POWER;
				wantedPowerRL = KICK_POWER;
			}
			break;
		}
		rollerMotorR.set(wantedPowerRR);
		rollerMotorL.set(wantedPowerRL);
		return false;
	}
	
	/**
	 * fires the flipper
	 */
	private void fire(){
		currentFlipperState = flipperState.FIRING;
	}
	
	/**
	 * Toggles roller.
	 */
	private void toggleRoller(){
		if(rollerOn){
			rollerOn = false;
			currentRollerState = rollerState.STANDBY;
		}else{
			rollerOn = true;
			currentRollerState = rollerState.ROLLER_ON;
		}
		
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
	 * writes a log to the console every 5 seconds
	 */
	@Override
	protected void writeLog() {
		LOG.logMessage("Current arm state: " + currentArmState);
	}
	
	/**
	 * the states for the arm 
	 */
	public enum armState{
		STANDBY,
		ROTATING,
		ROTATE_FINDING_HOME,
		PUT_BALL_IN_FLIPPER,
		OP_CONTROL;
	
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
				return "Finding Home";
			case PUT_BALL_IN_FLIPPER:
				return "Placing ball in flipper from floor";
			case OP_CONTROL:
				return "Rachel is in control";
			default:
				return "Error :(";
			}
		}
	}
	
	/**
	 * the states for the roller
	 */
	public enum rollerState{
		STANDBY,
		CENTERING,
		ROLLER_ON;
		
		/**
		 * Gets the state name
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "In Standby";
			case CENTERING:
				return "Centering Ball";
			case ROLLER_ON:
				return "Roller is on";
			default:
				return "Error :(";
				
			}
		}
	}
	
	/**
	 * the states for the flipper
	 */
	public enum flipperState{
		STANDBY,
		FIRING;
		
		/**
		 * Gets the state name
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "In Standby";
			case FIRING:
				return "Firing";
			default:
				return "Error :(";
			}
		}
	}
	
	/**
	 * the possible powers for the roller
	 */
	public enum rollerPower{
		STANDBY,
		DROP,
		KICK;
		
		/**
	 	* Gets the current power
	 	* @return the name of the current speed
	 	*/
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "In Standby";
			case DROP:
				return "Dropping ball";
			case KICK:
				return "Kicking ball";
			default:
				return "Error :(";
			}
		}
	}
}
