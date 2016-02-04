package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.AbsoluteEncoderData;
import org.gosparx.team1126.robot.sensors.EncoderData;
import org.gosparx.team1126.robot.sensors.MagnetSensor;

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
	private static final double CENTERING_POWER = 0.2;

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

	/**
	 * The power to use when putting the ball in the flipper
	 */
	private static final double PUT_IN_FLIP_POWER = 0.3;
	
	/**
	 * The degrees from home the arm has to be at to hold the ball against the bumper
	 */
	private static final double HOLD_BUMPER_DEGREE = 0;
	
	/**
	 * The degrees from home the arm has to be catch the drawbridge
	 */
	private static final double CATCH_DRAW_DEGREE = 0;
	
	/**
	 * The degrees from home the arm has to be to put the ball in the flipper
	 */
	private static final double PUT_IN_FLIPPER_DEGREE = 0;
	
	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE = 0;
	 
	/**
	 * The degrees from home the arm has to be to get boulders from the flipper
	 */
	private static final double CATCH_BALL_DEGREE = 0;
	
	/**
	 * The degrees the arm can be off by and still be considered in a certain spot
	 */
	private static final double DEADBAND = 0.5;
	
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
	private EncoderData rollerEncoderData;

	/**
	 * Magnetic sensor for the arm's home position
	 */
	private MagnetSensor armHomeSwitch;

	/**
	 * the photo electric sensor to see if the ball is in
	 */
	private DigitalInput ballEntered;

	/**
	 * the photo electric sensor to see if the ball is fully in the robot.
	 */
	private DigitalInput ballFullyIn;

	/**
	 * the limit switch to make sure the arm doesn't  run into the robot
	 */
	private DigitalInput armLimit;

	//*****************************Variables*******************************************

	/**
	 * the current state of the arm
	 */
	private ArmState currentArmState;

	/**
	 * the current state of the flipper
	 */
	private FlipperState currentFlipperState;

	/**
	 * the current state of the roller
	 */
	private RollerState currentRollerState;

	/**
	 * the current power of the roller
	 */
	private RollerPower currentRollerPower;

	/**
	 * the wanted angle of the arm
	 */
	private double wantedArmAngle;

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

	/**
	 * The wanted power of the arm motor
	 */
	private double wantedArmPower;

	/**
	 * Whether the arms are in their home position
	 */
	private boolean armHome;

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
	@Override
	protected boolean init() {
		armMotor = new CANTalon(IO.CAN_ACQ_SHOULDER);
		rollerMotorR = new CANTalon(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorL = new CANTalon(IO.CAN_ACQ_ROLLERS_L);
		armEncoder = new AbsoluteEncoderData(IO.ANALOG_IN_ABS_ENC_SHOULDER_L, DEGREE_PER_VOLT);
		rollerEncoder = new Encoder(IO.DIO_ROLLER_ENC_A,IO.DIO_ROLLER_ENC_B);
		rollerEncoderData = new EncoderData(rollerEncoder, DISTANCE_PER_TICK_ROLLER);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivot = new Solenoid(IO.PNU_CIRCLE_POS);
		currentArmState = ArmState.STANDBY;
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		armHomeSwitch = new MagnetSensor(IO.DIO_ACQ_SHOULDER_HOME, false);
		ballEntered = new DigitalInput(IO.DIO_BALL_ENTERED);
		ballFullyIn = new DigitalInput(IO.DIO_BALL_COMPLETELY_IN);
		armLimit = new DigitalInput(7);
		wantedArmAngle = 0;
		timeFired = 0;
		firing = false;
		timeCentered = 0;
		centering = false;
		rollerOn = false;
		wantedPowerRR = 0;
		wantedPowerRL = 0;
		currentRollerPower = RollerPower.STANDBY;
		armHome = false;
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
		armHome = armHomeSwitch.isTripped();
		switch(currentArmState){
		case STANDBY:
			wantedArmPower = 0;
			break;
		case ROTATING:
			if(wantedArmAngle == GATE_POSITION_DEGREE){
				if(armEncoder.getDegrees() > GATE_POSITION_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < GATE_POSITION_DEGREE + DEADBAND){
					//do later
				}else{
					//do later
				}
			}else if(wantedArmAngle == HOLD_BUMPER_DEGREE){
				if(armEncoder.getDegrees() > HOLD_BUMPER_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < HOLD_BUMPER_DEGREE + DEADBAND){
					//do later
				}else{
					//do later
				}
			}else if (wantedArmAngle == CATCH_DRAW_DEGREE){
				if(armEncoder.getDegrees() > CATCH_DRAW_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < CATCH_DRAW_DEGREE + DEADBAND){
					//do later
				}else{
					//do later
				}
			}else if (wantedArmAngle == CATCH_BALL_DEGREE){
				if(armEncoder.getDegrees() > CATCH_BALL_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < CATCH_BALL_DEGREE + DEADBAND){
					//do later
				}else{
					//do later
				}
			}else{
				//do later
			}
			break;
		case ROTATE_FINDING_HOME:
			if(armHome){
				LOG.logMessage("Arm is home");
				armEncoder.reset();
				currentArmState = ArmState.STANDBY;
			}else{
				wantedArmPower = -0.3;
			}
			break;
		case PUT_BALL_IN_FLIPPER:
			if(ballEntered.get()){
				wantedArmPower = PUT_IN_FLIP_POWER;
			}
			if(ballFullyIn.get()){
				wantedArmPower = 0;
				currentArmState = ArmState.ROTATE_FINDING_HOME;
			}
			break;
		case OP_CONTROL:
			//FIXME:: stop the other way??
			if(armLimit.get()){
				wantedArmPower = 0;
				currentArmState = ArmState.STANDBY;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}




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
				currentFlipperState = FlipperState.STANDBY;
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
				currentRollerState = RollerState.STANDBY;
				centering = false;
			}
			break;
		case ROLLER_ON:
			if( currentRollerPower == RollerPower.DROP){
				wantedPowerRR = DROP_POWER;
				wantedPowerRL = DROP_POWER;
			}
			if(currentRollerPower == RollerPower.KICK){
				wantedPowerRR = KICK_POWER;
				wantedPowerRL = KICK_POWER;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentRollerState);
			break;
		}
		rollerMotorR.set(wantedPowerRR);
		rollerMotorL.set(wantedPowerRL);
		armMotor.set(wantedArmPower);
		return false;
	}
	
	/**
	 * sets the home position
	 */
	private void setHome(){
		currentArmState = ArmState.ROTATE_FINDING_HOME;
	}
	
	/**
	 * sets the power based off the controller
	 * @param pow the controller input
	 */
	private void setPower(double pow){
		if(pow == 0)
			currentArmState = ArmState.STANDBY;
		else{ 
			currentArmState = ArmState.OP_CONTROL;
			wantedArmPower = pow;
		}
	}
	
	/**
	 * moves the arms to bring the ball against the bumper
	 */
	private void moveToBumper(){
		wantedArmAngle = HOLD_BUMPER_DEGREE;
		currentArmState = ArmState.ROTATING;
	}
	
	/**
	 * moves the arms to catch the drawbridge
	 */
	 private void catchDrawbridge(){
		 
	 }
	 
	 /**
	  * moves the arms to put the ball in the flipper 
	  */
	 private void putBallInFlipper(){
		 wantedArmAngle = PUT_IN_FLIPPER_DEGREE;
		 currentArmState = ArmState.PUT_BALL_IN_FLIPPER;
	 }
	 
	 /**
	  * moves the ball to raise the gate
	  */
	 private void raiseGate(){
		 wantedArmAngle = GATE_POSITION_DEGREE;
		 currentArmState = ArmState.ROTATING;
	 }
	 
	 /**
	  * moves the ball to the position to catch the ball from the flipper
	  */
	 private void catchBall(){
		 wantedArmAngle = CATCH_BALL_DEGREE;
		 currentArmState = ArmState.ROTATING;
	 }

	/**
	 * fires the flipper
	 * @return true if the flipper fires and false if the flipper is already firing
	 */
	private boolean fire(){
		if(currentFlipperState == FlipperState.FIRING)
			return false;
		else{
			currentFlipperState = FlipperState.FIRING;
			return true;
		}
	}

	/**
	 * Toggles roller.
	 * @return true if the roller is on, false if the roller is off
	 */
	private boolean toggleRoller(){
		if(rollerOn){
			rollerOn = false;
			currentRollerState = RollerState.STANDBY;
			return false;
		}else{
			rollerOn = true;
			currentRollerState = RollerState.ROLLER_ON;
			return true;
		}
	}

	/**
	 * Reverses roller.
	 * @return true if the roller is forward, false if the roller is backwards
	 */
	private boolean reverseRoller(){
		wantedPowerRR*=-1;
		wantedPowerRL*=-1;
		currentRollerState = RollerState.ROLLER_ON;
		if(rollerEncoderData.getSpeed() < 0){
			return false;
		}else
			return true;
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
		LOG.logMessage("Current roller state: " + currentRollerState);
		LOG.logMessage("Current flipper state: " + currentFlipperState);
	}

	/**
	 * the states for the arm 
	 */
	public enum ArmState{
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
	public enum RollerState{
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
	public enum FlipperState{
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
	public enum RollerPower{
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
