package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.AbsoluteEncoderData;
import org.gosparx.team1126.robot.sensors.MagnetSensor;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Purpose: to acquire/get the balls and then score them or pass them to teammates.
 * @author Allison and Jack
 */

public class BallAcq extends GenericSubsystem{

	//*****************************Constants*******************************************	

	/**
	 * the maximun angle for the arms
	 */
	private static final double MAX_ANGLE = 125;

	/**
	 * the distance the arm will travel per tick
	 */
	// We need to know real value.  The current design (which is not ideal).
	// Encoder mounted in 12 gear to 40
	private static final double DEGREE_PER_VOLT = 12/40;

	/**
	 * The amount of time we want the flipper to stay up after firing (in seconds)
	 */
	private static final double WAIT_FIRE_TIME = 0.25;

	/**
	 * The amount of time we want the roller to run while centering the ball (in seconds) 
	 */
	private static final double WAIT_CENTERING_TIME = 0.6;

	/**
	 * The power to use when kicking the ball out of the robot
	 */
	private static final double HIGH_ROLLER_POWER = 0.9;

	/**
	 * The power to use when dropping the ball to a teammate
	 */
	private static final double LOW_ROLLER_POWER = 0.1;

	/**
	 * The power to use when putting the ball in the flipper
	 */
	//probably different
	private static final double PUT_IN_FLIP_POWER = 0.3;

	/**
	 * The degrees from home the arm has to be at to hold the ball against the bumper
	 */
	private static final double HOLD_BUMPER_DEGREE = 65;

	/**
	 * The degrees from home the arm has to be catch the drawbridge
	 */
	//FIXME:: We don't know what it is-true
	private static final double CATCH_DRAW_DEGREE = 0;

	/**
	 * The degrees from home the arm has to be to put the ball in the flipper
	 */
	private static final double PUT_IN_FLIPPER_DEGREE = 50;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE = 120;

	/**
	 * The degrees from home the arm has to be to get boulders from the flipper
	 */
	private static final double ARM_FIRE_DEGREE = 30;

	/**
	 * the degree we need to the arms to be at to acquire the ball
	 */
	private static final double ACQUIRE_BALL_DEGREE = 90;

	/**
	 * the degree we need the arms in to score
	 */
	private static final double SCORE_BALL_DEGREE = 100;
	
	/**
	 * the floor degree
	 */
	//this is not the real value
	private static final double FLOOR_POSITION_DEGREE = 125;

	/**
	 * The degrees the arm can be off by and still be considered in a certain spot
	 */
	private static final double DEADBAND = 0.5;

	/**
	 * The power that the arms need to go home
	 */
	private static final double GOING_HOME_POWER = -0.3;

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
	 * the current state of lifting the ball
	 */
	private BallLiftSate currentLiftState;

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
	private Solenoid circPivotA;

	/**
	 * the pnu that controls the circular pivot as well
	 */
	private Solenoid circPivotB;

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

	/**
	 * the general wanted power for both the rollers
	 */
	private double wantedRollerPower;

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
		armEncoder = new AbsoluteEncoderData(IO.ANALOG_IN_ABS_ENC_SHOULDER, DEGREE_PER_VOLT);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivotA = new Solenoid(IO.PNU_CIRCLE_POSITION_A);
		circPivotB = new Solenoid(IO.PNU_CIRCLE_POSITION_B);
		currentArmState = ArmState.STANDBY;
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		currentLiftState = BallLiftSate.BALL_ACQ;
		armHomeSwitch = new MagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME, false);
		ballEntered = new DigitalInput(IO.DIO_PHOTO_BALL_ENTER);
		ballFullyIn = new DigitalInput(IO.DIO_PHOTO_BALL_IN);
		wantedArmAngle = 0;
		timeFired = 0;
		firing = false;
		timeCentered = 0;
		centering = false;
		rollerOn = false;
		wantedPowerRR = 0;
		wantedPowerRL = 0;
		armHome = false;
		wantedRollerPower = 0;
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
		LiveWindow.addActuator(subsystemName, "Flipper", flipper);
		LiveWindow.addActuator(subsystemName, "Circular Pivot A", circPivotA);
		LiveWindow.addActuator(subsystemName, "Circular Pivot B", circPivotB);
		LiveWindow.addSensor(subsystemName, "Ball Entered Sensor", ballEntered);
		LiveWindow.addSensor(subsystemName, "Ball Fully In Sensor", ballFullyIn);
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
			if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
					armEncoder.getDegrees() < wantedArmAngle + DEADBAND)){
				if(armEncoder.getDegrees()>wantedArmAngle)	
					wantedArmPower =  -1 * PUT_IN_FLIP_POWER;
				else
					wantedArmPower = PUT_IN_FLIP_POWER;
			}else{
				wantedArmPower = 0;
				currentArmState = ArmState.STANDBY;
			}
			break;
		case ROTATE_FINDING_HOME:
			if(armHome){
				LOG.logMessage("Arm is home");
				armEncoder.reset();
				currentArmState = ArmState.STANDBY;
			}else{
				wantedArmPower = GOING_HOME_POWER;
			}
			break;
		case PUT_BALL_IN_FLIPPER:
			switch(currentLiftState){
			case BALL_ACQ:
				circPivotA.set(false);
				flipper.set(false);
				wantedArmAngle = ACQUIRE_BALL_DEGREE;
				wantedRollerPower = HIGH_ROLLER_POWER;
				if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
						armEncoder.getDegrees() < wantedArmAngle + DEADBAND) &&
						flipper.get() == false){
					currentLiftState = BallLiftSate.LIFT_1;
				}
				break;
			case LIFT_1:
				circPivotA.set(true);
				flipper.set(false);
				wantedArmAngle = ACQUIRE_BALL_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(circPivotA.get() == true){
					currentLiftState = BallLiftSate.LIFT_2;
				}
				break;
			case LIFT_2:
				circPivotA.set(true);
				flipper.set(true);
				wantedArmAngle = HOLD_BUMPER_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
						armEncoder.getDegrees() < wantedArmAngle + DEADBAND) &&
						flipper.get() == true){
					currentLiftState = BallLiftSate.LIFT_3;
				}
				break;
			case LIFT_3:
				circPivotA.set(false);
				flipper.set(true);
				wantedArmAngle = HOLD_BUMPER_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(circPivotA.get() == false){
					currentLiftState = BallLiftSate.LIFT_4;
				}
				break;
			case LIFT_4:
				circPivotA.set(false);
				flipper.set(false);
				wantedArmAngle = PUT_IN_FLIPPER_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
						armEncoder.getDegrees() < wantedArmAngle + DEADBAND) &&
						flipper.get() == false){
					currentLiftState = BallLiftSate.BALL_STORE;
				}
				break;
			case BALL_STORE:
				circPivotA.set(false);
				flipper.set(false);
				wantedArmAngle = ARM_FIRE_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
						armEncoder.getDegrees() < wantedArmAngle + DEADBAND)){
					currentArmState = ArmState.STANDBY;
				}
				break;
			default:
				System.out.println("INVALID STATE: " + currentLiftState);
				break;
			}
			break;
		case MOVE_AGAINST_BUMPER:
			switch(currentLiftState){
			case BALL_ACQ:
				circPivotA.set(false);
				flipper.set(false);
				wantedArmAngle = ACQUIRE_BALL_DEGREE;
				wantedRollerPower = HIGH_ROLLER_POWER;
				if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
						armEncoder.getDegrees() < wantedArmAngle + DEADBAND) &&
						flipper.get() == false){
					currentLiftState = BallLiftSate.LIFT_1;
				}
				break;
			case LIFT_1:
				circPivotA.set(true);
				flipper.set(false);
				wantedArmAngle = ACQUIRE_BALL_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(circPivotA.get() == true){
					currentLiftState = BallLiftSate.LIFT_2;
				}
				break;
			case LIFT_2:
				circPivotA.set(true);
				flipper.set(true);
				wantedArmAngle = HOLD_BUMPER_DEGREE;
				wantedRollerPower = LOW_ROLLER_POWER;
				if(!(armEncoder.getDegrees() > wantedArmAngle - DEADBAND && 
						armEncoder.getDegrees() < wantedArmAngle + DEADBAND) &&
						flipper.get() == true){
					currentArmState = ArmState.STANDBY;
				}
				break;
			default:
				System.out.println("INVALID STATE: " + currentLiftState);
				break;
			}
			break;
		case OP_CONTROL:
			// might have to change the > to a < depending on testing
			if(armHome || (armEncoder.getDegrees() > MAX_ANGLE && wantedArmPower > 0)){
				wantedArmPower = 0;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}

		switch(currentFlipperState){
		case STANDBY:
			flipper.set(false);
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
				wantedPowerRR = LOW_ROLLER_POWER;
				wantedPowerRL = LOW_ROLLER_POWER;
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
			wantedPowerRR = wantedRollerPower;
			wantedPowerRL = wantedRollerPower;
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
	public void setHome(){
		currentArmState = ArmState.ROTATE_FINDING_HOME;
	}

	/**
	 * sets the power based off the controller
	 * @param pow the controller input
	 */
	public void setArmPower(double pow){
		if(pow == 0){
			currentArmState = ArmState.STANDBY;
		}else{ 
			currentArmState = ArmState.OP_CONTROL;
			wantedArmPower = pow;
		}
	}

	/**
	 * acquires the ball from the ground to the flipper
	 */
	public void acquireBall(){
		if(ballEntered.get()){
			wantedArmAngle = ACQUIRE_BALL_DEGREE;
			currentArmState = ArmState.ROTATING;
			wantedRollerPower = HIGH_ROLLER_POWER;
			currentRollerState = RollerState.ROLLER_ON;
			currentFlipperState = FlipperState.STANDBY;
		}
	}

	/**
	 * moves the arms to bring the ball against the bumper
	 */
	public void moveToBumper(){
		// Do we need to center, then move arm?
		currentArmState = ArmState.MOVE_AGAINST_BUMPER;
	}

	/**
	 * moves the arms to catch the drawbridge
	 */
	public void catchDrawbridge(){


	}

	/**
	 * moves the arms to put the ball in the flipper 
	 */
	public void putBallInFlipper(){
		// Do we need to center, then move arm?
		currentArmState = ArmState.PUT_BALL_IN_FLIPPER;
	}

	/**
	 * moves the ball to raise the gate
	 */
	//Candidate for removal
	public void raiseGate(){
		wantedArmAngle = GATE_POSITION_DEGREE;
		currentArmState = ArmState.ROTATING;
	}
	
	/**
	 * moves to floor position
	 */
	public void goToFloor(){
		wantedArmAngle = FLOOR_POSITION_DEGREE;
		currentArmState = ArmState.ROTATING;
	}

	/**
	 * moves the ball to the position to clip the ball after firing it from the flipper
	 */
	public void clipBall(){
		// Candidate for removal.  This is the crazy shoot the ball into the roller
		wantedArmAngle = ARM_FIRE_DEGREE;
		currentArmState = ArmState.ROTATING;
	}

	/**
	 * fires the flipper
	 * @return true if the flipper fires and false if the flipper is already firing
	 */
	public boolean fire(){
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
	public boolean toggleRoller(){
		if(rollerOn){
			rollerOn = false;
			currentRollerState = RollerState.STANDBY;
			return false;
		}else{
			rollerOn = true;
			wantedPowerRR = LOW_ROLLER_POWER;
			wantedPowerRL = LOW_ROLLER_POWER;
			currentRollerState = RollerState.ROLLER_ON;
			return true;
		}
	}

	/**
	 * Reverses roller.
	 * @return true if the roller is forward, false if the roller is backwards
	 */
	public void reverseRoller(){
		if(rollerOn){
			wantedRollerPower*=-1;
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
	//FIXME: More info to log
	@Override
	protected void writeLog() {
		LOG.logMessage("Current arm state: " + currentArmState);
		LOG.logMessage("Current roller state: " + currentRollerState);
		LOG.logMessage("Current flipper state: " + currentFlipperState);
		LOG.logMessage("Current state of the ball: " + currentLiftState);
		LOG.logMessage("Right Roller Motor speed:" + rollerMotorR.get());
		LOG.logMessage("Left Roller Motor speed:" + rollerMotorL.get());
		LOG.logMessage("Arm Motor speed:" + armMotor.get());
		LOG.logMessage("Arm Home Sensor:" + armHomeSwitch.isTripped());
		LOG.logMessage("Ball Entered Sensor:" + ballEntered.get());
		LOG.logMessage("Ball Fully In Sensor:" + ballFullyIn.get());
		LOG.logMessage("The Arm Degrees: " + armEncoder.getDegrees());
		
	}

	/**
	 * the states for the arm 
	 */
	public enum ArmState{
		STANDBY,
		ROTATING,
		ROTATE_FINDING_HOME,
		PUT_BALL_IN_FLIPPER,
		MOVE_AGAINST_BUMPER,
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
			case MOVE_AGAINST_BUMPER:
				return "Moves the ball to the bumper positions";
			case OP_CONTROL:
				return "Operator is in control";
			default:
				return "Error :(";
			}
		}
	}
	
	/**
	 * the states for lifting the ball
	 */
	public enum BallLiftSate{
		BALL_ACQ,
		LIFT_1,
		LIFT_2,
		LIFT_3,
		LIFT_4,
		BALL_STORE;
		
		/**
		 * Gets the state name
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case BALL_ACQ:
				return "Acquiring the ball";
			case LIFT_1:
				return "Lifting the ball for the 2nd time";
			case LIFT_2:
				return "Lifting the ball for the 3rd time";
			case LIFT_3:
				return "Lifting the ball for the 4th time";
			case LIFT_4:
				return "Lifting the ball for the 5th time";
			case BALL_STORE:
				return "Stores the ball";
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
}
