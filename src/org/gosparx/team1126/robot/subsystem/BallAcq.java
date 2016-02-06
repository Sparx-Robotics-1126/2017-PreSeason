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
 * Purpose: to acquire/get the balls and then score them
 * @author Allison and Jack
 */

public class BallAcq extends GenericSubsystem{

	//*****************************Constants*******************************************	

	/**
	 * the distance the arm will travel per tick
	 */
	// FIXME:: We need to know real value.  The current design (which is not ideal).
	// Encoder mounted in 12 gear to 40
	private static final double DEGREE_PER_VOLT = 12/40;

	/**
	 * The amount of time we want the flipper to stay up after firing (in seconds)
	 */
	private static final double WAIT_FIRE_TIME = 0.25;

	/**
	 * The amount of time we want the roller to run while centering the ball (in seconds) 
	 */
	private static final double WAIT_CENTERING_TIME = 0.75;

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
	//FIXME:: We don't know what it is
	private static final double CATCH_DRAW_DEGREE = 0;

	/**
	 * The degrees from home the arm has to be to put the ball in the flipper
	 */
	private static final double PUT_IN_FLIPPER_DEGREE = 50;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	//FIXME:: We don't know what it is
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
	// FIXME:: No sure if we will et one
	//private DigitalInput armLimit;

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
		armEncoder = new AbsoluteEncoderData(IO.ANALOG_IN_ABS_ENC_SHOULDER_L, DEGREE_PER_VOLT);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivot = new Solenoid(IO.PNU_CIRCLE_POS);
		currentArmState = ArmState.STANDBY;
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		armHomeSwitch = new MagnetSensor(IO.DIO_ACQ_SHOULDER_HOME, false);
		ballEntered = new DigitalInput(IO.DIO_BALL_ENTERED);
		ballFullyIn = new DigitalInput(IO.DIO_BALL_COMPLETELY_IN);
		//armLimit = new DigitalInput(7);
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
		LiveWindow.addActuator(subsystemName, "Circular Pivot", circPivot);
		// Add Digital inputs
		// Think about adding MagnetSensor
		// Think about adding AbsoluteEncoder
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
		//FIXME: Lets actually do this state?
		case ROTATING:
			if(wantedArmAngle == GATE_POSITION_DEGREE){
				if(armEncoder.getDegrees() > GATE_POSITION_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < GATE_POSITION_DEGREE + DEADBAND){
					//do later
				}else{
					wantedArmAngle = GATE_POSITION_DEGREE;
					wantedArmPower = PUT_IN_FLIP_POWER;
				}
			}else if(wantedArmAngle == HOLD_BUMPER_DEGREE){
				if(armEncoder.getDegrees() > HOLD_BUMPER_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < HOLD_BUMPER_DEGREE + DEADBAND){
					circPivot.set(false);
					wantedRollerPower = LOW_ROLLER_POWER; 
					currentRollerState = RollerState.ROLLER_ON;
					flipper.set(true);
					currentFlipperState = FlipperState.FIRING;
					firing = true;
					wantedArmAngle = PUT_IN_FLIPPER_DEGREE;
				}else{
					circPivot.set(true);
					wantedRollerPower = LOW_ROLLER_POWER; 
					currentRollerState = RollerState.ROLLER_ON;
					flipper.set(true);
					currentFlipperState = FlipperState.FIRING;
					firing = true;
					wantedArmAngle = HOLD_BUMPER_DEGREE;
					wantedArmPower = PUT_IN_FLIP_POWER;
				}
			}else if (wantedArmAngle == CATCH_DRAW_DEGREE){
				if(armEncoder.getDegrees() > CATCH_DRAW_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < CATCH_DRAW_DEGREE + DEADBAND){
					//do later
				}else{
					wantedArmAngle = CATCH_DRAW_DEGREE;
					wantedArmPower = PUT_IN_FLIP_POWER;
				}
			}else if (wantedArmAngle == ARM_FIRE_DEGREE){
				if(armEncoder.getDegrees() > ARM_FIRE_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < ARM_FIRE_DEGREE + DEADBAND){
					//do later
				}else{
					wantedArmAngle = ARM_FIRE_DEGREE;
					wantedArmPower = PUT_IN_FLIP_POWER;
				}
			}else if (wantedArmAngle == ACQUIRE_BALL_DEGREE){
				if(armEncoder.getDegrees() > ACQUIRE_BALL_DEGREE - DEADBAND && 
						armEncoder.getDegrees() < ACQUIRE_BALL_DEGREE + DEADBAND){
					circPivot.set(true);
					wantedRollerPower = LOW_ROLLER_POWER; 
					currentRollerState = RollerState.ROLLER_ON;
					flipper.set(false);
					currentFlipperState = FlipperState.STANDBY;
					wantedArmAngle = HOLD_BUMPER_DEGREE;
				}else{
					wantedArmAngle = ACQUIRE_BALL_DEGREE;
					wantedArmPower = PUT_IN_FLIP_POWER;
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
			//WHAT IF BOTH ARE TRIGGERED?
			// Add if BOTH are triggered.. What do we do?
			// Meaning do we assume one is broken (Which one?)
			// Log it but still do functionality of one of them?
			if(ballEntered.get()){
				wantedArmPower = PUT_IN_FLIP_POWER;
				// Probably wants to go into ROTATING 
			}
			// If we go out of PUT_BALL_IN_FLIPPER and into ROTATING. How
			// de we know and test that the ball is in to stop ROTATING?
			if(ballFullyIn.get()){
				wantedArmPower = 0;
				// FIXME:: Find out what do they really want us to do after
				// getting ball in flipper
				currentArmState = ArmState.ROTATE_FINDING_HOME;
			}
			// What if NONE are triggered?
			// Log hey we don't have a ball. And then what?
			// How do we tell operator?  Rumble the control?
			break;
		case OP_CONTROL:
			//FIXME:: stop the other way??
			// Does Alex mean used the PowerDistributionPanel?
			// If it is see Drives example from last year under AutoLineUP
//			if(armLimit.get()){
//				wantedArmPower = 0;
//				currentArmState = ArmState.STANDBY;
//			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}

		switch(currentFlipperState){
		case STANDBY:
			// Consider setting the solenoid false here
			break;
		case FIRING:
			// You could use the flipper.get() to get the current solenoid state which matches
			// your firing flag
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
			// Add method for Control to call to acquire ball
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
	// WHat if the operator moves the lever by accident?
	// This is more of a question for Controls
	// I think you guys should suggest more than the lever being moved
	// Maybe a combo of a lever and a free button
	// In Controls how do we tell the operator to stop doing what they are
	// trying to do.  Rumble the control?
	// Please consider renaming to something that says arm
	// Controls to dictate power or read it from joy
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
		wantedArmAngle = ACQUIRE_BALL_DEGREE;
		currentArmState = ArmState.ROTATING;
		wantedRollerPower = HIGH_ROLLER_POWER;
		currentRollerState = RollerState.ROLLER_ON;
		currentFlipperState = FlipperState.STANDBY;
	}

	/**
	 * moves the arms to bring the ball against the bumper
	 */
	public void moveToBumper(){
		// Do we need to center, then move arm?
		wantedArmAngle = HOLD_BUMPER_DEGREE;
		currentArmState = ArmState.ROTATING;
	}

	/**
	 * moves the arms to catch the drawbridge
	 */
	public void catchDrawbridge(){
		// Candidat for removal

	}

	/**
	 * moves the arms to put the ball in the flipper 
	 */
	public void putBallInFlipper(){
		// Do we need to center, then move arm?
        // Then change into getting it to bumper and then doing below?
		wantedArmAngle = PUT_IN_FLIPPER_DEGREE;
		currentArmState = ArmState.PUT_BALL_IN_FLIPPER;
	}

	/**
	 * moves the ball to raise the gate
	 */
	public void raiseGate(){
		// Candidate for removal
		// It may be enough to support operator manula movement
		wantedArmAngle = GATE_POSITION_DEGREE;
		currentArmState = ArmState.ROTATING;
	}

	/**
	 * moves the ball to the position to catch the ball from the flipper
	 */
	//FIXME: Needs a better name
	public void catchBall(){
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
	//FIXME: This won't work
	public boolean reverseRoller(){
		// Test if ON and if it is then do below
		wantedPowerRR*=-1;
		wantedPowerRL*=-1;
		// Remove? 
		currentRollerState = RollerState.ROLLER_ON;
//		if(rollerEncoderData.getSpeed() < 0){
//			return false;
//		}else
//			return true;
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
	//FIXME: More info to log
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
}
