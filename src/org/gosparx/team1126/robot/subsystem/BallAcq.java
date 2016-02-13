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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private static final double HIGH_ROLLER_POWER = 0.75;

	/**
	 * The power to use when dropping the ball to a teammate
	 */
	private static final double LOW_ROLLER_POWER = 0.1;

	/**
	 * The power to use on ball lift 1
	 */
	private static final double LIFT_1_ROLLER_POWER = 0.3;

	/**
	 * The medium power for the roller
	 */
	private static final double MED_ROLLER_POWER = 0.5;

	/**
	 * the 0 power for the roller
	 */
	private static final double ROllER_OFF_POWER = 0.0;

	/**
	 * The power to use when putting the ball in the flipper
	 */
	//probably different
	private static final double GENERAL_ARM_POWER = 0.3;

	/**
	 * The degrees from home the arm has to be at to hold the ball against the bumper
	 */
	private static final double HOLD_BUMPER_DEGREE = 85;

	/**
	 * The degrees from home the arm has to be catch the drawbridge
	 */
	//FIXME:: We don't know what it is-true
	private static final double CATCH_DRAW_DEGREE = 0;

	/**
	 * The degrees from home the arm has to be to put the ball in the flipper
	 */
	private static final double PUT_IN_FLIPPER_DEGREE = 30;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	//may change
	private static final double GATE_POSITION_DEGREE = 120;

	/**
	 * The degrees from home the arm has to be for lift 1 and 2
	 */
	private static final double ARM_LIFT_1_2_DEGREE = 90;

	/**
	 * the degree we need to the arms to be at to acquire the ball
	 */
	private static final double ACQUIRE_BALL_DEGREE = 93;

	/**
	 * the degree for the arms on lift 5
	 */
	private static final double ARM_LIFT_5_DEGREE = 70;

	/**
	 * the degree for the arms on lift 6
	 */
	private static final double ARM_LIFT_6_DEGREE = 54;

	/**
	 * the degree for the arms on lift 7 and 9
	 */
	private static final double ARM_LIFT_7_9_DEGREE = 47;

	/**
	 * the degree for the arms on lift 8
	 */
	private static final double ARM_LIFT_8 = 64;

	/**
	 * the degree we need the arms in to score
	 */
	private static final double SCORE_BALL_DEGREE = 0;

	/**
	 * the floor degree
	 */
	private static final double FLOOR_POSITION_DEGREE = 121;

	/**
	 * the degree for the low bar
	 */
	private static final double LOW_BAR_POSITION_DEGREE = 110;

	/**
	 * the degree for the sally port
	 */
	private static final double SALLY_PORT_POSITION_DEGREE = 85;

	/**
	 * The degrees the arm can be off by and still be considered in a certain spot
	 */
	private static final double DEADBAND = 0.5;

	/**
	 * The power that the arms need to go home
	 */
	private static final double GOING_HOME_POWER = -0.3;

	/**
	 * the time to wait in between to steps to put the ball in the flipper in seconds
	 */
	private static final double WAIT_LIFT_TIME = .1;

	/**
	 * the contracted state of the pnus
	 */
	private static final boolean CONTRACTED = false;

	/**
	 * the contracted state of the pnus
	 */
	private static final boolean EXTENDED = true;

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
	private Encoder armEncoder;

	/**
	 * the encoder data for the arm encoder
	 */
	private EncoderData armEncoderData;

	/**
	 * Magnetic sensor for the arm's home position
	 */
	private MagnetSensor armHomeSwitch;

	/**
	 * the photo electric sensor to see if the ball is in
	 */
	private DigitalInput ballEntered;

	/**
	 * the limit switch to see if the ball is fully in the robot.
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
	private BallLiftState currentLiftState;

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

	/**
	 * the time to hold the state
	 */
	private double stateHoldTime;

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
		armEncoder = new Encoder(IO.DIO_SHOULDER_ENC_A, IO.DIO_SHOULDER_ENC_B);
		armEncoderData = new EncoderData(armEncoder, DEGREE_PER_VOLT);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivotA = new Solenoid(IO.PNU_CIRCLE_POSITION_A);
		circPivotB = new Solenoid(IO.PNU_CIRCLE_POSITION_B);
		currentArmState = ArmState.STANDBY;
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		currentLiftState = BallLiftState.BALL_ACQ;
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
		stateHoldTime = 0;
		return false;
	}

	/**
	 * to add data to objects while in test mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "BallAcq1";
		String subsyst = "BallAcq2";
		LiveWindow.addActuator(subsystemName, "Arm Motor", armMotor);
		LiveWindow.addActuator(subsystemName, "Right Roller Motor", rollerMotorR);
		LiveWindow.addActuator(subsystemName, "Left Roller Motor", rollerMotorL);
		LiveWindow.addActuator(subsystemName, "Arm Encoder", armEncoder);
		LiveWindow.addActuator(subsyst, "Flipper", flipper);
		LiveWindow.addActuator(subsyst, "Circular Pivot A", circPivotA);
		LiveWindow.addActuator(subsyst, "Circular Pivot B", circPivotB);
		LiveWindow.addSensor(subsyst, "Ball Entered Sensor", ballEntered);
		LiveWindow.addSensor(subsyst, "Ball Fully In Sensor", ballFullyIn);
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
		case ROTATING_TO_ANGLE:
			if(!(armEncoderData.getDistance() > wantedArmAngle - DEADBAND && 
					armEncoderData.getDistance() < wantedArmAngle + DEADBAND)){
				if(armEncoderData.getDistance() > wantedArmAngle)	
					wantedArmPower =  -1 * GENERAL_ARM_POWER;
				else
					wantedArmPower = GENERAL_ARM_POWER;
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
			moveBallToBumper();
			moveBumperToFlipper();
			currentArmState = ArmState.STANDBY;
			break;
		case MOVE_AGAINST_BUMPER:
			moveBallToBumper();
			currentArmState = ArmState.STANDBY;
			break;
		case MOVE_BUMPER_TO_FLIPPER:
			moveBumperToFlipper();
			currentArmState = ArmState.STANDBY;
		case OP_CONTROL:
			// might have to change the > to a < depending on testing
			if(armHome || (armEncoderData.getDistance() > MAX_ANGLE && wantedArmPower > 0)){
				wantedArmPower = 0;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}

		switch(currentFlipperState){
		case STANDBY:
			// If we need to make it false then we need to make sure we didn't try setting
			// it true above
			//flipper.set(false);
			break;
		case FIRING:
			circPivotA.set(EXTENDED);
			circPivotB.set(CONTRACTED);
			wantedRollerPower = MED_ROLLER_POWER * -1;
			if(circPivotA.get() == EXTENDED && circPivotB.get() == CONTRACTED){
				if(!firing){
					flipper.set(EXTENDED);
					timeFired = Timer.getFPGATimestamp();
					firing = EXTENDED;
				}else if(Timer.getFPGATimestamp() >= timeFired + WAIT_FIRE_TIME && firing){
					flipper.set(CONTRACTED);
					firing = CONTRACTED;
					currentFlipperState = FlipperState.STANDBY;
				}
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
			SmartDashboard.putBoolean("Ball Entered?", ballEntered.get());
			SmartDashboard.putBoolean("Ball in Flipper?", ballFullyIn.get());
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
			currentArmState = ArmState.PUT_BALL_IN_FLIPPER;
			currentLiftState = BallLiftState.BALL_ACQ;
		}

		/**
		 * moves the arms to bring the ball against the bumper
		 */
		public void moveToBumper(){
			currentArmState = ArmState.MOVE_AGAINST_BUMPER;
			currentLiftState = BallLiftState.BALL_ACQ;
		}

		/**
		 * moves the arms to catch the drawbridge
		 */
		public void catchDrawbridge(){


		}

		/**
		 * moves the arms to put the ball in the flipper from the bumper 
		 */
		public void putBallInFlipperFromBumper(){
			currentArmState = ArmState.MOVE_BUMPER_TO_FLIPPER;
		}

		/**
		 * moves the ball to raise the gate
		 */
		public void raiseGate(){
			// FIXME: We don't think we need the wanted variables
			wantedArmAngle = GATE_POSITION_DEGREE;
			currentArmState = ArmState.ROTATING_TO_ANGLE;
		}

		/**
		 * 
		 */
		public void goToSallyPortPosition(){
			wantedArmAngle = SALLY_PORT_POSITION_DEGREE;
			currentArmState = ArmState.ROTATING_TO_ANGLE;
			circPivotA.set(CONTRACTED);
			circPivotB.set(EXTENDED);
			flipper.set(EXTENDED);
		}

		/**
		 * moves to low bar position
		 */
		public void goToLowBarPosition(){
			wantedArmAngle = LOW_BAR_POSITION_DEGREE;
			currentArmState = ArmState.ROTATING_TO_ANGLE;
			circPivotA.set(CONTRACTED);
			circPivotB.set(EXTENDED);
			flipper.set(EXTENDED);
		}

		//	/**
		//	 * moves the ball to the position to clip the ball after firing it from the flipper
		//	 */
		//	public void clipBall(){
		//		// Candidate for removal.  This is the crazy shoot the ball into the roller
		//		// We don't think we need the wanted variables
		//		wantedArmAngle = ARM_LIFT_1_2_DEGREE;
		//		currentArmState = ArmState.ROTATING_TO_ANGLE;
		//	}

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
		 * stops the entire system
		 */
		public void stopAll(){
			currentArmState = ArmState.STANDBY;
			flipper.set(CONTRACTED);
			currentFlipperState = FlipperState.STANDBY;
			currentRollerState = RollerState.STANDBY;
		}

		/**
		 * Completes the actions during a lift state
		 * @param current the current Lift state we are on
		 * @return whether the state is done or not
		 */
		private boolean run(BallLiftState current){
			circPivotA.set(current.extendA);
			circPivotB.set(current.extendB);
			flipper.set(current.flipperExtend);
			wantedArmAngle = current.armDegrees;
			wantedRollerPower = current.rollerSpeed;
			if(stateHoldTime == 0)
				stateHoldTime = Timer.getFPGATimestamp() + WAIT_LIFT_TIME;
			if((armEncoderData.getDistance() > wantedArmAngle - DEADBAND && 
					armEncoderData.getDistance() < wantedArmAngle + DEADBAND) &&
					flipper.get() == current.flipperExtend && circPivotA.get() == current.extendA &&
					circPivotB.get() == current.extendB && stateHoldTime >= Timer.getFPGATimestamp()){
				stateHoldTime = 0;
				return true;
			}else
				return false;
		}

		/**
		 * moves the ball to the bumper
		 */
		private void moveBallToBumper(){
			switch(currentLiftState){
			case BALL_ACQ:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_1;
				}
				break;
			case LIFT_1:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_2;
				}
				break;
			case LIFT_2:
				wantedArmPower = GENERAL_ARM_POWER;
				wantedArmPower = 0;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_3;
				}
				break;
			case LIFT_3:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_4;
				}
				break;
			case LIFT_4:
				wantedArmPower = 0;
				if(run(currentLiftState)){
					currentArmState = ArmState.STANDBY;
				}
				break;
			default:
				System.out.println("INVALID STATE: " + currentLiftState);
				break;
			}
			if(armEncoderData.getDistance() > wantedArmAngle)	
				wantedArmPower *= -1;
		}

		/**
		 * moves the ball from the bumper to the flipper
		 */
		private void moveBumperToFlipper(){
			switch(currentLiftState){
			case LIFT_5:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_6;
				}
				break;
			case LIFT_6:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_7;
				}
				break;
			case LIFT_7:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_8;
				}
				break;
			case LIFT_8:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_9;
				}
				break;
			case LIFT_9:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.LIFT_10;
				}
				break;
			case LIFT_10:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentLiftState = BallLiftState.BALL_STORE;
				}
				break;
			case BALL_STORE:
				wantedArmPower = GENERAL_ARM_POWER;
				if(run(currentLiftState)){
					currentArmState = ArmState.STANDBY;
				}
				break;
			default:
				System.out.println("INVALID STATE: " + currentLiftState);
				break;
			}
			if(armEncoderData.getDistance() > wantedArmAngle)	
				wantedArmPower *= -1;
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
		//More info to log
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
			LOG.logMessage("The Arm Degrees: " + armEncoderData.getDistance());
		}

		/**
		 * the states for the arm 
		 */
		public enum ArmState{
			STANDBY,
			ROTATING_TO_ANGLE,
			ROTATE_FINDING_HOME,
			PUT_BALL_IN_FLIPPER,
			MOVE_AGAINST_BUMPER,
			MOVE_BUMPER_TO_FLIPPER,
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
				case ROTATING_TO_ANGLE:
					return "Rotating";
				case ROTATE_FINDING_HOME:
					return "Finding Home";
				case PUT_BALL_IN_FLIPPER:
					return "Placing ball in flipper from floor";
				case MOVE_AGAINST_BUMPER:
					return "Moving the ball to the bumper position";
				case MOVE_BUMPER_TO_FLIPPER:
					return "Moving the ball from the bumper to flipper";
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
		private enum BallLiftState{
			BALL_ACQ(CONTRACTED, CONTRACTED, ACQUIRE_BALL_DEGREE, HIGH_ROLLER_POWER, EXTENDED),
			LIFT_1(CONTRACTED, CONTRACTED, ARM_LIFT_1_2_DEGREE, LIFT_1_ROLLER_POWER, EXTENDED),
			LIFT_2(CONTRACTED, CONTRACTED, ARM_LIFT_1_2_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
			LIFT_3(CONTRACTED, CONTRACTED, HOLD_BUMPER_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
			LIFT_4(CONTRACTED, EXTENDED, HOLD_BUMPER_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
			LIFT_5(CONTRACTED, EXTENDED, ARM_LIFT_5_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
			LIFT_6(EXTENDED, CONTRACTED, ARM_LIFT_6_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
			LIFT_7(EXTENDED, EXTENDED, ARM_LIFT_7_9_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
			LIFT_8(EXTENDED, EXTENDED, ARM_LIFT_8, MED_ROLLER_POWER, CONTRACTED),
			LIFT_9(EXTENDED, EXTENDED, ARM_LIFT_7_9_DEGREE, MED_ROLLER_POWER, CONTRACTED),
			LIFT_10(EXTENDED, EXTENDED, PUT_IN_FLIPPER_DEGREE, MED_ROLLER_POWER, CONTRACTED),
			BALL_STORE(EXTENDED, EXTENDED, PUT_IN_FLIPPER_DEGREE, ROllER_OFF_POWER, EXTENDED);

			boolean extendA;
			boolean extendB;
			boolean flipperExtend;
			double armDegrees;
			double rollerSpeed;

			/**
			 * Constructs the BallLiftState object
			 * @param a the position of circle pivot a
			 * @param b the position of circle pivot b
			 * @param armD the wanted angle for the arms
			 * @param rollerS the wanted roller speed
			 * @param f the position of the flipper
			 */
			private BallLiftState(boolean a, boolean b, double armD, double rollerS, boolean f){
				extendA = a;
				extendB = b;
				armDegrees = armD;
				rollerSpeed = rollerS;
				flipperExtend = f;
			}

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
					return "Lifting the ball for the 1st time";
				case LIFT_2:
					return "Lifting the ball for the 2nd time";
				case LIFT_3:
					return "Lifting the ball for the 3rd time";
				case LIFT_4:
					return "Lifting the ball for the 4th time";
				case LIFT_5:
					return "Lifting the ball for the 5th time";
				case LIFT_6:
					return "Lifting the ball for the 6th time";
				case LIFT_7:
					return "Lifting the ball for the 7th time";
				case LIFT_8:
					return "Lifting the ball for the 8th time";
				case LIFT_9:
					return "Lifting the ball for the 9th time";
				case LIFT_10:
					return "Lifting the ball for the 10th time";
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
