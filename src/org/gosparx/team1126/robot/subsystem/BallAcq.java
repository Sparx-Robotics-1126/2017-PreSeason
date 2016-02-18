package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.EncoderData;
import org.gosparx.team1126.robot.sensors.MagnetSensor;
import org.gosparx.team1126.robot.subsystem.Drives;

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
	private static final double MAX_ANGLE = 120;

	/**
	 * the distance the arm will travel per tick
	 */
	// as of now we are assuming the encoders will have the same distance per tick
	// TODO: Could we recreate and document formula here
	private static final double DISTANCE_PER_TICK = 0.421875;

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
	private static final double ROLLER_OFF_POWER = 0.0;

	/**
	 * The degrees from home the arm has to be to put the ball in the flipper
	 */
	private static final double PUT_IN_FLIPPER_DEGREE = 30;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE_1 = 120;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE_2 = 103;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE_3 = 115;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE_4 = 84;

	/**
	 * The degrees from home the arm has to be to pick up boulders and such
	 */
	private static final double GATE_POSITION_DEGREE_5 = 40;

	/**
	 * the distance to drive after lifting the gate once in inches
	 */
	//need the real distance
	private static final double GATE_DISTANCE_1 = 15;

	/**
	 * the distance to drive after lifting the gate once in inches
	 */
	//need the real distance
	private static final double GATE_DISTANCE_2 = 15;

	/**
	 * the distance to drive after lifting the gate once in inches
	 */
	//need the real distance
	private static final double GATE_DISTANCE_3 = 15;

	/**
	 * the distance to drive after lifting the gate once in inches
	 */
	//need the real distance
	private static final double GATE_DISTANCE_4 = 15;

	/**
	 * the distance to drive after lifting the gate once in inches
	 */
	//need the real distance
	private static final double GATE_DISTANCE_5 = 15;

	/**
	 * The degrees from home the arm has to be for lift 1 and 2
	 */
	private static final double ARM_LIFT_1_DEGREE = 90;

	/**
	 * the degrees from home the arm has to be for lift 2
	 */
	private static final double ARM_LIFT_2_DEGREE = 90;

	/**
	 * The degrees from home the arm has to be for lift 3
	 */
	private static final double ARM_LIFT_3_DEGREE = 85;

	/**
	 * The degrees from home the arm has to be for lift 4, holding the ball against the bumper
	 */
	private static final double ARM_LIFT_4_DEGREE = 85;

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
	private static final double ARM_LIFT_7_DEGREE = 47;

	/**
	 * the degree for the arms on lift 8
	 */
	private static final double ARM_LIFT_8 = 64;

	/**
	 * the degree for the arms on lift 9
	 */
	private static final double ARM_LIFT_9_DEGREE = 47;

	/**
	 * the degree to move up to let go of the drawbridge
	 */
	//as of now we have not been given this angle
	private static final double LET_GO_DRAW_DEGREE = 45;

	/**
	 * the degree to push the drawbridge down
	 */
	//as of now we haven't been given this angle either
	private static final double PUSH_DRAWBRIDGE_DEGREE = 120;

	/**
	 * the degree we need the arms in to score
	 */
	private static final double SCORE_BALL_DEGREE = 0;

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
	 * the time to wait in between to steps to put the ball in the flipper in seconds
	 */
	private static final double WAIT_LIFT_TIME = 0.1;

	/**
	 * the contracted state of the pnus
	 */
	private static final boolean CONTRACTED = false;

	/**
	 * the contracted state of the pnus
	 */
	// FIXME: I suggest you make this one the opposite of CONTRACTED
	private static final boolean EXTENDED = true;

	/**
	 * the time to wait when catching the drawbridge
	 */
	private static final double DRAW_WAIT_TIME = 1;

	/**
	 * the power to hold the arm in place during standby
	 */
	private static final double HOLDING_POWER = 0.05;

	/**
	 * the time to wait in between setting holding power
	 */
	private static final double HOLD_WAIT_TIME = .25;

	/**
	 * the ramping constant
	 */
	// need a real value
	private static final double RAMPING_CONSTANT = 41.0/40.0;

	//*****************************Objects*********************************************

	/**
	 * creates an instance of BallAcq
	 */
	private static BallAcq acq;

	/**
	 * creates an instance of Drives
	 */
	private static Drives drives;

	/**
	 * The rightmost motor that rotates the arm
	 */
	private CANTalon armMotorR;

	/**
	 * The leftmost motor that rotates the arm
	 */
	private CANTalon armMotorL;

	/**
	 * the motor that rotates the right roller motor
	 */
	private CANTalon rollerMotorR;

	/**
	 * the motor that rotates the left roller motor
	 */
	private CANTalon rollerMotorL;

	/**
	 * the rightmost encoder that tracks the motion of the arm
	 */
	private Encoder armEncoderR;

	/**
	 * the encoder data for the rightmost arm encoder
	 */
	private EncoderData armEncoderDataR;

	/**
	 * the leftmost encoder that tracks the motion of the arm
	 */
	private Encoder armEncoderL;

	/**
	 * the encoder data for the leftmost arm encoder
	 */
	private EncoderData armEncoderDataL;

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
	private Solenoid circPivotLong;

	/**
	 * the pnu that controls the circular pivot as well
	 */
	private Solenoid circPivotShort;

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
	// FIXME: You can ask the Talon if on
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
	 * The wanted power of the arm motors
	 */
	private double wantedArmPower;

	/**
	 * The wanted power of the right arm motor
	 */
	private double wantedArmPowerRight;

	/**
	 * The wanted power of the left arm motor
	 */
	private double wantedArmPowerLeft;

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

	/**
	 * the time we started waiting before catching the drawbridge (in seconds)
	 */
	private double drawbridgeWaitTime;

	/**
	 * whether we are catching the drawbridge or not
	 */
	private boolean catchingBridge;

	/**
	 * time we started to wait before correcting the arm position.
	 */
	private double stepTime;

	/**
	 * the average distance traveled between the two arm encoders
	 */
	private double averageArmDistance;

	/**
	 * the distance traveled by the left encoder
	 */
	private double leftDistance;

	/**
	 * the distance traveled by the right encoder
	 */
	private double rightDistance;

	/**
	 * to tell if we are trying to raise the gate
	 */
	private boolean raisingGate;

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
	protected boolean initi() {
		drives = Drives.getInstance();
		armMotorR = new CANTalon(IO.CAN_ACQ_SHOULDER_R);
		armMotorL = new CANTalon(IO.CAN_ACQ_SHOULDER_L);
		rollerMotorR = new CANTalon(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorL = new CANTalon(IO.CAN_ACQ_ROLLERS_L);
		armEncoderR = new Encoder(IO.DIO_SHOULDER_ENC_A, IO.DIO_SHOULDER_ENC_B);
		armEncoderDataR = new EncoderData(armEncoderR, DISTANCE_PER_TICK);
		// FIXME: Use IO
		armEncoderL = new Encoder(45, 34);
		armEncoderDataL = new EncoderData(armEncoderL, DISTANCE_PER_TICK);
		flipper = new Solenoid(IO.PNU_FLIPPER_RELEASE);
		circPivotLong = new Solenoid(IO.PNU_CIRCLE_POSITION_A);
		circPivotShort = new Solenoid(IO.PNU_CIRCLE_POSITION_B);
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
		wantedArmPowerRight = 0;
		wantedArmPowerLeft = 0;
		armHome = false;
		wantedRollerPower = 0;
		stateHoldTime = 0;
		drawbridgeWaitTime = 0;
		catchingBridge = false;
		stepTime = 0;
		averageArmDistance = 0;
		leftDistance = 0;
		rightDistance = 0;
		raisingGate = false;
		return false;
	}

	/**
	 * to add data to objects while in test mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "BallAcq1";
		String subsyst = "BallAcq2";
		LiveWindow.addActuator(subsystemName, "Right Arm Motor", armMotorR);
		LiveWindow.addActuator(subsystemName, "Left Arm Motor", armMotorL);
		LiveWindow.addActuator(subsystemName, "Right Roller Motor", rollerMotorR);
		LiveWindow.addActuator(subsystemName, "Left Roller Motor", rollerMotorL);
		LiveWindow.addActuator(subsystemName, "Right Arm Encoder", armEncoderR);
		LiveWindow.addActuator(subsystemName, "Left Arm Encoder", armEncoderL);
		LiveWindow.addActuator(subsyst, "Flipper", flipper);
		LiveWindow.addActuator(subsyst, "Circular Pivot A", circPivotLong);
		LiveWindow.addActuator(subsyst, "Circular Pivot B", circPivotShort);
		LiveWindow.addSensor(subsyst, "Ball Entered Sensor", ballEntered);
		LiveWindow.addSensor(subsyst, "Ball Fully In Sensor", ballFullyIn);
		// TODO: Add Home switch
	}

	/**
	 * Acquires the ball and then scores/passes the ball
	 * @return false to continue loop
	 */
	@Override
	protected boolean execute() {
		// FIXME: Use abs value for getting distance
		leftDistance = armEncoderDataL.getDistance();
		rightDistance = armEncoderDataR.getDistance();
		averageArmDistance = (leftDistance + rightDistance)/2;
		armHome = armHomeSwitch.isTripped();
		switch(currentArmState){ 
		case STANDBY:
			// FIXME: This way the if is always true (Which is fine)
			stepTime = Timer.getFPGATimestamp();
			if(Timer.getFPGATimestamp() >= stepTime + HOLD_WAIT_TIME){
				// TODO: Just for initial testing add SYstem outs
				if(averageArmDistance > wantedArmAngle + DEADBAND)
					wantedArmPower += HOLDING_POWER;
				else if(averageArmDistance < wantedArmAngle - DEADBAND)
					wantedArmPower -= HOLDING_POWER;
				// TODO: Panic if (wanted Arm power is greater than .20
			}
			break;
		case ROTATE:
			// FIXME: One of them will need to be reversed
			if(!(leftDistance > wantedArmAngle - DEADBAND && 
					leftDistance < wantedArmAngle + DEADBAND)){
				if(leftDistance < wantedArmAngle)	
					wantedArmPowerLeft =  -1 * setRampedArmPower(leftDistance, wantedArmAngle);
				else
					wantedArmPowerLeft = setRampedArmPower(leftDistance, wantedArmAngle);
			}else{
				wantedArmPowerLeft = 0;
				currentArmState = ArmState.STANDBY;
			}
			// FIXME: Make them symetrical
			if(!(rightDistance > wantedArmAngle - DEADBAND && 
					rightDistance < wantedArmAngle + DEADBAND)){
				if(rightDistance < wantedArmAngle)	
					wantedArmPowerRight =  -1 * setRampedArmPower(rightDistance, wantedArmAngle);
				else
					wantedArmPowerRight = setRampedArmPower(rightDistance, wantedArmAngle);
			}else{
				// FIXME:Move out of this else after making symetrical calls
				if(raisingGate){
					// TODO: Add debug success
					wantedArmPowerRight = 0;
					wantedArmPowerLeft = 0;
					currentArmState = ArmState.RAISING_GATE;
				}else{
					// TODO: Add debug success
					wantedArmPowerRight = 0;
					wantedArmPowerLeft = 0;
					currentArmState = ArmState.STANDBY;
				}
			}
			break;
		case ROTATE_FINDING_HOME:
			if(armHome){
				LOG.logMessage("Arm is home");
				resetEncodersAndDatas();
				currentArmState = ArmState.STANDBY;
			}else{
				wantedArmPower = setRampedArmPower(averageArmDistance, 0);
			}
			break;
		case PUT_BALL_IN_FLIPPER:
			moveBallToBumper();
			moveBumperToFlipper();
			// FIXME: remove it is set by moveBall
			currentArmState = ArmState.STANDBY;
			break;
		case MOVE_AGAINST_BUMPER:
			moveBallToBumper();
			// FIXME: remove it is set by moveBall
			currentArmState = ArmState.STANDBY;
			break;
		case MOVE_BUMPER_TO_FLIPPER:
			moveBumperToFlipper();
			// FIXME: remove it is set by moveBall
			currentArmState = ArmState.STANDBY;
			break;
		case CATCH_BRIDGE:
			circPivotLong.set(CONTRACTED);
			circPivotShort.set(EXTENDED);
			wantedArmAngle = LET_GO_DRAW_DEGREE;
			// FIXME: One of them needs to be reversed
			if(leftDistance < wantedArmAngle)	
				wantedArmPowerLeft =  -1 * setRampedArmPower(leftDistance, wantedArmAngle);
			else
				wantedArmPowerLeft = setRampedArmPower(leftDistance, wantedArmAngle);
			if(rightDistance < wantedArmAngle)	
				wantedArmPowerRight =  -1 * setRampedArmPower(rightDistance, wantedArmAngle);
			else
				wantedArmPowerRight = setRampedArmPower(rightDistance, wantedArmAngle);
			if((rightDistance > wantedArmAngle - DEADBAND && rightDistance < wantedArmAngle + DEADBAND) 
					&& (leftDistance > wantedArmAngle - DEADBAND && leftDistance < wantedArmAngle + DEADBAND) 
					&& circPivotLong.get() == CONTRACTED && circPivotShort.get() == EXTENDED){
				// FIXME: You can use the waitTime as a flag instead of the bool
				if(!catchingBridge){	
					drawbridgeWaitTime = Timer.getFPGATimestamp();
					catchingBridge = true;
				}else if(Timer.getFPGATimestamp() >= drawbridgeWaitTime + DRAW_WAIT_TIME){
					wantedArmAngle = PUSH_DRAWBRIDGE_DEGREE;
					wantedArmAngle = LET_GO_DRAW_DEGREE;
					// FIXME: One of them needs to be reversed
					if(leftDistance < wantedArmAngle)	
						wantedArmPowerLeft =  -1 * setRampedArmPower(leftDistance, wantedArmAngle);
					else
						wantedArmPowerLeft = setRampedArmPower(leftDistance, wantedArmAngle);
					if(rightDistance < wantedArmAngle)	
						wantedArmPowerRight =  -1 * setRampedArmPower(rightDistance, wantedArmAngle);
					else
						wantedArmPowerRight = setRampedArmPower(rightDistance, wantedArmAngle);
				}
				// FIXME:Move inside else if above
				// TODO: Add debug success
				if((rightDistance > wantedArmAngle - DEADBAND && rightDistance < wantedArmAngle + DEADBAND) 
						&& (leftDistance > wantedArmAngle - DEADBAND && leftDistance < wantedArmAngle + DEADBAND))
					currentArmState = ArmState.STANDBY;
			}
			break;
		case RAISING_GATE:
			if(!drives.autoFunctionDone()){
				// FIXME: This way the if is always true (Which is fine)
				// I suggest to move to a holding function
				stepTime = Timer.getFPGATimestamp();
				if(Timer.getFPGATimestamp() >= stepTime + HOLD_WAIT_TIME){
					if(averageArmDistance > wantedArmAngle + DEADBAND)
						wantedArmPower += HOLDING_POWER;
					else if(averageArmDistance < wantedArmAngle - DEADBAND)
						wantedArmPower -= HOLDING_POWER;
				}
			}else{
				if(wantedArmAngle == GATE_POSITION_DEGREE_1)
					raiseGate2();
				else if(wantedArmAngle == GATE_POSITION_DEGREE_2)
					raiseGate3();
				else if(wantedArmAngle == GATE_POSITION_DEGREE_3)
					raiseGate4();
				else if(wantedArmAngle == GATE_POSITION_DEGREE_4)
					raiseGate5();
				else if(wantedArmAngle == GATE_POSITION_DEGREE_5){
					raisingGate = false;
					currentArmState = ArmState.STANDBY;
				}
			}
			break;
		case OP_CONTROL:
			//			// might have to change the > to a < depending on testing
			//			if(armHome || (armEncoderData.getDistance() > MAX_ANGLE && wantedArmPower > 0)){
			//				currentArmState = ArmState.STANDBY;
			//			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}

		switch(currentFlipperState){
		case STANDBY:
			break;
		case FIRING:
			circPivotLong.set(EXTENDED);
			circPivotShort.set(CONTRACTED);
			wantedRollerPower = MED_ROLLER_POWER * -1;
			if(currentArmState == ArmState.STANDBY && circPivotLong.get() == EXTENDED && circPivotShort.get() == CONTRACTED){
				if(!firing){
					flipper.set(EXTENDED);
					timeFired = Timer.getFPGATimestamp();
					firing = true;
				}else if(Timer.getFPGATimestamp() >= timeFired + WAIT_FIRE_TIME && firing){
					flipper.set(CONTRACTED);
					firing = false;
					// TODO: Add debug success
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
				wantedPowerRL = -LOW_ROLLER_POWER;
				timeCentered = Timer.getFPGATimestamp();
				centering = true;
			}
			if(Timer.getFPGATimestamp() >= timeCentered + WAIT_CENTERING_TIME && centering){
				wantedPowerRR = 0;
				wantedPowerRL = 0;
				// TODO: Add debug success
				currentRollerState = RollerState.STANDBY;
				centering = false;
			}
			break;
		case ROLLER_ON: 
			wantedPowerRR = wantedRollerPower;
			wantedPowerRL = -wantedRollerPower;
			break;
		default:
			System.out.println("INVALID STATE: " + currentRollerState);
			break;
		}
		if(armHome || (averageArmDistance > MAX_ANGLE && wantedArmPower > 0)){
			// TODO: Debug statement that this is bad
			currentArmState = ArmState.STANDBY;
			wantedArmPower = 0;
		}
		syncMotors();
		rollerMotorR.set(wantedPowerRR);
		rollerMotorL.set(wantedPowerRL);
		armMotorR.set(wantedArmPowerRight);
		armMotorL.set(wantedArmPowerLeft);
		SmartDashboard.putBoolean("Ball Entered?", ballEntered.get());
		SmartDashboard.putBoolean("Ball in Flipper?", ballFullyIn.get());
		return false;
	}

	/**
	 * makes sure the motors are going the same speed
	 */
	private void syncMotors(){
		wantedArmPowerRight = wantedArmPower;
		wantedArmPowerLeft = wantedArmPower;
		// FIXME: abs
		if(armEncoderDataL.getDistance() > armEncoderDataR.getDistance() + DEADBAND){
			wantedArmPowerRight += RAMPING_CONSTANT;
			wantedArmPowerLeft -= RAMPING_CONSTANT;
		// FIXME: use distance not speed
		}else if(armEncoderDataR.getSpeed() > armEncoderDataL.getSpeed() + DEADBAND){
			wantedArmPowerRight -= RAMPING_CONSTANT;
			wantedArmPowerLeft += RAMPING_CONSTANT;
		}
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
	 * called to ramp the speed of arm power based of distance left
	 */
	private double setRampedArmPower(double currentAngle, double endAngle){
		// TODO: Please pull constants
		double wantedPower = (1/10) * Math.sqrt(Math.abs(endAngle-currentAngle));
		if(wantedPower == 0)
			return 0;
		wantedPower = wantedPower > .8 ? .8 : wantedPower;
		wantedPower = wantedPower < .35 ? .35 : wantedPower;
		return wantedPower;
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
	//Possible they just want to move arms to the floor, then operator does the "catch"
	public void catchDrawbridge(){
		wantedArmAngle = LET_GO_DRAW_DEGREE;
		catchingBridge = false;
		currentArmState = ArmState.CATCH_BRIDGE;
	}

	/**
	 * moves the arms to put the ball in the flipper from the bumper 
	 */
	public void putBallInFlipperFromBumper(){
		currentArmState = ArmState.MOVE_BUMPER_TO_FLIPPER;
	}

	/**
	 * raise the gate
	 */
	//FIXME::
	public void raiseGate(){
		raisingGate = true;
		wantedArmAngle = GATE_POSITION_DEGREE_1;
		currentArmState = ArmState.ROTATE;
		drives.driveWantedDistance(GATE_DISTANCE_1);
	}
	
	/**
	 * raise the gate part 2
	 */
	public void raiseGate2(){
		wantedArmAngle = GATE_POSITION_DEGREE_2;
		currentArmState = ArmState.ROTATE;
		drives.driveWantedDistance(GATE_DISTANCE_2);
	}
	
	/**
	 * raise the gate part 3
	 */
	public void raiseGate3(){
		wantedArmAngle = GATE_POSITION_DEGREE_3;
		currentArmState = ArmState.ROTATE;
		drives.driveWantedDistance(GATE_DISTANCE_3);
	}
	
	/**
	 * raise the gate part 4
	 */
	public void raiseGate4(){
		wantedArmAngle = GATE_POSITION_DEGREE_4;
		currentArmState = ArmState.ROTATE;
		drives.driveWantedDistance(GATE_DISTANCE_4);
	}
	
	/**
	 * moves the gate part 5
	 */
	public void raiseGate5(){
		wantedArmAngle = GATE_POSITION_DEGREE_5;
		currentArmState = ArmState.ROTATE;
		drives.driveWantedDistance(GATE_DISTANCE_5);			
	}

	/**
	 * goes to the angle we need to be at when crossing the sally port 
	 */
	public void goToSallyPortPosition(){
		wantedArmAngle = SALLY_PORT_POSITION_DEGREE;
		currentArmState = ArmState.ROTATE;
		circPivotLong.set(CONTRACTED);
		circPivotShort.set(EXTENDED);
		flipper.set(EXTENDED);
	}
	/**
	 * moves to low bar position
	 */
	public void goToLowBarPosition(){
		wantedArmAngle = LOW_BAR_POSITION_DEGREE;
		currentArmState = ArmState.ROTATE;
		circPivotLong.set(CONTRACTED);
		circPivotShort.set(EXTENDED);
		flipper.set(EXTENDED);
	}

	/**
	 * resets the encoders and encoderDatas
	 */
	public void resetEncodersAndDatas(){
		armEncoderL.reset();
		armEncoderR.reset();
		armEncoderDataL.reset();
		armEncoderDataR.reset();
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
			wantedArmAngle = SCORE_BALL_DEGREE;
			currentArmState = ArmState.ROTATE;
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
			wantedPowerRL = -1 * LOW_ROLLER_POWER;
			currentRollerState = RollerState.ROLLER_ON;
			return true;
		}
	}

	/**
	 * Reverses roller.
	 */
	public void reverseRoller(){
		if(rollerOn){
			wantedRollerPower *= -1;
		}
	}

	/**
	 * toggles the position of circle pivot A
	 */
	public void togglePivotLong(){
		if(circPivotLong.get() == EXTENDED)
			circPivotLong.set(CONTRACTED);
		else
			circPivotLong.set(EXTENDED);
	}

	/**
	 * toggles the position of circle pivot B
	 */
	public void togglePivotShort(){
		if(circPivotShort.get() == EXTENDED)
			circPivotShort.set(CONTRACTED);
		else
			circPivotShort.set(EXTENDED);
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
	 * @param currentState the current Lift state we are on
	 * @return whether the state is done or not
	 */
	private boolean run(BallLiftState currentState){
		circPivotLong.set(currentState.extendA);
		circPivotShort.set(currentState.extendB);
		flipper.set(currentState.flipperExtend);
		wantedArmAngle = currentState.armDegrees;
		wantedRollerPower = currentState.rollerSpeed;
		if(stateHoldTime == 0)
			stateHoldTime = Timer.getFPGATimestamp() + WAIT_LIFT_TIME;
		if((averageArmDistance > wantedArmAngle - DEADBAND && 
				averageArmDistance < wantedArmAngle + DEADBAND) &&
				flipper.get() == currentState.flipperExtend && circPivotLong.get() == currentState.extendA &&
				circPivotShort.get() == currentState.extendB && stateHoldTime <= Timer.getFPGATimestamp()){
			resetEncodersAndDatas();
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
			wantedArmPower = setRampedArmPower(averageArmDistance, ACQUIRE_BALL_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_1;
			}
			break;
		case LIFT_1:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_1_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_2;
			}
			break;
		case LIFT_2:
			wantedArmPower = 0; //for hold in place
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_3;
			}
			break;
		case LIFT_3:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_3_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_4;
			}
			break;
		case LIFT_4:
			wantedArmPower = 0;//for hold in place
			if(run(currentLiftState)){
				// TODO: Log success
				currentArmState = ArmState.STANDBY;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentLiftState);
			break;
		}
		if(averageArmDistance > wantedArmAngle)	
			wantedArmPower *= -1;
	}

	/**
	 * moves the ball from the bumper to the flipper
	 */
	private void moveBumperToFlipper(){
		switch(currentLiftState){
		case LIFT_5:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_5_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_6;
			}
			break;
		case LIFT_6:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_6_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_7;
			}
			break;
		case LIFT_7:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_7_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_8;
			}
			break;
		case LIFT_8:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_8);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_9;
			}
			break;
		case LIFT_9:
			wantedArmPower = setRampedArmPower(averageArmDistance, ARM_LIFT_9_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.LIFT_10;
			}
			break;
		case LIFT_10:
			wantedArmPower = setRampedArmPower(averageArmDistance, PUT_IN_FLIPPER_DEGREE);
			if(run(currentLiftState)){
				// TODO: Log success
				currentLiftState = BallLiftState.BALL_STORE;
			}
			break;
		case BALL_STORE:
			wantedArmPower = 0;
			//wantedArmPower = setRampedArmPower(averageArmDistance, wantedArmAngle);
			if(run(currentLiftState)){
				// TODO: Log success
				currentArmState = ArmState.STANDBY;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentLiftState);
			break;
		}
		if(averageArmDistance < wantedArmAngle)	
			wantedArmPower *= -1;
	}

	/**
	 * moves the arms to an okay position so the arms aren't in to the way of the scaling arms
	 * @return true if the arms are out of the way, false if they are in the way
	 */
	public boolean moveToScale(){
		wantedArmAngle = LOW_BAR_POSITION_DEGREE;
		currentArmState = ArmState.ROTATE;
		if(averageArmDistance > wantedArmAngle - DEADBAND && 
				averageArmDistance < wantedArmAngle + DEADBAND){
			return true;
		}else
			return false;
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
		//		LOG.logMessage("Current arm state: " + currentArmState);
		//		LOG.logMessage("Current roller state: " + currentRollerState);
		//		LOG.logMessage("Current flipper state: " + currentFlipperState);
		//		LOG.logMessage("Current state of the ball: " + currentLiftState);
		//		LOG.logMessage("Right Roller Motor speed:" + rollerMotorR.get());
		//		LOG.logMessage("Left Roller Motor speed:" + rollerMotorL.get());
		//		LOG.logMessage("Arm Motor speed:" + armMotor.get());
		//		LOG.logMessage("Arm Home Sensor:" + armHomeSwitch.isTripped());
		//		LOG.logMessage("Ball Entered Sensor:" + ballEntered.get());
		//		LOG.logMessage("Ball Fully In Sensor:" + ballFullyIn.get());
		//		LOG.logMessage("The Arm Degrees: " + armEncoderData.getDistance());
		// FIXME: Add Logs back in
		System.out.println("Current arm state: " + currentArmState);
		System.out.println("Current roller state: " + currentRollerState);
		System.out.println("Current flipper state: " + currentFlipperState);
		System.out.println("Current state of the ball: " + currentLiftState);
		System.out.println("Right Roller Motor speed:" + rollerMotorR.get());
		System.out.println("Left Roller Motor speed:" + rollerMotorL.get());
		System.out.println("Arm Motor speed:" + armMotorR.get());
		System.out.println("Arm Home Sensor:" + armHomeSwitch.isTripped());
		System.out.println("Ball Entered Sensor:" + ballEntered.get());
		System.out.println("Ball Fully In Sensor:" + ballFullyIn.get());
		System.out.println("The Arm Degrees: " + averageArmDistance);
	}

	/**
	 * the states for the arm 
	 */
	public enum ArmState{
		STANDBY,
		ROTATE,
		ROTATE_FINDING_HOME,
		PUT_BALL_IN_FLIPPER,
		MOVE_AGAINST_BUMPER,
		MOVE_BUMPER_TO_FLIPPER,
		CATCH_BRIDGE,
		RAISING_GATE,
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
			case ROTATE:
				return "Rotating";
			case ROTATE_FINDING_HOME:
				return "Finding Home";
			case PUT_BALL_IN_FLIPPER:
				return "Placing ball in flipper from floor";
			case MOVE_AGAINST_BUMPER:
				return "Moving the ball to the bumper position";
			case MOVE_BUMPER_TO_FLIPPER:
				return "Moving the ball from the bumper to flipper";
			case CATCH_BRIDGE:
				return "Catching the drawbridge";
			case RAISING_GATE:
				return "Raising the Gate";
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
		LIFT_1(CONTRACTED, CONTRACTED, ARM_LIFT_1_DEGREE, LIFT_1_ROLLER_POWER, EXTENDED),
		LIFT_2(CONTRACTED, CONTRACTED, ARM_LIFT_2_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
		LIFT_3(CONTRACTED, CONTRACTED, ARM_LIFT_3_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
		LIFT_4(CONTRACTED, EXTENDED, ARM_LIFT_4_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
		LIFT_5(CONTRACTED, EXTENDED, ARM_LIFT_5_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
		LIFT_6(EXTENDED, CONTRACTED, ARM_LIFT_6_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
		LIFT_7(EXTENDED, EXTENDED, ARM_LIFT_7_DEGREE, LOW_ROLLER_POWER, CONTRACTED),
		LIFT_8(EXTENDED, EXTENDED, ARM_LIFT_8, MED_ROLLER_POWER, CONTRACTED),
		LIFT_9(EXTENDED, EXTENDED, ARM_LIFT_9_DEGREE, MED_ROLLER_POWER, CONTRACTED),
		LIFT_10(EXTENDED, EXTENDED, PUT_IN_FLIPPER_DEGREE, MED_ROLLER_POWER, CONTRACTED),
		BALL_STORE(EXTENDED, EXTENDED, PUT_IN_FLIPPER_DEGREE, ROLLER_OFF_POWER, EXTENDED);

		boolean extendA;
		boolean extendB;
		boolean flipperExtend;
		double armDegrees;
		double rollerSpeed;

		/**
		 * Constructs the BallLiftState object
		 * @param a the position of circle pivot
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
