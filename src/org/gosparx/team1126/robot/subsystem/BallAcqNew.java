package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.interfaces.CANTalonIF;
import org.gosparx.team1126.interfaces.DigitalInputIF;
import org.gosparx.team1126.interfaces.EncoderDataIF;
import org.gosparx.team1126.interfaces.EncoderIF;
import org.gosparx.team1126.interfaces.MagnetSensorIF;
import org.gosparx.team1126.interfaces.PowerDistributionPanelIF;
import org.gosparx.team1126.interfaces.SolenoidIF;
import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.util.WPI_Factory;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Purpose: to acquire/get the balls and then score them or pass them to teammates.
 * @author Allison 
 * @author Jack
 */

public class BallAcqNew extends GenericSubsystem{

	//****************************Constants******************

	/**
	 * deadband for arm angles.
	 */
	private final double DEADBAND = 1;

	/**
	 * Distance per tick
	 */
	private final double DISTANCE_PER_TICK = (0.1690141 * 4);

	/**
	 * The higher arm power
	 */
	private final double HIGH_ARM_POWER = 0.3;
	
	/**
	 * the power for raising the porticullis
	 */
	private final double PORT_ARM_POWER = 0.6;

	/**
	 * The amount of time we want the flipper to stay up after firing (in seconds)
	 */
	private static final double WAIT_FIRE_TIME = 1;

	/**
	 * The power to use when kicking the ball out of the robot
	 */
	private static final double HIGH_ROLLER_POWER = .9;

	/**
	 * The power to use when dropping the ball to a teammate
	 * The power to use when holding the arms at acquire position.
	 */
	private static final double HOLDING_POWER = 0.05;

	/**
	 * for the flipper and circ pivot long/a
	 */
	private static final boolean CONTRACTED_LONG = false;

	/**
	 * same as above
	 */
	private static final boolean EXTENDED_LONG = !CONTRACTED_LONG;

	/**
	 * for the circ pivot short/b
	 */
	private static final boolean CONTRACTED_SHORT = true;

	/**
	 * same as above
	 */
	private static final boolean EXTENDED_SHORT = !CONTRACTED_SHORT;
	
	/**
	 * the maximum angle the arms can go
	 */
	private static final int MAX_ANGLE = 125;

	/**
	 * the power for the left roller
	 */
	private static final int LEFT_ROLLER_PDP = 10;

	/**
	 * the power for the right roller
	 */
	private static final int RIGHT_ROLLER_PDP = 11;
	
	/**
	 * the offset for the left encoder
	 */
	private static final int LEFT_ENC_OFFSET = 3;
	
	/**
	 * the offset for the right encoder
	 */
	private static final int RIGHT_ENC_OFFSET = 0;

	//*****************************Objects*******************

	/**
	 * the instance of BallAcqNew
	 */
	private static BallAcqNew acqui;

	/**
	 * the current state of the arms
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
	 * the rightmost arm motor
	 */
	private CANTalonIF armMotorRight;

	/**
	 * the leftmost arm motor
	 */
	private CANTalonIF armMotorLeft;

	/**
	 * the rightmost roller motor
	 */
	private CANTalonIF rollerMotorRight;

	/**
	 * the leftmost roller motor
	 */
	private CANTalonIF rollerMotorLeft;

	/**
	 * the solenoid of the flipper
	 */
	private SolenoidIF flipper;

	/**
	 * the rightmost arm encoder
	 */
	private EncoderIF armEncoderRight;

	/**
	 * the leftmost arm encoder
	 */
	private EncoderIF armEncoderLeft;

	/**
	 * the encoder data for the rightmost arm encoder
	 */
	private EncoderDataIF armEncoderDataR;

	/**
	 * the encoder data for the leftmost arm encoder
	 */
	private EncoderDataIF armEncoderDataL;

	/**
	 * Magnetic sensor for the left arm's home position
	 */
	private MagnetSensorIF armHomeSwitchL;
	
	/**
	 * Magnetic sensor for the right arm's home position
	 */
	private MagnetSensorIF armHomeSwitchR;
	
	/**
	 * Magnetic sensor for the left stop position
	 */
	private MagnetSensorIF armStopSwitchL;
	
	/**
	 * Magnetic sensor for the right stop position
	 */
	private MagnetSensorIF armStopSwitchR;

	/**
	 * the photo electric sensor to see if the ball is in
	 */
	private DigitalInputIF ballEntered;

	/**
	 * the photo electric sensor to see if the ball is fully in the robot.
	 */
	private DigitalInputIF ballFullyIn;

	/**
	 * the power distribution panel
	 */
	private PowerDistributionPanelIF pdp;

	//************************Variables*********************

	/**
	 * the wanted angle of the arm
	 */
	private double wantedArmAngle;

	/**
	 * the time we fired the ball during the match (in seconds)
	 */
	private double timeFired;

	/**
	 * The wanted power of the right roller motor.
	 */
	private double wantedPowerRR;

	/**
	 * The wanted power of the left roller motor
	 */
	private double wantedPowerRL;

	/**
	 * The wanted power of the right arm motor
	 */
	private double wantedArmPowerRight;

	/**
	 * The wanted power of the left arm motor
	 */
	private double wantedArmPowerLeft;

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
	 * Do we want to reverse the roller direction?
	 */
	private boolean reverseRollers;

	/**
	 * Whether the left arm is in its home position
	 */
	private boolean armHomeL;
	
	/**
	 * Whether the right arm is in its home position
	 */
	private boolean armHomeR;
	
	/**
	 * Whether we need to set the left arm home
	 */
	private boolean armHomeSetL;
	
	/**
	 * Whether we need to set the right arm home
	 */
	private boolean armHomeSetR;

	/**
	 * Whether we are firing or not
	 */
	private boolean firing;

	/**
	 * temporary
	 */
	private double highAmp = 0;

	/**
	 * The time we started to go to the home position
	 */
	private double startFixHome;
	
	/**
	 * Whether we have started to go to the home position
	 */
	private boolean fixHomeStarted;
	
	private boolean firstHome;
	
	/**
	 * constructs a BallAcqNew Object
	 */
	private BallAcqNew() {
		super("BallAcqNew", Thread.NORM_PRIORITY);
	}

	/**
	 * makes sure that there is only one instance of BallAcq
	 * @return a BallAcq object
	 */
	public static synchronized BallAcqNew getInstance(){
		if(acqui == null){
			acqui = new BallAcqNew();
		}
		return acqui;
	}
	
	/**
	 * Instantiates all of the objects and gives data to the variables
	 * @return true if it runs once and false continues; should be true
	 */
	@Override
	protected boolean init() {
		currentArmState = ArmState.ROTATE_FINDING_HOME;
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		armMotorRight = WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_SHOULDER_R);
		armMotorLeft = WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_SHOULDER_L);
		rollerMotorRight = WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorLeft = WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_ROLLERS_L);
		flipper = WPI_Factory.getInstance().getSolenoid(IO.PNU_FLIPPER_RELEASE);
		armEncoderRight = WPI_Factory.getInstance().getEncoder(IO.DIO_SHOULDER_ENC_RIGHT_A, IO.DIO_SHOULDER_ENC_RIGHT_B);
		armEncoderLeft = WPI_Factory.getInstance().getEncoder(IO.DIO_SHOULDER_ENC_LEFT_A, IO.DIO_SHOULDER_ENC_LEFT_B);
		armEncoderDataR = WPI_Factory.getInstance().getEncoderData(armEncoderRight, DISTANCE_PER_TICK);
		armEncoderDataL = WPI_Factory.getInstance().getEncoderData(armEncoderLeft, DISTANCE_PER_TICK);
		armHomeSwitchL = WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME_L, true);
		armHomeSwitchR = WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME_R, true);
		armStopSwitchL = WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_STOP_L, true);
		armStopSwitchR = WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_STOP_R, true);
		ballEntered = WPI_Factory.getInstance().getDigitalInput(IO.DIO_PHOTO_BALL_ACQ);
		ballFullyIn = WPI_Factory.getInstance().getDigitalInput(IO.DIO_PHOTO_BALL_IN);
		pdp = WPI_Factory.getInstance().createPowerDistributionPanel();
		wantedArmAngle = 0;
		timeFired = 0;
		wantedPowerRR = 0;
		wantedPowerRL = 0;
		wantedArmPowerRight = 0;
		wantedArmPowerLeft = 0;
		averageArmDistance = 0;
		leftDistance = 0;
		rightDistance = 0;
		armHomeL = false;
		firing = false;
		reverseRollers = false;
		armHomeSetL = false;
		armHomeSetR = false;
		fixHomeStarted = false;
		firstHome = false;
		startFixHome = 0;
		return false;
	}

	/**
	 * used to set data during testing mode
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "BallAcq1";
		String subsyst = "BallAcq2";
		String sub = "BallAcq3";
		LiveWindow.addActuator(subsystemName, "Right Arm Motor", armMotorRight);
		LiveWindow.addActuator(subsystemName, "Left Arm Motor", armMotorLeft);
		LiveWindow.addActuator(subsystemName, "Right Roller Motor", rollerMotorRight);
		LiveWindow.addActuator(subsystemName, "Left Roller Motor", rollerMotorLeft);
		LiveWindow.addActuator(sub, "Right Arm Encoder", armEncoderRight);
		LiveWindow.addActuator(sub, "Left Arm Encoder", armEncoderLeft);
		LiveWindow.addActuator(subsyst, "Flipper", flipper);
		LiveWindow.addSensor(subsyst, "Ball Entered Sensor", ballEntered);
		LiveWindow.addSensor(subsyst, "Ball Fully In Sensor", ballFullyIn);

	}

	/**
	 * it runs on a loop until returned false, don't return true
	 * it is what actually makes the robot do things
	 */
	@Override
	protected boolean execute() {
		leftDistance = armEncoderLeft.getDistance() + LEFT_ENC_OFFSET;
		rightDistance = -armEncoderRight.getDistance() + RIGHT_ENC_OFFSET;
		armHomeL = armHomeSwitchL.isTripped();
		armHomeR = armHomeSwitchR.isTripped();
		switch(currentArmState){
		case STANDBY:
			wantedArmPowerRight = 0;
			wantedArmPowerLeft = 0;
			break;
		case ROTATE:
			if(!((leftDistance > wantedArmAngle - DEADBAND) && 
					(leftDistance < wantedArmAngle + DEADBAND))){
				if(wantedArmAngle > leftDistance){
					wantedArmPowerLeft = -HIGH_ARM_POWER;
				}else{
					wantedArmPowerLeft = HIGH_ARM_POWER;
				}
			}else{
				wantedArmPowerLeft = 0;
			}
			if(!((rightDistance > wantedArmAngle - DEADBAND) &&
					(rightDistance < wantedArmAngle + DEADBAND))){
				if(wantedArmAngle > rightDistance){
					wantedArmPowerRight = -HIGH_ARM_POWER;
				}else{
					wantedArmPowerRight = HIGH_ARM_POWER;
				}
			}else{
				wantedArmPowerRight = 0;
			}
			if(wantedArmPowerRight == 0 && wantedArmPowerLeft == 0){
				currentArmState = ArmState.HOLDING;
			}
			break;
		case ROTATE_FINDING_HOME:
			if(armHomeL){
				armMotorLeft.set(0);
				wantedArmPowerLeft = 0;
				armEncoderLeft.reset();
				armHomeSetL = true;
			}else if(!armHomeSetL){
				if(leftDistance > 45){
					wantedArmPowerLeft = 0.45;
				}else
					wantedArmPowerLeft = .20;
			}
			if(armHomeR){
				armMotorRight.set(0);
				wantedArmPowerRight = 0;
				armEncoderRight.reset();
				armHomeSetR = true;
			} if(!armHomeSetR){
				if(rightDistance > 45){
					wantedArmPowerRight = 0.45;
				}else
					wantedArmPowerRight = .20;
			}
			if((armHomeSetL && armHomeSetR) || (firstHome && (leftDistance < -2.5 || rightDistance < -2.5))){
				firstHome = true;
				currentArmState = ArmState.STANDBY;
				currentRollerState = RollerState.STANDBY;
				armMotorLeft.set(0);
				armMotorRight.set(0);
				wantedArmPowerRight = 0;
				wantedArmPowerLeft = 0;
			}
			break;
		case HOLDING:
			if(armEncoderDataL.getDistance() < wantedArmAngle){
				wantedArmPowerRight = 0;
				wantedArmPowerLeft = 0;
			}else{
				wantedArmPowerLeft = HOLDING_POWER;
				wantedArmPowerRight = HOLDING_POWER;
			}
			break;
		case OP_CONTROL:
			break;
		case FIX_STOP:
			wantedArmPowerLeft = -HIGH_ARM_POWER;
			wantedArmPowerRight = -HIGH_ARM_POWER;
			if((armHomeL || armHomeR) && !fixHomeStarted){
				fixHomeStarted = true;
				
			}
			
			if(fixHomeStarted && startFixHome + .75 < timer.getFPGATimestamp()){
				wantedArmPowerLeft = 0;
				wantedArmPowerRight = 0;
				currentArmState = ArmState.ROTATE_FINDING_HOME;
				fixHomeStarted = false;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentArmState);
			break;
		}

		switch(currentFlipperState){
		case STANDBY:
			flipper.set(CONTRACTED_LONG);
			break;
		case FIRING:
			if(!firing){
				flipper.set(EXTENDED_LONG);
				timeFired = timer.getFPGATimestamp();
				firing = true;
			}else if(timer.getFPGATimestamp() >= timeFired + WAIT_FIRE_TIME && firing){
				flipper.set(CONTRACTED_LONG);
				firing = false;
				LOG.logMessage("Succeeded in firing the flipper");
				currentFlipperState = FlipperState.STANDBY;
			}
			break;
		case HOLD_UP:
			flipper.set(EXTENDED_LONG);
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
		case ROLLER_ON:
			if(pdp.getCurrent(LEFT_ROLLER_PDP) > highAmp){
				highAmp = pdp.getCurrent(LEFT_ROLLER_PDP);
				System.out.println("new high: " + highAmp);
			}else if(pdp.getCurrent(RIGHT_ROLLER_PDP) > highAmp){
				highAmp = pdp.getCurrent(RIGHT_ROLLER_PDP);
				System.out.println("new high: " + highAmp);
			}

			wantedPowerRR = HIGH_ROLLER_POWER;
			wantedPowerRL = HIGH_ROLLER_POWER;
			if(currentArmState != ArmState.HOLDING && currentArmState != ArmState.ACQUIRING){
				wantedPowerRL = .5;
				wantedPowerRR = .5;
			}
			break;
		default:
			System.out.println("INVALID STATE: " + currentRollerState);
			break;
		}
		
		if((armStopSwitchL.isTripped() || armStopSwitchR.isTripped()) && !fixHomeStarted){
			LOG.logMessage("OH NOES, WE HIT THE STOP!");
			fixHomeStarted = true;
			wantedArmPowerLeft = 0;
			wantedArmPowerRight = 0;
			currentArmState = ArmState.FIX_STOP;
			currentRollerState = RollerState.STANDBY;
			startFixHome = timer.getFPGATimestamp();
		}
		
		wantedPowerRR = (reverseRollers) ? wantedPowerRR * -1: wantedPowerRR;
		wantedPowerRL = (reverseRollers) ? wantedPowerRL * -1: wantedPowerRL;
		rollerMotorRight.set(wantedPowerRR);
		rollerMotorLeft.set(-wantedPowerRL);
		armMotorRight.set(-wantedArmPowerRight);
		armMotorLeft.set(wantedArmPowerLeft);
		sd.putBoolean("Ball Entered?", ballEntered.get());
		sd.putBoolean("Ball in Flipper?", ballFullyIn.get());
		return false;
	}

	/**
	 * sets the power based off the controller
	 * @param pow the controller input
	 */
	public void setArmPower(double pow){
		if(currentArmState == ArmState.OP_CONTROL){
			wantedArmPowerLeft = pow;
			wantedArmPowerRight = pow;
		}
	}

	/**
	 * 
	 * @param opControl
	 * sets the arm state for operator control
	 */
	public void setOpControl(boolean opControl){
		currentArmState = (opControl) ? ArmState.OP_CONTROL : ArmState.STANDBY;
	}

	/**
	 * sets the home position
	 */
	public void setHome(){
		currentArmState = ArmState.ROTATE_FINDING_HOME;
		currentRollerState = RollerState.STANDBY;
		reverseRoller(false);
		armHomeSetL = false;
		armHomeSetR = false;
	}

	/**
	 * home with rollers on
	 */
	public void homeRollers(){
		currentArmState = ArmState.ROTATE_FINDING_HOME;
		currentRollerState = RollerState.ROLLER_ON;
		reverseRoller(false);
		armHomeSetL = false;
		armHomeSetR = false;
	}

	/**
	 * acquires the ball from the ground to the flipper
	 */
	public void acquireBall(){
		wantedArmAngle = 92;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.ROLLER_ON;
		reverseRoller(false);
	}

	/**
	 * raise the gate
	 */
	public void raiseGate(){
		wantedArmAngle = 0;
		currentArmState = ArmState.ROTATE;
		reverseRoller(false);
		currentRollerState = RollerState.STANDBY;
	}

	/**
	 * goes to the angle we need to be at when crossing the sally port 
	 */
	public void goToSallyPortPosition(){
		wantedArmAngle = 85;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.STANDBY;
		//circPivotLong.set(CONTRACTED_LONG);
		//circPivotShort.set(EXTENDED_SHORT);
		flipper.set(EXTENDED_LONG);
		reverseRoller(false);
	}
	/**
	 * moves to low bar position
	 */
	public void goToLowBarPosition(){
		wantedArmAngle = 115;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.STANDBY;
		//		circPivotLong.set(CONTRACTED_LONG);
		//		circPivotShort.set(EXTENDED_SHORT);
		flipper.set(EXTENDED_LONG);
		reverseRoller(false);
	}

	/**
	 * moves the arms to an okay position so the arms aren't in to the way of the scaling arms
	 * @return true if the arms are out of the way, false if they are in the way
	 */
	public boolean moveToScale(){
		wantedArmAngle = 115;
		currentArmState = ArmState.ROTATE;
		currentRollerState = RollerState.STANDBY;
		reverseRoller(false);
		if(averageArmDistance > wantedArmAngle - DEADBAND && 
				averageArmDistance < wantedArmAngle + DEADBAND){
			return true;
		}else
			return false;
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
		if(currentRollerState == RollerState.ROLLER_ON){
			currentRollerState = RollerState.STANDBY;
			return false;
		}else{
			currentRollerState = RollerState.ROLLER_ON;
			return true;
		}
	}

	/**
	 * Reverses roller.
	 */
	public void reverseRoller(){
		reverseRollers = !reverseRollers;
	}

	/**
	 * Set the reverse of the rollers
	 */
	public void reverseRoller(boolean rev){
		reverseRollers = rev;
	}

	/**
	 * stops the entire system
	 */
	public void stopAll(){
		currentArmState = ArmState.STANDBY;
		flipper.set(CONTRACTED_LONG);
		currentFlipperState = FlipperState.STANDBY;
		currentRollerState = RollerState.STANDBY;
		reverseRoller(false);
	}

	public boolean isDone(){
		return currentArmState == ArmState.HOLDING || currentArmState == ArmState.STANDBY;
	}
	/**
	 * time to rest the system between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * for log messages
	 */
	@Override
	protected void writeLog() {
		LOG.logMessage("Current arm state: " + currentArmState);
		LOG.logMessage("Current roller state: " + currentRollerState);
		LOG.logMessage("Current flipper state: " + currentFlipperState);
		LOG.logMessage("Right Roller Motor speed:" + rollerMotorRight.get());
		LOG.logMessage("Left Roller Motor speed:" + rollerMotorLeft.get());
		LOG.logMessage("Arm Motor Right speed:" + armMotorRight.get());
		LOG.logMessage("Arm Motor Left speed:" + armMotorLeft.get());
		LOG.logMessage("Arm Home Sensor:" + armHomeSwitchL.isTripped());
		LOG.logMessage("Ball Entered Sensor:" + ballEntered.get());
		LOG.logMessage("Ball Fully In Sensor:" + ballFullyIn.get());
		LOG.logMessage("The Arm Left Degrees: " + armEncoderDataL.getDistance());
		LOG.logMessage("The Arm Right Degrees: " + -armEncoderDataR.getDistance());
		LOG.logMessage("Arm Stop L: " + armStopSwitchL.isTripped());
		LOG.logMessage("Arm Stop R: " + armStopSwitchR.isTripped());
	}
	
	public enum ArmState{
		STANDBY,
		ROTATE,
		ROTATE_FINDING_HOME,
		HOLDING,
		ACQUIRING,
		OP_CONTROL,
		FIX_STOP;

		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "The arms are in standby";
			case ROTATE:
				return "The arms are rotating";
			case ROTATE_FINDING_HOME:
				return "The arms are going home";
			case HOLDING:
				return "The arms are being held";
			case ACQUIRING:
				return "We are acquiring";
			case OP_CONTROL:
				return "The operator is in control";
			case FIX_STOP:
				return "Fixing stop";
			default:
				return "Error :( The arms are in " + this;
			}
		}
	}
	public enum FlipperState{
		STANDBY,
		FIRING,
		HOLD_UP;

		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){	
			case STANDBY:
				return "The flipper is in Standby";
			case FIRING:
				return "The flipper is firing";
			case HOLD_UP:
				return "The flipper is being held up";
			default:
				return "Error :( The flipper is in " + this;
			}
		}
	}
	public enum RollerState{
		STANDBY,
		ROLLER_ON;
		
		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "The roller is in Standby";
			case ROLLER_ON:
				return "The roller is on";
			default:
				return "Error :( The roller is in " + this;
			}
		}
	}
}