package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.subsystem.BallAcq.ArmState;
import org.gosparx.team1126.robot.util.AdvancedJoystick;
import org.gosparx.team1126.robot.util.AdvancedJoystick.ButtonEvent;
import org.gosparx.team1126.robot.util.AdvancedJoystick.JoystickListener;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A class for controlling the inputs from controls.
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class Controls extends GenericSubsystem implements JoystickListener{

	/**
	 * Support for singleton
	 */
	private static Controls controls;
	
	/**
	 * The advanced joystick for the right driver stick
	 */
	private AdvancedJoystick driverRight;
	
	/**
	 * The advanced joystick for the left driver stick
	 */
	private AdvancedJoystick driverLeft;
	
	/**
	 * The advanced joystick for the operator
	 */
	private AdvancedJoystick opJoy;
	
	/**
	 * declares a BallAcq named ballAcq
	 */
	private BallAcq ballAcq;
	
	//xbox mapping
	private static final int XBOX_A = 1;
	private static final int XBOX_B = 2;
	private static final int XBOX_X = 3;
	private static final int XBOX_Y = 4;
	private static final int XBOX_L1 = 5;
	private static final int XBOX_R1 = 6;
	private static final int XBOX_BACK = 7;
	private static final int XBOX_START = 8;
	private static final int XBOX_L3 = 9;
	private static final int XBOX_R3 = 10;
	private static final int XBOX_LEFT_X = 0;
	private static final int XBOX_LEFT_Y = 1;
	private static final int XBOX_L2 = 2;
	private static final int XBOX_R2 = 3;
	private static final int XBOX_RIGHT_X = 4;
	private static final int XBOX_RIGHT_Y = 5;
	private static final int XBOX_POV = 0;
	
	/**
	 * @return the only instance of Controls ever.
	 */
	public static synchronized Controls getInstance(){
		if(controls == null){
			controls = new Controls();
		}
		return controls;
	}
	
	/**
	 * Creates a new controls
	 */
	private Controls() {
		super("Controls", Thread.MAX_PRIORITY);
	}

	/**
	 * Sets everything up.
	 */
	@Override
	protected boolean init() {
		driverLeft = new AdvancedJoystick("Driver Left", 0);
		driverLeft.addActionListener(this);
		driverRight = new AdvancedJoystick("Driver Right", 1);
		driverRight.addActionListener(this);
		opJoy = new AdvancedJoystick("Operator Joy", IO.USB_OPERATOR, 10, 0.25);
		opJoy.addActionListener(this);
		opJoy.addButton(XBOX_Y);
		opJoy.addButton(XBOX_R1);
		opJoy.addButton(XBOX_BACK);
		opJoy.addButton(XBOX_L1);
		opJoy.addButton(XBOX_L2);
		opJoy.addButton(XBOX_START);
		opJoy.addButton(XBOX_B);
		opJoy.addButton(XBOX_A);
		opJoy.addButton(XBOX_X);
		opJoy.start();
		ballAcq = BallAcq.getInstance();
		return true;
	}

	/**
	 * Pointless in this class
	 */
	@Override
	protected void liveWindow() {
		
	}

	/**
	 * Loops, controls drives
	 */
	@Override
	protected boolean execute() {
		if(ds.isOperatorControl()){
			ballAcq.startOPControl();
			ballAcq.setArmPower(-opJoy.getAxis(XBOX_LEFT_Y));
			if(opJoy.getPOV(XBOX_POV) == 90){
				ballAcq.toggleRoller();
			}else if(opJoy.getPOV(XBOX_POV) == 270){
				ballAcq.reverseRoller();
			}else if(opJoy.getPOV(XBOX_POV) == 180){
				ballAcq.togglePivotA();
			}else if(opJoy.getPOV(XBOX_POV) == 0){
				ballAcq.togglePivotB();
			}
			if(opJoy.getAxis(XBOX_R2) > .5){
				ballAcq.fire();
			}
		}
		return false;
	}

	/**
	 * Small sleeps for accurate control
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * Writes info to a log every 5 seconds.
	 */
	@Override
	protected void writeLog() {
	
	}

	public void actionPerformed(ButtonEvent e) {
		if(ds.isOperatorControl()){
			switch(e.getID()){
			case XBOX_A:
				//Acquire Fully (I hope)
				if(e.isRising()){
					ballAcq.acquireBall();
					LOG.logMessage("OP Button: Acquire Ball fully");
				}
			case XBOX_B:
				//Acquire Ball to the Bumper (maybe)
				if(e.isRising()){
					ballAcq.moveToBumper();
					LOG.logMessage("OP Button: Acquiring Ball to the bumper);
				}
				case XBOX_X:
				//Acquire Ball to the Flipper from the Bumper
				if(e.isRising()){
					ballAcq.putBallInFlipperFromBumper();
			
				}
			}
		}
	}
}
