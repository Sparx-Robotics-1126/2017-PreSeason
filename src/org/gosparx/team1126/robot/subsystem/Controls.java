package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.util.AdvancedJoystick;
import org.gosparx.team1126.robot.util.AdvancedJoystick.ButtonEvent;
import org.gosparx.team1126.robot.util.AdvancedJoystick.JoystickListener;

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
	 * declares a Drives object named drives
	 */
	private static Drives drives;

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
	 * the input from the left joystick
	 */
	private double leftPower;

	/**
	 * the input from the right joystick
	 */
	private double rightPower;
	
	/**
	 * the deadband on the joystick of which we don't want it to move
	 */
	private static final double DEADBAND = 0.05;

	/**
	 * The outputs for the joysticks 
	 */
	private static final int NEW_JOY_X_AXIS = 0;
	private static final int NEW_JOY_Y_AXIS = 1;
	private static final int NEW_JOY_TRIGGER = 1;//TRIGGEr
	private static final int NEW_JOY_LEFT = 2;//LEFT
	private static final int NEW_JOY_RIGHT = 3;//RIGHT
	private static final int NEW_JOY_MIDDLE = 4;

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
		driverLeft = new AdvancedJoystick("Driver Left", 0,4,DEADBAND);
		driverLeft.addActionListener(this);
		driverLeft.addButton(NEW_JOY_LEFT);
		driverLeft.addButton(NEW_JOY_TRIGGER);
		driverLeft.addMultibutton(NEW_JOY_LEFT, NEW_JOY_TRIGGER);
		
		driverRight = new AdvancedJoystick("Driver Right", 1,4,DEADBAND);
		driverRight.addActionListener(this);
		driverRight.addButton(NEW_JOY_LEFT);
		driverRight.addButton(NEW_JOY_TRIGGER);
		driverRight.addMultibutton(NEW_JOY_LEFT, NEW_JOY_TRIGGER);
		opJoy = new AdvancedJoystick("Op Joy", 2);
		opJoy.addActionListener(this);
		leftPower = 0;
		rightPower = 0;
		drives = Drives.getInstance();
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
			leftPower = driverLeft.getAxis(NEW_JOY_Y_AXIS);
			rightPower = driverRight.getAxis(NEW_JOY_Y_AXIS);
			drives.setPower(leftPower, rightPower);
			if(Math.abs(driverLeft.getAxis(NEW_JOY_X_AXIS))> .5){
				drives.driveWantedDistance(120);
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

	/**
	 * Called whenever a button is pressed
	 */
	@Override
	public void actionPerformed(ButtonEvent e) {

	}
}
