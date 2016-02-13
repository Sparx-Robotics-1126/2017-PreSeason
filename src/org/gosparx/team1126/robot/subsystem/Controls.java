package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.util.AdvancedJoystick;
import org.gosparx.team1126.robot.util.AdvancedJoystick.ButtonEvent;
import org.gosparx.team1126.robot.util.AdvancedJoystick.JoystickListener;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * A class for controlling the inputs from controls.
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class Controls extends GenericSubsystem implements JoystickListener{

	/**
	 * the instance of the camera controller
	 */
	private static CameraController camCont;

	/**
	 * The instance of driver station
	 */
	private static DriverStation ds;

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
	 * used to check if we want to manually control the pto
	 */
	private boolean manualPto = false;

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
		driverLeft = new AdvancedJoystick("Driver Left", IO.DRIVER_JOY_LEFT,4,DEADBAND);
		driverLeft.addActionListener(this);
		driverLeft.addButton(NEW_JOY_LEFT);
		driverLeft.addButton(NEW_JOY_TRIGGER);
		driverLeft.addMultibutton(NEW_JOY_LEFT, NEW_JOY_TRIGGER);

		driverRight = new AdvancedJoystick("Driver Right", IO.DRIVER_JOY_RIGHT,4,DEADBAND);
		driverRight.addActionListener(this);
		driverRight.addButton(NEW_JOY_LEFT);
		driverRight.addButton(NEW_JOY_TRIGGER);
		driverRight.addMultibutton(NEW_JOY_LEFT, NEW_JOY_TRIGGER);
		opJoy = new AdvancedJoystick("Op Joy", 2);
		opJoy.addActionListener(this);
		leftPower = 0;
		rightPower = 0;
		drives = Drives.getInstance();
		ds = DriverStation.getInstance();
		camCont = CameraController.getInstance();
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
		if(ds.isOperatorControl()){
			switch(e.getPort()){
			case IO.DRIVER_JOY_LEFT:
				switch(e.getID()){
				case NEW_JOY_TRIGGER:
					camCont.switchCamera();
					break;
				case NEW_JOY_LEFT:
					drives.toggleShifting();
					break;
				case NEW_JOY_RIGHT:
					drives.driverShifting();
					break;
				}
				break;
			case IO.DRIVER_JOY_RIGHT:
				switch(e.getID()){
				case NEW_JOY_TRIGGER:
					drives.manualPtoEngage();
					manualPto = !manualPto;
					break;
				case NEW_JOY_LEFT:
					drives.eStopScaling();
					break;
				case NEW_JOY_RIGHT:
					//andrews method in scaling
					break;
				}
				break;
			}
		}
	}
}
