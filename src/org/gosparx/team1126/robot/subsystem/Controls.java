package org.gosparx.team1126.robot.subsystem;

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
	 * The drivers left joystick
	 */
	private Joystick driverLeftJoy;
	
	/**
	 * The drivers right joystick
	 */
	private Joystick driverRightJoy;
	
	/**
	 * The operators joystick
	 */
	private Joystick operatorJoystick;
	
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
		driverLeftJoy = new Joystick(0);
		driverRightJoy = new Joystick(1);
		operatorJoystick = new Joystick(2);
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
