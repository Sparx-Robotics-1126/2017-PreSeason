package org.gosparx.team1126.robot.subsystem;
import org.gosparx.team1126.robot.subsystem.Drives;
import org.gosparx.team1126.robot.IO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * Allows the robot to scale the tower
 * @author Andrew Thompson {andrewt015@gmail.com}
 */
public class Scaling extends GenericSubsystem{
	
	//******************************OBJECTS***********************************
	
	/**
	 * The only instance of Scaling 
	 */
	private static Scaling scaling;
	
	/**
	 * Instance of drives
	 */
	private Drives drives;
	
	/**
	 * Right hook sensor 
	 */
	private DigitalInput rightHook;  
	
	/**
	 * Left hook sensor 
	 */
	private DigitalInput leftHook;
	
	/**
	 * Solenoid to extend arms to scaling position
	 */
	private Solenoid arms;
	
	/**
	 * Solenoid for the winch ratchet
	 */
	private Solenoid ratchet;
	
	//******************************CONSTANTS***********************************

	/**
	 * Winch in position
	 */
	private final double WINCH_IN_DISTANCE = 16; //FIXME find actual distance
	
	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = true;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;
	
	/**
	 * Value for the solenoid if the ratchet is locked
	 */
	public static final boolean LOCK = false;
	
	/**
	 * Value for the solenoid if the ratchet is unlocked
	 */
	public static final boolean UNLOCK = !LOCK;
	
	//******************************VARIABLES***********************************
	
	/**
	 * The current scaling state
	 */
	private State currentScalingState;
	
	/**
	 * Returns the only instance of scaling
	 */
	public static synchronized Scaling getInstance(){
		if(scaling == null){
			scaling = new Scaling();
		}
		return scaling;
	}
	
	/**
	 * Creates a new scaling 
	 */
	private Scaling() {
		super("Scaling", Thread.NORM_PRIORITY);
	}

	/**
	 * Initializes things 
	 */
	@Override
	protected boolean init() {
		
		//Right 
		rightHook = new DigitalInput(IO.DIO_PHOTO_RIGHT_HOOK);
		
		//Left
		leftHook = new DigitalInput(IO.DIO_PHOTO_LEFT_HOOK);
		
		//Other
		drives = Drives.getInstance(); 
		arms = new Solenoid(IO.PNU_CLIMBER_SCALE);
		ratchet = new Solenoid(IO.PNU_WINCH_RATCHET);
		currentScalingState = State.STANDBY;
		setArms(ARMS_DOWN);
		setLock(LOCK);
		return true;
	}

	/**
	 * Adds items to the live window, overrides genericSubsystems 
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "Scaling";
		LiveWindow.addActuator(subsystemName, "Arms", arms);
		LiveWindow.addActuator(subsystemName, "Lock", ratchet);
		LiveWindow.addSensor(subsystemName, "Right Hook", rightHook);
		LiveWindow.addSensor(subsystemName, "Left Hook", leftHook);
	}
	
	/**
	 * Loops
	 */
	@Override
	protected boolean execute() {
		switch(currentScalingState){
		case STANDBY:{
			break;
		}
		case HOOKING:{
			setArms(ARMS_UP);
			if (rightHook.get() && leftHook.get()){
				setArms(ARMS_DOWN);
				currentScalingState = State.SCALING;
			}
			break;
		}
		case SCALING:			
			if(drives.isScaleScalingDone()){
				LOG.logMessage("Scaling complete");
				currentScalingState = State.STANDBY;
			}
			break;
		default:
			break;
			}
		return false;		
	}
	
	/**
	 * Sleep time between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}
	
	/**
	 * Writes info about the subsystem to the log 
	 */
	@Override
	protected void writeLog() {
		LOG.logMessage("Current Scaling State" + currentScalingState);
	}
	
	/**
	 *Makes the states for scaling
	 */
	public enum State{
		STANDBY,
		HOOKING,
		SCALING;
		
		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "Scaling standby";
			case HOOKING:
				return "Hooking";
			case SCALING:
				return "Scaling";
					default:
				return "Unknown scaling state";
			}
		}
	}

	/**
	 * Tell drives that we are not hooking 
	 */
	public boolean hooked(){
		return currentScalingState != State.HOOKING;
	}
	
	/**
	 * Method that controls the calls for scaling
	 */
	public void scale(){
		currentScalingState = State.HOOKING;
		drives.scaleWinch(WINCH_IN_DISTANCE);
	}
		
	/**
	 * Method that estops scaling
	 */
	public void estop(){
		drives.eStopScaling();
		currentScalingState = State.STANDBY;
		LOG.logMessage("Scaling ESTOP");
	}
	
	/**
	 * Sets the position of the arms
	 * @param solenoidValue is the value to send to both solenoids
	 */
	private void setArms(boolean solenoidValue){
		if (arms.get() != solenoidValue){
			arms.set(solenoidValue);
		}
	}
	
	/**
	 * Sets the position of the ratchet 
	 * @param solenoidValue is the value to send to both solenoids
	 */
	public void setLock(boolean solenoidValue){
		if (ratchet.get() != solenoidValue)
		{
			ratchet.set(solenoidValue);
		}
	}
}
