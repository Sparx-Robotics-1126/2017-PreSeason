package org.gosparx.team1126.robot.subsystem;
import org.gosparx.team1126.robot.subsystem.Drives;
import org.gosparx.team1126.robot.IO;
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
	 * Solenoid to extend arms to scaling position
	 */
	private Solenoid arms;
	
	/**
	 * Solenoid to lock the ratchet 
	 */
	private Solenoid ratchet;
	
	//******************************CONSTANTS***********************************

	/**
	 * Winch in position
	 */
	private final double WINCH_IN_DISTANCE = 30;
	
	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = true;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;
	
	/**
	 * Value of the solenoid if the ratchet is locked
	 */
	public static final boolean LOCK = false;
	
	//******************************VARIABLES***********************************
	
	/**
	 * The current scaling state
	 */
	private State currentScalingState;
	
	private boolean hooked;
	
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
		drives = Drives.getInstance(); 
		arms = new Solenoid(IO.PNU_CLIMBER_SCALE);
		ratchet = new Solenoid(IO.PNU_WINCH_RATCHET);
		currentScalingState = State.STANDBY;
		setArms(ARMS_DOWN);
		setLock(LOCK);
		hooked = false;
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
			currentScalingState = State.SCALING;
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
		return hooked;
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
	 * Sets the ratchet lock
	 * @param solenoidValue is the value sent to the ratchet solenoid
	 */
	private void setLock(boolean solenoidValue){
			if (ratchet.get() != solenoidValue){
				ratchet.set(solenoidValue);
			}
	}
	public void setHooked() {
			hooked = true;
	}
}