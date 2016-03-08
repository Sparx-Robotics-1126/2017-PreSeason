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
	
	//******************************CONSTANTS***********************************

	/**
	 * Winch in position
	 */
	private final double WINCH_IN_DISTANCE = 30; //FIXME find actual distance
	
	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = true;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;
	
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
		drives = Drives.getInstance(); 
		arms = new Solenoid(IO.PNU_CLIMBER_SCALE);
		currentScalingState = State.STANDBY;
		setArms(ARMS_DOWN);
		//drives.setWinchDistance(WINCH_IN_DISTANCE);
		return true;
	}

	/**
	 * Adds items to the live window, overrides genericSubsystems 
	 */
	@Override
	protected void liveWindow() {
		String subsystemName = "Scaling";
		LiveWindow.addActuator(subsystemName, "Arms", arms);
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
		return currentScalingState != State.HOOKING;
	}
	
	/**
	 * Method that controls the calls for scaling
	 */
	public void scale(){
		currentScalingState = State.HOOKING;
	}
	
	/**
	 * Sets the position of the arms
	 * @param solenoidValue is the value to send to both solenoids
	 */
	public void setArms(boolean solenoidValue){
		if (arms.get() != solenoidValue){
			arms.set(solenoidValue);
		}
	}
}