package org.gosparx.team1126.robot.subsystem;
import org.gosparx.team1126.robot.subsystem.Drives;
import org.gosparx.team1126.robot.IO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

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
	private Solenoid scale;
	
	//******************************CONSTANTS***********************************

	/**
	 * The distance to the bar and how much line the winch must take in 
	 */
	private final double DISTANCE_TO_BAR_INCHES = 16; //TODO find actual height

	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = false;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;
	
	//******************************VARIABLES***********************************

	/**
	 * Wanted speed for the right motors
	 */
	private double wantedRightPower;
	
	/**
	 * Wanted speed for the left motors
	 */
	private double wantedLeftPower;
	
	/**
	 * The current state that scaling is in
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
	public Scaling() {
		super("Scaling", Thread.NORM_PRIORITY);
	}

	/**
	 * Initializes things 
	 */
	@Override
	protected boolean init() {
		
		//Right 
		rightHook = new DigitalInput(10);
		wantedRightPower = 0;
		
		//Left
		leftHook = new DigitalInput(9);
		wantedLeftPower = 0;
		
		//Other
		currentScalingState = State.STANDBY;
		drives = Drives.getInstance(); 
		scale= new Solenoid(7);
		return true;
	}

	/**
	 * Adds items to the live window, overrides genericSubsystems 
	 */
	@Override
	protected void liveWindow() {
		
	}
	
	/**
	 * Loops
	 */
	@Override
	protected boolean execute() {
		switch(currentScalingState) {
		case STANDBY: 
		{
			
			break;
		}
		case EXTEND_FULL:
		{
			break;
		}
		case EXTENDING_FULL:
		{
			
			break;
		}
		case EXTENDED_FULL:
		{
			
			break;
		}
		case SCALING:
		{
			
			
			break;
		}
		case SCALED:
		{
		
			break;
		}
		case OVERRIDE:
			break;	
		}
		return false;
	}
	
	/**
	 * @param solenoidValue is the value to send to both solenoids
	 */
	private void setArms(boolean solenoidValue)
	{
		if (scale.get() != solenoidValue)
		{
			scale.set(solenoidValue);
		}
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
		
	}
	/**
	 *Makes the states for scaling
	 */
	public enum State{
		STANDBY,
		EXTEND_FULL,
		EXTENDING_FULL,
		EXTENDED_FULL,
		SCALING,
		SCALED,
		OVERRIDE;

		/**
		 * Gets the name of the state
		 * @return the correct state 
		 */
		@Override
		public String toString(){
			switch(this){
			case STANDBY:
				return "Standby";
			case EXTEND_FULL:
				return "Extend full";
			case EXTENDING_FULL:
				return "Extending full";
			case EXTENDED_FULL:
				return "Extended full";
			case SCALING:
				return "Scaling";
			case SCALED:
				return "Scaled";
			case OVERRIDE:
				return "Overriding";
			default:
				return "Unknown state";
			}
		}
	}
}
