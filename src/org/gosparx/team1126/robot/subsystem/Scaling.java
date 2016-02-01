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
	private Solenoid arms;
	
	//******************************CONSTANTS***********************************

	/**
	 * The distance to the bar and how much line the winch must take in 
	 */
	private final double DISTANCE_TO_BAR_INCHES = 16; //FIXME find actual distance

	/**
	 * Winch in position
	 */
	private final double WINCH_IN = 0; //FIXME find actual distance
	
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
	public Scaling() {
		super("Scaling", Thread.NORM_PRIORITY);
	}

	/**
	 * Initializes things 
	 */
	@Override
	protected boolean init() {
		
		//Right 
		rightHook = new DigitalInput(IO.DIO_HOOK_R);
		
		//Left
		leftHook = new DigitalInput(IO.DIO_HOOK_L);
		
		//Other
		drives = Drives.getInstance(); 
		arms= new Solenoid(IO.PNU_CLIMBER_SCALE);
		currentScalingState = State.STANDBY;
		setArms(ARMS_DOWN);
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
		switch(currentScalingState){
		case STANDBY:
			break;
		case EXTENDING:
			if(drives.isScaleExtendingDone())
			{
				currentScalingState = State.SCALING;
			}
			break;
		case EXTENDED:
			break;
		case SCALING:
			if (rightHook.get() && leftHook.get())
			{
				setArms(ARMS_DOWN);
				drives.scaleWinch(WINCH_IN);
				if(drives.isScaleScalingDone())
				{
					currentScalingState = State.SCALED;
				}
			}		
			break;
		case SCALED:
			System.out.println("Scaled");
			break;
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
		if (arms.get() != solenoidValue)
		{
			arms.set(solenoidValue);
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
		EXTENDING,
		EXTENDED,
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
			case EXTENDING:
				return "Extending";
			case EXTENDED:
				return "Extended";
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

	/**
	 * Method that Controls the calls for scaling  //FIXME whats this
	 */
	public void scale()  //FIXME does this need to be changed to extend
	{
		setArms(ARMS_UP);
		drives.scaleExtend(DISTANCE_TO_BAR_INCHES);
		currentScalingState = State.EXTENDING;
	}
}
