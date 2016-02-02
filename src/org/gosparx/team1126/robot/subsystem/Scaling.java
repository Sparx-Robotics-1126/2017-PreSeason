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
	private final double WINCH_IN_DISTANCE = 0; //FIXME find actual distance
	
	/**
	 * The value of the solenoid if the arms are up
	 */
	private static final boolean ARMS_UP = false;
	
	/**
	 * The value of the solenoid if the arms are down 
	 */
	private static final boolean ARMS_DOWN = !ARMS_UP;

	/**
	 * Value for the extend in power 
	 */
	private static final double EXTEND_POWER = 0.25; //FIXME get actual power

	/**
	 * Value for the power to winch in
	 */
	private static final double WINCH_IN_POWER = .5; //FIXME get actual power
	
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
			if(drives.isScaleExtendingDone()){
			
				currentScalingState = State.SCALING;
			}
			break;
		case SCALING:
			if (rightHook.get() && leftHook.get()){
				setArms(ARMS_DOWN);
				drives.scaleWinch(WINCH_IN_DISTANCE,WINCH_IN_POWER);
				if(drives.isScaleScalingDone())
				{
					currentScalingState = State.SCALED;
				}
			}
			else {
				System.out.println("Hooks not found");
			}
				
			break;
		case SCALED:
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
		SCALING,
		SCALED;

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
			case SCALING:
				return "Scaling";
			case SCALED:
				return "Scaled";
					default:
				return "Unknown state";
			}
		}
	}

	/**
	 * Method that Controls the calls for scaling  
	 */
	public void extendArms()  
	{
		setArms(ARMS_UP);
		drives.scaleExtend(DISTANCE_TO_BAR_INCHES,EXTEND_POWER);
		currentScalingState = State.EXTENDING;
	}
}
