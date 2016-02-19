package org.gosparx.team1126.robot;

import org.gosparx.team1126.robot.subsystem.BallAcq;
import org.gosparx.team1126.robot.subsystem.Drives;
import org.gosparx.team1126.robot.subsystem.GenericSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for handling the autonomous functions of the robot
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class Autonomous extends GenericSubsystem{

	/**
	 * Support for singleton
	 */
	private static Autonomous auto;

	/**
	 * Stores the current autonomous
	 */
	private int[][] currentAuto;

	/**
	 * The Sendable chooser for defense
	 */
	private SendableChooser defChooser;

	/**
	 * The SendableChooser for the position of the defense
	 */
	private SendableChooser locChooser;

	/**
	 * The SendableChooser for what to do after crossing
	 */
	private SendableChooser postChooser;

	/**
	 * The current step of the auto we are performing
	 */
	private int currStep = 0;

	/**
	 * Shoudl we move to the next step?
	 */
	private boolean incStep = true;

	/**
	 * Are we running auto?
	 */
	private boolean runAuto = false;

	/**
	 * Are we waiting before moving to the next step?
	 */
	private boolean waiting = false;

	/**
	 * When we should stop waiting
	 */
	private double waitTime = 0.0;

	/**
	 * The "critical" step of auto, what must happen if all else fails.
	 */
	private int critStep = 0;

	/**
	 * When we need to do this step by
	 */
	private double critTime = 0.0;

	/**
	 * When we started auto
	 */
	private double autoStartTime;

	/**
	 * Are we checking the time for a critical step?
	 */
	private boolean checkTime = false;

	/**
	 * An instance of drives
	 */
	private Drives drives;
	
	/**
	 * An instance of BallAcq
	 */
	private BallAcq ballAcq;
	
	/**
	 * START PRESET ARRAYS
	 */

	private final String LOW_BAR_NAME = "Low Bar";
	private final int[][] LOW_BAR = {};

	private final String PORT_NAME = "Porticullis";
	private final int[][] PORT = {};

	private final String CHIVAL_NAME = "Chival de Fris";
	private final int[][] CHIVAL = {};

	private final String MOAT_NAME = "Moat";
	private final int[][] MOAT = {};

	private final String RAMPARTS_NAME = "Ramparts";
	private final int[][] RAMPARTS = {};

	private final String DRAWBRIDGE_NAME = "Drawbridge";
	private final int[][] DRAWBRIDGE = {};

	private final String SALLY_PORT_NAME = "Sally Port";
	private final int[][] SALLY_PORT = {};

	private final String ROCK_WALL_NAME = "Rock Wall";
	private final int[][] ROCK_WALL = {};

	private final String UNEVEN_TERRAIN_NAME = "Uneven Terrarin";
	private final int[][] UNEVEN_TERRAIN = {};

	private final String SLOT_1_LOW_GOAL_NAME = "Slot 1 to Low Goal";
	private final int[][] SLOT_1_LOW_GOAL = {};

	private final String SLOT_2_LOW_GOAL_NAME = "Slot 2 to Low Goal";
	private final int[][] SLOT_2_LOW_GOAL = {};

	private final String SLOT_3_LOW_GOAL_NAME = "Slot 3 to Low Goal";
	private final int[][] SLOT_3_LOW_GOAL = {};

	private final String SLOT_4_LOW_GOAL_NAME = "Slot 4 to Low Goal";
	private final int[][] SLOT_4_LOW_GOAL = {};

	private final String SLOT_5_LOW_GOAL_NAME = "Slot 5 to Low Goal";
	private final int[][] SLOT_5_LOW_GOAL = {};

	private final String CLEAR_DEF_NAME = "Clear defense";
	private final int[][] CLEAR_DEF = {};

	private final int[][] EMPTY_ARRAY = {};

	private final int[][] TEST_ARRAY = {
			{AutoCommand.DRIVES_FORWARD.toId(), 80},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_AUTO_DEF.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 12},
			{AutoCommand.END.toId()}
			};
	
	/**
	 * Enum of all possible autocommands
	 */
	public enum AutoCommand{

		/*DRIVES_FORWARD, inches*/
		DRIVES_FORWARD(1),

		/*DRIVES_REVERSE, inches*/
		DRIVES_REVERSE(2),

		/*DRIVES_TURN_LEFT, degrees*/
		DRIVES_TURN_LEFT(3),

		/*DRIVES_TURN_RIGHT, degrees*/
		DRIVES_TURN_RIGHT(4),

		/*DRIVES_AUTO_DEF*/
		DRIVES_AUTO_DEF(5),

		/*DRIVES_STOP*/
		DRIVES_STOP(6),

		/*DRIVES_DONE*/
		DRIVES_DONE(7),

		/*BALL_ACQ_PRESET, preset number*/
		BALL_ACQ_PRESET(10),

		/*BALL_ACQ_ROLLER_STATE, on(1)/off(0)*/
		BALL_ACQ_ROLLER_STATE(11),

		/*BALL_ACQ_STOP*/
		BALL_ACQ_STOP(12),

		/*BALL_ACQ_DONE*/
		BALL_ACQ_DONE(13),

		/*CHECK_TIME, critTime, critStep*/
		CHECK_TIME(97),

		/*WAIT, waitTime*/
		WAIT(98),

		/*END*/
		END(99);

		/**
		 * The ID of the autocommand
		 */
		private int id;

		/**
		 * Creates a new AutoCommand
		 * @param id Tjhe autoCommand ID
		 */
		private AutoCommand(int id){
			this.id = id;
		}

		/**
		 * @return The ID of this AutoCommand
		 */
		public int toId(){
			return id;
		}

		/**
		 * @param id The desired autocommands id
		 * @return An autocommand with the matching ID
		 */
		public static AutoCommand fromId(int id){
			for(AutoCommand ac: AutoCommand.values()){
				if(ac.toId() == id){
					return ac;
				}
			}
			throw new RuntimeException("No auto exists for ID " + id);
		}
	}

	/**
	 * Private constructor to aid in singleton.
	 */
	private Autonomous() {
		super("Autonomous", Thread.NORM_PRIORITY);
	}

	/**
	 * @return The only instance of Autonomous
	 */
	public static Autonomous getInstance(){
		if(auto == null){
			auto  = new Autonomous();
		}
		return auto;
	}

	/**
	 * Performed once when the subsystems .start() method is called.
	 */
	@Override
	protected boolean init() {
		defChooser = new SendableChooser();
		defChooser.addObject("Low Bar", new Integer(1));
		defChooser.addObject("Porticullis", new Integer(2));
		defChooser.addObject("Chival de Fris", new Integer(3));
		defChooser.addObject("Moat", new Integer(4));
		defChooser.addObject("Ramparts", new Integer(5));
		defChooser.addObject("Drawbridge", new Integer(6));
		defChooser.addObject("Sally Port", new Integer(7));
		defChooser.addObject("Rock Wall", new Integer(8));
		defChooser.addObject("Uneven Terrain", new Integer(9));
		defChooser.addDefault("Do Nothing", new Integer(10));

		locChooser = new SendableChooser();
		locChooser.addObject("Position 1", new Integer(1));
		locChooser.addObject("Position 2", new Integer(2));
		locChooser.addObject("Position 3", new Integer(3));
		locChooser.addObject("Position 4", new Integer(4));
		locChooser.addObject("Position 5", new Integer(5));
		locChooser.addObject("Clear Defense", new Integer(6));
		locChooser.addDefault("Do Nothing", new Integer(7));

		postChooser = new SendableChooser();
		postChooser.addObject("Score Goal", new Integer(1));
		postChooser.addDefault("Do Nothing", new Integer(2));

		SmartDashboard.putData("Defense", defChooser);
		SmartDashboard.putData("Location", locChooser);
		SmartDashboard.putData("Post-Cross", postChooser);
		
		drives = Drives.getInstance();
		ballAcq = BallAcq.getInstance();
		
		return true;
	}

	/**
	 * Loops after .start() is called.
	 */
	@Override
	protected boolean execute() {
		if(runAuto && ds.isEnabled()){
			runAuto();
		}else{
			buildAuto();
			currentAuto = TEST_ARRAY;
			currStep = 0;
			autoStartTime = Timer.getFPGATimestamp();
		}
		return false;
	}

	/**
	 * Actually loops through auto commands
	 */
	private void runAuto(){
		incStep = true;
		if(ds.isEnabled() && ds.isAutonomous() && currStep < currentAuto.length){
			switch(AutoCommand.fromId(currentAuto[currStep][0])){
			case DRIVES_FORWARD:
				drives.driveWantedDistance(currentAuto[currStep][1]);
				break;
			case DRIVES_REVERSE:
				drives.driveWantedDistance(-currentAuto[currStep][1]);
				break;
			case DRIVES_TURN_LEFT:
				drives.turn(-currentAuto[currStep][1]);
				break;
			case DRIVES_TURN_RIGHT:
				drives.turn(currentAuto[currStep][1]);
				break;
			case DRIVES_STOP:
				drives.autoEStop();
				break;
			case DRIVES_DONE:
				incStep = drives.autoFunctionDone();
				break;
			case BALL_ACQ_PRESET:
				break;
			case BALL_ACQ_ROLLER_STATE:
				break;
			case BALL_ACQ_STOP:
				break;
			case BALL_ACQ_DONE:
				break;
			case CHECK_TIME:
				checkTime = true;
				critTime = currentAuto[currStep][1];
				critStep = currentAuto[currStep][2];
				break;
			case WAIT:
				if(!waiting){
					waiting = true;
					waitTime = Timer.getFPGATimestamp() + currentAuto[currStep][1];
				}
				break;
			case END:
				break;
			default:
				incStep = false;
				LOG.logError("Unknown auto command: " + currentAuto[currStep]);
				break;
			}

			if(waiting && waitTime < Timer.getFPGATimestamp()){
				waiting = false;
				waitTime = Double.MAX_VALUE;
				incStep = true;
			}else if(waiting){
				incStep = false;
			}

			if(incStep){
				currStep++;
			}

			if(checkTime && Timer.getFPGATimestamp() - autoStartTime >= critTime && currStep < critStep){
				checkTime = false;
				currStep = critStep;
				LOG.logMessage("Jumping to crit step: " + critStep);
			}
		}
	}

	/**
	 * Updates the livewindow functions
	 */
	@Override
	protected void liveWindow() {
		
	}	

	/**
	 * @return How long to sleep between loops
	 */
	@Override
	protected long sleepTime() {
		return 20;
	}

	/**
	 * Writes to the log file every 5 seconds.
	 */
	@Override
	protected void writeLog() {

	}

	/**
	 * Build our custom auto from chosen def and pos
	 */
	private void buildAuto(){
		int[][] defense = {};
		int[][] posToGoal = {};
		int wantedDef = (Integer)defChooser.getSelected();
		int wantedPos = (Integer)locChooser.getSelected();
		int wantedPost = (Integer)postChooser.getSelected();
		StringBuilder sb = new StringBuilder();
		switch(wantedDef){
		case 1:
			defense = LOW_BAR;
			sb.append(LOW_BAR_NAME);
			break;
		case 2:
			defense = PORT;
			sb.append(PORT_NAME);
			break;
		case 3:
			defense = CHIVAL;
			sb.append(CHIVAL_NAME);
			break;
		case 4:
			defense = MOAT;
			sb.append(MOAT_NAME);
			break;
		case 5:
			defense = RAMPARTS;
			sb.append(RAMPARTS_NAME);
			break;
		case 6:
			defense = DRAWBRIDGE;
			sb.append(DRAWBRIDGE_NAME);
			break;
		case 7:
			defense = SALLY_PORT;
			sb.append(SALLY_PORT_NAME);
			break;
		case 8:
			defense = ROCK_WALL;
			sb.append(ROCK_WALL_NAME);
			break;
		case 9:
			defense = UNEVEN_TERRAIN;
			sb.append(UNEVEN_TERRAIN_NAME);
			break;
		case 10:
			defense = EMPTY_ARRAY;
			sb.append("do nothing");
			break;
		}
		sb.append(" then ");
		switch(wantedPos){
		case 1:
			posToGoal = SLOT_1_LOW_GOAL;
			sb.append(SLOT_1_LOW_GOAL_NAME);
			break;
		case 2: 
			posToGoal = SLOT_2_LOW_GOAL;
			sb.append(SLOT_2_LOW_GOAL_NAME);
			break;
		case 3:
			posToGoal = SLOT_3_LOW_GOAL;
			sb.append(SLOT_3_LOW_GOAL_NAME);
			break;
		case 4:
			posToGoal = SLOT_4_LOW_GOAL;
			sb.append(SLOT_4_LOW_GOAL_NAME);
			break;
		case 5:
			posToGoal = SLOT_5_LOW_GOAL;
			sb.append(SLOT_5_LOW_GOAL_NAME);
			break;
		case 6:
			posToGoal = CLEAR_DEF;
			sb.append(CLEAR_DEF_NAME);
			break;
		case 7:
			posToGoal = EMPTY_ARRAY;
			sb.append("do nothing");
			break;

		}
		sb.append(" then ");
		switch(wantedPost){
		case 1:
			sb.append("score");
			break;
		case 2:
			posToGoal = EMPTY_ARRAY;
			sb.append("do nothing");
			break;
		}

		currentAuto = new int[defense.length+posToGoal.length][];
		System.arraycopy(defense,0,currentAuto,0,defense.length);
		System.arraycopy(posToGoal,0,currentAuto,defense.length,posToGoal.length);
		SmartDashboard.putString("Auto Name: ", sb.toString());
	}
	
	public void setRunAuto(boolean n){
		runAuto = n;
	}
	
}
