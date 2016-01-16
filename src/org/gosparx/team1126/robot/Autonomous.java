package org.gosparx.team1126.robot;

import org.gosparx.team1126.robot.subsystem.GenericSubsystem;

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
	 * START PRESET ARRAYS
	 */
	private final int[][] LOW_BAR = {};
	
	private final int[][] PORT = {};
	
	private final int[][] CHIVAL = {};
	
	private final int[][] MOAT = {};
	
	private final int[][] RAMPARTS = {};
	
	private final int[][] DRAWBRIDGE = {};
	
	private final int[][] SALLY_PORT = {};
	
	private final int[][] ROCK_WALL = {};
	
	private final int[][] UNEVEN_TERRAIN = {};
	
	private final int[][] SLOT_1_LOW_GOAL = {};
	
	private final int[][] SLOT_2_LOW_GOAL = {};
	
	private final int[][] SLOT_3_LOW_GOAL = {};

	private final int[][] SLOT_4_LOW_GOAL = {};
	
	private final int[][] SLOT_5_LOW_GOAL = {};
	
	private final int[][] EMPTY_ARRAY = {};
	
	/**
	 * Enum of all possible autocommands
	 */
	private enum AutoCommand{

		DRIVES_FORWARD(1);

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
		public AutoCommand fromId(int id){
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
		defChooser.addDefault("Low Bar", new Integer(1));
		defChooser.addObject("Porticullis", new Integer(2));
		defChooser.addObject("Chival de Fris", new Integer(3));
		defChooser.addObject("Moat", new Integer(4));
		defChooser.addObject("Ramparts", new Integer(5));
		defChooser.addObject("Drawbridge", new Integer(6));
		defChooser.addObject("Sally Port", new Integer(7));
		defChooser.addObject("Rock Wall", new Integer(8));
		defChooser.addObject("Uneven Terrain", new Integer(9));
		
		locChooser = new SendableChooser();
		locChooser.addDefault("Position 1", new Integer(1));
		locChooser.addObject("Position 2", new Integer(2));
		locChooser.addObject("Position 3", new Integer(3));
		locChooser.addObject("Position 4", new Integer(4));
		locChooser.addObject("Position 5", new Integer(5));
		
		postChooser = new SendableChooser();
		postChooser.addDefault("Score Goal", new Integer(1));
		postChooser.addObject("Do Nothing", new Integer(2));
		
		SmartDashboard.putData("Defense", defChooser);
		SmartDashboard.putData("Location", locChooser);
		SmartDashboard.putData("Post-Cross", postChooser);
		return true;
	}

	/**
	 * Loops after .start() is called.
	 */
	@Override
	protected boolean execute() {
		return false;
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
	
	private void buildAuto(){
		int[][] defense = {};
		int[][] posToGoal = {};
		int wantedDef = (Integer)defChooser.getSelected();
		int wantedPos = (Integer)locChooser.getSelected();
		int wantedPost = (Integer)postChooser.getSelected();
		switch(wantedDef){
		case 1:
			defense = LOW_BAR;
			break;
		case 2:
			defense = PORT;
			break;
		case 3:
			defense = CHIVAL;
			break;
		case 4:
			defense = MOAT;
			break;
		case 5:
			defense = RAMPARTS;
			break;
		case 6:
			defense = DRAWBRIDGE;
			break;
		case 7:
			defense = SALLY_PORT;
			break;
		case 8:
			defense = ROCK_WALL;
			break;
		case 9:
			defense = UNEVEN_TERRAIN;
			break;
		}
		switch(wantedPos){
		case 1:
			posToGoal = SLOT_1_LOW_GOAL;
			break;
		case 2: 
			posToGoal = SLOT_2_LOW_GOAL;
			break;
		case 3:
			posToGoal = SLOT_3_LOW_GOAL;
			break;
		case 4:
			posToGoal = SLOT_4_LOW_GOAL;
			break;
		case 5:
			posToGoal = SLOT_5_LOW_GOAL;
			break;
		}
		switch(wantedPost){
		case 1:
			break;
		case 2:
			posToGoal = EMPTY_ARRAY;
			break;
		}
		
		currentAuto = new int[defense.length+posToGoal.length][];
		System.arraycopy(defense,0,currentAuto,0,defense.length);
		System.arraycopy(posToGoal,0,currentAuto,defense.length,posToGoal.length);
	}

}
