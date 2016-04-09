package org.gosparx.team1126.robot;

import org.gosparx.team1126.robot.subsystem.BallAcqNew;
//import org.gosparx.team1126.robot.subsystem.BallAcq;
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
	 * The selector for the AutoMode
	 */
	private SendableChooser chooser;

	private SendableChooser actChooser;
	private SendableChooser posChooser;

	/**
	 * Stores the current autonomous
	 */
	private int[][] currentAuto;

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
	private BallAcqNew ballAcq;

	/*
	 * current selected Auto
	 */
	private String currentAutoName;

	/**
	 * START PRESET ARRAYS
	 */

	private final int[][] LOW_BAR_SETUP = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 132},
			{AutoCommand.DRIVES_DONE.toId()},
	};

	private final int[][] LOW_BAR_POINTGUARD = {
			{AutoCommand.DRIVES_FORWARD.toId(), 12},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_REVERSE.toId(), 114},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 160},
			{AutoCommand.BALL_ACQ_ACQ.toId()},
			{AutoCommand.BALL_ACQ_ROLLER_TOGGLE.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final int[][] LOW_BAR_GOAL = {
			{AutoCommand.DRIVES_FORWARD.toId(), 108},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.CHECK_TIME.toId(), 13, 16},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 62},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 140},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final int[][] PORT_SETUP = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 62},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 120},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
	};

	private final int[][] CHIVAL_SETUP = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 50},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.DRIVES_FORWARD.toId(), 136},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
	};

	private final int[][] PORT_POINTGUARD = {
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 180},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 66},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 120},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.END.toId()}
	};

	private final int[][] CHIVAL_POINTGUARD = {
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.DRIVES_TURN_LEFT.toId(), 181},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 62},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.DRIVES_FORWARD.toId(), 120},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.END.toId()}
	};

	private final int[][] SCORE_2 = {
			{AutoCommand.CHECK_TIME.toId(), 13, 16},
			{AutoCommand.DRIVES_FORWARD.toId(), 84},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 67},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 96},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.END.toId()}
	};
	
	private final int[][] SCORE_3 = {
			{AutoCommand.CHECK_TIME.toId(), 13, 16},
			{AutoCommand.DRIVES_TURN_LEFT.toId(), 45},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 84},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 112},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 96},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.END.toId()}			
	};
	private final int[][] SCORE_4 = {
			{AutoCommand.CHECK_TIME.toId(), 13, 16},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 45},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 84},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_TURN_LEFT.toId(), 112},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 96},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.END.toId()}			
	};
	
	private final int[][] SCORE_5 = {
			{AutoCommand.CHECK_TIME.toId(), 13, 16},
			{AutoCommand.DRIVES_FORWARD.toId(), 84},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_TURN_LEFT.toId(), 45},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 96},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.END.toId()}
	};

	private final String LOW_BAR_GOAL_NAME = "Low bar to low goal";
	private final Integer LOW_BAR_GOAL_NUM = 0;
	private final int[][] LOW_BAR_GOAL_dsafh = {
			{AutoCommand.CHECK_TIME.toId(), 12, 16},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 168},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 72},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 62},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 140},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final String REACH_DEF_NAME = "Reach a Defense";
	private final Integer REACH_DEF_NUM = 1;
	private final int[][] REACH_DEF = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 50},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.END.toId()}
	};

	private final String CROSS_PASSIVE_NAME = "Cross a Passive Defense";
	private final Integer CROSS_PASSIVE_NUM = 2;
	private final int[][] CROSS_PASSIVE = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 144},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.END.toId()}
	};

	private final String PICKUP_REACH_NAME = "#pointgaurdin'";
	private final Integer PICKUP_REACH_NUM = 4;
	private final int[][] PICKUP_REACH = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 144},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_REVERSE.toId(), 138},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 160},
			{AutoCommand.BALL_ACQ_ACQ.toId()},
			{AutoCommand.BALL_ACQ_ROLLER_TOGGLE.toId()},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final String SPY_BOT_NAME = "Score as Spy";
	private final Integer SPY_BOT_NUM = 5;
	private final int[][] SPY_BOT = {
			{AutoCommand.CHECK_TIME.toId(), 12, 4},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 120},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final String PORTICULLIS_NAME = "Cross Porticullis";
	private final Integer PORTICULLIS_NUM = 6;

	private final String CHIVAL_NAME = "#chivauto";
	private final Integer CHIVAL_NUM = 7;

	private final String EMPTY_NAME = "La tortuga (Do nothing)";
	private final Integer EMPTY_NUM = 99;
	private final int[][] EMPTY = {
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

		DRIVES_RETURN_TO_ZERO(7),

		/*DRIVES_DONE*/
		DRIVES_DONE(9),

		/*BALL_ACQ_FLOOR*/
		BALL_ACQ_FLOOR(10),

		BALL_ACQ_ACQ(11),

		BALL_ACQ_HOME(12),

		BALL_ACQ_HOME_NO_ROLLER(13),

		/*BALL_ACQ_ROLLER_TOGGLE*/
		BALL_ACQ_ROLLER_TOGGLE(16),

		/*BALL_ACQ_STOP*/
		BALL_ACQ_STOP(18),

		/*BALL_ACQ_FIRE*/
		BALL_ACQ_FIRE(17),

		/*BALL_ACQ_DONE*/
		BALL_ACQ_DONE(19),

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

		/**
		 * Gets the name of the state
		 * @return the correct state
		 */
		@Override
		public String toString(){
			switch(this){
			case DRIVES_FORWARD:
				return "DRIVES_FORWARD";

			case DRIVES_REVERSE:
				return "DRIVES_REVERSE";

			case DRIVES_TURN_LEFT:
				return "DRIVES_TURN_LEFT";

			case DRIVES_TURN_RIGHT:
				return "DRIVES_TURN_RIGHT";

			case DRIVES_AUTO_DEF:
				return "DRIVES_AUTO_DEF";

			case DRIVES_STOP:
				return "DRIVES_STOP";

			case DRIVES_RETURN_TO_ZERO:
				return "DRIVES_RETURN_TO_ZERO";

			case DRIVES_DONE:
				return "DRIVES_DONE";

			case BALL_ACQ_FLOOR:
				return "BALL_ACQ_FLOOR";

			case BALL_ACQ_ACQ:
				return "BALL_ACQ_ACQ";

			case BALL_ACQ_HOME:
				return "BALL_ACQ_HOME";

			case BALL_ACQ_HOME_NO_ROLLER:
				return "BALL_ACQ_HOME_NO_ROLLER";

			case BALL_ACQ_ROLLER_TOGGLE:
				return "BALL_ACQ_ROLLER_TOGGLE";

			case BALL_ACQ_STOP:
				return "BALL_ACQ_STOP";

			case BALL_ACQ_FIRE:
				return "BALL_ACQ_FIRE";

			case BALL_ACQ_DONE:
				return "BALL_ACQ_DONE";

			case CHECK_TIME:
				return "CHECK_TIME";

			case WAIT:
				return "WAIT";

			case END:
				return "END";

			default:
				return "Error :( Auto in " + this;
			}
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

		drives = Drives.getInstance();
		ballAcq = BallAcqNew.getInstance();

		chooser = new SendableChooser();
		chooser.addDefault(EMPTY_NAME, EMPTY_NUM);
		chooser.addObject(REACH_DEF_NAME, REACH_DEF_NUM);
		chooser.addObject(LOW_BAR_GOAL_NAME, LOW_BAR_GOAL_NUM);
		chooser.addObject(CROSS_PASSIVE_NAME, CROSS_PASSIVE_NUM);
		chooser.addObject(SPY_BOT_NAME, SPY_BOT_NUM);
		chooser.addObject(PORTICULLIS_NAME, PORTICULLIS_NUM);
		chooser.addObject(CHIVAL_NAME, CHIVAL_NUM);

		actChooser = new SendableChooser();
		actChooser.addDefault("Cross", new Integer(0));
		actChooser.addObject("Point Guard", new Integer(1));
		actChooser.addObject("Score", new Integer(2));

		posChooser = new SendableChooser();
		posChooser.addDefault("1", new Integer(1));
		posChooser.addObject("2", new Integer(2));
		posChooser.addObject("3", new Integer(3));
		posChooser.addObject("4", new Integer(4));
		posChooser.addObject("5", new Integer(5));

		SmartDashboard.putData("Auto Chooser", chooser);
		SmartDashboard.putData("Action Chooser", actChooser);
		SmartDashboard.putData("Position", posChooser);
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
			if(currStep == 0) {
				LOG.logMessage("runAuto start: " + currentAutoName);
			}

			LOG.logMessage("runAuto step: " + AutoCommand.fromId(currentAuto[currStep][0]).toString());

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
			case DRIVES_RETURN_TO_ZERO:
				drives.returnToZero();
				break;
			case DRIVES_STOP:
				drives.autoEStop();
				break;
			case DRIVES_AUTO_DEF:

				break;
			case DRIVES_DONE:
				incStep = drives.autoFunctionDone();
				break;
			case BALL_ACQ_FLOOR:
				ballAcq.goToLowBarPosition();
				break;
			case BALL_ACQ_ACQ:
				ballAcq.acquireBall();
				break;
			case BALL_ACQ_HOME:
				ballAcq.homeRollers();
				break;
			case BALL_ACQ_HOME_NO_ROLLER:
				ballAcq.setHome();
				break;
			case BALL_ACQ_ROLLER_TOGGLE:
				ballAcq.toggleRoller();
				break;
			case BALL_ACQ_STOP:
				ballAcq.stopAll();
				break;
			case BALL_ACQ_DONE:
				incStep = ballAcq.isDone();
				break;
			case BALL_ACQ_FIRE:
				ballAcq.fire();
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
		switch ((Integer)chooser.getSelected()){
		case 0:
			currentAutoName = buildLowBar();
			break;
		case 1:
			currentAuto = REACH_DEF;
			currentAutoName = REACH_DEF_NAME;
			break;
		case 2:
			currentAuto = CROSS_PASSIVE;
			currentAutoName = CROSS_PASSIVE_NAME;
			break;
		case 5:
			currentAuto = SPY_BOT;
			currentAutoName = SPY_BOT_NAME;
			break;
		case 6:
			currentAutoName = buildPort();
			break;
		case 7:
			currentAutoName = buildChival();
			break;
		case 99:
			currentAuto = EMPTY;
			currentAutoName = EMPTY_NAME;
			break;
		default:
			currentAuto = EMPTY;
			currentAutoName = "ERROR!";
			break;
		}
		SmartDashboard.putString("Auto Name: ", currentAutoName);
	}

	public void setRunAuto(boolean n){
		runAuto = n;
		LOG.logMessage("runAuto: " + n + " auto " + SmartDashboard.getString("Auto Name: ", "error"));
	}

	private String buildLowBar(){
		currentAuto = LOW_BAR_SETUP;
		switch ((Integer)actChooser.getSelected()){
		case 1:
			currentAuto = new int[LOW_BAR_SETUP.length + LOW_BAR_POINTGUARD.length][];
			System.arraycopy(LOW_BAR_SETUP, 0, currentAuto, 0, LOW_BAR_SETUP.length);
			System.arraycopy(LOW_BAR_POINTGUARD, 0, currentAuto, LOW_BAR_SETUP.length, LOW_BAR_POINTGUARD.length);
			return "Low Bar Point Guard";
		case 2:
			currentAuto = new int[LOW_BAR_SETUP.length + LOW_BAR_GOAL.length][];
			System.arraycopy(LOW_BAR_SETUP, 0, currentAuto, 0, LOW_BAR_SETUP.length);
			System.arraycopy(LOW_BAR_GOAL, 0, currentAuto, LOW_BAR_SETUP.length, LOW_BAR_GOAL.length);
			return "Low Bar Goal";
		default:
			return "Low Bar Cross";
		}
	}

	private String buildPort(){
		currentAuto = PORT_SETUP;
		switch ((Integer)actChooser.getSelected()){
		case 1:
			currentAuto = new int[PORT_SETUP.length + PORT_POINTGUARD.length][];
			System.arraycopy(PORT_SETUP, 0, currentAuto, 0, PORT_SETUP.length);
			System.arraycopy(PORT_POINTGUARD, 0, currentAuto, PORT_SETUP.length, PORT_POINTGUARD.length);
			return "Portculis Point Guard";
		case 2:
			switch((Integer)posChooser.getSelected()){
			case 2:
				currentAuto = new int[PORT_SETUP.length + SCORE_2.length][];
				System.arraycopy(PORT_SETUP, 0, currentAuto, 0, PORT_SETUP.length);
				System.arraycopy(SCORE_2, 0, currentAuto, PORT_SETUP.length, SCORE_2.length);
				return "Port Score Pos 2";
			case 3:
				currentAuto = new int[PORT_SETUP.length + SCORE_3.length][];
				System.arraycopy(PORT_SETUP, 0, currentAuto, 0, PORT_SETUP.length);
				System.arraycopy(SCORE_3, 0, currentAuto, PORT_SETUP.length, SCORE_3.length);
				return "Port Score Pos 3";
			case 4:
				currentAuto = new int[PORT_SETUP.length + SCORE_4.length][];
				System.arraycopy(PORT_SETUP, 0, currentAuto, 0, PORT_SETUP.length);
				System.arraycopy(SCORE_4, 0, currentAuto, PORT_SETUP.length, SCORE_4.length);
				return "Port Score Pos 4";
			case 5:
				currentAuto = new int[PORT_SETUP.length + SCORE_5.length][];
				System.arraycopy(PORT_SETUP, 0, currentAuto, 0, PORT_SETUP.length);
				System.arraycopy(SCORE_5, 0, currentAuto, PORT_SETUP.length, SCORE_5.length);
				return "Port Score Pos 5";
			default:
				currentAuto = EMPTY;
				return "Not implemented";
			}
		default:
			return "Portculis Cross";
		}
	}

	private String buildChival(){
		currentAuto = CHIVAL_SETUP;
		switch ((Integer)actChooser.getSelected()){
		case 1:
			currentAuto = new int[CHIVAL_SETUP.length + CHIVAL_POINTGUARD.length][];
			System.arraycopy(CHIVAL_SETUP, 0, currentAuto, 0, CHIVAL_SETUP.length);
			System.arraycopy(CHIVAL_POINTGUARD, 0, currentAuto, CHIVAL_SETUP.length, CHIVAL_POINTGUARD.length);
			return "Chival Point Guard";
		case 2:
			switch((Integer)posChooser.getSelected()){
			case 2:
				currentAuto = new int[CHIVAL_SETUP.length + SCORE_2.length][];
				System.arraycopy(CHIVAL_SETUP, 0, currentAuto, 0, CHIVAL_SETUP.length);
				System.arraycopy(SCORE_2, 0, currentAuto, CHIVAL_SETUP.length, SCORE_2.length);
				return "Chival Score Pos 2";
			case 3:
				currentAuto = new int[CHIVAL_SETUP.length + SCORE_3.length][];
				System.arraycopy(CHIVAL_SETUP, 0, currentAuto, 0, CHIVAL_SETUP.length);
				System.arraycopy(SCORE_3, 0, currentAuto, CHIVAL_SETUP.length, SCORE_3.length);
				return "Chival Score Pos 3";
			case 4:
				currentAuto = new int[CHIVAL_SETUP.length + SCORE_4.length][];
				System.arraycopy(CHIVAL_SETUP, 0, currentAuto, 0, CHIVAL_SETUP.length);
				System.arraycopy(SCORE_4, 0, currentAuto, CHIVAL_SETUP.length, SCORE_4.length);
				return "Chival Score Pos 4";
			case 5:
				currentAuto = new int[CHIVAL_SETUP.length + SCORE_5.length][];
				System.arraycopy(CHIVAL_SETUP, 0, currentAuto, 0, CHIVAL_SETUP.length);
				System.arraycopy(SCORE_5, 0, currentAuto, CHIVAL_SETUP.length, SCORE_5.length);
				return "Chival Score Pos 5";
			default:
				currentAuto = EMPTY;
				return "Not implemented";

			}
		default:
			return "Chival Cross";
		}
	}

}
