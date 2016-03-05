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

	/**
	 * START PRESET ARRAYS
	 */

	private final String LOW_BAR_GOAL_NAME = "Low Bar to Low Goal";
	private final Integer LOW_BAR_GOAL_NUM = 0;
	private final int[][] LOW_BAR_GOAL = {
			{AutoCommand.CHECK_TIME.toId(), 12, 11},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			//{AutoCommand.DRIVES_FORWARD.toId(), 96},
			//{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 230},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_HOME_NO_ROLLER.toId()},
			{AutoCommand.DRIVES_TURN_RIGHT.toId(), 63},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 72},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
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

	private final String CROSS_LOW_NAME = "Cross the Low Bar";
	private final Integer CROSS_LOW_NUM = 3;
	private final int[][] CROSS_LOW = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_FLOOR.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 144},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.END.toId()}
	};

	private final String PICKUP_REACH_NAME = "Pickup a Ball then Reach";
	private final Integer PICKUP_REACH_NUM = 4;
	private final int[][] PICKUP_REACH = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.BALL_ACQ_ACQ.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.WAIT.toId(), 1},
			{AutoCommand.BALL_ACQ_HOME.toId()},
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_REVERSE.toId(), 96},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final String SPY_BOT_NAME = "Score as Spy";
	private final Integer SPY_BOT_NUM = 5;
	private final int[][] SPY_BOT = {
			{AutoCommand.BALL_ACQ_DONE.toId()},
			{AutoCommand.DRIVES_FORWARD.toId(), 132},
			{AutoCommand.DRIVES_DONE.toId()},
			{AutoCommand.BALL_ACQ_FIRE.toId()},
			{AutoCommand.DRIVES_STOP.toId()},
			{AutoCommand.END.toId()}
	};

	private final String EMPTY_NAME = "DO NOTHING";
	private final Integer EMPTY_NUM = 6;
	private final int[][] EMPTY = {
			{AutoCommand.END.toId()}
	};

	private final int[][] TEST_ARRAY = {

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
		chooser.addObject(LOW_BAR_GOAL_NAME, LOW_BAR_GOAL_NUM);
		chooser.addObject(REACH_DEF_NAME, REACH_DEF_NUM);
		chooser.addObject(CROSS_PASSIVE_NAME, CROSS_PASSIVE_NUM);
		chooser.addObject(CROSS_LOW_NAME, CROSS_LOW_NUM);
		chooser.addObject(PICKUP_REACH_NAME, PICKUP_REACH_NUM);
		chooser.addObject(SPY_BOT_NAME, SPY_BOT_NUM);

		SmartDashboard.putData("Auto Chooser", chooser);
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
			System.out.println(AutoCommand.fromId(currentAuto[currStep][0]));
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
		String curr;
		switch ((Integer)chooser.getSelected()){
		case 0:
			currentAuto = LOW_BAR_GOAL;
			curr = LOW_BAR_GOAL_NAME;
			break;
		case 1:
			currentAuto = REACH_DEF;
			curr = REACH_DEF_NAME;
			break;
		case 2:
			currentAuto = CROSS_PASSIVE;
			curr = CROSS_PASSIVE_NAME;
			break;
		case 3:
			currentAuto = CROSS_LOW;
			curr = CROSS_LOW_NAME;
			break;
		case 4:
			currentAuto = PICKUP_REACH;
			curr = PICKUP_REACH_NAME;
			break;
		case 5:
			currentAuto = SPY_BOT;
			curr = SPY_BOT_NAME;
			break;
		case 6:
			currentAuto = EMPTY;
			curr = EMPTY_NAME;
			break;
		default:
			currentAuto = EMPTY;
			curr = "ERROR!";
			break;
		}
		SmartDashboard.putString("Auto Name: ", curr);
	}

	public void setRunAuto(boolean n){
		runAuto = n;
	}

}
