package org.gosparx.team1126.robot;

import org.gosparx.team1126.robot.subsystem.Controls;
import org.gosparx.team1126.robot.subsystem.Drives;
import org.gosparx.team1126.robot.subsystem.CameraController;
import org.gosparx.team1126.robot.subsystem.GenericSubsystem;
import org.gosparx.team1126.robot.util.LogWriter;
import edu.wpi.first.wpilibj.SampleRobot;

/**
 * The entry point for the robot. The constructor is called once the robot is turned on.
 */
public class Robot extends SampleRobot{
	/**
	 * An array of all of the subsystems on the robot
	 */
	private GenericSubsystem[] subsystems;

	/**
	 * Called once every time the robot is powered on
	 */
	public Robot() {
		subsystems = new GenericSubsystem[]{
//			ScalingNew.getInstance(),
        	Drives.getInstance(),
//			Autonomous.getInstance(),
//        	BallAcqNew.getInstance(),
        	Controls.getInstance(),
//			CameraController.getInstance(), 
			LogWriter.getInstance()
		};

		for(GenericSubsystem system: subsystems){
			system.start();
			System.out.println(system.getName());
		}
	}

	/**
	 *  Called one time when the robot enters autonomous
	 */
	public void autonomous() {
		System.out.println("AUTO STARTED");
		Autonomous.getInstance().setRunAuto(true);
	}

	/**
	 *  Called one time when the robot enters teleop
	 */
	public void operatorControl() {
		Autonomous.getInstance().setRunAuto(false);
//		Drives.getInstance().killAutoDrive();
	}

	/**
	 *  Called one time when the robot enters test
	 */
	public void test() {
		Autonomous.getInstance().setRunAuto(false);
	}
	
	@Override
	public void disabled(){
		Autonomous.getInstance().setRunAuto(false);
	}
}
