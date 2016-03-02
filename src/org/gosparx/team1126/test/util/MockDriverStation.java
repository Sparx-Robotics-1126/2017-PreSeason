package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.RobotStateInterface;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MockDriverStation implements RobotStateInterface {

	public MockDriverStation() {
	}
	
	
	public boolean isDisabled() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public boolean isEnabled() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public boolean isOperatorControl() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public boolean isAutonomous() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public boolean isTest() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public boolean isFMSAttached() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public Alliance getAlliance() {
		// TODO Auto-generated method stub
		return null;
	}

}
