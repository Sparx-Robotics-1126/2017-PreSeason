package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.RobotStateInterface;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MockDriverStation implements RobotStateInterface {

	public MockDriverStation() {
	}
	
	@Override
	public boolean isDisabled() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isEnabled() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isOperatorControl() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isAutonomous() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isTest() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isFMSAttached() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Alliance getAlliance() {
		// TODO Auto-generated method stub
		return null;
	}

}
