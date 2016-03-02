package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.RobotStateInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockDriverStation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationWrapper implements RobotStateInterface, UnitTestInterface {

	private boolean inUnitTest;
	private RobotStateInterface mock;
	private static RobotStateInterface instance = new DriverStationWrapper();

	public static RobotStateInterface getInstance() {
	    return DriverStationWrapper.instance;
	  }

	private DriverStationWrapper() {
		inUnitTest = false;
	}

	@Override
	public boolean isDisabled() {
		if(inUnitTest) {
			return mock.isDisabled();
		} else {
			return DriverStation.getInstance().isDisabled();
		}
	}

	@Override
	public boolean isEnabled() {
		if(inUnitTest) {
			return mock.isEnabled();
		} else {
			return DriverStation.getInstance().isEnabled();
		}
	}

	@Override
	public boolean isOperatorControl() {
		if(inUnitTest) {
			return mock.isOperatorControl();
		} else {
			return DriverStation.getInstance().isOperatorControl();
		}
	}

	@Override
	public boolean isAutonomous() {
		if(inUnitTest) {
			return mock.isAutonomous();
		} else {
			return DriverStation.getInstance().isAutonomous();
		}
	}

	@Override
	public boolean isTest() {
		if(inUnitTest) {
			return mock.isTest();
		} else {
			return DriverStation.getInstance().isTest();
		}
	}

	@Override
	public boolean isFMSAttached() {
		if(inUnitTest) {
			return mock.isFMSAttached();
		} else {
			return DriverStation.getInstance().isFMSAttached();
		}
	}

	@Override
	public Alliance getAlliance() {
		if(inUnitTest) {
			return mock.getAlliance();
		} else {
			return DriverStation.getInstance().getAlliance();
		}
	}

	@Override
	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockDriverStation();
		}
	}

	@Override
	public boolean inUnitTest() {
		return inUnitTest;
	}
}
