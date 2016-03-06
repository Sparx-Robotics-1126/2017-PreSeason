package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.DriverStationIF;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriverStationWrapper implements DriverStationIF {
	private static DriverStationIF instance = new DriverStationWrapper();

	public static DriverStationIF getInstance() {
	    return instance;
	}

	private DriverStationWrapper() {
	}

	public boolean isDisabled() {
		return DriverStation.getInstance().isDisabled();
	}

	public boolean isEnabled() {
		return DriverStation.getInstance().isEnabled();
	}

	public boolean isOperatorControl() {
		return DriverStation.getInstance().isOperatorControl();
	}

	public boolean isAutonomous() {
		return DriverStation.getInstance().isAutonomous();
	}

	public boolean isTest() {
		return DriverStation.getInstance().isTest();
	}

	public boolean isFMSAttached() {
		return DriverStation.getInstance().isFMSAttached();
	}

	public Alliance getAlliance() {
		return DriverStation.getInstance().getAlliance();
	}
}
