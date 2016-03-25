package org.gosparx.team1126.interfaces;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface DriverStationIF {
	boolean isDisabled();
	boolean isEnabled();
	boolean isOperatorControl();
	boolean isAutonomous();
	boolean isTest();
	boolean isFMSAttached();
	Alliance getAlliance();
}