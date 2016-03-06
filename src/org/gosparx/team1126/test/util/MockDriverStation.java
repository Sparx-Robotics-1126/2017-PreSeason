package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.DriverStationIF;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MockDriverStation implements DriverStationIF {
	private static DriverStationIF instance = new MockDriverStation();

	public static DriverStationIF getInstance() {
	    return instance;
	}

	private MockDriverStation() {
	}

	public boolean disabled;
	public boolean isDisabled() {
		return disabled;
	}
	
	public boolean enabled;
	public boolean isEnabled() {
		return enabled;
	}

	public boolean operatorControl;	
	public boolean isOperatorControl() {
		return operatorControl;
	}

	public boolean autonomous;		
	public boolean isAutonomous() {
		return autonomous;
	}

	public boolean test;			
	public boolean isTest() {
		return test;
	}

	public boolean fmsAttached;				
	public boolean isFMSAttached() {
		return fmsAttached;
	}

	public Alliance alliance;					
	public Alliance getAlliance() {
		return alliance;
	}
}
