package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.MagnetSensorIF;

public class MockMagnetSensor implements MagnetSensorIF{
	// from MagnetSensor
	public boolean tripped;
	public boolean isTripped() {
		return tripped;
	}	
}
