package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.EncoderDataIF;

public class MockEncoderData implements EncoderDataIF{
	// from EncoderData
	public double distance;
	public double getDistance() {
		return distance;
	}

	public boolean calculateSpeedCalled;
	public void calculateSpeed() {
		calculateSpeedCalled = true;
	}

	public double getSpeedValue;
	public double getSpeed() {
		return getSpeedValue;
	}	
}
