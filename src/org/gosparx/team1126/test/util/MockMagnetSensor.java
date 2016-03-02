package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.MagnetSensorInterface;

public class MockMagnetSensor implements MagnetSensorInterface{

	public MockMagnetSensor(int port, boolean inverse) {
    }

	public boolean isTripped() {
		// TODO Auto-generated method stub
		return false;
	}
}
