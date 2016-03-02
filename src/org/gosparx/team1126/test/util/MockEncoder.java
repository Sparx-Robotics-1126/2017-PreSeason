package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.EncoderInterface;

public class MockEncoder implements EncoderInterface {

	public MockEncoder(final int _channelA, final int _channelB) {
	}
	
	@Override
	public double getDistance() {
		// TODO
		return 0;
	}

	@Override
	public void setDistancePerPulse(double distancePerPulse) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int get() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void reset() {
		// TODO Auto-generated method stub
		
	}
}