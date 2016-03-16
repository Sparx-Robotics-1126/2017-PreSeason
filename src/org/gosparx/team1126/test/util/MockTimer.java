package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.TimerIF;

public class MockTimer implements TimerIF {
	private static TimerIF instance = new MockTimer();

	public static TimerIF getInstance() {
	    return instance;
	}

	private MockTimer() {
	}

	public double fpgaTimestamp;
	public double getFPGATimestamp() {
		return fpgaTimestamp;
	}

	public double delayValue;
	public void delay(double _value) {
		delayValue = _value;
	}
}
