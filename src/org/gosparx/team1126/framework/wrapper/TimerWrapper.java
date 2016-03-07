package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.TimerIF;

import edu.wpi.first.wpilibj.Timer;

public class TimerWrapper implements TimerIF {
	private static TimerIF instance = new TimerWrapper();

	public static TimerIF getInstance() {
	    return instance;
	}

	private TimerWrapper() {
	}

	public double getFPGATimestamp() {
		return Timer.getFPGATimestamp();
	}
}
