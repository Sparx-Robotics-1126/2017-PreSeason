package org.gosparx.team1126.test.util;

import edu.wpi.first.wpilibj.Timer.Interface;
import edu.wpi.first.wpilibj.Timer.StaticInterface;

public class MockTimer implements StaticInterface{

	@Override
	public double getFPGATimestamp() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getMatchTime() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void delay(double seconds) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public Interface newTimer() {
		// TODO Auto-generated method stub
		return null;
	}

}
