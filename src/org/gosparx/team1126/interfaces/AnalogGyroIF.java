package org.gosparx.team1126.interfaces;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;

public interface AnalogGyroIF extends LiveWindowSendable {
	void calibrate();
	double getAngle();
	void reset();
}
