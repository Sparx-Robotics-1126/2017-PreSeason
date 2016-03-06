package org.gosparx.team1126.interfaces;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;

public interface EncoderIF extends LiveWindowSendable {
	double getDistance();
	void setDistancePerPulse(double distancePerPulse);
	int get();
	void reset();
}
