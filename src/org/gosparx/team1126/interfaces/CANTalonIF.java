package org.gosparx.team1126.interfaces;

import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;

public interface CANTalonIF extends LiveWindowSendable {
	void set(double speed);
	double get();
	void setInverted(boolean b);
}
