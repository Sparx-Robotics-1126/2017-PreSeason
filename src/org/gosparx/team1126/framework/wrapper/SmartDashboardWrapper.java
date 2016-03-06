package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.SmartDashboardIF;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardWrapper implements SmartDashboardIF {
	private static SmartDashboardIF instance = new SmartDashboardWrapper();

	public static SmartDashboardIF getInstance() {
	    return instance;
	}

	private SmartDashboardWrapper() {
	}

	public void putBoolean(String _key, boolean _value) {
		SmartDashboard.putBoolean(_key, _value);
	}
}
