package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.SmartDashboardIF;

public class MockSmartDashboard implements SmartDashboardIF {
	private static SmartDashboardIF instance = new MockSmartDashboard();

	public static SmartDashboardIF getInstance() {
	    return instance;
	}

	private MockSmartDashboard() {
	}

    public String key;
    public boolean value;
	public void putBoolean(String _key, boolean _value) {
		key = _key;
		value = _value;
	}
}
