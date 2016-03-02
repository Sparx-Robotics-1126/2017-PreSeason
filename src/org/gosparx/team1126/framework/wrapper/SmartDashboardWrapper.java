package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.SmartDashboardInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockSmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardWrapper implements SmartDashboardInterface, UnitTestInterface  {

	private boolean inUnitTest;
	private SmartDashboardInterface mock;
	private static SmartDashboardInterface instance = new SmartDashboardWrapper();

	public static SmartDashboardInterface getInstance() {
	    return SmartDashboardWrapper.instance;
	}

	private SmartDashboardWrapper() {
		inUnitTest = false;
	}

	public void putBoolean(String _key, boolean _value) {
		if(inUnitTest) {
			mock.putBoolean(_key, _value);
		} else {
			SmartDashboard.putBoolean(_key, _value);
		}
	}

	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockSmartDashboard();
		}
	}

	public boolean inUnitTest() {
		return inUnitTest;
	}
}
