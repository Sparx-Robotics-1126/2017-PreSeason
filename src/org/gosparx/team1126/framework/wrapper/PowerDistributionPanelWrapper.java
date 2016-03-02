package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.PowerDistributionPanelInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockPowerDistributionPanel;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerDistributionPanelWrapper implements PowerDistributionPanelInterface, UnitTestInterface  {

	private boolean inUnitTest;
	private PowerDistributionPanelInterface mock;
	private PowerDistributionPanel real;
	private static PowerDistributionPanelInterface instance = new PowerDistributionPanelWrapper();

	public static PowerDistributionPanelInterface getInstance() {
	    return PowerDistributionPanelWrapper.instance;
	}

	private PowerDistributionPanelWrapper() {
		inUnitTest = false;
	}

	public double getCurrent(int _channel) {
		if(inUnitTest) {
			return mock.getCurrent(_channel);
		} else {
			if(real == null) {
				real = new PowerDistributionPanel();
			}
			return real.getCurrent(_channel);
		}
	}

	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockPowerDistributionPanel();
		}
	}

	public boolean inUnitTest() {
		return inUnitTest;
	}
}
