package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.PowerDistributionPanelIF;

public class MockPowerDistributionPanel implements PowerDistributionPanelIF {
	public double current;
	public double getCurrent(int channel) {
		return current;
	}
}