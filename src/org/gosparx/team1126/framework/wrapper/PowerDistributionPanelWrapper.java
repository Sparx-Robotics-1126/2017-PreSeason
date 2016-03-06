package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.PowerDistributionPanelIF;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class PowerDistributionPanelWrapper implements PowerDistributionPanelIF {
	private static PowerDistributionPanel pdp = new PowerDistributionPanel();
	public double getCurrent(int _channel) {
		return pdp.getCurrent(_channel);
	}
	
}
