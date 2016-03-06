package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.DigitalInputIF;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.tables.ITable;

public class DigitalInputWrapper implements DigitalInputIF {

	private DigitalInput device;

	public DigitalInputWrapper(int _channel) {
		device = new DigitalInput(_channel);
	}

	// from DigitalInput
	public boolean get() {
		return device.get();
	}

	// from LiveWindowSendable
	public void startLiveWindowMode() {
		device.startLiveWindowMode();
	}

	public void stopLiveWindowMode() {
		device.stopLiveWindowMode();
	}

	public void updateTable() {		
		device.updateTable();
	}

	public String getSmartDashboardType() {
		return device.getSmartDashboardType();
	}

	public ITable getTable() {
		return device.getTable();
	}

	public void initTable(ITable _arg0) {		
		device.initTable(_arg0);
	}
}
