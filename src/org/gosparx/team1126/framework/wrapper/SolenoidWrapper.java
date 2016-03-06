package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.SolenoidIF;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.tables.ITable;

public class SolenoidWrapper implements SolenoidIF {

	private Solenoid device;

	public SolenoidWrapper(int _channel) {
		device = new Solenoid(_channel);
	}

	// from Solenoid
	public void set(boolean _value) {
		device.set(_value);
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
