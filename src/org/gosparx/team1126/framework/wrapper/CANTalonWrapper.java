package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.CANTalonIF;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.tables.ITable;

public class CANTalonWrapper implements CANTalonIF {

	private CANTalon device;

	public CANTalonWrapper(int _deviceNumber) {
		device = new CANTalon(_deviceNumber);
	}

	// from CANTalon
	public void set(double _speed) {
		device.set(_speed);
	}

	public double get() {
		return device.get();
	}

	public void setInverted(boolean _value) {
		device.setInverted(_value);
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
