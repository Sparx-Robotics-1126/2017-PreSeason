package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.AnalogGyroIF;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.tables.ITable;

public class AnalogGyroWrapper  implements AnalogGyroIF {
	private AnalogGyro device;

	public AnalogGyroWrapper(int _channel) {
		device = new AnalogGyro(_channel);
	}

	// from AnalogGyro
	public void calibrate() {
		device.calibrate();
	}

	public double getAngle() {
		return device.getAngle();
	}

	public void reset() {
		device.reset();
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
