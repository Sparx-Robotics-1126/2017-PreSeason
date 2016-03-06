package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.EncoderIF;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.tables.ITable;

public class EncoderWrapper implements EncoderIF {

	private Encoder device;

	public EncoderWrapper(int _aChannel, int _bChannel) {
		device = new Encoder(_aChannel, _bChannel);
	}

	// from Encoder
	public double getDistance() {
		return device.getDistance();
	}

	public void setDistancePerPulse(double _distancePerPulse) {
		device.setDistancePerPulse(_distancePerPulse);
	}

	public int get() {
		return device.get();
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
