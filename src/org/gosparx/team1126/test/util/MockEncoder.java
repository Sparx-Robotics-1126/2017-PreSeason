package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.EncoderIF;

import edu.wpi.first.wpilibj.tables.ITable;

public class MockEncoder implements EncoderIF{
	// from Encoder
	double distance;
	public double getDistance() {
		return distance;
	}

	double distancePerPulse;
	public void setDistancePerPulse(double _distancePerPulse) {
		distancePerPulse = _distancePerPulse;
	}

	int value;
	public int get() {
		return value;
	}

	boolean reset_Called;
	public void reset() {
		reset_Called = true;
	}	

	// from LiveWindowSendable
	boolean startLiveWindowMode_Called;
	public void startLiveWindowMode() {
		startLiveWindowMode_Called = true;
	}

	boolean stopLiveWindowMode_Called;
	public void stopLiveWindowMode() {
		stopLiveWindowMode_Called = true;
	}

	boolean updateTable_Called;
	public void updateTable() {		
		updateTable_Called = true;
	}

	String getSmartDashboardType_Str;
	public String getSmartDashboardType() {
		return getSmartDashboardType_Str;
	}

	ITable getTable_Tbl;
	public void initTable(ITable _arg0) {		
		getTable_Tbl = _arg0;
	}

	public ITable getTable() {
		return getTable_Tbl;
	}
}
