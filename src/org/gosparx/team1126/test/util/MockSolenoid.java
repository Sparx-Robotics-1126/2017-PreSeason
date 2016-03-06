package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.SolenoidIF;

import edu.wpi.first.wpilibj.tables.ITable;

public class MockSolenoid implements SolenoidIF{
	// from Solenoid
	boolean value;
	public void set(boolean _value) {
		value = _value;
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