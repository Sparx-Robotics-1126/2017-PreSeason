package org.gosparx.team1126.test.util;

import org.gosparx.team1126.interfaces.DigitalInputIF;

import edu.wpi.first.wpilibj.tables.ITable;

public class MockDigitalInput implements DigitalInputIF{
	// from DigitalInput
	public boolean value;
	public boolean get() {
		return value;
	}

	// from LiveWindowSendable
	public boolean startLiveWindowMode_Called;
	public void startLiveWindowMode() {
		startLiveWindowMode_Called = true;
	}

	public boolean stopLiveWindowMode_Called;
	public void stopLiveWindowMode() {
		stopLiveWindowMode_Called = true;
	}

	public boolean updateTable_Called;
	public void updateTable() {		
		updateTable_Called = true;
	}

	public String getSmartDashboardType_Str;
	public String getSmartDashboardType() {
		return getSmartDashboardType_Str;
	}

	public ITable getTable_Tbl;
	public void initTable(ITable _arg0) {		
		getTable_Tbl = _arg0;
	}

	public ITable getTable() {
		return getTable_Tbl;
	}	
}
