package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.SolenoidInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

public class SolenoidWrapper implements SolenoidInterface, UnitTestInterface, LiveWindowSendable {

	private int channel;
	private boolean inUnitTest;
	private SolenoidInterface mock;
	private Solenoid real;

	public SolenoidWrapper(final int _channel) {
		channel = _channel;
		inUnitTest = false;
	}

	public void set(boolean _value) {
		if(inUnitTest) {
			mock.set(_value);
		} else {
			if(real == null) {
				real = new Solenoid(channel);
			}
			real.set(_value);
		}
	}

	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockSolenoid(channel);
		}
	}

	public boolean inUnitTest() {
		return inUnitTest;
	}

	public void initTable(ITable subtable) {
		// TODO Auto-generated method stub
		
	}

	public ITable getTable() {
		// TODO Auto-generated method stub
		return null;
	}

	public String getSmartDashboardType() {
		// TODO Auto-generated method stub
		return null;
	}

	public void updateTable() {
		// TODO Auto-generated method stub
		
	}

	public void startLiveWindowMode() {
		// TODO Auto-generated method stub
		
	}

	public void stopLiveWindowMode() {
		// TODO Auto-generated method stub
		
	}
}
