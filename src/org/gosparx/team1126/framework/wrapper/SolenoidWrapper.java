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

	@Override
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

	@Override
	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockSolenoid(channel);
		}
	}

	@Override
	public boolean inUnitTest() {
		return inUnitTest;
	}

	@Override
	public void initTable(ITable subtable) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public ITable getTable() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getSmartDashboardType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void updateTable() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void startLiveWindowMode() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void stopLiveWindowMode() {
		// TODO Auto-generated method stub
		
	}
}
