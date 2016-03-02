package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.DigitalInputInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockDigitalInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

public class DigitalInputWrapper implements DigitalInputInterface, UnitTestInterface, LiveWindowSendable {

	private int channel;
	private boolean inUnitTest;
	private DigitalInputInterface mock;
	private DigitalInput real;

	public DigitalInputWrapper(final int _channel) {
		channel = _channel;
		inUnitTest = false;
	}

	@Override
	public boolean get() {
		if(inUnitTest) {
			return mock.get();
		} else {
			if(real == null) {
				real = new DigitalInput(channel);
			}
			return real.get();
		}
	}

	@Override
	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockDigitalInput(channel);
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
