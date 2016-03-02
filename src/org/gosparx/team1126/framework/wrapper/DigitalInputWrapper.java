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

	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockDigitalInput(channel);
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
