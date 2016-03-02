package org.gosparx.team1126.framework.wrapper;

import org.gosparx.team1126.interfaces.EncoderInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

public class EncoderWrapper implements EncoderInterface, UnitTestInterface, LiveWindowSendable {

	private int channelA;
	private int channelB;
	private boolean inUnitTest;
	private EncoderInterface mock;
	private Encoder real;

	public EncoderWrapper(final int _channelA, final int _channelB) {
		channelA = _channelA;
		channelB = _channelB;
		inUnitTest = false;
	}

	@Override
	public double getDistance() {
		if(inUnitTest) {
			return mock.getDistance();
		} else {
			if(real == null) {
				real = new Encoder(channelA, channelB);
			}
			return real.getDistance();
		}
	}

	@Override
	public void setDistancePerPulse(double _distancePerPulse) {
		if(inUnitTest) {
			mock.setDistancePerPulse(_distancePerPulse);
		} else {
			if(real == null) {
				real = new Encoder(channelA, channelB);
			}
			real.setDistancePerPulse(_distancePerPulse);
		}
	}

	@Override
	public int get() {
		if(inUnitTest) {
			return mock.get();
		} else {
			if(real == null) {
				real = new Encoder(channelA, channelB);
			}
			return real.get();
		}
	}

	@Override
	public void reset() {
		if(inUnitTest) {
			mock.reset();
		} else {
			if(real == null) {
				real = new Encoder(channelA, channelB);
			}
			real.reset();
		}
	}

	@Override
	public void SetToUnitTest() {
		inUnitTest = true;
		if(mock == null)
		{
			mock = new MockEncoder(channelA, channelB);
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
