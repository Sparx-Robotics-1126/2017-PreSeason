package org.gosparx.team1126.test.util;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.interfaces.AnalogGyroIF;
import org.gosparx.team1126.interfaces.CANTalonIF;
import org.gosparx.team1126.interfaces.WPI_FactoryIF;
import org.gosparx.team1126.interfaces.DigitalInputIF;
import org.gosparx.team1126.interfaces.DriverStationIF;
import org.gosparx.team1126.interfaces.EncoderDataIF;
import org.gosparx.team1126.interfaces.EncoderIF;
import org.gosparx.team1126.interfaces.MagnetSensorIF;
import org.gosparx.team1126.interfaces.PowerDistributionPanelIF;
import org.gosparx.team1126.interfaces.SmartDashboardIF;
import org.gosparx.team1126.interfaces.SolenoidIF;
import org.gosparx.team1126.interfaces.TimerIF;

public class MockWPI_Factory implements WPI_FactoryIF {
	private Map<Integer, CANTalonIF> canTalons;
	private Map<Integer, SolenoidIF> solenoids;
	private Map<Integer, Map<Integer, EncoderIF> > encoders;
	private Map<Integer, EncoderDataIF> encoderDatas;
	private Map<Integer, MagnetSensorIF> magnetSensors;
	private Map<Integer, DigitalInputIF> digitalInputs;
	private Map<Integer, AnalogGyroIF> analogInputs;

	public MockWPI_Factory() {
		canTalons = new HashMap<Integer, CANTalonIF>();
		solenoids = new HashMap<Integer, SolenoidIF>();
		encoders = new HashMap<Integer, Map<Integer, EncoderIF> >();
		encoderDatas = new HashMap<Integer, EncoderDataIF>();
		magnetSensors = new HashMap<Integer, MagnetSensorIF>();
		digitalInputs = new HashMap<Integer, DigitalInputIF>();
    }

	public void setFactory(WPI_FactoryIF _fact) {		
	}

	public CANTalonIF getCANTalon(int _id) {
		if (!canTalons.containsKey(_id)) {
			canTalons.put(_id, new MockCANTalon());
	    }
		return canTalons.get(_id);
	}

	public SolenoidIF getSolenoid(int _channel) {
		if (!solenoids.containsKey(_channel)) {
			solenoids.put(_channel, new MockSolenoid());
	    }
		return solenoids.get(_channel);
	}

	public EncoderIF getEncoder(int _aChannel, int _bChannel) {
		if (!encoders.containsKey(_aChannel) ||
			!encoders.get(_aChannel).containsKey(_bChannel)) {
			Map<Integer, EncoderIF> innerM = new HashMap<Integer, EncoderIF>();
    		innerM.put(_bChannel, new MockEncoder());
    		encoders.put(_aChannel, innerM);
	    }
		return encoders.get(_aChannel).get(_bChannel);
	}

	public EncoderIF getEncoder(DigitalInputIF leftA, DigitalInputIF leftB) {
		int encoderId1 = java.lang.System.identityHashCode(leftA);
		int encoderId2 = java.lang.System.identityHashCode(leftB);
		if (!encoders.containsKey(encoderId1) ||
			!encoders.get(encoderId1).containsKey(encoderId2)) {
			Map<Integer, EncoderIF> innerM = new HashMap<Integer, EncoderIF>();
    		innerM.put(encoderId2, new MockEncoder());
    		encoders.put(encoderId1, innerM);
	    }
		return encoders.get(encoderId1).get(encoderId2);
	}

	public EncoderDataIF getEncoderData(EncoderIF _encoder, double _distPerTick) {
		int encoderId = java.lang.System.identityHashCode(_encoder);
		if (!encoderDatas.containsKey(encoderId)) {
			encoderDatas.put(encoderId, new MockEncoderData());
	    }
		return encoderDatas.get(encoderId);
	}

	public MagnetSensorIF getMagnetSensor(int _channel, boolean _inverse) {
		if (!magnetSensors.containsKey(_channel)) {
			magnetSensors.put(_channel, new MockMagnetSensor());
	    }
		return magnetSensors.get(_channel);
	}

	public DigitalInputIF getDigitalInput(int _channel) {
		if (!digitalInputs.containsKey(_channel)) {
			digitalInputs.put(_channel, new MockDigitalInput());
	    }
		return digitalInputs.get(_channel);
	}

	public DriverStationIF getDriverStation() {
		return MockDriverStation.getInstance();
	}

	public PowerDistributionPanelIF createPowerDistributionPanel() {
		return new MockPowerDistributionPanel();
	}

	public SmartDashboardIF getSmartDashboard() {
		return MockSmartDashboard.getInstance();
	}

	public TimerIF getTimer() {
		return MockTimer.getInstance();
	}

	public AnalogGyroIF getAnalogGyro(int _channel) {
		if (!analogInputs.containsKey(_channel)) {
			analogInputs.put(_channel, new MockAnalogGyro());
	    }
		return analogInputs.get(_channel);
	}
}
