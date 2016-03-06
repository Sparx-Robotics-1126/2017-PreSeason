package org.gosparx.team1126.robot.util;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.framework.wrapper.CANTalonWrapper;
import org.gosparx.team1126.framework.wrapper.DigitalInputWrapper;
import org.gosparx.team1126.framework.wrapper.DriverStationWrapper;
import org.gosparx.team1126.framework.wrapper.EncoderWrapper;
import org.gosparx.team1126.framework.wrapper.PowerDistributionPanelWrapper;
import org.gosparx.team1126.framework.wrapper.SmartDashboardWrapper;
import org.gosparx.team1126.framework.wrapper.SolenoidWrapper;
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
import org.gosparx.team1126.robot.sensors.EncoderData;
import org.gosparx.team1126.robot.sensors.MagnetSensor;

public class WPI_Factory implements WPI_FactoryIF {
	private static WPI_FactoryIF instance = new WPI_Factory();
	private Map<Integer, CANTalonIF> canTalons;
	private Map<Integer, SolenoidIF> solenoids;
	private Map<Integer, Map<Integer, EncoderIF> > encoders;
	private Map<Integer, EncoderDataIF> encoderDatas;
	private Map<Integer, MagnetSensorIF> magnetSensors;
	private Map<Integer, DigitalInputIF> digitalInputs;

	public static WPI_FactoryIF getInstance() {
	    return instance;
	}

	public void setFactory(WPI_FactoryIF _fact) {
		instance = _fact;
	}

	private WPI_Factory() {
		canTalons = new HashMap<Integer, CANTalonIF>();
		solenoids = new HashMap<Integer, SolenoidIF>();
		encoders = new HashMap<Integer, Map<Integer, EncoderIF> >();
		encoderDatas = new HashMap<Integer, EncoderDataIF>();
		magnetSensors = new HashMap<Integer, MagnetSensorIF>();
		digitalInputs = new HashMap<Integer, DigitalInputIF>();
    }

	public CANTalonIF getCANTalon(int _id) {
		if (!canTalons.containsKey(_id)) {
			canTalons.put(_id, new CANTalonWrapper(_id));
	    }
		return canTalons.get(_id);
	}

	public SolenoidIF getSolenoid(int _channel) {
		if (!solenoids.containsKey(_channel)) {
			solenoids.put(_channel, new SolenoidWrapper(_channel));
	    }
		return solenoids.get(_channel);
	}

	public EncoderIF getEncoder(int _aChannel, int _bChannel) {
		if (!encoders.containsKey(_aChannel) ||
			!encoders.get(_aChannel).containsKey(_bChannel)) {
			Map<Integer, EncoderIF> innerM = new HashMap<Integer, EncoderIF>();
    		innerM.put(_bChannel, new EncoderWrapper(_aChannel, _bChannel));
    		encoders.put(_aChannel, innerM);
	    }
		return encoders.get(_aChannel).get(_bChannel);
	}

	public EncoderDataIF getEncoderData(EncoderIF _encoder, double _distPerTick) {
		int encoderId = java.lang.System.identityHashCode(_encoder);
		if (!encoderDatas.containsKey(encoderId)) {
			encoderDatas.put(encoderId, new EncoderData(_encoder, _distPerTick));
	    }
		return encoderDatas.get(encoderId);
	}

	public MagnetSensorIF getMagnetSensor(int _channel, boolean _inverse) {
		if (!magnetSensors.containsKey(_channel)) {
			magnetSensors.put(_channel, new MagnetSensor(getDigitalInput(_channel), _inverse));
	    }
		return magnetSensors.get(_channel);
	}

	public DigitalInputIF getDigitalInput(int _channel) {
		if (!digitalInputs.containsKey(_channel)) {
			digitalInputs.put(_channel, new DigitalInputWrapper(_channel));
	    }
		return digitalInputs.get(_channel);
	}

	public DriverStationIF getDriverStation() {
		return DriverStationWrapper.getInstance();
	}

	public PowerDistributionPanelIF createPowerDistributionPanel() {
		return new PowerDistributionPanelWrapper();
	}

	public SmartDashboardIF getSmartDashboard() {
		return SmartDashboardWrapper.getInstance();
	}
}
