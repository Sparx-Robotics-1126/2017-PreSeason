package org.gosparx.team1126.interfaces;

public interface WPI_FactoryIF {
	void setFactory(WPI_FactoryIF _fact);
	CANTalonIF getCANTalon(int _id);
	SolenoidIF getSolenoid(int _channel);
	EncoderIF getEncoder(int _aChannel, int _bChannel);
	EncoderIF getEncoder(DigitalInputIF leftA, DigitalInputIF leftB);
	EncoderDataIF getEncoderData(EncoderIF _encoder, double _distPerTick);
	MagnetSensorIF getMagnetSensor(int _channel, boolean _inverse);
	DigitalInputIF getDigitalInput(int _channel);
	DriverStationIF getDriverStation();
	PowerDistributionPanelIF createPowerDistributionPanel();
	SmartDashboardIF getSmartDashboard();
	TimerIF getTimer();
	AnalogGyroIF getAnalogGyro(int _channel);
}
