package org.gosparx.team1126.interfaces;

public interface WPI_FactoryIF {
	void setFactory(WPI_FactoryIF _fact);
	CANTalonIF getCANTalon(int _id);
	SolenoidIF getSolenoid(int _channel);
	EncoderIF getEncoder(int _aChannel, int _bChannel);
	EncoderDataIF getEncoderData(EncoderIF _encoder, double _distPerTick);
	MagnetSensorIF getMagnetSensor(int _channel, boolean _inverse);
	DigitalInputIF getDigitalInput(int _channel);
	DriverStationIF getDriverStation();
	PowerDistributionPanelIF createPowerDistributionPanel();
	SmartDashboardIF getSmartDashboard();
}
