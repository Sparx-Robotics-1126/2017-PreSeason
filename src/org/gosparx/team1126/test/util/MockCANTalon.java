package org.gosparx.team1126.test.util;

import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.tables.ITable;

public class MockCANTalon implements CANSpeedController{

	/**
    * Constructor for the CANTalon device.
    * @param deviceNumber The CAN ID of the Talon SRX
    */
    public MockCANTalon(int deviceNumber) {
    }
	
	
	public double get() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public void set(double speed, byte syncGroup) {
		// TODO Auto-generated method stub
		
	}

	
	public void set(double speed) {
		// TODO Auto-generated method stub
		
	}

	
	public void setInverted(boolean isInverted) {
		// TODO Auto-generated method stub
		
	}

	
	public boolean getInverted() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public void disable() {
		// TODO Auto-generated method stub
		
	}

	
	public void stopMotor() {
		// TODO Auto-generated method stub
		
	}

	
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}

	
	public void setPID(double p, double i, double d) {
		// TODO Auto-generated method stub
		
	}

	
	public double getP() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getI() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getD() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public void setSetpoint(double setpoint) {
		// TODO Auto-generated method stub
		
	}

	
	public double getSetpoint() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getError() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public void enable() {
		// TODO Auto-generated method stub
		
	}

	
	public boolean isEnabled() {
		// TODO Auto-generated method stub
		return false;
	}

	
	public void reset() {
		// TODO Auto-generated method stub
		
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

	
	public ControlMode getControlMode() {
		// TODO Auto-generated method stub
		return null;
	}

	
	public void setControlMode(int mode) {
		// TODO Auto-generated method stub
		
	}

	
	public void setP(double p) {
		// TODO Auto-generated method stub
		
	}

	
	public void setI(double i) {
		// TODO Auto-generated method stub
		
	}

	
	public void setD(double d) {
		// TODO Auto-generated method stub
		
	}

	
	public double getBusVoltage() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getOutputVoltage() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getOutputCurrent() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getTemperature() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getPosition() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public double getSpeed() {
		// TODO Auto-generated method stub
		return 0;
	}

	
	public void setVoltageRampRate(double rampRate) {
		// TODO Auto-generated method stub
		
	}

	
}
