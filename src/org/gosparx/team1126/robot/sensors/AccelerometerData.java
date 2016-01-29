package org.gosparx.team1126.robot.sensors;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 * A class for interpreting the data we receive from the accelerometer
 * @author Alex Mechler {amechler1998@gmail.com} 
 */
public class AccelerometerData {

	/**
	 * The Accelerometer to measure
	 */
	private Accelerometer acc;
	
	/**
	 * The amount we offset the X readings by
	 */
	private double offX;
	
	/**
	 * The amount we offset the Y readings by
	 */
	private double offY;
	
	/**
	 * The amount we offset the Z readings by
	 */
	private double offZ;
	
	/**
	 * The maximum reading we will consider 0
	 */
	private double deadband;
	
	/**
	 * Makes a new Accelerometer data
	 * @param a The accelerometer to measure
	 * @param offX The amount to offset X readings by
	 * @param offY The amount to offset Y readings by
	 * @param offZ The amount to offset Z readings by
	 * @param deadzone The max reading to count as 0
	 */
	public AccelerometerData(Accelerometer a, double offX, double offY, double offZ, double deadzone){
		acc = a;
		this.offX = offX;
		this.offY = offY;
		this.offZ = offZ;
		deadband = deadzone;
	}
		
	/**
	 * @return The offset and deadband corrected X reading
	 */
	public double getX(){
		return Math.abs(acc.getX()-offX) < deadband ? 0 : acc.getX() - offX;
	}
	
	/**
	 * @return The offset and deadband corrected Y reading
	 */
	public double getY(){
		return Math.abs(acc.getY()-offY) < deadband ? 0 : acc.getY() - offY;
	}

	/**
	 * @return The offset and deadband corrected Z reading
	 */
	public double getZ(){
		return Math.abs(acc.getZ()-offZ) < deadband ? 0 : acc.getZ() - offZ;
	}
	
	/**
	 * @return The angle in degrees between the acc and the X axis
	 */
	public double getXAngle(){
		return calcAngle(getZ(), getX());
	}	
	
	/**
	 * @return The angle in degrees between the acc and the Y axis
	 */
	public double getYAngle(){
		return calcAngle(getZ(), getY());
	}	
	
	/**
	 * @return The angle in degrees between the acc and the Z axis
	 */
	public double getZAngle(){
		return calcAngle(getX(), getZ());
	}	
	
	/**
	 * Calculates the arctan and converts to degrees
	 * @param opp The opposite side length
	 * @param adj The adjacent side length
	 * @return
	 */
	private double calcAngle(double opp, double adj){
		return Math.atan((opp/adj)) * (180.0/Math.PI);
	}
}
