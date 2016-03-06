package org.gosparx.team1126.robot.sensors;

import org.gosparx.team1126.interfaces.DigitalInputIF;
import org.gosparx.team1126.interfaces.MagnetSensorIF;

/**
 * A class for interpreting the data from Magnetic Limit Switch
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class MagnetSensor implements MagnetSensorIF {

	/**
	 * The digital input for the Magnetic Sensor
	 */
	public DigitalInputIF in;
	
	/**
	 * Is the output inversed
	 */
	private boolean inversed;
	/**
	 * Creates a new magnetic sensor
	 * @param dio - The digitalinput the sensor is in
	 * @param inverse - do we inverse the output
	 */
	public MagnetSensor(DigitalInputIF dio, boolean inverse){
		in = dio;
		inversed = inverse;
	}

	/**
	 * @return if the sensor is tripped.
	 */
	public boolean isTripped(){
		if(inversed)
			return !in.get();
		return in.get();
	}
}