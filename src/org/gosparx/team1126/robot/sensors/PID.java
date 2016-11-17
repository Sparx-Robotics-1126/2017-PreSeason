package org.gosparx.team1126.robot.sensors;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Logic of a Proportional Integral Derivative loop. Must be constructed first,
 * then it must receive continual updates in order to receive accurate output
 * values.
 */
public class PID {
	
	private static double kp;
	
	private static double ki;
	
	private static double error = 0;
	
	private static double integral = 0;
	
	private static double proportional = 0;
	
	private static double totalizer = 0;
	 
	public PID(double kI, double kP){
		ki = kI;
		kp = kP;
	}
    
	public double loop(double speed, double setPoint){
		error = setPoint - speed;
		proportional = error * kp;
		totalizer += error;
		integral = totalizer * ki;
		return (proportional + integral);
	}
   
    
}
