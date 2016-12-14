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
	
	private static double currentTime = 0;
	

	private static double pastTime = 0;
	 
	public PID(double kI, double kP){
		ki = kI;
		kp = kP;
	}
    
	public double loop(double speed, double setPoint){
		double ellapsedTime;
		currentTime = (double)(System.currentTimeMillis())/1000;
		ellapsedTime = currentTime - pastTime;
		if(ellapsedTime > .1)
			ellapsedTime = .1;
		error = setPoint - speed;
		proportional = error * kp;
		totalizer += error * (ellapsedTime);
		integral = totalizer * ki;
		pastTime = currentTime;
		return (proportional + integral);
	}
   
    public void pidconstants (double newkp, double newki)
    {
    	kp = newkp;
    	ki = newki;
    }

}
