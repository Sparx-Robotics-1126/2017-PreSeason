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
	
	private static boolean stopFunction = false;
	
	private static double kf = 0;
	
	private static double feedForward = 0;
	 
	public PID(double kI, double kP){
		ki = kI;
		kp = kP;
	}
	
	public PID(double kI, double kP, double ff){
		ki = kI;
		kp = kP;
		kf = ff;
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
		if(integral > 1){
			totalizer = 1.0/ki;
		}else if(integral < -1){
			totalizer = -1.0/ki;
		}
		feedForward = kf * setPoint;
		pastTime = currentTime;
		if((setPoint == 0) && (stopFunction == true)){
			totalizer = 0;
			return 0;
		}
		return (proportional + integral + feedForward);
	}
	
	public void breakMode(boolean condition){
		stopFunction = condition;
	}
   
    public void pidconstants (double newkp, double newki)
    {
    	kp = newkp;
    	ki = newki;
    }

}
