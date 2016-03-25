package org.gosparx.team1126.robot.subsystem;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.sensors.PressureSensor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ScalingNew extends GenericSubsystem {

	private static ScalingNew scalingNew;
	
	//private DoubleSolenoid pto;
	private Solenoid forward;
	private Solenoid reverse;
	private Solenoid arms;
	private Drives drives;
	private PressureSensor press;
	private boolean firstLoop;
	
	public static synchronized ScalingNew getInstance(){
		if(scalingNew == null){
			scalingNew = new ScalingNew();
		}
		return scalingNew;
	}
	
	private ScalingNew() {
		super("Scaling2.0", Thread.NORM_PRIORITY);
	}

	@Override
	protected boolean init() {
		drives = Drives.getInstance();
		//press = new PressureSensor(IO.ANALOG_IN_PNU_PRESSURE_SENSOR);
		//pto = new DoubleSolenoid(5, 7);
		forward = new Solenoid(5);
		reverse = new Solenoid(7);
		arms = new Solenoid(6);
		forward.set(false);
		reverse.set(true);
		firstLoop = true;
		return true;
	}

	@Override
	protected void liveWindow() {

	}

	@Override
	protected boolean execute() {
		//SmartDashboard.putNumber("Pressue", press.getPressure());
		if(firstLoop && ds.isEnabled()){
			forward.set(false);
			reverse.set(false);
			firstLoop = false;
		}
		return false;
	}

	@Override
	protected long sleepTime() {
		return 100;
	}

	@Override
	protected void writeLog() {
		
	}
	
	public void armsUp(){
		BallAcqNew.getInstance().flipperScale();
		Timer.delay(.25);
		arms.set(true);
	}
	
	public void scale(){
		reverse.set(false);
		forward.set(true);
		Timer.delay(.5);
		drives.scale();
	}
	
	public void estop(){
		drives.autoEStop();
	}
	
	public void armsDown(){
		arms.set(false);
	}
}
