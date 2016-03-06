package org.gosparx.team1126.test;

import static org.junit.Assert.*;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.subsystem.BallAcqNew;
import org.gosparx.team1126.robot.util.WPI_Factory;
import org.gosparx.team1126.test.util.MockCANTalon;
import org.gosparx.team1126.test.util.MockDigitalInput;
import org.gosparx.team1126.test.util.MockEncoder;
import org.gosparx.team1126.test.util.MockEncoderData;
import org.gosparx.team1126.test.util.MockMagnetSensor;
import org.gosparx.team1126.test.util.MockSolenoid;
import org.gosparx.team1126.test.util.TestBase;
import org.junit.Test;

public class BallAcqNewTest extends TestBase{
	private static BallAcqNew testSubject = BallAcqNew.getInstance();
	private MockCANTalon armMotorRight;
    private MockCANTalon armMotorLeft;
    private MockCANTalon rollerMotorRight;
    private MockCANTalon rollerMotorLeft;
    private MockSolenoid flipper;
    private MockEncoder armEncoderRight;
    private MockEncoder armEncoderLeft;
    private MockEncoderData armEncoderDataR;
    private MockEncoderData armEncoderDataL;
    private MockMagnetSensor armHomeSwitchL;
    private MockMagnetSensor armHomeSwitchR;
    private MockMagnetSensor armStopSwitchL;
    private MockMagnetSensor armStopSwitchR;
    private MockDigitalInput ballEntered;
    private MockDigitalInput ballFullyIn;

	// called only once
    public void setupInternal() {
		System.out.println("Intializing");
    	boolean success = invokePrivateMethod(testSubject, "init", null);
		assertEquals(success, false);
		armMotorRight = (MockCANTalon) WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_SHOULDER_R);
		armMotorLeft = (MockCANTalon) WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_SHOULDER_L);
		rollerMotorRight = (MockCANTalon) WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorLeft = (MockCANTalon) WPI_Factory.getInstance().getCANTalon(IO.CAN_ACQ_ROLLERS_L);
		flipper = (MockSolenoid) WPI_Factory.getInstance().getSolenoid(IO.PNU_FLIPPER_RELEASE);
		armEncoderRight = (MockEncoder) WPI_Factory.getInstance().getEncoder(IO.DIO_SHOULDER_ENC_RIGHT_A, IO.DIO_SHOULDER_ENC_RIGHT_B);
		armEncoderLeft = (MockEncoder) WPI_Factory.getInstance().getEncoder(IO.DIO_SHOULDER_ENC_LEFT_A, IO.DIO_SHOULDER_ENC_LEFT_B);
		armEncoderDataR = (MockEncoderData) WPI_Factory.getInstance().getEncoderData(armEncoderRight, 0);
		armEncoderDataL = (MockEncoderData) WPI_Factory.getInstance().getEncoderData(armEncoderLeft, 0);
		armHomeSwitchL = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME_L, true);
		armHomeSwitchR = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME_R, true);
		armStopSwitchL = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_STOP_L, true);
		armStopSwitchR = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_STOP_R, true);
		ballEntered = (MockDigitalInput) WPI_Factory.getInstance().getDigitalInput(IO.DIO_PHOTO_BALL_ACQ);
		ballFullyIn = (MockDigitalInput) WPI_Factory.getInstance().getDigitalInput(IO.DIO_PHOTO_BALL_IN);
	}

	@Test
	public void testPowerUp_NotHome()  {
		System.out.println("testPowerUp_NotHome");
		boolean success = invokePrivateMethod(testSubject, "execute", null);
		assertEquals(success, false);
	}
}
