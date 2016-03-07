package org.gosparx.team1126.test;

import static org.junit.Assert.*;

import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.subsystem.BallAcqNew;
import org.gosparx.team1126.robot.util.WPI_Factory;
import org.gosparx.team1126.test.util.MockCANTalon;
import org.gosparx.team1126.test.util.MockDigitalInput;
import org.gosparx.team1126.test.util.MockEncoder;
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
    private MockMagnetSensor armHomeSwitchL;
    private MockMagnetSensor armHomeSwitchR;
    private MockMagnetSensor armStopSwitchL;
    private MockMagnetSensor armStopSwitchR;
    private MockDigitalInput ballFullyIn;
    private double expectedArmSpeedL;
    private double expectedArmSpeedR;
    private double expectedRollerSpeedL;
    private double expectedRollerSpeedR;
    private boolean expectedFlipperFired;

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
		armHomeSwitchL = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME_L, true);
		armHomeSwitchR = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_HOME_R, true);
		armStopSwitchL = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_STOP_L, true);
		armStopSwitchR = (MockMagnetSensor) WPI_Factory.getInstance().getMagnetSensor(IO.DIO_MAG_ACQ_SHOULDER_STOP_R, true);
		ballFullyIn = (MockDigitalInput) WPI_Factory.getInstance().getDigitalInput(IO.DIO_PHOTO_BALL_IN);

		// test we didn't turn anything ON in init
		testOutputs();
    }

    void testOutputs() {
    	assertEquals(expectedArmSpeedR, armMotorRight.speed, 0.001);
		assertEquals(expectedArmSpeedL, armMotorLeft.speed, 0.001);
		assertEquals(expectedRollerSpeedR, rollerMotorRight.speed, 0.001);
		assertEquals(expectedRollerSpeedL, rollerMotorLeft.speed, 0.001);
		assertEquals(expectedFlipperFired, flipper.value);
    }

	@Test
	public void testPowerUp_NotHome_NotAtLimit_NoBall()  {
		System.out.println("testPowerUp_NotHome");
		
		// inputs before execute are set to default which is not home
		boolean success = invokePrivateMethod(testSubject, "execute", null);
		// only testing execute return once, it is hardcoded to false in acqui
		assertEquals(success, false);
		// check outputs
		expectedArmSpeedL = 0.3;
		expectedArmSpeedR = -expectedArmSpeedL;
		testOutputs();

		// left got home
		armHomeSwitchL.tripped = true;
		invokePrivateMethod(testSubject, "execute", null);
		expectedArmSpeedL = 0;
		testOutputs();

		// left coasted over
		armHomeSwitchL.tripped = false;
		invokePrivateMethod(testSubject, "execute", null);
		testOutputs();

		// right got home
		armHomeSwitchR.tripped = true;
		invokePrivateMethod(testSubject, "execute", null);
		expectedArmSpeedR = 0;
		testOutputs();

		// right coasted over
		armHomeSwitchR.tripped = false;
		invokePrivateMethod(testSubject, "execute", null);
		testOutputs();

		// left coasted to stop
		armStopSwitchL.tripped = true;
		invokePrivateMethod(testSubject, "execute", null);
		testOutputs();

		// right coasted to stop
		armStopSwitchR.tripped = true;
		invokePrivateMethod(testSubject, "execute", null);
		testOutputs();
	}
}
