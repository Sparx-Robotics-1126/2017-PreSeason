package org.gosparx.team1126.test;

import static org.junit.Assert.*;
import java.lang.reflect.InvocationTargetException;
import org.gosparx.team1126.framework.factories.CANTalonFactory;
import org.gosparx.team1126.framework.factories.DigitalInputFactory;
import org.gosparx.team1126.framework.factories.EncoderDataFactory;
import org.gosparx.team1126.framework.factories.EncoderFactory;
import org.gosparx.team1126.framework.factories.MagnetSensorFactory;
import org.gosparx.team1126.framework.factories.SolenoidFactory;
import org.gosparx.team1126.framework.wrapper.PowerDistributionPanelWrapper;
import org.gosparx.team1126.interfaces.DigitalInputInterface;
import org.gosparx.team1126.interfaces.EncoderInterface;
import org.gosparx.team1126.interfaces.PowerDistributionPanelInterface;
import org.gosparx.team1126.interfaces.SolenoidInterface;
import org.gosparx.team1126.robot.IO;
import org.gosparx.team1126.robot.subsystem.BallAcqNew;
import org.gosparx.team1126.test.util.MockCANTalon;
import org.gosparx.team1126.test.util.MockEncoderData;
import org.gosparx.team1126.test.util.MockMagnetSensor;
import org.gosparx.team1126.test.util.TestBase;
import org.junit.Before;
import org.junit.Test;

public class BallAcqNewTest extends TestBase{

	private static BallAcqNew testSubject = BallAcqNew.getInstance();
	private MockCANTalon armMotorRight;
    private MockCANTalon armMotorLeft;
    private MockCANTalon rollerMotorRight;
    private MockCANTalon rollerMotorLeft;
    private SolenoidInterface flipper;
    private EncoderInterface armEncoderRight;
    private EncoderInterface armEncoderLeft;
    private MockEncoderData armEncoderDataR;
    private MockEncoderData armEncoderDataL;
    private MockMagnetSensor armHomeSwitchL;
    private MockMagnetSensor armHomeSwitchR;
    private MockMagnetSensor armStopSwitchL;
    private MockMagnetSensor armStopSwitchR;
    private DigitalInputInterface ballEntered;
    private DigitalInputInterface ballFullyIn;
    private PowerDistributionPanelInterface pdp;

	// called only once
    public void setupInternal() {
		System.out.println("Intializing");
    	boolean success = invokePrivateMethod(testSubject, "init", null);
		assertEquals(success, false);
		armMotorRight = (MockCANTalon) CANTalonFactory.getInstance().get(IO.CAN_ACQ_SHOULDER_R);
		armMotorLeft = (MockCANTalon) CANTalonFactory.getInstance().get(IO.CAN_ACQ_SHOULDER_L);
		rollerMotorRight = (MockCANTalon) CANTalonFactory.getInstance().get(IO.CAN_ACQ_ROLLERS_R);
		rollerMotorLeft = (MockCANTalon) CANTalonFactory.getInstance().get(IO.CAN_ACQ_ROLLERS_L);
		flipper = SolenoidFactory.getInstance().get(IO.PNU_FLIPPER_RELEASE);
		armEncoderRight = EncoderFactory.getInstance().get(IO.DIO_SHOULDER_ENC_RIGHT_A, IO.DIO_SHOULDER_ENC_RIGHT_B);
		armEncoderLeft = EncoderFactory.getInstance().get(IO.DIO_SHOULDER_ENC_LEFT_A, IO.DIO_SHOULDER_ENC_LEFT_B);
		armEncoderDataR = (MockEncoderData) EncoderDataFactory.getInstance().get(armEncoderRight, 0);
		armEncoderDataL = (MockEncoderData) EncoderDataFactory.getInstance().get(armEncoderLeft, 0);
		armHomeSwitchL = (MockMagnetSensor) MagnetSensorFactory.getInstance().get(IO.DIO_MAG_ACQ_SHOULDER_HOME_L, true);
		armHomeSwitchR = (MockMagnetSensor) MagnetSensorFactory.getInstance().get(IO.DIO_MAG_ACQ_SHOULDER_HOME_R, true);
		armStopSwitchL = (MockMagnetSensor) MagnetSensorFactory.getInstance().get(IO.DIO_MAG_ACQ_SHOULDER_STOP_L, true);
		armStopSwitchR = (MockMagnetSensor) MagnetSensorFactory.getInstance().get(IO.DIO_MAG_ACQ_SHOULDER_STOP_R, true);
		ballEntered = DigitalInputFactory.getInstance().get(IO.DIO_PHOTO_BALL_ACQ);
		ballFullyIn = DigitalInputFactory.getInstance().get(IO.DIO_PHOTO_BALL_IN);
		pdp = PowerDistributionPanelWrapper.getInstance();
	}

	@Test
	public void testPowerUp_NotHome()  {
		System.out.println("testPowerUp_NotHome");
		boolean success = invokePrivateMethod(testSubject, "execute", null);
		assertEquals(success, false);
	}
}
