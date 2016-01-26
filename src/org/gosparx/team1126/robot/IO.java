package org.gosparx.team1126.robot;

/**
 * Stores all of the IO information
 * @author Alex Mechler
 */
public class IO {

	/**************************************PWM*****************************************/

	public static final int PWM_GYRO_POWER                                  = 9;

	/***********************************ANALONG_IN*************************************/

	public static final int ANALOG_IN_GYRO_HIGH                             = 0;

	public static final int ANALOG_IN_GYRO_LOW                              = 1;

	public static final int ANALOG_IN_ULTRASONIC_FL                         = 2;

	public static final int ANALOG_IN_ULTRASONIC_FR                         = 3;

	public static final int ANALOG_IN_ULTRASONIC_BL                         = 10;

	public static final int ANALOG_IN_ULTRASONIC_BR                         = 11;

	/************************************DIO*******************************************/

	public static final int DIO_LEFT_DRIVES_ENC_A                           = 0;

	public static final int DIO_LEFT_DRIVES_ENC_B                           = 1;

	public static final int DIO_RIGHT_DRIVES_ENC_A                          = 2;

	public static final int DIO_RIGHT_DRIVES_ENC_B                          = 3;

	public static final int DIO_ROLLER_ENC_A                                = 4;

	public static final int DIO_ROLLER_ENC_B                                = 5;

	public static final int DIO_SHOULDER_ENC_A                              = 6;

	public static final int DIO_SHOULDER_ENC_B                              = 7;

	public static final int DIO_CIRCLE_ENC_A                                = 8;

	public static final int DIO_CIRCLE_ENC_B                                = 9;

	public static final int DIO_MAG_SHOULDER_HOME                           = 10;

	public static final int DIO_MAG_CIRCLE_HOME                             = 11;

	public static final int DIO_BALL_ENTERED                                = 12;

	public static final int DIO_BALL_COMPLETELY_IN                          = 13;

	/**********************************PNU********************************************/

	public static final int PNU_PTO                                         = 0;

	public static final int PNU_WINCH_RATCHET                               = 1;

	public static final int PNU_CLIMBER_A                                   = 2;

	public static final int PNU_CLIMBER_B                                   = 3;

	public static final int PNU_FLIPPER_RELEASE                             = 4;

	public static final int PNU_SHIFTER                                     = 5;

	/**********************************CAN********************************************/

	public static final int CAN_DRIVES_LEFT_FRONT                           = 0;

	public static final int CAN_DRIVES_LEFT_BACK                            = 1;

	public static final int CAN_DRIVES_LEFT_BOTTOM                          = 2;

	public static final int CAN_DRIVES_RIGHT_FRONT                          = 3;

	public static final int CAN_DRIVES_RIGHT_BACK                           = 4;

	public static final int CAN_DRIVES_RIGHT_BOTTOM                         = 5;
	
	public static final int CAN_ACQ_SHOULDER                                = 6;
	
	public static final int CAN_ACQ_CIRCLE                                  = 7;
	
	public static final int CAN_ACQ_ROLLERS                                 = 8;
	
}
