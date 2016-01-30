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

	public static final int ANALOG_IN_ABSOLUTE_ENC_SHOULDER                 = 2;
	
	public static final int ANALOG_ULTRASONIC_FRONT_LEFT                    = 10;
	
	public static final int ANALOG_ULTRASONIC_FRONT_RIGHT                   = 11;
	
	public static final int ANALOG_ULTRASONIC_BACK_LEFT                     = 12;
	
	public static final int ANALOG_ULTRASONIC_BACK_RIGHT                    = 13;
	
	/************************************DIO*******************************************/

	public static final int DIO_LEFT_DRIVES_ENC_A                           = 0;

	public static final int DIO_LEFT_DRIVES_ENC_B                           = 1;

	public static final int DIO_RIGHT_DRIVES_ENC_A                          = 2;

	public static final int DIO_RIGHT_DRIVES_ENC_B                          = 3;

	public static final int DIO_ROLLER_ENC_A                                = 4;

	public static final int DIO_ROLLER_ENC_B                                = 5;

	public static final int DIO_MAG_ACQ_SHOULDER_HOME                       = 6;

	public static final int DIO_PHOTO_BALL_ENTER                            = 7;

	public static final int DIO_PHOTO_BALL_IN                               = 8;

	public static final int DIO_MAG_LEFT_HOOK                               = 9;

	public static final int DIO_MAG_RIGHT_HOOK                              = 10;
	

	

	/**********************************PNU********************************************/

	public static final int PNU_SHIFTER                                     = 0;

	public static final int PNU_FLIPPER_RELEASE                             = 1;

	public static final int PNU_WINCH_RATCHET                               = 2;

	public static final int PNU_PTO                                         = 4;

	public static final int PNU_CIRCLE_POSITION                             = 5;

	public static final int PNU_CLIMBER_SCAL                                = 6;

	/**********************************CAN********************************************/

	public static final int CAN_DRIVES_LEFT_FRONT                           = 0;

	public static final int CAN_DRIVES_LEFT_BACK                            = 1;

	public static final int CAN_DRIVES_RIGHT_FRONT                          = 2;

	public static final int CAN_DRIVES_RIGHT_BACK                           = 3;
	
	public static final int CAN_ACQ_SHOULDER                                = 4;
	
	public static final int CAN_ACQ_ROLLERS_L                               = 5;
	
	public static final int CAN_ACQ_ROLLERS_R                               = 6;
	
}
