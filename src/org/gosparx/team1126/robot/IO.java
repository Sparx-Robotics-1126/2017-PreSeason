package org.gosparx.team1126.robot;

/**
 * Stores all of the IO information
 * @author Alex Mechler
 */
public class IO {

	/**************************************PWM*****************************************/

	public static final int PWM_GYRO_POWER                                  = 7;

	/***********************************ANALONG_IN*************************************/

	public static final int ANALOG_IN_ANGLE_GYRO                            = 0;

	public static final int ANALOG_IN_TILT_GYRO                             = 1;

	public static final int ANALOG_IN_ABS_ENC_SHOULDER                      = 2;
	
	public static final int ANALOG_IN_PNU_PRESSURE_SENSOR                   = 3;
	
	/************************************DIO*******************************************/

	public static final int DIO_LEFT_DRIVES_ENC_A                           = 20;

	public static final int DIO_LEFT_DRIVES_ENC_B                           = 17;

	public static final int DIO_RIGHT_DRIVES_ENC_A                          = 21;

	public static final int DIO_RIGHT_DRIVES_ENC_B                          = 18;

	public static final int DIO_ROLLER_ENC_A                                = 19;

	public static final int DIO_ROLLER_ENC_B                                = 16;

	public static final int DIO_MAG_ACQ_SHOULDER_HOME                       = 10;
	
	public static final int DIO_PHOTO_BALL_ENTER                            = 11;
	
	public static final int DIO_PHOTO_BALL_IN                               = 12;

	public static final int DIO_PHOTO_LEFT_HOOK                               = 13;
	
	public static final int DIO_PHOTO_RIGHT_HOOK                              = 14;
	
	/**********************************PNU********************************************/

	public static final int PNU_SHIFTER                                     = 0;

	public static final int PNU_FLIPPER_RELEASE                             = 6;

	public static final int PNU_WINCH_RATCHET                               = 2;

	public static final int PNU_PTO                                         = 4;
	
	public static final int PNU_CIRCLE_POSITION_A                           = 2;

	public static final int PNU_CIRCLE_POSITION_B                           = 3;

	public static final int PNU_CLIMBER_SCALE                               = 5;

	/**********************************CAN********************************************/

	public static final int CAN_DRIVES_LEFT_FRONT                           = 1;

	public static final int CAN_DRIVES_LEFT_BACK                            = 2;

	public static final int CAN_DRIVES_RIGHT_FRONT                          = 9;
	
	public static final int CAN_DRIVES_RIGHT_BACK                           = 8;

	public static final int CAN_ACQ_SHOULDER                                = 7;
	
	public static final int CAN_ACQ_ROLLERS_L                               = 3;
	
	public static final int CAN_ACQ_ROLLERS_R                               = 4;
	
	/**********************************USB********************************************/
	
	public static final int USB_DRIVER_LEFT                                 = 0;
	
	public static final int USB_DRIVER_RIGHT                                = 1;
	
	public static final int USB_OPERATOR                                    = 2;
}
