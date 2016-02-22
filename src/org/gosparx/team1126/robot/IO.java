package org.gosparx.team1126.robot;

/**
 * Stores all of the IO information
 * @author Alex Mechler
 */
public class IO {

	/**************************************PWM*****************************************/

	public static final int PWM_GYRO_POWER                                  = 7;

	/***********************************ANALONG_IN*************************************/

	public static final int ANALOG_IN_ANGLE_GYRO                            = 1;

	public static final int ANALOG_IN_TILT_GYRO                             = 0;
	
	public static final int ANALOG_IN_PNU_PRESSURE_SENSOR                   = 3;
	
	/************************************DIO*******************************************/

	public static final int DIO_LEFT_DRIVES_ENC_A                           = 10;

	public static final int DIO_LEFT_DRIVES_ENC_B                           = 11;

	public static final int DIO_RIGHT_DRIVES_ENC_A                          = 22;

	public static final int DIO_RIGHT_DRIVES_ENC_B                          = 23;

	public static final int DIO_MAG_ACQ_SHOULDER_HOME                       = 14;
	
	public static final int DIO_PHOTO_BALL_ENTER                            = 16;

	public static final int DIO_PHOTO_LEFT_HOOK                             = 17;
	
	public static final int DIO_PHOTO_RIGHT_HOOK                            = 18;
	
	public static final int DIO_SHOULDER_ENC_LEFT_A                         = 12;
	
	public static final int DIO_SHOULDER_ENC_LEFT_B                         = 13;
	
	public static final int DIO_SHOULDER_ENC_RIGHT_A                        = 24;
	
	public static final int DIO_SHOULDER_ENC_RIGHT_B                        = 25;
	
	/**********************************PNU********************************************/

	public static final int PNU_SHIFTER                                     = 0;

	public static final int PNU_FLIPPER_RELEASE                             = 4;

	public static final int PNU_WINCH_RATCHET                               = 1;

	public static final int PNU_PTO                                         = 5;
	
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
	
	/*********************************CAMS********************************************/
	
	public static final String[] CAMS                                       = {
			                                                                "cam1",
			                                                                "cam2"
	                                                                        };
	
	/*********************************CONTROLS****************************************/
	
	public static final int DRIVER_JOY_LEFT                                 = 0;
	
	public static final int DRIVER_JOY_RIGHT                                = 1;
}
