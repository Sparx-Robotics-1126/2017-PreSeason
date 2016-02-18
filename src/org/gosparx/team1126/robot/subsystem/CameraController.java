package org.gosparx.team1126.robot.subsystem;

import java.util.ArrayList;

import org.gosparx.team1126.robot.IO;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.Image;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 * A class for controlling multiple USBCameras at once.
 * @author Alex Mechler {amechler1998@gmail.com}
 */
public class CameraController extends GenericSubsystem {

	/**
	 * Support for singleton
	 */
	private static CameraController camera;

	/**
	 * The image to push to the CameraServer
	 */
	private Image frame;

	/**
	 * The list of all attached cameras
	 */
	private ArrayList<USBCamera> cams;

	/**
	 * The index of the current camera we are looking at
	 */
	private int currCam;

	/**
	 * When we error, we kill our self to keep the robot running and avoid crashes
	 */
	private boolean isKilled;

	/**
	 * The current camera we are viewing.
	 */
	private USBCamera cam;

	/**
	 * The maximum fps for all of the cameras
	 */
	private final int MAX_FPS = 15;

	/**
	 * The quality of image to push back to the driver station. Lower numbers save more bandwidth (0-100)
	 */
	private final int QUALITY = 10;
	
	/**
	 * Time to sleep after changing camera views. This is to prevent errors as USBCamera.startCapture() returns before it is ready to be seen 
	 */
	private final long SLEEP_TIME = 100;

	/**
	 * Singleton
	 * @return The only CameraController that will be constructed
	 */
	public static synchronized CameraController getInstance(){
		if(camera == null){
			camera = new CameraController();
		}
		return camera;
	}

	/**
	 * Creates a new CameraController
	 */
	private CameraController() {
		super("CameraController", Thread.MIN_PRIORITY);
	}

	/**
	 * Runs once when .start() is called
	 */
	@Override
	protected boolean initi() {
		cams = new ArrayList<USBCamera>();
		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		currCam = 0;
		CameraServer.getInstance().setQuality(QUALITY);
		for(String s: IO.CAMS){
			addCamera(s);
		}
		cam = cams.get(currCam);
		cam.openCamera();
		cam.startCapture();
		return true;
	}

	/**
	 * runs every sleepTime() ms
	 */
	@Override
	protected boolean execute() {
		if(!isKilled){
			cams.get(currCam).getImage(frame);
			CameraServer.getInstance().setImage(frame);
		}
		return isKilled;
	}

	/**
	 * Adds the camera to our list to switch between and sets the FPS max
	 * @param camName The name of the camera
	 */
	private void addCamera(String camName){
		USBCamera temp = new USBCamera(camName);
		temp.setFPS(MAX_FPS);
		cams.add(temp);
		temp = null;
	}

	/**
	 * Switch to the next camera in our ArrayList
	 */
	public void switchCamera(){
		try{
			cam.stopCapture();
			cam.closeCamera();
			currCam++;
			currCam %= cams.size();
			cam = cams.get(currCam);
			cam.openCamera();
			cam.startCapture();
			Thread.sleep(SLEEP_TIME);
		}catch(Exception e){
			e.printStackTrace();
			isKilled = true;
		}
	}

	/**
	 * @return The time to sleep between thread loops. This thread should run at roughly 15Hz
	 */
	@Override
	protected long sleepTime() {
		return 60;
	}

	/**
	 * Does nothing in this case
	 */
	@Override
	protected void liveWindow() {

	}

	/**
	 * writes nothing in this case
	 */
	@Override
	protected void writeLog() {

	}
}
