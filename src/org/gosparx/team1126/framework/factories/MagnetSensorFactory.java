package org.gosparx.team1126.framework.factories;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.interfaces.MagnetSensorInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.robot.sensors.MagnetSensor;
import org.gosparx.team1126.test.util.MockMagnetSensor;

public final class MagnetSensorFactory implements UnitTestInterface {
	private boolean inUnitTest;
	private static MagnetSensorFactory instance = new MagnetSensorFactory();
	private Map<Integer, MagnetSensorInterface> magnetSensors;

	public static MagnetSensorFactory getInstance() {
	    return MagnetSensorFactory.instance;
	}

    private MagnetSensorFactory() {
		inUnitTest = false;
		magnetSensors = new HashMap<Integer, MagnetSensorInterface>();
    }

    public MagnetSensorInterface get(int _port, boolean _inverse) {
    	if (!magnetSensors.containsKey(_port)) {
    	  if(inUnitTest) {
    		  magnetSensors.put(_port, new MockMagnetSensor(_port, _inverse));
    	  } else {
    		  magnetSensors.put(_port, new MagnetSensor(_port, _inverse));
    	  }
    	}
		return magnetSensors.get(_port);
    }

	public void SetToUnitTest() {
		inUnitTest = true;		
	}

	public boolean inUnitTest() {
		return inUnitTest;
	}
}
