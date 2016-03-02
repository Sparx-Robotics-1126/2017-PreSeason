package org.gosparx.team1126.framework.factories;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.test.util.MockCANTalon;
import edu.wpi.first.wpilibj.CANSpeedController;
import edu.wpi.first.wpilibj.CANTalon;

public final class CANTalonFactory implements UnitTestInterface {
	private boolean inUnitTest;
	private static CANTalonFactory instance = new CANTalonFactory();
	private Map<Integer, CANSpeedController> canTalons;

	public static CANTalonFactory getInstance() {
	    return CANTalonFactory.instance;
	}

    private CANTalonFactory() {
		inUnitTest = false;
		canTalons = new HashMap<Integer, CANSpeedController>();
    }

    public CANSpeedController get(int _deviceNumber) {
    	if (!canTalons.containsKey(_deviceNumber)) {
    	  if(inUnitTest) {
    		  canTalons.put(_deviceNumber, new MockCANTalon(_deviceNumber));
    	  } else {
    		  canTalons.put(_deviceNumber, new CANTalon(_deviceNumber));
    	  }
    	}
		return canTalons.get(_deviceNumber);
    }

	public void SetToUnitTest() {
		inUnitTest = true;		
	}

	public boolean inUnitTest() {
		return inUnitTest;
	}
}
