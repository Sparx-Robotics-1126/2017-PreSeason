package org.gosparx.team1126.framework.factories;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.interfaces.EncoderDataInterface;
import org.gosparx.team1126.interfaces.EncoderInterface;
import org.gosparx.team1126.interfaces.UnitTestInterface;
import org.gosparx.team1126.robot.sensors.EncoderData;
import org.gosparx.team1126.test.util.MockEncoderData;

public final class EncoderDataFactory implements UnitTestInterface {
	private boolean inUnitTest;
	private static EncoderDataFactory instance = new EncoderDataFactory();
	private Map<Integer, EncoderDataInterface> encoderDatas;

	public static EncoderDataFactory getInstance() {
	    return EncoderDataFactory.instance;
	}

    private EncoderDataFactory() {
		inUnitTest = false;
		encoderDatas = new HashMap<Integer, EncoderDataInterface>();
    }

    public EncoderDataInterface get(EncoderInterface armEncoderRight, double _distPerTick) {
    	if (!encoderDatas.containsKey(java.lang.System.identityHashCode(armEncoderRight))) {
    	  if(inUnitTest) {
    		  encoderDatas.put(java.lang.System.identityHashCode(armEncoderRight),
    				  new MockEncoderData(armEncoderRight, _distPerTick));
    	  } else {
    		  encoderDatas.put(java.lang.System.identityHashCode(armEncoderRight),
    				  new EncoderData(armEncoderRight, _distPerTick));
    	  }
    	}
		return encoderDatas.get(java.lang.System.identityHashCode(armEncoderRight));
    }

	@Override
	public void SetToUnitTest() {
		inUnitTest = true;		
	}

	@Override
	public boolean inUnitTest() {
		return inUnitTest;
	}
}
