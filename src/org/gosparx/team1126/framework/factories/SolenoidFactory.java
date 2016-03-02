package org.gosparx.team1126.framework.factories;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.framework.wrapper.SolenoidWrapper;
import org.gosparx.team1126.interfaces.UnitTestInterface;

public final class SolenoidFactory implements UnitTestInterface {
	private boolean inUnitTest;
	private static SolenoidFactory instance = new SolenoidFactory();
	private Map<Integer, SolenoidWrapper> solenoids;

	public static SolenoidFactory getInstance() {
	    return SolenoidFactory.instance;
	}

    private SolenoidFactory() {
		inUnitTest = false;
		solenoids = new HashMap<Integer, SolenoidWrapper>();
    }

    public SolenoidWrapper get(int _deviceNumber) {
    	if (!solenoids.containsKey(_deviceNumber)) {
    		SolenoidWrapper solW = new SolenoidWrapper(_deviceNumber);
    		if(inUnitTest) {
    			((UnitTestInterface)solW).SetToUnitTest();
    		}
    		solenoids.put(_deviceNumber, solW);
    	}
		return solenoids.get(_deviceNumber);
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