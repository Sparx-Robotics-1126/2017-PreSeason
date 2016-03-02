package org.gosparx.team1126.framework.factories;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.framework.wrapper.DigitalInputWrapper;
import org.gosparx.team1126.interfaces.UnitTestInterface;

public final class DigitalInputFactory implements UnitTestInterface {
	private boolean inUnitTest;
	private static DigitalInputFactory instance = new DigitalInputFactory();
	private Map<Integer, DigitalInputWrapper> digitalInputs;

	public static DigitalInputFactory getInstance() {
	    return DigitalInputFactory.instance;
	}

    private DigitalInputFactory() {
		inUnitTest = false;
		digitalInputs = new HashMap<Integer, DigitalInputWrapper>();
    }

    public DigitalInputWrapper get(int _deviceNumber) {
    	if (!digitalInputs.containsKey(_deviceNumber)) {
    		DigitalInputWrapper solW = new DigitalInputWrapper(_deviceNumber);
    		if(inUnitTest) {
    			((UnitTestInterface)solW).SetToUnitTest();
    		}
    		digitalInputs.put(_deviceNumber, solW);
    	}
		return digitalInputs.get(_deviceNumber);
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