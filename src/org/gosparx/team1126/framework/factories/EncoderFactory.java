package org.gosparx.team1126.framework.factories;

import java.util.HashMap;
import java.util.Map;

import org.gosparx.team1126.framework.wrapper.EncoderWrapper;
import org.gosparx.team1126.interfaces.UnitTestInterface;

public final class EncoderFactory implements UnitTestInterface {
	private boolean inUnitTest;
	private static EncoderFactory instance = new EncoderFactory();
	private Map<Integer, Map<Integer, EncoderWrapper> > encoders;

	public static EncoderFactory getInstance() {
	    return EncoderFactory.instance;
	}

    private EncoderFactory() {
		inUnitTest = false;
		encoders = new HashMap<Integer, Map<Integer, EncoderWrapper> >();
    }

    public EncoderWrapper get(int channellA, int channelB) {
    	if (!encoders.containsKey(channellA) ||
    		!encoders.get(channellA).containsKey(channelB)) {
    		EncoderWrapper solW = new EncoderWrapper(channellA, channelB);
    		if(inUnitTest) {
    			((UnitTestInterface)solW).SetToUnitTest();
    		}
    		Map<Integer, EncoderWrapper> innerM = new HashMap<Integer, EncoderWrapper>();
    		innerM.put(channelB, solW);
    		encoders.put(channellA, innerM);
    	}
    	return encoders.get(channellA).get(channelB);
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