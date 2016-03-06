package org.gosparx.team1126.test.util;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.gosparx.team1126.robot.util.WPI_Factory;
import org.junit.Before;
import org.junit.BeforeClass;
import junit.framework.AssertionFailedError;

public abstract class TestBase {

    private static boolean intialized = false;

	// This happens once before anything else
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {
		WPI_Factory.getInstance().setFactory(new MockWPI_Factory());
	}

	@Before
	public void setup() throws Exception {
		if (!intialized)
		{
			intialized = true;
			setupInternal();
		}
	}

	protected abstract void setupInternal();

	protected <T> T invokePrivateMethod(
			Object _obj, String _method, Object[] _args) {
		T returnValue;
		try {
            Method method = _obj.getClass().getDeclaredMethod(_method);
            method.setAccessible(true);
            returnValue = (T) method.invoke(_obj, _args);
        }
        catch (NoSuchMethodException e) {
            // Should happen only rarely, because most times the
            // specified method should exist. If it does happen, just let
            // the test fail so the programmer can fix the problem.
            throw new AssertionFailedError();
        }
        catch (SecurityException e) {
            // Should happen only rarely, because the setAccessible(true)
            // should be allowed in when running unit tests. If it does
            // happen, just let the test fail so the programmer can fix
            // the problem.
            throw new AssertionFailedError();
        }
        catch (IllegalAccessException e) {
            // Should never happen, because setting accessible flag to
            // true. If setting accessible fails, should throw a security
            // exception at that point and never get to the invoke. But
            // just in case, wrap it in a TestFailedException and let a
            // human figure it out.
            throw new AssertionFailedError();
        }
        catch (IllegalArgumentException e) {
            // Should happen only rarely, because usually the right
            // number and types of arguments will be passed. If it does
            // happen, just let the test fail so the programmer can fix
            // the problem.
            throw new AssertionFailedError();
        }
		catch (InvocationTargetException e) {
            throw new AssertionFailedError();
		}
		return returnValue;
    }

	protected <T> T getPrivateVariable(Object _obj, String _field) {
		T returnValue;
		try {
            Field field = _obj.getClass().getDeclaredField(_field);
            field.setAccessible(true);
            returnValue = (T) field.get(_obj);
        }
        catch (NoSuchFieldException e) {
            // Should happen only rarely, because most times the
            // specified method should exist. If it does happen, just let
            // the test fail so the programmer can fix the problem.
            throw new AssertionFailedError();
        }
        catch (SecurityException e) {
            // Should happen only rarely, because the setAccessible(true)
            // should be allowed in when running unit tests. If it does
            // happen, just let the test fail so the programmer can fix
            // the problem.
            throw new AssertionFailedError();
        }
        catch (IllegalAccessException e) {
            // Should never happen, because setting accessible flag to
            // true. If setting accessible fails, should throw a security
            // exception at that point and never get to the invoke. But
            // just in case, wrap it in a TestFailedException and let a
            // human figure it out.
            throw new AssertionFailedError();
        }
        catch (IllegalArgumentException e) {
            // Should happen only rarely, because usually the right
            // number and types of arguments will be passed. If it does
            // happen, just let the test fail so the programmer can fix
            // the problem.
            throw new AssertionFailedError();
        }
		return returnValue;
    }
}
