/*
 * Created on Jun 7, 2010
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.util.datastruct;

import java.util.HashMap;

public class ParameterMap {
	protected HashMap<String,Object> map = new HashMap<String,Object>();
	
	public void putBoolean(String key, boolean value) {
		map.put(key, value);
	}
	
	public boolean getBoolean(String key) {
		return (Boolean)map.get(key);
	}
}
