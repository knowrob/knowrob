package org.knowrob.interfaces.mongo.types;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.vecmath.Vector3d;


import com.mongodb.BasicDBList;
import com.mongodb.BasicDBObject;

public class Designator {

	protected Map<String, Object> values;
	protected String detectionType = "";
	
	public Designator() {
		values = new LinkedHashMap<String, Object>();
	}

	public boolean containsKey(Object arg0) {
		return values.containsKey(arg0);
	}

	public Object get(Object arg0) {
		return values.get(arg0);
	}

	public boolean isEmpty() {
		return values.isEmpty();
	}

	public Object put(String arg0, Object arg1) {
		return values.put(arg0, arg1);
	}

	public int size() {
		return values.size();
	}
	public void clear() {
		values.clear();
	}

	public boolean containsValue(Object arg0) {
		return values.containsValue(arg0);
	}

	public Set<Entry<String, Object>> entrySet() {
		return values.entrySet();
	}

	public boolean equals(Object arg0) {
		return values.equals(arg0);
	}

	public Set<String> keySet() {
		return values.keySet();
	}

	public void putAll(Map<? extends String, ? extends Object> arg0) {
		values.putAll(arg0);
	}

	public Object remove(Object arg0) {
		return values.remove(arg0);
	}

	public Collection<Object> values() {
		return values.values();
	}
	
	public String getType() {
		return values.get("_designator_type").toString();
	}

	public void setType(String type) {
		this.values.put("_designator_type", type);
	}
	
	public String getDetectionType() {
		return detectionType;
	}

	public void setDetectionType(String type) {
		this.detectionType = type;
	}

	
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of Designator with values from row
	 */
	public Designator readFromDBObject(BasicDBObject row) {

		for(String key : row.keySet()) {
			Object val = null;
			
			// Pose properties
			if(key.equalsIgnoreCase("pose") || key.equalsIgnoreCase("goal-pose")) {
				val = PoseStampedFactory.readFromDBObject((BasicDBObject) row.get(key));
				if(val == null)
					val = PoseFactory.readFromDBObject((BasicDBObject) row.get(key));

			// Dimension properties
			} else if(key.equalsIgnoreCase("dimensions")) {
				val = new Vector3d();
				
				if(row.get(key) instanceof BasicDBList) {
					BasicDBList dim = ((BasicDBList) row.get(key));
					if(dim!=null) {
						((Vector3d) val).x = Double.valueOf(dim.get(0).toString());
						((Vector3d) val).y = Double.valueOf(dim.get(1).toString());
						((Vector3d) val).z = Double.valueOf(dim.get(2).toString());
					}
				}
				
			// Designator properties
			} else if(key.equalsIgnoreCase("at") || 
					  key.equalsIgnoreCase("handle") || 
					  key.equalsIgnoreCase("obj") || 
					  key.equalsIgnoreCase("phases")) {
				val = new Designator().readFromDBObject((BasicDBObject) row.get(key));
				
				// Numerical properties
			} else if(key.equalsIgnoreCase("z-offset")) {
				val = row.getDouble(key);

			// String properties
			} else {
				val = row.getString(key);
			}

			if(val!=null)
				values.put(key, val);
		}
		return this;
	}
}
