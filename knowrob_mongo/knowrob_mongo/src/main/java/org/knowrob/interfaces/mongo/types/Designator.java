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
			Object val = row.get(key);
			Object valAccepted = null;
			
			// Pose properties
			if(key.equalsIgnoreCase("pose") || key.equalsIgnoreCase("goal-pose")) {
				valAccepted = PoseStampedFactory.readFromDBObject((BasicDBObject) val);
				if(valAccepted == null)
					valAccepted = PoseFactory.readFromDBObject((BasicDBObject) val);

			// Dimension properties
			} else if(key.equalsIgnoreCase("dimensions")) {
				
				if(val instanceof BasicDBList) {
					BasicDBList dim = ((BasicDBList) val);
					
					valAccepted = new Vector3d();
					((Vector3d) valAccepted).x = Double.valueOf(dim.get(0).toString());
					((Vector3d) valAccepted).y = Double.valueOf(dim.get(1).toString());
					((Vector3d) valAccepted).z = Double.valueOf(dim.get(2).toString());
				}
			
			} else if(key.equalsIgnoreCase("dimensions-3d")) {
				
				if(val instanceof BasicDBObject) {
					BasicDBObject dimRow = (BasicDBObject)val;
					valAccepted = new Vector3d();
					
					((Vector3d) valAccepted).x = Double.valueOf(dimRow.get("DEPTH").toString());
					((Vector3d) valAccepted).y = Double.valueOf(dimRow.get("WIDTH").toString());
					((Vector3d) valAccepted).z = Double.valueOf(dimRow.get("HEIGHT").toString());
				}

			// Color properties
			} else if(key.equalsIgnoreCase("color")) {
				if(val instanceof BasicDBObject) {
					BasicDBObject colRow = (BasicDBObject)val;
					valAccepted = new Vector3d();
					((Vector3d) valAccepted).x = Double.valueOf(colRow.get("RED").toString());
					((Vector3d) valAccepted).y = Double.valueOf(colRow.get("GREEN").toString());
					((Vector3d) valAccepted).z = Double.valueOf(colRow.get("BLUE").toString());
				}
				else {
					String colorString = row.getString(key);
					if("RED".equals(colorString)) {
						valAccepted = new Vector3d(0.75,0.0,0.0);
					}
					else if("GREEN".equals(colorString)) {
						valAccepted = new Vector3d(0.0,0.75,0.0);
					}
					else if("BLUE".equals(colorString)) {
						valAccepted = new Vector3d(0.0,0.0,0.75);
					}
					else if("YELLOW".equals(colorString)) {
						valAccepted = new Vector3d(0.75,0.75,0.0);
					}
				}
				
			// Numerical properties
			} else if(key.equalsIgnoreCase("z-offset")) {
				valAccepted = row.getDouble(key);

			// String properties
			} else if(key.equalsIgnoreCase("response")) {
				// Convert perception response to knowrob class name
				valAccepted = toCamelCase(row.getString(key));
			}
			
			if(valAccepted == null) {
				// Designator properties
				if(val instanceof BasicDBObject) {
					valAccepted = new Designator().readFromDBObject((BasicDBObject) val);
				
				} else {
					valAccepted = row.getString(key);
				}
			}
			
			if(valAccepted!=null)
				values.put(key.toUpperCase(), valAccepted);
		}
		return this;
	}
	
	static String toCamelCase(String s){
		String[] parts = s.split("_");
		String camelCaseString = "";
		for (String part : parts){
			camelCaseString += camelCaseString +
				part.substring(0, 1).toUpperCase() + part.substring(1);
		}
		return camelCaseString;
	}
}
