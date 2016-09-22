/*
 * Copyright (c) 2013 Moritz Tenorth, 2015 Daniel Beßler
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

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
        protected final java.util.Date instant;
	
	public Designator(java.util.Date instant) {
		values = new LinkedHashMap<String, Object>();
		this.instant = instant;
	}
        
        public Designator() {
                values = new LinkedHashMap<String, Object>();
                this.instant = new java.util.Date();
        }

	public boolean containsKey(Object arg0) {
		return values.containsKey(arg0);
	}

	public Object get(Object arg0) {
		Object out = values.get(arg0.toString().toLowerCase());
		if(out==null)
			out = values.get(arg0.toString().toUpperCase());
		return out;
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

	public java.util.Date getInstant() {
		return instant;
	}

	public String getDetectionType() {
		return detectionType;
	}

	public void setDetectionType(String type) {
		this.detectionType = type;
	}

	public Designator readFromDBList(BasicDBList l) {
		Integer count = 0;
		for(Object val : l) {
			Object valAccepted = null;
			if(val instanceof BasicDBObject) {
				valAccepted = new Designator(instant).readFromDBObject((BasicDBObject)val);
			}
			else {
				valAccepted = val.toString();
			}
			if(valAccepted!=null)
				values.put(count.toString(), valAccepted);
			count += 1;
		}
		return this;
	}
	
	
	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of Designator with values from row
	 */
	public Designator readFromDBObject(BasicDBObject row) {

		for(String key : row.keySet()) {
			if(key.isEmpty()) continue;
			
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
					
					if(dim.size()!=3) {
						System.err.println("Invalid designator: Expecting 3 values in 'dimensions' array!");
					}
					else {
						valAccepted = new Vector3d();
						((Vector3d) valAccepted).x = Double.valueOf(dim.get(0).toString());
						((Vector3d) valAccepted).y = Double.valueOf(dim.get(1).toString());
						((Vector3d) valAccepted).z = Double.valueOf(dim.get(2).toString());
					}
				}
			
			} else if(key.equalsIgnoreCase("dimensions-3d")) {
				
				if(val instanceof BasicDBObject) {
					BasicDBObject dimRow = (BasicDBObject)val;

					if(!dimRow.containsField("DEPTH") || !dimRow.containsField("WIDTH") || !dimRow.containsField("HEIGHT")) {
						System.err.println("Invalid designator: Expecting WIDTH,HEIGHT and DEPTH keys in 'dimensions-3d'!");
					}
					else {
						valAccepted = new Vector3d();
						((Vector3d) valAccepted).x = Double.valueOf(dimRow.get("DEPTH").toString());
						((Vector3d) valAccepted).y = Double.valueOf(dimRow.get("WIDTH").toString());
						((Vector3d) valAccepted).z = Double.valueOf(dimRow.get("HEIGHT").toString());
					}
				}

			// Color properties
			} else if(key.equalsIgnoreCase("color")) {
				if(val instanceof BasicDBObject) {
					BasicDBObject colRow = (BasicDBObject)val;
					if(!colRow.containsField("RED") || !colRow.containsField("GREEN") || !colRow.containsField("BLUE")) {
						System.err.println("Invalid designator: Expecting RED,GREEN and BLUE keys in 'dimensions-3d'!");
					}
					else {
						valAccepted = new Vector3d();
						((Vector3d) valAccepted).x = Double.valueOf(colRow.get("RED").toString());
						((Vector3d) valAccepted).y = Double.valueOf(colRow.get("GREEN").toString());
						((Vector3d) valAccepted).z = Double.valueOf(colRow.get("BLUE").toString());
					}
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
				try {
					valAccepted = row.getDouble(key);
				}
				catch(Exception exc) {
					System.err.println("Invalid designator: Number as value of z-offset!");
				}

			// String properties
			} else if(key.equalsIgnoreCase("response")) {
				// Convert perception response to knowrob class name
				valAccepted = toCamelCase(row.getString(key));
			}
			
			if(valAccepted == null) {
				// Designator properties
				if(val instanceof BasicDBObject) {
					valAccepted = new Designator(instant).readFromDBObject((BasicDBObject) val);
				
				} else if(val instanceof BasicDBList) {
					valAccepted = new Designator(instant).readFromDBList((BasicDBList) val);
				
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
			camelCaseString += part.substring(0, 1).toUpperCase() + part.substring(1);
		}
		return camelCaseString;
	}
}
