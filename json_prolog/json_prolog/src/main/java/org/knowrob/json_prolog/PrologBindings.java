/* 
 * Copyright (c) 2010, Moritz Tenorth
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

package org.knowrob.json_prolog;

import java.util.HashMap;
import java.util.Vector;

import org.jpl7.Term;

import org.knowrob.json_prolog.query.JSONQuery;
import org.knowrob.json_prolog.query.JSONQuery.InvalidJSONQuery;

import net.sf.json.JSONArray;
import net.sf.json.JSONObject;
import net.sf.json.util.JSONUtils;


/**
 * Representation of a set of Prolog variable bindings 
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */
public class PrologBindings {

	
	private HashMap<String, PrologValue> bdgs_;
	

	public PrologBindings() {
		bdgs_ = new HashMap<String, PrologValue>();
	}
	
	/**
	 * Convert a JSON-encoded string into a PrologBindings object
	 * 
	 * @param json_bdgs JSON encoded string
	 * @return The corresponding Prolog bindings
	 */
	static public PrologBindings parseJSONBindings(String json_bdgs) {
		
		PrologBindings res = new PrologBindings();		
		if(json_bdgs!=null) {

			JSONObject json_obj = JSONObject.fromObject(json_bdgs);
			for(Object k : json_obj.keySet()) {
				
				Object bdg = json_obj.get(k.toString());
				
				try {

					PrologValue r = new PrologValue();

					if(JSONUtils.isArray(bdg)) {
						
						Term[] objs = JSONQuery.decodeJSONArray(JSONArray.fromObject(bdg));
						
						Vector<PrologValue> values = new Vector<PrologValue>(); 
						for(Term o : objs) {
							values.add(new PrologValue(o.toString()));
						}
						r = new PrologValue(values);

					} else if(JSONUtils.isObject(bdg)) {
						Term obj = JSONQuery.decodeJSONValue(JSONObject.fromObject(bdg));
						r = new PrologValue(obj.toString());
						
					} else if(JSONUtils.isString(bdg)) {
						r = new PrologValue(bdg.toString());
						
					} else if(JSONUtils.isNumber(bdg)) {
						r = new PrologValue(Double.valueOf(bdg.toString()));
						
					}
					res.bdgs_.put(k.toString(), r);	
					
				} catch (InvalidJSONQuery e) {
					e.printStackTrace();
				}
					
			}
			
		}
		return res;
	}


	public HashMap<String, PrologValue> getBdgs_() {
		return bdgs_;
	}


	public void setBdgs_(HashMap<String, PrologValue> bdgs) {
		bdgs_ = bdgs;
	}


	class VariableUnbound extends Exception {

		private static final long serialVersionUID = -5329179598962011281L;

		public VariableUnbound(String var_name)	{
			super(var_name);
		}
	};


	class JSONParseError extends Exception {

		private static final long serialVersionUID = 6704618653787680364L;

		public JSONParseError(String msg) {
			super(msg);
		}
	};

	
}
