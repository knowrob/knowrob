/* 
 * Copyright (c) 2010, Lorenz Moesenlechner
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

import java.util.Vector;


public class PrologValue {

	public enum value_type { DOUBLE, INT, STRING, LIST, TERM, EMPTY } ;
	
	private Object value_;
	private value_type type_;

		
	public PrologValue() {
		type_=value_type.EMPTY;
	}
	
	    
	public PrologValue(double value) {
		value_ = value;
		type_  = value_type.DOUBLE;
	}
	
	public PrologValue(int value) {
			value_ = value;
			type_  = value_type.INT;
	  }

	public PrologValue(long value) {
			value_ = value;
			type_  = value_type.INT;
	  }
	  
	public PrologValue(String value) {
			value_ = value;
			type_  = value_type.STRING;		  
	  }

	public PrologValue(PrologTerm value) {
			value_ = value;
			type_  = value_type.TERM;
	  }

	public PrologValue(Vector<PrologValue> value) {
		  value_ = value;
		  type_ = value_type.LIST;
	  }
	  
	public value_type type() { return type_; }
	  public  boolean isDouble() { return type_ == value_type.DOUBLE; }
	  public  boolean isInt()    { return type_ == value_type.INT; }
	  public  boolean isString() { return type_ == value_type.STRING; }
	  public  boolean isTerm()   { return type_ == value_type.TERM; }
	  public  boolean islist()   { return type_ == value_type.LIST; }
	  public  boolean isValid()  { return type_ != value_type.EMPTY; }

	
	@Override
	@SuppressWarnings("unchecked")
	public String toString() {
		  
		  switch(type_) {
		    case DOUBLE:
		    case INT:
		    case STRING:
		    case TERM:
		      return value_.toString();
		      

		    case LIST: {
		    	
		    	String res = "[";
		    	Vector<PrologValue> vals =  (Vector<PrologValue>) value_;
		    	
		    	for(int i=0;i<vals.size();i++) {
		    		
		    		res+=vals.get(i).toString();
		    		
		    		if(i<vals.size()-1)
		    			res+=", ";
		    	}
		    	return res + "]";
		    }
		    default:
		      return "";
		  }
		  
	  }
	  
	public Object getValue() {
		return value_;
	}
}


