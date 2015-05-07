/*
 * Copyright (c) 2013 Moritz Tenorth
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

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.GregorianCalendar;

import com.mongodb.DBObject;

import org.ros.message.Time;

public class ISODate {

	protected Date date;

		
	/**
	 * Create from Java Date object
	 * 
	 * @param d
	 */
	public ISODate(Date d) {
		this.date = d;
	}

	/**
	 * Create from ROS Time object
	 * 
	 * @param t
	 */
	public ISODate(Time t) {
		date = new Date((long)t.secs * 1000 + t.nsecs/1000000);
	}

	/**
	 * Create from Java time stamp
	 * 
	 * @param t Time as milliseconds since 1.1.1970
	 */
	public ISODate(long t) {
		date = new Date(t);
	}

	
	
	@Override
	public String toString() {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		return sdf.format(this.date);
	}

	public Date getDate() {
		return date;
	}

	public long getNanoSeconds() {
		return date.getTime() * 1000000;
	}

	public long getMilliSeconds() {
		return date.getTime();
	}
	
	public Time toROSTime() {
		GregorianCalendar datum = new GregorianCalendar();
		datum.setTime(date);
		long ms = datum.getTimeInMillis();
		
		Time t  = new Time();
		t.secs  = (int) (ms / 1000);
		t.nsecs = (int) (ms % 1000) * 1000000;
		return t;	
	}
	

	/**
	 * Read from a MongoDB result object
	 * 
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of PoseStamped with values from row
	 */
	public ISODate readFromDBObject(DBObject row) {
		
		this.date = (Date) row;
//		SimpleDateFormat sdf;
//		
//		if(isodate.contains("."))
//			sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
//		else 
//			sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'");
//		
//	    GregorianCalendar datum = new GregorianCalendar();
//	    try {
//	    	datum.setTime(sdf.parse(isodate));
//	    	this.date = datum.getTime();
//	    	
//	    } catch (ParseException e) {
//		    e.printStackTrace();
//	    }
		return this;
	}
	
}
