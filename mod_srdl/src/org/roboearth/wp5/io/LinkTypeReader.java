/* \file LinkTypeReader.java
 * 
 * \brief URDF2SRDL is a tool for automatically creating a SRDL description 
 * from an URDF document. 
 *
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2012 by Philipp Freyer 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *    <UL>
 *     <LI> Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     <LI> Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     <LI> Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *    </UL>
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
 * \author Philipp Freyer 
 * \version 1.0
 * \date 2012
 */

package org.roboearth.wp5.io;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import org.w3c.dom.Node;

public class LinkTypeReader {

	private static List<String> tagsForTypeInclusion = Arrays.asList(new String[]{"sensor"});

	private static HashMap<String, HashMap<String, String>> linkTags = new HashMap<String, HashMap<String, String>>();

	
	
	private static void initLinkTypes() {	
		HashMap<String, String> sensorTypes = new HashMap<String, String>();
		sensorTypes.put("camera", "&srdl2-comp;Camera");
		sensorTypes.put("contact", "&srdl2-comp;PressureSensor");
		sensorTypes.put("ray", "&srdl2-comp;LaserScanner");
		sensorTypes.put("default", "&srdl2-comp;Sensor");
		linkTags.put("sensor", sensorTypes);
	}
	
	
	public static String getLinkTypeTag(String indent, Node gazebo) {

		String tagname = gazebo.getNodeName().toLowerCase();

		boolean include = false;

		// check if tag has form 'sensor:type' or just 'type' with type being in linkTags
		String prefix = "";
		String tag = "";
		
		if(tagname.contains(":")){
			prefix = tagname.split(":")[0];	
			tag    = tagname.split(":")[1];
		}

		if(tagsForTypeInclusion.contains(prefix)) {
			include = true;
		}
		
		for(HashMap<String, String> types : linkTags.values()) {
			if(types.keySet().contains(tag))
				include = true;
		}
		
		if(include)
			return indent + "<rdf:type rdf:resource=\"" + nameToLinkType(tagname.split(":")) + "\"/>\n";
		else
			return "";
	}

	
	private static String nameToLinkType(String[] info) {
		
		if(linkTags.isEmpty()) {
			initLinkTypes();
		}
		
		if(linkTags.containsKey(info[0])) {
			
			if(linkTags.get(info[0]).containsKey(info[1])) {
				return linkTags.get(info[0]).get(info[1]);
				
			} else {
				System.out.println("[PIF]\tUnknown Type - returning default: " + info[0] + ":" + info[1]);
				return linkTags.get(info[0]).get("default");
			}
		}
		System.out.println("[PIF]\tIgnored Type: " + info[0] + ":" + info[1]);

		return "&srdl2-comp;Component";
	}

}
