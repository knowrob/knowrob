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
import java.util.HashMap;

import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class GazeboInformationReader {

	private HashMap<String, ArrayList<Element>> gazeboMap = new HashMap<String, ArrayList<Element>>();
	private NameToUnit n2u;
	
	
	/**
	 * Constructor: Reads all gazebo-information out of docElement into a
	 * private HashMap for fast further use.
	 * 
	 * @param docElement URDF XML element to be read
	 * 
	 */
	public GazeboInformationReader(Element docElement) {
		
		NodeList gazebos = docElement.getElementsByTagName("gazebo");
		for (int n = 0; n < gazebos.getLength(); n++) {

			Element gazebo = (Element) gazebos.item(n);
			String gazeboName = gazebo.getAttribute("reference");
			
			if (!gazeboName.isEmpty()) {
				
				// only add to list if associated with a link
				if (gazeboMap.containsKey(gazeboName)) {
					gazeboMap.get(gazeboName).add(gazebo);
					
				} else {
					ArrayList<Element> gazeboList = new ArrayList<Element>();
					gazeboList.add(gazebo);
					gazeboMap.put(gazeboName, gazeboList);
				}
			}
		}
		n2u = NameToUnit.getObjectReference();
	}
	

	/**
	 * Returns the properties to be asserted for a component.
	 * 
	 * @param indent
	 * @param prefix
	 * @param name
	 * @return
	 */
	public String getAttributeIncludesFor(String indent, String prefix, String name) {
		
		if (gazeboMap.containsKey(name)) {
			
			ArrayList<Element> gazeboList = gazeboMap.get(name);
			StringBuffer s = new StringBuffer();
			
			for (Element gazebo : gazeboList) {
				s.append(getAttributeIncludesFor(indent, prefix, name, gazebo));
			}
			return s.toString();
		}
		return "";
	}

	
	/**
	 * Returns the properties to be asserted for a component with a given XML node
	 * 
	 * @param indent Indentation string
	 * @param prefix
	 * @param name
	 * @param gazebo
	 * @return
	 */
	public String getAttributeIncludesFor(String indent, String prefix, String name, Node gazebo) {
		
		if (GeneralInformationReader.blacklist.contains(name)) {
			return "";
		}
		
		StringBuffer s = new StringBuffer();
		if (!gazebo.hasChildNodes()) {
			if (gazebo.getNodeType() != Node.TEXT_NODE) {
				if (gazebo.hasAttributes()) {
					s.append(createAttributeString(indent, gazebo.getNodeName(), gazebo.getAttributes().item(0).getNodeValue()));
				}
			}
		} else {
			
			// The node includes other child nodes:
			NodeList children = gazebo.getChildNodes();
			if (children.getLength() == 1) {
				Node child = children.item(0);
				if (child.getNodeType() == Node.TEXT_NODE) {
					s.append(createAttributeString(indent, gazebo.getNodeName(), child.getNodeValue()));
				}
				
			} else {
				s.append(LinkTypeReader.getLinkTypeTag(indent, gazebo));
				for (int n = 0; n < children.getLength(); n++) {
					Node child = children.item(n);
					String childName = child.getNodeName();
					if (child.hasAttributes()) {
						Element c = (Element) child;
						if (c.hasAttribute("reference")) {
							childName = c.getAttribute("reference");
						}
						if (c.hasAttribute("name")) {
							childName = c.getAttribute("name");
						}
					}
					s.append(getAttributeIncludesFor(indent, prefix, name + "_"	+ childName, child));
				}
			}
		}
		return s.toString();
	}

	
	/**
	 * Add property assertion based on inferred datatype
	 * 
	 * @param indent Indentation string
	 * @param prop Property name
	 * @param value Assertion value
	 * @return OWL string describing the assertion
	 */
	private String createAttributeString(String indent, String prop, String value) {
		return createAttributeString(indent, prop, n2u.ntu(prop, value), value);
	}

	/**
	 * Add property assertion using given datatype
	 * 
	 * @param indent Indentation string
	 * @param prop Property name
	 * @param datatype Datatype identifier
	 * @param value Assertion value
	 * @return OWL string describing the assertion
	 */
	private String createAttributeString(String indent, String prop, String datatype, String value) {
		
		if(prop.equals("imageSize")) {
			return  indent + "<srdl2-comp:" + NormalizeName.normalize(prop)
					+ "X rdf:datatype=\"" + datatype + "\">" + value.split(" ")[0] + "</srdl2-comp:"
					+ NormalizeName.normalize(prop) + "X>\n" + 
					indent + "<srdl2-comp:" + NormalizeName.normalize(prop)
					+ "Y rdf:datatype=\"" + datatype + "\">" + value.split(" ")[1] + "</srdl2-comp:"
					+ NormalizeName.normalize(prop) + "Y>\n";
			
		} else {

			return indent + "<srdl2-comp:" + NormalizeName.normalize(prop)
					+ " rdf:datatype=\"" + datatype + "\">" + value + "</srdl2-comp:"
					+ NormalizeName.normalize(prop) + ">\n";	
		}
	}
}
