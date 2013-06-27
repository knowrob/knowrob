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
 * 
 */
package org.roboearth.wp5.io;

import java.util.Arrays;
import java.util.HashSet;

import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class GeneralInformationReader {

	private NameToUnit n2u;
	
	public static HashSet<String> blacklist  = new HashSet<String>(
			Arrays.asList(new String[]{
					"gazebo", 
					"parent", 
					"child", 
					"origin", 
					"axis", 
					"safety_controller", 
					"limit", 
					"turnGravityOff", 
					"calibration", 
					"dynamics"}));

	/**
	 * Constructor
	 */
	public GeneralInformationReader() {
		n2u = NameToUnit.getObjectReference();
	}

	
	/**
	 * Returns the properties to be asserted for a component
	 * 
	 * @param indent
	 * @param prefix
	 * @param component
	 * @return
	 */
	public String getAttributeIncludesFor(String indent, String prefix,	Node component) {
		return getAttributeIncludesFor(indent, prefix, "", component);
	}

	
	/**
	 * Returns the properties to be asserted for a component with a given XML node
	 * 
	 * @param indent
	 * @param prefix
	 * @param name
	 * @param component
	 * @return
	 */
	public String getAttributeIncludesFor(String indent, String prefix,	String name, Node component) {
		
		if (GeneralInformationReader.blacklist.contains(component.getNodeName())) {
			return "";
		}
		
		StringBuffer s = new StringBuffer();
		if (!component.hasChildNodes()) {
			if (component.getNodeType() != Node.TEXT_NODE) {
				if (component.hasAttributes()) {
					NamedNodeMap attrs = component.getAttributes();
					for (int a = 0; a < attrs.getLength(); a++) {
						s.append(createAttributeString(indent,
								component.getNodeName()
								+ "_"
								+ component.getAttributes().item(a)
								.getNodeName(), component
								.getAttributes().item(a).getNodeValue()));
					}
				}
			}
		} else {
			// The node includes other child nodes:
			NodeList children = component.getChildNodes();
			if (children.getLength() == 1) {
				Node child = children.item(0);
				if (child.getNodeType() == Node.TEXT_NODE) {
					s.append(createAttributeString(indent,
							component.getNodeName(), child.getNodeValue()));
				}
			} else {
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
					s.append(getAttributeIncludesFor(indent, prefix, name + "_"
							+ childName, child));
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
	private String createAttributeString(String indent, String name, String value) {
		return createAttributeString(indent, name, n2u.ntu(name, value), value);
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
	private String createAttributeString(String indent, String name, String type, String value) {
		
		return indent + "<srdl2-comp:" + NormalizeName.normalize(name)
				+ " rdf:datatype=\"" + type + "\">" + value + "</srdl2-comp:"
				+ NormalizeName.normalize(name) + ">\n";
	}
}
