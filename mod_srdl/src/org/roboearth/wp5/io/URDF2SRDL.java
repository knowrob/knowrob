/* \file URDF2SRDL.java
 * \brief URDF2SRDL is a tool for automatically creating a SRDL description 
 * from an URDF document. 
 *
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the 
 * European Union Seventh Framework Programme FP7/2007-2013 
 * under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by 
 * <a href=" mailto:perzylo@cs.tum.edu">Alexander Perzylo</a>
 * Technische Universitaet Muenchen
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
 * \author Alexander Perzylo
 * \version 1.0
 * \date 2011
 */

package org.roboearth.wp5.io;

import jargs.gnu.CmdLineParser;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.StringTokenizer;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.jgrapht.alg.CycleDetector;
import org.jgrapht.graph.DefaultEdge;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.jgrapht.traverse.TopologicalOrderIterator;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.SafeConstructor;

import edu.tum.cs.ias.knowrob.owl.OWLThing;


public class URDF2SRDL {

	// TODO read list of available capabilities from srdl2-cap.owl ? 
	public static enum Capability { // SRDL capabilities (case sensitive)
		move_base, move_arm, gripper_action, GraspingCapability, 
		OpeningADoorCapability, ClosingADoorCapability;
	}

	public static enum UrdfJointType { // URDF joint types (lower case only)
		continuous, fixed, revolute, prismatic, floating, planar;
	}

	public final static String SRDL_PATH = "file://@SRDL_PACKAGE_PATH@/owl/";
	public final static String ROBOEARTH_PATH = "file://@RE_PATH_PREFIX@/owl/";
	
//	public final static String SRDL2_IMPORT = "&srdl_path;srdl2.owl";
	public final static String SRDL2_IMPORT = "http://ias.cs.tum.edu/kb/srdl2.owl";
	public final static String SRDL2_COMP_IMPORT = "http://ias.cs.tum.edu/kb/srdl2-comp.owl";
	public final static String ROBOEARTH_IMPORT = "&roboearth_path;roboearth.owl";

	public final static String XSD = "http://www.w3.org/2001/XMLSchema#";
	public final static String RDF = "http://www.w3.org/1999/02/22-rdf-syntax-ns#";
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";
	public final static String OWL = "http://www.w3.org/2002/07/owl#";
	public final static String OWL2XML = "http://www.w3.org/2006/12/owl2-xml#";
	public final static String SRDL2 = "http://ias.cs.tum.edu/kb/srdl2.owl#";
	public final static String SRDL2_CAP = "http://ias.cs.tum.edu/kb/srdl2-cap.owl#";
	public final static String SRDL2_COMP = "http://ias.cs.tum.edu/kb/srdl2-comp.owl#";
	public final static String QUDT_UNIT = "http://qudt.org/vocab/unit#";
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";
	public final static String ROBOEARTH = "http://www.roboearth.org/kb/roboearth.owl#";
	

	public final static String indent = "    "; // one indentation step

	protected static Document urdf;
	protected static StringBuffer srdl;

	public URDF2SRDL() {
		urdf = null;
		srdl = null;
	}

	public boolean loadURDF(String filename) {

		System.out.println("[INFO]\tLoading URDF file '" + filename + "' ...");

		try {

			File file = new File(filename);
			DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
			DocumentBuilder db = dbf.newDocumentBuilder();
			urdf = db.parse(file);
			Element el = urdf.getDocumentElement();
			el.normalize();

			String docElementName = urdf.getDocumentElement().getNodeName();
			if (!docElementName.equalsIgnoreCase("robot")) {
				throw new RuntimeException("Document element is '" + 
						docElementName + "', but should be 'robot'");
			}
			
			srdl = new StringBuffer();

		} catch (Exception e) {
			System.out.println("[ERROR]\tCould not load URDF file:");
			e.printStackTrace();
			return false;
		}
		System.out.println("[INFO]\tLoading URDF file '" + filename + "' -> DONE");

		return true;  

	}

	protected boolean buildSRDL(String targetURI, ArrayList<Capability> caps, ArrayList<PlanningGroup> groups) {

		System.out.println("[INFO]\tCreating SRDL description ...");

		try {

			if (urdf == null) {
				System.out.println("[ERROR]\tNo valid URDF model parsed");
				return false;
			}

			Element docElement = urdf.getDocumentElement();
			if (!docElement.getNodeName().toLowerCase().equals("robot")) {
				System.out.println("[ERROR]\tCould not find element 'robot' in URDF file");
				return false;
			}

			String robotName = docElement.getAttribute("name").trim();
			if (robotName.length() == 0) {
				System.out.println("[ERROR]\tCould not find non-empty attribute 'name' in element 'robot'");
				return false;
			}

			// ontologies, which shall be imported into the SRDL description
			ArrayList<String> imports = new ArrayList<String>();
			imports.add(SRDL2_COMP_IMPORT);

			// build SRDL document
			srdl.append(getHeader(targetURI));
			srdl.append(getOntologyImports(targetURI, imports));
			srdl.append(getCapabilitySpec(robotName, caps));
			srdl.append(getLinksAndJoints(docElement, targetURI, robotName));
			srdl.append(getPlanningGroups(groups, robotName));
			srdl.append(getFooter());

		} catch (Exception e) {
			System.out.println("[ERROR]\tCould not build SRDL description:");
			e.printStackTrace();
			return false;
		}

		System.out.println("[INFO]\tCreating SRDL description -> DONE");

		return true;

	}

	public boolean saveSRDL(String filename, String targetURI, ArrayList<Capability> caps, ArrayList<PlanningGroup> parsedGroups) {

		if (!buildSRDL(targetURI, caps, parsedGroups)) {
			return false;
		}

		System.out.println("[INFO]\tSaving SRDL file '" + filename + "' ...");

		BufferedWriter bw = null;
		try {

			FileWriter fw = new FileWriter(filename);
			bw = new BufferedWriter(fw);
			bw.write(srdl.toString());

		} catch (Exception e) {
			System.out.println("\n[ERROR]\tCould not save SRDL description:");
			e.printStackTrace();
			return false;
		} finally {
			if (bw != null) {
				try {
					bw.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}

		System.out.println("[INFO]\tSaving SRDL file '" + filename + "' -> DONE");

		return true;

	}

	
	protected static String getHeader(String targetURI) {

		String targetURI2; // used for xml:base attribute without '#'
		if (targetURI.endsWith("#")) {
			targetURI2 = targetURI.substring(0, targetURI.length()-1);
		} else {
			targetURI2 = targetURI;
		}

		return "<?xml version=\"1.0\"?>\n\n" +
		"<!-- =============================================== -->\n" +
		"<!-- | This file was autogenerated by URDF2SRDL    | -->\n" +
		"<!-- =============================================== -->\n" +
		"\n<!DOCTYPE rdf:RDF [ \n" +
		"  <!ENTITY srdl_path '" + SRDL_PATH + "'>\n" +
		"  <!ENTITY owl \"" + OWL + "\" >\n" +
		"  <!ENTITY xsd \"" + XSD + "\" >\n" +
		"  <!ENTITY srdl2 \"" + SRDL2 + "\" >\n" +
		"  <!ENTITY owl2xml \"" + OWL2XML + "\" >\n" +
		"  <!ENTITY knowrob \"" + KNOWROB + "\" >\n" +
		"  <!ENTITY rdfs \"" + RDFS + "\" >\n" +
		"  <!ENTITY rdf \"" + RDF + "\" >\n" +
		"  <!ENTITY srdl2-comp \"" + SRDL2_COMP + "\" >\n" +
		"  <!ENTITY srdl2-cap \"" + SRDL2_CAP + "\" >\n" +
		"  <!ENTITY qudt-unit \"" + QUDT_UNIT + "\" >\n" +
		"  <!ENTITY robot \"" + targetURI + "\" >\n" +
		"]>\n\n\n" +
		"<rdf:RDF xmlns=\"" + targetURI + "\"\n" +
		"  xml:base=\"" + targetURI2 + "\"\n" +
		"  xmlns:knowrob=\"" + KNOWROB +  "\"\n" +
		"  xmlns:rdfs=\"" + RDFS +  "\"\n" +
		"  xmlns:owl2xml=\"" + OWL2XML +  "\"\n" +
		"  xmlns:srdl2=\"" + SRDL2 +  "\"\n" +
		"  xmlns:xsd=\"" + XSD +  "\"\n" +
		"  xmlns:owl=\"" + OWL +  "\"\n" +
		"  xmlns:srdl2-cap=\"" + SRDL2_CAP +  "\"\n" +
		"  xmlns:srdl2-comp=\"" + SRDL2_COMP +  "\"\n" +
		"  xmlns:qudt-unit=\"" + QUDT_UNIT +  "\"\n" +
		"  xmlns:rdf=\"" + RDF +  "\">\n";
		
	}

	protected static String getOntologyImports(String targetURI, ArrayList<String> imports) {

		String i1 = indent;
		String i2 = i1+indent;

		if (targetURI.endsWith("#")) {
			targetURI = targetURI.substring(0, targetURI.length()-1);
		}

		StringBuffer s = new StringBuffer();
		s.append("\n\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append(i1+"<!-- |   Ontology Imports                      | -->\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append("\n\n");
		s.append(i1+"<owl:Ontology rdf:about=\"" + targetURI + "\">\n");
		for (String imp : imports) {
			s.append(i2+"<owl:imports rdf:resource=\"" + imp + "\"/>\n");	
		}
		s.append(i1+"</owl:Ontology>\n");			

		return s.toString();

	}

	protected static String getCapabilitySpec(String robotName, ArrayList<Capability> caps) {

		String robotType = "&knowrob;Robot"; // TODO add as command line parameter
		
		String i1 = indent;
		String i2 = i1+indent;
		String i3 = i2+indent;
		String i4 = i3+indent;
		String i5 = i4+indent;
		String i6 = i5+indent;

		StringBuffer s = new StringBuffer();
		s.append("\n\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append(i1+"<!-- |   SRDL2 Robot Capabilities              | -->\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append("\n\n");
		s.append(i1+"<owl:Class rdf:about=\"&robot;" + robotName + "\">\n");
		s.append(i2+"<rdfs:subClassOf rdf:resource=\"" + robotType + "\"/>\n");
		if (caps.size() > 0) {
			s.append(i2+"<rdfs:subClassOf>\n");
			s.append(i3+"<owl:Class>\n");
			s.append(i4+"<owl:intersectionOf rdf:parseType=\"Collection\">\n");
			for (Capability c : caps) {
				s.append(i5+"<owl:Restriction>\n");
				s.append(i6+"<owl:onProperty rdf:resource=\"&srdl2-cap;hasCapability\"/>\n");
				s.append(i6+"<owl:someValuesFrom rdf:resource=\"&srdl2-cap;" + c + "\"/>\n");
				s.append(i5+"</owl:Restriction>\n");
			}
			s.append(i4+"</owl:intersectionOf>\n");
			s.append(i3+"</owl:Class>\n");
			s.append(i2+"</rdfs:subClassOf>\n");
		}
		s.append(i1+"</owl:Class>\n");

		return s.toString();
		
	}

	protected static String getLinksAndJoints(Element docElement, String targetURI, String robotName) {

		String i1 = indent;
		String i2 = i1+indent;

		StringBuffer s = new StringBuffer();

		ArrayList<Element> jointList = new ArrayList<Element>();
		ArrayList<Element> linkList = new ArrayList<Element>();
		
		// mapping of link to succeeding joint(s)
		HashMap<String,ArrayList<String>> succJoints;
		succJoints = new HashMap<String,ArrayList<String>>();
		
		// mapping of link to preceding rotational matrix
		HashMap<String,String> precJoints;
		precJoints = new HashMap<String,String>();
		
		// mapping from matrix index to matrix IRI
		HashMap<Integer,String> jointPoses = new HashMap<Integer, String>();
		
		// check if urdf description is complete and fill convenience data structures
		collectCheckedLinksAndJoints(docElement, jointList, linkList, succJoints, precJoints, jointPoses);

		//Read Gazebo Tags of docElement file
		GazeboInformationReader gazeboReader = new GazeboInformationReader(docElement);
		GeneralInformationReader generalReader = new GeneralInformationReader();
		
		Element rootLink = findRootLinkOfJointTree(jointList, linkList);
		String rootLinkName = rootLink.getAttribute("name");
		System.out.println("[INFO]\tRoot link of robot is '" + rootLinkName + "'");
		
		// add robot instance
		s.append(getRobotInstance(robotName, rootLinkName));
		
		s.append("\n\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append(i1+"<!-- |   Robot Joints                          | -->\n");
		s.append(i1+"<!-- =========================================== -->\n");
		
		// add joints
		int mCount = 0;
		String prefix = robotName + "_";
		double[] rotTransMat = new double[16];
		for (Element joint : jointList) {

			String jointName = joint.getAttribute("name");
			String jointType = joint.getAttribute("type");

			Element parent = (Element)joint.getElementsByTagName("parent").item(0);
			String parentLinkName = parent.getAttribute("link");

			Element child = (Element)joint.getElementsByTagName("child").item(0);
			String childLinkName = child.getAttribute("link");

			s.append("\n\n");
			s.append(i1+"<!-- &robot;" + prefix + jointName + " -->\n\n");
			s.append(i1+"<owl:NamedIndividual rdf:about=\"&robot;" + prefix + jointName + "\">\n");
			s.append(i2+"<rdf:type rdf:resource=\"&srdl2-comp;");
			switch (UrdfJointType.valueOf(jointType.toLowerCase())) {
			case continuous:
				s.append("ContinuousUrdfJoint");
				break;
			case fixed:
				s.append("FixedUrdfJoint");
				break;
			case floating:
				s.append("FloatingUrdfJoint");
				break;
			case planar:
				s.append("PlanarUrdfJoint");
				break;
			case prismatic:
				s.append("PrismaticUrdfJoint");
				break;
			case revolute:
				s.append("RevoluteUrdfJoint");
				break;
			}				
			s.append("\"/>\n");
			s.append(i2+"<srdl2-comp:urdfName>"+jointName+"</srdl2-comp:urdfName>\n");
//			s.append(i2+"<srdl2-comp:precedingLink rdf:resource=\"&robot;" + prefix + parentLinkName + "\"/>\n");
			s.append(i2+"<srdl2-comp:succeedingLink rdf:resource=\"&robot;" + prefix + childLinkName + "\"/>\n");
			//Include hasAttribute-Tags for gazebo information
			s.append(generalReader.getAttributeIncludesFor(i2, prefix, joint));
			//Include hasAttribute-Tags for gazebo information
			s.append(gazeboReader.getAttributeIncludesFor(i2, prefix, jointName));
//			s.append(i2+"<knowrob:orientation rdf:resource=\""+ jointPoses.get(++mCount) + "\"/>\n");
			s.append(i1+"</owl:NamedIndividual>\n");

			// add related rotation matrix
			Element origin = (Element)joint.getElementsByTagName("origin").item(0);

			String originXYZ = origin.getAttribute("xyz");
			StringTokenizer t = new StringTokenizer(originXYZ, " ");
			double x = Double.valueOf(t.nextToken().trim());
			double y = Double.valueOf(t.nextToken().trim());
			double z = Double.valueOf(t.nextToken().trim());

			String originRPY = origin.getAttribute("rpy");
			t = new StringTokenizer(originRPY, " ");
			double roll = Double.valueOf(t.nextToken().trim());
			double pitch = Double.valueOf(t.nextToken().trim());
			double yaw = Double.valueOf(t.nextToken().trim());

			rotTransMat = calcRotTransMatrix3D(x, y, z, roll, pitch, yaw, rotTransMat);
			
			// preceding joint, used for 'knowrob:relativeTo' object property
			String precMatrix = precJoints.get(parentLinkName);
			// current joint
			String currMatrix = precJoints.get(childLinkName);
			
			
			// create Proprioception instance for this joint
			
			String proprio = OWLThing.getUniqueID("&robot;Proprioception");
			
			s.append("\n\n");
			s.append(i1+"<!-- " + proprio + " -->\n\n");
			s.append(i1+"<owl:NamedIndividual rdf:about=\"" + proprio + "\">\n");
			s.append(i2+"<rdf:type rdf:resource=\"&knowrob;Proprioception\"/>\n");
			s.append(i2+"<knowrob:eventOccursAt rdf:resource=\"" + currMatrix + "\"/>\n");
			s.append(i2+"<knowrob:startTime rdf:resource=\"&robot;timepoint_1357020000\"/>\n");
			s.append(i2+"<knowrob:objectActedOn rdf:resource=\"&robot;" + prefix + jointName + "\"/>\n");
			s.append(i1+"</owl:NamedIndividual>\n\n");

			// create RotationMatrix3D for the default orientation
			s.append("\n\n");
			s.append(i1+"<!-- " + currMatrix + " -->\n\n");
			s.append(i1+"<owl:NamedIndividual rdf:about=\"" + currMatrix + "\">\n");
			s.append(i2+"<rdf:type rdf:resource=\"&knowrob;RotationMatrix3D\"/>\n");
			if (precMatrix != null) {
				s.append(i2+"<knowrob:relativeTo rdf:resource=\"" + precMatrix + "\"/>\n");	
			}
			for (int r=0; r<4; r++) { // 4 rows
				for (int c=0; c<4; c++) { // 4 columns
					double value = rotTransMat[c + r*4];
					String id = r + "" + c;
					s.append(i2+"<knowrob:m"+id+" rdf:datatype=\"&xsd;double\">"+value+"</knowrob:m"+id+">\n");
				}				
			}
			
			s.append(i1+"</owl:NamedIndividual>\n\n");
			
		}

		s.append("\n\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append(i1+"<!-- |   Robot Links                           | -->\n");
		s.append(i1+"<!-- =========================================== -->\n");
		
		// add links
		for (Element link : linkList) {

			String linkName = link.getAttribute("name");

			s.append("\n\n");
			s.append(i1+"<!-- &robot;" + prefix + linkName + " -->\n\n");
			s.append(i1+"<owl:NamedIndividual rdf:about=\"&robot;" + prefix + linkName + "\">\n");
			s.append(i2+"<rdf:type rdf:resource=\"&srdl2-comp;UrdfLink\"/>\n");
			s.append(i2+"<srdl2-comp:urdfName>" + linkName + "</srdl2-comp:urdfName>\n");
			//Include hasAttribute-Tags for gazebo information
			s.append(generalReader.getAttributeIncludesFor(i2, prefix, link));
			
			ArrayList<String> succs = succJoints.get(linkName);
			if (succs != null) {
				for (String succ : succs) {
					s.append(i2+"<srdl2-comp:succeedingJoint rdf:resource=\"&robot;" + prefix + succ + "\"/>\n");	
				}				
			}
			
			//Include hasAttribute-Tags for gazebo information
			s.append(gazeboReader.getAttributeIncludesFor(i2, prefix, linkName));

			s.append(i1+"</owl:NamedIndividual>\n");
		}

		return s.toString();

	}

	protected static String getRobotInstance(String robotName, String rootLinkInstance) {
		
		String i1 = indent;
		String i2 = i1+indent;
		
		String prefix = robotName + "_";
		String rob = prefix + "robot1"; // TODO assumes only 1 robot of a certain type exists
		
		StringBuffer s = new StringBuffer();
		s.append("\n\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append(i1+"<!-- |   Robot Instance                        | -->\n");
		s.append(i1+"<!-- =========================================== -->\n");
		s.append("\n\n");
		s.append(i1+"<owl:NamedIndividual rdf:about=\"&robot;" + rob + "\">\n");
		s.append(i2+"<rdf:type rdf:resource=\"&robot;" + robotName + "\"/>\n");
		s.append(i2+"<srdl2-comp:succeedingLink rdf:resource=\"&robot;" + prefix + rootLinkInstance +"\"/>\n");
		s.append(i1+"</owl:NamedIndividual>\n\n\n\n");
		
		
		s.append(i1+"<owl:NamedIndividual rdf:about=\"&robot;timepoint_1357020000\">\n");
		s.append(i2+"<rdf:type rdf:resource=\"&knowrob;TimePoint\"/>\n");
		s.append(i1+"</owl:NamedIndividual>\n\n\n");
		
		return s.toString();
		
	}

	protected static String getPlanningGroups(ArrayList<PlanningGroup> groups, String robotName) {

		String i1 = indent;
		String i2 = i1 + indent;

		StringBuffer s = new StringBuffer();

		if (groups != null && groups.size() > 0) {

			s.append("\n\n");
			s.append(i1 + "<!-- =========================================== -->\n");
			s.append(i1 + "<!-- | Planning Groups                         | -->\n");
			s.append(i1 + "<!-- =========================================== -->\n");
			
			for (PlanningGroup pg : groups) {

				s.append("\n\n");
				s.append(i1 + "<!-- &robot;" + pg.getName() + " -->\n\n");
				s.append(i1 + "<owl:NamedIndividual rdf:about=\"&robot;" + pg.getName() + "\">\n");
				s.append(i2 + "<rdf:type rdf:resource=\"&srdl2-comp;ComponentComposition\"/>\n");
				s.append(i2 + "<rdf:type rdf:resource=\"&srdl2-comp;ArmComponent\"/>\n");
				s.append(i2 + "<srdl2-comp:baseLinkOfComposition rdf:resource=\"&robot;" + pg.getBaseLink() + "\"/>\n");
				s.append(i2 + "<srdl2-comp:endLinkOfComposition rdf:resource=\"&robot;" + pg.getTipLink() + "\"/>\n");
				s.append(i1 + "</owl:NamedIndividual>\n");
				
			}

		}

		return s.toString();

	}
	protected static String getFooter() {
		return "\n\n</rdf:RDF>\n";
	}
	
	protected static void collectCheckedLinksAndJoints(Element docElement, 
			ArrayList<Element> jointList, 
			ArrayList<Element> linkList, 
			HashMap<String,ArrayList<String>> succJoints, 
			HashMap<String,String> precMatrices,
			HashMap<Integer,String> jointPoses) {

		// list of all joints
		jointList.clear();
		
		// list of all links
		linkList.clear();

		// mapping of link to succeeding joint(s)
		succJoints.clear();
		
		// mapping of link to preceding rotational matrix
		precMatrices.clear();
		
		// counter for naming rotational matrices
		int mCount = 1;
		
		// get all joints, check for needed attributes and elements
		NodeList joints = docElement.getElementsByTagName("joint");
		for (int n=0; n<joints.getLength(); n++) {

			Element joint = (Element)joints.item(n);
			if (!joint.getParentNode().isSameNode(docElement)) {
				continue;
			}

			String jointName = joint.getAttribute("name");
			if (jointName.isEmpty()) {
				throw new RuntimeException("Attribute 'name' not specified for joint " + n + " (URDF document order)");
			}

			String type = joint.getAttribute("type");
			if (type.isEmpty()) {
				throw new RuntimeException("Attribute 'type' not specified for joint '" + jointName + "'");
			} else {
				try {
					UrdfJointType.valueOf(type.toLowerCase());	
				} catch (IllegalArgumentException iae) {
					throw new RuntimeException("Unknown joint type '" + type + "' in joint '" + jointName + "'");
				}
			}

			NodeList children = joint.getElementsByTagName("child");
			if (children.getLength() == 0) {
				throw new RuntimeException("Element 'child' not specified for joint '" + jointName + "'");
			} else if (children.getLength() > 1) {
				throw new RuntimeException("Element 'child' specified " + children.getLength() + " times for joint '" + jointName + "'");
			}
			Element child = (Element)children.item(0);
			String childLink = child.getAttribute("link");
			if (childLink.isEmpty()) {
				throw new RuntimeException("Attribute 'link' not specified for element 'child' in joint '" + jointName + "'");	
			}
	
			NodeList parents = joint.getElementsByTagName("parent");
			if (parents.getLength() == 0) {
				throw new RuntimeException("Element 'parent' not specified for joint '" + jointName + "'");
			} else if (parents.getLength() > 1) {
				throw new RuntimeException("Element 'parent' specified " + parents.getLength() + " times for joint '" + jointName + "'");
			}
			Element parent = (Element)parents.item(0);
			String parentLink = parent.getAttribute("link");
			if (parentLink.isEmpty()) {
				throw new RuntimeException("Attribute 'link' not specified for element 'parent' in joint '" + jointName + "'");	
			}

			NodeList origins = joint.getElementsByTagName("origin");
			if (origins.getLength() == 0) {
				Element newOrigin = urdf.createElement("origin");
				newOrigin.setAttribute("xyz", "0 0 0");
				newOrigin.setAttribute("rpy", "0 0 0");
				joint.appendChild(newOrigin);
				origins = joint.getElementsByTagName("origin");
				System.out.println("[INFO]\tElement 'origin' not specified for joint '" + jointName + "' -> using zero vectors for 'xyz' and 'rpy'");
			} else if (origins.getLength() > 1) {
				throw new RuntimeException("Element 'origin' specified " + origins.getLength() + " times for joint '" + jointName + "'");
			} else {
				Element origin = (Element)origins.item(0);

				String originXYZ = origin.getAttribute("xyz");
				if (originXYZ.isEmpty()) {
					origin.setAttribute("xyz", "0 0 0");
					System.out.println("[INFO]\tAttribute 'xyz' not specified for element 'origin' in joint '" + jointName + "' -> using zero vector");	
				} else {
					StringTokenizer t = new StringTokenizer(originXYZ, " ");
					if (t.countTokens() != 3) {
						throw new RuntimeException("Wrong format in attribute 'xyz' of element 'origin' in joint '" + jointName + "'");
					} else {
						try {
							Double.valueOf(t.nextToken());
							Double.valueOf(t.nextToken());
							Double.valueOf(t.nextToken());						
						} catch (NumberFormatException nfe) {
							throw new RuntimeException("Wrong format in attribute 'xyz' of element 'origin' in joint '" + jointName + "'");	
						}
					}
				}

				String originRPY = origin.getAttribute("rpy");
				if (originRPY.isEmpty()) {
					origin.setAttribute("rpy", "0 0 0");
					System.out.println("[INFO]\tAttribute 'rpy' not specified for element 'origin' in joint '" + jointName + "' -> using zero vector");
				} else {
					StringTokenizer t = new StringTokenizer(originRPY, " ");
					if (t.countTokens() != 3) {
						throw new RuntimeException("Wrong format in attribute 'rpy' of element 'origin' in joint '" + jointName + "'");
					} else {
						try {
							Double.valueOf(t.nextToken());
							Double.valueOf(t.nextToken());
							Double.valueOf(t.nextToken());						
						} catch (NumberFormatException nfe) {
							throw new RuntimeException("Wrong format in attribute 'rpy' of element 'origin' in joint '" + jointName + "'");	
						}
					}
				}
			}

			jointList.add(joint);

			// fill mapping of link to succeeding joint
			if (!succJoints.containsKey(parentLink)) {
				ArrayList<String> succs = new ArrayList<String>(2);
				succs.add(jointName);
				succJoints.put(parentLink, succs);
			} else {
				succJoints.get(parentLink).add(jointName);
			}

			// fill mapping of link to preceding joint's rotational matrix
			jointPoses.put(mCount, OWLThing.getUniqueID("&robot;RotationMatrix3D"));
			precMatrices.put(childLink, jointPoses.get(mCount++));
			
		}

		// get all links, check for needed attributes and elements
		NodeList links = docElement.getElementsByTagName("link");
		for (int n=0; n<links.getLength(); n++) {

			Element link = (Element)links.item(n);
			if (!link.getParentNode().isSameNode(docElement)) {
				continue;
			}

			String linkName = link.getAttribute("name").trim();
			if (linkName.isEmpty()) {
				throw new RuntimeException("Attribute 'name' not specified for link " + n + " (URDF document order)");
			}

			linkList.add(link);
			
		}

	}

	protected static Element findRootLinkOfJointTree(ArrayList<Element> jointList, ArrayList<Element> linkList) {
		
		SimpleDirectedGraph<String, DefaultEdge> graph = new SimpleDirectedGraph<String, DefaultEdge>(DefaultEdge.class);
		
		for (Element joint : jointList) {
			
			Element parent = (Element)joint.getElementsByTagName("parent").item(0);
			Element child = (Element)joint.getElementsByTagName("child").item(0);

			String parentLinkName = parent.getAttribute("link");
			String childLinkName = child.getAttribute("link");
			
			if (!graph.containsVertex(parentLinkName)) {
				graph.addVertex(parentLinkName);	
			}
			if (!graph.containsVertex(childLinkName)) {
				graph.addVertex(childLinkName);	
			}
			
			if (graph.containsEdge(parentLinkName, childLinkName)) {
				throw new RuntimeException("Multiple joints between parent link '" + parentLinkName + "' and child link '" + childLinkName + "'");
			} else if (graph.containsEdge(childLinkName, parentLinkName)) {
				throw new RuntimeException("Inverse joint between parent link '" + parentLinkName + "' and child link '" + childLinkName + "' already exists");
			} else {
				graph.addEdge(parentLinkName, childLinkName);
			}
			
		}
		
		CycleDetector<String, DefaultEdge> cycDet;
		cycDet = new CycleDetector<String, DefaultEdge>(graph);
		Set<String> cycSet = cycDet.findCycles();
		if (cycSet.size() > 0) {
			Iterator<String> it = cycSet.iterator();
			StringBuffer sb = new StringBuffer(it.next());
			while (it.hasNext()) {
				sb.append(", " + it.next());
			}
			throw new RuntimeException("Cycle in joint graph, involving elements '" + sb + "'");
		}
		
		TopologicalOrderIterator<String,DefaultEdge> toit;
		toit = new TopologicalOrderIterator<String,DefaultEdge>(graph);
		String rootLinkName = toit.next();

		Element rootLink = null;
		for (Element link : linkList) {
			if (link.getAttribute("name").equals(rootLinkName)) {
				rootLink = link;
				break;
			}
		}
		
		if (rootLink == null) {
			throw new RuntimeException("Root joint's parent '" + rootLinkName + "' doesn't exist");
		}
		
		return rootLink;
		
	}

	protected static double[] calcRotTransMatrix3D(double x, double y, double z,
			double roll, double pitch, double yaw, double[] result) {

		if (result == null || result.length != 16) {
			result = new double[16];
		}

		// X is forward, Y is left, Z is up
		// fixed axis rotations about XYZ (roll pitch yaw)
		result[0]  = Math.cos(pitch) * Math.cos(yaw);
		result[1]  = - Math.cos(pitch) * Math.sin(yaw);
		result[2]  = Math.sin(pitch);
		result[3]  = x;
		result[4]  = Math.cos(roll) * Math.sin(yaw) + Math.cos(yaw) * Math.sin(roll) * Math.sin(pitch);
		result[5]  = Math.cos(roll) * Math.cos(yaw) - Math.sin(roll) * Math.sin(pitch) * Math.sin(yaw);
		result[6]  = - Math.cos(pitch) * Math.sin(roll);
		result[7]  = y;
		result[8]  = Math.sin(roll) * Math.sin(yaw) - Math.cos(roll) * Math.cos(yaw) * Math.sin(pitch);
		result[9]  = Math.cos(yaw) * Math.sin(roll) + Math.cos(roll) * Math.sin(pitch) * Math.sin(yaw);
		result[10] = Math.cos(roll) * Math.cos(pitch);
		result[11] = z;
		result[12] = 0;
		result[13] = 0;
		result[14] = 0;
		result[15] = 1;

		return result;

	}

	public ArrayList<PlanningGroup> loadPlanningGroups(String filename) {

		ArrayList<PlanningGroup> groups = new ArrayList<PlanningGroup>();

		System.out.println("[INFO]\tLoading planning groups from '" + filename + "' ...");

		try {
			InputStream input = new FileInputStream(new File(filename));

			Yaml yaml = new Yaml(new SafeConstructor());
			Map<?, ?> data = (Map<?, ?>) yaml.load(input);
			Set<?> set = data.keySet();
			String[] keys = set.toArray(new String[set.size()]);
			for (String key : keys) {
				if (key.equalsIgnoreCase("groups")) {
					ArrayList<?> values = (ArrayList<?>) data.get(key);
					Map<?, ?>[] groupMaps = values.toArray(new Map<?, ?>[values.size()]);
					PlanningGroup pg = null;
					for (Map<?, ?> groupMap : groupMaps) {
						Set<?> groupKeySet = groupMap.keySet();
						String[] groupKeys = groupKeySet.toArray(new String[groupKeySet.size()]);
						for (String groupKey : groupKeys) {
							if (groupKey.equalsIgnoreCase("name")) {

								if (pg == null) {
									pg = new PlanningGroup();
								}
								String groupName = (String) groupMap.get(groupKey);
								if (pg.isNameSet()) {
									throw new Exception(
											"[ERROR] name is set more than one time ("
													+ pg.getName() + ", "
													+ groupName + ")");
								} else {
									pg.setName(groupName);
								}

							} else if (groupKey.equalsIgnoreCase("base_link")) {

								if (pg == null) {
									pg = new PlanningGroup();
								}
								String baseLink = (String) groupMap.get(groupKey);
								if (pg.isBaseLinkSet()) {
									throw new Exception(
											"[ERROR] base_link is set more than one time ("
													+ pg.getBaseLink() + ", "
													+ baseLink + ")");
								} else {
									pg.setBaseLink(baseLink);
								}

							} else if (groupKey.equalsIgnoreCase("tip_link")) {

								if (pg == null) {
									pg = new PlanningGroup();
								}
								String tipLink = (String) groupMap.get(groupKey);
								if (pg.isTipLinkSet()) {
									throw new Exception(
											"[ERROR] tip_link is set more than one time ("
													+ pg.getTipLink() + ", "
													+ tipLink + ")");
								} else {
									pg.setTipLink(tipLink);
								}

							}

						}
						
						if (pg != null && pg.isAllSet()) {
							groups.add(pg);
							pg = null;
						} else {
							throw new Exception("[ERROR] planning group misses parameter");
						}
						
					}
					break;
				}
			}

		} catch (FileNotFoundException fnfe) {
			System.out.println("[WARNING] Could not find file " + filename
					+ "! " + "No planning groups will be created in SRDL.");
			fnfe.printStackTrace();
			groups.clear();
		} catch (Exception e) {
			System.out.println("[WARNING] Unexpected file format (" + filename
					+ ")! " + "No planning groups will be created in SRDL.");
			e.printStackTrace();
			groups.clear();
		}

		return groups;

	}

	protected class PlanningGroup {

		protected String name;
		protected String baseLink = null;
		protected String tipLink = null;

		public String getName() {
			return name;
		}

		public String getBaseLink() {
			return baseLink;
		}

		public String getTipLink() {
			return tipLink;
		}

		public boolean isAllSet() {
			return isNameSet() && isBaseLinkSet() && isTipLinkSet();
		}		
		
		public boolean isNameSet() {
			return (name != null);
		}

		public boolean isBaseLinkSet() {
			return (baseLink != null);
		}

		public boolean isTipLinkSet() {
			return (tipLink != null);
		}

		public void setName(String name) {
			this.name = name;
		}

		public void setBaseLink(String baseLink) {
			this.baseLink = baseLink;
		}

		public void setTipLink(String tipLink) {
			this.tipLink = tipLink;
		}

	}
	protected static void printUsage() {

		StringBuffer usage = new StringBuffer(); 
		usage.append("\nUsage:\n" +
				"This program builds an SRDL description based on the " +
				"URDF model of a robot.\n" +
				"\nOptions:\n" +
				"-h, --help\tshow this description\n" +
				"-u, --urdf\tURDF file to read and work with\n" +
				"-s, --srdl\ttarget SRDL file\n" +
				"-i, --uri\tURI to use for SRDL description\n" +
				"-c, --caps\tlist of SRDL capabilities the robot shall " +
				"advertise. Single capabilities must be seperated by commas. " +
				"No spaces allowed. List of available capabilities:  ");

		Capability[] capList = Capability.values();
		if (capList.length > 0) {
			for (int i=0; i<capList.length-1; i++) {
				usage.append(capList[i].toString()+", ");
			}
			usage.append(capList[capList.length-1].toString());
		}

		usage.append("\n\nExample:\n" +
				"java -jar URDF2SRDL.jar -u pr2.urdf -s pr2.owl " +
				"-i http://ias.cs.tum.edu/kb/PR2.owl# " +
				"-c move_base,move_arm,GraspingCapability\n");

		System.out.println(usage.toString());

	}

	public static void main(String[] args) {

		CmdLineParser parser = new CmdLineParser();
		CmdLineParser.Option help = parser.addBooleanOption('h', "help");
		CmdLineParser.Option urdfFile = parser.addStringOption('u', "urdf");
		CmdLineParser.Option groupsFile = parser.addStringOption('g', "groups");
		CmdLineParser.Option srdlFile = parser.addStringOption('s', "srdl");
		CmdLineParser.Option targetUri = parser.addStringOption('i', "uri");
		CmdLineParser.Option capsList = parser.addStringOption('c', "caps");

		System.out.println();

		try {
			parser.parse(args);
		} catch (CmdLineParser.OptionException e) {
			System.out.println("[ERROR]\t" + e.getMessage());
			printUsage();
			System.exit(2);
		}

		boolean showUsage = (Boolean) parser.getOptionValue(help, Boolean.FALSE);
		String urdfValue = (String) parser.getOptionValue(urdfFile);
		String groupsValue = (String) parser.getOptionValue(groupsFile);
		String srdlValue = (String) parser.getOptionValue(srdlFile);
		String uriValue = (String) parser.getOptionValue(targetUri);
		String capsValue = (String) parser.getOptionValue(capsList);
		ArrayList<Capability> caps = new ArrayList<Capability>();

		String[] otherArgs = parser.getRemainingArgs();
		for (String s : otherArgs) {
			System.out.println("[INFO]\tIgnoring other argument '" + s + "'");
		}

		if (!showUsage) {
			if (urdfValue == null || urdfValue.length() == 0) {
				System.out.println("[ERROR]\tNo URDF file specified (-u / --urdf)");
				showUsage = true;
			}
			if (srdlValue == null || srdlValue.length() == 0) {
				System.out.println("[ERROR]\tNo target SRDL file specified (-s / --srdl)");
				showUsage = true;
			}
			if (uriValue == null || uriValue.length() == 0) {
				System.out.println("[ERROR]\tNo URI specified to use for SRDL description (-i / --uri)");
				showUsage = true;
			} else {
				if (!uriValue.endsWith("#")) {
					uriValue += "#";
				}
			}
			if (capsValue == null) {
				System.out.println("[WARN]\tNo SRDL capabilities specified (-c / --caps)");
			} else {
				StringTokenizer t = new StringTokenizer(capsValue, ",");
				while (t.hasMoreTokens()) {
					String token = t.nextToken().trim();
					Capability c = null;
					try {
						c = Capability.valueOf(token);
					} catch (Exception e) {
						System.out.println("[ERROR]\tUnknown SRDL capability '" + token + "'");
						showUsage = true;
					}
					if (c != null) {
						caps.add(c);
					}
				}
			}
		}

		if (showUsage) {
			printUsage();
			System.exit(2);
		}

		URDF2SRDL ui = new URDF2SRDL();
		if (ui.loadURDF(urdfValue)) {
			ArrayList<PlanningGroup> parsedGroups = null;
			if (groupsValue != null) {
				parsedGroups = ui.loadPlanningGroups(groupsValue);
			}
			ui.saveSRDL(srdlValue, uriValue, caps, parsedGroups);
		}

		System.out.println();

	}

}
