/*
 * Copyright (C) 2010 by Moritz Tenorth
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package edu.tum.cs.ias.knowrob.comp_germandeli;

import java.io.File;
import java.util.HashMap;
import javax.xml.parsers.*;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.semanticweb.owlapi.vocab.OWL2Datatype;
import org.semanticweb.owlapi.vocab.OWLRDFVocabulary;

import org.w3c.dom.*;




public class GermanDeliOWLImport {


	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//
	
	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";
	
	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";
	
	// Base IRI for semantic map ontology	
	public final static String GERMANDELI = "http://ias.cs.tum.edu/kb/germandeli.owl#";
	

	////////////////////////////////////////////////////////////////////////////////
	// Define names of XML tags and attributes in the input file
	//
	public final static String XML_GROUPNAME 	= "group";
	public final static String XML_PAGENAME 	= "page";
	public final static String XML_PRODNAME 	= "product";
	
	public final static String XML_PROP_DESC 	= "description";
	public final static String XML_PROP_ID 		= "id";
	public final static String XML_PROP_LINK 	= "link";
	public final static String XML_PROP_NAME 	= "name";
	public final static String XML_PROP_LOC		= "location";
	public final static String XML_PROP_PERISH 	= "perishability";
	public final static String XML_PROP_COUNTRY	= "country_of_origin";
	public final static String XML_PROP_CODE	= "product_code";
	public final static String XML_PROP_PRICE	= "sale_price";
	public final static String XML_PROP_WEIGHT	= "weight";
	public final static String XML_PROP_BRAND	= "brand";
	
	
	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(GERMANDELI);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("germandeli:", GERMANDELI);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
	}
	
	
	HashMap<String, OWLDataProperty> data_properties; 
	
	OWLDataFactory factory;
	OWLOntologyManager manager;
	DefaultPrefixManager pm;
	
	int inst_counter=0;	// counter to create unique instance identifiers
	
	
	
	
	/**
	 * 
	 */
	public GermanDeliOWLImport() {
	
		data_properties = new HashMap<String, OWLDataProperty>();
	}
	
	
	
	/**
	 * 
	 * @param xmlfile
	 * @return
	 */
	public OWLOntology import_xml(String xmlfile) {
		

		OWLOntology ontology = null;
		
		try{

			// read XML file
			DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
			DocumentBuilder db = dbf.newDocumentBuilder();
			
			System.out.println("Input: " + xmlfile);
			
			String[] path= xmlfile.split("/");
			String filename = path[path.length-1];
			String mapname = filename.split("\\.")[0];
			  
			Document doc = db.parse(new File(xmlfile));

		      
			

			// Create ontology manager and data factory
			manager = OWLManager.createOWLOntologyManager();
			factory = manager.getOWLDataFactory();
			
			// Get prefix manager using the base IRI of the JoystickDrive ontology as default namespace
			pm = PREFIX_MANAGER;
			
			// Create empty OWL ontology
			ontology = manager.createOntology(IRI.create(GERMANDELI));
			manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());
			
			// Import KnowRob ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(KNOWROB));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);
			
			// Get classes from the KnowRob ontology
			OWLClass clsVelocityCommand = factory.getOWLClass("knowrob:Action", pm);
			
			
			// fill the internal list of object properties to be used 
			create_data_properties(ontology);
			
			
			// Iterate over all classes in the XML file and create the respective OWL representation
			NodeList nl = doc.getElementsByTagName(XML_GROUPNAME);
			
			// create parent class
			OWLClass germandeli_prod = create_owl_class("GermanDeliObject", factory.getOWLClass("knowrob:HumanScaleObject", pm), ontology);
			
			for(int i = 0; i < nl.getLength(); i++) {
				Node cn = nl.item(i);
				
				if(cn.getNodeName().equals(XML_GROUPNAME))
					parse_group(cn, germandeli_prod, ontology);
				
				else if (cn.getNodeName().equals(XML_PAGENAME))
					parse_page(cn, germandeli_prod, ontology);
				
				else if (cn.getNodeName().equals(XML_PRODNAME))
						parse_product(cn, germandeli_prod, ontology);
			}
			
			
		    
			// save the ontology to a file
			//SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");
			//String outfile = "ias_semantic_map_"+sdf.format(new Date())+".owl";
			//saveOntologyToFile(ontology, outfile);
			File file = new File("/work/germandeli/germandeli.owl"); 
			manager.saveOntology(ontology, IRI.create(file.toURI())); 

		} catch (Exception e) {
			ontology = null;
			e.printStackTrace();
		}

		return ontology;
	}
    

	
	
	/**
	 * 
	 * @param group
	 * @param parent
	 * @param ontology
	 */
	protected void parse_group(Node group, OWLClass parent, OWLOntology ontology) {

        OWLClass current_group = parent;
		NamedNodeMap nnm =  group.getAttributes();

		
        // only create a separate group if it has a name and link set
        // otherwise just re-use the parent
        
		if((nnm.getNamedItem(XML_PROP_LINK)!=null) && (nnm.getNamedItem(XML_PROP_NAME) != null)) {
			
	        HashMap<String, String> attr = new HashMap<String, String>();
	        attr.put(XML_PROP_LINK,   "http://www.germandeli.com/"+nnm.getNamedItem(XML_PROP_LINK).getTextContent());
			
			current_group = create_owl_class(prologify(nnm.getNamedItem(XML_PROP_NAME).getTextContent()), parent, ontology);			
			add_property_restrictions(current_group, attr, ontology);
		}
		
		// recursively parse child nodes
		for(int i=0;i<group.getChildNodes().getLength();i++) {
			Node cn = group.getChildNodes().item(i);
			
			if(cn.getNodeName().equals(XML_GROUPNAME))
				parse_group(cn, current_group, ontology);
			
			else if (cn.getNodeName().equals(XML_PAGENAME))
				parse_page(cn, current_group, ontology);
		}
	}
	
	
	/**
	 * 
	 * @param page
	 * @param parent
	 * @param ontology
	 */
	protected void parse_page(Node page, OWLClass parent, OWLOntology ontology) {
		
        NamedNodeMap nnm =  page.getAttributes();
        HashMap<String, String> attr = new HashMap<String, String>();

        OWLClass current_page = parent;
        if( (nnm.getNamedItem(XML_PROP_LINK)!=null) && (nnm.getNamedItem(XML_PROP_NAME)!=null)) {
        	attr.put(XML_PROP_LINK,   "http://www.germandeli.com/"+nnm.getNamedItem(XML_PROP_LINK).getTextContent());
	
			// create ID from the link without '.html'
			current_page = create_owl_class(prologify(nnm.getNamedItem(XML_PROP_NAME).getTextContent()), parent, ontology);
			add_property_restrictions(current_page, attr, ontology);
        }
        
		// recursively parse child nodes
		for(int i=0;i<page.getChildNodes().getLength();i++) {
			Node cn = page.getChildNodes().item(i);
			
			if(cn.getNodeName().equals(XML_GROUPNAME))
				parse_group(cn, current_page, ontology);

			else if (cn.getNodeName().equals(XML_PAGENAME))
				parse_page(cn, current_page, ontology);

			else if (cn.getNodeName().equals(XML_PRODNAME))
				parse_product(cn, current_page, ontology);
		}
	}

	
	/**
	 * 
	 * @param product
	 * @param parent
	 * @param ontology
	 */
	protected void parse_product(Node product, OWLClass parent, OWLOntology ontology) {
		
        NamedNodeMap nnm =  product.getAttributes();
        HashMap<String, String> attr = new HashMap<String, String>();
        
        
        attr.put(XML_PROP_ID,     nnm.getNamedItem(XML_PROP_ID).getTextContent());
        attr.put(XML_PROP_CODE,   nnm.getNamedItem(XML_PROP_CODE).getTextContent());
		attr.put(XML_PROP_DESC,   nnm.getNamedItem(XML_PROP_DESC).getTextContent());
		attr.put(XML_PROP_LINK,   nnm.getNamedItem(XML_PROP_LINK).getTextContent());
		attr.put(XML_PROP_LOC,    nnm.getNamedItem(XML_PROP_LOC).getTextContent());
		attr.put(XML_PROP_PERISH, nnm.getNamedItem(XML_PROP_PERISH).getTextContent());
		attr.put(XML_PROP_COUNTRY,nnm.getNamedItem(XML_PROP_COUNTRY).getTextContent());
		attr.put(XML_PROP_PRICE,  nnm.getNamedItem(XML_PROP_PRICE).getTextContent());
		attr.put(XML_PROP_WEIGHT, nnm.getNamedItem(XML_PROP_WEIGHT).getTextContent());
		attr.put(XML_PROP_BRAND,  nnm.getNamedItem(XML_PROP_BRAND).getTextContent());
		
		OWLClass current_product = create_owl_class(prologify(nnm.getNamedItem(XML_PROP_DESC).getTextContent()), parent, ontology);
		add_property_restrictions(current_product, attr, ontology);        
	}
	

	
	/**
	 * 
	 * @param name
	 * @param parent
	 * @param ontology
	 * @return
	 */
	protected OWLClass create_owl_class(String name, OWLClass parent, OWLOntology ontology) {

		OWLClass res = factory.getOWLClass(IRI.create(GERMANDELI + name));
		OWLAxiom subClassAxiom = factory.getOWLSubClassOfAxiom(res, parent);
		manager.addAxiom(ontology, subClassAxiom); 
		 
		return res;
	}
	
	
	
	/**
	 * 
	 * @param cl
	 * @param attr
	 * @param ontology
	 */
	protected void add_property_restrictions(OWLClass cl, HashMap<String, String> attr, OWLOntology ontology) {
		
		
		// handle perishability separately: subClassOf Perishable
		if(attr.containsKey(XML_PROP_PERISH) && !attr.get(XML_PROP_PERISH).equals("")) {
			
			if(attr.get(XML_PROP_PERISH).equals("Refrigerated")) {
				OWLClass perish = factory.getOWLClass(IRI.create(KNOWROB + "Perishable"));
				manager.addAxiom(ontology, factory.getOWLSubClassOfAxiom(cl, perish));
				attr.remove(XML_PROP_PERISH);	
			} else if(attr.get(XML_PROP_PERISH).equals("Frozen")) {
				OWLClass perish = factory.getOWLClass(IRI.create(KNOWROB + "Frozen"));
				manager.addAxiom(ontology, factory.getOWLSubClassOfAxiom(cl, perish));
				attr.remove(XML_PROP_PERISH);	
			} else if(attr.get(XML_PROP_PERISH).equals("Heat-Sensitive")) {
				OWLClass perish = factory.getOWLClass(IRI.create(KNOWROB + "HeatSensitive"));
				manager.addAxiom(ontology, factory.getOWLSubClassOfAxiom(cl, perish));
				attr.remove(XML_PROP_PERISH);	
			} else {
				attr.remove(XML_PROP_PERISH);
			}
		}

		// set description as RDFS:LABEL
		if(attr.containsKey(XML_PROP_DESC)) {
			
			OWLAnnotation commentAnno = factory.getOWLAnnotation(
					factory.getOWLAnnotationProperty(OWLRDFVocabulary.RDFS_LABEL.getIRI()),
					factory.getOWLStringLiteral(prologify(attr.get(XML_PROP_DESC)), "en"));
			
			manager.addAxiom(ontology, factory.getOWLAnnotationAssertionAxiom(cl.getIRI(), commentAnno));
			attr.remove(XML_PROP_DESC);
		}
		

		// set link as RDFS:seeAlso
		if(attr.containsKey(XML_PROP_LINK)) {
			
			OWLAnnotation commentAnno = factory.getOWLAnnotation(
					factory.getOWLAnnotationProperty(OWLRDFVocabulary.RDFS_SEE_ALSO.getIRI()),
					factory.getOWLStringLiteral(attr.get(XML_PROP_LINK)));
			
			manager.addAxiom(ontology, factory.getOWLAnnotationAssertionAxiom(cl.getIRI(), commentAnno));
			attr.remove(XML_PROP_LINK);
		}
		
		
		
		
		// currently not using the loc key
		if(attr.containsKey(XML_PROP_LOC)) { 
			attr.remove(XML_PROP_LOC);
		}
		
		
		// for the other attributes: add as hasValue restriction
		for(String key : attr.keySet()) {
			
			// don't add properties for empty values
			if(attr.get(key).equals(""))
				continue;
			
			OWLLiteral val = factory.getOWLTypedLiteral(attr.get(key));
			OWLDataHasValue restr = factory.getOWLDataHasValue(this.data_properties.get(key), val);

			manager.addAxiom(ontology, factory.getOWLSubClassOfAxiom(cl, restr));
			
		}
		
	}
	


	/**
	 * Create new data type properties and read existing ones (e.g. from KnowRob)
	 * into the internal buffer map
	 * 
	 * @param ontology
	 */
	protected void create_data_properties(OWLOntology ontology) {


		OWLDataProperty data_prop;
		OWLDataPropertyRangeAxiom rangeAxiom;
		OWLDatatype integerDatatype = factory.getIntegerOWLDatatype();  
		OWLDatatype stringDatatype  = factory.getOWLDatatype(OWL2Datatype.XSD_STRING.getIRI());
		
		data_properties.put(XML_PROP_DESC,    factory.getOWLDataProperty("rdfs:label", pm));
		data_properties.put(XML_PROP_PRICE,   factory.getOWLDataProperty("knowrob:purchasePrice", pm));
		data_properties.put(XML_PROP_WEIGHT,  factory.getOWLDataProperty("knowrob:weight", pm));
		

		data_prop = factory.getOWLDataProperty(IRI.create(GERMANDELI + "productCode"));
		rangeAxiom = factory.getOWLDataPropertyRangeAxiom(data_prop, integerDatatype);
		manager.addAxiom(ontology, rangeAxiom); 
		data_properties.put(XML_PROP_CODE, data_prop);
		

		data_prop = factory.getOWLDataProperty(IRI.create(GERMANDELI + "productID"));
		rangeAxiom = factory.getOWLDataPropertyRangeAxiom(data_prop, stringDatatype);
		manager.addAxiom(ontology, rangeAxiom); 
		data_properties.put(XML_PROP_ID, data_prop);
		
		
		data_prop = factory.getOWLDataProperty(IRI.create(GERMANDELI + "linkToProductPage"));
		data_properties.put(XML_PROP_LINK, data_prop);
		rangeAxiom = factory.getOWLDataPropertyRangeAxiom(data_prop, stringDatatype);
		manager.addAxiom(ontology, rangeAxiom);
		
		
		data_prop = factory.getOWLDataProperty(IRI.create(GERMANDELI + "brand"));
		data_properties.put(XML_PROP_BRAND, data_prop);
		rangeAxiom = factory.getOWLDataPropertyRangeAxiom(data_prop, stringDatatype);
		manager.addAxiom(ontology, rangeAxiom);
		
		
		data_prop = factory.getOWLDataProperty(IRI.create(GERMANDELI + "countryOfOrigin"));
		data_properties.put(XML_PROP_COUNTRY, data_prop);
		rangeAxiom = factory.getOWLDataPropertyRangeAxiom(data_prop, stringDatatype);
		manager.addAxiom(ontology, rangeAxiom);
		
	}
	
	
	

	protected String prologify(String str) {

		// first replace well-known special characters
		str=str.replaceAll("&quot;", "");
		str=str.replaceAll("&amp;",  "_");
		str=str.replaceAll(",", "_");
		str=str.replaceAll(" ", "_");
		str=str.replaceAll("-", "_");
		str=str.replaceAll("/", "_");
		
		// replace common umlaut characters
		str=str.replaceAll("&Auml;", "Ae");
		str=str.replaceAll("&auml;", "ae");
		str=str.replaceAll("&Ouml;", "Oe");
		str=str.replaceAll("&ouml;", "oe");
		str=str.replaceAll("&Uuml;", "Ue");
		str=str.replaceAll("&uuml;", "ue");
		str=str.replaceAll("&szlig;","ss");
		
		str=str.replaceAll("Ä", "Ae");
		str=str.replaceAll("ä", "ae");
		str=str.replaceAll("Ö", "Oe");
		str=str.replaceAll("ö", "oe");
		str=str.replaceAll("Ü", "Ue");
		str=str.replaceAll("ü", "ue");
		str=str.replaceAll("ß", "ss");
		
		// replace everything else that is neither char nor num nor '_'
		str=str.replaceAll("[^A-Za-z0-9_]","");
		
		// remove multiple underscores
		str=str.replaceAll("_+", "_");
		str=str.replaceAll("^_", "");
		
		return str;
	}

	
	public static void main(String[] args) {
		
		GermanDeliOWLImport gdi = new GermanDeliOWLImport();
		System.out.println(gdi.import_xml("/work/germandeli/download/structure.xml").toString());
		
		
	}

	
}

