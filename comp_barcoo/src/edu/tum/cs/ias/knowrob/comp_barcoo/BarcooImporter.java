
package edu.tum.cs.ias.knowrob.comp_barcoo;

import java.io.File;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.SimpleIRIMapper;
import org.semanticweb.owlapi.vocab.OWL2Datatype;
import org.semanticweb.owlapi.vocab.OWLFacet;

import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.*;

public class BarcooImporter {
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
	public final static String BARCOO = "http://www.barcoo.com/barcoo.owl#";
	public final static String BARCOO_PATH = "owl/barcoo.owl"; //TO BE CHANGED LATER
	
	//
	//
	////////////////////////////////////////////////////////////////////////////////
	
	private OWLOntologyManager manager;
	private IRI documentIRI;
	private IRI ontologyIRI;
	private OWLOntology ontology;
	private SimpleIRIMapper mapper;
	private OWLDataFactory factory;
	private Document doc;
	private HashMap<String, OWLDataProperty> data_properties; 
	
	public BarcooImporter()
	{
		try
		{
			// Get hold of an ontology manager
			manager = OWLManager.createOWLOntologyManager();
			
			// Let's load an ontology from the web
			File file = new File(BARCOO_PATH);
			documentIRI = IRI.create(file);
			ontologyIRI = IRI.create(BARCOO);
			ontology = manager.loadOntologyFromOntologyDocument(documentIRI);
			
			System.out.println("Loaded ontology: " + ontology);
			
			// Set up a mapping, which maps the ontology to the document IRI
			mapper = new SimpleIRIMapper(ontologyIRI, documentIRI);
			manager.addIRIMapper(mapper);
			factory = manager.getOWLDataFactory();
			data_properties = new HashMap<String, OWLDataProperty>();
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}		
	}
	
	protected Vector<AttrValue> parseXML(File file) throws Exception
	{				
		doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file);
		//get the root element
		Element docEle = doc.getDocumentElement();
		//get a nodelist of elements
		NodeList nl = docEle.getElementsByTagName("product");
		Vector<AttrValue> list = new Vector<AttrValue>();
		
		if(nl != null && nl.getLength() > 0) 
		{
			String arg1 = null;
			String arg2 = null;
			getNodeInfo(nl,1,list,arg1,arg2);
		}
		return list;
	}
	
	protected void getNodeInfo(NodeList nodeList, int num,Vector<AttrValue> list,String arg1, String arg2)
	{
		
		Node node;		
		Text myText = null;
		Boolean exception;
		
		for(int i = 0;i < nodeList.getLength(); i++)
		{			
			node  = nodeList.item(i);
			exception = false;
			try
			{
					myText = (Text) node;
			}
			catch(ClassCastException ex)
			{
				exception = true;
			}
			if(exception)
			{
				if(node.getNodeName() != null)
				{					
					arg1 = node.getNodeName();
				}
			}
			else
			{
				arg2 = myText.getNodeValue();				
				list.addElement(new AttrValue(arg1, arg2));
			}			
			
			if(node.hasChildNodes())
			{
				getNodeInfo(node.getChildNodes(),++num,list,arg1,arg2);
			}			
		}
	}	
	
	protected OWLClass createClass(String className, String parent)
	{	
		className.replace(' ', '_'); // replace spaces with underscores
		
		OWLClass myClassA = factory.getOWLClass(IRI.create(ontologyIRI + className));
		OWLClass myClassB = factory.getOWLClass(IRI.create(ontologyIRI + parent ));
		
		
		// The following axiom states that ClassA is subclass of ClassB
		OWLAxiom axiom = factory.getOWLSubClassOfAxiom(myClassA, myClassB);
		AddAxiom addAxiom = new AddAxiom(ontology, axiom);
		
		// We now use the manager to apply the change
		manager.applyChange(addAxiom);
		
		return myClassA;
		
	}
	protected void create_data_properties(Vector<AttrValue> list) 
	{
		OWLDataPropertyRangeAxiom rangeAxiom;
		OWLDatatype stringDatatype  = factory.getOWLDatatype(OWL2Datatype.XSD_STRING.getIRI());
		
		//The following is a data property stuff
		for(int i = 1; i < list.size(); i++)
		{
			OWLDataProperty data_prop = factory.getOWLDataProperty(IRI.create(ontologyIRI + prologify(list.elementAt(i).attribute)));
			rangeAxiom = factory.getOWLDataPropertyRangeAxiom(data_prop, stringDatatype);
			manager.addAxiom(ontology, rangeAxiom); 
			data_properties.put(prologify(list.elementAt(i).attribute), data_prop);			
		}
		
	}
	
	protected void add_property_restrictions(OWLClass myClass, Vector<AttrValue> list) 
	{
		for(int i = 1 ; i< list.size(); i++)
		{
			OWLLiteral val = factory.getOWLTypedLiteral(prologify(list.elementAt(i).value));
			OWLDataHasValue restr = factory.getOWLDataHasValue(this.data_properties.get(prologify(list.elementAt(i).attribute)), val);

			manager.addAxiom(ontology, factory.getOWLSubClassOfAxiom(myClass, restr));
		}
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
	
	public void Process(File file) throws Exception
	{		
		//PARSE XML FILE
		Vector<AttrValue> list = parseXML(file);
		
		//for(int i = 0; i < list.size(); i++)
		//	System.out.println(list.elementAt(i).attribute+": "+list.elementAt(i).value);
		
		//CREATE OWL CLASS
		OWLClass myClass = createClass(list.elementAt(0).value, "BarcooObject");
		
		//CREATE PROPERTIES IN CASE THEY DON'T EXIST
		create_data_properties(list); 
		
		//ADD DATA PROPERTIES TO THE CLASS
		add_property_restrictions(myClass,list);
		
		//Save Ontology		
		manager.saveOntology(ontology);
		
		
		
	}
	public static void main(String[] args)
	{		
		BarcooImporter b = new BarcooImporter();
		try
		{
			File f = new File(args[1]);
			b.Process(f);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
		
	}

}
class AttrValue
{
	public String attribute;
	public String value;
	
	public AttrValue(String attribute, String value)
	{
		this.attribute = attribute;
		this.value = value;
	}
	
}
