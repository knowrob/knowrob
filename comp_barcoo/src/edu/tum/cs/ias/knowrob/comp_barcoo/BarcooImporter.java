
package edu.tum.cs.ias.knowrob.comp_barcoo;

import java.io.*;
import java.net.*;
import java.util.*;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.reasoner.ConsoleProgressMonitor;
import org.semanticweb.owlapi.reasoner.NodeSet;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerConfiguration;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;
import org.semanticweb.owlapi.util.SimpleIRIMapper;
import org.semanticweb.owlapi.vocab.OWL2Datatype;
import org.semanticweb.owlapi.vocab.OWLFacet;
import org.semanticweb.HermiT.Reasoner;

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
	public static String BARCOO_PATH = "owl/barcoo.owl"; //TO BE CHANGED LATER
	
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
	private MyNode treeNode;
	private HashMap<String, String> classMapping;
	private OWLReasoner reasoner;
	
	public BarcooImporter(String barcooPath, String barcooMapping)
	{
		try
		{
			BARCOO_PATH = barcooPath;
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
			
			//Load class mapping from disk
			classMapping = new HashMap<String, String>();
			BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(barcooMapping)));
			String line;
			StringTokenizer stringTokenizer = null;
			while((line = br.readLine()) != null)
			{				
				stringTokenizer = new StringTokenizer(line,"\t");
				String key = stringTokenizer.nextToken();
				String value = stringTokenizer.nextToken();
				classMapping.put(key, value);
				
			}
			//Class mapping loaded
			
			OWLReasonerFactory reasonerFactory = new Reasoner.ReasonerFactory();
			ConsoleProgressMonitor progressMonitor = new ConsoleProgressMonitor();
			OWLReasonerConfiguration config = new SimpleConfiguration(progressMonitor);
			reasoner = reasonerFactory.createReasoner(this.ontology, config);
			
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
		NodeList l = docEle.getElementsByTagName("answer");
		Node n = l.item(0);
		n = n.getFirstChild();		
		if(n.getNodeValue() == null || n.getNodeValue() == "0")
			return null;
		
		NodeList nl = docEle.getElementsByTagName("product");
		Vector<AttrValue> list = new Vector<AttrValue>();
		treeNode = new MyNode(false, "product");
		if(nl != null && nl.getLength() > 0) 
		{
			String arg1 = null;
			String arg2 = null;
			getNodeInfo(nl,1,list,arg1,arg2);
			constructTree(nl, treeNode);			
		}
		return list;
	}
	
	protected void constructTree(NodeList nodelist, MyNode parent)
	{
		Node node;		
		Text myText = null;
		Boolean exception;
		
		for(int i = 0; i < nodelist.getLength(); i++)
		{
			node = nodelist.item(i);
			exception = false;
			try
			{
				myText = (Text) node;				
			}
			catch(Exception ex)
			{
				exception = true;
			}
			
			if(exception) // THIS MEANS THAT THE NODE IS NOT TEXT
			{
				if(node.getNodeName() != null)
				{
					MyNode myNode = new MyNode(false, node.getNodeName());
					try
					{
						parent.addChild(myNode);
						constructTree(node.getChildNodes(),myNode);
					}
					catch(Exception ex)
					{
						ex.printStackTrace();
					}					
				}
			}
			else
			{
				MyNode myNode = new MyNode(true,myText.getNodeValue());
				try
				{
					parent.addChild(myNode);					
				}
				catch(Exception ex)
				{
					ex.printStackTrace();
				}	
			}
		}
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
					//System.out.print("\n" + arg1);
				}
			}
			else
			{
				arg2 = myText.getNodeValue();				
				list.addElement(new AttrValue(arg1, arg2));
				//System.out.println(": " + arg2);
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
	//THIS FUNCTION ASSUMES THAT THE PARENT NODE IS PRODUCT IN BARCOO XML FILE AND USES THE LOWER TO CREATE OWL ATTRIBUTES
	protected void dataPropertiesCreation(MyNode n)
	{
		MyNode child;
		while(n.hasNext())
		{
			child = n.getNext();
			
			//CREATE AN OWL DATA PROPERTY
			OWLDataProperty prop = factory.getOWLDataProperty(IRI.create(ontologyIRI + prologify(child.getValue())));
			
			//CREATE THE STRUCTURE OF THE DATA PROPERTY
			create_data_properties(child, prop);
			
			//ADD TO THE PROPERTY LIST
			data_properties.put(child.getValue(), prop);
		}
	}
	
	protected void create_data_properties(MyNode myNode,OWLDataProperty parent_prop)
	{
		MyNode child;
		OWLDataPropertyRangeAxiom rangeAxiom;
		OWLDatatype stringDatatype  = factory.getOWLDatatype(OWL2Datatype.XSD_STRING.getIRI());
		
		while(myNode.hasNext())
		{
			child = myNode.getNext();
			if(child.isTextNode())
			{				 
				rangeAxiom = factory.getOWLDataPropertyRangeAxiom(parent_prop, stringDatatype);
				manager.addAxiom(ontology, rangeAxiom);				
			}
			else
			{
				OWLDataProperty child_prop = factory.getOWLDataProperty(IRI.create(ontologyIRI + prologify(child.getValue())));
				OWLSubDataPropertyOfAxiom  subAxiom = factory.getOWLSubDataPropertyOfAxiom(child_prop,parent_prop);
				manager.addAxiom(ontology, subAxiom);
				//RECURSIVE CALL
				create_data_properties(child,child_prop);
				//ADD TO THE PROPERTY LIST
				data_properties.put(child.getValue(), child_prop);
			}
		}
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
	
	protected void add_property_restrictions(OWLClass myClass, MyNode node)
	{
		MyNode child;		
		while(node.hasNext())
		{
			child = node.getNext();			
			if(!child.isTextNode())
			{
				Set<OWLDataPropertyExpression>list = this.data_properties.get(child.getValue()).getSubProperties(ontology);
				Iterator<OWLDataPropertyExpression> iterator = list.iterator();
				while(iterator.hasNext())
				{
					 OWLDataProperty o = (OWLDataProperty) iterator.next();
					 
					 
				}
			}
			else
			{
				
			}
		}
	}
	
	//TO BE IMPLEMENTED
	public String categoryCleaning(String parentRaw)
	{
		//System.out.println("Category = "+parentRaw);
		return parentRaw;
	}
	
	public String getParent()
	{
		MyNode node = treeNode.getNext();
		MyNode child = null;
		String parentRaw = null;
		treeNode.resetCursor();
		while(node.hasNext())
		{
			child = node.getNext();
			if(child.getValue().equalsIgnoreCase("category_key"))
			{
				parentRaw = child.getNext().getValue();
				break;
			}
		}
		//Clean String
		parentRaw = categoryCleaning(parentRaw);
		
		return classMapping.get(parentRaw);
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
	
	public boolean Process(File file) throws Exception
	{		
		//PARSE XML FILE
		Vector<AttrValue> list = parseXML(file);
		if(list == null)
		{
			
			return false;
		}		
		//GET PARENT CLASS
		String parent = getParent();
		if(parent == null || parent.isEmpty())
			parent = "BarcooObject";
		
		//CREATE OWL CLASS
		OWLClass myClass = createClass(list.elementAt(0).value, parent);
		
		//CREATE PROPERTIES IN CASE THEY DON'T EXIST
		//create_data_properties(list);
		treeNode.resetCursor();
		dataPropertiesCreation(treeNode.getNext());
		
		//ADD DATA PROPERTIES TO THE CLASS
		add_property_restrictions(myClass,list);
		treeNode.resetCursor();
		//add_property_restrictions(null, treeNode.getNext());
		
		//Save Ontology		
		manager.saveOntology(ontology);	
		
		return true;
		
	}
	//Retrieves XML page
	public static String getPage(URL url) throws Exception
	{
		String line = null;
		String file = "";
		HttpURLConnection connection = (HttpURLConnection) url.openConnection();
		connection.setRequestProperty("User-Agent", "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:12.0) Gecko/20100101 Firefox/12.0");
		connection.connect();
		
		BufferedReader br = new BufferedReader(new InputStreamReader(connection.getInputStream()));
		if(connection.getInputStream() == null)
			return null;
		
		while((line = br.readLine()) != null)
			file += line;
		
		return file;
	}
	
	public boolean classExist(String barcode)
	{
		
		OWLClass myClass = factory.getOWLClass(IRI.create(ontologyIRI + barcode));
		NodeSet<OWLClass> set = reasoner.getSuperClasses(myClass, true);
		
		if(set == null || set.isEmpty())
			return false;
		else
			return true;
	}
	
	public int getNextIndividual(String barcode, OWLClass myClass)
	{		
		NodeSet<OWLNamedIndividual> set = reasoner.getInstances(myClass, true);
		
		
		
		if(set == null || set.isEmpty())
		{
			System.out.println("Hey the set is empty");
			return 1;
		}
		OWLNamedIndividual individual = null;		
		int count = 0;
		
		while(true)
		{
			count++;
			individual = factory.getOWLNamedIndividual(IRI.create(ontologyIRI + barcode + "_" + count));			
			if(!set.containsEntity(individual))
				return count;				
		}
	}
	
	public OWLClass getClass(String barcode)
	{
		return factory.getOWLClass(IRI.create(ontologyIRI + barcode));
	}
	
	public OWLNamedIndividual createIndividual(String name,OWLClass myClass) throws OWLOntologyStorageException
	{
		OWLNamedIndividual individual = factory.getOWLNamedIndividual(IRI.create(ontologyIRI + name));
		OWLClassAssertionAxiom classAssertion = factory.getOWLClassAssertionAxiom(myClass, individual);
		manager.addAxiom(ontology, classAssertion);
		manager.saveOntology(ontology);
		return individual;
	}
	
	public String createNewIndividual(String barcode)
	{
		File barcooFile = null;
		BufferedWriter bw = null;
		reasoner.flush();
		try
		{
			barcooFile = new File("../data/" + barcode + ".xml");
			bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(barcooFile)));
			barcooFile.createNewFile();
			String link = "http://www.barcoo.com/api/get_product_complete?pi="+barcode+"&amp;pins=ean&amp;format=xml&amp;source=ias-tum";
			System.out.println(link);
			URL url = new URL(link);
			String content = getPage(url);
			if(content !=null && !content.isEmpty())
			{
				bw.write(content);
				bw.flush();
				Vector<AttrValue> list = parseXML(barcooFile);
				if(list !=null && !list.isEmpty())
				{
					Process(barcooFile);
					System.out.println("File processed.");
					
					OWLClass myClass = getClass(barcode);
					
					//create individual at this point
					int number = getNextIndividual(barcode,myClass);
					//The following function creates the ontology, creates the axiom and save the changes					
					createIndividual(barcode + "_" + number, myClass);
					System.out.println("Individual name: "+barcode + "_" + number);
					return (barcode + "_" + number);
				}
			}
			bw.close();
			barcooFile.delete();
			
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
		return null;
	}
	
	public void createNotExistingClass(String barcode)
	{		
		File barcooFile = null;
		BufferedWriter bw = null;
		boolean processed = true;
		reasoner.flush();		
		try
		{
			if(!classExist(barcode))
			{				
				System.out.println("Class does not exist");
				barcooFile = new File("../data/" + barcode + ".xml");
				bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(barcooFile)));
				barcooFile.createNewFile();
				String link = "http://www.barcoo.com/api/get_product_complete?pi="+barcode+"&amp;pins=ean&amp;format=xml&amp;source=ias-tum";
				System.out.println(link);
				URL url = new URL(link);
				String content = getPage(url);
				if(content != null && !content.isEmpty())
				{
					bw.write(content);
					bw.flush();
					processed = Process(barcooFile);
					System.out.println("File processed: "+ processed);
				}
				bw.close();
				barcooFile.delete();
			}			
			if(!processed)
				return;
			
			OWLClass myClass = getClass(barcode);
			
			//create individual at this point
			int number = getNextIndividual(barcode,myClass);
			//The following function creates the ontology, creates the axiom and save the changes
			//System.out.println("Individual name: "+barcode + "_" + number);
			createIndividual(barcode + "_" + number, myClass);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	public static void main(String[] args) //5413987056260
	{		
		if(args == null || args[0] == null || args[1] == null || args[2] == null)
		{
			System.out.println("First argument should be the PATH of BARCOO.OWL.IN");
			System.out.println("Second argument should be the PATH of barcodes list file");
			System.out.println("Third argument should be the PATH of barcoo mapping file");
			System.exit(1);
		}
		
		
		BarcooImporter b = new BarcooImporter(args[0],args[2]);
		
		BufferedReader br = null;
		try
		{
			File barcooFile = null;
			br = new BufferedReader(new InputStreamReader(new FileInputStream(args[1])));
			BufferedWriter bw = null;
			
			String barcode = null;
			URL url;
			boolean done = false;
			while((barcode = br.readLine()) != null)
			{
				try
				{
					if(barcode.equalsIgnoreCase("5400269227876") || done)
					{
						done = true;
						barcooFile = new File( barcode + ".xml");
						bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(barcooFile)));
						barcooFile.createNewFile();
						String link = "http://www.barcoo.com/api/get_product_complete?pi="+barcode+"&amp;pins=ean&amp;format=xml&amp;source=ias-tum";
						System.out.println(link);
						url = new URL(link);
						String content = getPage(url);
						if(content !=null && !content.isEmpty())
						{
							bw.write(content);
							bw.flush();
							b.Process(barcooFile);
							bw.close();
							barcooFile.delete();
						}
					}
				}
				catch(Exception ex)
				{
					ex.printStackTrace();
				}
			}
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
		finally
		{
			try 
			{
				br.close();
			} 
			catch (IOException e) 
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
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
