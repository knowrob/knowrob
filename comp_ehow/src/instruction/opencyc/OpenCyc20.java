package instruction.opencyc;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Iterator;

import org.opencyc.api.CycApiException;
import org.opencyc.api.CycObjectFactory;
import org.opencyc.cycobject.CycList;
import org.opencyc.cycobject.CycObject;
import org.opencyc.cycobject.CycVariable;

public class OpenCyc20 extends OpenCyc {
	
	protected CycObject defaultMt = null;
	
	protected OpenCyc20() throws UnknownHostException, CycApiException,
			IOException {
		super();
		defaultMt = cyc.getConstantByName( "#$CurrentWorldDataCollectorMt-NonHomocentric" );
	}
	
	public static OpenCyc getInstance() throws UnknownHostException, CycApiException, IOException {

		if ( me == null ) {
			me = new OpenCyc20();
			me.initializeCyc();
		}
		return me;
	}
	
	@SuppressWarnings("deprecation")
	public ArrayList<String> getCycConceptFromWordNetID(String synsetURI)
			throws Exception {
		// Ask Cyc for the concepts
		CycList query = cyc.makeCycList( "(#$ist (#$ContextOfPCWFn (#$OWLOntologyFn \"http://www.w3.org/2006/03/wn/wn20/instances\")) "
				+ "(#$seeAlsoURI ?CONCEPT \"" + synsetURI + "\"))" );
		
		CycVariable var = CycObjectFactory.makeCycVariable( "?CONCEPT" );
		
		CycObject mt = defaultMt;

		CycList ret = cyc.askWithVariable( query, var, mt );

		ArrayList<String> list = new ArrayList<String>();
		for ( Iterator i = ret.iterator(); i.hasNext(); ) {
			String str = i.next().toString();
			if ( str.indexOf( "(" ) == - 1 && str.indexOf( ")" ) == - 1 && str.indexOf( " " ) == - 1 )
				list.add( str );
		}
		return list;
	}
	
	@Override
	public void addMapping(String synset, String concept) throws UnknownHostException, CycApiException, IOException {
		CycList assertion = cyc.makeCycList( "(#$ist (#$ContextOfPCWFn (#$OWLOntologyFn \"http://www.w3.org/2006/03/wn/wn20/instances\")) " +
				"(#$seeAlsoURI #$" + concept + " \"" + synset + "\"))");
		
		cyc.assertGaf( assertion, defaultMt );
	}
	
	public static void main(String[] args) {
		OpenCyc cyc;
		try {
			cyc = OpenCyc20.getInstance();
			
			cyc.addMapping("http://www.w3.org/2006/03/wn/wn20/instances/synset-own-verb-3", "possesses");
			
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (CycApiException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
