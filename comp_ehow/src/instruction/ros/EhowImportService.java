package instruction.ros;

import ros.*;
import ros.pkg.comp_ehow.srv.EhowToOWL;
import instruction.exporter.owl.OWLExporter;


public class EhowImportService {

	Ros ros;
	NodeHandle n;
	OWLExporter owl_ex;
	
	public EhowImportService() {
		
  		try {

  			// wait for Cyc to be available
  			// MT: does not work at the moment; there are lots of exceptions if a CycConnection
  			// has been tried to open while Cyc was not available and then successfully opened
  			//
//  			System.out.print("Waiting for OpenCyc..");
//  			int i=0;
//  			while(true) {
//	  			try {
//	  				if((i++)%100==0)
//	  					System.out.print(".");
//	  				CycConnection cyc = new CycConnection();
//	  				System.out.println("\nCyc found: "+cyc.connectionInfo());
//	  				cyc.close();
//					break;
//					
//				} catch (UnknownHostException e) { }
//				catch (CycApiException e) { }
//				catch (IOException e) { }
//  			}
  			
  			owl_ex = new OWLExporter();
  			
  			ros = Ros.getInstance();
  			ros.init("ehow_importer");
  			n = ros.createNodeHandle();
  			
			n.advertiseService("/ehow_importer/ehow_to_owl",  new EhowToOWL(),  new EhowToOWLCallback());
			ros.logInfo("Started ehow import service.");
			
			while(n.isValid())
				ros.spinOnce();
	    		
  		} catch(RosException e) {
  			e.printStackTrace();
  		}
	}
	
	/**
	 * 
	 * The callback class for updating the left image in the visualization
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class EhowToOWLCallback implements ServiceServer.Callback<EhowToOWL.Request, EhowToOWL.Response> {
		
		@Override
		public EhowToOWL.Response call(EhowToOWL.Request req) {

			EhowToOWL.Response res = new EhowToOWL.Response();
			res.owl_instructions="";

			if (req.command != null && req.command.length() > 0) {
				res.owl_instructions = owl_ex.convertHowtoToOWLOntology(req.command);
			}

			return res;
		}
	}

	public static void main(String args[]) {
		new EhowImportService();
	}
}
