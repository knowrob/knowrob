package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.communication.*;
import ros.roscpp.*;
import ros.pkg.shopping_demo.srv.*;
import org.semanticweb.HermiT.Reasoner;

public class BarcooLocationInferer {
	
	private final Ros ros;
	private NodeHandle n;
	ServiceClient<QueryBestObjLocation.Request,QueryBestObjLocation.Response,QueryBestObjLocation> client;
	
	
	public BarcooLocationInferer()
	{
		//Initialisation of the ros object and nodeHandle
		ros = Ros.getInstance();		
		ros.init("Barcoo_location_inferer");				
		n = ros.createNodeHandle();
		
		//Creation of the service client object
		client = n.serviceClient("/best_object_location/query" , new QueryBestObjLocation(), false);		
	}
	
	public void inferLocation(String barcode)
	{
		//first, look for object name using the reasoner
		
		//Second, begin the service client object

		QueryBestObjLocation.Request rq = new QueryBestObjLocation.Request();
		
		rq.name_space  = "comp_barcoo";  
		rq.object_type =  "BEER";
		
		try 
		{			
			QueryBestObjLocation.Response resp = client.call(rq);			 
		}
		catch (RosException e) 
		{
			ros.logError("Failed to call service"); 
			System.exit(1); 
		}
		
		//If don't call n.shutdown, client thread will hang until ctrl+c
		n.shutdown();
		
		
	}
		
	public static void main(String[] args)
	{		
		try
		{
			BarcooLocationInferer inferer = new BarcooLocationInferer();
			inferer.inferLocation("");
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}

}
