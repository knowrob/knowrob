package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.communication.*;
import ros.roscpp.*;
import ros.pkg.comp_barcoo.srv.*;


public class BarcooSubscriber{
	
	//private String topic = "/barcode_image_node/barcode";
	//private Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;
	private BarcooImporter barcooImporter;
	private BarcooKnowrobLoader knowrobLoader;
	
	public BarcooSubscriber(final String barcooOWLPath) throws Exception
	{		
		
		final Ros ros = Ros.getInstance();		
		ros.init("BarcooService");				
		NodeHandle n = ros.createNodeHandle();
		barcooImporter = new BarcooImporter(barcooOWLPath);
		knowrobLoader = new BarcooKnowrobLoader(n,"knowrobTopic");
		
		ServiceServer.Callback<send_barcode.Request,send_barcode.Response> scb = new ServiceServer.Callback<send_barcode.Request,send_barcode.Response>() 
		{
            public send_barcode.Response call(send_barcode.Request request) {
            	send_barcode.Response res = new send_barcode.Response();
            	if(request.knowrob == 1)
            	{
            		String indName = knowrobLoader.passToKnowrob(request.barcode.data);
            		res.recieved.data = indName;                 
            		return res;
            	}
            	
            	else if(request.knowrob == 0)
            	{
            		String indName = barcooImporter.createNewIndividual(request.barcode.data);
            		res.recieved.data = indName;                 
            		return res;
            	}
            	else
            	{
            		res.recieved.data = "Wrong knowrob attribute value";                 
            		return res;
            	}
            }
		};            
		
        ServiceServer<send_barcode.Request,send_barcode.Response,send_barcode> srv = n.advertiseService("send_barcode", new send_barcode(), scb);
        
        ros.logInfo("Ready to receive barcode and create new individuals"); 
        System.out.println("LOADING COMPLETED.");
		n.spin();		
	}	
	
	public static void main(String[] args)
	{		
		try
		{
			if(args == null || args.length != 1 || args[0].equalsIgnoreCase("help"))
			{
				System.out.println("Usage: barcoo_subscriber <barcoo_ontology_file>.owl");				
				System.exit(0);
			}			
			System.out.println("PLEASE WAIT.\n PROGRAM LOADING...");
			
			BarcooSubscriber subscriber = new BarcooSubscriber(args[0]);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	
}
