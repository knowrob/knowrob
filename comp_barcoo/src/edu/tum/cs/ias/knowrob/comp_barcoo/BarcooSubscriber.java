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
	
	public BarcooSubscriber(final String barcooOWLPath,final String mappingPath) throws Exception
	{		
		
		final Ros ros = Ros.getInstance();		
		ros.init("BarcooService");				
		NodeHandle n = ros.createNodeHandle();
		barcooImporter = new BarcooImporter(barcooOWLPath, mappingPath);
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
			//System.out.println("arg0 = barcoo owl path");
			//System.out.println("arg1 = barcoo mapping path");
			System.out.println("PLEASE WAIT.\n PROGRAM LOADING...");
			
			BarcooSubscriber subscriber = new BarcooSubscriber(args[0],args[1]);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	
}
