package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.communication.*;
import ros.roscpp.*;



public class BarcooSubscriber{
	
	private String topic = "/barcode_reader_node/barcode";
	private Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;
	private BarcooImporter barcooImporter;
	
	public BarcooSubscriber(final String barcooOWLPath,final String mappingPath) throws Exception
	{		
		barcooImporter = new BarcooImporter(barcooOWLPath, mappingPath);
		final Ros ros = Ros.getInstance();		
		ros.init("Barcoosubscriber");		
		ros.pkg.std_msgs.msg.String msg = new ros.pkg.std_msgs.msg.String();
		NodeHandle n = ros.createNodeHandle();
		Subscriber.Callback<ros.pkg.std_msgs.msg.String> callback = new Subscriber.Callback<ros.pkg.std_msgs.msg.String>() 
		{
			public void call(ros.pkg.std_msgs.msg.String msg) 
			{
				ros.logInfo("Received [" + msg.data + "]");
				barcooImporter.createNotExistingClass(msg.data);
			}
		};
		
		Subscriber<ros.pkg.std_msgs.msg.String> sub; 
        sub = n.subscribe(topic,new ros.pkg.std_msgs.msg.String(), callback, 100);
		System.out.println("Subscribed to " + topic);
		
		n.spin();		
		sub.shutdown();
	}	
	
	public static void main(String[] args)
	{
		System.out.println("Hey nacer");
		try
		{
			System.out.println("arg0 = barcoo owl path");
			System.out.println("arg1 = barcoo mapping path");
			
			BarcooSubscriber subscriber = new BarcooSubscriber(args[0],args[1]);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	
}
