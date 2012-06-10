package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.communication.*;
import ros.roscpp.*;


public class BarcooSubscriber{
	
	private String topic = "/barcode_reader_node/barcode";
	private Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;
	
	public BarcooSubscriber(String barcooOWLPath, String mappingPath) throws Exception
	{		
		Ros ros = Ros.getInstance();
		ros.init("Barcoosubscriber");		
		NodeHandle n = ros.createNodeHandle();
		callback = new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>();
		Subscriber<ros.pkg.std_msgs.msg.String> sub = n.subscribe(topic, new ros.pkg.std_msgs.msg.String(), callback, 10);
		 
         System.out.println("Subscribed to " + topic);
		 n.spinOnce();
         while(true)
         {
	         while (!callback.isEmpty()) 
	         {
	        	 System.out.println(callback.pop().data);
	        	 //write your code here
	        	 createIndividuals(callback.pop().data, barcooOWLPath,mappingPath);
	         }
         }
         //sub.shutdown();
	}
	
	public void createIndividuals(String str, String path,String mappingPath)
	{
		BarcooImporter.createNotExistingClass(str, path, mappingPath);
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
