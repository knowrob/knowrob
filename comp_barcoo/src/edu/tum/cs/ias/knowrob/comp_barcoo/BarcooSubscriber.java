package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.communication.*;


public class BarcooSubscriber{
	
	private String topic = "topic";
	private Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;
	
	public BarcooSubscriber(String barcooOWLPath, String mappingPath) throws Exception
	{
		Ros ros = Ros.getInstance();
		ros.init("Barcoo subscriber");		
		NodeHandle n = ros.createNodeHandle();
		 callback = new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>();
		 Subscriber<ros.pkg.std_msgs.msg.String> sub = n.subscribe(topic, new ros.pkg.std_msgs.msg.String(), callback, 10);
         
		 n.spinOnce();
         
         while (!callback.isEmpty()) 
         {
        	 System.out.println(callback.pop().data);
        	 //write your code here
        	 createIndividuals(callback.pop().data, barcooOWLPath,mappingPath);
         }
         
         sub.shutdown();
	}
	
	public void createIndividuals(String str, String path,String mappingPath)
	{
		BarcooImporter.createNotExistingClass(str, path, mappingPath);
	}
	
	public static void main(String[] args)
	{
		try
		{
			BarcooSubscriber subscriber = new BarcooSubscriber(args[0],args[1]);
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}
	
}
