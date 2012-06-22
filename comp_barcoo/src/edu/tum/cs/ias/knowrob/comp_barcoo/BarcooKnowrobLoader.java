package edu.tum.cs.ias.knowrob.comp_barcoo;

import java.util.HashMap;

import ros.*;
import ros.pkg.vision_msgs.msg.aposteriori_position;
import ros.pkg.vision_msgs.msg.cop_answer;
import ros.pkg.vision_msgs.msg.cop_descriptor;
import ros.pkg.vision_srvs.srv.cop_call;
import ros.pkg.vision_srvs.srv.srvjlo;
import ros.pkg.vision_srvs.srv.srvjlo.Response;
import ros.pkg.vision_srvs.srv.srvjlo.Request;
import ros.pkg.vision_msgs.msg.partial_lo;

public class BarcooKnowrobLoader {
	
	static String topic;
	private static HashMap<String, Integer> map;
	private static int objectCount = 0;
	Publisher<ros.pkg.vision_msgs.msg.cop_answer> pub;
	
	public BarcooKnowrobLoader(NodeHandle n) throws Exception
	{
		ros.pkg.vision_msgs.msg.cop_answer msgType = new ros.pkg.vision_msgs.msg.cop_answer();
		map = new HashMap<String, Integer>();				
		pub = n.advertise(topic, msgType, 1);				
	}
	
	public String passToKnowrob(String barcode)
	{
		
		//first find appropriate individual id
		Integer id = map.get(barcode);
		if(id == null || id == 0)
		{
			id = 1;
			map.put(barcode, id);
		}
		else
		{
			id++;
			map.remove(barcode);
			map.put(barcode, id);
		}
		
		//create individual by sending cop_answer to rosprolog
		objectCount++;
		
		//	Constructing the cop_answer object
		cop_answer msg = new cop_answer();
		
		msg.perception_primitive = 0;
		msg.error = "";
		aposteriori_position position = new aposteriori_position();
		
		position.objectId = 0;
		position.probability = 1;
		position.position = 0;
		
		cop_descriptor descriptor = new cop_descriptor();
		
		descriptor.object_id = objectCount;
		descriptor.sem_class = "http://www.barcoo.com/barcoo.owl#" + barcode;
		descriptor.type = "BarcooFinder";
		descriptor.quality = 0;
		
		position.models.add(descriptor);
		
		msg.found_poses.add(position);
		pub.publish(msg);
		
		return barcode + "_k"+ id;
	}	
}
