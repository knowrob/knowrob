package org.knowrob.utils.ros;

import com.mongodb.BasicDBObject;
import org.knowrob.interfaces.mongo.MongoMessages;
import org.knowrob.utils.ros.ImageEncoding;

public class MongoImageEncoding {
	public static String encodeBase64(BasicDBObject mngObj) {
		try {
			return ImageEncoding.encodeBase64(MongoMessages.get().create(mngObj, sensor_msgs.Image.class, "sensor_msgs/Image"));
		}
		catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}
	
	public static String encodeBase64_compressed(BasicDBObject mngObj) {
		try {
			return ImageEncoding.encodeBase64(MongoMessages.get().create(mngObj, sensor_msgs.CompressedImage.class, "sensor_msgs/CompressedImage"));
		}
		catch(Exception e) {
			e.printStackTrace();
			return null;
		}
	}
}
