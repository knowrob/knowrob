package org.knowrob.interfaces.mongo;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.ByteOrder;
import java.util.GregorianCalendar;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import com.mongodb.BasicDBObject;

public class MongoRosMessage<RosType> {
	Publisher<RosType> pub = null;
	// e.g. "std_msgs/String"
	final String typeName;
	final String topic;
	
	public MongoRosMessage(final String typeName, final String topic) {
		this.typeName = typeName;
		this.topic = topic;
	}

	public MongoRosMessage(ConnectedNode node, String typeName, String topic) {
		this.typeName = typeName;
		this.topic = topic;
		connect(node);
	}

	public void connect(final ConnectedNode node) {
		this.pub = node.newPublisher(topic, typeName);
	}

	public RosType createMessgae(BasicDBObject mngObj) {
		try {
			final RosType msg = pub.newMessage();
			createMessage(msg, mngObj);
			return msg;
		}
		catch (Exception e) {
			System.err.println("Failed to create message: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}

	public boolean publish(BasicDBObject mngObj) {
		if(pub==null) {
			System.err.println("Not connected.");
			return false;
		}
		try {
			final RosType msg = pub.newMessage();
			createMessage(msg, mngObj);
			pub.publish(msg);
	
			return true;
		}
		catch (Exception e) {
			System.err.println("Failed to publish message: " + e.getMessage());
			e.printStackTrace();
			return false;
		}
	}

	protected void createMessage(Object msg, BasicDBObject mngObj) throws IOException {
		for(Method m : msg.getClass().getMethods()) {
			String name = m.getName();
			if(!name.startsWith("set")) continue;
			name = name.substring(3);
			
			String[] nameParts = name.split("(?=\\p{Upper})");
			StringBuilder fieldName_ = new StringBuilder();
			for(String x : nameParts) {
				if(fieldName_.length()>0) fieldName_.append('_');
				fieldName_.append(x.toLowerCase());
			}
			final String fieldName = fieldName_.toString();

			final Object value = mngObj.get(fieldName);
			if(value==null) {
				System.err.println("Mongo entry missing " + fieldName + " field.");
				continue;
			}
			
			final Class<?> paramType = m.getParameterTypes()[0];
			if(value instanceof BasicDBObject) {
				try {
					Method getter = msg.getClass().getMethod("get"+name);
					createMessage(getter.invoke(msg), (BasicDBObject)value);
				} 
				catch (Exception e) {
					// TODO Auto-generated catch block
					System.err.println("Failed to get message field '" + fieldName + "'" +
							". Error: " + e.getMessage());
					e.printStackTrace();
				}
			}
			// Convert from Date to ROS Time
			else if(paramType == Time.class && value instanceof java.util.Date) {
				GregorianCalendar c = new GregorianCalendar();
				c.setTime((java.util.Date)value);
				long ms = c.getTimeInMillis();
				
				Time t  = new Time();
				t.secs  = (int) (ms / 1000);
				t.nsecs = (int) (ms % 1000) * 1000000;
				setMessageValue(m, fieldName, msg, t);
			}
			// Convert from number to byte
			else if(paramType == byte.class && value instanceof Number) {
				setMessageValue(m, fieldName, msg, ((Number)value).byteValue());
			}
			// Convert byte array to ChannelBuffer
			else if(paramType == ChannelBuffer.class) {
				setMessageValue(m, fieldName, msg, ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN, (byte[])value));
			}
			// Use value as is
			else {
				setMessageValue(m, fieldName, msg, value);
			}
		}
	}

	private void setMessageValue(Method m, String fieldName, Object msg, Object value) {
		try {
			m.invoke(msg, value);
		}
		catch (Exception e) {
			// TODO Auto-generated catch block
			System.err.println("Failed to set message field '" + fieldName + "'" +
					". Value type: " + value.getClass().getName() +
					". Method argument type: " + m.getParameterTypes()[0].getName() +
					". Error: " + e.getMessage());
			e.printStackTrace();
		}
	}
}
