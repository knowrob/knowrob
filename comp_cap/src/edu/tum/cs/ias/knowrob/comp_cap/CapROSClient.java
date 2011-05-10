package edu.tum.cs.ias.knowrob.comp_cap;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;

import ros.NodeHandle;
import ros.Ros;
import ros.Topic;

public class CapROSClient {
	boolean isInitialized = false;

	Ros ros;
	NodeHandle node;
	
	//Native method declaration
	private static native Object getHeader(String dest_addr, String dest_port);
	//Load the library
//	static{
//		System.loadLibrary("servicecpp");
//	}
	
	public CapROSClient(String node_name) {
		ros = Ros.getInstance();

		if (!Ros.getInstance().isInitialized()) {
			ros.init(node_name);
		}
		node = ros.createNodeHandle();
	}

	/*
	 * returns a Object containing published, subscribed Topics and published
	 * Services with information about which nodes is subscribed to, or is
	 * publishing the Topic and Service
	 */
	public Object[] getSystemState() {
		String ros_master_uri = "http://" + node.getMasterHost() + ":"
				+ node.getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { new String(node.getName()) };
		try {
			return (Object[]) rosmaster.execute("getSystemState", params);
		} catch (XmlRpcException e) {
		}

		return null;
	}

	/*
	 * returns a Object containing the published Topics
	 */
	public Object[] getXMLRPCPublishedTopics() {
		String ros_master_uri = "http://" + node.getMasterHost() + ":"
				+ node.getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { new String(node.getName()),
				new String("") };
		try {
			return (Object[]) rosmaster.execute("getPublishedTopics", params);
		} catch (XmlRpcException e) {
		}

		return null;
	}

	/*
	 * returns a Object containing all Topics with msgsType
	 */
	public Object[] getXMLRPCTopicTypes() {
		String ros_master_uri = "http://" + node.getMasterHost() + ":"
				+ node.getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { new String(node.getName()) };
		try {
			return (Object[]) rosmaster.execute("getTopicTypes", params);
		} catch (XmlRpcException e) {
		}

		return null;
	}

	/*
	 * return an Object containing the service_uri
	 */
	public Object[] getXMLRPClookupService(String service) {
		String ros_master_uri = "http://" + node.getMasterHost() + ":"
				+ node.getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { new String(node.getName()), service };
		try {
			return (Object[]) rosmaster.execute("lookupService", params);
		} catch (XmlRpcException e) {
		}

		return null;
	}

	/*
	 * returns the service msgsType
	 */
	public String getServiceType(String service) {
		String srvsType = "Still unknown";
		Object[] lookup = getXMLRPClookupService(service);
		Integer code = (Integer) lookup[0];
		String service_uri = (String) lookup[2];
		if (code.compareTo(new Integer(1)) == 0) {
			String dest_addr = service_uri.substring(0, service_uri
					.lastIndexOf(":"));
			String dest_port = service_uri.substring(service_uri
					.lastIndexOf(":") + 1);
			Object serviceHeader = getHeader(dest_addr, dest_port);
		}
		
		
		/**
		 * probe the service for it's header
		 * needs a TCPROS implementation for java
		 */

		return srvsType;
	}

	
	/*
	 * returns published topics with messageType
	 */
	public Collection<Topic> getPublishedTopics() {
		ArrayList<Topic> pubTopics = new ArrayList<Topic>();
		Object[] xmlrpcPublished = this.getXMLRPCPublishedTopics();
		xmlrpcPublished = (Object[]) xmlrpcPublished[2];

		for (Object input : xmlrpcPublished) {
			Object[] output = (Object[]) input;
			String topicName = (String) output[0];
			String msgsType = (String) output[1];
			pubTopics.add(new Topic(topicName, msgsType, null));
		}

		return pubTopics;
	}

	/*
	 * return subscribed Topics with messageType
	 */
	public Collection<Topic> getSubscribedTopics() {
		ArrayList<Topic> subTopics = new ArrayList<Topic>();
		Object[] systemState = this.getSystemState();
		systemState = (Object[]) systemState[2];
		Object[] subscribed = (Object[]) systemState[1];
		Object[] topics = getXMLRPCTopicTypes();
		topics = (Object[]) topics[2];

		for (Object input : subscribed) {
			Object[] output = (Object[]) input;
			String topicName = (String) output[0];
			String msgsType = "Still unknown";
			for (Object o : topics) {
				Object[] type = (Object[]) o;
				if (topicName.equals((String) type[0])) {
					msgsType = (String) type[1];
				}
			}
			subTopics.add(new Topic(topicName, msgsType, null));
		}

		return subTopics;
	}

	/*
	 * returns published services with serviceType
	 */
	public Collection<Service> getService() {
		ArrayList<Service> service = new ArrayList<Service>();
		Object[] systemState = this.getSystemState();
		systemState = (Object[]) systemState[2];
		Object[] srvs = (Object[]) systemState[2];

		for (Object input : srvs) {
			Object[] output = (Object[]) input;
			String serviceName = (String) output[0];
			String serviceType = getServiceType(serviceName);
			output = (Object[]) output[1];
			String[] provider = new String[output.length];
			for (int i = 0; i < output.length; i++) {
				provider[i] = (String) output[i];
			}
			service.add(new Service(serviceName, serviceType, null, provider));
		}

		return service;
	}

	public void destroy() {
		node.shutdown();
	}
	
}
