package rosjava;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;

public class Master {

	private String masterHost;
	private int masterPort;

	public String getMasterHost() {
		return masterHost;
	}

	public void setMasterHost(String masterHost) {
		this.masterHost = masterHost;
	}

	public int getMasterPort() {
		return masterPort;
	}

	public void setMasterPort(int masterPort) {
		this.masterPort = masterPort;
	}

	public Master(String masterHost, int masterPort) {
		this.setMasterHost(masterHost);
		this.setMasterPort(masterPort);
	}

	public Object[] executeXMLRPCCommand(String command, Object[] params) {
		String ros_master_uri = "http://" + getMasterHost() + ":"
				+ getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);
		try {
			return (Object[]) rosmaster.execute(command, params);
		} catch (XmlRpcException e) {
			System.out
					.println("Error: An Error occured  during rosjava.Master.executeXMLRPCCommand("
							+ command + ", params)");
			System.out.println(e.getMessage());
		}

		return null;
	}

	/**
	 * Register the caller as a provider of the specified service.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param service
	 *            Fully-qualified name of service
	 * @param service_api
	 *            ROSRPC Service URI
	 * @param caller_api
	 *            XML-RPC URI of caller node
	 * @return code
	 */
	public Integer registerService(String caller_id, String service,
			String service_api, String caller_api) {
		Object[] params = new Object[] { caller_id, service, service_api,
				caller_api };
		Object[] result = this.executeXMLRPCCommand("registerService", params);
		Integer code = (Integer) result[0];
		return code;
	}

	/**
	 * Unregister the caller as a provider of the specified service.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param service
	 *            Fully-qualified name of service
	 * @param service_api
	 *            ROSRPC Service URI
	 * @return (code, statusMessage, numUnregistered) Number of unregistrations
	 *         (either 0 or 1). If this is zero it means that the caller was not
	 *         registered as a service provider. The call still succeeds as the
	 *         intended final state is reached.
	 */
	public Object unregisterService(String caller_id, String service,
			String service_api) {
		Object[] params = new Object[] { caller_id, service, service_api };
		Object[] result = this
				.executeXMLRPCCommand("unregisterService", params);
		return result;
	}

	/**
	 * Subscribe the caller to the specified topic. In addition to receiving a
	 * list of current publishers, the subscriber will also receive
	 * notifications of new publishers via the publisherUpdate API.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param topic
	 *            Fully-qualified name of topic.
	 * @param topic_type
	 *            Datatype for topic. Must be a package-resource name, i.e. the
	 *            .msg name.
	 * @param caller_api
	 *            API URI of subscriber to register. Will be used for new
	 *            publisher notifications.
	 * @return (code, statusMessage, publishers) Publishers is a list of XMLRPC
	 *         API URIs for nodes currently publishing the specified topic.
	 */
	public Object[] registerSubscriber(String caller_id, String topic,
			String topic_type, String caller_api) {
		Object[] params = new Object[] { caller_id, topic, topic_type,
				caller_api };
		Object[] result = this.executeXMLRPCCommand("registerSubscriber",
				params);
		return result;
	}

	/**
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param topic
	 *            Fully-qualified name of topic.
	 * @param caller_api
	 *            API URI of subscriber to register. Will be used for new
	 *            publisher notifications.
	 * @return (code, statusMessage, numUnsubscribed) If numUnsubscribed is zero
	 *         it means that the caller was not registered as a subscriber. The
	 *         call still succeeds as the intended final state is reached.
	 */
	public Object[] unregisterSubscriber(String caller_id, String topic,
			String caller_api) {
		Object[] params = new Object[] { caller_id, topic, caller_api };
		Object[] result = this.executeXMLRPCCommand("unregisterSubscriber",
				params);
		return result;
	}

	/**
	 * Register the caller as a publisher the topic.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param topic
	 *            Fully-qualified name of topic.
	 * @param topic_type
	 *            Datatype for topic. Must be a package-resource name, i.e. the
	 *            .msg name.
	 * @param caller_api
	 *            API URI of subscriber to register. Will be used for new
	 *            publisher notifications.
	 * @return (code, statusMessage, subscriberApis) List of current subscribers
	 *         of topic in the form of XMLRPC URIs.
	 */
	public Object[] registerPublisher(String caller_id, String topic,
			String topic_type, String caller_api) {
		Object[] params = new Object[] { caller_id, topic, topic_type,
				caller_api };
		Object[] result = this
				.executeXMLRPCCommand("registerPublisher", params);
		return result;
	}

	/**
	 * Unregister the caller as a publisher of the topic.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param topic
	 *            Fully-qualified name of topic.
	 * @param caller_api
	 *            API URI of subscriber to register. Will be used for new
	 *            publisher notifications.
	 * @return (code, statusMessage, numUnregistered) If numUnregistered is zero
	 *         it means that the caller was not registered as a publisher. The
	 *         call still succeeds as the intended final state is reached.
	 */
	public Object[] unregisterPublisher(String caller_id, String topic,
			String caller_api) {
		Object[] params = new Object[] { caller_id, topic, caller_api };
		Object[] result = this.executeXMLRPCCommand("unregisterPublisher",
				params);
		return result;
	}

	/**
	 * Get the XML-RPC URI of the node with the associated name/caller_id. This
	 * API is for looking information about publishers and subscribers. Use
	 * lookupService instead to lookup ROS-RPC URIs.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param node_name
	 *            Name of node to lookup
	 * @return
	 */
	public String lookupNode(String caller_id, String node_name) {
		Object[] params = new Object[] { caller_id, node_name };
		Object[] result;

		Integer code = new Integer(0);
		do {
			result = executeXMLRPCCommand("lookupNode", params);
			code = (Integer) result[0];
		} while (code.compareTo(new Integer(1)) != 0);

		return (String) result[2];
	}

	/**
	 * Returns an Object that contains Published Topics with Publisher,
	 * Subscribed Topics with Subscribers and Services with service Providers
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @return an Object(String[String[]], String[String[]], String[String[]])
	 */
	public Object[] getSystemState(String caller_id) {
		Object[] params = new Object[] { caller_id };
		Object[] result;

		Integer code = new Integer(0);
		do {
			result = executeXMLRPCCommand("getSystemState", params);
			code = (Integer) result[0];
		} while (code.compareTo(new Integer(1)) != 0);

		return (Object[]) result[2];
	}

	/**
	 * returns an Object containing Topic and msgsType
	 * 
	 * @return an Object[](String, String)
	 */
	public Object[] getTopicTypes() {
		Object[] params = new Object[] { "rosjava.Master" };
		Object[] result;

		Integer code = new Integer(0);
		do {
			result = executeXMLRPCCommand("getTopicTypes", params);
			code = (Integer) result[0];
		} while (code.compareTo(new Integer(1)) != 0);

		return (Object[]) result[2];
	}

	/**
	 * Lookup all provider of a particular service.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @param service
	 *            Fully-qualified name of service
	 * @return serviceUrl
	 */
	public String lookupService(String caller_id, String service) {
		Object[] params = new Object[] { caller_id, service };
		Object[] result;

		Integer code = new Integer(0);
		do {
			result = executeXMLRPCCommand("lookupService", params);
			code = (Integer) result[0];
		} while (code.compareTo(new Integer(1)) != 0);

		return (String) result[2];
	}

	/**
	 * Returns the msgsType to a given Topic
	 * 
	 * @param topicName
	 * @return msgsType
	 */
	public String getTopicType(String topicName) {
		String topicType = "Topic is not existing";
		Object[] Types = getTopicTypes();
		for (Object o : Types) {
			Object[] output = (Object[]) o;
			if (topicName.equals((String) output[0])) {
				return (String) output[1];
			}
		}
		return topicType;
	}

	/**
	 * Returns a String[] with the Publishers for a given Topic
	 * 
	 * @param topicName
	 * @return a String[] with the Publishers for a given Topic, is empty if
	 *         there is no publisher
	 */
	public String[] getPublisher(String topicName) {
		ArrayList<String> publisher = new ArrayList<String>();
		Object[] systemState = this
				.getSystemState("rosjava.Master.getPublisher");
		Object[] publishedTopics = (Object[]) systemState[0];

		for (Object o : publishedTopics) {
			Object[] output = (Object[]) o;
			if (topicName.equals((String) output[0])) {
				output = (Object[]) output[1];
				for (Object ob : output) {
					publisher.add((String) ob);
				}
				break;
			}
		}

		return publisher.toArray(new String[publisher.size()]);
	}

	/**
	 * Returns a String[] with the Subscribers for a given Topic
	 * 
	 * @param topicName
	 * @return a String[] with the Subscribers for a given Topic, is empty if
	 *         there is no subscriber
	 */
	public String[] getSubscriber(String topicName) {
		ArrayList<String> subscriber = new ArrayList<String>();
		Object[] systemState = this
				.getSystemState("rosjava.Master.getSubscriber");
		Object[] subscribedTopics = (Object[]) systemState[1];

		for (Object o : subscribedTopics) {
			Object[] output = (Object[]) o;
			if (topicName.equals((String) output[0])) {
				output = (Object[]) output[1];
				for (Object ob : output) {
					subscriber.add((String) ob);
				}
				break;
			}
		}

		return subscriber.toArray(new String[subscriber.size()]);
	}

	/**
	 * return an string[] that contains all providers of a given service
	 * 
	 * @param service
	 * @return
	 */
	public String[] getServiceProviders(String service) {
		ArrayList<String> provider = new ArrayList<String>();
		Object[] systemState = this
				.getSystemState("rosjava.Master.getServiceProviders");
		Object[] srvs = (Object[]) systemState[2];

		for (Object o : srvs) {
			Object[] output = (Object[]) o;
			if (service.equals((String) output[0])) {
				output = (Object[]) output[1];
				for (Object ob : output) {
					provider.add((String) ob);
				}
				break;
			}
		}

		return provider.toArray(new String[provider.size()]);
	}

	/**
	 * Get the srvsType
	 * 
	 * @param service
	 *            Fully-qualified name of service
	 * @return
	 */
	public String getServiceType(String service) {
		String lookup;
		String host;
		int port;
		String[] srvsHeader;

		lookup = lookupService("rosjava.Master.getServiceType", service);
		host = lookup.substring(9, lookup.lastIndexOf(":"));
		port = Integer.parseInt(lookup.substring(lookup.lastIndexOf(":") + 1));

		srvsHeader = rosjava.Network.getServiceHeader(host, port, service);

		return srvsHeader[2].substring(srvsHeader[2].indexOf("=") + 1);
	}

	/**
	 * Get the md5sum of a given service
	 * 
	 * @param service
	 *            Fully-qualified name of service
	 * @return
	 */
	public String getMD5Sum(String service) {
		String lookup;
		String host;
		int port;
		String[] srvsHeader;

		lookup = lookupService("rosjava.Master.getMD5Sum", service);
		host = lookup.substring(9, lookup.lastIndexOf(":"));
		port = Integer.parseInt(lookup.substring(lookup.lastIndexOf(":") + 1));

		srvsHeader = rosjava.Network.getServiceHeader(host, port, service);

		return srvsHeader[1].substring(srvsHeader[1].indexOf("=") + 1);
	}

	/**
	 * Get list of topics that can be subscribed to.
	 * 
	 * @return
	 */
	public Collection<Topic> getPublishedTopics() {
		ArrayList<Topic> pubTopics = new ArrayList<Topic>();
		Object[] systemState = this
				.getSystemState("rosjava.Master.getPublishedTopics");
		Object[] publishedTopics = (Object[]) systemState[0];

		for (Object o : publishedTopics) {
			Object[] output = (Object[]) o;
			String topicName = (String) output[0];
			String msgsType = getTopicType(topicName);
			String[] publisher = getPublisher(topicName);
			String[] subscriber = getSubscriber(topicName);
			pubTopics.add(new Topic(topicName, msgsType, null, publisher,
					subscriber));
		}
		pubTopics.trimToSize();
		return pubTopics;
	}

	/**
	 * Get a collection of topics that are subscribed.
	 * 
	 * @return
	 */
	public Collection<Topic> getSubscribedTopics() {
		ArrayList<Topic> subTopics = new ArrayList<Topic>();
		Object[] systemState = this
				.getSystemState("rosjava.Master.getSubscribedTopics");
		Object[] subscribedTopics = (Object[]) systemState[1];

		for (Object o : subscribedTopics) {
			Object[] output = (Object[]) o;
			String topicName = (String) output[0];
			String msgsType = getTopicType(topicName);
			String[] publisher = getPublisher(topicName);
			String[] subscriber = getSubscriber(topicName);
			subTopics.add(new Topic(topicName, msgsType, null, publisher,
					subscriber));
		}
		subTopics.trimToSize();
		return subTopics;
	}

	/**
	 * returns a collection that contains all provided services
	 * 
	 * @return
	 */
	public Collection<Service> getServices() {
		ArrayList<Service> service = new ArrayList<Service>();
		Object[] systemState = this
				.getSystemState("rosjava.Master.getServices");
		Object[] srvs = (Object[]) systemState[2];

		for (Object input : srvs) {
			Object[] output = (Object[]) input;
			String serviceName = (String) output[0];
			String serviceType = getServiceType(serviceName);
			String md5Sum = getMD5Sum(serviceName);
			String[] provider = getServiceProviders(serviceName);
			service
					.add(new Service(serviceName, serviceType, md5Sum, provider));
		}
		service.trimToSize();
		return service;
	}

	/**
	 * Get the URI of the the master.
	 * 
	 * @param caller_id
	 *            ROS caller ID
	 * @return
	 */
	public String getUri(String caller_id) {
		Object[] params = new Object[] { caller_id };
		Object[] result;

		Integer code = new Integer(0);
		do {
			result = executeXMLRPCCommand("getUri", params);
			code = (Integer) result[0];
		} while (code.compareTo(new Integer(1)) != 0);

		return (String) result[2];
	}

}
