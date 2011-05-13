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

	public Master(String master_uri, int master_uri_port) {
		this.setMasterHost(master_uri);
		this.setMasterPort(master_uri_port);
	}

	/**
	 * Returns an Object containing Published Topics with Publisher, Subscribed
	 * Topics with Subscribers and Services with service Provider
	 * 
	 * @return an Object(String[String[]], String[String[]], String[String[]])
	 */
	public Object[] getSystemState() {
		String ros_master_uri = "http://" + getMasterHost() + ":"
				+ getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { "rosjava.Master" };
		try {
			Object[] result = (Object[]) rosmaster.execute("getSystemState",
					params);
			Integer code = (Integer) result[0];
			if (code.compareTo(new Integer(1)) == 0) {
				return (Object[]) result[2];
			}
		} catch (XmlRpcException e) {
			System.out
					.println("Error: An Error occured  during rosjava.Master.getSystemState()");
			System.out.println(e.getMessage());
		}

		return null;
	}

	/**
	 * returns an Object containing Topic and msgsType
	 * 
	 * @return an Object[](String, String)
	 */
	public Object[] getTopicTypes() {
		String ros_master_uri = "http://" + getMasterHost() + ":"
				+ getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { "rosjava.Master" };
		try {
			Object[] result = (Object[]) rosmaster.execute("getTopicTypes",
					params);
			Integer code = (Integer) result[0];
			if (code.compareTo(new Integer(1)) == 0) {
				return (Object[]) result[2];
			}
		} catch (XmlRpcException e) {
			System.out
					.println("Error: An Error occured  during rosjava.Master.getTopicTypes()");
		}
		;
		return null;
	}

	/**
	 * 
	 * @param service
	 * @return
	 */
	public String lookupService(String service) {
		String ros_master_uri = "http://" + getMasterHost() + ":"
				+ getMasterPort();

		XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
		try {
			config.setServerURL(new URL(ros_master_uri));
		} catch (MalformedURLException e) {
		}
		XmlRpcClient rosmaster = new XmlRpcClient();
		rosmaster.setConfig(config);

		Object[] params = new Object[] { "rosjava.Master", service };
		try {
			Object[] result = (Object[]) rosmaster.execute("lookupService",
					params);
			Integer code = (Integer) result[0];
			if (code.compareTo(new Integer(1)) == 0) {
				return (String) result[2];
			}
		} catch (XmlRpcException e) {
			System.out
					.println("Error: An Error occured  during rosjava.Master.lookupService("
							+ service + ")");
		}

		return null;
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
		Object[] systemState = this.getSystemState();
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
		Object[] systemState = this.getSystemState();
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
	 * 
	 * @param service
	 * @return
	 */
	public String[] getServiceProviders(String service) {
		ArrayList<String> provider = new ArrayList<String>();
		Object[] systemState = this.getSystemState();
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
	 * Returns the srvsType
	 * 
	 * @param serviceName
	 * @return a String
	 */
	public String getServiceType(String serviceName) {
		String lookup;
		String host;
		int port;
		String[] srvsHeader ;
		
		lookup = lookupService(serviceName);
		host = lookup.substring(9, lookup.lastIndexOf(":")); 
		port = Integer.parseInt(lookup.substring(lookup.lastIndexOf(":") + 1));
		
		srvsHeader = rosjava.Network.getServiceHeader(host, port, serviceName); 
		
		return srvsHeader[2];
	}

	/**
	 * 
	 * @return
	 */
	public Collection<Topic> getPublishedTopics() {
		ArrayList<Topic> pubTopics = new ArrayList<Topic>();
		Object[] systemState = this.getSystemState();
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

		return pubTopics;
	}

	/**
	 * 
	 * @return
	 */
	public Collection<Topic> getSubscribedTopics() {
		ArrayList<Topic> subTopics = new ArrayList<Topic>();
		Object[] systemState = this.getSystemState();
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

		return subTopics;
	}

	/**
	 * 
	 * @return
	 */
	public Collection<Service> getService() {
		ArrayList<Service> service = new ArrayList<Service>();
		Object[] systemState = this.getSystemState();
		Object[] srvs = (Object[]) systemState[2];

		for (Object input : srvs) {
			Object[] output = (Object[]) input;
			String serviceName = (String) output[0];
			String serviceType = getServiceType(serviceName);
			String[] provider = getServiceProviders(serviceName);
			service.add(new Service(serviceName, serviceType, null, provider));
		}

		return service;
	}

}
