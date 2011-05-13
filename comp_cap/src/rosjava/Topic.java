package rosjava;

public class Topic {

	private String[] publisher;
	private String[] subscriber;
	private String name;
	private String datatype;
	private String md5sum;
	
	public Topic(String name, String datatype, String md5sum,
			String[] publisher, String[] subscriber) {
		this.publisher = publisher;
		this.subscriber = subscriber;
		this.name = name;
		this.datatype = datatype;
		this.md5sum = md5sum;
	}

	public String[] getPublisher() {
		return publisher;
	}

	public String[] getSubscriber() {
		return subscriber;
	}

	public String getName() {
		return name;
	}

	public String getDatatype() {
		return datatype;
	}

	public String getMd5sum() {
		return md5sum;
	}

	
}
