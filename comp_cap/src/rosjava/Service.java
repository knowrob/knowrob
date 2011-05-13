package rosjava;

public class Service {
	
	private String[] provider;
	private String name;
	private String datatype;
	private String md5sum;

	public Service(String name, String datatype, String md5sum,
			String[] provider) {
		this.provider = provider;
		this.name = name;
		this.datatype = datatype;
		this.md5sum = md5sum;
	}

	public String[] getProvider() {
		return provider;
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
