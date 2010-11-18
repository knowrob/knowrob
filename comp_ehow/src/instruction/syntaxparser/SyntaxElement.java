package instruction.syntaxparser;


public class SyntaxElement {
	
	String type;
	String name;
	
	public SyntaxElement(String type, String name) {
		this.type = type;
		this.name = name;
	}
	
	public String getName() {
		return name;
	}
	
	public String getType() {
		return type;
	}
	
	public void setName(String name) {
		this.name = name;
	}
	
	public void setType(String type) {
		this.type = type;
	}
	
}
