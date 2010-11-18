package instruction.semanticObjects;


public class Precondition extends SemanticObject {
	
	private Sentence condition = new Sentence();
	
	public Precondition() {
	}
	
	public Precondition(Sentence condition) {
		this.condition = condition;
	}
	
	public Sentence getCondition() {
		return condition;
	}
	
	public void setCondition(Sentence condition) {
		this.condition = condition;
	}	
	
	public String toString() {
		return condition.toString();
	}
	
	public boolean equals(Object c) {
		if (! (c instanceof Precondition))
			return false;
		return condition.equals( ((Precondition) c).getCondition() );
	}

}
