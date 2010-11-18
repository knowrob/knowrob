package instruction.semanticObjects;


public class Postcondition extends SemanticObject {
	
	private Sentence condition = new Sentence();
	
	public Postcondition() {
	}
	
	public Postcondition(Sentence condition) {
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
		if (! (c instanceof Postcondition))
			return false;
		return condition.equals( ((Postcondition) c).getCondition() );
	}
}
