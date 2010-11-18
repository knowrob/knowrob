package instruction.semanticObjects;

import java.util.ArrayList;

public class Sentence extends Instruction {
	
	private ArrayList<ObjectX> actors = new ArrayList<ObjectX>();
	
	public Sentence() {
		
	}
	
	public ArrayList<ObjectX> getActors() {
		return actors;
	}
	
	public void addActor(ObjectX actor) {
		actors.add( actor );
	}
	
	public void setActors(ArrayList<ObjectX> actors) {
		this.actors = actors;
	}
	
	public boolean equals(Object desc) {
		if (! (desc instanceof Sentence) )
				return false;
		return super.equals( desc ) && actors.equals( ((Sentence) desc).getActors() );
	}
	
	public String toString() {
		String str = "";
		str += action.getAction().getLabel();
		for (int i = 0; i < objects.size(); i++) {
			str += objects.get( i );
			if (i < objects.size() - 1)
				str += ",";
		}
		str += "(";
		for (int i = 0; i < actors.size(); i++) {
			str += actors.get( i );
			if (i < actors.size() - 1)
				str += ",";
		}
		str += ")";
		return str;
	}
}
