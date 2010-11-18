package instruction.semanticObjects;

import java.util.ArrayList;
import java.util.List;

public class Action extends SemanticObject {

	Word action = new Word();
	List<Word> adverb = new ArrayList<Word>();
	boolean dont;

	public Action () {

	}

	public Action ( Word a ) {

		action = a;
	}

	public Action ( Word a, List<Word> adv ) {

		action = a;
		adverb = adv;
	}

	public boolean dontDoIt() {

		return dont;
	}

	public void setDontDoIt( boolean dont ) {

		this.dont = dont;
	}

	public String toString() {

		return ( dont ? "! " : "" ) + action.toString();
	}

	public void setAction( Word action ) {

		this.action = action;
	}

	public void setAdverbs( List<Word> adverb ) {

		this.adverb = adverb;
	}

	public Word getAction() {

		return action;
	}

	public List<Word> getAdverb() {

		return adverb;
	}

	public boolean equals( Object a ) {
		if (! (a instanceof Action))
			return false;
		if ( ((Action) a).getAction().equals( action ) && ((Action) a).getAdverb().equals( adverb ) )
			return true;

		return false;
	}
}
