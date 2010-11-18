package instruction.postprocessor;

import instruction.semanticObjects.Action;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.Word;
import instruction.wordnet.WordNetRDF2;

import java.util.List;

public class InstructionFilterPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		for ( int i = 0; i < instructions.size(); i++ ) {
			Instruction cur = instructions.get( i );
			Action a = cur.getAction();
			try {
				if ( ! WordNetRDF2.wordExactlyExistsAs( a.getAction().getLabel(), WordNetRDF2.convertPOS(Word.TYPE_VERB_INFINITIVE) ) ) {
				//	System.out.println("Word " + a.getAction().getLabel() + " not found in WordNet");
					instructions.remove( cur );
					i -= 1;
					continue;
				}
			}
			catch ( Exception e ) {
				System.out.println( e.getMessage() );
				instructions.remove( cur );
				i -= 1;
				continue;
			}
		}
	}

}
