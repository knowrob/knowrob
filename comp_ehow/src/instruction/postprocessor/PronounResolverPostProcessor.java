package instruction.postprocessor;

import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Word;
import java.util.List;

public class PronounResolverPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		ObjectX lastObjectTalkedAbout = null;
		for ( int i = 0; i < instructions.size(); i++ ) {
			lastObjectTalkedAbout = resolvePronouns( instructions.get( i ), lastObjectTalkedAbout );
		}
	}

	public ObjectX resolvePronouns( Instruction inst, ObjectX object ) {

		ObjectX lastObjectTalkedAbout = object;

		List<ObjectX> objects = inst.getObjects();

		for ( int i = 0; i < objects.size(); i++ ) {
			List<Word> objName = objects.get( i ).getName();

			/**
			 * Resolution is ony possible, if the word name consists only of 1 Word
			 * which is a pronoun. lastObjectTalkedAbout must be non-null.
			 */
			if ( objName.size() == 1 && objName.get( 0 ).getType() == Word.TYPE_PRONOUN ) {
				if ( lastObjectTalkedAbout == null ) {
					inst.getObjects().remove( i );
					i -= 1;
					continue;
				}
				else {
					inst.getObjects().remove( i );
					inst.getObjects().add( i, lastObjectTalkedAbout );
				}
			}
			else
				lastObjectTalkedAbout = objects.get( i );

		}

		return lastObjectTalkedAbout;
	}
}
