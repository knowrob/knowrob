package instruction.postprocessor;

import instruction.opencyc.OpenCyc;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Quantifier;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.List;

import org.opencyc.api.CycApiException;

public class TimeConstraintPostProcessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		for ( int i = 0; i < instructions.size(); i++ ) {
			findTimeConstraint( instructions.get( i ), instructions.get( i ), instructions.get( i ) );
		}
	}

	private void findTimeConstraint( SemanticObject so, Instruction parentInstruction, SemanticObject parentSO ) {

		if ( so instanceof Instruction ) {
			Instruction inst = (Instruction) so;

			List<ObjectX> objects = inst.getObjects();
			for ( int i = 0; i < objects.size(); i++ )
				findTimeConstraint( objects.get( i ), parentInstruction, parentSO );

			List<Preposition> pp = inst.getPrepositions();
			for ( int i = 0; i < pp.size(); i++ )
				findTimeConstraint( pp.get( i ), parentInstruction, parentSO );

		}

		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;

			List<Preposition> pp = o.getPrepositions();
			for ( int i = 0; i < pp.size(); i++ )
				findTimeConstraint( pp.get( i ), parentInstruction, o );
		}

		else if ( so instanceof Preposition ) {
			Preposition pp = (Preposition) so;

			// Explore the Prepositions
			for ( int j = 0; j < pp.getPrepositions().size(); j++ ) {

				if ( pp.getPrepositions().get( j ).getLabel().equalsIgnoreCase( "for" ) ) {

				//	System.out.println( "found prep \"for\"" );
					for ( int k = 0; k < pp.getObjects().size(); k++ ) {

						Word time = null;
						try {
							time = getTimeMeasure( pp.getObjects().get( k ) );
						}
						catch ( Exception e ) {
							System.out.println(e.getMessage());
						}
						

						if ( time != null ) {

							// Build the time constraint
							Quantifier q = new Quantifier();
							q.setMeasure( time );
							q.setAlternatives( pp.getObjects().get( k ).getQuantifier().getAlternatives() );

							if ( q.getAlternatives().size() == 0 )
								q.getAlternatives().add( new Word( Word.TYPE_NUMBER, "1" ) );

							parentInstruction.setTimeConstraint( q );

							// Remove the preposition identified as time constraint from the
							// parent Semantic Object
							if ( parentSO instanceof Instruction )
								( (Instruction) parentSO ).getPrepositions().remove( pp );
							else if ( parentSO instanceof ObjectX )
								( (ObjectX) parentSO ).getPrepositions().remove( pp );
						}
					}
				}
			}
		}
	}

	/**
	 * Checks if the given <code>ObjectX o</code> represents a time measuring
	 * unit and returns the corresponding <code>Word</code>
	 * 
	 * @param o
	 * @return
	 * @throws IOException
	 * @throws CycApiException
	 * @throws UnknownHostException
	 */
	private Word getTimeMeasure( ObjectX o ) throws UnknownHostException, CycApiException, IOException {

		for ( int i = 0; i < o.getName().size(); i++ ) {
			for ( int j = 0; j < o.getName().get( i ).getCycConcepts().size(); j++ ) {
				String c = o.getName().get( i ).getCycConcepts().get( j );
				if ( OpenCyc.getInstance().isaInUnivVocMt( c, "UnitOfTime" ) )
					return o.getName().get( i );
			}
		}
		return null;
	}

}
