package instruction.gui.tab;

import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Quantifier;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import processing.core.PApplet;
import processing.core.PFont;

public class DataStructurePanel extends PApplet {

	private static final long serialVersionUID = -4061768594124955627L;

	private static int FONT_SIZE = 18;

	private static int FRAME_WIDTH = 1;
	private static int GAP = 50;

	PFont font = null;
	List<Instruction> instructions = null;
	Map<SemanticObject, String> designators = new HashMap<SemanticObject, String>();

	int activeInstructionIndex = 0;

	int iCounter = 1;
	int oCounter = 1;
	int pCounter = 1;
	int qCounter = 1;

	@Override
	public void setup() {

		size( 2500, 10000, JAVA2D );
		background( 255 );
		noLoop();
		font = createFont( "Arial", FONT_SIZE, true );
		smooth();

	}

	@Override
	public void draw() {

		fill( 0 );
		iCounter = oCounter = qCounter = pCounter = 1;
		textFont( font );
		background( 255 );
		if ( instructions == null ) {
			return;
		}
		generateDesignatorMap( instructions );

		float vSum = 0;
		// for ( Iterator<Instruction> i = instructions.iterator(); i.hasNext(); ) {
		// vSum += _draw( i.next(), GAP, vSum + GAP );
		//			
		// }
		_draw( instructions.get( activeInstructionIndex ), GAP, vSum + GAP );
		// size(2500, (int) vSum, JAVA2D);
	}

	public float _draw( SemanticObject so, float hOffset, float vOffset ) {

		float vSum = 0;

		if ( so instanceof Instruction ) {
			Instruction inst = (Instruction) so;

			float[] size = drawSemanticObject( getInstructionString( inst ), hOffset, vOffset );

			for ( Iterator<ObjectX> i = inst.getObjects().iterator(); i.hasNext(); ) {
				float sum = vSum;
				vSum += _draw( i.next(), hOffset + size[0] + GAP, vOffset + vSum ) + GAP;
				stroke( 1 );
				fill( 232, 238, 247 );
				line( hOffset + size[0], vOffset + size[1] / 2 + 5, hOffset + size[0] + GAP, vOffset + sum + 50 );
				ellipse( hOffset + size[0] - 5, vOffset + size[1] / 2, 10, 10 );
				noStroke();
			}

			for ( Iterator<Preposition> i = inst.getPrepositions().iterator(); i.hasNext(); ) {
				float sum = vSum;
				vSum += _draw( i.next(), hOffset + size[0] + GAP, vOffset + vSum ) + GAP;
				stroke( 1 );
				fill( 232, 238, 247 );
				line( hOffset + size[0], vOffset + size[1] / 2 + 25, hOffset + size[0] + GAP, vOffset + sum + 50 );
				ellipse( hOffset + size[0] - 5, vOffset + size[1] / 2 + 20, 10, 10 );
				noStroke();
			}

			if ( inst.getTimeConstraint() != null ) {
				float sum = vSum;
				vSum += _draw( inst.getTimeConstraint(), hOffset + size[0] + GAP, vOffset + vSum ) + GAP;
				stroke( 1 );
				fill( 232, 238, 247 );
				line( hOffset + size[0], vOffset + size[1] / 2 + 45, hOffset + size[0] + GAP, vOffset + sum + 50 );
				ellipse( hOffset + size[0] - 5, vOffset + size[1] / 2 + 40, 10, 10 );
				noStroke();
			}

			if ( inst.getTimeConstraint() == null && inst.getObjects().isEmpty()
					&& inst.getPrepositions().isEmpty() )
				vSum += size[1];

		}

		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;

			float[] size = drawSemanticObject( getObjectString( o ), hOffset, vOffset );

			for ( Iterator<Preposition> i = o.getPrepositions().iterator(); i.hasNext(); ) {
				float sum = vSum;
				vSum += _draw( i.next(), hOffset + size[0] + GAP, vOffset + vSum ) + GAP;
				stroke( 1 );
				fill( 232, 238, 247 );
				line( hOffset + size[0], vOffset + size[1] / 2 + 15, hOffset + size[0] + GAP, vOffset + sum + 50 );
				ellipse( hOffset + size[0] - 5, vOffset + size[1] / 2 + 10, 10, 10 );
				noStroke();
			}

			if ( ! o.getQuantifier().getAlternatives().isEmpty()
					|| ! o.getQuantifier().getMeasure().getLabel().isEmpty() ) {
				float sum = vSum;
				vSum += _draw( o.getQuantifier(), hOffset + size[0] + GAP, vOffset + vSum );
				stroke( 1 );
				fill( 232, 238, 247 );
				line( hOffset + size[0], vOffset + size[1] / 2 + 35, hOffset + size[0] + GAP, vOffset + sum + 40 );
				ellipse( hOffset + size[0] - 5, vOffset + size[1] / 2 + 30, 10, 10 );
				noStroke();
			
			}
			else
				vSum += size[1];
		}

		else if ( so instanceof Preposition ) {
			Preposition p = (Preposition) so;

			float[] size = drawSemanticObject( getPrepositionString( p ), hOffset, vOffset );

			for ( Iterator<ObjectX> i = p.getObjects().iterator(); i.hasNext(); ) {
				float sum = vSum;
				vSum += _draw( i.next(), hOffset + size[0] + GAP, vOffset + vSum );
				stroke( 1 );
				fill( 232, 238, 247 );
				line( hOffset + size[0], vOffset + size[1] / 2 + 20, hOffset + size[0] + GAP, vOffset + sum + 50 );
				ellipse( hOffset + size[0] - 5, vOffset + size[1] / 2 + 15, 10, 10 );
				noStroke();
			}
			if ( p.getObjects().isEmpty() )
				vSum += size[1];
		}

		else if ( so instanceof Quantifier ) {
			float[] size = drawSemanticObject( getQuantifierString( (Quantifier) so ), hOffset, vOffset );
			vSum += size[1];
		}

		return vSum;
	}

	public void roundRect( float x, float y, float width, float height, float radius ) {

		// Draw the frame
		stroke( 1 );
		fill( 0 );
		rect( x + radius, y, width - 2 * radius, height );
		rect( x, y + radius, width, height - 2 * radius );
		ellipseMode( CORNER );
		ellipse( x, y, radius * 2, radius * 2 );
		ellipse( x + width - 2 * radius, y, 2 * radius, 2 * radius );
		ellipse( x, y + height - 2 * radius, 2 * radius, 2 * radius );
		ellipse( x + width - 2 * radius, y + height - 2 * radius, 2 * radius, 2 * radius );

		// Draw internal rect
		fill( 232, 238, 247 );
		noStroke();
		rect( x + radius + FRAME_WIDTH, y + FRAME_WIDTH, width - 2 * radius - FRAME_WIDTH, height - FRAME_WIDTH );
		rect( x + FRAME_WIDTH, y + radius + FRAME_WIDTH, width - FRAME_WIDTH, height - 2 * radius - FRAME_WIDTH );

		ellipse( x + FRAME_WIDTH, y + FRAME_WIDTH, radius * 2, radius * 2 );
		ellipse( x + width - 2 * radius + FRAME_WIDTH, y + FRAME_WIDTH, 2 * radius - FRAME_WIDTH, 2 * radius
				- FRAME_WIDTH );
		ellipse( x + FRAME_WIDTH, y + height - 2 * radius + FRAME_WIDTH, 2 * radius - FRAME_WIDTH, 2 * radius
				- FRAME_WIDTH );
		ellipse( x + width - 2 * radius + FRAME_WIDTH, y + height - 2 * radius + FRAME_WIDTH, 2 * radius
				- FRAME_WIDTH, 2 * radius - FRAME_WIDTH );

	}

	public float[] drawSemanticObject( String str, float hOffset, float vOffset ) {

		float width = textWidth( str );
		int lines = 1;
		int idx = 0;
		while ( ( idx = str.indexOf( "\n", idx ) ) != - 1 ) {
			lines++;
			idx++;
		}
		float height = lines * ( textDescent() + FONT_SIZE + 4 );
		// System.out.println( "width: " + width + ", height: " + height + ", lines:

		// " + lines );
		// width = 120;
		roundRect( hOffset, vOffset, width + 20, height + 20, 5 );
		fill( 0 );
		text( str, 10 + hOffset, vOffset + 10, width, height );
		float[] ret = new float[2];
		ret[0] = width + 20;
		ret[1] = height;
		return ret;
	}

	public String getInstructionString( Instruction i ) {

		StringBuffer str = new StringBuffer();
		str.append( "Instruction " + designators.get( i ) + "\n" );
		str.append( "Action: \"" + i.getAction().getAction().getLabel() + "\" ("
				+ i.getAction().getAction().getCycConcepts().get( 0 ) + ")\n" );
		str.append( "Objects: { " );

		for ( Iterator<ObjectX> j = i.getObjects().iterator(); j.hasNext(); ) {
			str.append( designators.get( j.next() ) + ( j.hasNext() ? "," : "" ) );
		}
		str.append( " }\n" );
		str.append( "Postcondition: { " );
		for ( Iterator<Preposition> j = i.getPrepositions().iterator(); j.hasNext(); ) {
			str.append( designators.get( j.next() ) + ( j.hasNext() ? "," : "" ) );
		}
		str.append( " }\n" );
		str.append( "Duration: " + designators.get( i.getTimeConstraint() ) );

		return str.toString();
	}

	public String getObjectString( ObjectX o ) {

		StringBuffer str = new StringBuffer();
		str.append( "Object " + designators.get( o ) + "\n" );
		str.append( "Name: \"" + o.getName().get( 0 ).getLabel() + "\" ("
				+ o.getName().get( 0 ).getCycConcepts().get( 0 ) + ")\n" );
		str.append( "Prepositions: { " );
		for ( Iterator<Preposition> j = o.getPrepositions().iterator(); j.hasNext(); ) {
			str.append( designators.get( j.next() ) + ( j.hasNext() ? "," : "" ) );
		}
		str.append( " }\n" );
		String q = designators.get( o.getQuantifier() );
		str.append( "Quantifier: " + ((o.getQuantifier().getAlternatives().isEmpty() && o.getQuantifier().getMeasure().getLabel().isEmpty()) ? "null" : q));

		return str.toString();
	}

	public String getPrepositionString( Preposition p ) {

		StringBuffer str = new StringBuffer();
		str.append( "Preposition " + designators.get( p ) + "\n" );
		str.append( "Prep: \"" + p.getPrepositions().get( 0 ).getLabel() + "\"\n" );
		str.append( "Objects: { " );
		for ( Iterator<ObjectX> j = p.getObjects().iterator(); j.hasNext(); ) {
			str.append( designators.get( j.next() ) + ( j.hasNext() ? "," : "" ) );
		}
		str.append( " }\n" );

		return str.toString();
	}

	public String getQuantifierString( Quantifier q ) {

		StringBuffer str = new StringBuffer();
		str.append( "Quantifier " + designators.get( q ) + "\n" );
		str.append( "Amount: {" );
		for ( Iterator<Word> j = q.getAlternatives().iterator(); j.hasNext(); ) {
			str.append( j.next().getLabel() + ( j.hasNext() ? ", " : "" ) );
		}
		str.append( "}\n" );
		str.append( "Measure: " + q.getMeasure().getLabel() );
		if ( ! q.getMeasure().getLabel().isEmpty() )
			str.append( " (" + q.getMeasure().getCycConcepts().get( 0 ) + ")" );

		return str.toString();
	}

	public void setInstructions( List<Instruction> instructions ) {

		this.instructions = instructions;
	}

	@SuppressWarnings("unchecked")
	public void generateDesignatorMap( List so ) {

		for ( Iterator<SemanticObject> i = so.iterator(); i.hasNext(); ) {
			generateDesignatorMap( i.next() );
		}
	}

	public void generateDesignatorMap( SemanticObject so ) {

		if ( so instanceof Instruction ) {
			Instruction i = (Instruction) so;
			designators.put( i, "i" + iCounter++ );
			generateDesignatorMap( i.getObjects() );
			generateDesignatorMap( i.getPrepositions() );
			generateDesignatorMap( i.getTimeConstraint() );
		}
		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;
			designators.put( o, "o" + oCounter++ );
			generateDesignatorMap( o.getPrepositions() );
			generateDesignatorMap( o.getQuantifier() );
		}
		else if ( so instanceof Quantifier ) {
			designators.put( so, "q" + qCounter++ );
		}
		else if ( so instanceof Preposition ) {
			Preposition p = (Preposition) so;
			designators.put( p, "p" + pCounter++ );
			generateDesignatorMap( p.getObjects() );
		}
	}

	public int getActiveInstruction() {

		return activeInstructionIndex;
	}

	public void setActiveInstruction( int idx ) {

		activeInstructionIndex = idx;
	}

	public int getInstructionCount() {

		if ( instructions == null )
			return 0;
		else
			return instructions.size();
	}
}
