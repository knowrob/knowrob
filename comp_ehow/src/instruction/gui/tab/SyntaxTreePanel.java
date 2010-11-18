package instruction.gui.tab;

import instruction.syntaxparser.SyntaxElement;
import instruction.syntaxparser.SyntaxTree;
import java.util.Iterator;
import java.util.List;
import processing.core.PApplet;
import processing.core.PFont;

public class SyntaxTreePanel extends PApplet {

	private static final long serialVersionUID = 8312785333436185487L;
	private static int NODE_SIZE = 40;
	private static int COLUMN_WIDTH = 50;
	private static int ROW_HEIGHT = 60;
	private static int FONT_SIZE = 12;

	PFont font = null;

	List<SyntaxTree> tree = null;
	int leafCount = 0;
	int offset = 0;
	int maxdepth = 0;
	
	int activeTreeIndex = 0;

	int height_total = 100;

	public void setSyntaxTree( List<SyntaxTree> tree ) {

		this.tree = tree;
	}

	@Override
	public void setup() {

		size( 2500, height_total, JAVA2D );
		background( 255 );
		noLoop();
		font = createFont( "Arial", FONT_SIZE, true );
		smooth();

	}

	@Override
	public void draw() {

		if ( tree == null )
			return;

	// Compute the necessary height of the panel
		if ( tree != null ) {
			height_total = 0;
			maxdepth = 0;
			offset = 0;
			leafCount = 0;
//			for ( Iterator<SyntaxTree> i = tree.iterator(); i.hasNext(); ) {
//				SyntaxTree t = i.next();
//				height_total += t.getdepth();
//			}
			height_total += tree.get( activeTreeIndex ).getdepth();
			height_total *= ROW_HEIGHT;
			height_total += ROW_HEIGHT * tree.size();
		}
		size( 2500, height_total, JAVA2D );
		background( 255 );
//		setBounds( getBounds() );
//		invalidate();
		
//		for ( Iterator<SyntaxTree> i = tree.iterator(); i.hasNext(); )
//			drawTree( i.next() );
		drawTree(tree.get( activeTreeIndex ));

	}

	private void drawTree( SyntaxTree root ) {

		drawTreeRecursive( root, 0 );
		offset += ( maxdepth + 1 ) * ROW_HEIGHT + ROW_HEIGHT;
		maxdepth = 0;
		leafCount = 0;
	}

	private int drawTreeRecursive( SyntaxTree node, int depth ) {

		if ( node.getChildren().isEmpty() ) {
			if ( depth > maxdepth )
				maxdepth = depth;
			return ( ++leafCount * COLUMN_WIDTH );
		}
		else {
			int pos = 0;
			int children = node.getChildren().size();
			int[] childPositions = new int[children];
			int counter = 0;
			for ( Iterator<SyntaxTree> i = node.getChildren().iterator(); i.hasNext(); ) {
				childPositions[counter] = drawTreeRecursive( i.next(), depth + 1 );
				pos += childPositions[counter];
				counter++;
			}
			pos = (int) Math.round( (double) pos / (double) children );
			for ( int i = 0; i < childPositions.length; i++ ) {
				line( pos, ( depth + 1 ) * ROW_HEIGHT + offset, childPositions[i], ( depth + 2 ) * ROW_HEIGHT
						+ offset );
				drawNode( node.getChildren().get( i ), childPositions[i], ( depth + 2 ) * ROW_HEIGHT + offset );
			}
			drawNode( node, pos, ( depth + 1 ) * ROW_HEIGHT + offset );
			return pos;
		}
	}

	private void drawNode( SyntaxTree node, int x, int y ) {

		stroke( 0 );
		strokeWeight( 2.0f );
		fill( 232, 238, 247 );
		textAlign( CENTER, CENTER );

		rectMode( CENTER );
		ellipse( x, y, NODE_SIZE, NODE_SIZE );
		fill( 0 );
		textFont( font );

		SyntaxElement el = node.getElement();
		text( el.getType(), x, y, NODE_SIZE, NODE_SIZE );
		if ( el.getName() != null && ! el.getName().isEmpty() )
			text( el.getName(), x, y + NODE_SIZE );

	}
	
	public void setActiveTree(int index) {
		activeTreeIndex = index;
//		redraw();
	}
	
	public int getActiveTree() {
		return activeTreeIndex;
	}
	
	public int getTreeCount() {
		if (tree == null)
			return 0;
		else
			return tree.size();
	}
}
