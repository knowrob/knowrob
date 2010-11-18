package instruction.syntaxparser;

import instruction.syntaxparser.SyntaxElement;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

public class SyntaxTree {

	SyntaxElement element;

	SyntaxTree parent;
	ArrayList<SyntaxTree> children = new ArrayList<SyntaxTree>();

	// Internal fields
	private int tree_depth;
	private int curr_depth;

	public ArrayList<SyntaxTree> getChildren() {

		return children;
	}

	public SyntaxElement getElement() {

		return element;
	}

	public void setElement( SyntaxElement element ) {

		this.element = element;
	}

	public void addChild( SyntaxTree child ) {

		children.add( child );
	}

	public void setParent( SyntaxTree parent ) {

		this.parent = parent;
	}

	public void printTree() {

		printTreeInternal( 0, this );
	}

	public SyntaxTree getParent() {

		return parent;
	}

	public ArrayList<SyntaxTree> getSiblings() {

		if ( parent == null ) {
			ArrayList<SyntaxTree> tmp = new ArrayList<SyntaxTree>();
			tmp.add( this );
			return tmp;
		}
		else
			return parent.getChildren();
	}

	private void _getdepth( SyntaxTree node ) {

		if ( node != null ) {
			curr_depth++;
			if ( curr_depth > tree_depth )
				tree_depth = curr_depth;
			for ( Iterator<SyntaxTree> i = node.getChildren().iterator(); i.hasNext(); )
				_getdepth( i.next() );
			curr_depth--;
		}
	}

	public int getdepth( ) {

		tree_depth = 0;
		_getdepth( this );
		return tree_depth;
	}

	private void printTreeInternal( int level, SyntaxTree tree ) {

		for ( int i = 0; i < level; i++ )
			System.out.print( "\t" );

	//	if ( tree.getElement() != null )
	//		System.out.println( "(" + tree.getElement().getType() + ", " + tree.getElement().getName() + ")" );

		for ( Iterator i = tree.getChildren().iterator(); i.hasNext(); ) {
			printTreeInternal( level + 1, (SyntaxTree) i.next() );
		}
	}

	public String toString() {

		StringBuilder str = new StringBuilder();
		toStringInternal( 0, this, str );
		return str.toString();
	}

	private void toStringInternal( int level, SyntaxTree tree, StringBuilder str ) {

		for ( int i = 0; i < level; i++ )
			str.append( "\t" );

		if ( tree.getElement() != null )
			str.append( "(" + tree.getElement().getType() + ", " + tree.getElement().getName() + ")" + "\n" );

		for ( Iterator i = tree.getChildren().iterator(); i.hasNext(); ) {
			toStringInternal( level + 1, (SyntaxTree) i.next(), str );
		}
	}

	public void writeToFile( FileWriter writer ) throws IOException {

		// writer.append("<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>");
		writeToFileInternal( 0, this, writer );
	}

	private void writeToFileInternal( int level, SyntaxTree tree, FileWriter writer ) throws IOException {

		for ( int i = 0; i < level; i++ )
			writer.append( "\t" );

		if ( tree.getElement() != null )
			writer.append( "<" + tree.getElement().getType() + " " + tree.getElement().getName() + ">\n" );

		for ( Iterator i = tree.getChildren().iterator(); i.hasNext(); ) {
			writeToFileInternal( level + 1, (SyntaxTree) i.next(), writer );
		}

		// writer.append("<" + tree.getElement().getType());
	}

	public String buildSentence() {

		StringBuffer str = new StringBuffer();
		buildSentenceInternal( str );
		return str.toString();
	}

	private void buildSentenceInternal( StringBuffer str ) {

		if ( getElement() != null && ! getElement().getName().equals( "" ) )
			str.append( getElement().getName() + " " );

		if ( getChildren().size() > 0 ) {
			for ( Iterator i = getChildren().iterator(); i.hasNext(); ) {
				SyntaxTree t = (SyntaxTree) i.next();
				t.buildSentenceInternal( str );
			}
		}
	}

}
