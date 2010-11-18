package instruction.patterns;

import java.util.ArrayList;

import instruction.semanticObjects.SemanticObject;
import instruction.syntaxparser.Parser;
import instruction.syntaxparser.SyntaxTree;

/**
 * Abstract super-class for Syntax Tree Patterns
 * @author daniel
 *
 */
public abstract class AbstractSyntaxPattern {
	
	SyntaxTree tree;
	String pattern;
	
	/**
	 * Creates the Syntax Pattern
	 * @param p The Pattern String
	 */
	public AbstractSyntaxPattern(String p)  {
		pattern = p;
	}
	
	/**
	 * Initializes the Pattern by parsing the Pattern String
	 * into a Syntax Tree.
	 * @param p an initialized Parser Object
	 */
	public final void init(Parser p) {
		ArrayList<String> tokens = p.tokenize(pattern);
		tree = p.generateTree(null, tokens);
	}
	
	/**
	 * Returns the parsed Syntax Tree.
	 * @return
	 */
	public final SyntaxTree getPattern() {
		return tree;
	}
	
	public final String getPatternString() {
		return pattern;
	}
	
	/**
	 * Transforms a Set of Semantic Objects into a new set of Semantic Objects
	 * according to the syntactic structure of the Pattern
	 * @return
	 */
	public abstract ArrayList<SemanticObject> doModelTransformation(ArrayList<SemanticObject> obj);
}
