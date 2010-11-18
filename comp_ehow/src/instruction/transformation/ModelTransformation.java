package instruction.transformation;

import instruction.semanticObjects.SemanticObject;

import java.util.List;

public interface ModelTransformation {
	/**
	 * Transforms a Set of Semantic Objects into a new set of Semantic Objects
	 * according to the syntactic structure of the Pattern
	 * @return
	 */
	public abstract List<SemanticObject> doModelTransformation(List<SemanticObject> obj);
}
