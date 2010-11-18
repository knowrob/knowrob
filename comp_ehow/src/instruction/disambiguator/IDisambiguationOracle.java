package instruction.disambiguator;

import instruction.semanticObjects.Instruction;

import java.util.List;

/**
 * Interface for an Entity which serves as an 'oracle' for the disambiguator. An
 * Implementation of <code>IDisambiguatorOracle</code> communicates with the
 * disambiguator via callback-methods that are invoked by the disambiguator.
 * 
 * @author Daniel Nyga
 * 
 */
public interface IDisambiguationOracle {

	/**
	 * Returns the correct meaning of the word <code>word</code> in the
	 * Instruction <code>instruction</code>. This meaning must be one of the
	 * Cyc-Concepts contained in the <code>meanings</code> list. If
	 * <code>null</code> is returned the first meaning in the list is assumed
	 * to be correct.
	 * 
	 * @param action
	 * @param meanings
	 * @param sentence
	 * @return
	 */
	public String retrieveMeaningOfWord(String action, List<String> meanings,
			Instruction instruction);	
	

}
