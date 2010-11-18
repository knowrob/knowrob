package instruction.postprocessor;

import instruction.semanticObjects.Instruction;

import java.util.List;

/**
 * Abstract interface for PostProcessors
 * @author Daniel Nyga
 *
 */
public interface PostProcessor {
	
	/**
	 * Method is invoked when the PostProcessor is launched.
	 * 
	 * @param instructions Complete List of Instructions
	 */
	public void run( List<Instruction> instructions );

}
