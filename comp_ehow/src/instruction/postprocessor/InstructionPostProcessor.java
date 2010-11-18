package instruction.postprocessor;

import instruction.importer.AddCycMappingListener;
import instruction.semanticObjects.Instruction;

import java.util.ArrayList;
import java.util.List;


/**
 * 
 * @author Daniel Nyga
 * 
 */
public class InstructionPostProcessor {
	
	private List<PostProcessor> registry = new ArrayList<PostProcessor>();
	private MeaningsPostProcessor meaningsPostProc;
	
	
	public InstructionPostProcessor() {
		
	//	registry.add( new InstructionFilterPostProcessor() );
		registry.add( new PrepareActionsPostProcessor() );
		registry.add( new PreparePrepositionalPhrasesPostProcessor() );
		registry.add( new BuildObjectNamesPostProcessor() );
		registry.add( new PronounResolverPostProcessor() );	
		meaningsPostProc = new MeaningsPostProcessor();
		registry.add( meaningsPostProc );
		registry.add( new ObjectAmountPostProcessor() );
		registry.add( new TimeConstraintPostProcessor() );
		registry.add( new OptionalPostprocessor() );
	}

	/**
	 * Runs the PostProcessor on the given List of Instructions
	 * 
	 * @param instructions
	 * @throws Exception
	 */
	public void run( List<Instruction> instructions ) throws Exception {
		
		// Run all registered PostProcessors
		for (int i = 0; i < registry.size(); i++) 
			registry.get( i ).run( instructions );
		
	}


	public void printArray( ArrayList a ) {

		for ( int i = 0; i < a.size(); i++ )
			System.out.print( a.get( i ).toString() + " | " );
		System.out.println( "" );
	}
	
	public void setAddMappingListener(AddCycMappingListener listener) {
		meaningsPostProc.setAddMappingListener(listener);
	}
}
