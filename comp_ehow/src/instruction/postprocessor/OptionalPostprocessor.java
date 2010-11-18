package instruction.postprocessor;

import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.Word;

import java.util.List;

public class OptionalPostprocessor implements PostProcessor {

	public void run( List<Instruction> instructions ) {

		for (int i = 0; i < instructions.size(); i++) {
			Instruction in = instructions.get(i);
		
			for (int j = 0; j < in.getAction().getAdverb().size(); j++) {
				Word adv = in.getAction().getAdverb().get( j );
				if (adv.getCycConcepts().size() > 0 && adv.getCycConcepts().get( 0 ).equals( "Alternative" )) {
					in.setOptional( Instruction.OPTIONAL_GLOBAL );
				//	System.out.println("optional found");
				}
			}
			
			if (in.isGlobalOptional() && i > 1) {
				instructions.get( i-1 ).setOptional( Instruction.OPTIONAL_LOCAL );
			}
					
		}

	}

}
