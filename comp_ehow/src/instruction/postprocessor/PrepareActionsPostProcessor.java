package instruction.postprocessor;

import instruction.semanticObjects.Action;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Word;
import instruction.wordnet.WordNetRDF2;

import java.util.List;

public class PrepareActionsPostProcessor implements PostProcessor {

	public void run(List<Instruction> instructions) {

		try {
			for (int i = 0; i < instructions.size(); i++) {
				Instruction curInst = instructions.get(i);
				Action action = curInst.getAction();

				for (int j = 0; j < curInst.getPrepositions().size(); j++) {

					String tmpAction = action.getAction().getLabel();
					Preposition prep = curInst.getPrepositions().get(j);

					if (prep.getPrepositions().size() == 1) {

						tmpAction += " "
								+ prep.getPrepositions().get(0).getLabel();
						if (WordNetRDF2.wordExistsAs(tmpAction, WordNetRDF2
								.convertPOS(Word.TYPE_VERB_INFINITIVE))
								&& curInst.getObjects().isEmpty()) {
							action.setAction(new Word(
									Word.TYPE_VERB_INFINITIVE, tmpAction));
							action
									.getAction()
									.setSynSetIDs(
											WordNetRDF2
													.getSynsets(
															tmpAction,
															WordNetRDF2
																	.convertPOS(Word.TYPE_VERB_INFINITIVE)));
							curInst.getObjects().addAll(prep.getObjects());
							curInst.getPrepositions().remove(j);
						}
					}
				}

			}
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}
	}

}
