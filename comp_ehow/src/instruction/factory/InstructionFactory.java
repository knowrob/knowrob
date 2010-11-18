package instruction.factory;

import instruction.converter.Instruction2CycLConverter;
import instruction.disambiguator.Disambiguator;
import instruction.exceptions.InstructionException;
import instruction.postprocessor.InstructionPostProcessor;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;
import instruction.syntaxparser.Parser;
import instruction.syntaxparser.SyntaxTree;
import instruction.transformation.Transformations;
import instruction.wordnet.WordNetRDF2;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class InstructionFactory {

	Parser parser = null;
	Disambiguator disambiguator = new Disambiguator();

	public InstructionFactory() {

	}

	public Disambiguator getDisambiguator() {

		return disambiguator;
	}

	/**
	 * Initializes the Parser and generates Tree Patterns
	 * 
	 * @param p
	 */
	public InstructionFactory(Parser p) {

		parser = p;
	}

	public void makeHowTo(String path) throws URISyntaxException {

		Parser parser = new Parser();
		InstructionFactory factory = new InstructionFactory(parser);
		InstructionPostProcessor postProc = new InstructionPostProcessor();

		File howtoFile = new File(path);
		if (howtoFile.exists()) {

			parser.setFileToParse(howtoFile.getPath());

			ArrayList<Instruction> howto = new ArrayList<Instruction>();

			for (Iterator<String> iter = parser.iterator(); iter.hasNext();) {
				String sentence = iter.next();
				// System.out.println( sentence );

				for (int k = 0; k <= Parser.PRE_PROCESSOR_LOWER_CASE; k += Parser.PRE_PROCESSOR_LOWER_CASE) {
					parser.usePreProcessor(Parser.PRE_PROCESSOR_QUOTATION_MARKS
							| Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY | k);

					SyntaxTree tree = parser.parse(sentence);

					try {
						List<Instruction> instructions = factory
								.makeInstructions(tree);

						howto.addAll(instructions);

						if (!instructions.isEmpty())
							break;
					} catch (Exception e) {
						e.printStackTrace();
						continue;
					}
				}
			}

			for (int i = 0; i < howto.size(); i++)
				System.out.println(howto.get(i));

			try {
				postProc.run(howto);
			} catch (Exception e2) {
				e2.printStackTrace();
			}

			for (int i = 0; i < howto.size(); i++)
				System.out.println(howto.get(i));

			try {
				disambiguator.disambiguateInstructions(howto);
			} catch (Exception e1) {
				e1.printStackTrace();
			}

			for (int i = 0; i < howto.size(); i++)
				System.out.println(howto.get(i));
			// Get title of HowTo
			String title = "Then " + howtoFile.getName().replaceAll("_", " ");
			SyntaxTree t = parser.parse(title);
			t.printTree();
			try {
				List<Instruction> titles = factory.makeInstructions(t);

				postProc.run(titles);
				disambiguator.disambiguateInstructions(titles);

				if (titles.size() == 0 || titles.size() > 2)
					throw new InstructionException(
							"Title of HowTo could not be resolved.");
				System.out.println(titles.get(0));
				Instruction2CycLConverter converter = new Instruction2CycLConverter();
				converter.convertHowTo(howto, titles.get(0));
			} catch (Exception e) {
				System.out.println(e.getMessage());
				e.printStackTrace();
			}

		} else
			System.out.println("File " + path + " not found.");

	}

	public List<Instruction> makeInstructions(SyntaxTree tree) throws Exception {

		List<SemanticObject> semObjects = transformTree(tree);
		List<Instruction> inst = new ArrayList<Instruction>();
		for (int i = 0; i < semObjects.size(); i++) {
			if (semObjects.get(i) instanceof Instruction) {
				inst.add((Instruction) semObjects.get(i));
			}
		}
		return inst;
	}

	private List<SemanticObject> transformTree(SyntaxTree tree)
			throws Exception {

		List<SemanticObject> objList = new ArrayList<SemanticObject>();

		if (tree.getChildren().isEmpty()) {
			Word word = mapSyntaxTreeNodeToWord(tree);
			word.setSynSetIDs(WordNetRDF2.getSynsets(word.getLabel(), WordNetRDF2
					.convertPOS(word.getType())));
			objList.add(word);
		} else {
			for (int i = 0; i < tree.getChildren().size(); i++) {
				List<SemanticObject> obj = transformTree(tree.getChildren()
						.get(i));
				for (int j = 0; j < obj.size(); j++)
					objList.add(obj.get(j));
			}
		}

		return Transformations.get(tree.getElement().getType())
				.doModelTransformation(objList);
	}

	private Word mapSyntaxTreeNodeToWord(SyntaxTree tree) {

		if (tree == null || tree.getElement() == null
				|| tree.getElement().getName().isEmpty())
			return null;

		Word word = new Word();

		word.setLabel(tree.getElement().getName());

		String type = tree.getElement().getType();

		if (type.matches("NN.*"))
			word.setType(Word.TYPE_NOUN);

		else if (type.matches("VBG"))
			word.setType(Word.TYPE_GERUND);

		else if (type.matches("VBN"))
			word.setType(Word.TYPE_PAST_PARTICIPLE);

		else if (type.matches("VB.*"))
			word.setType(Word.TYPE_VERB_INFINITIVE);

		else if (type.equals("RB"))
			word.setType(Word.TYPE_ADV);

		else if (type.matches("JJ.*"))
			word.setType(Word.TYPE_ADJ);

		else if (type.matches("RP"))
			word.setType(Word.TYPE_PARTICLE);

		else if (type.matches("CD"))
			word.setType(Word.TYPE_NUMBER);

		else if (type.matches(".*PRP.*|WDT"))
			word.setType(Word.TYPE_PRONOUN);

		else if (type.matches("IN|TO|WRB"))
			word.setType(Word.TYPE_PREPOSITION);

		else if (type.matches("CC"))
			word.setType(Word.TYPE_CONJ);

		else if (type.matches("DT"))
			word.setType(Word.TYPE_ARTICLE);

		else if (type.equals(",") || type.equals("``") || type.equals("''")
				|| type.equals("."))
			word.setType(Word.TYPE_PUNCTUATION);

		return word;
	}

}
