package instruction.postprocessor;

import instruction.exceptions.UnknownWordException;
import instruction.opencyc.OpenCyc;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;
import instruction.wordnet.WordNetRDF2;

import java.util.ArrayList;
import java.util.List;

public class BuildObjectNamesPostProcessor implements PostProcessor {

	public void run(List<Instruction> instructions) {

		for (int i = 0; i < instructions.size(); i++) {

			try {
				buildObjectNames(instructions.get(i));
			} catch (Exception e) {
				System.out.println(e.getMessage());
			}
		}
	}

	public void buildObjectNames(SemanticObject so) throws Exception {

		if (so instanceof Instruction) {
			Instruction i = (Instruction) so;

			List<ObjectX> objects = i.getObjects();
			for (int j = 0; j < objects.size(); j++) {
				try {
					buildObjectNames(objects.get(j));
					// System.out.println( "OBJECT READY: " + objects.get( j )
					// );
				} catch (Exception e) {
					System.out.println(e.getMessage());
					objects.remove(j);
				}
			}

			List<Preposition> pp = i.getPrepositions();
			for (int j = 0; j < pp.size(); j++)
				buildObjectNames(pp.get(j));
		}

		else if (so instanceof ObjectX) {
			ObjectX o = (ObjectX) so;

			buildObjectName(o);

			List<Preposition> pp = o.getPrepositions();
			for (int j = 0; j < pp.size(); j++)
				buildObjectNames(pp.get(j));
		}

		else if (so instanceof Preposition) {
			Preposition pp = (Preposition) so;
			List<ObjectX> o = pp.getObjects();
			for (int j = 0; j < o.size(); j++) {
				try {
					buildObjectNames(o.get(j));
				} catch (Exception e) {
					o.remove(j);
				}
			}
		}
	}

	private void buildObjectName(ObjectX object) throws Exception {

		int step = 0;
		int max_steps = 5;

		List<Word> objectName = object.getName();

		for (int i = 0; i < objectName.size(); i++) {
			if (objectName.get(i).getType() == Word.TYPE_PRONOUN
					&& objectName.size() > 1)
				objectName.remove(i);

			else if (objectName.get(i).getType() == Word.TYPE_PRONOUN)
				return;
		}

		if (getNouns(objectName).isEmpty())
			return;

		while (step < max_steps) {

			switch (step++) {

			// ==================================================================
			// 1. Step: Check if every Word is classified correctly
			// ==================================================================
			case 0:
				for (int i = 0; i < objectName.size(); i++) {
					if (WordNetRDF2.wordExistsAs(objectName.get(i).getLabel(),
							WordNetRDF2.convertPOS(objectName.get(i)
									.getType())))
						continue;
					else {
						if (WordNetRDF2.wordExistsAs(objectName.get(i)
								.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_NOUN)))
							objectName.get(i).setType(Word.TYPE_NOUN);
						else if (WordNetRDF2.wordExistsAs(objectName.get(i)
								.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_ADJ)))
							objectName.get(i).setType(Word.TYPE_ADJ);
						else if (WordNetRDF2.wordExistsAs(objectName.get(i)
								.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_ADV)))
							objectName.get(i).setType(Word.TYPE_ADV);
						else if (WordNetRDF2.wordExistsAs(objectName.get(i)
								.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_VERB_INFINITIVE)))
							objectName.get(i)
									.setType(Word.TYPE_VERB_INFINITIVE);
						else if (WordNetRDF2.wordExistsAs(objectName.get(i)
								.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_PAST_PARTICIPLE)))
							objectName.get(i)
									.setType(Word.TYPE_PAST_PARTICIPLE);
						else if (WordNetRDF2.wordExistsAs(objectName.get(i)
								.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_GERUND)))
							objectName.get(i).setType(Word.TYPE_GERUND);
						else
							throw new UnknownWordException("The Word \""
									+ objectName.get(i).getLabel()
									+ "\" is unknown in WordNetRDF2");
					}
				}
				break;

			// ==================================================================
			// 2. Step: Find noun compositions like "stove top" or "place mat"
			// ==================================================================
			case 1: {
				objectName = getNounCompositions(objectName);

				//
				// If there is only one object name remaining, we can stop right
				// here
				//
				if (getNouns(objectName).size() == 1)
					step = max_steps;

				break;
			}

				// ==================================================================
				// 3. Step: Try to convert the leading nouns into adjectives
				// ==================================================================
			case 2:
				// for ( int i = 0; i < objectName.size(); i++ ) {
				// Word w = objectName.get( i );
				// if ( w.getType() == Word.TYPE_NOUN && i < objectName.size() -
				// 1 ) {
				// if ( WordNetRDF2.wordExistsAs( w.getLabel(), Word.TYPE_ADJ ) )
				// {
				// w.setType( Word.TYPE_ADJ );
				// }
				// }
				// if ( w.getType() == Word.TYPE_NOUN && i == objectName.size()
				// - 1 ) {
				// objectName = getNounCompositions( objectName );
				//
				// if ( getNouns( objectName ).size() == 1 )
				// step = max_steps;
				// break;
				// }
				//
				// }
				break;

			// ==================================================================
			// 4. Step: If there is a Word determining a measure
			// (UnitOfMeasureConcept), create a Quantifier
			// ==================================================================
			case 3: {
				for (int i = 0; i < objectName.size(); i++) {
					ArrayList<String> synsets = WordNetRDF2.getSynsets(
							objectName.get(i).getLabel(), WordNetRDF2
									.convertPOS(objectName.get(i).getType()));
					for (int j = 0; j < synsets.size(); j++) {
						ArrayList<String> concepts = OpenCyc.getInstance()
								.getCycConceptFromWordNetID(
										"N" + synsets.get(j));
						for (int k = 0; k < concepts.size(); k++) {
							if (OpenCyc.getInstance().isaInUnivVocMt(
									concepts.get(k), "UnitOfMeasureConcept")
									&& i == 0) {
								System.out.println("Recognized quantifier: "
										+ objectName.get(i) + " ("
										+ concepts.get(k) + ")");
								object.getQuantifier().setMeasure(
										objectName.get(i));
								objectName.remove(i);
								objectName = getNounCompositions(objectName);
								if (objectName.size() == 1)
									step = max_steps;

								break;
							}
						}
					}
				}
				break;
			}

			case 4:
				break;
			}
		}

		// ==================================================================
		// 6. Step: Remove all the adjectives from the Object's name
		// ==================================================================
		for (int i = 0; i < objectName.size(); i++) {
			if (objectName.get(i).getType() == Word.TYPE_ADJ
					|| objectName.get(i).getType() == Word.TYPE_PAST_PARTICIPLE) {
				object.getAdjectives().add(objectName.get(i));
				objectName.remove(i--);
			}
		}

		// ==================================================================
		// 5. Step: Nest Objects using prepositional "of" relations
		// ==================================================================
		if (objectName.size() > 1) {
			ObjectX last = null;
			System.out.println("object name size " + objectName.size());
			for (int i = 0; i < objectName.size(); i++) {
				System.out.println("Building new object " + objectName.get(i));
				ObjectX newOb = new ObjectX();
				ArrayList<Word> name = new ArrayList<Word>();
				name.add(objectName.get(i));
				newOb.setName(name);
				if (last != null) {
					Preposition p = new Preposition();
					p.addPreposition(new Word(Word.TYPE_PREPOSITION, "of"));
					p.addObject(last);
					newOb.addPreposition(p);
					System.out.println("adding prp: " + p.toString());
				}
				last = newOb;
			}
			object.getPrepositions().addAll(last.getPrepositions());
			objectName = last.getName();
		}

		// ==================================================================
		// 7. Step: Check if the word now can be recognized
		// ==================================================================
		// if ( objectName.size() > 1 || objectName.size() == 0 ) {
		// String w = "";
		// for ( int i = 0; i < objectName.size(); i++ )
		// w = ( w + " " + objectName.get( i ).getLabel() ).trim();
		// throw new UnknownWordException( "The word \"" + w + "\" could not be
		// recognized" );
		// }

		object.setName(objectName);
		// System.out.println(object);
	}

	/**
	 * Extracts all nouns in the list of Words
	 * 
	 * @param words
	 * @return
	 */
	private ArrayList<Word> getNouns(List<Word> words) {

		ArrayList<Word> nouns = new ArrayList<Word>();
		for (int i = 0; i < words.size(); i++)
			if (words.get(i).getType() == Word.TYPE_NOUN)
				nouns.add(words.get(i));
		return nouns;
	}

	/**
	 * Finds known nouns that are composed by multiple nouns (e.g. "stove top")
	 * 
	 * @param oldList
	 * @return
	 * @throws Exception
	 */
	private List<Word> getNounCompositions(List<Word> oldList) throws Exception {

		ArrayList<Word> nouns = getNouns(oldList);
		ArrayList<Word> newList = new ArrayList<Word>();

		for (int i = 0; i < oldList.size(); i++) {
			if (oldList.get(i).getType() == Word.TYPE_ADJ
					|| oldList.get(i).getType() == Word.TYPE_PAST_PARTICIPLE)
				newList.add(oldList.get(i));
		}

		for (int i = 0; i < nouns.size(); i++) {

			Word objectName = new Word(nouns.get(i).getType(), nouns.get(i)
					.getLabel());
			Word tmpObjName = new Word(objectName.getType(), objectName
					.getLabel());
			int j = i + 1;
			while (j <= nouns.size()) {

				if (WordNetRDF2.wordExistsAs(tmpObjName.getLabel(), WordNetRDF2
						.convertPOS(Word.TYPE_NOUN))) {

					objectName.setLabel(tmpObjName.getLabel());
					if (j < nouns.size())
						tmpObjName.setLabel(tmpObjName.getLabel() + " "
								+ nouns.get(j++).getLabel());
					else {
						i = j - 1;
						break;
					}
				} else {
					i = j - 2;
					break;
				}

			}
			newList.add(objectName);
			objectName.setSynSetIDs(WordNetRDF2.getSynsets(
					objectName.getLabel(), WordNetRDF2.convertPOS(objectName
							.getType())));
		}

		return newList;
	}
}
