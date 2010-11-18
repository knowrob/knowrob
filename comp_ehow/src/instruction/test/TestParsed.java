package instruction.test;

import java.io.File;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import instruction.configuration.ConfigurationManager;
import instruction.importer.PlanImporter;
import instruction.opencyc.OpenCyc;
import instruction.wrapper.LocalParseTree;

public class TestParsed {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		ConfigurationManager.loadSettings();

		PlanImporter importer = new PlanImporter();
		try {
			importer.initialize();

			System.out.println("Initializing Plan-Importer...");
			Map<String, List<String>> mappings = ConfigurationManager
					.getMappings();
			Set<String> synsets = mappings.keySet();
			for (Iterator<String> i = synsets.iterator(); i.hasNext();) {
				String synset = i.next();
				List<String> concepts = mappings.get(synset);
				for (Iterator<String> j = concepts.iterator(); j.hasNext();) {
					OpenCyc.getInstance().addMapping(synset, j.next());
				}
			}
			importer.getDisambiguator().load(
					ConfigurationManager.getPathDisambiguator());
			System.out.println("Plan-Importer initialized.");

			int r;
			String s = "";
			do {
				r = System.in.read();
				if (r != (char) '\n' && r != (char) '\r')
					s += (char) r;
			} while (r != (char) '\n');

			String filePath = "D:/ba_workspace/instruction_factory/parsed/"
					+ s.replaceAll(" ", "_");

			if (new File(filePath).exists()) {
				LocalParseTree wrapper = new LocalParseTree();
				wrapper.load(filePath);
				importer.setWrapper(wrapper);
				importer.parseInstructions();
				importer.recognizeAndDisambiguateInstructions();
				importer.convert2CycAssertions();
			}
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
