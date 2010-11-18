package instruction.gui.internal;

import instruction.importer.PlanImporter;

public class PlanImporterWrapper {
	
	static private PlanImporter importer = null;
	
	static public PlanImporter getImporter() {
		if (importer == null)
			importer = new PlanImporter();
		
		return importer;
	}
}
