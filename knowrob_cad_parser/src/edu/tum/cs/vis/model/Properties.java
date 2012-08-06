package edu.tum.cs.vis.model;

import java.util.HashMap;
import java.util.Vector;

import edu.tum.cs.ias.knowrob.prolog.PrologInterface;

/**
 * Static class to obtain model properties such as path to model file, width, height, depth.
 * 
 * 
 * @author Stefan Profanter
 * @see ItemModel
 */
public class Properties {

	/**
	 * Instantiate an ItemModel from prolog identifier of Instance or Class. The path to the model
	 * will be gathered with prolog an this model will then be parsed.
	 * 
	 * @param identifier
	 *            Class or Instance identifier
	 * @return ItemModel instance if model parsed successfully. null otherwise.
	 */
	public static ItemModel getModelOfItem(String identifier) {
		String path = null;
		try {
			if (!identifier.startsWith("'") || !identifier.endsWith("'")) {
				identifier = "'" + identifier + "'";
			}
			HashMap<String, Vector<String>> nfo = PrologInterface.executeQuery("get_model_path("+ identifier + ",P)");

			if (nfo!=null && nfo.get("P") != null && nfo.get("P").size() > 0) {
				String str = nfo.get("P").get(0);

				if (str == null)
					return null;

				if (str.startsWith("'") && str.endsWith("'")) {
					str = str.substring(1, str.length() - 1);
				}
				path = str;

			}

		} catch (Exception e) {
			System.err.println("Calling get_model_path for Identifier " + identifier + " failed:");
			e.printStackTrace();
			return null;
		}

		if (path == null)
			return null;
		return new ItemModel(path);
	}

	/**
	 * Get x-Dimension (width) of class or instance identified by ObjClassOrIdentifier from the
	 * CAD-Model.
	 * 
	 * @param ObjClassOrIdentifier
	 *            ClassName or instance identifier
	 * @return x-Dimension or -1 if model not found
	 */
	public static float xDimOfObject(String ObjClassOrIdentifier) {
		ItemModel model = getModelOfItem(ObjClassOrIdentifier);
		if (model == null || model.getParser() == null)
			return -1;
		return model.getParser().getModel().getGroup().getTotalWidth();
	}

	/**
	 * Get y-Dimension (depth) of class or instance identified by ObjClassOrIdentifier from the
	 * CAD-Model.
	 * 
	 * @param ObjClassOrIdentifier
	 *            ClassName or instance identifier
	 * @return y-Dimension or -1 if model not found
	 */
	public static float yDimOfObject(String ObjClassOrIdentifier) {
		ItemModel model = getModelOfItem(ObjClassOrIdentifier);
		if (model == null || model.getParser() == null)
			return -1;
		return model.getParser().getModel().getGroup().getTotalDepth();
	}

	/**
	 * Get z-Dimension (height) of class or instance identified by ObjClassOrIdentifier from the
	 * CAD-Model.
	 * 
	 * @param ObjClassOrIdentifier
	 *            ClassName or instance identifier
	 * @return z-Dimension or -1 if model not found
	 */
	public static float zDimOfObject(String ObjClassOrIdentifier) {
		ItemModel model = getModelOfItem(ObjClassOrIdentifier);
		if (model == null || model.getParser() == null)
			return -1;
		return model.getParser().getModel().getGroup().getTotalHeight();
	}

}
