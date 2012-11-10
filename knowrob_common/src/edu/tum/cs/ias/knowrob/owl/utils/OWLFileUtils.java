package edu.tum.cs.ias.knowrob.owl.utils;

import java.io.*;
import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.vocab.PrefixOWLOntologyFormat;


/**
 * 
 * Utilities for reading and writing OWL files 
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 * @author Lars Kunze, kunzel@cs.tum.edu
 *
 */


public class OWLFileUtils {

	/**
	 * Loads an ontology from an OWL file
	 * 
	 * @param filename File that is to be loaded
	 * @return OWL ontology loaded from filename
	 * @throws OWLOntologyCreationException
	 */
	public static OWLOntology loadOntologyFromFile(String filename) throws OWLOntologyCreationException {
		OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
		return manager.loadOntologyFromOntologyDocument(new File(filename));
	}
	
	
	/**
	 * Saves an OWL ontology to an output stream in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param stream output stream
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToStream(OWLOntology ontology, OutputStream stream, OWLOntologyFormat format) {

		boolean ok = false;

		if (stream != null) {

			try {

				// Get hold of the ontology manager
				OWLOntologyManager manager = ontology.getOWLOntologyManager();

				// By default ontologies are saved in the format from which they were loaded.
				// We can get information about the format of an ontology from its manager
				OWLOntologyFormat currFormat = manager.getOntologyFormat(ontology);

				if (currFormat.equals(format)) {

					// Save a copy of the ontology to the given stream.
					manager.saveOntology(ontology, stream);

				} else {

					// Some ontology formats support prefix names and prefix IRIs. When we save the ontology in
					// the new format we will copy the prefixes over so that we have nicely abbreviated IRIs in
					// the new ontology document
					if (format.isPrefixOWLOntologyFormat() && currFormat.isPrefixOWLOntologyFormat()) {
						((PrefixOWLOntologyFormat)format).copyPrefixesFrom(currFormat.asPrefixOWLOntologyFormat());
					}
					manager.saveOntology(ontology, format, stream);

				}

				ok = true;

			} catch (Exception e) {
				System.out.println("Could not save ontology: " + e.getMessage());
			}

		}

		return ok;
	}


	/**
	 * Saves an OWL ontology to a String object.
	 * @param ontology ontology to be saved
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return String object containing the string representation of the ontology
	 */
	public static String saveOntologytoString(OWLOntology ontology, OWLOntologyFormat format) {

		String s = null;
		ByteArrayOutputStream os = new ByteArrayOutputStream(4096);

		if (saveOntologyToStream(ontology, os, format)) {
			try {
				s = new String(os.toByteArray(), "UTF-8");
			} catch (UnsupportedEncodingException e) {
				System.out.println("UTF-8 encoding is unsupported: " + e.getMessage());
			}			
		}

		return s;

	}



	/**
	 * Saves an OWL ontology to a file.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, OWLOntologyFormat format, String file) {

		boolean ok = false;

		try {
			ok = saveOntologyToFile(ontology, file, format);			
		} catch (NullPointerException e) {
			System.out.println("Could not save ontology: null pointer argument found\n" + e.getMessage());
		}

		return ok; 

	}


	/**
	 * Saves an OWL ontology to a file in a given ontology format.
	 * @param ontology ontology to be saved
	 * @param file name of target file
	 * @param format desired ontology format (@see OWLOntologyFormat)
	 * @return <tt>true</tt> - if saving was successfully completed<br>
	 * <tt>false</tt> - otherwise
	 */
	public static boolean saveOntologyToFile(OWLOntology ontology, String file, OWLOntologyFormat format) {

		boolean ok = false;

		try {

			File f = new File(file);
			OWLOntologyManager manager = ontology.getOWLOntologyManager();
			manager.saveOntology(ontology, format, IRI.create(f.toURI()));

			ok = true;

		} catch (Exception e) {
			System.out.println("Could not save ontology: " + e.getMessage());
		}

		return ok;
	}


}
