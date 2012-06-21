package edu.tum.cs.util;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.Enumeration;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

/**
 * File Utils for reading text files and unzip.
 * 
 * @author Stefan Profanter
 * 
 */
public class FileUtil {

	/**
	 * Read a text file and return it as a String.
	 * 
	 * @param file
	 *            File to read
	 * @return the content of the file
	 * @throws FileNotFoundException
	 *             If file not found or invalid path
	 * @throws IOException
	 *             if there was an error reading the file
	 */
	public static String readTextFile(File file) throws FileNotFoundException, IOException {
		FileReader fr = new FileReader(file);
		char[] cbuf = new char[(int) file.length()];
		fr.read(cbuf);
		String content = new String(cbuf);
		fr.close();
		return content;
	}

	/**
	 * Read a text file and return it as a String.
	 * 
	 * @param filename
	 *            Filename as String. File must exist otherwise a FileNotFound exception will be
	 *            thrown.
	 * @return the content of the file
	 * @throws FileNotFoundException
	 *             If file not found or invalid path
	 * @throws IOException
	 *             if there was an error reading the file
	 */
	public static String readTextFile(String filename) throws FileNotFoundException, IOException {
		return readTextFile(new File(filename));
	}

	/**
	 * Unzip a zipped file (eg. kmz, zip) into given directory
	 * 
	 * @param zipFile
	 *            zipped file to unzip
	 * @param outputDirectory
	 *            destination directory for unzipped content
	 * @return true if successfully unzipped
	 */
	public static boolean Unzip(String zipFile, String outputDirectory) {
		String outDir = outputDirectory;
		if (!outDir.endsWith("/") && !outDir.endsWith("\\"))
			outDir += "/";

		BufferedOutputStream dest = null;
		BufferedInputStream is = null;
		int BUFFER = 2048;
		ZipEntry entry;
		ZipFile zipfile;
		try {
			zipfile = new ZipFile(zipFile);
			Enumeration<? extends ZipEntry> e = zipfile.entries();
			while (e.hasMoreElements()) {
				entry = e.nextElement();
				if (entry.isDirectory()) {
					(new File(outDir + entry.getName())).mkdir();
					continue;
				}

				String filename = outDir + entry.getName();
				String filePath = filename.substring(0, filename.lastIndexOf(File.separator));

				// Create directory if not existing
				if (!(new File(filePath)).exists()) {
					(new File(filePath)).mkdirs();
				}
				is = new BufferedInputStream(zipfile.getInputStream(entry));
				int count;
				byte data[] = new byte[BUFFER];
				FileOutputStream fos = new FileOutputStream(filename);
				dest = new BufferedOutputStream(fos, BUFFER);
				while ((count = is.read(data, 0, BUFFER)) != -1) {
					dest.write(data, 0, count);
				}
				dest.flush();
				dest.close();
				is.close();
			}
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			System.err.println("Couldn't unzip file: " + zipFile);
			e1.printStackTrace();
			return false;
		}
		return true;
	}

}
