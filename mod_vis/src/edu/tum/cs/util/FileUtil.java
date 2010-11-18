package edu.tum.cs.util;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class FileUtil {

	public static String readTextFile(File file) throws FileNotFoundException, IOException {		
		FileReader fr = new FileReader(file);
		char[] cbuf = new char[(int)file.length()];
		fr.read(cbuf);
		String content = new String(cbuf);
		fr.close();
		return content;
	}
	
	public static String readTextFile(String filename) throws FileNotFoundException, IOException {
		return readTextFile(new File(filename));
	}

}
