package edu.tum.cs.ias.knowrob.comp_cap;
import java.io.*;

public class CapabilityBase{
	
	String pathFile;
	
	CapabilityBase() {
		pathFile = "tmp/topics";
	}

  private String[] readFile(String path) throws Exception{
    int length = 0; 
    String line;
    String[] listLines;
        
    try{
	      File file = new File(path);
	      BufferedReader in = new BufferedReader(new FileReader(file));
	
	      if(!file.exists()){
	    	  throw new ExistsException("File doesn't exist.");
	      }
	      if(!file.isFile()){
	    	  throw new IsFileException("Is not a File");
	      }
	      if(!file.canRead()){
	    	  throw new ReadException("Cannot read");
	      }
	
	      while(in.readLine() != null){
	    	  length++;
	      }
	
	      BufferedReader in2 = new BufferedReader(new FileReader(file));
	      listLines = new String[length];
	      
	      for(int i = 0; i < length; i++){
		   line = in2.readLine();
	           listLines[i] = line;
	      }
	
	      in.close();
	      in2.close();
      
      return listLines;
    } catch (ExistsException e) {
    	System.out.println("ExistException");
    } catch (IsFileException e) {
    	System.out.println("IsFileException");
    } catch (ReadException e) {
    	System.out.println("ReadException");
    }
    return null; 
  }
  
  private boolean check(String[] d, String topicFile){
	  String[] topics = null;
	  String[] dependencies = d;
	  boolean[] exists = new boolean[dependencies.length];
	  boolean result = false;
			  
	  try {
		topics = this.readFile(topicFile);
	  } catch (Exception e) {
		System.out.println("Wrong Filepath");
	  }
	  
	  if(topics != null){
		  for(int i = 0; i < dependencies.length; i++){
			  for(int j = 0; j < topics.length; j++){
				  if(topics[j] == dependencies[i]){
					  exists[i] = true;
					  break;
				  }
			  }
		  }
	  }
		
	  for(int i = 0; i < exists.length; i++){
		if(!exists[i]){
			result = false;
			break;
		}
		result = true;
	  }
	  
	  return result;
  }
  
  /**
   * 
   * @param debug
   * @return true if debug is 1 or check returns true else false
   */
  public boolean cap_move_base(String debug){
	  if(debug == "-1"){
		  return false;
	  }
	  if(debug == "1"){
		  return true;
	  }
	  String[] depency = {"a","b"};
	  
	  try {
			java.lang.Runtime.getRuntime().exec("rostopic list >> " + pathFile);
		} catch (IOException e) {}
	  
	  return check(depency, pathFile);
  }

  /**
   * 
   * @param debug
   * @return true if debug is 1 or check returns true else false
   */
  public boolean cap_move_arm(String debug){
	  if(debug == "-1"){
		  return false;
	  }
	  if(debug == "1"){
		  return true;
	  }
	  String[] depency = {"a","b"};
	  
	  try {
		java.lang.Runtime.getRuntime().exec("rostopic list >> " + pathFile);
	} catch (IOException e) {}
	
	  return check(depency, pathFile);
  }
 
} 
