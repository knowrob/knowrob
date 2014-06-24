package org.knowrob.utils.ros;

import java.io.*;
import java.util.HashMap;
import java.util.UUID;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.AbstractNodeMain;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.google.common.base.Joiner;
import com.google.common.collect.Lists;


public class RosUtilities {

	static HashMap<String, Process> processMap = new HashMap<String, Process>();
	
    /**
     * Finds a ROS package using rospack and returns its path. 
     * @param name of the ROS package
     * @return path to the package - if it was found<br>
     * <tt>null</tt> - otherwise
     */
    public static String rospackFind(String pkg) {

        String path = null;
        try {
        	
       	
            Process p = Runtime.getRuntime().exec("rospack find " + pkg);
            
            if(p.waitFor()==127) {
            //	throw new RuntimeException("External program 'rospack' not found");
            }

            BufferedReader out = new BufferedReader(new InputStreamReader(p.getInputStream()));
            BufferedReader err = new BufferedReader(new InputStreamReader(p.getErrorStream()));

            if ((path = out.readLine()) != null){
                ;//System.out.println("Package: " + pkg + ", Path: " + path);                    
            } else {
            	
            	// print error output
            	String l = null;
            	while ( (l = err.readLine()) != null)
                    System.out.println(l);
            	
                ;//System.out.println("Package: " + pkg + ", Error: package not found!");
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        return path;
    }
    

    /**
     * Finds a ROS package using rospack and returns its path. 
     * @param pkg name of the ROS package
     * @param binary name of the binary
     * @param args Arguments
     * @return Handle to this process
     * @throws IOException 
     */
    public static String rosrun(String pkg, String binary, String[] arg) throws IOException {

        String handle = null;
        try
        {
        	String args = Joiner.on(" ").join(arg);
        	Process p = Runtime.getRuntime().exec("rosrun " + pkg + " " + binary + " " + args);

        	handle = UUID.randomUUID().toString();
            processMap.put(handle, p);
            
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        return handle;
    }
    
    
    public static void runRosjavaNode(AbstractNodeMain node, String[] args) {
    	
    	Logger.getLogger("ros").setLevel(Level.WARNING);
    	
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(args));
        NodeConfiguration nodeConfiguration = loader.build();

        NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(node, nodeConfiguration);

    }
    
    
    
    /**
     * Kill a process based on its UUID (which has been returned 
     * by the RosUtilities.rosrun() method
     * @param UUID of the process to be killed
     * @param binary name of the binary
     */
    public static void kill(String handle) {
    	processMap.get(handle).destroy();
    }
    
}
