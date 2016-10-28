/*
 * Copyright (c) 2012-14 Moritz Tenorth
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

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
   
    /**
     * Runs the given launch file of the given ROS package 
     * @param pkg name of the ROS package
     * @param launch_file name of the launch file
     * @param args Arguments
     * @return Handle to this process
     * @throws IOException 
     */
    public static String roslaunch(String pkg, String launch_file, String[] arg) throws IOException {

        String handle = null;
        try
        {
        	String args = Joiner.on(" ").join(arg);
        	Process p = Runtime.getRuntime().exec("roslaunch " + pkg + " " + launch_file + " " + args);

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
