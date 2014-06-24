/* 
 * Copyright (c) 2011, Sjoerd van den Dries
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
 */

package tfjava;

import org.ros.internal.loader.CommandLineLoader;
import org.ros.message.Time;

import javax.vecmath.Vector3d;
import javax.vecmath.Quat4d;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;

import com.google.common.collect.Lists;


import tf.tfMessage;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import geometry_msgs.Quaternion;

/**
 * 
 * @author Sjoerd van den Dries
 * @version March 3, 2011
 * 
 * Class for broadcasting tf messages.
 * 
 */
public class TFBroadcaster  extends AbstractNodeMain {

	Publisher<tfMessage> pub;
	public ConnectedNode node;
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		pub = connectedNode.newPublisher("/tf", tf.tfMessage._TYPE); //TODO: does this actually work? Will this be started from Prolog?
	}
	
	//TODO: make methods static again?
	
    /**
     * Publishes a tf message on the tf topic with the specified parameters.
     */    
    public void sendTransform(Vector3d transl, Quat4d rot, Time time, String parentFrame, String childFrame) {
        
        try {
            // convert translation vector and quaternion to geometry messages
            Vector3 tMsg = node.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
            Quaternion rMsg = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
            tMsg.setX(transl.x); tMsg.setY(transl.y); tMsg.setZ(transl.z);
            rMsg.setX(rot.x); rMsg.setY(rot.y); rMsg.setZ(rot.z); rMsg.setW(rot.w);
                
            // create TransformStamped message (is a geometry msg, do NOT confuse with StampedTransform class)
            TransformStamped tfMsg = node.getTopicMessageFactory().newFromType(geometry_msgs.TransformStamped._TYPE);        
            tfMsg.getHeader().setFrameId(parentFrame);
            tfMsg.getHeader().setStamp(time);
            tfMsg.setChildFrameId(childFrame);
            tfMsg.getTransform().setTranslation(tMsg);
            tfMsg.getTransform().setRotation(rMsg);
            
            // create tfMessage and add TransformStamped message to it
            tfMessage msg = node.getTopicMessageFactory().newFromType(tf.tfMessage._TYPE);
            msg.getTransforms().add(tfMsg);
            
            // publish the message
            pub.publish(msg);
            
        } catch (Exception e) {
            node.getLog().error("TFBroadcaster: " + e.toString());
        }
    }
    
    public void sendTransform(StampedTransform t) {
    	sendTransform(t.getTranslation(), t.getRotation(), t.timeStamp, t.frameID, t.childFrameID);
    }

    @Override
    public GraphName getDefaultNodeName() {
    	return GraphName.of("tfjava_broadcaster");
    }
    
    
    
    public static void main(String argv[]) throws java.io.IOException {
    	
        // Pulling the internals of rosrun to slightly customize what to run
        String[] args = { "org.knowrob.tf_prolog.tfjava.TFBroadcaster" };
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(args));
        NodeConfiguration nodeConfiguration = loader.build();
        TFBroadcaster tfBroadcaster = new TFBroadcaster();

        NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(tfBroadcaster, nodeConfiguration);
        try {
        	
        	Thread.sleep(2000);
            tfBroadcaster.sendTransform(new Vector3d(), new Quat4d(), tfBroadcaster.node.getCurrentTime(), "parent", "child");
        } catch(Exception e) {
            System.out.println(e.getMessage());
        }

    }  
    
    
}
