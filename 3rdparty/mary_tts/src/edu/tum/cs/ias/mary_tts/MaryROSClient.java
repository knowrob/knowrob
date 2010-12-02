package edu.tum.cs.ias.mary_tts;

/**
 * ROS Client for Mary TTS. Written by Moritz Tenorth, TUM IAS
 * 
 * Based on the original MaryClientUser example, 
 * 
 * Copyright 2000-2006 DFKI GmbH.
 * All Rights Reserved.  Use is subject to license terms.
 * 
 * Permission is hereby granted, free of charge, to use and distribute
 * this software and its documentation without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of this work, and to
 * permit persons to whom this work is furnished to do so, subject to
 * the following conditions:
 * 
 * 1. The code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 * 2. Any modifications must be clearly marked as such.
 * 3. Original authors' names are not deleted.
 * 4. The authors' names are not used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * DFKI GMBH AND THE CONTRIBUTORS TO THIS WORK DISCLAIM ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL DFKI GMBH NOR THE
 * CONTRIBUTORS BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
 * THIS SOFTWARE.
 */


import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;

import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.LineEvent;
import javax.sound.sampled.LineListener;
import javax.sound.sampled.UnsupportedAudioFileException;

import marytts.util.data.audio.AudioPlayer;
import marytts.client.MaryClient;
import marytts.client.http.Address;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.Subscriber;
import ros.pkg.mary_tts.srv.*;


/**
 * Simple ROS client
 * 
 * @author marc
 *
 */

public class MaryROSClient {

	static MaryClient mary;
	static LineListener lineListener;
	static ByteArrayOutputStream baos;
	static AudioInputStream ais;
	
	static Ros ros;
	static NodeHandle n;
    static Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback;
	
    static String voice = "dfki-obadiah";
        
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("mary_text_to_speech");
		}
		n = ros.createNodeHandle();
        callback = new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>();
	}
	
	public MaryROSClient() {
		
		try {
			init_mary();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		// start ROS environment
		initRos();    

        Thread speechIn = new Thread( new SpeechOutListenerThread() );
        speechIn.start();

        Thread answerQueries = new Thread( new AnswerQueriesThread() );
        answerQueries.start();
	    
        Thread startRosServices = new Thread( new RosServiceThread() );
        startRosServices.start();
        
	}
	


    protected void init_mary() throws IOException {
    	
    	 String serverHost = System.getProperty("server.host", "localhost");
         int serverPort = Integer.getInteger("server.port", 59125).intValue();
         
         boolean initialized=false;
         
         while(!initialized) {
	         try{
	        	 Thread.sleep(1000);
	        	 mary = MaryClient.getMaryClient(new Address(serverHost, serverPort));
	        	 initialized=true;
	         } catch(IOException e) {  } 
	           catch (InterruptedException e) { }
         }

	        
         lineListener = new LineListener() {
			public void update(LineEvent event) {
			    if (event.getType() == LineEvent.Type.START) {
			        System.err.println("Audio started playing.");
				} else if (event.getType() == LineEvent.Type.STOP) {
				    System.err.println("Audio stopped playing.");
				} else if (event.getType() == LineEvent.Type.OPEN) {
				    System.err.println("Audio line opened.");
				} else if (event.getType() == LineEvent.Type.CLOSE) {
				    System.err.println("Audio line closed.");
				    }
				}
			};  
		}

    public static class SpeechOutListenerThread implements Runnable {

        @Override public void run() {

                try {

                        n.advertise("/mary/tts", new ros.pkg.std_msgs.msg.String(), 100);
                        Subscriber<ros.pkg.std_msgs.msg.String> sub = n.subscribe("/mary/tts", new ros.pkg.std_msgs.msg.String(), callback, 10);

                        n.spin();
                        sub.shutdown();

                } catch(RosException e) {
                        e.printStackTrace();
                }
        }
    }


	
    public static class AnswerQueriesThread implements Runnable {
    	
    	
    	public AnswerQueriesThread() {
			
		}
    	
    	@Override public void run() {

                try {

                        ros.pkg.std_msgs.msg.String res;
                        while (n.isValid()) {

   				     res = callback.pop();

                                     try{

   				            baos = new ByteArrayOutputStream();
   				        	mary.process(res.data, "TEXT", "AUDIO", "en_GB", "WAVE", voice, baos);
   							ais = AudioSystem.getAudioInputStream(new ByteArrayInputStream(baos.toByteArray()));
	   				        AudioPlayer ap = new AudioPlayer(ais, lineListener);
	   				        ap.start();

   				        } catch (IOException e) {
   				            e.printStackTrace();
   				        }  catch (UnsupportedAudioFileException e) {
   							e.printStackTrace();
   					}
    			}

    		} catch (InterruptedException e) {
    			e.printStackTrace();
    		}
    	}
    }

    
    public class RosServiceThread implements Runnable {

        @Override public void run() {

                try {
                    n.advertiseService("/mary_tts/set_voice",  new SetMaryVoice(),  new SetVoiceCallback());
                    ros.spin();

                } catch(RosException e) {
                        e.printStackTrace();
                }
        }
  }
    
    /**
     * 
     * callback class for setting the voice
     * 
     * @author Moritz Tenorth, tenorth@cs.tum.edu
     *
     */
    class SetVoiceCallback implements ServiceServer.Callback<SetMaryVoice.Request, SetMaryVoice.Response> {
        
        @Override
        public SetMaryVoice.Response call(SetMaryVoice.Request req) {

            SetMaryVoice.Response res = new SetMaryVoice.Response();
            res.result=0;
            
            if (req.voice_name != null && req.voice_name.length() > 0) {
                voice = req.voice_name;
                res.result=1;
            }

            return res;
        }
    }
    
    
    public static void main(String[] args) {
    	new MaryROSClient();
    }
}
