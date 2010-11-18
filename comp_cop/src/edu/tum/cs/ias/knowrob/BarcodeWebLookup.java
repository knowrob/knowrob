/*
 * Copyright (C) 2010 by Moritz Tenorth
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package edu.tum.cs.ias.knowrob;

import ros.*;
import ros.pkg.comp_cop.srv.LookupBarcode;

import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.io.IOException;
import java.io.UnsupportedEncodingException;

import org.apache.http.HttpEntity;
import org.apache.http.HttpResponse;
import org.apache.http.HttpVersion;
import org.apache.http.client.*;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.params.CoreProtocolPNames;
import org.apache.http.util.EntityUtils;

import com.google.api.translate.Language;
import com.google.api.translate.Translate;


public class BarcodeWebLookup {

	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;
	
	
	public BarcodeWebLookup() {

  		try {

  			initRos();
  			
			n.advertiseService("/barcode_web_lookup",  new LookupBarcode(),  new LookupBarcodeCallback());
			ros.spin();
	    		
  		} catch(RosException e) {
  			e.printStackTrace();
  		}
	}
	
	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("knowrob_barcode_web_lookup");
		}
		n = ros.createNodeHandle();

	}
	

	public static String lookUpUpcDatabase(String ean) {
		
		String res="";
		if (ean != null && ean.length()>0) {
			
			HttpClient httpclient = new DefaultHttpClient();

			try {
				
				HttpGet httpget = new HttpGet("http://www.upcdatabase.com/item/"+ean);
				httpget.getParams().setParameter(CoreProtocolPNames.HTTP_CONTENT_CHARSET, "UTF-8");
				
		        // Create a response handler
				
				ResponseHandler<byte[]> handler = new ResponseHandler<byte[]>() {
				    public byte[] handleResponse(
				            HttpResponse response) throws ClientProtocolException, IOException {
				        HttpEntity entity = response.getEntity();
				        if (entity != null) {
				            return EntityUtils.toByteArray(entity);
				        } else {
				            return null;
				        }
				    }
				};

				String responseBody= new String(httpclient.execute(httpget, handler), "UTF-8");
			
		        
				// return null if not found
				if(responseBody.contains("Item Not Found"))
					return null;
				else if(responseBody.contains("Item Record")) {

					// Parse response document
					Matcher matcher = Pattern.compile("<tr><td>Description<\\/td><td><\\/td><td>(.*)<\\/td><\\/tr>").matcher(responseBody);
					if(matcher.find()) {
						res=matcher.group(1);
					}
					
				}
				
			} catch (UnsupportedEncodingException uee) {
				uee.printStackTrace();
			} catch (ClientProtocolException cpe) {
				cpe.printStackTrace();
			} catch (IOException ioe) {
				ioe.printStackTrace();
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}
			
		}
		return res;
	}
	
	
	public static String translate(String word) {
		
	    String translatedText="";
	    if(word!=null && word.length()>0) {
		    try{
		    	Translate.setHttpReferrer("http://ias.cs.tum.edu");
		    	translatedText = Translate.execute(word, Language.GERMAN, Language.ENGLISH);
		    } catch (Exception e) {
		    	e.printStackTrace();
		    }
	    }
	    
	    return translatedText;
	}


	
	// MT: xmlrpc seems to be broken, is always adding a '0' at the beginning
	public static String getUPCText(String upc) {
	    String text = "";
//	    try
//	    {
//	        XmlRpcClient client  = new XmlRpcClient( "http://www.upcdatabase.com/rpc", false);
//	        String[] args = new String[1];
//	        args[0]=upc;
//	        Object result  = client.invoke( "lookupUPC", args );
//	        return result.toString();
//
//	    }
//	    catch (Exception e)
//	    {
//	    	e.printStackTrace();
//	    }
	    return text;
	}
	


	
	// MT: does not work -- they seem to check if there is a real user agent here (?)
	public static String lookUpEANsearch(String ean) {
		
		String res="";
		if (ean != null && ean.length()>0) {
			

			HttpClient httpclient = new DefaultHttpClient();
			httpclient.getParams().setParameter(CoreProtocolPNames.PROTOCOL_VERSION, HttpVersion.HTTP_1_1);
			//httpclient.getParams().setParameter(CoreProtocolPNames.HTTP_CONTENT_CHARSET, "UTF-16");


			try {
				
				HttpGet httpget = new HttpGet("http://www.ean-search.org/perl/ean-search.pl?ean="+ean+"&os=1");
				httpget.getParams().setParameter(CoreProtocolPNames.HTTP_CONTENT_CHARSET, "UTF-8");
				httpget.getParams().setParameter(CoreProtocolPNames.HTTP_ELEMENT_CHARSET, "ASCII");

				 
				System.out.println(httpget.getURI());

		        // Create a response handler
				
				ResponseHandler<byte[]> handler = new ResponseHandler<byte[]>() {
				    public byte[] handleResponse(
				            HttpResponse response) throws ClientProtocolException, IOException {
				        HttpEntity entity = response.getEntity();
				        if (entity != null) {
				            return EntityUtils.toByteArray(entity);
				        } else {
				            return null;
				        }
				    }
				};
				
		        byte[] response = httpclient.execute(httpget, handler);

		        
		        
			//	String responseBody = httpclient.execute(httpget, handler);
				

//				new HeapByteBuffer(responseBody.getBytes(), 0, responseBody.getBytes().length));
//				
//				Charset a  = Charset.forName("UTF-8");
//				a.newEncoder().encode(responseBody.getBytes());
//				
//				System.out.println(responseBody);

				// Parse response document
				res = response.toString();
				
			} catch (UnsupportedEncodingException uee) {
				uee.printStackTrace();
			} catch (ClientProtocolException cpe) {
				cpe.printStackTrace();
			} catch (IOException ioe) {
				ioe.printStackTrace();
			} finally {
				if (httpclient != null) {
					httpclient.getConnectionManager().shutdown();
				}
			}
			
		}
		return res;
	}
	
	
		/**
		 * 
		 * Callback class for querying the Web for the object type of a barcode
		 * 
		 * @author Moritz Tenorth, tenorth@cs.tum.edu
		 *
		 */
		class LookupBarcodeCallback implements ServiceServer.Callback<LookupBarcode.Request, LookupBarcode.Response> {
			
			@Override
			public LookupBarcode.Response call(LookupBarcode.Request req) {

				LookupBarcode.Response res = new LookupBarcode.Response();
				res.objectclass="";

				if (req.ean != null) {
					
					if(! ((req.ean.length()==8 ) || (req.ean.length()==13 ))) {
						ros.logError("LookupBarcode: malformed EAN (not 8 or 13 digits)");
					}
					
					res.objectclass = BarcodeWebLookup.lookUpUpcDatabase(req.ean);
					
				}
				return res;
				
			}
		}
	
	
	
	public static void main(String[] args) {
	
		// test client: lookup all EANs in the array eans		
		//String[] eans = new String[] {"4006144617636", "40084015", "87303810", "4305399070501", "4305399060052", "4004764841561"};
		//
		//for(String ean : eans)
		//	System.out.println(translate(BarcodeWebLookup.lookUpUpcDatabase(ean)));
		
		// default: launch the ROS node
		new BarcodeWebLookup();
		
	}

}
