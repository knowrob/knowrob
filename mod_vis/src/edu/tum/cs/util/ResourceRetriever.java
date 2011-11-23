package edu.tum.cs.util;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.URL;
import java.net.URLConnection;

import org.apache.commons.net.ftp.FTPClient;

public class ResourceRetriever {

	public static File retrieve(String url)
	{
		int start = url.indexOf('/')+2;
		int end = url.indexOf('/', start);
		String serverName = url.substring(start,end);
		if (url.startsWith("package://"))
		{
			String filePath = url.substring(end+1);
			String pkgPath = findPackage(serverName);
			if (pkgPath == null)
				return null;
			else
				return new File(pkgPath, filePath);
		} else if (url.startsWith("http://"))
		{

		    OutputStream out = null;
		    URLConnection conn = null;
		    InputStream in = null;
		    String filename = url.substring(url.lastIndexOf('/')+1);
			File tmpPath = new File(ResourceRetriever.createTempDirectory(),filename);
		    try {
		        // Get the URL
		        URL urlClass = new URL(url);
		        // Open an output stream to the destination file on our local filesystem
		        out = new BufferedOutputStream(new FileOutputStream(tmpPath));
		        conn = urlClass.openConnection();
		        in = conn.getInputStream();
		        
		        // Get the data
		        byte[] buffer = new byte[1024];
		        int numRead;
		        while ((numRead = in.read(buffer)) != -1) {
		            out.write(buffer, 0, numRead);
		        }
		        // Done! Just clean up and get out
	        } catch (Exception exception) {
	        	exception.printStackTrace();
	        } finally {
		        try {
		            if (in != null) {
			                in.close();
			        }
			        if (out != null) {
			            out.close();
			        }
		        } catch (IOException ioe) {
		        }
		    }
		    return tmpPath;
		} else if (url.startsWith("ftp://"))
		{
			FTPClient client = new FTPClient( );
			OutputStream outStream = null;
		    String filename = url.substring(url.lastIndexOf('/')+1);
			File tmpPath = new File(ResourceRetriever.createTempDirectory(),filename);
			try {
			    // Connect to the FTP server as anonymous
			    client.connect( serverName );
			    client.login( "anonymous", "knowrob@example.com" );
			    String remoteFile = url.substring(end);
			    // Write the contents of the remote file to a FileOutputStream
			    outStream = new FileOutputStream( tmpPath );
			    client.retrieveFile( remoteFile, outStream );
			    
			} catch(IOException ioe) {
			    System.out.println( "ResourceRetriever: Error communicating with FTP server: " + serverName + "\n" + ioe.getMessage() );
			} finally {
				try {
					outStream.close();
				} catch (IOException e1) {
				}
			    try {
			        client.disconnect( );
			    } catch (IOException e) {
			        System.out.println( "ResourceRetriever: Problem disconnecting from FTP server: " + serverName + "\n" + e.getMessage() );
			    }
			}
			return tmpPath;
		}
		return null;
	}
	

	/**
	 * creates a temporary directory used for unzipping
	 * @return File with path to created dir
	 */
	public static String createTempDirectory()
	{
		File temp;

		try {
			temp = File.createTempFile("temp", Long.toString(System.nanoTime()));
			if (!(temp.delete())) {
				throw new IOException("Could not delete temp file: "
						+ temp.getAbsolutePath());
			}

			if (!(temp.mkdir())) {
				throw new IOException("Could not create temp directory: "
						+ temp.getAbsolutePath());
			}
		} catch (IOException e) {
			e.printStackTrace();
			return "";
		}


		return temp.getAbsolutePath();
	}
	
	public static String findPackage(String pkgname) {
		
		try {	
			String line;		
			String cmd = "rospack find " + pkgname;
			Process p = Runtime.getRuntime().exec(cmd );
			BufferedReader errReader = new BufferedReader(new InputStreamReader(p.getErrorStream()));
            while ((line = errReader.readLine()) != null) {
            	System.err.println(line);
            }
			BufferedReader pathreader = new BufferedReader(new InputStreamReader(p.getInputStream(), "UTF-8"));
            if( (line = pathreader.readLine()) != null) {
            	return line;
            }
		} catch (IOException e) {
			e.printStackTrace(System.err);
		}
		return null;
	}
}
