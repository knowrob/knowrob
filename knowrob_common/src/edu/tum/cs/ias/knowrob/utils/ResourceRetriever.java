/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.ias.knowrob.utils;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.net.URL;
import java.net.URLConnection;

import org.apache.commons.net.ftp.FTPClient;

/**
 * Retrieves a file from given url to a temporary path and returns this path. Supported protocol
 * types: http, ftp, package
 * 
 * Notes about package type: You can use an url like 'package://mod_vis/path/to/file.zip' to get the
 * physical file path of the specified file in the package. So this url will be transformed to
 * something like: '/path_to_ros/mod_vis/path/to/file.zip' ResourceRetriever tries to find the
 * specified package by calling "rospack find PACKAGE"
 * 
 * @author Stefan Profanter
 * 
 */
public class ResourceRetriever {

	/**
	 * Prefix for files in /tmp folder
	 */
	private static final String	tmpPrefix	= "knowrob_resource_";

	/**
	 * Creates a temporary directory (normally in the /tmp folder)
	 * 
	 * @return File with path to created dir
	 */
	public static String createTempDirectory() {

		String tmpDir = System.getProperty("java.io.tmpdir");

		File temp = new File(tmpDir, "temp" + Long.toString(System.nanoTime()));

		if (!(temp.mkdir())) {
			System.err.println("Could not create temp directory: " + temp.getAbsolutePath());
		}

		return temp.getAbsolutePath();
	}

	/**
	 * Tries to find the specified ros package by calling 'rospack find'.
	 * 
	 * @param pkgname
	 *            Package to search for
	 * @return Absolute path to the package or null if not found
	 */
	public static String findPackage(String pkgname) {

		try {
			String line;
			String cmd = "rospack find " + pkgname;
			Process p = Runtime.getRuntime().exec(cmd);
			BufferedReader errReader = new BufferedReader(new InputStreamReader(p.getErrorStream()));
			while ((line = errReader.readLine()) != null) {
				System.err.println(line);
			}
			BufferedReader pathreader = new BufferedReader(new InputStreamReader(
					p.getInputStream(), "UTF-8"));
			if ((line = pathreader.readLine()) != null) {
				return line;
			}
		} catch (IOException e) {
			e.printStackTrace(System.err);
		}
		return null;
	}

	/**
	 * Creates a temporary filename. The filename is in the format:
	 * {tmpPrefix}+{MD5(url)}+{ext(filename)}. So this function will create always the same
	 * temporary name for the same URL.
	 * 
	 * @param url
	 *            Url as identifier.
	 * @param filename
	 *            Filename for getting the file name extension.
	 * @return null if MD5(url) returns null. Otherwise it returns the temp filename.
	 * @see MD5
	 */
	private static File getTmpName(String url, String filename) {
		String hashName = MD5(url);
		if (hashName == null)
			return null;

		int idx = filename.lastIndexOf('.');
		String ext = "";
		if (idx > 0)
			ext = filename.substring(idx);

		String tmpDir = System.getProperty("java.io.tmpdir");

		File f = new File(tmpDir, tmpPrefix + hashName + ext);

		return f;
	}

	/**
	 * Calculate MD5 checksum of the given string.
	 * 
	 * @param md5
	 *            Value to calculate md5 for.
	 * @return null if encoding unsupported or algorithm not found
	 */
	private static String MD5(String md5) {
		try {
			java.security.MessageDigest md = java.security.MessageDigest.getInstance("MD5");
			byte[] array = md.digest(md5.getBytes("UTF-8"));
			StringBuffer sb = new StringBuffer();
			for (int i = 0; i < array.length; ++i) {
				sb.append(Integer.toHexString((array[i] & 0xFF) | 0x100).substring(1, 3));
			}
			return sb.toString();
		} catch (java.security.NoSuchAlgorithmException e) {
			System.err.println("ResourceRetriever NoSuchAlgorithmException Error: "
					+ e.getMessage());
		} catch (UnsupportedEncodingException e) {
			System.err.println("ResourceRetriever UnsupportedEncodingException Error: "
					+ e.getMessage());
		}
		return null;
	}

	/**
	 * Retrieve a file to a temporary path. This temporary path will be returned. Valid protocol
	 * types: ftp, http, package or local file If a file has already be downloaded (the file is
	 * existing in tmp directory) it will not be redownloaded again. Simply the path to this file
	 * will be returned.
	 * 
	 * @param url
	 *            URL to retrieve
	 * @return NULL on error. On success the path to the file is returned.
	 */
	public static File retrieve(String url) {
		return retrieve(url, true);
	}

	/**
	 * Retrieve a file to a temporary path. This temporary path will be returned. Valid protocol
	 * types: ftp, http, package or local file If a file has already be downloaded (the file is
	 * existing in tmp directory) it will not be redownloaded again. Simply the path to this file
	 * will be returned.
	 * 
	 * @param url
	 *            URL to retrieve
	 * @param checkAlreadyRetrieved
	 *            if false, always download the file and ignore, if it is already existing
	 * @return NULL on error. On success the path to the file is returned.
	 */
	public static File retrieve(String url, boolean checkAlreadyRetrieved) {
		if (url.indexOf("://") <= 0) {
			// Is local file
			return new File(url);
		}
		int start = url.indexOf('/') + 2;
		int end = url.indexOf('/', start);
		String serverName = url.substring(start, end);
		if (url.startsWith("package://")) {
			String filePath = url.substring(end + 1);
			String pkgPath = findPackage(serverName);
			if (pkgPath == null)
				return null;
			return new File(pkgPath, filePath);
		} else if (url.startsWith("http://")) {

			OutputStream out = null;
			URLConnection conn = null;
			InputStream in = null;
			String filename = url.substring(url.lastIndexOf('/') + 1);

			File tmpPath = getTmpName(url, filename);

			if (checkAlreadyRetrieved && tmpPath.exists()) {
				return tmpPath;
			}

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
					// Ignore if stream not possible to close
				}
			}
			return tmpPath;
		} else if (url.startsWith("ftp://")) {
			FTPClient client = new FTPClient();
			OutputStream outStream = null;
			String filename = url.substring(url.lastIndexOf('/') + 1);

			File tmpPath = getTmpName(url, filename);

			if (checkAlreadyRetrieved && tmpPath.exists()) {
				System.out
						.println("Already retrieved: " + url + " to " + tmpPath.getAbsolutePath());
				return tmpPath;
			}

			try {
				// Connect to the FTP server as anonymous
				client.connect(serverName);
				client.login("anonymous", "knowrob@example.com");
				String remoteFile = url.substring(end);
				// Write the contents of the remote file to a FileOutputStream
				outStream = new FileOutputStream(tmpPath);
				client.retrieveFile(remoteFile, outStream);

			} catch (IOException ioe) {
				System.out.println("ResourceRetriever: Error communicating with FTP server: "
						+ serverName + "\n" + ioe.getMessage());
			} finally {
				try {
					outStream.close();
				} catch (IOException e1) {
					// Ignore if stream not possible to close
				}
				try {
					client.disconnect();
				} catch (IOException e) {
					System.out.println("ResourceRetriever: Problem disconnecting from FTP server: "
							+ serverName + "\n" + e.getMessage());
				}
			}
			return tmpPath;
		}
		return null;
	}
}
