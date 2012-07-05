package edu.tum.cs.ias.knowrob.comp_barcoo;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Vector;

import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class CategoryGetterThread extends Thread {

	public boolean done;
	public String barcode;
	private Document doc;
	
	public CategoryGetterThread(String barcode)
	{
		this.barcode = barcode;
		done = false;
	}
	
	public void run()
	{
		try
		{
			File barcooFile = null;			
			BufferedWriter bw = null;
			URL url;							
			
			barcooFile = new File( barcode + ".xml");
			bw = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(barcooFile)));
			barcooFile.createNewFile();
			
			String link = "http://www.barcoo.com/api/get_product_complete?pi=" + barcode + "&amp;pins=ean&amp;format=xml&amp;source=ias-tum";
			
			System.out.println(link);
			url = new URL(link);
			String content = getPage(url);
			if(content !=null && !content.isEmpty())
			{
				bw.write(content);
				bw.flush();
				
				//Processing part
				int response = checkCategory(barcooFile);
				
				if(response == 0)
				{
					//Save the fact that it is ROOT
					BarcooCategoryTester.incrementRoot();
					BarcooCategoryTester.writeBarcode(barcode);
				}
				if(response == 1)
					//save the fact that it is not root
					BarcooCategoryTester.incrementNotRoot();
				
				bw.close();
				barcooFile.delete();
			}
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
		finally
		{
			done = true;
			
		}
	}
	
	//Retrieves XML page
	public static String getPage(URL url) throws Exception
	{
		String line = null;
		String file = "";
		HttpURLConnection connection = (HttpURLConnection) url.openConnection();
		connection.setRequestProperty("User-Agent", "Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:12.0) Gecko/20100101 Firefox/12.0");
		connection.connect();
		
		BufferedReader br = new BufferedReader(new InputStreamReader(connection.getInputStream()));
		if(connection.getInputStream() == null)
			return null;
		
		while((line = br.readLine()) != null)
			file += line;
		
		return file;
	}
	
	//Check if the category is ROOT or something else
	protected int checkCategory(File file) throws Exception
	{				
		doc = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file);
		//get the root element
		Element docEle = doc.getDocumentElement();
		//get a nodelist of elements
		NodeList l = docEle.getElementsByTagName("answer");
		Node n = l.item(0);
		n = n.getFirstChild();		
		if(n.getNodeValue() == null || n.getNodeValue() == "0")
			return -1;
		
		NodeList nl = docEle.getElementsByTagName("category_key");
		String category = nl.item(0).getFirstChild().getNodeValue();
		if(category.equalsIgnoreCase("ROOT"))
			return 0;
		return 1;
			
		
	}
}
