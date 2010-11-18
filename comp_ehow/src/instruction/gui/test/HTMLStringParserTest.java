package instruction.gui.test;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.util.EntityUtils;


public class HTMLStringParserTest {

	String title = "";
	List<String> instructions = new ArrayList<String>();
	String url = null;
	
	
	
	public String[] search_wikihow( String query ) {

		String url = "http://www.wikihow.com/Special:LSearch?search=" + query.replaceAll(" ", "\\+");

		try {

			HttpClient httpclient = new DefaultHttpClient();
			HttpGet httpget = new HttpGet(url);
			
			String page = EntityUtils.toString(httpclient.execute(httpget).getEntity());
			String[] rows = page.split("\n");
			
			
			String[] res = new String[2];
			for ( int i = 0; i < rows.length; i++ ) {
				
				String p = "(<div class='searchresult_1'><a href=\"([a-zA-Z:./_\\-]*)\">([a-zA-Z <>/]*)</a>)";
				Matcher matcher = Pattern.compile(p).matcher(rows[i]);
				
				if(matcher.find()) {
					
					res[0] = matcher.group(2);
					res[1] = matcher.group(3).replaceAll("<[a-zA-z0-9/]*>", "");

					return res;
				}
			}
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
		return null;
	}

	
	public void read(String url) {


		try {
			HttpClient httpclient = new DefaultHttpClient();
			HttpGet httpget = new HttpGet(url);
			
			String page = EntityUtils.toString(httpclient.execute(httpget).getEntity());
			String[] rows = page.split("\n");
		
			System.out.println("Steps:");
			for ( int i = 0; i < rows.length; i++ ) {

				String p = "(<b class='whb'>([0-9A-Za-z \\._]*)</b>\\.)";
				Matcher matcher = Pattern.compile(p).matcher(rows[i]);
				
				if(matcher.find()) {
					
					String u = matcher.group(2);
					System.out.println("* " + u);

				}
			}
			
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	
	public static void main(String[] args ) {
		
		HTMLStringParserTest t = new HTMLStringParserTest();
		
		String[] result = t.search_wikihow("Make Pancakes Using Pancake Mix");
		
		if(result!=null)
			t.read(result[1]);
	}
	
}
