package instruction.wrapper;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.util.EntityUtils;

public class WikihowWebsiteWrapper implements IHowtoWebsiteWrapper {
	
	String title = "";
	List<String> instructions = new ArrayList<String>();
	String url = null;

	public String getHowtoTitle() {
		return title;
	}

	public List<String> getInstructions() {
		return instructions;
	}


	public void load( String query ) {
		
		String[] howto = search_wikihow( query );
		
		this.url = howto[0];
		this.title = howto[1];
		
		instructions = read(url);
	}

	public String getUrl() {
		return url;
	}

	

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

	
	public ArrayList<String> read(String url) {

		ArrayList<String> res = new ArrayList<String>();
		try {
			HttpClient httpclient = new DefaultHttpClient();
			HttpGet httpget = new HttpGet(url);
			
			String page = EntityUtils.toString(httpclient.execute(httpget).getEntity());
			String[] rows = page.split("\n");
		
			System.out.println("Steps:");
			for ( int i = 0; i < rows.length; i++ ) {

				Matcher matcher = Pattern.compile("(<b class='whb'>([0-9A-Za-z \\._]*)</b>\\.)").matcher(rows[i]);
				
				if(matcher.find()) {

					res.add(matcher.group(2));
				}
			}	
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
		return res;
	}
	
}
