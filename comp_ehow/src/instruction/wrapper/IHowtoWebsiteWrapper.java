package instruction.wrapper;

import java.util.List;

/**
 * Interface for Wrapper classes to extract Howtos from a distinct website
 * 
 * @author Daniel Nyga
 *
 */
public interface IHowtoWebsiteWrapper {
	
	/**
	 * Loads the content of the page given by <code>url</code>
	 * @param url
	 */
	public void load(String url);
	
	/**
	 * Returns the title of the Howto loaded, e.g. "set a table"
	 * @return
	 */
	public String getHowtoTitle();
	
	/**
	 * Returns the list of instructions given by the Howto
	 * @return
	 */
	public List<String> getInstructions();
	
	/**
	 * Returns the location of the currently loaded Howto
	 * @return
	 */
	public String getUrl();

}
