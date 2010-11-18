package edu.tum.cs.util.db;

import java.sql.Connection;
import java.sql.ResultSet;
import java.sql.ResultSetMetaData;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.HashMap;

/**
 * a class containing some static functions related to databases that may come in handy 
 * @author Dominik Jain
 */
public class DatabaseTool {
	/**
	 * fetches a row from the given result set (by calling the next() member function) and
	 * converts it to a HashMap<String,String> (each key being a column name)
	 * @param rs			the result set
	 * @return				a corresponding HashMap or null if there are no more rows in the 
	 * 						result set
	 * @throws Exception	if there is a null value in the result row 
	 */
	public static HashMap<String,String> resultSetRow2HashMap(ResultSet rs) throws Exception {
		HashMap<String,String> hm = new HashMap<String,String>();
		boolean erroneous = false;
		try {			
			ResultSetMetaData rsmd = rs.getMetaData();
			int numCols = rsmd.getColumnCount();			
			if(rs.next()) {				
				for(int i = 1; i <= numCols; i++) {
					String val = rs.getString(i);
					if(val == null)
						erroneous = true;
					hm.put(rsmd.getColumnName(i), val);
				}
			}
			else
				return null;
		}
		catch(SQLException e) {
			e.printStackTrace();
		}
		if(erroneous)
			throw new Exception("Result set contains null entry!");
		return hm;
	}

	/**
	 * returns the first column of the first row in the result set for a given query
	 * @param conn		the database connection
	 * @param query		the query to execute
	 * @return			the first column of the first row in the result set
	 * @throws SQLException
	 */
	public static String queryResult(Connection conn, String query) throws SQLException {
		Statement stmt = conn.createStatement();
		ResultSet rs = stmt.executeQuery(query);
		rs.next();
		return rs.getString(1);
	}
}
