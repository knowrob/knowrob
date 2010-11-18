package edu.tum.cs.util.db;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

public class MySQLConnection {
	protected Connection conn; 
	
	public MySQLConnection(String host, String user, String password, String database) throws InstantiationException, IllegalAccessException, ClassNotFoundException, SQLException {
		Class.forName("com.mysql.jdbc.Driver").newInstance();
		String port = "3306";
		String connectString = "jdbc:mysql://" + host + ":" + port + "/" + database + "?user=" + user + "&password=" + password;
		conn = DriverManager.getConnection(connectString);	
	}
	
	public void execute(String query) throws SQLException {
		Statement statement = conn.createStatement();
		statement.execute(query);
	}
	
	public ResultSet select(String query) throws SQLException {
		Statement statement = conn.createStatement();
		ResultSet rs = statement.executeQuery(query);
		return rs;
	}
}
