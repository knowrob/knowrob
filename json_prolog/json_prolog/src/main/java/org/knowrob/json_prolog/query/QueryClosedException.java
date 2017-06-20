package org.knowrob.json_prolog.query;

import org.jpl7.Query;

public class QueryClosedException extends Exception {
	private static final long serialVersionUID = -2345651829510699915L;
	
	public QueryClosedException(Query query) {
		super("The query was closed during execution.");
	}
}
