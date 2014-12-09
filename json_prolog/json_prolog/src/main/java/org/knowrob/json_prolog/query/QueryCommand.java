package org.knowrob.json_prolog.query;

/**
 * Abstract command for JPL queries.
 * @author Daniel Beßler
 */
public abstract class QueryCommand {
	public Object result = null;
	public abstract Object execute(jpl.Query query);
}
