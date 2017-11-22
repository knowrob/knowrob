package org.knowrob.json_prolog.query;

/**
 * Abstract command for jpl7.JPL queries.
 * @author Daniel Beßler
 */
public abstract class QueryCommand {
	public Object result = null;
	public abstract Object execute(org.jpl7.Query query);
}
