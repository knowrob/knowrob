package org.knowrob.json_prolog.query;

import org.jpl7.Query;

public class ResetCommand extends QueryCommand {
	@Override
	public Object execute(Query query) {
		query.close();
		return new Boolean(true);
	}
}
