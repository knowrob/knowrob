package org.knowrob.json_prolog.query;

public class AllSolutionsCommand extends QueryCommand {
	@Override
	public Object execute(jpl.Query query) {
		if(query.isOpen()) {
			// Query must be closed before allSolutions() called
			query.close();
		}
		return query.allSolutions();
	}
}
