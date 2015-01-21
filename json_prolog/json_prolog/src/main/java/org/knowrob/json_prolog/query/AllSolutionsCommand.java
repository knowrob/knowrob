package org.knowrob.json_prolog.query;

public class AllSolutionsCommand extends QueryCommand {
	@Override
	public Object execute(jpl.Query query) {
		return query.allSolutions();
	}
}
