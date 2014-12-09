package org.knowrob.json_prolog.query;

public class HasMoreSolutionsCommand extends QueryCommand {
	@Override
	public Object execute(jpl.Query query) {
		if(!query.isOpen()) query.open();
		return new Boolean(query.hasMoreSolutions());
	}
}
