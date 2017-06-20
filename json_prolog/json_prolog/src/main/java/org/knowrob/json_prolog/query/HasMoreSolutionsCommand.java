package org.knowrob.json_prolog.query;

public class HasMoreSolutionsCommand extends QueryCommand {
	@Override
	public Object execute(org.jpl7.Query query) {
		if(!query.isOpen()) query.open();
		return new Boolean(query.hasMoreSolutions());
	}
}
