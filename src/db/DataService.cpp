/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/DataService.h"

using namespace knowrob;

DataService::DataService(const URI &uri, std::string_view format)
	: DataSource(uri, format)
{
}

/*
static rasqal_query_results*
roqet_call_sparql_service(rasqal_world* world, raptor_uri* service_uri,
                          const unsigned char* query_string,
                          raptor_sequence* data_graphs,
                          const char* format)
{
  rasqal_service* svc;
  rasqal_query_results* results;

  svc = rasqal_new_service(world, service_uri, query_string,
                           data_graphs);
  if(!svc) {
    fprintf(stderr, "%s: Failed to create service object\n", program);
    return NULL;
  }

  rasqal_service_set_format(svc, format);

  results = rasqal_service_execute(svc);

  rasqal_free_service(svc);

  return results;
}
 */

 /*
        results = roqet_call_sparql_service(world, service_uri, query_string,
                                            data_graphs,  NULL);
  */
