/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/bson_pl.h"
#include <sstream>
#include <iostream>

bool get_boolean_(const PlTerm &atomic_value)
{
	std::string val(atomic_value.name());
	if(val.compare("true")==0) {
		return true;
	}
	else if(val.compare("false")==0) {
		return false;
	}
	else {
		return (bool)((int)atomic_value);
	}
}

bson_t *bson_new_from_term(const PlTerm &term, bson_error_t *err)
{
	PlTail list(term);
	PlTerm member;
	if(!list.next(member)) {
		return NULL;
	}
	if(PL_is_list((term_t)member)) {
		bson_t *out = bson_new();
		do {
			bson_t *member_bson = bson_new_from_term(member,err);
			if(!member_bson)
				return NULL;
			bson_concat(out,member_bson);
			bson_destroy(member_bson);
		} while(list.next(member));
		return out;
	}
	else {
		const char *key = (char*)member;
		list.next(member);
		if(PL_is_list((term_t)member)) {
			bson_t *nested_bson = bson_new_from_term(member,err);
			if(!nested_bson)
				return NULL;
			bson_t *ret = bson_new();
			BSON_APPEND_DOCUMENT(ret, key, nested_bson);
			bson_destroy(nested_bson);
			return ret;
		}
		else {
			std::string type_name(member.name());
			const PlTerm &atomic_value = member[1];
			//
			if(type_name.compare("int")==0)
				return BCON_NEW(key, BCON_INT32((int)atomic_value));
			else if(type_name.compare("bool")==0)
				return BCON_NEW(key, BCON_BOOL(get_boolean_(atomic_value)));
			else if(type_name.compare("double")==0) {
				// NOTE: must use canonical form for "Infinity"
				bson_decimal128_t dec;
				bson_decimal128_from_string ((char*)atomic_value, &dec);
				return BCON_NEW(key, BCON_DECIMAL128(&dec));
			}
			else if(type_name.compare("id")==0) {
				bson_oid_t oid;
				bson_oid_init_from_string (&oid, (char*)atomic_value);
				return BCON_NEW(key, BCON_OID(&oid));
			}
			else if(type_name.compare("regex")==0)
				return BCON_NEW(key, BCON_REGEX((char*)atomic_value,"msi"));
			else if(type_name.compare("string")==0)
				return BCON_NEW(key, BCON_UTF8((char*)atomic_value));
			else if(type_name.compare("time")==0) {
				unsigned long long time = (unsigned long long)(1000.0*((double)atomic_value));
				return BCON_NEW(key, BCON_DATE_TIME(time));
			}
			else if(type_name.compare("list")==0) {
				PlTail list2(atomic_value);
				PlTerm member;
				bson_t *list_bson = bson_new();
				bson_t child;
				int counter=0;
				BSON_APPEND_ARRAY_BEGIN(list_bson, key, &child);
				while(list2.next(member)) {
					bson_t *member_bson = bson_new_from_term(member,err);
					if(member_bson==NULL) {
						bson_destroy(list_bson);
						return NULL;
					}
					BSON_APPEND_DOCUMENT(&child, std::to_string(counter).c_str(), member_bson);
					bson_destroy(member_bson);
					counter+=1;
				}
				bson_append_array_end(list_bson, &child);
				return list_bson;
			}
			else if(type_name.compare("array")==0) {
				PlTail list2(atomic_value);
				PlTerm member;
				bson_t *list_bson = bson_new();
				long counter=0;
				while(list2.next(member)) {
					PlTerm member_term;
					PlTail l(member_term);
					l.append(counter);
					l.append(member);
					l.close();
					//
					bson_t *member_bson = bson_new_from_term(member_term,err);
					if(member_bson==NULL) {
						bson_destroy(list_bson);
						return NULL;
					}
					bson_concat(list_bson,member_bson);
					bson_destroy(member_bson);
					counter+=1;
				}
				bson_t *out = BCON_NEW(key, BCON_ARRAY(list_bson));
				bson_destroy(list_bson);
				return out;
			}
			else {
				bson_set_error(err,
					MONGOC_ERROR_BSON,
					MONGOC_ERROR_BSON_INVALID,
					"invalid type: %s", type_name.c_str()
				);
				return NULL;
			}
		}
	}
}
