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
#include <charconv>

#define APPEND_BSON_PL_PAIR(list,key,value,type) ((PlTail*)list)->append(PlCompound("-", \
		PlTermv(PlTerm(key), PlCompound(type, PlTerm(value)))))

static PlAtom ATOM_Pos_Infinity("Infinity");
static PlAtom ATOM_Neg_Infinity("-Infinity");

static bool bson_visit_double(const bson_iter_t *iter, const char *key, double v_double, void *data)
{
	APPEND_BSON_PL_PAIR(data,key,v_double,"double");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_decimal128(const bson_iter_t *iter, const char *key, const bson_decimal128_t *v_decimal128, void *data)
{
	PlTail *out_list = (PlTail*)data;
	PlTermv v_pl(2);
	v_pl[0] = PlTerm(key);
	// positive infinity
	if(v_decimal128->high == 0x7800000000000000) {
		v_pl[1] = PlCompound("double", PlTerm(ATOM_Pos_Infinity));
	}
	// negative infinity
	else if(v_decimal128->high == 0xf800000000000000) {
		v_pl[1] = PlCompound("double", PlTerm(ATOM_Neg_Infinity));
	}
	else {
		char buffer[BSON_DECIMAL128_STRING];
		bson_decimal128_to_string(v_decimal128,buffer);
		setlocale(LC_NUMERIC,"C");
		v_pl[1] = PlCompound("double", PlTerm(atof(buffer)));
		setlocale(LC_NUMERIC,"");
	}
	out_list->append(PlCompound("-",v_pl));
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_int32(const bson_iter_t *iter, const char *key, int32_t v_int32, void *data)
{
	APPEND_BSON_PL_PAIR(data,key,(long)v_int32,"int");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_int64(const bson_iter_t *iter, const char *key, int64_t v_int64, void *data)
{
	APPEND_BSON_PL_PAIR(data,key,(long)v_int64,"int");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_oid(const bson_iter_t *iter, const char *key, const bson_oid_t *v_oid, void *data)
{
	char str[25];
	bson_oid_to_string(v_oid, str);
	APPEND_BSON_PL_PAIR(data,key,str,"id");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_bool(const bson_iter_t *iter, const char *key, bool v_bool, void *data)
{
	APPEND_BSON_PL_PAIR(data,key,(long)v_bool,"bool");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_utf8(const bson_iter_t *iter, const char *key, size_t v_utf8_len, const char *v_utf8, void *data)
{
	APPEND_BSON_PL_PAIR(data,key,v_utf8,"string");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_date_time(const bson_iter_t *iter, const char *key, int64_t msec_since_epoch, void *data)
{
	double sec_since_epoch = ((double)msec_since_epoch)/1000.0;
	APPEND_BSON_PL_PAIR(data,key,sec_since_epoch,"double");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_array(const bson_iter_t *iter, const char *key, const bson_t *v_array, void *data)
{
	PlTermv av(1);
	PlTail pl_array(av[0]);
	bson_iter_t array_iter;
	if(bson_iter_init(&array_iter, v_array)) {
		while(bson_iter_next(&array_iter)) {
			// TODO: support other array types
			pl_array.append(PlCompound("string",
					PlTerm(bson_iter_utf8(&array_iter,NULL))));
		}
	}
	pl_array.close();
	APPEND_BSON_PL_PAIR(data,key,av[0],"array");
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_document(const bson_iter_t *iter, const char *key, const bson_t *v_document, void *data)
{
	PlTail *out_list = (PlTail*)data;
	out_list->append(PlCompound("-",
			PlTermv(PlTerm(key), bson_to_term(v_document))));
	return false; // NOTE: returning true stops further iteration of the document
}

static bson_visitor_t get_bson_visitor()
{
	bson_visitor_t visitor = {0};
	visitor.visit_double     = bson_visit_double;
	visitor.visit_decimal128 = bson_visit_decimal128;
	visitor.visit_int32      = bson_visit_int32;
	visitor.visit_int64      = bson_visit_int64;
	visitor.visit_bool       = bson_visit_bool;
	visitor.visit_oid        = bson_visit_oid;
	visitor.visit_utf8       = bson_visit_utf8;
//	visitor.visit_timestamp  = bson_visit_timestamp;
	visitor.visit_date_time  = bson_visit_date_time;
	visitor.visit_array      = bson_visit_array;
	visitor.visit_document   = bson_visit_document;
	return visitor;
}

PlTerm bson_to_term(const bson_t *doc)
{
	static bson_visitor_t visitor = get_bson_visitor();
	bson_iter_t iter;
	PlTerm term;
	PlTail out_list(term);
	if (bson_iter_init(&iter, doc)) {
		bson_iter_visit_all(&iter, &visitor, &out_list);
	}
	out_list.close();
	return term;
}

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
