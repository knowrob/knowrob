/* 
 * Copyright (c) 2020, Daniel Beßler
 * All rights reserved.
 * 
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reasoner/mongolog/bson_pl.h"
#include "knowrob/backend/mongo/MongoException.h"
#include <sstream>
#include <iostream>

// NOTE: returning true would stop further iteration of the document
#define APPEND_BSON_PL_PAIR(list,key,value,type) ((PlTail*)list)->append(PlCompound("-", \
		PlTermv(PlTerm(key), PlCompound(type, PlTerm(value))))) && false

static bool bson_visit_double(const bson_iter_t *iter, const char *key, double v_double, void *data)
{
	return APPEND_BSON_PL_PAIR(data,key,v_double,"double");
}

static bool bson_visit_int32(const bson_iter_t *iter, const char *key, int32_t v_int32, void *data)
{
	return APPEND_BSON_PL_PAIR(data,key,(long)v_int32,"int");
}

static bool bson_visit_int64(const bson_iter_t *iter, const char *key, int64_t v_int64, void *data)
{
	return APPEND_BSON_PL_PAIR(data,key,(long)v_int64,"int");
}

static bool bson_visit_oid(const bson_iter_t *iter, const char *key, const bson_oid_t *v_oid, void *data)
{
	char str[25];
	bson_oid_to_string(v_oid, str);
	return APPEND_BSON_PL_PAIR(data,key,str,"id");
}

static bool bson_visit_bool(const bson_iter_t *iter, const char *key, bool v_bool, void *data)
{
    static PlAtom ATOM_true("true");
    static PlAtom ATOM_false("false");

	if(v_bool) {
		return APPEND_BSON_PL_PAIR(data,key,ATOM_true,"bool");
	}
	else {
		return APPEND_BSON_PL_PAIR(data,key,ATOM_false,"bool");
	}
}

static bool bson_visit_utf8(const bson_iter_t *iter, const char *key, size_t v_utf8_len, const char *v_utf8, void *data)
{
	return APPEND_BSON_PL_PAIR(data,key,v_utf8,"string");
}

static bool bson_visit_date_time(const bson_iter_t *iter, const char *key, int64_t msec_since_epoch, void *data)
{
	double sec_since_epoch = ((double)msec_since_epoch)/1000.0;
	return APPEND_BSON_PL_PAIR(data,key,sec_since_epoch,"time");
}

static bool bson_visit_decimal128(const bson_iter_t *iter, const bson_decimal128_t *v_decimal128, void *data)
{
    static PlAtom ATOM_Neg_Infinity("-Infinity");
    static PlAtom ATOM_Pos_Infinity("Infinity");

    auto *v_pl = (PlTail*)data;
	// positive infinity
	if(v_decimal128->high == 0x7800000000000000) {
		v_pl->append(PlCompound("double", PlTerm(ATOM_Pos_Infinity)));
	}
	// negative infinity
	else if(v_decimal128->high == 0xf800000000000000) {
		v_pl->append(PlCompound("double", PlTerm(ATOM_Neg_Infinity)));
	}
	else {
		char buffer[BSON_DECIMAL128_STRING];
		bson_decimal128_to_string(v_decimal128,buffer);
		setlocale(LC_NUMERIC,"C");
		v_pl->append(PlCompound("double", PlTerm(atof(buffer))));
		setlocale(LC_NUMERIC,"");
	}
	return false; // NOTE: returning true stops further iteration of the document
}

static bool bson_visit_decimal128(const bson_iter_t *iter, const char *key, const bson_decimal128_t *v_decimal128, void *data)
{
    static PlAtom ATOM_Neg_Infinity("-Infinity");
    static PlAtom ATOM_Pos_Infinity("Infinity");

    auto *out_list = (PlTail*)data;
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

static bool bson_iter_append_array(bson_iter_t *iter, PlTail *pl_array)
{
    static PlAtom ATOM_true("true");
    static PlAtom ATOM_false("false");

	if(BSON_ITER_HOLDS_UTF8(iter)) {
		pl_array->append(PlCompound("string",
				PlTerm(bson_iter_utf8(iter, nullptr))));
	}
	else if(BSON_ITER_HOLDS_DOCUMENT(iter)) {
		const uint8_t *data = nullptr;
		uint32_t len = 0;
		bson_iter_document(iter, &len, &data);
		bson_t *doc = bson_new_from_data(data, len);
		pl_array->append(bson_to_term(doc));
	}
	else if(BSON_ITER_HOLDS_ARRAY(iter)) {
		const uint8_t *array = nullptr;
		uint32_t array_len = 0;
		bson_iter_array(iter, &array_len, &array);
		bson_t *inner_doc = bson_new_from_data(array, array_len);
		bson_iter_t inner_iter;
		PlTerm inner_term;
		PlTail inner_array(inner_term);
		if(bson_iter_init(&inner_iter, inner_doc)) {
			while(bson_iter_next(&inner_iter)) {
				bson_iter_append_array(&inner_iter,&inner_array);
			}
		}
		inner_array.close();
		pl_array->append(PlCompound("array",inner_term));
	}
	else if(BSON_ITER_HOLDS_DOUBLE(iter)) {
		pl_array->append(PlCompound("double",
				PlTerm(bson_iter_double(iter))));
	}
	else if(BSON_ITER_HOLDS_DECIMAL128(iter)) {
		bson_decimal128_t v_decimal128;
		bson_iter_decimal128(iter, &v_decimal128);
		bson_visit_decimal128(iter, &v_decimal128, pl_array);
	}
	else if(BSON_ITER_HOLDS_INT32(iter)) {
		pl_array->append(PlCompound("int",
				PlTerm((long)bson_iter_int32(iter))));
	}
	else if(BSON_ITER_HOLDS_INT64(iter)) {
		pl_array->append(PlCompound("int",
				PlTerm((long)bson_iter_int32(iter))));
	}
	else if(BSON_ITER_HOLDS_BOOL(iter)) {
		if(bson_iter_bool(iter)) {
			pl_array->append(PlCompound("bool", PlTerm(ATOM_true)));
		}
		else {
			pl_array->append(PlCompound("bool", PlTerm(ATOM_false)));
		}
	}
	else if(BSON_ITER_HOLDS_DATE_TIME(iter)) {
		double sec_since_epoch = ((double)bson_iter_date_time(iter))/1000.0;
		pl_array->append(PlCompound("time", PlTerm(sec_since_epoch)));
	}
	else {
		bson_type_t iter_t = bson_iter_type(iter);
		std::cout << "WARN: unsupported array type '" << iter_t << "'" << std::endl;
		return false;
	}
	return true;
}

static bool bson_visit_array(const bson_iter_t *iter, const char *key, const bson_t *v_array, void *data)
{
	PlTermv av(1);
	PlTail pl_array(av[0]);
	bson_iter_t array_iter;
	if(bson_iter_init(&array_iter, v_array)) {
		while(bson_iter_next(&array_iter)) {
			bson_iter_append_array(&array_iter, &pl_array);
		}
	}
	pl_array.close();
	return APPEND_BSON_PL_PAIR(data,key,av[0],"array");
}

static bool bson_visit_document(const bson_iter_t *iter, const char *key, const bson_t *v_document, void *data)
{
	auto *out_list = (PlTail*)data;
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
		if(bson_iter_visit_all(&iter, &visitor, &out_list)) {
			bson_error_t err;
			bson_set_error(&err,
				MONGOC_ERROR_BSON,
				MONGOC_ERROR_BSON_INVALID,
				"BSON iteration prematurely stopped.");
			throw knowrob::mongo::MongoException("bson", err);
		}
	}
	out_list.close();
	return term;
}

static bool bsonpl_append_typed(bson_t *doc, const char *key, const PlTerm &term, bson_error_t *err)
{
    static PlAtom ATOM_array("array");
    static PlAtom ATOM_time("time");
    static PlAtom ATOM_id("id");
    static PlAtom ATOM_regex("regex");
    static PlAtom ATOM_double("double");
    static PlAtom ATOM_bool("bool");
    static PlAtom ATOM_string("string");
    static PlAtom ATOM_integer("integer");
    static PlAtom ATOM_int("int");
    static PlAtom ATOM_true("true");
    static PlAtom ATOM_false("false");

	const PlAtom type_atom(term.name());
	const PlTerm &pl_value = term[1];
	if(type_atom == ATOM_string) {
		BSON_APPEND_UTF8(doc, key, (char*)pl_value);
	}
	else if(type_atom == ATOM_id) {
		bson_oid_t oid;
		bson_oid_init_from_string(&oid, (char*)pl_value);
		BSON_APPEND_OID(doc,key,&oid);
	}
	else if(type_atom == ATOM_double) {
		// NOTE: must use canonical form for "Infinity"
		bson_decimal128_t dec;
		bson_decimal128_from_string ((char*)pl_value, &dec);
		BSON_APPEND_DECIMAL128(doc,key,&dec);
	}
	else if(type_atom == ATOM_time) {
		BSON_APPEND_DATE_TIME(doc,key,(unsigned long long)(1000.0*((double)pl_value)));
	}
	else if(type_atom == ATOM_int || type_atom == ATOM_integer) {
		BSON_APPEND_INT32(doc,key,(int)pl_value);
	}
	else if(type_atom == ATOM_bool) {
		if(pl_value == ATOM_true) {
			BSON_APPEND_BOOL(doc,key,true);
		}
		else if(pl_value == ATOM_false) {
			BSON_APPEND_BOOL(doc,key,false);
		}
		else {
			BSON_APPEND_BOOL(doc,key,(bool)((int)pl_value));
		}
	}
	else if(type_atom == ATOM_regex) {
		BSON_APPEND_REGEX(doc,key,(char*)pl_value,"msi");
	}
	else if(type_atom == ATOM_array) {
		PlTail pl_list(pl_value);
		PlTerm pl_member;
		bson_t bson_array;
		int counter=0;
		const char *index_key;
		char index_str[16];
		bool status=true;

		BSON_APPEND_ARRAY_BEGIN(doc, key, &bson_array);
		while(pl_list.next(pl_member)) {
			bson_uint32_to_string(counter, &index_key, index_str, sizeof index_str);
			if(!bsonpl_append(&bson_array,index_key,pl_member,err)) {
				status=false;
				break;
			}
			counter+=1;
		}
		bson_append_array_end(doc, &bson_array);
		return status;
	}
	else {
		bson_set_error(err,
			MONGOC_ERROR_BSON,
			MONGOC_ERROR_BSON_INVALID,
			"invalid type: %s", (char*)term
		);
		return false;
	}
	return true;
}

bool bsonpl_append(bson_t *doc, const char *key, const PlTerm &term, bson_error_t *err)
{
	if(PL_is_list((term_t)term)) {
		// append a document if term is a list
		bson_t *nested_doc = bson_new();
		if(bsonpl_concat(nested_doc, term, err)) {
			BSON_APPEND_DOCUMENT(doc, key, nested_doc);
			bson_destroy(nested_doc);
			return true;
		}
		else {
			bson_destroy(nested_doc);
			return false;
		}
	}
	else if(std::string_view(term.name()) == "bson_t") {
	    // the term contains a sub-pipeline encoded as bson_t* object which can be concatenated
	    // with the pipeline under construction
	    if(term.arity()!=1) {
            bson_set_error(err,
                MONGOC_ERROR_BSON,
                MONGOC_ERROR_BSON_INVALID,
                "invalid bson_t term -- arity should be 1: %s", (char*)term
            );
            return false;
	    }
        const PlTerm &bson_value = term[1];
	    auto *sub_bson = (bson_t*)bson_value.operator void *();
	    if(sub_bson) {
            bool success = bson_concat(doc, sub_bson);
            bson_destroy(sub_bson);
            return success;
	    }
	    else {
            bson_set_error(err,
                MONGOC_ERROR_BSON,
                MONGOC_ERROR_BSON_INVALID,
                "invalid bson_t term -- void pointer is null: %s", (char*)term
            );
            return false;
	    }
	}
	else {
		// append typed value otherwise
		return bsonpl_append_typed(doc,key,term,err);
	}
}

bool bsonpl_concat(bson_t *doc, const PlTerm &term, bson_error_t *err)
{
	// the term given to bson_from_term must be a list
	PlTail list(term);
	PlTerm member;
	if(!list.next(member)) {
		// the term is a list []
		return true;
	}
	else if(PL_is_list((term_t)member)) {
		// the term is a list [[..]|_]
		do {
			if(!bsonpl_concat(doc,member,err)) {
				return false;
			}
		} while(list.next(member));
		return true;
	}
	else {
		// the term is a list [key,value]
		const char *key = (char*)member;
		list.next(member);
		return bsonpl_append(doc,key,member,err);
	}
}
