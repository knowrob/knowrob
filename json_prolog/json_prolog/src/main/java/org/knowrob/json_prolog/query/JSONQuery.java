/* 
 * Copyright (c) 2010, Lorenz Moesenlechner
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package org.knowrob.json_prolog.query;

import net.sf.json.*;
import net.sf.json.util.JSONUtils;
import org.jpl7.JPLException;
import org.jpl7.Term;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class JSONQuery {

  static public class InvalidJSONQuery extends Exception {
    public InvalidJSONQuery() {
    }

    public InvalidJSONQuery(String msg) {
      super(msg);
    }

    private static final long serialVersionUID = 1L;
  }

  static public ThreadedQuery makeQuery(String json) throws InvalidJSONQuery {
    try {
      JSONArray json_query = JSONArray.fromObject(json);

      return new ThreadedQuery(makeCompoundTerm(json_query));
    } catch (Exception e) {
      throw new InvalidJSONQuery("Unable to parse JSON query: " + json + ", Exception: " + e.toString());
    }
  }

  
  static public JSONObject encodeResult(Map<java.lang.String,org.jpl7.Term> bindings) {
    JSONObject result = new JSONObject();

    Iterator<Map.Entry<String, org.jpl7.Term>> it = bindings.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, org.jpl7.Term> e = it.next();

      if (e.getValue().isFloat())
        result.put(e.getKey(), e.getValue().doubleValue());
      else if (e.getValue().isInteger())
        result.put(e.getKey(), e.getValue().intValue());
      else if (e.getValue().isCompound() && !e.getValue().isAtom()) {
        if (isList(e.getValue()))
          result.put(e.getKey(), encodeList(e.getValue()));
        else
          result.put(e.getKey(), encodeCompoundTerm(e.getValue()));
      }
      else
        result.put(e.getKey(), e.getValue().name());
    }
    return result;
  }
	
  static Term makeCompoundTerm(JSONArray val) throws InvalidJSONQuery {
    if (val.isEmpty())
      throw new InvalidJSONQuery("makeCompoundTerm: empty sequence.");

    String predicate = val.getString(0);
    if (predicate == null)
      throw new InvalidJSONQuery("makeCompoundTerm: no predicate given.");

    String[] split_predicate = predicate.split(":");
    if(split_predicate.length == 1) {
      if(val.size() > 1)
        return new org.jpl7.Compound(predicate, decodeJSONArray(val, 1));
      else
        return new org.jpl7.Atom(predicate);
    }
    else if(split_predicate.length == 2) {
      if(val.size() > 1)
        return new org.jpl7.Compound(":", new Term[] {
            new org.jpl7.Atom(split_predicate[0]), 
            new org.jpl7.Compound(split_predicate[1], decodeJSONArray(val, 1)) });
      else
        return new org.jpl7.Compound(":", new Term[] {
            new org.jpl7.Atom(split_predicate[0]), 
            new org.jpl7.Atom(split_predicate[1]) });
    }
    else
      throw new InvalidJSONQuery("Predicate encoding wrong. Found more than one ':' in " + predicate);
  }

  public static org.jpl7.Term[] decodeJSONArray(JSONArray val) throws InvalidJSONQuery {
    return decodeJSONArray(val, 0);
  }

  static private org.jpl7.Term[] decodeJSONArray(JSONArray val, int startIndex)
      throws InvalidJSONQuery {
    if(startIndex >= val.size())
      return new org.jpl7.Term[0];

    org.jpl7.Term[] terms = new org.jpl7.Term[val.size() - startIndex];

    for (int i = startIndex; i < val.size(); i++) {
      if (JSONUtils.isString(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getString(i));
      else if (JSONUtils.isBoolean(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getBoolean(i));
      else if (JSONUtils.isDouble(val.get(i).getClass()))
        terms[i - startIndex] = decodeJSONValue(val.getDouble(i));
      else if (JSONUtils.isNumber(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getInt(i));
      else if (JSONUtils.isObject(val.get(i)))
        terms[i - startIndex] = decodeJSONValue(val.getJSONObject(i));
      else if (JSONUtils.isArray(val.get(i)))
    	terms[i - startIndex] = org.jpl7.Util.termArrayToList(decodeJSONArray(val.getJSONArray(i)));
      else
        throw new InvalidJSONQuery("decodeJSONArray: unable to parse "
            + val.toString());
    }
    return terms;
  }

  static org.jpl7.Term decodeJSONValue(String val) {

	  // wrap string in single quotes if it contains special characters
	  val = val.replace("\\", "");
	  Matcher matcher = Pattern.compile("^[\\w_]*$").matcher(val);
	  if(!matcher.find()) {
		  val="'"+val+"'";
	  }
	 return org.jpl7.Util.textToTerm(val);
  }

  static org.jpl7.Term decodeJSONValue(boolean val) {
    // Hack. No idea if we need to serialize bools and how.
    return new org.jpl7.Integer(val ? 1 : 0);
  }

  static org.jpl7.Term decodeJSONValue(double val) {
    return new org.jpl7.Float(val);
  }

  static org.jpl7.Term decodeJSONValue(int val) {
    return new org.jpl7.Integer(val);
  }

  public static org.jpl7.Term decodeJSONValue(JSONObject val) throws InvalidJSONQuery {
    if (val.has("variable"))
      return new org.jpl7.Variable(val.getString("variable"));
    else if (val.has("term"))
      return makeCompoundTerm(val.getJSONArray("term"));
    else if (val.has("list")) {
      JSONArray json_elements = val.getJSONArray("list");
      return org.jpl7.Util.termArrayToList(decodeJSONArray(json_elements));
    } else
      throw new InvalidJSONQuery("decodeJSONValue: unable to parse "
          + val.toString());
  }
  
  static private JSONObject encodeCompoundTerm(Term term) {
    JSONObject result = new JSONObject();
    JSONArray term_list = new JSONArray();

    term_list.element(term.name());
    JSONArray args_list = encodeList(term.args());

    for (int i = 0; i < args_list.size(); i++)
      term_list.element(args_list.get(i));

    result.put("term", term_list);
    return result;
  }

  static private JSONArray encodeList(Term term) throws JPLException {
    Term[] term_list = term.toTermArray();
    return encodeList(term_list);
  }

  static private JSONArray encodeList(Term[] term_list) throws JPLException {
    JSONArray result = new JSONArray();
    for (int i = 0; i < term_list.length; i++) {
      if (term_list[i].isFloat())
        result.element(term_list[i].doubleValue());
      else if (term_list[i].isInteger())
        result.element(term_list[i].intValue());
      else if (term_list[i].isAtom())
        result.element(term_list[i].name());
      else if (term_list[i].isCompound()) {
        if (isList(term_list[i]))
          result.element(encodeList(term_list[i]));
        else
          result.element(encodeCompoundTerm(term_list[i]));
      } else
        result.element(term_list[i].name());
    }

    return result;
  }

  static private boolean isList(Term term) {
    try {
      term.listLength();
      return true;
    } catch (org.jpl7.JPLException e) {
      return false;
    }
  }

}	// 
