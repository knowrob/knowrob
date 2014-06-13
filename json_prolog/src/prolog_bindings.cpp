/* 
 * Copyright (c) 2010, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

#include <json-glib/json-glib.h>

#include <json_prolog/prolog_bindings.h>

namespace json_prolog
{

namespace detail
{

class InitJsonGlib
{
public:
  InitJsonGlib()
  {
    g_type_init();
  }
};

static InitJsonGlib init_glib;

static PrologValue parseJSONValue(JsonNode *);

static std::vector<PrologValue> parseJSONArrayList(GList *nodes)
{
  std::vector<PrologValue> result(g_list_length(nodes));

  for(int i=0;nodes; i++, nodes=g_list_next(nodes))
    result[i] = parseJSONValue((JsonNode *)nodes->data);
  return result;
}
 
static PrologValue parseJSONTerm(JsonObject *obj)
{
  JsonArray *arr = json_object_get_array_member(obj, "term");
  if(!arr)
    throw PrologBindings::JSONParseError("Malformed term. Value for key `term' invalid.");

  if(json_array_get_length(arr) < 2)
    throw PrologBindings::JSONParseError("Malformed term. Not enought arguments");

  const gchar *functor = json_array_get_string_element(arr, 0);

  if(!functor)
    throw PrologBindings::JSONParseError("Functor name invalid.");
  
  return PrologValue(
    PrologTerm(
      functor,
      parseJSONArrayList(g_list_next(json_array_get_elements(arr)))));
}

static PrologValue parseJSONArray(JsonArray *arr)
{
  return PrologValue(parseJSONArrayList(json_array_get_elements(arr)));
}

static PrologValue parseJSONSimpleValue(JsonNode *node)
{
  switch(json_node_get_value_type(node))
  {
    case G_TYPE_STRING:
      return PrologValue(json_node_get_string(node));
    case G_TYPE_DOUBLE:
      return PrologValue(json_node_get_double(node));
    case G_TYPE_INT64:
      return PrologValue(json_node_get_int(node));
    default:
      throw PrologBindings::JSONParseError("Unsupported value base type.");
  }
}

static PrologValue parseJSONValue(JsonNode *node)
{
  switch(json_node_get_node_type(node))
  {
    case JSON_NODE_OBJECT:
      return parseJSONTerm(json_node_get_object(node));
    case JSON_NODE_ARRAY:
      return parseJSONArray(json_node_get_array(node));
    case JSON_NODE_VALUE:
      return parseJSONSimpleValue(node);
    default:
      throw PrologBindings::JSONParseError("Unsupported value of type NULL.");
  }
}

}

PrologBindings PrologBindings::parseJSONBindings(const std::string &json_bdgs)
{
  JsonParser *parser = json_parser_new();
  GError *error;

  if(!json_parser_load_from_data(parser, json_bdgs.c_str(), json_bdgs.size(), &error))
  {
    std::string msg(error->message);
    g_error_free(error);
    throw JSONParseError(msg);
  }

  try
  {

    JsonNode *root = json_parser_get_root(parser);
    if(!root)
      throw JSONParseError("No JSON root node found.");

    if(json_node_get_node_type(root) != JSON_NODE_OBJECT)
      throw JSONParseError("Malformed bindings.");

    PrologBindings result;

    JsonObject *root_obj = json_node_get_object(root);
    GList *keys = json_object_get_members(root_obj);
    for(; keys; keys = g_list_next(keys))
    {
      std::string var_name = reinterpret_cast<char *>(keys->data);
      result.bdgs_.insert(
        std::make_pair(
          var_name,
          detail::parseJSONValue(json_object_get_member(root_obj, (const gchar *)keys->data))));
    }
    return result;
  }
  catch(std::exception &e)
  {
    g_object_unref(parser);
    throw e;
  }
}

const PrologValue &PrologBindings::operator[](const std::string &var_name) const
{
  std::map<std::string, PrologValue>::const_iterator it=bdgs_.find(var_name);
  if( it == bdgs_.end() )
    throw VariableUnbound(var_name);
  return it->second;
}

}
