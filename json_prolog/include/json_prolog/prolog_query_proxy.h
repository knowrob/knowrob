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

#ifndef __JSON_PROLOG_QUERY_H__
#define __JSON_PROLOG_QUERY_H__

#include <string>
#include <stdexcept>

#include <boost/iterator/iterator_facade.hpp>

#include <json_prolog/prolog_bindings.h>

namespace json_prolog
{

class Prolog;

class PrologQueryProxy
{
public:
  class QueryError
    : public std::runtime_error
  {
  public:
    QueryError(const std::string &msg)
      : std::runtime_error(msg) {}
  };

  class ServerNotFound
    : public std::runtime_error
  {
  public:
    ServerNotFound(const std::string &msg)
      : std::runtime_error(msg) {}
  };
  
  class iterator
    : public boost::iterator_facade<iterator,
                                    PrologBindings,
                                    boost::single_pass_traversal_tag>
  {
    friend class PrologQueryProxy;
    friend class boost::iterator_core_access;
    
  public:
    iterator()
      : query_(0) {}
    iterator(const iterator &src)
      : query_(src.query_), data_(src.data_) {}

  private:
    PrologQueryProxy *query_;
    std::list<PrologBindings>::iterator data_;

    iterator(PrologQueryProxy &query)
      : query_(&query), data_(query.bindings_.begin()) {}
    
    void increment();
    bool requestNextSolution();
    bool equal(const iterator &other) const;
    PrologBindings &dereference() const { return *data_; }
  };

  PrologQueryProxy(Prolog &prolog, const std::string &query_str);

  iterator begin();
  iterator end() const { return iterator(); }
  void finish();

private:
  // We support only single traversal. This variable is used to throw
  // an error when begin is executed more than once.
  bool finished_;
  Prolog *prolog_;
  std::string query_id_;
  std::list<PrologBindings> bindings_;
  
  static std::string makeQueryId();
};

}

#endif
