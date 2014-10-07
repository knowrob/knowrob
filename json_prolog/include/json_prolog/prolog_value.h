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

#ifndef __JSON_PROLOG_VALUE_H__
#define __JSON_PROLOG_VALUE_H__

#include <string>
#include <vector>

#include <boost/any.hpp>

namespace json_prolog
{

class PrologTerm;

class PrologValue
{
public:
  typedef enum { DOUBLE, INT, STRING, LIST, TERM, EMPTY } value_type;
  PrologValue()
    : type_(EMPTY) {}
  PrologValue(double value)
    : value_(value), type_(DOUBLE) {}
  PrologValue(int value)
    : value_(static_cast<int64_t>(value)), type_(INT) {}
  PrologValue(int64_t value)
    : value_(value), type_(INT) {}
  PrologValue(const std::string &value)
    : value_(value), type_(STRING) {}
  PrologValue(const PrologTerm &value)
    : value_(value), type_(TERM) {}
  PrologValue(const std::vector<PrologValue> &value)
    : value_(value), type_(LIST) {}

  template<typename T>
  const T &as() const { return *boost::any_cast<T>(&value_); }

  template<typename T>
  operator T() const { return boost::any_cast<T>(value_); }

  value_type type() const { return type_; }
  bool isDouble() const { return type_ == DOUBLE; }
  bool isInt() const { return type_ == INT; }
  bool isString() const { return type_ == STRING; }
  bool isTerm() const { return type_ == TERM; }
  bool isList() const { return type_ == LIST; }
  bool isValid() const { return type_ != EMPTY; }

  std::string toString() const;

private:
  boost::any value_;
  value_type type_;
};

class PrologTerm
{
public:
  PrologTerm(const std::string &name, const std::vector<PrologValue> &values)
    : name_(name), values_(values) {}
  
  const std::string &name() const { return name_; }
  const std::vector<PrologValue> &values() const { return values_; }
  int arity() const { return values_.size(); }

  const PrologValue &operator[](int index) { return values_[index]; }
  operator std::vector<PrologValue>() const { return values_; }

private:
  std::string name_;
  std::vector<PrologValue> values_;
};

std::ostream &operator<<(std::ostream &strm, const PrologValue &v);

}

#endif
