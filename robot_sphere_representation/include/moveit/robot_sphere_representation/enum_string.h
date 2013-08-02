/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Acorn Pooley */

#define ENUM_STRING_EXAMPLE_CODE 0
#if ENUM_STRING_EXAMPLE_CODE

//  H   H   OOO   W   W        TTTTT   OOO        U   U   SSSS  EEEE
//  H   H  O   O  W W W          T    O   O       U   U  S      E
//  HHHHH  O   O  W W W          T    O   O       U   U   SSS   EEE
//  H   H  O   O  WWWWW          T    O   O       U   U      S  E
//  H   H   OOO    W W           T     OOO         UUU   SSSS   EEEEE
//
//
// To define an enum type named MyEnum with members NAME12
// In header file my_enum.h:
//
//     #include <enum_string.h>
//     #define MyEnum__VALUES(VD,VS,VV,VVS,VA) \
//        VS(NAME1, "Name 1") \
//        VS(NAME2, "Name 2") \
//        VS(NAME3, "Name 3") \
//
//     ENUM_STRING_DECLARE(MyEnum, MyEnum__VALUES);
//
// In source file my_enum.cpp:
//
//     #include <my_enum.h>
//     ENUM_STRING_DEFINE(MyEnum, MyEnum__VALUES);
//
// To use it in other places:
//
//     #include <my_enum.h>
//     ...
//     MyEnum e = MyEnum::NAME1;
//
//     std::cout << e.toStr(); // prints "Name 1"
//


//==============================================================
// EXAMPLE HEADER:
//==============================================================

//
// VD(e)       - enum e with next available value and name-string matching enum name
// VS(e,s)     - enum e with next available value and name-string s
// VV(e,v)     - enum e with value v and name-string matching enum name
// VVS(e,v,s)  - enum e with value v and name-string s
// VA(e,v)     - enum e with value v. No name-string.  For aliases (2 names w/ 1 value).

namespace my_namespace
{
#define my_namespace__Example1EnumString__Values(VD,VS,VV,VVS,VA) \
  VD(MODE1) \
  VD(MODE2) \
  VS(MODE3, "Sphere Mode") \
  VS(MODE4, "Cube Mode") \
  VV(MODE5, 98) \
  VV(MODE6, 107) \
  VVS(MODE7, 100, "mode 100") \
  VVS(MODE8, 103, "other mode") \
  VA(DEFAULT,MODE1) \
  VA(SPECIAL,1000) \

ENUM_STRING_DECLARE(Example2Enum,my_namespace__Example1EnumString__Values);
}

//==============================================================
// EXAMPLE SOURCEFILE:
//==============================================================

namespace my_namespace
{
ENUM_STRING_DEFINE(Example2Enum,my_namespace__Example1EnumString__Values);
}


//  EEEE  N   N DDDD     H   H  OOO  W   W     TTTTT  OOO      U   U  SSSS EEEE
//  E     NN  N D   D    H   H O   O W W W       T   O   O     U   U S     E
//  EEE   N N N D   D    HHHHH O   O W W W       T   O   O     U   U  SSS  EEE
//  E     N  NN D   D    H   H O   O WWWWW       T   O   O     U   U     S E
//  EEEEE N   N DDDD     H   H  OOO   W W        T    OOO       UUU  SSSS  EEEEE

#endif // ENUM_STRING_EXAMPLE_CODE





#ifndef MOVEIT_ROBOT_SPHERE_REPRESENTATION_ENUM_STRING_H_
#define MOVEIT_ROBOT_SPHERE_REPRESENTATION_ENUM_STRING_H_

#include <string>
#include <vector>
#include <map>
#include <stdexcept>

namespace enum_string
{

class EnumStringImpl
{
public:
  const std::string& toName(int value) const
  {
    std::map<int, std::string>::const_iterator it = to_name_.find(value);
    return it == to_name_.end() ? default_name_ : it->second;
  }

  int toValue(const std::string& name) const
  {
    std::map<std::string, int>::const_iterator it = to_val_.find(name);
    return it == to_val_.end() ? default_value_ : it->second;
  }

  bool isValid(int value) const
  {
    std::map<int, std::string>::const_iterator it = to_name_.find(value);
    return it != to_name_.end();
  }

  bool isValid(const std::string& name) const
  {
    std::map<std::string, int>::const_iterator it = to_val_.find(name);
    return it != to_val_.end();
  }

  const std::vector<std::string>& getNames() const
  {
    return names_;
  }

  int getDefaultValue() const
  {
    return default_value_;
  }

  const std::string& getDefaultName() const
  {
    return default_name_;
  }

  void setDefaultValue(int value)
  {
    default_value_ = value;
    found_default_ = true;
  }

  void setDefaultName(const std::string& name)
  {
    default_name_ = name;
    found_default_ = true;
  }

  void setDefault(int value)
  {
    default_value_ = value;
    default_name_ = toName(value);
    found_default_ = true;
  }

  void setDefault(const std::string& name)
  {
    default_name_ = name;
    default_value_ = toValue(name);
    found_default_ = true;
  }

protected:
  EnumStringImpl(int default_value = -1,
                 const std::string default_name = "")
    : default_value_(default_value)
    , default_name_(default_name)
    , found_default_(false)
  {}

  void insert(const std::string& name, int value)
  {
    to_val_.insert(std::pair<std::string, int>(name, value));
    to_name_.insert(std::pair<int, std::string>(value, name));
    names_.push_back(name);
    if (!found_default_)
    {
      default_value_ = value;
      default_name_ = name;
      found_default_ = true;
    }
  }

private:
  int default_value_;           // defaults to first value inserted
  std::string default_name_;    // defaults to first value inserted
  bool found_default_;

  // map values to strings and back
  std::map<std::string, int> to_val_;
  std::map<int, std::string> to_name_;
  std::vector<std::string> names_;
};

#if 0
// code similar to this is generated by ENUM_STRING_DECLARE for each enum.
class EnumStringImplHolder
{
public:
  typedef EnumStringImpl ImplType;
  static const EnumStringImpl& getImpl() { return impl_; }
  static EnumStringImpl impl_;
};
#endif

// thrown by the set methods.
class EnumStringValueException : public std::invalid_argument
{
public:
  explicit EnumStringValueException(const std::string& what_arg)
    : invalid_argument(what_arg)
  {}
};

template<class ImplHolder>
class EnumString
{
public:
  EnumString() : value_(getImpl().getDefaultValue()) {}
  explicit EnumString(int value) : value_(value) {}
  explicit EnumString(const std::string& name) : value_(getImpl().toValue(name)) {}

  operator int() const { return value_; }

  const std::string& toName() const
  {
    return getImpl().toName(value_);
  }

  int toValue() const
  {
    return value_;
  }

  bool isValid() const
  {
    return getImpl().isValid(value_);
  }

  // set to value, but throw on failure.
  void set(int value)
  {
    if (!getImpl().isValid(value))
      throw EnumStringValueException("Enum out of range");
    value_ = value;
  }

  // set to string, but throw on failure.
  void set(const std::string& name)
  {
    int value = getImpl().toValue(name);
    if (value == getImpl().getDefaultValue() && !getImpl().isValid(name))
      throw EnumStringValueException("name is not in enum");
    value_ = value;
  }


  static const std::string& toName(int value)
  {
    return getImpl().toName(value);
  }

  static int toValue(const std::string& name)
  {
    return getImpl().toValue(name);
  }

  static bool isValid(int value)
  {
    return getImpl().isValid(value);
  }

  static bool isValid(const std::string& name)
  {
    return getImpl().isValid(name);
  }

  static const std::vector<std::string>& getNames()
  {
    return getImpl().getNames();
  }

  static int getDefaultValue()
  {
    return getImpl().getDefaultValue();
  }

  static const std::string& getDefaultName()
  {
    return getImpl().getDefaultName();
  }

  static void setDefaultValue(int value)
  {
    return getImpl().setDefaultValue(value);
  }

  static void setDefaultName(const std::string& name)
  {
    return getImpl().setDefaultName(name);
  }

  static const typename ImplHolder::ImplType& getImpl() { return ImplHolder::getImpl(); }

private:
  int value_;
};



//==============================================================
// MACROS
//==============================================================

// Use this to declare the enum in a header file.
#define ENUM_STRING_DECLARE(name, values) \
  class name; \
  class name##Impl : public enum_string::EnumStringImpl \
  { \
  public: \
    typedef name ValueType; \
    name##Impl(); \
  }; \
  class name##ImplHolder \
  { \
  public: \
    typedef name##Impl ImplType; \
    static const name##Impl& getImpl() { return impl_; } \
    static name##Impl impl_; \
  }; \
  class name : public enum_string::EnumString<name##ImplHolder> \
  { \
  public: \
    enum TheEnum { \
      values(ENUM_STRING_DECLARE_VD, \
             ENUM_STRING_DECLARE_VS, \
             ENUM_STRING_DECLARE_VV, \
             ENUM_STRING_DECLARE_VVS, \
             ENUM_STRING_DECLARE_VA) \
    }; \
    name() : EnumString() {} \
    name(TheEnum value) : EnumString(int(value)) {} \
    explicit name(int value) : EnumString(value) {} \
    explicit name(const std::string& ename) : EnumString(ename) {} \
  }

#define ENUM_STRING_DECLARE_VD(e)      e,
#define ENUM_STRING_DECLARE_VS(e,s)    e,
#define ENUM_STRING_DECLARE_VV(e,v)    e=v,
#define ENUM_STRING_DECLARE_VVS(e,v,s) e=v,
#define ENUM_STRING_DECLARE_VA(e,v)    e=v,


// Use this to define the enum in a .cpp file
#define ENUM_STRING_DEFINE(name, values) \
  inline name##Impl::name##Impl() \
  { \
    values(ENUM_STRING_DEFINE_VD, \
           ENUM_STRING_DEFINE_VS, \
           ENUM_STRING_DEFINE_VV, \
           ENUM_STRING_DEFINE_VVS, \
           ENUM_STRING_DEFINE_VA) \
    setDefaultValue(0); \
  } \
  name##Impl name##ImplHolder::impl_


#define ENUM_STRING_DEFINE_VD(e)      insert(#e, ValueType::e);
#define ENUM_STRING_DEFINE_VS(e,s)    insert(s,  ValueType::e);
#define ENUM_STRING_DEFINE_VV(e,v)    insert(#e, ValueType::e);
#define ENUM_STRING_DEFINE_VVS(e,v,s) insert(s,  ValueType::e);
#define ENUM_STRING_DEFINE_VA(e,v)

}

#endif // MOVEIT_ROBOT_SPHERE_REPRESENTATION_ENUM_STRING_H_
