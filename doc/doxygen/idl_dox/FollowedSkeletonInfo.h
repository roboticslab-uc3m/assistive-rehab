/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_FollowedSkeletonInfo
#define YARP_THRIFT_GENERATOR_STRUCT_FollowedSkeletonInfo

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class FollowedSkeletonInfo;


/**
 * FollowedSkeletonInfo
 * IDL structure to send info on followed skeleton.
 */
class FollowedSkeletonInfo : public yarp::os::idl::WirePortable {
public:
  // Fields
  /**
   * the tag of the followed skeleton; empty otherwise.
   */
  std::string tag;
  /**
   * the x-coordinate.
   */
  double x;
  /**
   * the y-coordinate.
   */
  double y;
  /**
   * the z-coordinate.
   */
  double z;

  // Default constructor
  FollowedSkeletonInfo() : tag(""), x(0), y(0), z(0) {
  }

  // Constructor with field values
  FollowedSkeletonInfo(const std::string& tag,const double x,const double y,const double z) : tag(tag), x(x), y(y), z(z) {
  }

  // Copy constructor
  FollowedSkeletonInfo(const FollowedSkeletonInfo& __alt) : WirePortable(__alt)  {
    this->tag = __alt.tag;
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
  }

  // Assignment operator
  const FollowedSkeletonInfo& operator = (const FollowedSkeletonInfo& __alt) {
    this->tag = __alt.tag;
    this->x = __alt.x;
    this->y = __alt.y;
    this->z = __alt.z;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader) override;
  bool read(yarp::os::ConnectionReader& connection) override;
  bool write(const yarp::os::idl::WireWriter& writer) const override;
  bool write(yarp::os::ConnectionWriter& connection) const override;

private:
  bool write_tag(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_tag(const yarp::os::idl::WireWriter& writer) const;
  bool write_x(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_x(const yarp::os::idl::WireWriter& writer) const;
  bool write_y(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_y(const yarp::os::idl::WireWriter& writer) const;
  bool write_z(const yarp::os::idl::WireWriter& writer) const;
  bool nested_write_z(const yarp::os::idl::WireWriter& writer) const;
  bool read_tag(yarp::os::idl::WireReader& reader);
  bool nested_read_tag(yarp::os::idl::WireReader& reader);
  bool read_x(yarp::os::idl::WireReader& reader);
  bool nested_read_x(yarp::os::idl::WireReader& reader);
  bool read_y(yarp::os::idl::WireReader& reader);
  bool nested_read_y(yarp::os::idl::WireReader& reader);
  bool read_z(yarp::os::idl::WireReader& reader);
  bool nested_read_z(yarp::os::idl::WireReader& reader);

public:

  std::string toString() const;

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<FollowedSkeletonInfo > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new FollowedSkeletonInfo;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(FollowedSkeletonInfo& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(FollowedSkeletonInfo& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }

    virtual ~Editor() {
    if (obj_owned) delete obj;
    }

    bool isValid() const {
      return obj!=0/*NULL*/;
    }

    FollowedSkeletonInfo& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_tag(const std::string& tag) {
      will_set_tag();
      obj->tag = tag;
      mark_dirty_tag();
      communicate();
      did_set_tag();
    }
    void set_x(const double x) {
      will_set_x();
      obj->x = x;
      mark_dirty_x();
      communicate();
      did_set_x();
    }
    void set_y(const double y) {
      will_set_y();
      obj->y = y;
      mark_dirty_y();
      communicate();
      did_set_y();
    }
    void set_z(const double z) {
      will_set_z();
      obj->z = z;
      mark_dirty_z();
      communicate();
      did_set_z();
    }
    const std::string& get_tag() {
      return obj->tag;
    }
    double get_x() {
      return obj->x;
    }
    double get_y() {
      return obj->y;
    }
    double get_z() {
      return obj->z;
    }
    virtual bool will_set_tag() { return true; }
    virtual bool will_set_x() { return true; }
    virtual bool will_set_y() { return true; }
    virtual bool will_set_z() { return true; }
    virtual bool did_set_tag() { return true; }
    virtual bool did_set_x() { return true; }
    virtual bool did_set_y() { return true; }
    virtual bool did_set_z() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection) override;
    bool write(yarp::os::ConnectionWriter& connection) const override;
  private:

    FollowedSkeletonInfo *obj;

    bool obj_owned;
    int group;

    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_tag() {
      if (is_dirty_tag) return;
      dirty_count++;
      is_dirty_tag = true;
      mark_dirty();
    }
    void mark_dirty_x() {
      if (is_dirty_x) return;
      dirty_count++;
      is_dirty_x = true;
      mark_dirty();
    }
    void mark_dirty_y() {
      if (is_dirty_y) return;
      dirty_count++;
      is_dirty_y = true;
      mark_dirty();
    }
    void mark_dirty_z() {
      if (is_dirty_z) return;
      dirty_count++;
      is_dirty_z = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_tag = flag;
      is_dirty_x = flag;
      is_dirty_y = flag;
      is_dirty_z = flag;
      dirty_count = flag ? 4 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_tag;
    bool is_dirty_x;
    bool is_dirty_y;
    bool is_dirty_z;
  };
};

#endif
