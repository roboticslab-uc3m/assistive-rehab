/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_attentionManager_IDL
#define YARP_THRIFT_GENERATOR_attentionManager_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <src/FollowedSkeletonInfo.h>

class attentionManager_IDL;


/**
 * attentionManager_IDL
 * IDL Interface to Attention Manager services.
 */
class attentionManager_IDL : public yarp::os::Wire {
public:
  attentionManager_IDL();
  /**
   * Look at the specified skeleton.
   * @param tag the tag of the skeleton to look at.
   * @param keypoint the keypoint of the skeleton to look at.
   * @return true/false on success/failure.
   */
  virtual bool look(const std::string& tag, const std::string& keypoint = "head");
  /**
   * Stop any ongoing actions.
   * @return true/false on success/failure.
   */
  virtual bool stop();
  /**
   * Check if any action is being performed.
   * @return true/false on running/stationary.
   */
  virtual bool is_running();
  /**
   * Check if the robot is following a skeleton.
   * @return info on the followed skeleton in \ref FollowedSkeletonInfo format.
   */
  virtual FollowedSkeletonInfo is_following();
  /**
   * Check if any skeleton is with one hand raised.
   * @return the list of skeletons' tags.
   */
  virtual std::vector<std::string>  is_any_raised_hand();
  /**
   * Check if the specified skeleton is with one hand raised.
   * @param tag the tag of the skeleton.
   * @return true/false on success/failure.
   */
  virtual bool is_with_raised_hand(const std::string& tag);
  /**
   * Enable autonomous mode.
   * @return true/false on success/failure.
   */
  virtual bool set_auto();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
