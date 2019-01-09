/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_motionAnalyzer_IDL
#define YARP_THRIFT_GENERATOR_motionAnalyzer_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class motionAnalyzer_IDL;


/**
 * motionAnalyzer_IDL
 * IDL Interface to Motion Analyzer services.
 */
class motionAnalyzer_IDL : public yarp::os::Wire {
public:
  motionAnalyzer_IDL();
  /**
   * Load motion repertoire from file.
   * @return true/false on success/failure.
   */
  virtual bool loadMotionList();
  /**
   * Load metric to analyze.
   * @param metric_tag name of the metric to analyze
   * @return true/false on failure.
   */
  virtual bool loadMetric(const std::string& metric_tag);
  /**
   * Get the type of motion.
   * @return string containing the type of motion / empty string on failure.
   */
  virtual std::string getMotionType();
  /**
   * List available metrics.
   * @return the list of the available metrics as defined in the motion-repertoire.
   */
  virtual std::vector<std::string>  listMetrics();
  /**
   * Start processing.
   * @return true/false on success/failure.
   */
  virtual bool start();
  /**
   * Stop processing.
   * @return true/false on success/failure.
   */
  virtual bool stop();
  /**
   * Select skeleton by its tag.
   * @param skel_tag tag of the skeleton to process
   * @return true/false on success/failure.
   */
  virtual bool selectSkel(const std::string& skel_tag);
  /**
   * List joints on which feedback is computed.
   * @return the list of joints on which feedback is computed.
   */
  virtual std::vector<std::string>  listJoints();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
