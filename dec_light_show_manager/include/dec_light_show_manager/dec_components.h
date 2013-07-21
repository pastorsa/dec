/*
 * dec_components.h
 *
 *  Created on: Jul 14, 2013
 *      Author: pastor
 */

#ifndef DEC_COMPONENTS_H_
#define DEC_COMPONENTS_H_

#include <vector>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <dec_utilities/assert.h>
#include <dec_utilities/param_server.h>

namespace dec_light_show_manager
{

// ===================================================================
// Connector
// ===================================================================

class Component
{
  friend class DecStructure;

public:
  Component() {};
  virtual ~Component() {};

  void setIds(const unsigned int id);

  geometry_msgs::Pose getPose(const unsigned int local_index) const
  {
    ROS_ASSERT_MSG(local_index < poses_.size(), "Invalid local index requested >%i< when getting pose. Should be less then >%i<.",
                   (int)local_index, (int)poses_.size());
    return poses_[local_index];
  }
  geometry_msgs::Point getPosition(const unsigned int local_index) const
  {
    ROS_ASSERT_MSG(local_index < poses_.size(), "Invalid local index requested >%i< when getting position. Should be less then >%i<.",
                   (int)local_index, (int)poses_.size());
    return poses_[local_index].position;
  }

  unsigned int getId(const unsigned int local_index) const
  {
    ROS_ASSERT_MSG(local_index < poses_.size(), "Invalid local index requested >%i< when getting id. Should be less then >%i<.",
                   (int)local_index, (int)poses_.size());
    ROS_ASSERT(local_index < poses_.size());
    return ids_[local_index];
  }
  unsigned int getNumComponents() const
  {
    ROS_WARN_COND(poses_.empty(), "No components setup.");
    return poses_.size();
  }

protected:

  std::vector<geometry_msgs::Pose> poses_;
  std::vector<unsigned int> ids_;

  bool read(XmlRpc::XmlRpcValue& rpc_values,
            const std::string& key,
            std::vector<std::pair<unsigned int, unsigned int> >& nodes);

  bool read(XmlRpc::XmlRpcValue& rpc_values,
            const std::string& key,
            std::pair<unsigned int, unsigned int>& node_pair);

private:

};

class Connector
{

  friend class Sensor;
  friend class LightNode;
  friend class LightBeam;

public:
  Connector() :
   teensy_(0), pin_(0) {};
  virtual ~Connector() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);

  unsigned int getTeeynsyId() const
  {
    return teensy_;
  }

  void setPin(const unsigned int pin)
  {
    ROS_ASSERT(pin < 255);
    pin_ = pin;
  }
  unsigned int getPin() const
  {
    return pin_;
  }

protected:
  unsigned int teensy_;
  unsigned int pin_;

private:

};

// ===================================================================
// Node
// ===================================================================

class Node : public Component
{
public:
  Node() {};
  virtual ~Node() {};

  bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id);

  void update(const geometry_msgs::Point& point)
  {
    ROS_ASSERT(poses_.size() == 1);
    poses_[0].position = point;
  }

protected:

};

// ===================================================================
// Beam
// ===================================================================

class Beam : public Component
{
public:
  Beam() {};
  virtual ~Beam() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                          const std::vector<geometry_msgs::Point>& node_positions);

  std::vector<std::pair<unsigned int, unsigned int> > nodes_;

protected:

  void setPoses(const unsigned int local_index,
                const std::vector<geometry_msgs::Point>& node_positions,
                const float center = 0.5);
private:

};


// ===================================================================
// Sensor
// ===================================================================

class Sensor : public Beam, public Connector
{
public:
  Sensor() {};
  virtual ~Sensor() {};
  /*! Pairs of nodes in node_positions_ (indexed from 0)
   */

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                          const std::vector<geometry_msgs::Point>& node_positions);

  geometry_msgs::Point getAvgPosition() const
  {
    return avg_position_;
  }

protected:
  geometry_msgs::Point avg_position_;

private:
  void setAvgPose();

};

// ===================================================================
// Light
// ===================================================================

class Light: public Connector
{
public:
  Light() : num_leds_per_meter_(-1) {};
  virtual ~Light() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config);

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                          const std::vector<geometry_msgs::Point>& node_positions) = 0;

  unsigned int getTotalNumLeds() const;

  unsigned int getNumLeds(const unsigned int local_index) const
  {
    ROS_ASSERT_MSG(local_index < num_leds_.size(), "Invalid local index requested >%i< when getting number of LEDs. Should be less then >%i<.",
                   (int)local_index, (int)num_leds_.size());
    return num_leds_[local_index];
  }

protected:
  std::vector<unsigned int> num_leds_;
  int num_leds_per_meter_;
};

// ===================================================================
// LightNode
// ===================================================================

class LightNode : public Node, public Light
{

public:
  LightNode() {};
  virtual ~LightNode() {};

  bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                  const std::vector<geometry_msgs::Point>& node_positions);

  std::vector<unsigned int> nodes_;

protected:

private:

};

// ===================================================================
// LightBeam
// ===================================================================

class LightBeam : public Beam, public Light
{
public:
  LightBeam() {};
  virtual ~LightBeam() {};

  virtual bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                          const std::vector<geometry_msgs::Point>& node_positions);

protected:
  // std::vector<std::pair<int, int> > nodes_;
  std::vector<float> centers_;
  std::vector<float> length_;

private:
  void setLightBeamPoses(const unsigned int local_index,
                         const std::vector<geometry_msgs::Point>& node_positions);

};

// ===================================================================
// PixelLightBeam
// ===================================================================

class PixelLightBeam : public LightBeam
{
public:
  PixelLightBeam() {};
  virtual ~PixelLightBeam() {};

  bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                  const std::vector<geometry_msgs::Point>& node_positions);

  std::vector<std::vector<geometry_msgs::Pose> > pixel_poses_;

protected:

private:
  void setPixelPoses(const unsigned int local_index,
                     const std::vector<geometry_msgs::Point>& node_positions);

};

// ===================================================================
// BlockLightBeam
// ===================================================================

class BlockLightBeam : public LightBeam
{
public:
  BlockLightBeam() {};
  virtual ~BlockLightBeam() {};

  bool initialize(XmlRpc::XmlRpcValue& config, const unsigned int id,
                  const std::vector<geometry_msgs::Point>& node_positions);
private:

};

}

#endif /* DEC_COMPONENTS_H_ */
