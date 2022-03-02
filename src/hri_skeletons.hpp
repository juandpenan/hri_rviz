// Copyright 2021 PAL Robotics S.L.
// Copyright 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc., PAL Robotics S.L. nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RVIZ_HRI_SKELETONS_H
#define RVIZ_HRI_SKELETONS_H

#include <OGRE/OgreVector3.h>
#include <hri_msgs/IdsList.h>
#include <rviz/display.h>

#include <map>

namespace Ogre {
class Entity;
class SceneNode;
}  // namespace Ogre

namespace rviz {
class Axes;
}

namespace rviz {
class FloatProperty;
class Property;
class Robot;
class StringProperty;

typedef std::shared_ptr<Robot> RobotPtr;

/**
 * \class HumansModelDisplay
 * \brief Uses a robot xml description to display the pieces of a robot at the
 * transforms broadcast by rosTF
 */
class HumansModelDisplay : public Display {
  Q_OBJECT
 public:
  HumansModelDisplay();
  ~HumansModelDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void fixedFrameChanged() override;
  void reset() override;
  using Display::load;

  void clear();

 private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateTfPrefix();
  void updateAlpha();
  // void updateRobotDescription();

 protected:
  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models. */
  // virtual void load();

  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  bool has_new_transforms_;  ///< Callback sets this to tell our update
                             ///< function it needs to update the
                             /// transforms

  float time_since_last_transform_;

  std::string robot_description_;

  Property* visual_enabled_property_;
  Property* collision_enabled_property_;
  FloatProperty* update_rate_property_;
  FloatProperty* alpha_property_;
  StringProperty* tf_prefix_property_;

  std::map<std::string, RobotPtr> humans_;

 private:
  std::vector<std::string> ids_;
  ros::Subscriber idsSub_;  // Subscriber for the ids list

  void idsCallback(const hri_msgs::IdsListConstPtr& msg);
  void initializeRobot(std::map<std::string, RobotPtr>::iterator it);
};
  
}  // namespace rviz

#endif
