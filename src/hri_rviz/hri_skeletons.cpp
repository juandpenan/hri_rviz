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

#include "hri_rviz/hri_skeletons.hpp"
#include "std_msgs/msg/string.hpp"
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_default_plugins/robot/robot.hpp"
#include "rviz_default_plugins/robot/robot_link.hpp"
#include "rviz_default_plugins/robot/tf_link_updater.hpp"
#include <urdf/model.h>

#include "rclcpp/rclcpp.hpp"

#include <QTimer>


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rviz {

HumansModelDisplay::HumansModelDisplay()
    : has_new_transforms_(false), time_since_last_transform_(0.0f) {

  ids_sub_ =
        rviz_ros_node_.lock()->get_raw_node()->create_subscription<hri_msgs::msg::IdsList>(
        "/humans/bodies/tracked",
        1,
        std::bind(&HumansModelDisplay::idsCallback, this, _1));


  pluginEnabled_ = true;
}

HumansModelDisplay::~HumansModelDisplay() {
  if (initialized()) {
    for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
         it != humans_.end(); it++)
      it->second = nullptr;
  }
}


void HumansModelDisplay::onInitialize() {
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
}

void HumansModelDisplay::updateAlpha() {
  for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void HumansModelDisplay::updateVisualVisible() {
  for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisualVisible(visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void HumansModelDisplay::updateCollisionVisible() {
  for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisualVisible(
          collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void HumansModelDisplay::updateTfPrefix() {
  clearStatuses();
  context_->queueRender();
}

void HumansModelDisplay::initializeRobot(
    std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it) {
  context_->queueRender();

  if (robot_descriptions_.empty())
  {
    return;
  }


 for (const auto& content : robot_descriptions_)
 {
    load_urdf_from_string(content);
    robot_description_ = content;
    display_urdf_content();
 }
}

void HumansModelDisplay::onEnable() {
  for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisible(true);
  pluginEnabled_ = true;
}

void HumansModelDisplay::onDisable() {
  // robot_->setVisible(false);
  for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisible(false);
  pluginEnabled_ = false;
  //clear();
}

void HumansModelDisplay::update(float wall_dt, float /*ros_dt*/) {
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  if (has_new_transforms_ || update) {
    for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin(); it != humans_.end(); it++){

      if(it->second != nullptr){
        updateRobot();
        context_->queueRender();

        has_new_transforms_ = false;
        time_since_last_transform_ = 0.0f;
        // it->second->update(TFLinkUpdater(
        //     context_->getFrameManager(),
        //     std::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
        //     tf_prefix_property_->getStdString()));
      }
    }
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void HumansModelDisplay::fixedFrameChanged() { has_new_transforms_ = true; }

void HumansModelDisplay::clear() {
  // robot_->clear();
  for (std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->clear();
  clearStatuses();
  robot_description_.clear();
}

void HumansModelDisplay::reset() {
  Display::reset();
  has_new_transforms_ = true;
}

void HumansModelDisplay::idsCallback(const hri_msgs::msg::IdsList::ConstSharedPtr msg) {
  if (pluginEnabled_) {
    ids_ = msg->ids;

    // Check for bodies that are no more in the list
    // Remove them from the map0

    std::map<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>::iterator itH;
    for (itH = humans_.begin(); itH != humans_.end();) {
      if (std::find(ids_.begin(), ids_.end(), itH->first) == ids_.end()) {
        if(itH->second)
          itH->second = nullptr;
        humans_.erase((itH++)->first);
      } else
        ++itH;
    }

    // Check for new faces
    // Create a bounding box message and insert it in the map

    for (const auto& id : ids_) {

        // auto sub_ =
        rviz_ros_node_.lock()->get_raw_node()->create_subscription<std_msgs::msg::String>(
        "/humans/bodies/" + id + "/robot_description",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),      
        [this](const typename std_msgs::msg::String::ConstSharedPtr msg) {robot_descriptions_.push_back(msg->data);});

        if(!robot_descriptions_.empty()){
          auto ins = humans_.insert(std::pair<std::string, std::unique_ptr<rviz_default_plugins::robot::Robot>>(id, nullptr));
          if (ins.second) initializeRobot(ins.first); // Maybe I could remove this

        }
    }
  }
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::HumansModelDisplay, rviz_common::Display)
