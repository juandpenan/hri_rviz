// Copyright 2021 PAL Robotics S.L.
// Copyright 2012, Willow Garage, Inc.
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

#ifndef RVIZ_HRI_HUMANS_HPP
#define RVIZ_HRI_HUMANS_HPP

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>
#include <cv_bridge/cv_bridge.h>
#include "hri_msgs/msg/ids_list.hpp"
#include <hri_msgs/msg/normalized_point_of_interest2_d.hpp>

#include <QObject>
#include <map>
#include <string>
#include <iostream>
#include <vector>

#include "hri/hri.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "image_transport/image_transport.hpp"
#include "image_transport/subscriber_filter.hpp"
#include "rviz_common/ros_topic_display.hpp"
// #include "rviz/image/image_display_base.h"
#include "rviz_common/message_filter_display.hpp"
// #include "rviz/image/ros_image_texture.h"
#include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
// #include "rviz/properties/bool_property.h"
#include "rviz_common/properties/bool_property.hpp"
// #include "rviz/properties/float_property.h"
# include "rviz_common/properties/float_property.hpp"
// #include "rviz/properties/int_property.h"
#include "rviz_common/properties/int_property.hpp"
// #include "rviz/render_panel.h"
#include "rviz_common/render_panel.hpp"
#include "hri_rviz/image_transport_display.hpp"
#endif
using std::placeholders::_1;


namespace Ogre
{
class SceneNode;
class Rectangle2D;
}  // namespace Ogre

namespace rviz
{
class HumansDisplay : public rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>
{
  Q_OBJECT

public:
  explicit HumansDisplay(
    std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture);
  HumansDisplay();
  ~HumansDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  virtual void updateNormalizeOptions();
  void updateShowFaces();
  void updateShowBodies();
  void updateShowFacialLandmarks();
  void updateShowSkeletons();
  void setupRenderPanel();
  void setupScreenRectangle();
  // void subscribe();
  // void unsubscribe();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  // skeleton drawing function
  void drawSkeleton(
    std::string id, int width, int height,
    std::vector<hri_msgs::msg::NormalizedPointOfInterest2D> & skeleton);

  /* This is called by incomingMessage(). */
  void processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg);

  // void incomingMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  // template <typename MessageType>
  // void incomingMessage(const MessageType msg);
  // bool isRawTransport(const std::string & topic);

  // Ogre::SceneManager* img_scene_manager_;


  // ros::NodeHandle nh_;

private:
  void clear();
  // Ogre::SceneNode* img_scene_node_;
  std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
  Ogre::MaterialPtr material_;
  std::unique_ptr<rviz_common::RenderPanel> render_panel_;


  // message_filters::Connection subscription_callback_;


  std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture_;

  rviz_common::properties::BoolProperty * normalize_property_;
  rviz_common::properties::BoolProperty * show_faces_property_;
  rviz_common::properties::BoolProperty * show_facial_landmarks_property_;
  rviz_common::properties::BoolProperty * show_bodies_property_;
  rviz_common::properties::BoolProperty * show_skeletons_property_;
  rviz_common::properties::FloatProperty * min_property_;
  rviz_common::properties::FloatProperty * max_property_;
  rviz_common::properties::IntProperty * median_buffer_size_property_;
  bool got_float_image_;
  bool show_faces_, show_facial_landmarks_, show_bodies_, show_skeletons_;
  // auto hri_listener = std::make_shared<hri::HRIListener>();
  hri::HRIListener hri_listener;
  cv_bridge::CvImageConstPtr cvBridge_;
};

}  // namespace rviz

#endif
