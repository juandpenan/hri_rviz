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

#include "hri_rviz/hri_humans.hpp"

#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <cv_bridge/cv_bridge.h>
#include "hri_msgs/msg/ids_list.hpp"
#include <hri_msgs/msg/normalized_point_of_interest2_d.hpp>
#include <hri_msgs/msg/skeleton2_d.hpp>
#include "image_transport/image_transport.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_rendering/material_manager.hpp"
// #include <rviz/ogre_helpers/compatibility.h>
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture_iface.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <stdlib.h>  // srand, rand
#include <boost/bind/bind.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>

#define SKELETON_POINTS 18

using namespace std;
using namespace boost::placeholders;

cv::Scalar get_color_from_id(std::string id)
{
  hash<string> hasher;
  size_t hash = hasher(id);
  srand(hash);
  cv::Mat3f hsv(cv::Vec3f(rand() % 360, 0.7, 0.8));
  cv::Mat3f bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return cv::Scalar(bgr(0, 0) * 255);
}

int clip(int n, int lower, int upper)
{
  return std::max(lower, std::min(n, upper));
}

namespace rviz
{
HumansDisplay::HumansDisplay()
: HumansDisplay(std::make_unique<rviz_default_plugins::displays::ROSImageTexture>()) {}
HumansDisplay::HumansDisplay(
  std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture)
: texture_(std::move(texture))
{
  normalize_property_ = new rviz_common::properties::BoolProperty(
    "Normalize Range",
    true,
    "If set to true, will try to estimate the range of possible values from the received images.",
    this,
    SLOT(updateNormalizeOptions()));

  min_property_ = new rviz_common::properties::FloatProperty(
    "Min Value",
    0.0,
    "Value which will be displayed as black.",
    this,
    SLOT(updateNormalizeOptions()));

  max_property_ = new rviz_common::properties::FloatProperty(
    "Max Value",
    1.0,
    "Value which will be displayed as white.",
    this,
    SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ = new rviz_common::properties::IntProperty(
    "Median window",
    5,
    "Window size for median filter used for computing min/max.",
    this,
    SLOT(updateNormalizeOptions()));

  show_faces_property_ = new rviz_common::properties::BoolProperty(
    "Show face RoIs",
    true,
    "If set to true, show faces bounding boxes.",
    this,
    SLOT(updateShowFaces()));

  show_facial_landmarks_property_ = new rviz_common::properties::BoolProperty(
    "Show facial landmarks",
    true,
    "If set to true, show faces facial landmarks.",
    this,
    SLOT(updateShowFacialLandmarks()));

  show_bodies_property_ = new rviz_common::properties::BoolProperty(
    "Show body RoIs",
    true,
    "If set to true, show bodies bounding boxes.",
    this,
    SLOT(updateShowBodies()));

  show_skeletons_property_ = new rviz_common::properties::BoolProperty(
    "Show 2D Skeletons",
    true,
    "If set to true, show 2D skeletons.",
    this,
    SLOT(updateShowSkeletons()));

  show_faces_ = true;
  show_facial_landmarks_ = true;
  show_bodies_ = true;
  show_skeletons_ = true;
  got_float_image_ = false;
}

void HumansDisplay::onInitialize()
{
  ITDClass::onInitialize();

  updateNormalizeOptions();
  setupScreenRectangle();

  setupRenderPanel();

  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode * scene_node) {
      scene_node->attachObject(screen_rect_.get());
    });
}

HumansDisplay::~HumansDisplay() = default;
//   if (initialized()) {
//     delete render_panel_;
//     delete screen_rect_;
//     removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(),
//                               img_scene_node_);
//   }
// }

void HumansDisplay::onEnable()
{
ITDClass: subscribe();
}

void HumansDisplay::onDisable()
{
  ITDClass::unsubscribe();
  clear();
}

void HumansDisplay::updateShowFaces()
{
  show_faces_ = show_faces_property_->getBool();
}

void HumansDisplay::updateShowFacialLandmarks()
{
  show_facial_landmarks_ = show_facial_landmarks_property_->getBool();
}

void HumansDisplay::updateShowBodies()
{
  show_bodies_ = show_bodies_property_->getBool();
}

void HumansDisplay::updateShowSkeletons()
{
  show_skeletons_ = show_skeletons_property_->getBool();
}

void HumansDisplay::updateNormalizeOptions()
{
  if (got_float_image_) {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_->setNormalizeFloatImage(
      normalize, min_property_->getFloat(),
      max_property_->getFloat());
    texture_->setMedianFrames(median_buffer_size_property_->getInt());
  } else {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void HumansDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  try {
    texture_->update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_->getWidth();
    float img_height = texture_->getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0) {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect) {
        screen_rect_->setCorners(
          -1.0f, 1.0f * win_aspect / img_aspect, 1.0f, -1.0f * win_aspect / img_aspect, false);
      } else {
        screen_rect_->setCorners(
          -1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect, -1.0f, false);
      }
    }
  } catch (rviz_default_plugins::displays::UnsupportedImageEncoding & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", e.what());
  }
}

void HumansDisplay::reset()
{
  // ImageDisplayBase::reset();
  ITDClass::reset();
  // Display::reset();
  // messages_received_ = 0;
  clear();
  //TODO:
  // render_panel_->getCamera()->setPosition(
  //     Ogre::Vector3(999999, 999999, 999999));
}

void HumansDisplay::drawSkeleton(
  std::string id, int width, int height,
  std::vector<hri_msgs::msg::NormalizedPointOfInterest2D> & skeleton)
{
  /* Body chains:
     1 - 2 - 8 - 11 - 5 ==> Upper body chain
     2 - 3 - 4 ==> Right arm chain
     5 - 6 - 7 ==> Left arm chain
     8 - 9 - 10 ==> Right leg chain
     11 - 12 - 13 ==> Left leg chain
  */

  if (skeleton.size() == SKELETON_POINTS) {

    cv::Scalar skeletonColor = get_color_from_id(id);

    int neckX = clip((int)(skeleton[hri_msgs::msg::Skeleton2D::NECK].x * width), 0, width);
    int neckY = clip((int)(skeleton[hri_msgs::msg::Skeleton2D::NECK].y * height), 0, height);

    int rightShoulderX = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_SHOULDER].x * width),
      0, width);
    int rightShoulderY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_SHOULDER].y * height),
      0, height);

    int rightHipX = clip((int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_HIP].x * width), 0, width);
    int rightHipY =
      clip((int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_HIP].y * height), 0, height);

    int leftHipX = clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_HIP].x * width), 0, width);
    int leftHipY = clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_HIP].y * height), 0, height);

    int leftShoulderX = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_SHOULDER].x * width), 0,
      width);
    int leftShoulderY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_SHOULDER].y * height),
      0, height);

    int rightElbowX = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_ELBOW].x * width), 0,
      width);
    int rightElbowY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_ELBOW].y * height), 0,
      height);

    int rightWristX = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_WRIST].x * width), 0,
      width);
    int rightWristY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_WRIST].y * height), 0,
      height);

    int leftElbowX =
      clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_ELBOW].x * width), 0, width);
    int leftElbowY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_ELBOW].y * height), 0,
      height);

    int leftWristX =
      clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_WRIST].x * width), 0, width);
    int leftWristY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_WRIST].y * height), 0,
      height);

    int rightKneeX =
      clip((int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_KNEE].x * width), 0, width);
    int rightKneeY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_KNEE].y * height), 0,
      height);

    int rightAnkleX = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_ANKLE].x * width), 0,
      width);
    int rightAnkleY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::RIGHT_ANKLE].y * height), 0,
      height);

    int leftKneeX = clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_KNEE].x * width), 0, width);
    int leftKneeY =
      clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_KNEE].y * height), 0, height);

    int leftAnkleX =
      clip((int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_ANKLE].x * width), 0, width);
    int leftAnkleY = clip(
      (int)(skeleton[hri_msgs::msg::Skeleton2D::LEFT_ANKLE].y * height), 0,
      height);

    // Upper body
    cv::line(
      cvBridge_->image, cv::Point(neckX, neckY), cv::Point(
        rightShoulderX,
        rightShoulderY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(rightHipX, rightHipY),
      cv::Point(rightShoulderX, rightShoulderY), skeletonColor, 5, cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(neckX, neckY), cv::Point(
        leftShoulderX,
        leftShoulderY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(
        leftShoulderX,
        leftShoulderY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(
        rightHipX,
        rightHipY), skeletonColor, 5,
      cv::FILLED);

    // Right arm
    cv::line(
      cvBridge_->image, cv::Point(rightShoulderX, rightShoulderY),
      cv::Point(rightElbowX, rightElbowY), skeletonColor, 5, cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(rightElbowX, rightElbowY),
      cv::Point(rightWristX, rightWristY), skeletonColor, 5, cv::FILLED);

    // Left arm
    cv::line(
      cvBridge_->image, cv::Point(leftShoulderX, leftShoulderY),
      cv::Point(leftElbowX, leftElbowY), skeletonColor, 5, cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftElbowX, leftElbowY), cv::Point(
        leftWristX,
        leftWristY), skeletonColor, 5,
      cv::FILLED);

    // Right Leg
    cv::line(
      cvBridge_->image, cv::Point(rightHipX, rightHipY), cv::Point(
        rightKneeX,
        rightKneeY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(rightKneeX, rightKneeY), cv::Point(
        rightAnkleX,
        rightAnkleY), skeletonColor, 5,
      cv::FILLED);

    // Left leg
    cv::line(
      cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(
        leftKneeX,
        leftKneeY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftKneeX, leftKneeY), cv::Point(
        leftAnkleX,
        leftAnkleY), skeletonColor, 5,
      cv::FILLED);

  }
}

void HumansDisplay::processMessage(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  bool got_float_image =
    msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
    msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
    msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_) {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }

  if (!show_faces_ && !show_bodies_ && !show_skeletons_) {
    texture_->addMessage(msg);
    return;
  }

  cvBridge_ = cv_bridge::toCvShare(msg);
  sensor_msgs::msg::Image::SharedPtr modified_img;

  if (show_faces_ || show_facial_landmarks_) {
    auto faces = hri_listener.getFaces();
    for (auto const & face : faces) {
      if (auto face_ptr =
        face.second.lock())          // ensure the face is still here
      {
        if (show_faces_) {
          auto roi = face_ptr->roi();
          cv::rectangle(cvBridge_->image, roi, get_color_from_id(face.first), 5);
        }
        if (show_facial_landmarks_) {
          auto landmarks = *(face_ptr->facialLandmarks()); // boost::optional
          for (auto landmark : landmarks) {
            if (landmark.x > 0 || landmark.y > 0) {
              cv::circle(
                cvBridge_->image, cv::Point(landmark.x, landmark.y), 5,
                get_color_from_id(face.first), cv::FILLED);
            }
          }

        }
      }
    }
  }

  if (show_bodies_ || show_skeletons_) {
    auto bodies = hri_listener.getBodies();
    for (auto const & body : bodies) {
      if (auto body_ptr =
        body.second.lock())          // ensure the body is still here
      {
        if (show_bodies_) {
          auto roi = body_ptr->roi();
          cv::rectangle(cvBridge_->image, roi, get_color_from_id(body.first), 5);
        }
        if (show_skeletons_) {
          auto skeleton = body_ptr->skeleton();
          drawSkeleton(body.first, msg->width, msg->height, skeleton);
        }
      }
    }
  }
  // cv_bridge::CvImage(msg->header, msg->encoding, rotated_image).toImageMsg();
  // texture_->addMessage(cvBridge_->toImageMsg());
  modified_img = cv_bridge::CvImage(msg->header, msg->encoding, cvBridge_->image).toImageMsg();
  texture_->addMessage(modified_img);
}
void HumansDisplay::setupScreenRectangle()
{
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "ImageDisplay2Object" << count++;

  screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

  ss << "Material";
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(ss.str());
  material_->setSceneBlending(Ogre::SBT_REPLACE);
  material_->setDepthWriteEnabled(false);
  material_->setDepthCheckEnabled(false);

  Ogre::TextureUnitState * tu =
    material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);

  material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  screen_rect_->setBoundingBox(aabInf);
  screen_rect_->setMaterial(material_);
}

void HumansDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());

  static int count = 0;
  render_panel_->getRenderWindow()->setObjectName(
    "ImageDisplay2RenderWindow" + QString::number(count++));
}

// void HumansDisplay::subscribe() {
//    if (!isEnabled()) {
//       return;
//     }

//     if (topic_property_->isEmpty()) {
//       setStatus(
//         rviz_common::properties::StatusProperty::Error, "Topic",
//         QString("Error subscribing: Empty topic name"));
//       return;
//     }

//     try {
//       subscription_ = std::make_shared<image_transport::SubscriberFilter>();
//       subscription_->subscribe(
//         rviz_ros_node_.lock()->get_raw_node().get(),
//         getBaseTopicFromTopic(topic_property_->getTopicStd()),
//         getTransportFromTopic(topic_property_->getTopicStd()),
//         qos_profile.get_rmw_qos_profile());
//       subscription_callback_ = subscription_->registerCallback(
//         std::bind(
//           &HumansDisplay::incomingMessage<sensor_msgs::msg::Image>, this, _1));
//       setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
//     } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
//       setStatus(
//         rviz_common::properties::StatusProperty::Error, "Topic",
//         QString("Error subscribing: ") + e.what());
//     }
// }

// void HumansDisplay::unsubscribe()
// {
//     subscription_.reset();
// }

void HumansDisplay::clear()
{
  texture_->clear();
}


// void HumansDisplay::incomingMessage(const typename MessageType::ConstSharedPtr msg)
// {
//    if (!msg) {
//       return;
//     }

//     ++messages_received_;
//     setStatus(
//       rviz_common::properties::StatusProperty::Ok,
//       "Topic",
//       QString::number(messages_received_) + " messages received");

//     processMessage(msg);
// }


}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::HumansDisplay, rviz_common::Display)
