#include <string>

#include "hri_rviz/get_transport_from_topic.hpp"

namespace rviz_default_plugins
{
namespace displays
{

bool isRawTransport(const std::string & topic)
{
  std::string last_subtopic = topic.substr(topic.find_last_of('/') + 1);
  return last_subtopic != "compressed" && last_subtopic != "compressedDepth" &&
         last_subtopic != "theora";
}

std::string getTransportFromTopic(const std::string & topic)
{
  if (isRawTransport(topic)) {
    return "raw";
  }
  return topic.substr(topic.find_last_of('/') + 1);
}

std::string getBaseTopicFromTopic(const std::string & topic)
{
  if (isRawTransport(topic)) {
    return topic;
  }
  return topic.substr(0, topic.find_last_of('/'));
}

}  //  end namespace displays
}  //  end namespace rviz_default_plugins
