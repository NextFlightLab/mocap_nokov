#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mocap_nokov/rigid_body_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <cmath>

namespace mocap_nokov
{

namespace utilities
{
  geometry_msgs::msg::PoseStamped getRosPose(RigidBody const& body, bool newCoordinates)
  {
    geometry_msgs::msg::PoseStamped poseStampedMsg;
    if (newCoordinates)
    {
      poseStampedMsg.pose.position.x = body.pose.position.x;
      poseStampedMsg.pose.position.y = body.pose.position.y;
      poseStampedMsg.pose.position.z = body.pose.position.z;
  
      poseStampedMsg.pose.orientation.x = body.pose.orientation.x;
      poseStampedMsg.pose.orientation.y = body.pose.orientation.y;
      poseStampedMsg.pose.orientation.z = body.pose.orientation.z;
      poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
    }
    else
    {
      // y & z axes are swapped in the Nokov coordinate system
      poseStampedMsg.pose.position.x = body.pose.position.x;
      poseStampedMsg.pose.position.y = -body.pose.position.z;
      poseStampedMsg.pose.position.z = body.pose.position.y;
  
      poseStampedMsg.pose.orientation.x = body.pose.orientation.x;
      poseStampedMsg.pose.orientation.y = -body.pose.orientation.z;
      poseStampedMsg.pose.orientation.z = body.pose.orientation.y;
      poseStampedMsg.pose.orientation.w = body.pose.orientation.w;
    }
    return poseStampedMsg;
  }   
}

RigidBodyPublisher::RigidBodyPublisher(rclcpp::Node::SharedPtr &node, 
  Version const& sdkVersion,
  PublisherConfiguration const& config) :
    config(config), tfPublisher(node)
{
  if (config.publishPose)
    posePublisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(config.poseTopicName, 1000);

  if (config.publishPose2d)
    pose2dPublisher = node->create_publisher<geometry_msgs::msg::Pose2D>(config.pose2dTopicName, 1000);

  useNewCoordinates = (sdkVersion >= Version("3.0"));
}

RigidBodyPublisher::~RigidBodyPublisher()
{

}

void RigidBodyPublisher::publish(rclcpp::Time const& time, RigidBody const& body, rclcpp::Logger logger)
{
  // don't do anything if no new data was provided
  if (!body.hasValidData())
  {
    return;
  }

  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped pose = utilities::getRosPose(body, useNewCoordinates);

  double curTimeDifference = time.seconds() - body.trackTimestamp;

  // Improved clock synchronization logic
  if (!isClockSyncInitialized) {
    // Collect samples for initial sync estimation
    syncSamples.push_back(curTimeDifference);
    
    if (syncSamples.size() >= SYNC_SAMPLE_COUNT) {
      // Use robust statistics: median for outlier resistance + mean of central values for precision
      std::vector<double> sortedSamples = syncSamples;
      std::sort(sortedSamples.begin(), sortedSamples.end());
      
      // Use trimmed mean: remove top/bottom 20% and average the rest
      size_t trimCount = syncSamples.size() / 5; // Remove 20%
      size_t startIdx = trimCount;
      size_t endIdx = syncSamples.size() - trimCount;
      
      double sum = 0.0;
      for (size_t i = startIdx; i < endIdx; ++i) {
        sum += sortedSamples[i];
      }
      timeDifference = sum / (endIdx - startIdx);
      
      RCLCPP_INFO(logger, "Initial clock sync established: %.5f ms (trimmed mean of %zu samples)", 
                  timeDifference * 1000, syncSamples.size());
      isClockSyncInitialized = true;
      syncSamples.clear(); // Free memory
    } else {
      // Use current sample as temporary sync while collecting
      timeDifference = curTimeDifference;
    }
  } else {
    // Adaptive sync improvement with absolute threshold (handles bidirectional drift)
    double timeDrift = std::abs(timeDifference - curTimeDifference);
    if (timeDrift > SYNC_IMPROVEMENT_THRESHOLD) {
      RCLCPP_DEBUG(logger, "Updating clock sync: drift %.5f ms detected", timeDrift * 1000);
      timeDifference = curTimeDifference;
    }
  }

  // Calculate corrected timestamp using improved sync
  double corStamp = body.trackTimestamp + timeDifference;
  
  // More robust timestamp conversion
  int32_t sec = static_cast<int32_t>(std::floor(corStamp));
  uint32_t nanosec = static_cast<uint32_t>((corStamp - sec) * 1e9);
  pose.header.stamp = rclcpp::Time(sec, nanosec);

  if (config.publishPose)
  {
    pose.header.frame_id = config.parentFrameId;
    // pose.header.frame_id = std::to_string(body.iFrame);
    posePublisher->publish(pose);
  }

  tf2::Quaternion q(pose.pose.orientation.x,
                   pose.pose.orientation.y,
                   pose.pose.orientation.z,
                   pose.pose.orientation.w);


  // publish 2D pose
  if (config.publishPose2d)
  {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;
    pose2d.theta = tf2::getYaw(q);
    pose2dPublisher->publish(pose2d);
  }

  if (config.publishTf)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = config.parentFrameId;
    transformStamped.header.stamp = time;
    transformStamped.child_frame_id = config.childFrameId;
    transformStamped.transform.rotation = pose.pose.orientation;
    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;

    tfPublisher.sendTransform(transformStamped);
  }
}


RigidBodyPublishDispatcher::RigidBodyPublishDispatcher(
  rclcpp::Node::SharedPtr &node, 
  Version const& sdkVersion,
  PublisherConfigurations const& configs)
{
  for (auto const& config : configs)
  {
    rigidBodyPublisherMap[config.rigidBodyId] = 
      RigidBodyPublisherPtr(new RigidBodyPublisher(node, sdkVersion, config));
  }
}

void RigidBodyPublishDispatcher::publish(
  rclcpp::Time const& time, 
  std::vector<RigidBody> const& rigidBodies,
  rclcpp::Logger logger
  )
{
  for (auto const& rigidBody : rigidBodies)
  {
    auto const& iter = rigidBodyPublisherMap.find(rigidBody.bodyId);

    if (iter != rigidBodyPublisherMap.end())
    {
      (*iter->second).publish(time, rigidBody, logger);
    }
  }
}


} // namespace
