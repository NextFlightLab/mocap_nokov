// STL includes
#include <chrono>
#include <thread>
#include <atomic>
#include <condition_variable>

// Local includes
#include <mocap_nokov/mocap_config.h>
#include <mocap_nokov/data_model.h>
#include <mocap_nokov/rigid_body_publisher.h>

// STL includes
#include <mutex>   

// SDK includes
#include <NokovSDKClient.h>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace mocap_nokov
{
  // Forward declaration
  class NokovRosBridge;
  
  // Global pointer to bridge instance for callback access
  static NokovRosBridge* g_bridgeInstance = nullptr;
  static std::mutex g_callbackMutex;

  class NokovRosBridge
  {
  public:
    NokovRosBridge(
      rclcpp::Node::SharedPtr &node,
      ServerDescription const& serverDescr, 
      PublisherConfigurations const& pubConfigs) :
        node(node),
        clock(node->get_clock()),
        serverDescription(serverDescr),
        publisherConfigurations(pubConfigs),
        lastProcessedFrame(-1),
        isShutdownRequested(false)
    {
      g_bridgeInstance = this;
    }

    ~NokovRosBridge() {
      g_bridgeInstance = nullptr;
    }

    void initialize()
    {
      // Create client with optimized settings
      sdkClientPtr.reset(new NokovSDKClient());
      
      // Set high verbosity for debugging performance issues (can be reduced later)
      sdkClientPtr->SetVerbosityLevel(Verbosity_Warning);
      
      // Set callback before initialization
      sdkClientPtr->SetDataCallback(StaticDataHandler);

      unsigned char sdkVersion[4] = {0};
      sdkClientPtr->NokovSDKVersion(sdkVersion);    
      Version ver((int)sdkVersion[0], (int)sdkVersion[1], (int)sdkVersion[2], (int)sdkVersion[3]);
      RCLCPP_INFO(node->get_logger(), "Load SDK Ver:%s", (char*)ver.getVersionString().c_str());

      while(rclcpp::ok() && sdkClientPtr->Initialize((char*)serverDescription.IpAddress.c_str()))
      {
          RCLCPP_WARN(node->get_logger(), "Connecting to server again");
          std::this_thread::sleep_for(std::chrono::seconds(2));
      }

      // Once we have the server info, create publishers
      publishDispatcherPtr.reset(
        new RigidBodyPublishDispatcher(node, 
          ver, publisherConfigurations));
        
      RCLCPP_INFO(node->get_logger(), "Initialization complete");
    }

    void run()
    {
      // Event-driven architecture: just wait for shutdown signal
      RCLCPP_INFO(node->get_logger(), "Event-driven publishing started. Data will be published on callback.");
      
      while (rclcpp::ok() && !isShutdownRequested.load()) {
        // Sleep and check for shutdown periodically
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        rclcpp::spin_some(node);
      }
      
      RCLCPP_INFO(node->get_logger(), "Publishing stopped.");
    }

    void shutdown() {
      isShutdownRequested.store(true);
    }

    // Static callback for SDK
    static void StaticDataHandler(sFrameOfMocapData* pFrameOfData, void* pUserData)
    {
      std::lock_guard<std::mutex> lock(g_callbackMutex);
      if (g_bridgeInstance != nullptr) {
        g_bridgeInstance->DataHandler(pFrameOfData, pUserData);
      }
    }

  private:
    // Instance method for handling data
    void DataHandler(sFrameOfMocapData* pFrameOfData, void* pUserData)
    {
      if (nullptr == pFrameOfData) {
        return;
      }

      // Skip duplicate frames with atomic operation (lockless)
      int currentFrame = pFrameOfData->iFrame;
      int expectedFrame = lastProcessedFrame.load();
      if (currentFrame == expectedFrame) {
        return;
      }
      
      // Try to update lastProcessedFrame atomically
      if (!lastProcessedFrame.compare_exchange_weak(expectedFrame, currentFrame)) {
        // Another thread is processing the same or newer frame
        return;
      }

      // Convert SDK data to our format efficiently - minimize allocations
      std::vector<RigidBody> rigidBodies;
      rigidBodies.reserve(pFrameOfData->nRigidBodies);

      // Batch process rigid bodies for better cache locality
      const double timestamp = pFrameOfData->iTimeStamp / 1000.0;
      for(int i = 0; i < pFrameOfData->nRigidBodies; ++i)
      {
        const auto& sdkBody = pFrameOfData->RigidBodies[i];
        
        // Create body with move semantics
        rigidBodies.emplace_back();
        RigidBody& body = rigidBodies.back();
        
        body.bodyId = sdkBody.ID;
        body.iFrame = currentFrame;
        body.trackTimestamp = timestamp;

        
        // Direct assignment (no intermediate objects)
        body.pose.position.x = sdkBody.x * 0.001f;
        body.pose.position.y = sdkBody.y * 0.001f;
        body.pose.position.z = sdkBody.z * 0.001f;
        body.pose.orientation.x = sdkBody.qx;
        body.pose.orientation.y = sdkBody.qy;
        body.pose.orientation.z = sdkBody.qz;
        body.pose.orientation.w = sdkBody.qw;
        
        body.isTrackingValid = !(body.pose.position.x > 9999);

      }

      // Get timestamp once for all publications
      const rclcpp::Time currentTime = clock->now();
      
      // Publish immediately (this is already optimized to be fast)
      publishDispatcherPtr->publish(currentTime, rigidBodies, node->get_logger());
      
      // Lightweight performance tracking
      static std::atomic<int> frameCounter{0};
      static auto lastStatsTime = std::chrono::steady_clock::now();
      

      int currentCount = frameCounter.fetch_add(1) + 1;
      
      // Log stats every 5 seconds
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatsTime).count() >= 5) {
        double fps = currentCount / 5.0;
        
        if (fps < 90.0) {  // Warn if significantly below 100Hz
          RCLCPP_WARN(node->get_logger(), 
                     "%.1f FPS, (Frame: %d)", 
                     fps, currentFrame);
        } else {
          RCLCPP_DEBUG(node->get_logger(), 
                      "%.1f FPS, (Frame: %d)", 
                      fps, currentFrame);
        }
        
        // Reset counters
        frameCounter.store(0);
        lastStatsTime = now;
      }
    }

  private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Clock::SharedPtr clock;
    ServerDescription serverDescription;
    PublisherConfigurations publisherConfigurations;
    std::unique_ptr<RigidBodyPublishDispatcher> publishDispatcherPtr;
    std::unique_ptr<NokovSDKClient> sdkClientPtr;
    
    std::atomic<int> lastProcessedFrame;
    std::atomic<bool> isShutdownRequested;
  };

} // namespace


////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr rclcpp_node = std::make_shared<rclcpp::Node>("mocap_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
  
  // Grab node configuration from rosparam
  mocap_nokov::ServerDescription serverDescription;
  mocap_nokov::PublisherConfigurations publisherConfigurations;
  mocap_nokov::NodeConfiguration::fromRosParam(rclcpp_node, serverDescription, publisherConfigurations);

  // Create node object, initialize and run
  mocap_nokov::NokovRosBridge node(rclcpp_node, serverDescription, publisherConfigurations);
  node.initialize();
  node.run();

  return 0;
}
