#ifndef TOPIC_PROXY_SERVER_H
#define TOPIC_PROXY_SERVER_H

#include <topic_proxy/topic_proxy.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/serialization.h>

#include <ros/network.h>
#include <ros/connection_manager.h>
#include <ros/transport/transport_tcp.h>

#include <blob/shape_shifter.h>

#include <android/log.h>

#define LOG_NAME "topic_proxy_server"

#ifndef LOGI
#define LOGI(LOG_NAME, ...) \
  __android_log_print(ANDROID_LOG_INFO, LOG_NAME, __VA_ARGS__)
#endif  // LOGI

#ifndef LOGE
#define LOGE(LOG_NAME, ...) \
  __android_log_print(ANDROID_LOG_ERROR, LOG_NAME, __VA_ARGS__)
#endif  // LOGE

#ifndef CHECK
#define CHECK(condition)                                                            \
  if (!(condition)) {                                                               \
    LOGE("CHECK", "*** CHECK FAILED at %s:%d: %s", __FILE__, __LINE__, #condition); \
    abort();                                                                        \
  }
#endif  // CHECK

namespace topic_proxy
{

using blob::ShapeShifter;

class Server
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer get_message_server_;
  ros::ServiceServer publish_message_server_;

  typedef ros::MessageEvent<const ShapeShifter> MessageEvent;

  struct SubscriptionInfo {
    std::string topic;
    ros::Subscriber subscriber;
    ros::CallbackQueue callback_queue;
    MessageEvent event;
    ShapeShifter::ConstPtr last_message;
  };
  typedef boost::shared_ptr<SubscriptionInfo> SubscriptionInfoPtr;
  std::map<std::string, SubscriptionInfoPtr> subscriptions_;

  struct PublicationInfo {
    std::string topic;
    ros::Publisher publisher;
  };
  typedef boost::shared_ptr<PublicationInfo> PublicationInfoPtr;
  std::map<std::string, PublicationInfoPtr> publications_;

public:
  /*Server()
  {
    get_message_server_     = nh_.advertiseService(g_get_message_service, &Server::handleGetMessage, this);
    publish_message_server_ = nh_.advertiseService(g_publish_message_service, &Server::handlePublishMessage, this);
  }*/

  Server(const ros::NodeHandle& nh) : nh_(nh)
  {
    LOGI(LOG_NAME, "Server constructor");
    get_message_server_     = nh_.advertiseService<GetMessage::Request, GetMessage::Response>(g_get_message_service, boost::bind(&Server::handleGetMessage, this, _1, _2));
    publish_message_server_ = nh_.advertiseService(g_publish_message_service, &Server::handlePublishMessage, this);
  }

  ~Server()
  {
    clearSubscriptions();
    clearPublications();
  }

  const std::string& getHost() const
  {
    return ros::network::getHost();
  }

  uint16_t getTCPPort() const
  {
    return ros::ConnectionManager::instance()->getTCPPort();
  }

protected:
  const SubscriptionInfoPtr& getSubscription(const std::string& topic)
  {
    if (subscriptions_.count(topic)) return subscriptions_.at(topic);
    SubscriptionInfoPtr subscription(new SubscriptionInfo());
    return subscriptions_.insert(std::pair<std::string, SubscriptionInfoPtr>(topic, subscription)).first->second;
  }

  const PublicationInfoPtr& getPublication(const std::string& topic)
  {
    if (publications_.count(topic)) return publications_.at(topic);
    PublicationInfoPtr publication(new PublicationInfo());
    return publications_.insert(std::pair<std::string, PublicationInfoPtr>(topic, publication)).first->second;
  }

  bool handleGetMessage(GetMessage::Request& request, GetMessage::Response& response)
  {
    LOGI(LOG_NAME, "handleGetMessage callback");
    SubscriptionInfoPtr subscription = getSubscription(request.topic);
    LOGI(LOG_NAME, "handleGetMessage callback: got subscription");

    if (!subscription->subscriber) {
      LOGI(LOG_NAME, "Subscribing to topic %s", request.topic.c_str());
      ros::SubscribeOptions ops = ros::SubscribeOptions::create<ShapeShifter>(request.topic, 1, boost::bind(&Server::subscriberCallback, this, subscription, _1), ros::VoidConstPtr(), &(subscription->callback_queue));
      subscription->subscriber = nh_.subscribe(ops);
      subscription->topic = subscription->subscriber.getTopic();
    }

    // wait for exactly one callback and reset event pointer
    LOGI(LOG_NAME, "handleGetMessage callback: waiting for timeout...");
    ros::WallDuration timeout(request.timeout.sec, request.timeout.nsec);
    if (timeout > ros::WallDuration()) {
      // clear callback queue and ignore all messages received if a timeout was specified
      subscription->callback_queue.clear();
    }
    subscription->event = MessageEvent();
    subscription->callback_queue.callOne(timeout);
    LOGI(LOG_NAME, "handleGetMessage callback: waiting over");

    ShapeShifter::ConstPtr instance;
    try {
      LOGI(LOG_NAME, "handleGetMessage callback: getting message from subscription");
      instance = subscription->event.getConstMessage();
      LOGI(LOG_NAME, "handleGetMessage callback: got message from subscription");

    } catch(ros::Exception& e) {
      LOGI(LOG_NAME, "Catched exception while handling a request for topic %s: %s", request.topic.c_str(), e.what());
      return false;
    }

    // any message message has been received?
    if (instance) {
      LOGI(LOG_NAME, "handleGetMessage callback: message received");
      // fill response
      response.message.topic = subscription->topic;
      response.message.md5sum = instance->getMD5Sum();
      response.message.type = instance->getDataType();
      response.message.message_definition = instance->getMessageDefinition();
      // response.message.latching = subscription->event.getConnectionHeader()["latching"];
      response.message.blob = instance->blob();
      response.message.blob.setCompressed(request.compressed);

      subscription->last_message = instance;

    } else {
      LOGI(LOG_NAME, "handleGetMessage callback: NO message received");
      // fill response from last message (but without data)
      LOGI(LOG_NAME, "handleGetMessage callback: setting topic");
      LOGI(LOG_NAME, "handleGetMessage callback: setting topic with: %s", subscription->topic.c_str());
      response.message.topic = subscription->topic;
      if (subscription->last_message) {
        LOGI(LOG_NAME, "handleGetMessage callback: setting md5sum");
        LOGI(LOG_NAME, "handleGetMessage callback: setting md5sum with: %s", subscription->last_message->getMD5Sum().c_str());
        response.message.md5sum = subscription->last_message->getMD5Sum();
        LOGI(LOG_NAME, "handleGetMessage callback: setting message type");
        LOGI(LOG_NAME, "handleGetMessage callback: setting message type with: %s", subscription->last_message->getDataType().c_str());
        response.message.type = subscription->last_message->getDataType();
        LOGI(LOG_NAME, "handleGetMessage callback: setting message definition");
        LOGI(LOG_NAME, "handleGetMessage callback: setting message definition with: %s", subscription->last_message->getMessageDefinition().c_str());
        response.message.message_definition = subscription->last_message->getMessageDefinition();
      }
    }

    LOGI(LOG_NAME, "handleGetMessage callback: end of callback, returning true");
    return true;
  }

  void subscriberCallback(const SubscriptionInfoPtr& subscription, const MessageEvent& event)
  {
    LOGI(LOG_NAME, "subscriberCallback");
    LOGI(LOG_NAME, "subscriberCallback: setting subscription event.");
    subscription->event = event;
    LOGI(LOG_NAME, "subscriberCallback: end of callback");
  }

  bool handlePublishMessage(PublishMessage::Request& request, PublishMessage::Response& response)
  {
    PublicationInfoPtr publication = getPublication(request.message.topic);

    if (!publication->publisher) {
      ROS_INFO("Publishing topic %s (%s)", request.message.topic.c_str(), request.message.type.c_str());
      ros::AdvertiseOptions ops(request.message.topic, 10, request.message.md5sum, request.message.type, request.message.message_definition,
                                boost::bind(&Server::connectCallback, this, publication, _1), boost::bind(&Server::disconnectCallback, this, publication, _1));
      ops.latch = request.latch;
      publication->publisher = nh_.advertise(ops);
      publication->topic = publication->publisher.getTopic();
    }

    publication->publisher.publish(
      request.message.blob.asMessage().morph(request.message.md5sum, request.message.type, request.message.message_definition)
    );
    return true;
  }

  void clearSubscriptions() {
    for (std::map<std::string, SubscriptionInfoPtr>::iterator it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
      it->second->subscriber.shutdown();
    }
    subscriptions_.clear();
  }

  void clearPublications() {
    for (std::map<std::string, PublicationInfoPtr>::iterator it = publications_.begin(); it != publications_.end(); ++it) {
      it->second->publisher.shutdown();
    }
    publications_.clear();
  }

protected:
  void connectCallback(const PublicationInfoPtr&, const ros::SingleSubscriberPublisher&)
  {}

  void disconnectCallback(const PublicationInfoPtr&, const ros::SingleSubscriberPublisher&)
  {}
};

} // namespace topic_proxy

#endif // TOPIC_PROXY_SERVER_H
