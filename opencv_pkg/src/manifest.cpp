#include <pluginlib/class_list_macros.h>
#include <rnd_ws/resized_publisher.h>
#include <rnd_ws/resized_subscriber.h>

PLUGINLIB_REGISTER_CLASS(resized_pub, ResizedPublisher, image_transport::PublisherPlugin)

PLUGINLIB_REGISTER_CLASS(resized_sub, ResizedSubscriber, image_transport::SubscriberPlugin)

