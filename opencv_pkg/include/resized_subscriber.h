#include <image_transport/simple_subscriber_plugin.h>
#include <opencv_pkg/ResizedImage.h>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<opencv_pkg::ResizedImage>
{
public:
  virtual ~ResizedSubscriber() {}

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename opencv_pkg::ResizedImage::ConstPtr& message,
                                const Callback& user_cb);
};

