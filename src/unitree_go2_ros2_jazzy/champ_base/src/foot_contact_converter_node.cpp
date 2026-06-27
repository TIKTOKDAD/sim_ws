#include <array>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "champ_msgs/msg/contacts_stamped.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"

class FootContactConverter : public rclcpp::Node
{
public:
  FootContactConverter()
  : Node("foot_contact_converter_node"),
    hold_duration_(rclcpp::Duration::from_seconds(0.05))
  {
    const double publish_rate = this->declare_parameter<double>("publish_rate", 100.0);
    const double hold_duration = this->declare_parameter<double>("hold_duration", 0.05);
    hold_duration_ = rclcpp::Duration::from_seconds(hold_duration);

    foot_contacts_publisher_ =
      this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);

    const std::array<std::string, kLegCount> topics = {
      "lf_foot_contact",
      "rf_foot_contact",
      "lh_foot_contact",
      "rh_foot_contact",
    };

    const rclcpp::Time now = this->now();
    last_contact_time_.fill(now);

    for (size_t i = 0; i < kLegCount; ++i) {
      contact_subscribers_[i] =
        this->create_subscription<ros_gz_interfaces::msg::Contacts>(
          topics[i],
          10,
          [this, i](const ros_gz_interfaces::msg::Contacts::SharedPtr msg) {
            this->contactCallback(i, msg);
          });
    }

    const double safe_publish_rate = publish_rate > 0.0 ? publish_rate : 100.0;
    const auto timer_period = std::chrono::duration<double>(1.0 / safe_publish_rate);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&FootContactConverter::publishContacts, this));
  }

private:
  static constexpr size_t kLegCount = 4;

  void contactCallback(
    const size_t index,
    const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    if (!msg->contacts.empty()) {
      has_seen_contact_[index] = true;
      last_contact_time_[index] = this->now();
    }
  }

  bool contactActive(const size_t index, const rclcpp::Time & now) const
  {
    if (!has_seen_contact_[index]) {
      return false;
    }

    return (now - last_contact_time_[index]).seconds() <= hold_duration_.seconds();
  }

  void publishContacts()
  {
    const rclcpp::Time now = this->now();

    champ_msgs::msg::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = now;
    contacts_msg.header.frame_id = "base_link";
    contacts_msg.contacts.resize(kLegCount);

    for (size_t i = 0; i < kLegCount; ++i) {
      contacts_msg.contacts[i] = contactActive(i, now);
    }

    foot_contacts_publisher_->publish(contacts_msg);
  }

  rclcpp::Duration hold_duration_;
  std::array<bool, kLegCount> has_seen_contact_ = {false, false, false, false};
  std::array<rclcpp::Time, kLegCount> last_contact_time_;
  std::array<
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr,
    kLegCount> contact_subscribers_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootContactConverter>());
  rclcpp::shutdown();
  return 0;
}
