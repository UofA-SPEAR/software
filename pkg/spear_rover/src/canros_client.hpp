#include <canros/uavcan__equipment__actuator__ArrayCommand.h>
#include <canros/uavcan__equipment__actuator__Command.h>
#include <canros/uavcan__equipment__actuator__Status.h>
#include <ros/ros.h>

#include <boost/functional.hpp>
#include <unordered_set>
#include <utility>

using actuator_id_t =
    canros::uavcan__equipment__actuator__Command::_actuator_id_type;
using command_type_t =
    canros::uavcan__equipment__actuator__Command::_command_type_type;
using command_value_t =
    canros::uavcan__equipment__actuator__Command::_command_value_type;

class ActuatorArrayCommand {
 public:
  ActuatorArrayCommand() = default;
  ActuatorArrayCommand(ActuatorArrayCommand &) = delete;
  ActuatorArrayCommand(ActuatorArrayCommand &&) = default;

  ActuatorArrayCommand *add_speed(actuator_id_t id, command_value_t speed) {
    return this->add(
        id, canros::uavcan__equipment__actuator__Command::COMMAND_TYPE_SPEED,
        speed);
  }

  ActuatorArrayCommand *add_position(actuator_id_t id,
                                     command_value_t position) {
    return this->add(
        id, canros::uavcan__equipment__actuator__Command::COMMAND_TYPE_POSITION,
        position);
  }

  ActuatorArrayCommand *add(actuator_id_t id, command_type_t command_type,
                            command_value_t command_value) {
    canros::uavcan__equipment__actuator__Command command;
    command.actuator_id = id;
    command.command_type = command_type;
    command.command_value = command_value;
    this->commands.push_back(std::move(command));
    return this;
  }

  canros::uavcan__equipment__actuator__ArrayCommand build() {
    canros::uavcan__equipment__actuator__ArrayCommand message;
    message.commands = this->commands;
    return message;
  }

 private:
  std::vector<canros::uavcan__equipment__actuator__Command> commands;
};

/**
 * Provides a nicer callback interface to a Subscriber.
 */
template <typename Message>
class ObservableSubscriber {
 public:
  using StdObserver = std::function<void(const typename Message::ConstPtr &)>;
  using BoostObserver =
      boost::function<void(const typename Message::ConstPtr &)>;

  ObservableSubscriber(ros::NodeHandle &nh, const std::string &topic) {
    this->message_callback = [&](const typename Message::ConstPtr &msg) {
      for (auto &observer : this->observers) {
        observer(msg);
      }
    };
    this->subscriber = nh.subscribe<Message>(topic, 1, this->message_callback);
  }
  ObservableSubscriber() = delete;
  ObservableSubscriber(ObservableSubscriber<Message> &) = delete;
  ObservableSubscriber(ObservableSubscriber<Message> &&) = default;

  /**
   * Beware of dangling references
   */
  template <typename Func>
  void observe(Func &&observer) {
    this->observers.emplace_back(std::forward<Func>(observer));
  }

 private:
  ros::Subscriber subscriber;
  BoostObserver message_callback;
  std::vector<StdObserver> observers;
};

/*
 * XXX For unknown reasons, Command doesn't work, you have to use ArrayCommand
 */
class CanrosClient {
 public:
  CanrosClient(ros::NodeHandle &nh)
      : actuator_status_subscriber(
            nh, "/canros/msg/uavcan/equipment/actuator/Status") {
    this->actuator_array_command_publisher =
        nh.advertise<canros::uavcan__equipment__actuator__ArrayCommand>(
            "/canros/msg/uavcan/equipment/actuator/ArrayCommand", 1);
    ROS_ERROR("Make canros clien");
  }

  void send_actuator_commands(
      canros::uavcan__equipment__actuator__ArrayCommand &&command) {
    this->actuator_array_command_publisher.publish(command);
  }

  template <typename Func>
  void observe_actuator_status(Func &&observer) {
    ROS_ERROR("Set up observer");
    this->actuator_status_subscriber.observe(std::forward<Func>(observer));
  }

 private:
  ros::Publisher actuator_array_command_publisher;
  ObservableSubscriber<canros::uavcan__equipment__actuator__Status>
      actuator_status_subscriber;
};

template <typename T>
auto position_observer(T &&func) {
  using Status = canros::uavcan__equipment__actuator__Status;
  return [func](const Status::ConstPtr &status) {
    func(status->actuator_id, status->position);
  };
}

template <typename T>
auto speed_observer(T &&func) {
  using Status = canros::uavcan__equipment__actuator__Status;
  return [func](const Status::ConstPtr &status) {
    func(status->actuator_id, status->speed);
  };
}
