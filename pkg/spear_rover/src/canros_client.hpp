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
  ActuatorArrayCommand(ActuatorArrayCommand &&) = default;
  ActuatorArrayCommand(ActuatorArrayCommand &) = delete;

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

template <typename T>
using Listener = std::function<void(const typename T::ConstPtr &)>;

/*
 * XXX For unknown reasons, Command doesn't work, you have to use ArrayCommand
 */
class CanrosClient {
 public:
  CanrosClient(ros::NodeHandle &nh) {
    this->actuator_array_command_publisher =
        nh.advertise<canros::uavcan__equipment__actuator__ArrayCommand>(
            "/canros/msg/uavcan/equipment/actuator/ArrayCommand", 1);

    // Needs to be a boost function, not clear why
    boost::function<void(
        const canros::uavcan__equipment__actuator__Status::ConstPtr &)>
        listener =
            [&](const canros::uavcan__equipment__actuator__Status::ConstPtr
                    &status) { this->_on_actuator_status(status); };
    this->actuator_status_subscriber = nh.subscribe(
        "/canros/msg/uavcan/equipment/actuator/Status", 1, listener);
  }

  void send_actuator_commands(
      canros::uavcan__equipment__actuator__ArrayCommand &&command) {
    this->actuator_array_command_publisher.publish(command);
  }

  void recv_actuator_status(
      Listener<canros::uavcan__equipment__actuator__Status> listener) {
    this->actuator_status_listeners.push_back(std::move(listener));
  }

 private:
  void _on_actuator_status(
      const canros::uavcan__equipment__actuator__Status::ConstPtr &status) {
    for (auto &listener : this->actuator_status_listeners) {
      listener(status);
    }
  }

  ros::Publisher actuator_array_command_publisher;
  ros::Subscriber actuator_status_subscriber;
  std::vector<Listener<canros::uavcan__equipment__actuator__Status>>
      actuator_status_listeners;
};

template <typename T>
auto position_listener(T &&func) {
  using Status = canros::uavcan__equipment__actuator__Status;
  Listener<Status> listener = [&](const Status::ConstPtr &status) {
    func(status->actuator_id, status->position);
  };
  return listener;
}

template <typename T>
auto speed_listener(T &&func) {
  using Status = canros::uavcan__equipment__actuator__Status;
  Listener<Status> listener = [&](const Status::ConstPtr &status) {
    func(status->actuator_id, status->speed);
  };
  return listener;
}