#include <ros/ros.h>
#include "canard.h"

// ROS message types
#include <std_msgs/String.h>
#include <spear_msgs/drive_command.h>

// UAVCAN message types
#include "spear/drive/DriveCommand_1_0.h"

#include <mutex>
#include <chrono>

// System includes for CAN
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// I think I need to add an intermediate macro?
// TODO error handling
#define TX_TRANSFER(type, msg, _transfer_id, _port_id, _priority) {\
  using namespace std::chrono; \
  uint8_t _ser_buf[type ## _SERIALIZATION_BUFFER_SIZE_BYTES_]; \
  size_t _ser_size = type ## _SERIALIZATION_BUFFER_SIZE_BYTES_; \
  type ## _serialize_(&msg, _ser_buf, &_ser_size); \
  CanardTransfer _xfer; \
  _xfer.timestamp_usec = milliseconds(duration_cast<milliseconds>(steady_clock::now().time_since_epoch())).count(); \
  _xfer.priority = _priority; \
  _xfer.transfer_kind = CanardTransferKindMessage; \
  _xfer.port_id = _port_id; \
  _xfer.remote_node_id = CANARD_NODE_ID_UNSET; \
  _xfer.transfer_id = _transfer_id; \
  _xfer.payload = _ser_buf; \
  _xfer.payload_size = _ser_size; \
  m->send_transfer(&_xfer); \
}

// TODO error handling
#define RX_TRANSFER(type, xfer, out_msg) {\
  size_t _deser_size = xfer->payload_size; \
  type ## _deserialize_(&out_msg, static_cast<const uint8_t*>(xfer->payload), &_deser_size); \
}

// TODO improve visibility constraints
struct Can2Ros {
  CanardPortID subject_id;
  std::function<void(CanardTransfer*)> callback;
  CanardRxSubscription subscription;
};

class UavcanMapper {
  public:
    UavcanMapper(CanardNodeID can_id, int* can_sock, std::mutex* can_sock_mtx);
    void map_can2ros(Can2Ros sub);
    void handle_frame(struct can_frame* in_frame);
    void send_transfer(CanardTransfer* xfer);

  private:
    CanardInstance can_node;
    ros::NodeHandle ros_node;

    int* _can_sock;
    std::mutex* _can_sock_mtx;

    std::vector<Can2Ros> can2ros;
};

// Needs to be usable by all callbacks
// I really dislike this solution.
std::unique_ptr<ros::NodeHandle> nh;
UavcanMapper* m;

static void* canard_alloc(CanardInstance* ins, size_t amount) {
  (void)ins;
  return malloc(amount);
}

static void canard_free(CanardInstance* ins, void* pointer) {
  (void)ins;
  free(pointer);
}

UavcanMapper::UavcanMapper(CanardNodeID can_id, int* can_sock, std::mutex* can_sock_mtx) :
  _can_sock(can_sock), _can_sock_mtx(can_sock_mtx) {
  can_node = canardInit(canard_alloc, canard_free);
  // TODO anonymous nodes are unsupported for now
  can_node.node_id = can_id;
}

void UavcanMapper::map_can2ros(Can2Ros sub) {
  can2ros.push_back(sub);
  Can2Ros* _sub = &can2ros[can2ros.size() - 1];
  // TODO configure timeout
  canardRxSubscribe(
    &can_node,
    CanardTransferKindMessage,
    _sub->subject_id,
    100,
    10000,
    &_sub->subscription
  );
}

void UavcanMapper::handle_frame(struct can_frame* in_frame) {
  CanardFrame frame;
  frame.extended_can_id = in_frame->can_id;
  frame.payload = &in_frame->data;
  frame.payload_size = in_frame->can_dlc;
  // TODO timestamps
  frame.timestamp_usec = milliseconds(duration_cast<milliseconds>(steady_clock::now().time_since_epoch())).count();

  CanardTransfer xfer;
  int8_t rc = canardRxAccept(&can_node, &frame, 0, &xfer);
  // TODO proper error handling
  if (rc == 1) {
    for (auto const& sub: can2ros) {
      if (sub.subject_id == xfer.port_id) {
        sub.callback(&xfer);
      }
    }
  }

}

void UavcanMapper::send_transfer(CanardTransfer* xfer) {
  // TODO error handling
  canardTxPush(&can_node, xfer);

  std::lock_guard<std::mutex> _lock(*_can_sock_mtx);

  // Push all the frames out
  const CanardFrame* frame;
  while ((frame = canardTxPeek(&can_node)) != NULL) {
    struct can_frame out_frame;
    out_frame.can_id = frame->extended_can_id;
    out_frame.can_dlc = frame->payload_size;
    memcpy(out_frame.data, frame->payload, frame->payload_size);
    int rc = write(*_can_sock, &out_frame, sizeof(out_frame));
    if (rc < 0) {
      perror("Error sending CAN frame");
    }
    canardTxPop(&can_node);
  }
}

/* -------- ROS to CAN Callbacks -------- */
/// Example callback
void ros2can_drive_cb(const spear_msgs::drive_command::ConstPtr& msg) {
  // Transfer ID must monotonically increase
  static uint8_t transfer_id = 0;

  ROS_INFO("<Drive Command> id: %d, speed: %f", msg->id, msg->speed);

  // Business conversion logic, in this case it's a 1:1 mapping so very simple.
  spear_drive_DriveCommand_1_0 cmd;
  cmd.speed = msg->speed;
  cmd.id.value = msg->id;

  // Simplest I could make a macro.
  TX_TRANSFER(spear_drive_DriveCommand_1_0, cmd, transfer_id, 0, CanardPriorityNominal);
  transfer_id++;
}

/* -------- CAN to ROS Conversions -------- */
ros::Publisher* can2ros_drive_pub;
void can2ros_drive_cb(CanardTransfer* xfer) {
  // Need to deserialize UAVCAN message and deal with the transfer
  spear_drive_DriveCommand_1_0 cmd;
  RX_TRANSFER(spear_drive_DriveCommand_1_0, xfer, cmd);

  // Here we can do whatever we actually want with the message.
  spear_msgs::drive_command out_msg;
  out_msg.id = cmd.id.value;
  out_msg.speed = cmd.speed;
  can2ros_drive_pub->publish(out_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uavcan_mapper");

  nh = std::make_unique<ros::NodeHandle>();

  std::string can_iface = nh->param("can_interface", std::string("vcan0"));

  // Bind to socket for rx
  int can_sock;
  std::mutex can_sock_mtx;
  if ((can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while openning CAN socket");
    return -1;
  }
  struct ifreq can_ifr;
  strcpy(can_ifr.ifr_name, can_iface.c_str());
  ioctl(can_sock, SIOCGIFINDEX, &can_ifr);
  struct sockaddr_can can_addr;
  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;
  // TODO idiomatic C++ casts
  if (bind(can_sock, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
    perror("Error binding to CAN socket");
    return -1;
  }

  /* -------- Subscribe to relevant ROS topics here -------- */
  ros::Subscriber _ros2can_drive_sub = nh->subscribe("/core/drive", 100, ros2can_drive_cb);

  // Start spinning for incoming ROS messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  /* -------- Create any ROS advertisers you need here -------- */
  // I don't like this, there's probably a better way.
  ros::Publisher _can2ros_drive_pub = nh->advertise<spear_msgs::drive_command>("/core/drive", 100);
  can2ros_drive_pub = &_can2ros_drive_pub;

  // Create mapper object
  CanardNodeID can_id = nh->param("node_id", 100);
  UavcanMapper mapper(can_id, &can_sock, &can_sock_mtx);
  m = &mapper;

  /* -------- Subscribe to relevant UAVCAN broadcasts here -------- */
  Can2Ros sub{};
  sub.subject_id = 0;
  sub.callback = can2ros_drive_cb;
  m->map_can2ros(sub);

  // Configure epoll
  int epoll = epoll_create(1);
  if (epoll < 0) {
    perror("epoll_create");
    return -1;
  }
  struct epoll_event event_setup;
  struct epoll_event epoll_events[1];
  event_setup.events = EPOLLIN;
  if (epoll_ctl(epoll, EPOLL_CTL_ADD, can_sock, &event_setup)) {
    perror("epoll_ctl");
    return -1;
  }

  while (1) {
    int num_events = epoll_wait(epoll, epoll_events, 1, -1);
    if (num_events <= 0) {
      // Socket is closed or something
      return 0;
    }

    while (num_events-- > 0) {
      // Lock the CAN socket mutex
      std::lock_guard<std::mutex> _lock(can_sock_mtx);
      struct can_frame in_frame;

      int len = read(can_sock, &in_frame, sizeof(in_frame));
      if (len < 0) {
        perror("Couldn't receive frame");
        return -1;
      }

      mapper.handle_frame(&in_frame);
    }

  }

  ros::waitForShutdown();

  return -1;
}