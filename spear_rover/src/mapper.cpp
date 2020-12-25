#include <ros/ros.h>
#include "canard.h"

#include <std_msgs/String.h>

// System includes for CAN
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Needs to be usable by all callbacks
// I really dislike this solution.
ros::NodeHandle *nh;

// TODO improve visibility constraints
struct Can2Ros {
  CanardPortID subject_id;
  std::function<void(CanardTransfer*)> callback;
  CanardRxSubscription subscription;
};

class UavcanMapper {
  public:
    UavcanMapper(CanardNodeID can_id, std::string can_iface);
    void map_can2ros(Can2Ros sub);
    void handle_frame(struct can_frame* in_frame);

  private:
    CanardInstance can_node;
    ros::NodeHandle ros_node;
    int can_sock;

    std::vector<Can2Ros> can2ros;
};

static void* canard_alloc(CanardInstance* ins, size_t amount) {
  (void)ins;
  return malloc(amount);
}

static void canard_free(CanardInstance* ins, void* pointer) {
  (void)ins;
  free(pointer);
}

UavcanMapper::UavcanMapper(CanardNodeID can_id, std::string can_iface) {
  can_node = canardInit(canard_alloc, canard_free);
  // TODO anonymous nodes are unsupported for now
  can_node.node_id = can_id;

  // Bind to socket for CAN TX
  int can_sock;
  if ((can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while openning CAN socket");
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
  }
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
  frame.timestamp_usec = 0;

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

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "uavcan_mapper");

  ros::NodeHandle node;
  nh = &node;

  CanardNodeID id = nh->param("node_id", 100);
  std::string can_iface = nh->param("can_interface", std::string("vcan0"));
  UavcanMapper mapper(id, can_iface);

  // Start spinning for incoming messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Bind to socket for rx
  int sock;
  if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while openning CAN socket");
    return -1;
  }
  struct ifreq can_ifr;
  strcpy(can_ifr.ifr_name, can_iface.c_str());
  ioctl(sock, SIOCGIFINDEX, &can_ifr);
  struct sockaddr_can can_addr;
  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;
  // TODO idiomatic C++ casts
  if (bind(sock, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
    perror("Error binding to CAN socket");
    return -1;
  }

  // Configure epoll
  int epoll = epoll_create(1);
  if (epoll < 0) {
    perror("epoll_create");
    return -1;
  }
  struct epoll_event event_setup;
  struct epoll_event epoll_events[1];
  event_setup.events = EPOLLIN;
  if (epoll_ctl(epoll, EPOLL_CTL_ADD, sock, &event_setup)) {
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
      struct can_frame in_frame;
      int len = read(sock, &in_frame, sizeof(in_frame));
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