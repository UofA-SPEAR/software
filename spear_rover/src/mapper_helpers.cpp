#include "mapper.h"

UavcanMapper::UavcanMapper(CanardNodeID can_id, std::string can_iface) {
  can_node = canardInit(canard_alloc, canard_free);
  struct ifreq can_ifr;
  struct sockaddr_can can_addr;

  // TODO anonymous nodes are unsupported for now
  can_node.node_id = can_id;

  // Note: I was talking with some external people about some of the C++ stuff,
  // and substrate was recommended to me as a library: https://github.com/bad-alloc-heavy-industries/substrate
  //
  // It has much nicer abstractions for sockets and some core stuff that would be much nicer to use.
  // Integrating it could be as simple as adding it as a submodule and an include directory, but it's an
  // extra unnecessary dependancy, so I'm not adding it in yet unless others find it interesting.
  if ((_can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while openning CAN socket");
    throw;
  }
  strcpy(can_ifr.ifr_name, can_iface.c_str());
  ioctl(_can_sock, SIOCGIFINDEX, &can_ifr);
  can_addr.can_family = AF_CAN;
  can_addr.can_ifindex = can_ifr.ifr_ifindex;
  // TODO idiomatic C++ casts
  if (bind(_can_sock, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
    perror("Error binding to CAN socket");
    throw;
  }

}

void UavcanMapper::map_can2ros(CanardPortID port_id, std::function<void(CanardTransfer*)> callback) {
  Can2Ros subscription{};
  subscription.port_id = port_id;
  subscription.callback = callback;
  can2ros.push_back(subscription);
  Can2Ros* _sub = &can2ros[can2ros.size() - 1];
  // TODO configure timeout
  canardRxSubscribe(
    &can_node,
    CanardTransferKindMessage,
    _sub->port_id,
    100,
    10000,
    &_sub->subscription
  );
}

void UavcanMapper::handle_frame(struct can_frame* in_frame) {
  using namespace std::chrono;
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
      if (sub.port_id == xfer.port_id) {
        sub.callback(&xfer);
      }
    }
  }

}

void UavcanMapper::send_transfer(CanardTransfer* xfer) {
  // TODO error handling
  canardTxPush(&can_node, xfer);

  std::lock_guard<std::mutex> _lock(_can_sock_mtx);

  // Push all the frames out
  const CanardFrame* frame;
  while ((frame = canardTxPeek(&can_node)) != NULL) {
    struct can_frame out_frame;
    out_frame.can_id = frame->extended_can_id;
    out_frame.can_dlc = frame->payload_size;
    memcpy(out_frame.data, frame->payload, frame->payload_size);
    int rc = write(_can_sock, &out_frame, sizeof(out_frame));
    if (rc < 0) {
      perror("Error sending CAN frame");
    }
    canardTxPop(&can_node);
  }
}

void UavcanMapper::spin() {
  // Configure epoll
  int epoll = epoll_create(1);
  if (epoll < 0) {
    perror("epoll_create");
    return;
  }
  struct epoll_event event_setup;
  struct epoll_event epoll_events[1];
  event_setup.events = EPOLLIN;
  if (epoll_ctl(epoll, EPOLL_CTL_ADD, _can_sock, &event_setup)) {
    perror("epoll_ctl");
    return;
  }

  while (1) {
    int num_events = epoll_wait(epoll, epoll_events, 1, -1);
    if (num_events <= 0) {
      perror("epoll_wait");
      return;
    }

    while (num_events-- > 0) {
      // Lock the CAN socket mutex
      std::lock_guard<std::mutex> _lock(_can_sock_mtx);
      struct can_frame in_frame;

      int len = read(_can_sock, &in_frame, sizeof(in_frame));
      if (len < 0) {
        perror("Couldn't receive frame");
        return;
      }

      handle_frame(&in_frame);
    }
  }
}

static void* canard_alloc(CanardInstance* ins, size_t amount) {
  (void)ins;
  return malloc(amount);
}

static void canard_free(CanardInstance* ins, void* pointer) {
  (void)ins;
  free(pointer);
}
