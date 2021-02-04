#ifndef MAPPER_H
#define MAPPER_H

#include <ros/ros.h>
#include "canard.h"

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


/* -------- Convenience Macros -------- */
// TODO error handling
#define TX_TRANSFER(mapper, type, msg, _transfer_id, _port_id, _priority) {\
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
  mapper->send_transfer(&_xfer); \
}

// TODO error handling
#define RX_TRANSFER(type, xfer, out_msg) {\
  size_t _deser_size = xfer->payload_size; \
  type ## _deserialize_(&out_msg, static_cast<const uint8_t*>(xfer->payload), &_deser_size); \
}

/* -------- Main mapper helpers -------- */
// TODO improve visibility constraints
/// Internal subscription identifier.
struct Can2Ros {
  CanardPortID port_id;
  std::function<void(CanardTransfer*)> callback;
  CanardRxSubscription subscription;
};

class UavcanMapper {
  public:
    UavcanMapper(CanardNodeID can_id, std::string can_iface);
    void map_can2ros(CanardPortID port_id, std::function<void(CanardTransfer*)> callback);
    void send_transfer(CanardTransfer* xfer);

    void spin();

    void lock() { _can_sock_mtx.lock(); }
    void unlock() { _can_sock_mtx.unlock(); }

  private:
    CanardInstance can_node;
    ros::NodeHandle ros_node;

    int _can_sock;
    std::mutex _can_sock_mtx;

    std::vector<Can2Ros> can2ros;

    void handle_frame(struct can_frame* in_frame);
};

/* -------- libcanard wrapper functions -------- */

static void* canard_alloc(CanardInstance* ins, size_t amount);
static void canard_free(CanardInstance* ins, void* pointer);

/* -------- Helper functions -------- */

#endif // MAPPER_H