// Temporary testing node
#include <ros/ros.h>
#include "canard.h"

#include "spear/drive/DriveCommand_1_0.h"

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

#include <chrono>

static void* canard_alloc(CanardInstance* ins, size_t amount) {
  (void)ins;
  return malloc(amount);
}

static void canard_free(CanardInstance* ins, void* pointer) {
  (void)ins;
  free(pointer);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_sender");
    ros::NodeHandle n;

    int can_sock;
    if ((can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error openning CAN socket");
        return 1;
    }

    struct ifreq can_ifr;
    strcpy(can_ifr.ifr_name, "vcan0");
    ioctl(can_sock, SIOCGIFINDEX, &can_ifr);
    struct sockaddr_can can_addr;
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = can_ifr.ifr_ifindex;

    bind(can_sock, (struct sockaddr *)&can_addr, sizeof(can_addr));

    CanardInstance canard = canardInit(canard_alloc, canard_free);
    canard.node_id = 20;

    CanardTransferID transfer_id = 0;

    while (1) {
        using namespace std::chrono;
        spear_drive_DriveCommand_1_0 cmd{};
        cmd.id.value = 2;
        cmd.speed = 12.0;

        uint8_t buf[20];
        size_t size = 10;
        spear_drive_DriveCommand_1_0_serialize_(&cmd, buf, &size);

        CanardTransfer xfer{};
        xfer.timestamp_usec = milliseconds(duration_cast<milliseconds>(steady_clock::now().time_since_epoch())).count();
        xfer.priority = CanardPriorityNominal;
        xfer.transfer_kind = CanardTransferKindMessage;
        xfer.port_id = 0;
        xfer.remote_node_id = CANARD_NODE_ID_UNSET;
        xfer.transfer_id = transfer_id;
        xfer.payload = buf;
        xfer.payload_size = size;

        ROS_INFO("transfer_id: %d", transfer_id);

        transfer_id += 1;

        canardTxPush(&canard, &xfer);

        auto* canard_frame = canardTxPeek(&canard);
        canardTxPop(&canard);
        struct can_frame out_frame{};
        out_frame.can_id = canard_frame->extended_can_id;
        out_frame.can_dlc = canard_frame->payload_size;
        memcpy(out_frame.data, canard_frame->payload, canard_frame->payload_size);

        write(can_sock, &out_frame, sizeof(struct can_frame));

        sleep(1);
    }
}
