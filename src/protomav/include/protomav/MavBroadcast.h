#ifndef BROADCAST_HEADER
#define BROADCAST_HEADER
#include "MavMessenger.h"
#include <mavlink/v1.0/common/mavlink.h>
#include "vehicle.h"
namespace protomav{
  class MavBroadcast : public MavMessenger{
  public:
    MavBroadcast();
    void reached(int seq);
    void current(int seq);
    void send();
  private:
    mavlink_message_t out_msg;
  };
}
#endif