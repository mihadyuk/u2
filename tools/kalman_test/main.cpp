#include <iostream>
#include <fstream>
#include "stdint.h"

#include "../../firmware/lib/mavlink/C/lapwing/mavlink.h"
#include "navi6d_wrapper.hpp"
#include "param_registry.hpp"

using namespace std;
static mavlink_message_t rx_msg;
static mavlink_status_t rx_status;

static mavlink_navi6d_debug_input_t   dbg_in_struct;
static mavlink_navi6d_debug_output_t  dbg_out_struct;

Navi6dWrapper navi6d;
ParamRegistry param_registry("../../qgc_stuff/parameters/parameters_small.txt");

int main () {
  bool in_recvd = false;
  bool out_recvd = false;

  navi6d.start();

  char byte;
  ifstream log;
  log.open ("../log.raw", ios::binary);
  if (! log.is_open()) {
    throw std::exception(); // file not found
  }

  while (! log.eof()) {
    log.read(&byte, 1);
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
      switch (rx_msg.msgid) {
      case MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT:
        mavlink_msg_navi6d_debug_input_decode(&rx_msg, &dbg_in_struct);
        in_recvd = true;
        break;
      case MAVLINK_MSG_ID_NAVI6D_DEBUG_OUTPUT:
        mavlink_msg_navi6d_debug_output_decode(&rx_msg, &dbg_out_struct);
        out_recvd = true;
        break;
      default:
        // just drop message
        break;
      }

      // run test when both received
      if (in_recvd && out_recvd) {
        navi6d.update(dbg_in_struct, dbg_out_struct);
        in_recvd = false;
        out_recvd = false;
      }
    }
  }

  navi6d.stop();
  log.close();

  return 0;
}

