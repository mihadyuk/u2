#include <iostream>
#include <fstream>
#include "stdint.h"

#include "../../firmware/lib/mavlink/C/lapwing/mavlink.h"

using namespace std;
static mavlink_message_t rx_msg;
static mavlink_status_t rx_status;

static mavlink_navi6d_debug_input_t   dbg_in_struct;
static mavlink_navi6d_debug_output_t  dbg_out_struct;


int main () {
  char byte;
  ifstream log;
  log.open ("log.raw", ios::binary);

  while (! log.eof()) {
    log.read(&byte, 1);
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_msg, &rx_status)) {
      if (MAVLINK_MSG_ID_NAVI6D_DEBUG_INPUT == rx_msg.msgid) {
        mavlink_msg_navi6d_debug_input_decode(&rx_msg, &dbg_in_struct);
      }
    }
  }

  log.close();
  return 0;
}
