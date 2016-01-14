#ifndef CLI_CMD_H_
#define CLI_CMD_H_

thread_t* ps_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* uname_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* clear_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* list_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* loop_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* echo_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* reboot_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* sleep_clicmd(int argc, const char * const * argv, BaseChannel *sdp);
thread_t* selftest_clicmd(int argc, const char * const * argv, BaseChannel *sdp);

#endif /* CLI_CMD_H_ */
