#ifndef SHELL_MANAGER_H_
#define SHELL_MANAGER_H_

/**
 *
 */
typedef enum {
  SHELL_MANAGER_UNINIT = 0,
  SHELL_MANAGER_OVER_RF = 1,
  SHELL_MANAGER_OVER_USB = 2,
  SHELL_MANAGER_STOP = 3
} shellmgr_state;

/**
 *
 */
class ShellManager{
public:
  ShellManager(void);
  void start(void);
  void stop();
private:
  Thread *worker;
  shellmgr_state state;
};

#endif /* SHELL_MANAGER_H_ */
