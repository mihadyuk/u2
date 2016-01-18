#ifndef CLI_HPP_
#define CLI_HPP_

#include "microrl.h"

/**
 * Shell command structure
 */
typedef struct ShellCmd_t ShellCmd_t;

/**
 * Fucnction executing command job.
 * @return    Pointer to created thread. Must be NULL if no thread created.
 */
typedef thread_t* (*cmdfunction_t)(int argc, const char * const * argv, BaseChannel *bchnp);

/**
 *
 */
struct ShellCmd_t {
  /**
   * Printable command name. Must be zero terminated string without spaces
   */
  const char *name;
  /**
   * Function bounded to command
   */
  const cmdfunction_t func;
  /**
   * Short command description for help message.
   */
  const char *help;
};

void cli_print(const char *str);
void cli_print(int var);
void cli_print(unsigned int var);
void cli_print(double var);
void cli_put(char chr);
void cli_println(const char *str);
char get_char (void);
void cli_print_long(const char * str, int n, int nres);

void CliInit(void);
void SpawnShellThreads(void *bchnp);
void KillShellThreads(void);

#endif /* CLI_HPP_ */
