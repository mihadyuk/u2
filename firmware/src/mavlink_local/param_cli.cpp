#include <stdio.h>

#include "main.h"
#include "param_registry.hpp"
#include "global_flags.h"
#include "cli.hpp"
#include "mavlink_local.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 ******************************************************************************
 * LOCAL FUNCTIONS
 ******************************************************************************
 ******************************************************************************
 */
/*
 * confirmation of changes
 */
static void confirm(ParamStatus status){
  if (status == ParamStatus::OK)
    return;
    //cli_println("Success");
  else if (status == ParamStatus::CLAMPED)
    cli_println("WARNING: value clamped to safety limits.");
  else if (status == ParamStatus::NOT_CHANGED)
    cli_println("WARNING: value not changed.");
  else if (status == ParamStatus::INCONSISTENT)
    cli_println("ERROR: value inconsistent.");
  else
    cli_println("ERROR: Unhandled error.");
}

/**
 *
 */
static void print(const uavparam_t *param_p, bool need_help) {
  int n = 80;
  int nres = 0;
  char str[n];

  if (nullptr == param_p){
    cli_println("Something goes too bad with param registry");
    chThdSleepMilliseconds(50);
    return;
  }

  nres = snprintf(str, n, "%-17s", param_p->name);
  cli_print_long(str, n, nres);

  switch(param_p->param_type){
  case MAVLINK_TYPE_FLOAT:
    nres = snprintf(str, n, " %-15f %-15f %-15f",
        (double)param_p->min.f32,
        (double)param_p->valuep->f32,
        (double)param_p->max.f32);
    break;
  case MAVLINK_TYPE_INT32_T:
    nres = snprintf(str, n, " %-15d %-15d %-15d",
        (int)param_p->min.i32,
        (int)param_p->valuep->i32,
        (int)param_p->max.i32);
    break;
  default: // uint32_t
    nres = snprintf(str, n, " %-15u %-15u %-15u",
        (unsigned int)param_p->min.u32,
        (unsigned int)param_p->valuep->u32,
        (unsigned int)param_p->max.u32);
    break;
  }

  cli_print_long(str, n, nres);
  cli_print(ENDL);

  if (need_help && (param_p->help != NULL)){
    cli_println("");
    cli_println(param_p->help);
  }
}

/**
 *
 */
static void dump(size_t i) {

  int n = 80;
  int nres = 0;
  char str[n];
  const uavparam_t *param_p = param_registry.idx2ptr(i);
  if (nullptr == param_p){
    cli_println("Something goes too bad with param registry");
    chThdSleepMilliseconds(50);
    return;
  }

  nres = snprintf(str, n, "%s%s", "param ", param_p->name);
  cli_print_long(str, n, nres);

  switch(param_p->param_type){
  case MAVLINK_TYPE_FLOAT:
    nres = snprintf(str, n, " %-15f", (double)param_p->valuep->f32);
    break;
  case MAVLINK_TYPE_INT32_T:
    nres = snprintf(str, n, " %-15d", (int)param_p->valuep->i32);
    break;
  default: // uint32_t
    nres = snprintf(str, n, " %-15u",(unsigned int)param_p->valuep->u32);
    break;
  }
  cli_print_long(str, n, nres);
  cli_print(ENDL);
}

/**
 *
 */
static void print_header(void){
  cli_println("Name            min             value           max");
  cli_println("--------------------------------------------------------------");
}

/**
 *
 */
static void print_footer(void){
  int N = 80;
  char str[N];

  cli_println("--------------------------------------------------------------");
  snprintf(str, N, "Total/used: %d/%d", param_registry.capacity(), param_registry.paramcnt());
  cli_println(str);
}

/**
 *
 */
static void print_all(void){
  size_t paramcnt = param_registry.paramcnt();

  print_header();
  for (size_t i=0; i<paramcnt; i++) {
    print(param_registry.idx2ptr(i), false);
  }
  print_footer();
}

/**
 *
 */
static void dump_all(void){
  for (size_t i=0; i<param_registry.paramcnt(); i++) {
    dump(i);
  }
}

/**
 *
 */
static ParamStatus set(const char *val, const uavparam_t *param_p) {
  param_union_t v;
  int sscanf_status;

  if (nullptr == param_p) {
    cli_println("Something goes too bad with param registry");
    chThdSleepMilliseconds(50);
    return ParamStatus::UNKNOWN_ERROR;
  }

  switch(param_p->param_type) {
  case MAVLINK_TYPE_FLOAT:
    sscanf_status = sscanf(val, "%f", &v.f32);
    break;

  case MAVLINK_TYPE_INT32_T:
    sscanf_status = sscanf(val, "%i", (int*)&v.i32);
    break;

  default: // uint32_t
    sscanf_status = sscanf(val, "%u", (unsigned int*)&v.u32);
    break;
  }

  if (sscanf_status != 1)
    return ParamStatus::INCONSISTENT;
  else
    return param_registry.setParam(&v, param_p);
}

/**
 *
 */
static void help(void){
  cli_println("Run without parameters to get full parameters list.");
  cli_println("'param save' to save parameters to EEPROM.");
  cli_println("'param dump' to print parameters in form suitable for copy+paste.");
  cli_println("'param help' to get this message.");
  cli_println("'param PARAM_name' to get value of parameter.");
  cli_println("'param PARAM_name N' to set value of parameter to N.");
}

/*
 ******************************************************************************
 * EXPORTED FUNCTIONS
 ******************************************************************************
 */

/**
 * Working with parameters from CLI.
 */
thread_t* param_clicmd(int argc, const char * const * argv, BaseChannel *bchnp){
  (void)bchnp;

  const uavparam_t *param_p;
  ParamStatus status;

  /* wait until value uninitialized (just to be safe) */
  while (GlobalFlags.parameters_loaded != 1)
    chThdSleepMilliseconds(10);

  /* no arguments */
  if (argc == 0)
    print_all();

  /* one argument */
  else if (argc == 1){
    if (strcmp(*argv, "help") == 0)
      help();
    else if (strcmp(*argv, "dump") == 0)
      dump_all();
    else if (strcmp(*argv, "save") == 0){
      cli_print("Please wait. Saving to EEPROM... ");
      param_registry.saveAll();
      cli_println("Done.");
    }
    else{
      param_p = param_registry.search(*argv);
      if (nullptr != param_p) {
        print_header();
        print(param_p, true);
      }
      else{
        cli_println("ERROR: unknown parameter name.");
      }
    }
  }

  /* two arguments */
  else if (argc == 2){
    param_p = param_registry.search(argv[0]);
    if (nullptr != param_p) {
      status = set(argv[1], param_p);
      confirm(status);
    }
    else{
      cli_println("ERROR: unknown parameter name.");
    }
  }
  else{
    cli_println("ERROR: bad arguments. Try 'param help'.");
  }

  /* stub */
  return NULL;
}

