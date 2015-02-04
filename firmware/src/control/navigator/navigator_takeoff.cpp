#include <math.h>

#include "main.h"
#include "navigator.hpp"

/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
#define FAKE_TAKEOFF    TRUE

/*
 ******************************************************************************
 * EXTERNS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * PROTOTYPES
 ******************************************************************************
 */

/*
 ******************************************************************************
 * GLOBAL VARIABLES
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
 * Приблизительный алгоритм взлета:
 * - максимальный газ
 * - курс не имеет значения
 * - максимально допустимый тангаж
 * - нулевой крен
 * - держать до достижения заданной высоты
 * - по достижении заданной высоты, построить ЛЗП от текущего положения
 *   до следующего ППМ
 * Т.о. из всех данных ППМ takeoff используется только высота.
 *
 * На данный момент реализован тестовый код (FAKE_TAKEOFF), который
 * использует точку takeoff, как начало ЛЗП (конечная точка - следующий ППМ)
 */

/**
 *
 */
navigator_status_t Navigator::loop_takeoff(void){

#if FAKE_TAKEOFF
  broadcast_mission_item_reached(mi.seq);
  what_to_do_here();
  return NAVIGATOR_STATUS_DONE;
#else
#error "Sophisticated taking off functionality unrealized"
#endif /* FAKE_TAKEOFF */
}






