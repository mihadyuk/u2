#ifndef ZANZOTTERA_HPP_
#define ZANZOTTERA_HPP_

#include "impact.hpp"

#define ZANZ_PARAMS_CINJ_OFFSET         0
#define ZANZ_PARAMS_THROTTLE_OFFSET     8
#define ZANZ_PARAMS_RPM_LIMIT_OFFSET    10

/**
 * @brief   Status of operation returned by VOLZ servo
 */
typedef enum {
  ZANZ_STATUS_OK = 0,
  ZANZ_STATUS_WRONG_RESPONCE,
  ZANZ_STATUS_BAD_CRC,
  ZANZ_STATUS_INCOMPLETE_RESPONCE,
  ZANZ_STATUS_UNKNOWN_ERROR,
  ZANZ_STATUS_NO_RESPONCE,
  ZANZ_STATUS_ENUM_END,
}zanz_status_t;

/**
 *
 */
typedef struct arnConfiguration {
  uint16_t Length;          // size of this structure
  uint16_t RealtimepackLen; // size of arnRealTimePacket_t structure
  uint16_t CodesLen;        // size of arnCodesSTR_t structure
  uint16_t ParameterLen;    // size of Mappa_t structure
  uint16_t ToolsParamsLen;  // size of arnRealTimeParams_t structure
  uint16_t PasswordLen;     // size if the password to access infos
  uint16_t TextPagesLen;    // size of plain text infos
  uint16_t DiagnosiInfoLen; // size of Diagnosi_t structure
  uint32_t pRealtimePack;   // address of arnRealTimePacket_t structure
  uint32_t pCodes;          // address of arnCodesSTR_t structure
  uint32_t pParams;         // address of Mappa_t structure
  uint32_t pToolsParams;    // address of arnRealTimeParams_t structure
  uint32_t pPassword;       // not yet implemented
  uint32_t pTextPages;      // not yet implemented
  uint32_t pDiagnosiInfo;   // address of Diagnosi_t structure
} arnConfiguration_t;

/**
 *
 */
typedef struct arnRealTimePacket {
  uint16_t rev_counter; // Engine rev. counter (1 eng. revol. per Bit)
  uint16_t rpm;         // Rpm value (1 rpm per Bit)
  uint16_t injection[2];// Injection time (1 microsec. per Bit)
  uint16_t spark[4];    // Spark time (0.25 Degree per Bit )
  uint16_t phase;       // Injection phase (0.1 Degree per Bit)
  uint16_t throttle;    // Throttle position ( 0.1% per Bit )
  uint16_t barometric;  // Barometric ( 1 mBar per Bit )
  uint16_t vbatt;       // Battery voltage ( 0 is 0.65V – 1023 is 17.99 V )
  uint8_t  air_temp;    // Air temperature (0.1 °C per bit)
  uint8_t  triax;       // Triax accelerometer alarm value
  uint8_t  alt_temp;    // alternator temp. (0 is -30 °C – 255 is 129.75 °C)
  uint8_t  free1;       // for future use
  uint16_t duty_pump;   // Fuel pump command Duty (0.1 % per bit)
} arnRealTimePacket_t;

/**
 *
 */
typedef struct Diagnostic {
uint8_t ignition[4];
uint8_t smot;
uint8_t thr_b;
uint8_t injection[2];
uint8_t free;// for future use
} Diagnostic_t;

/**
 *
 */
typedef struct arnRealTimeParams{
  uint8_t  cinj[2];     // injection correction (range 0- 1,992 )
  uint8_t  cant[4];     // spark correction (range –32 – 31,75 degrees)
  uint8_t  cphase;       // phase correction (range –127 128 degrees)
  uint8_t  check_flag;  // 1 to set puel pump command; 0 otherwise
  uint16_t throttle_set;// throttle reference (range 0_100% 0-1000 bit)
  uint16_t rpm_limiter; // one bit per rpm (**)
} arnRealTimeParams_t ;
/* (**) Default value is 6800rpm, in order to use throttle with free revolutions instead of
governor please send 7000rpm command every time you power on the ECU.
If you leave the default governor value during the flight or ground test, when the propeller
will pass the 6800rpm, the ECU will close automatically the throttle in order to maintain
6800 RPM. */

/**
 *
 */
class Zanzottera: public BaseStaticThread<1024>{
public:
  Zanzottera(void);
  msg_t main(void);
  bool setIdle(bool state);
  bool isIdle();

private:
  msg_t main_impl(void);
  uint8_t *__comm_prologue(void);
  bool __comm_epilogue(uint8_t *bp);
  bool set_thrust_hystorical_sample(uint16_t hexval);
  bool quad_damage(uint16_t hexval);
  bool dbg_quad_damage(void);
  bool set_thrust(uint16_t hexval);
  zanz_status_t read_data(uint32_t addr, uint16_t data_size);
  zanz_status_t read_rt_packet(void);
  zanz_status_t read_diagnostic(void);
  zanz_status_t read_rt_params(void);
  void dissect_rt_packet(void);
  void dissect_configuration(void);
  void dissect_diagnostic(void);
  void dissect_rt_params(void);
  void telemetry_to_mavlink(zanz_status_t status);
  void diagnostic_to_mavlink(zanz_status_t status);
  void rt_params_to_mavlink(zanz_status_t status);
  uint16_t thrust2hex(float thrust);
  uint8_t checksum(const uint8_t *buf, size_t len);

  arnConfiguration_t arn_cfg;
  arnRealTimePacket_t arn_rt_packet;
  arnRealTimeParams_t arn_rt_params;
  Diagnostic_t diag;
  SerialDriver *sdp;
  uint8_t buf[sizeof(arnConfiguration_t) * 2 + 3 + 1]; /* size of config struct + header + checksum */
  bool idle;
  bool idleStatus;
  uint16_t health[ZANZ_STATUS_ENUM_END];
};


#endif /* ZANZOTTERA_HPP_ */
