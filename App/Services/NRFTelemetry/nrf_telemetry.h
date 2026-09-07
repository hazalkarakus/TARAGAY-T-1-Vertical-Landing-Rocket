#ifndef APP_SERVICES_NRF_TELEMETRY_H
#define APP_SERVICES_NRF_TELEMETRY_H

#include <stdint.h>

#define NRF_TELEMETRY_PACKET_SIZE       32U
#define NRF_TELEMETRY_PAYLOAD_SIZE      20U
#define NRF_TELEMETRY_MAGIC             0x5AU
#define NRF_TELEMETRY_PROTOCOL_VERSION  1U
#define NRF_TELEMETRY_PAGE_COUNT        19U
#define NRF_TELEMETRY_FAST_TYPE         0xFCU
#define NRF_TELEMETRY_FAST_PER_DETAIL   7U

void NRFTelemetry_Init(void);
void NRFTelemetry_Service(void);

extern volatile uint8_t nrf_tlm_page_id;
extern volatile uint8_t nrf_tlm_sequence;
extern volatile uint32_t nrf_tlm_schedule_count;
extern volatile uint32_t nrf_tlm_tx_start_count;
extern volatile uint32_t nrf_tlm_tx_success_count;
extern volatile uint32_t nrf_tlm_tx_fail_count;
extern volatile uint32_t nrf_tlm_pending_replace_count;
extern volatile uint32_t nrf_tlm_last_tx_duration_us;
extern volatile uint32_t nrf_tlm_max_tx_duration_us;
extern volatile uint32_t nrf_tlm_dup_schedule_count;
extern volatile uint32_t nrf_tlm_dup_tx_start_count;
extern volatile uint32_t nrf_tlm_dup_tx_success_count;
extern volatile uint32_t nrf_tlm_dup_tx_fail_count;
extern volatile uint32_t nrf_tlm_dup_cancel_count;

#endif
