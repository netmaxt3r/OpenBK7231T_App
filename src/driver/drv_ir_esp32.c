#include "../obk_config.h"

#if ENABLE_DRIVER_ESPIR

#include "drv_ir_esp32.h"
#include "../logging/logging.h"
#include "../new_pins.h"
#include "driver/rmt_rx.h"
#include "soc/soc_caps.h"
#include "../new_cfg.h"
#include "../mqtt/new_mqtt.h"
#include "../cmnds/cmd_public.h"

#define RMT_RX_QUEUE_SIZE 1
static QueueHandle_t rmt_rx_queue = NULL;

static bool IRAM_ATTR rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data) {
	BaseType_t high_task_wakeup = pdFALSE;
	QueueHandle_t receive_queue = (QueueHandle_t)user_data;
		// send the received RMT symbols to the parser task
	xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
	return high_task_wakeup == pdTRUE;
}

static rmt_symbol_word_t raw_signals[64];

static bool initialized = false;

static rmt_channel_handle_t rx_channel = NULL;
static rmt_receive_config_t rmt_rx_config = {
	.signal_range_min_ns = 1250,
	.signal_range_max_ns = 12000000,
};
rmt_rx_done_event_data_t rx_data;

void ESPIR_Init() {
	int pin = -1;
	pin = PIN_FindPinIndexForRole(IOR_IRRecv,pin);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: recv pinxx %i",pin);


	rmt_rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
	if (!rmt_rx_queue) {
		ADDLOG_ERROR(LOG_FEATURE_IR, "Failed to create RMT queue");
	}

	rmt_rx_channel_config_t rx_config = {
		.gpio_num = pin,
		.resolution_hz = 1000000, // 1MHz resolution
		.mem_block_symbols = 64,
		.intr_priority = 0,
		.clk_src = RMT_CLK_SRC_APB
	};
	int result = rmt_new_rx_channel(&rx_config, &rx_channel);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: rmt_new_rx_channel result %i",result);
	rmt_rx_event_callbacks_t cbs = {
		.on_recv_done = rx_done_callback,
	};
	result =  rmt_rx_register_event_callbacks(rx_channel, &cbs, rmt_rx_queue);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: rmt_rx_register_event_callbacks result %i",result);
	result = rmt_enable(rx_channel);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: rmt_enable result %i",result);
	result = rmt_receive(rx_channel, raw_signals, sizeof(raw_signals), &rmt_rx_config);
	ADDLOG_INFO(LOG_FEATURE_IR, (char *)"DRV_IR_Init: rmt_receive result %i",result);
	initialized = true;
}


static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

#define IR_NEC_DECODE_MARGIN 200
/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
	return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
	       (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
	return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
	       nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
	return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
	       nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}
static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
	return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
	       nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
	rmt_symbol_word_t *cur = rmt_nec_symbols;
	uint16_t address = 0;
	uint16_t command = 0;
	bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
	                          nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
	if (!valid_leading_code) {
		return false;
	}
	cur++;
	for (int i = 0; i < 16; i++) {
		if (nec_parse_logic1(cur)) {
			address |= 1 << i;
		} else if (nec_parse_logic0(cur)) {
			address &= ~(1 << i);
		} else {
			return false;
		}
		cur++;
	}
	for (int i = 0; i < 16; i++) {
		if (nec_parse_logic1(cur)) {
			command |= 1 << i;
		} else if (nec_parse_logic0(cur)) {
			command &= ~(1 << i);
		} else {
			return false;
		}
		cur++;
	}
	// save address and command
	s_nec_code_address = address;
	s_nec_code_command = command;
	return true;
}
static void on_nec_data(uint16_t address, uint8_t command, bool is_repeat)
{
	ADDLOG_INFO(LOG_FEATURE_IR,"NEC Address=%04X, Command=%02X", address, command);
	
#if ENABLE_MQTT
	char out[128];
	if(CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED)) {
		
		if (is_repeat) {
			snprintf(out, sizeof(out), "IR_NEC 0x%04X 0x%02X REPEAT", address, command);
		} else {
			snprintf(out, sizeof(out), "IR_NEC 0x%04X 0x%02X", address, command);
		}
		MQTT_PublishMain_StringString("ir",out, 0);
	}
	if (CFG_HasFlag(OBK_FLAG_IR_PUBLISH_RECEIVED_IN_JSON)) {
		snprintf(out, sizeof(out), "{\"IrReceived\":{\"Protocol\":\"%s\",\"address\":%i,\"command\":\"0x%i\"}}",
						"NEC", (int)address, (int)command);
		MQTT_PublishMain_StringString("RESULT", out, OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
				
	}
#endif
	EventHandlers_FireEvent3(CMD_EVENT_IR_NEC, address, command, is_repeat ? 1 : 0);
}
static void parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
	ADDLOG_DEBUG(LOG_FEATURE_IR,"NEC frame start---");
	for (size_t i = 0; i < symbol_num; i++) {
		ADDLOG_DEBUG(LOG_FEATURE_IR,"{%d:%d},{%d:%d}", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
		             rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
	}
	ADDLOG_DEBUG(LOG_FEATURE_IR,"---NEC frame end: ");
	// decode RMT symbols
	switch (symbol_num) {
	case 34: // NEC normal frame
		if (nec_parse_frame(rmt_nec_symbols)) {
			ADDLOG_DEBUG(LOG_FEATURE_IR,"Address=%04X, Command=%04X", s_nec_code_address, s_nec_code_command);
			on_nec_data(s_nec_code_address,(uint8_t) s_nec_code_command & 0xFF, false);
		}
		break;
	case 2: // NEC repeat frame
		if (nec_parse_frame_repeat(rmt_nec_symbols)) {
			ADDLOG_DEBUG(LOG_FEATURE_IR,"Address=%04X, Command=%04X, repeat", s_nec_code_address, s_nec_code_command);
			on_nec_data(s_nec_code_address,(uint8_t) s_nec_code_command & 0xFF, true);
		}
		break;
	default:
		ADDLOG_DEBUG(LOG_FEATURE_IR,"Unknown NEC frame");
		break;
	}
}
void ESPIR_RunFrame() {
	if(initialized) {
		if (xQueueReceive(rmt_rx_queue, &rx_data, portMAX_DELAY) == pdPASS) {
			ADDLOG_DEBUG(LOG_FEATURE_IR, "Received %u symbols", rx_data.num_symbols);
			parse_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
			int  result = rmt_receive(rx_channel, raw_signals, sizeof(raw_signals), &rmt_rx_config);
			ADDLOG_DEBUG(LOG_FEATURE_IR, (char *)"DRV_IR_RunFrame: rmt_receive result %i",result);
		}
	}
}

#endif

