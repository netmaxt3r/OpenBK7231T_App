#include "../obk_config.h"

#if ENABLE_DRIVER_I2C
#include "drv_i2c.h"
#include "../logging/logging.h"
#include "../hal/hal_i2c.h"
#include "../cmnds/cmd_public.h"


static obk_i2c_master_dev_t *i2c_dev = NULL;


static commandResult_t I2C_CmdHandler_probe(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	if (!i2c_dev) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c: not initialized");
		return CMD_RES_ERROR;
	}
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: %s   <devaddr_hex> ", cmd);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	

	const char* address = Tokenizer_GetArgFrom(0);
	int devaddr = (int)strtol(address, NULL, 16);
	obk_i2c_status_t rc = obk_i2c_master_probe(i2c_dev, (uint8_t)devaddr,0);
	if (rc == I2C_OK) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "i2c probe: dev=0x%02X found ", devaddr);
	} else {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c probe: dev=0x%02X not found rc=%d", devaddr, rc);
		return CMD_RES_ERROR;
	}
	return CMD_RES_OK;
}


static commandResult_t I2C_CmdHandler_read(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	if (!i2c_dev) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c: not initialized");
		return CMD_RES_ERROR;
	}
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: %s <devaddr_hex> <reg_hex> [len_dec]", cmd);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	const char* dev_s = Tokenizer_GetArgFrom(0);
	const char* reg_s = Tokenizer_GetArgFrom(1);
	int devaddr = (int)strtol(dev_s, NULL, 16);
	int reg = (int)strtol(reg_s, NULL, 16);
	int len = 1;
	if (Tokenizer_GetArgsCount() >= 4) {
		const char* len_s = Tokenizer_GetArgFrom(2);
		len = (int)strtol(len_s, NULL, 0);
		if (len <= 0) len = 1;
	}
	if (len > 64) len = 64;

	ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c read: device dev=0x%02X from %s", devaddr, dev_s);
	obk_i2c_connected_device_cfg_t c = {
		.address = (uint8_t)devaddr,
		.speed_hz = 100000
	};
	obk_i2c_connected_dev_t* dev = obk_i2c_connected_device_init(i2c_dev, &c);
	if (dev == NULL) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c read: failed to init connected device dev=0x%02X", devaddr);
		return CMD_RES_ERROR;
	}
	uint8_t buf[64];
	obk_i2c_status_t rc = obk_i2c_read_reg(dev, (uint8_t)reg, buf, (size_t)len,0);
	if (rc == I2C_OK) {
		// print returned bytes as hex
		char sbuf[3 * 64 + 1];
		char *p = sbuf;
		for (int i = 0; i < len; ++i) {
			sprintf(p, "%02X ", buf[i]);
			p += 3;
		}
		*p = '\0';
		ADDLOG_INFO(LOG_FEATURE_CMD, "i2c read: dev=0x%02X reg=0x%02X len=%d data=%s", devaddr, reg, len, sbuf);
	} else {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c read failed rc=%d", rc);
		return CMD_RES_ERROR;
	}
	return CMD_RES_OK;
}


static commandResult_t I2C_CmdHandler_write(const void *context, const char *cmd, const char *args, int cmdFlags)
{
	if (!i2c_dev) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c: not initialized");
		return CMD_RES_ERROR;
	}

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "Usage: %s <devaddr_hex> <reg_hex> <value_hex>", cmd);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	const char* dev_s = Tokenizer_GetArgFrom(0);
	const char* reg_s = Tokenizer_GetArgFrom(1);
	int devaddr = (int)strtol(dev_s, NULL, 16);
	int reg = (int)strtol(reg_s, NULL, 16);
	
	const char* val_s = Tokenizer_GetArgFrom(2);
	int value = (int)strtol(val_s, NULL, 16);
	uint8_t v = (uint8_t)value;

	ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c write: device dev=0x%02X reg=0x%02X value=0x%02X", devaddr, reg, v);
	obk_i2c_connected_device_cfg_t c = {
		.address = (uint8_t)devaddr,
		.speed_hz = 100000
	};
	obk_i2c_connected_dev_t* dev = obk_i2c_connected_device_init(i2c_dev, &c);
	if (dev == NULL) {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c read: failed to init connected device dev=0x%02X", devaddr);
		return CMD_RES_ERROR;
	}

	obk_i2c_status_t rc = obk_i2c_write_reg(dev, (uint8_t)reg, &v, 1,0);
	if (rc == I2C_OK) {
		ADDLOG_INFO(LOG_FEATURE_CMD, "i2c write: dev=0x%02X reg=0x%02X value=0x%02X", devaddr, reg, v);
	} else {
		ADDLOG_ERROR(LOG_FEATURE_CMD, "i2c write failed rc=%d", rc);
		return CMD_RES_ERROR;
	}
	return CMD_RES_OK;
}

void I2C_Init()
{
	ADDLOG_ERROR(LOG_FEATURE_I2C, "I2C drive init");
	// Initialization code for I2C driver can be added here if needed
	obk_i2c_master_cfg_t i2c_cfg = {	};
	i2c_dev = obk_i2c_master_init(&i2c_cfg);
	if (i2c_dev == NULL) {
		ADDLOG_ERROR(LOG_FEATURE_I2C, "I2C_Init: Failed to initialize I2C device");
	} else {
		ADDLOG_INFO(LOG_FEATURE_I2C, "I2C_Init: I2C device initialized successfully");
	}
	CMD_RegisterCommand("i2c_probe", I2C_CmdHandler_probe, NULL);
	CMD_RegisterCommand("i2c_read_reg", I2C_CmdHandler_read, NULL);
	CMD_RegisterCommand("i2c_write_reg", I2C_CmdHandler_write, NULL);
	ADDLOG_ERROR(LOG_FEATURE_I2C, "I2C drive init complete");
}
#endif

