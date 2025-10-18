#ifndef OBK_I2C_H
#define OBK_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef OBK_I2C_DEFAULT_TIMEOUT_MS
#define OBK_I2C_DEFAULT_TIMEOUT_MS  1000U
#endif

/* Status codes returned by HAL I2C functions */
typedef enum {
	I2C_OK = 0,
	I2C_ERROR,
	I2C_BUSY,
	I2C_TIMEOUT,
	I2C_INVALID_ARG,
} obk_i2c_status_t;

/* Common I2C speeds (in Hz). Driver should accept arbitrary Hz values as well. */
typedef enum {
	I2C_SPEED_STANDARD   = 100000U,  /* 100 kHz */
	I2C_SPEED_FAST       = 400000U,  /* 400 kHz */
	I2C_SPEED_FAST_PLUS  = 1000000U, /* 1 MHz */
} obk_i2c_speed_t;

/* Addressing mode */
typedef enum {
	I2C_ADDR_7BIT = 0,
	I2C_ADDR_10BIT,
} obk_i2c_addr_mode_t;

/*Master Bus*/
typedef struct {
	uint32_t default_timeout_ms;  
	uint8_t instance;                /* hardware instance / port index */
	//TODO gpio
	
} obk_i2c_master_cfg_t;
typedef struct obk_i2c_master_dev obk_i2c_master_dev_t;

obk_i2c_master_dev_t *obk_i2c_master_init(const obk_i2c_master_cfg_t *cfg);
obk_i2c_status_t obk_i2c_master_probe(obk_i2c_master_dev_t *dev, uint16_t address, int xfer_timeout_ms);
obk_i2c_status_t obk_i2c_master_deinit(obk_i2c_master_dev_t *dev);


/* Slave Device*/
typedef struct {
	uint8_t instance;                /* hardware instance / port index */
	uint32_t speed_hz;               /* bus speed in Hz */
	obk_i2c_addr_mode_t addr_mode;   /* 7-bit or 10-bit addressing */
	uint16_t address;
	uint32_t default_timeout_ms;     /* default timeout for blocking ops */
	void (*event_cb)(void *arg);     /* optional callback for async/event */
	void *cb_arg;                    /* user argument passed to callback */
} obk_i2c_slave_cfg_t;
typedef struct obk_i2c_slave_dev obk_i2c_slave_dev_t;

obk_i2c_slave_dev_t *obk_i2c_slave_init(const obk_i2c_slave_cfg_t *cfg);
obk_i2c_status_t obk_i2c_slave_deinit(obk_i2c_slave_dev_t *dev);

/* External Connected Device*/
typedef struct {
	uint32_t speed_hz;               /* bus speed in Hz */
	obk_i2c_addr_mode_t addr_mode;   /* 7-bit or 10-bit addressing */
	uint16_t address;
	uint32_t default_timeout_ms; 
	
	void (*event_cb)(void *arg);     /* optional callback for async/event */
	void *cb_arg;                    /* user argument passed to callback */
} obk_i2c_connected_device_cfg_t;
typedef struct obk_i2c_connected_dev obk_i2c_connected_dev_t;

obk_i2c_connected_dev_t *obk_i2c_connected_device_init(obk_i2c_master_dev_t *bus, const obk_i2c_connected_device_cfg_t *cfg);
obk_i2c_status_t obk_i2c_connected_device_deinit(obk_i2c_connected_dev_t *dev);


obk_i2c_status_t obk_i2c_write_to_device(obk_i2c_connected_dev_t *dev,
									  uint8_t *tx_buf,
									  size_t tx_len,
									  uint32_t timeout_ms);
obk_i2c_status_t obk_i2c_read_from_device(obk_i2c_connected_dev_t *dev,
									  uint8_t *rx_buf,
									  size_t rx_len,
									  uint32_t timeout_ms);
obk_i2c_status_t obk_i2c_write_read_from_device(obk_i2c_connected_dev_t *dev,
									  uint8_t *tx_buf,
									  size_t tx_len,
									  uint8_t *rx_buf,
									  size_t rx_len,
									  uint32_t timeout_ms);									  
//TODO Slave device



// i2c reg read/write 
obk_i2c_status_t obk_i2c_write_reg(obk_i2c_connected_dev_t *dev,
									uint8_t reg_addr,
									const uint8_t *data,
									size_t data_len,
									uint32_t timeout_ms);
obk_i2c_status_t obk_i2c_read_reg(obk_i2c_connected_dev_t *dev,
								   uint8_t reg_addr,
								   uint8_t *data,
								   size_t data_len,
								   uint32_t timeout_ms);	

#ifdef __cplusplus
}
#endif

#endif /* obk_I2C_H */