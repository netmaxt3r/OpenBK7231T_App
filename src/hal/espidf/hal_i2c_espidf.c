#if PLATFORM_ESPIDF 
#include "../hal_i2c.h"
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifndef HAL_I2C_DEFAULT_SDA_GPIO
#define HAL_I2C_DEFAULT_SDA_GPIO 21
#endif
#ifndef HAL_I2C_DEFAULT_SCL_GPIO
#define HAL_I2C_DEFAULT_SCL_GPIO 22
#endif

/* Simple mapping from esp_err_t -> obk_i2c_status_t */
static obk_i2c_status_t esp_to_obk_status(esp_err_t e)
{
    if (e == ESP_OK) return I2C_OK;
    if (e == ESP_ERR_INVALID_ARG) return I2C_INVALID_ARG;
    if (e == ESP_ERR_TIMEOUT) return I2C_TIMEOUT;
    return I2C_ERROR;
}



static TickType_t _ms_to_ticks(uint32_t ms)
{
    TickType_t t = (ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS;
    if (t == 0) t = 1;
    return t;
}

/* Internal device structure */

struct obk_i2c_master_dev {
    i2c_port_t port;
    obk_i2c_master_cfg_t cfg;
	i2c_master_bus_handle_t handle;
    SemaphoreHandle_t lock;
    volatile bool busy;
};

static uint32_t _bus_get_timeout_ms(obk_i2c_master_dev_t *dev, uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        if (dev && dev->cfg.default_timeout_ms) return dev->cfg.default_timeout_ms;
        return OBK_I2C_DEFAULT_TIMEOUT_MS;
    }
    return timeout_ms;
}

obk_i2c_master_dev_t *obk_i2c_master_init(const obk_i2c_master_cfg_t *cfg){
	if (!cfg) return NULL;
	if (cfg->instance > (uint8_t)I2C_NUM_MAX) return NULL;

	obk_i2c_master_dev_t *dev = (obk_i2c_master_dev_t *)calloc(1, sizeof(obk_i2c_master_dev_t));
	if (!dev) return NULL;

	dev->port = (i2c_port_t)cfg->instance;
	memcpy(&dev->cfg, cfg, sizeof(dev->cfg));
	dev->busy = false;
	dev->lock = xSemaphoreCreateMutex();
	if (!dev->lock) {
		free(dev);
		return NULL;
	}
 	i2c_master_bus_config_t conf = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = dev->port,
		.sda_io_num = HAL_I2C_DEFAULT_SDA_GPIO,
		.scl_io_num = HAL_I2C_DEFAULT_SCL_GPIO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,

	};
	i2c_master_bus_handle_t bus_handle;
	esp_err_t r = i2c_new_master_bus(&conf, &bus_handle);
	if (r != ESP_OK) {
		vSemaphoreDelete(dev->lock);
		free(dev);
		return NULL;
	}
	dev->handle = bus_handle;

	return dev;
}

obk_i2c_status_t obk_i2c_master_probe(obk_i2c_master_dev_t *dev, uint16_t address, int timeout_ms) {
	if (xSemaphoreTake(dev->lock, _ms_to_ticks(_bus_get_timeout_ms(dev, timeout_ms))) != pdTRUE) {
        return I2C_BUSY;
    }
    dev->busy = true;
	esp_err_t r = i2c_master_probe(dev->handle,address, _ms_to_ticks(_bus_get_timeout_ms(dev, timeout_ms)));
  	dev->busy = false;
    xSemaphoreGive(dev->lock);
    return esp_to_obk_status(r);	
}
obk_i2c_status_t obk_i2c_master_deinit(obk_i2c_master_dev_t *dev) {
	if (!dev) return I2C_INVALID_ARG;
	i2c_master_bus_reset(dev->handle);
	i2c_del_master_bus(dev->handle);
	if (dev->lock) vSemaphoreDelete(dev->lock);
	free(dev);
	return I2C_OK;
}

struct obk_i2c_slave_dev {
    i2c_port_t port;
    obk_i2c_slave_cfg_t cfg;
    SemaphoreHandle_t lock;
    volatile bool busy;
	i2c_slave_dev_handle_t handle;
};

static uint32_t _slave_get_timeout_ms(obk_i2c_slave_dev_t *dev, uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        if (dev && dev->cfg.default_timeout_ms) return dev->cfg.default_timeout_ms;
        return OBK_I2C_DEFAULT_TIMEOUT_MS;
    }
    return timeout_ms;
}

obk_i2c_slave_dev_t *obk_i2c_slave_init(const obk_i2c_slave_cfg_t *cfg){
	if (!cfg) return NULL;
	if (cfg->instance > (uint8_t)I2C_NUM_MAX) return NULL;

	obk_i2c_slave_dev_t *dev = (obk_i2c_slave_dev_t *)calloc(1, sizeof(obk_i2c_slave_dev_t));
	if (!dev) return NULL;

	dev->port = (i2c_port_t)cfg->instance;
	memcpy(&dev->cfg, cfg, sizeof(dev->cfg));
	dev->busy = false;
	dev->lock = xSemaphoreCreateMutex();
	if (!dev->lock) {
		free(dev);
		return NULL;
	}
	i2c_slave_config_t slave_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = dev->port,
		.sda_io_num = HAL_I2C_DEFAULT_SDA_GPIO,
		.scl_io_num = HAL_I2C_DEFAULT_SCL_GPIO,
		.slave_addr = cfg->address,
		.addr_bit_len = cfg->addr_mode == I2C_ADDR_7BIT ? I2C_ADDR_BIT_LEN_7 : I2C_ADDR_BIT_LEN_10,
		.send_buf_depth = 128
	};
	i2c_slave_dev_handle_t slave_handle;
	esp_err_t r = i2c_new_slave_device(&slave_config, &slave_handle);
	if (r != ESP_OK) {
		vSemaphoreDelete(dev->lock);
		free(dev);
		return NULL;
	}
	dev->handle = slave_handle;

	return dev;
}


obk_i2c_status_t obk_i2c_slave_deinit(obk_i2c_slave_dev_t *dev){
	if (!dev) return I2C_INVALID_ARG;
	i2c_del_slave_device(dev->handle);
	if (dev->lock) vSemaphoreDelete(dev->lock);
	free(dev);
	return I2C_OK;
}

struct obk_i2c_connected_dev {
    obk_i2c_master_dev_t *bus;
    obk_i2c_connected_device_cfg_t cfg;
    SemaphoreHandle_t lock;
    volatile bool busy;
	i2c_master_dev_handle_t handle;
};

static uint32_t _device_get_timeout_ms(obk_i2c_connected_dev_t *dev, uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        if (dev && dev->cfg.default_timeout_ms) return dev->cfg.default_timeout_ms;
        return OBK_I2C_DEFAULT_TIMEOUT_MS;
    }
    return timeout_ms;
}

obk_i2c_connected_dev_t *obk_i2c_connected_device_init(obk_i2c_master_dev_t *bus, const obk_i2c_connected_device_cfg_t *cfg){
	if (!cfg) return NULL;
	if (!bus) return NULL;
	
	obk_i2c_connected_dev_t *dev = (obk_i2c_connected_dev_t *)calloc(1, sizeof(obk_i2c_connected_dev_t));
	if (!dev) return NULL;
	dev->bus = bus;
	memcpy(&dev->cfg, cfg, sizeof(dev->cfg));
	dev->busy = false;
	dev->lock = xSemaphoreCreateMutex();
	if (!dev->lock) {
		free(dev);
		return NULL;
	}
	i2c_device_config_t dev_cfg = {
        .dev_addr_length = cfg->addr_mode == I2C_ADDR_7BIT ? I2C_ADDR_BIT_LEN_7 : I2C_ADDR_BIT_LEN_10,
        .device_address = cfg->address,
        .scl_speed_hz = cfg->speed_hz,
    };
    i2c_master_dev_handle_t dev_handle;
    esp_err_t r = i2c_master_bus_add_device(bus->handle, &dev_cfg, &dev_handle);
    if (r != ESP_OK) {
		vSemaphoreDelete(dev->lock);
		free(dev);
		return NULL;
	}
	dev->handle = dev_handle;

	return dev;
}

obk_i2c_status_t obk_i2c_connected_device_deinit(obk_i2c_connected_dev_t *dev){
	if (!dev) return I2C_INVALID_ARG;
	i2c_master_bus_rm_device(dev->handle);
	if (dev->lock) vSemaphoreDelete(dev->lock);
	free(dev);
	return I2C_OK;
}
obk_i2c_status_t obk_i2c_write_to_device(obk_i2c_connected_dev_t *dev,
									  uint8_t *tx_buf,
									  size_t tx_len,
									  uint32_t timeout_ms){
	if (xSemaphoreTake(dev->lock, _ms_to_ticks(_device_get_timeout_ms(dev, timeout_ms))) != pdTRUE) {
        return I2C_BUSY;
    }
    dev->busy = true;
	printf("writing to dev");
	esp_err_t r = i2c_master_transmit(dev->handle, tx_buf,tx_len, _ms_to_ticks(_device_get_timeout_ms(dev, timeout_ms)));
  	dev->busy = false;
    xSemaphoreGive(dev->lock);
    return esp_to_obk_status(r);
}
obk_i2c_status_t obk_i2c_read_from_device(obk_i2c_connected_dev_t *dev,
									  uint8_t *rx_buf,
									  size_t rx_len,
									  uint32_t timeout_ms){
	if (xSemaphoreTake(dev->lock, _ms_to_ticks(_device_get_timeout_ms(dev, timeout_ms))) != pdTRUE) {
        return I2C_BUSY;
    }
    dev->busy = true;
	esp_err_t r = i2c_master_receive(dev->handle, rx_buf,rx_len, _ms_to_ticks(_device_get_timeout_ms(dev, timeout_ms)));
  	dev->busy = false;
    xSemaphoreGive(dev->lock);
    return esp_to_obk_status(r);										
}
obk_i2c_status_t obk_i2c_write_read_from_device(obk_i2c_connected_dev_t *dev,
									  uint8_t *tx_buf,
									  size_t tx_len,
									  uint8_t *rx_buf,
									  size_t rx_len,
									  uint32_t timeout_ms){
	if (xSemaphoreTake(dev->lock, _ms_to_ticks(_device_get_timeout_ms(dev, timeout_ms))) != pdTRUE) {
        return I2C_BUSY;
    }
    dev->busy = true;
	esp_err_t r = i2c_master_transmit_receive(dev->handle,tx_buf, tx_len, rx_buf,rx_len, _ms_to_ticks(_device_get_timeout_ms(dev, timeout_ms)));
  	dev->busy = false;
    xSemaphoreGive(dev->lock);
    return esp_to_obk_status(r);	
}	

#endif