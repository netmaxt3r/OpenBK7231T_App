#include "../hal_i2c.h"
#include <stdlib.h>
#include <string.h>

obk_i2c_status_t obk_i2c_write_reg(obk_i2c_connected_dev_t *dev,
									uint8_t reg_addr,
									const uint8_t *data,
									size_t data_len,
									uint32_t timeout_ms)
{
	size_t buflen = 1 + data_len;
	uint8_t *buf = (uint8_t *)malloc(buflen);
	if (!buf) {
		return I2C_ERROR;
	}
	buf[0] = reg_addr;
	if (data_len && data) memcpy(buf + 1, data, data_len);
	obk_i2c_status_t ret = obk_i2c_write_to_device(dev, buf, buflen, timeout_ms);
	free(buf);
	return ret;

}

obk_i2c_status_t obk_i2c_read_reg(obk_i2c_connected_dev_t *dev,
                                   uint8_t reg_addr,
                                   uint8_t *data,
                                   size_t data_len,
                                   uint32_t timeout_ms)
{
    if (data_len && !data) return I2C_INVALID_ARG;

    /* send register address, then read data using repeated-start where supported */
    return obk_i2c_write_read_from_device(dev, &reg_addr, 1, data, data_len, timeout_ms);
}

