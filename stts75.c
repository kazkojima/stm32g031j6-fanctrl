/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/i2c.h>
#include "stts75.h"

void stts75_write_config(uint32_t i2c, uint8_t sensor)
{
  uint8_t wd[2];
  wd[0] = 0x01;
  wd[1] = 0x04;
  i2c_transfer7(i2c, sensor, wd, 2, NULL, 0);
}

void stts75_write_temp_os(uint32_t i2c, uint8_t sensor, uint16_t temp_os)
{
  uint8_t wd[3];
  wd[0] = 0x03; /* OvertemperatureShutdown register */
  wd[1] = temp_os >> 8;
  wd[2] = temp_os & 0xff;
  i2c_transfer7(i2c, sensor, wd, 3, NULL, 0);
}

void stts75_write_temp_hyst(uint32_t i2c, uint8_t sensor, uint16_t temp_hyst)
{
  uint8_t wd[3];
  wd[0] = 0x02; /* TemperatureHysteresis register */
  wd[1] = temp_hyst >> 8;
  wd[2] = temp_hyst & 0xff;
  i2c_transfer7(i2c, sensor, wd, 3, NULL, 0);
}

uint16_t stts75_read_temperature(uint32_t i2c, uint8_t sensor)
{
  uint16_t temperature;
  uint8_t wd[1], rd[2];
  wd[0] = 0x00; /* Temperature register */
  i2c_transfer7(i2c, sensor, wd, 1, rd, 2);
  temperature = (((uint16_t)rd[0]) << 8) + rd[1];
  return temperature;
}
