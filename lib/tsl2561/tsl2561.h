/*
 * LeafMate - Plant monitoring for the ESP32
 * Copyright (C) 2024  Leya Wehner
 * Copyright (C) 2024  Julian Frank
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LEAFMATE_TSL2561_H
#define LEAFMATE_TSL2561_H

void tsl2561_sensor_init(uint8_t bus);

float tsl2561_read_sensor_value(uint8_t bus);

#endif //LEAFMATE_TSL2561_H
