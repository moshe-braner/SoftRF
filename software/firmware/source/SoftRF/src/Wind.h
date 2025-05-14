/*
 * Wind.h
 * Copyright (C) 2022 Moshe Braner
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WIND_H
#define WIND_H

extern float wind_best_ns;    /* mps */
extern float wind_best_ew;
extern float wind_speed;
extern float wind_direction;
// extern time_t AirborneTime;

void this_airborne(bool validfix);
void project_this(container_t *);
void project_that(container_t *);
void Estimate_Wind(void);
float Estimate_Climbrate(void);

#endif /* WIND_H */
