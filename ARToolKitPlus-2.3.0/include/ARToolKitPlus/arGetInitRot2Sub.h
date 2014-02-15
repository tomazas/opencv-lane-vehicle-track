/**
 * Copyright (C) 2010  ARToolkitPlus Authors
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
 *
 * Authors:
 *  Daniel Wagner
 */

#ifndef __ARTOOLKITPLUS_ARGETINITROT2SUB_HEADERFILE__
#define __ARTOOLKITPLUS_ARGETINITROT2SUB_HEADERFILE__

#include <ARToolKitPlus/extra/rpp.h>

namespace rpp {

void AR_EXPORT arGetInitRot2_sub(rpp_float &err, rpp_mat &R, rpp_vec &t, const rpp_float cc[2], const rpp_float fc[2],
        const rpp_vec *model, const rpp_vec *iprts, const unsigned int model_iprts_size, const rpp_mat R_init,
        const bool estimate_R_init, const rpp_float epsilon, const rpp_float tolerance,
        const unsigned int max_iterations);

} // namespace rpp


#endif //__ARTOOLKITPLUS_ARGETINITROT2SUB_HEADERFILE__
