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
 *  Pavel Rojtberg
 */


#ifndef __ARTOOLKITMULTI_HEADERFILE__
#define __ARTOOLKITMULTI_HEADERFILE__

#include <ARToolKitPlus/config.h>

namespace ARToolKitPlus {

typedef struct {
    int     patt_id;
    ARFloat  width;
    ARFloat  center[2];
    ARFloat  trans[3][4];
    ARFloat  itrans[3][4];
    ARFloat  pos3d[4][3];
    int     visible;
/*---*/
    int     visibleR;
} AR_EXPORT ARMultiEachMarkerInfoT;

typedef struct {
    ARMultiEachMarkerInfoT  *marker;
    int                     marker_num;
    ARFloat                  trans[3][4];
    int                     prevF;
/*---*/
    ARFloat                  transR[3][4];
} AR_EXPORT ARMultiMarkerInfoT;


} // namespace ARToolKitPlus


#endif // __ARTOOLKITMULTI_HEADERFILE__
