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

#ifndef __ARTOOLKITAR_HEADERFILE__
#define __ARTOOLKITAR_HEADERFILE__

#include <stdlib.h>

#include <ARToolKitPlus/config.h>
#include <stdint.h>

#define arMalloc(V,T,S)  \
{ if( ((V) = (T *)malloc( sizeof(T) * (S) )) == 0 ) \
{printf("malloc error!!\n"); exit(1);} }

namespace ARToolKitPlus {

typedef struct {
    int area;
    int id;
    int dir;
    ARFloat cf;
    ARFloat pos[2];
    ARFloat line[4][3];
    ARFloat vertex[4][2];
} AR_EXPORT ARMarkerInfo;

typedef struct {
    int area;
    ARFloat pos[2];
    int coord_num;
    int x_coord[AR_CHAIN_MAX];
    int y_coord[AR_CHAIN_MAX];
    int vertex[5];
} AR_EXPORT ARMarkerInfo2;

typedef struct {
    ARMarkerInfo marker;
    int count;
} AR_EXPORT arPrevInfo;

} // namespace ARToolKitPlus


#endif  //__ARTOOLKITAR_HEADERFILE__
