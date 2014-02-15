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

#ifndef __ARTOOLKITMATRIX_HEADERFILE__
#define __ARTOOLKITMATRIX_HEADERFILE__

#include <ARToolKitPlus/config.h>

/* === matrix definition ===

 <---- clm --->
 [ 10  20  30 ] ^
 [ 20  10  15 ] |
 [ 12  23  13 ] row
 [ 20  10  15 ] |
 [ 13  14  15 ] v

 =========================== */

namespace ARToolKitPlus {

struct AR_EXPORT ARMat {
    ARFloat *m;
    int row;
    int clm;
};

namespace Matrix {

/* 0 origin */
#define ARELEM0(mat,r,c) ((mat)->m[(r)*((mat)->clm)+(c)])
/* 1 origin */
#define ARELEM1(mat,row,clm) ARELEM0(mat,row-1,clm-1)

AR_EXPORT ARMat *alloc(int row, int clm);
AR_EXPORT int free(ARMat *m);

AR_EXPORT int dup(ARMat *dest, ARMat *source);
AR_EXPORT ARMat *allocDup(ARMat *source);

AR_EXPORT int mul(ARMat *dest, ARMat *a, ARMat *b);
AR_EXPORT int selfInv(ARMat *m);

} // namespace Matrix


} // namespace ARToolKitPlus


#endif // __ARTOOLKITMATRIX_HEADERFILE__
