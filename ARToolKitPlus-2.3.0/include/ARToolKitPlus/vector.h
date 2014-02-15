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

#ifndef __ARTOOLKITVECTOR_HEADERFILE__
#define __ARTOOLKITVECTOR_HEADERFILE__

#include <ARToolKitPlus/config.h>

namespace ARToolKitPlus {

struct ARMat;

struct AR_EXPORT ARVec {
	ARFloat *v;
	int clm;
};

namespace Vector {
AR_EXPORT ARVec * alloc(int clm);
AR_EXPORT int free(ARVec *v);
AR_EXPORT ARFloat household(ARVec *x);
AR_EXPORT ARFloat innerproduct(ARVec *x, ARVec *y);
AR_EXPORT int tridiagonalize(ARMat *a, ARVec *d, ARVec *e);
}

} // namespace ARToolKitPlus


#endif // __ARTOOLKITVECTOR_HEADERFILE__
