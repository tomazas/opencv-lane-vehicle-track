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

#ifndef __ARTOOLKITPLUS_HULL_HEADERFILE__
#define __ARTOOLKITPLUS_HULL_HEADERFILE__

#include <ARToolKitPlus/config.h>

namespace ARToolKitPlus {

const int MAX_HULL_POINTS = 64; // support up to 16 visible markers


struct AR_EXPORT MarkerPoint {
    typedef int coord_type;

    coord_type x, y;
    unsigned short markerIdx, cornerIdx;
};

inline int iabs(int nValue) {
    return nValue >= 0 ? nValue : -nValue;
}

AR_EXPORT int nearHull_2D(const MarkerPoint* P, int n, int k, MarkerPoint* H);

AR_EXPORT void findLongestDiameter(const MarkerPoint* nPoints, int nNumPoints, int &nIdx0, int &nIdx1);

AR_EXPORT void findFurthestAway(const MarkerPoint* nPoints, int nNumPoints, int nIdx0, int nIdx1, int& nIdxFarthest);

AR_EXPORT void maximizeArea(const MarkerPoint* nPoints, int nNumPoints, int nIdx0, int nIdx1, int nIdx2, int& nIdxMax);

AR_EXPORT void sortIntegers(int& nIdx0, int& nIdx1, int& nIdx2);

AR_EXPORT void sortInLastInteger(int& nIdx0, int& nIdx1, int& nIdx2, int &nIdx3);

} // namespace ARToolKitPlus


#endif //__ARTOOLKITPLUS_HULL_HEADERFILE__
