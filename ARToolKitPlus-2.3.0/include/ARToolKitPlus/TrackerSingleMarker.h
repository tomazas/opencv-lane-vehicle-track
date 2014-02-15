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

#pragma once

#include <ARToolKitPlus/Tracker.h>

namespace ARToolKitPlus {

/**
 * Defines a simple interface for single-marker tracking with ARToolKitPlus
 *  ARToolKitPlus::TrackerSingleMarker provides all methods to access ARToolKit for
 *  single marker tracking without needing to mess around with it low level methods directly.
 *
 *  Per default the tracker searches for Id-based markers with normal border and uses
 *  the RPP algorithm for pose estimation. You can override this using the according methods.
 *
 *  If you need multi-marker tracking use TrackerMultiMarker.
 */
class AR_EXPORT TrackerSingleMarker: public Tracker {
public:
    /**
     * These parameters control the way the toolkit warps a found
     * marker to a perfect square. The square has size
     * pattWidth * pattHeight, the projected
     * square in the image is subsampled at a min of
     * pattWidth/pattHeight and a max of pattSamples
     * steps in both x and y direction
     *  @param imWidth width of the source image in px
     *  @param imHeight height of the source image in px
     *  @param maxImagePatterns describes the maximum number of patterns that can be analyzed in a camera image.
     *  @param pattWidth describes the pattern image width (must be 6 for binary markers)
     *  @param pattHeight describes the pattern image height (must be 6 for binary markers)
     *  @param pattSamples describes the maximum resolution at which a pattern is sampled from the camera image
     *  (6 by default, must a a multiple of pattWidth and pattHeight).
     *  @param maxLoadPatterns describes the maximum number of pattern files that can be loaded.
     *  Reduce maxLoadPatterns and maxImagePatterns to reduce memory footprint.
     */
    TrackerSingleMarker(int imWidth, int imHeight, int maxImagePatterns = 8, int pattWidth = 6, int pattHeight = 6, int pattSamples = 6,
            int maxLoadPatterns = 0);

    /**
     * initializes TrackerSingleMarker
     * @param nCamParamFile is the name of the camera parameter file
     */
    virtual bool init(const char* nCamParamFile, ARFloat nNearClip, ARFloat nFarClip);

    /**
     * adds a pattern to ARToolKit
     * pass the patterns filename
     */
    virtual int addPattern(const char* nFileName);

    /**
     * calculates the transformation matrix
     * pass the image as RGBX (32-bits)
     * @return detected markers in image
     */
    virtual std::vector<int> calc(const uint8_t* nImage, ARMarkerInfo** nMarker_info = NULL, int* nNumMarkers = NULL);

    /**
     * manually select one of the detected markers
     * instead of using the best one
     */
    virtual void selectDetectedMarker(const int id);

    /**
     * Select the best marker based on Confidence
     */
    virtual int selectBestMarkerByCf();

    /**
     * Sets the width and height of the patterns in OpenGL units
     * defaults to 2.0, so the unity cube fits the marker surface
     */
    virtual void setPatternWidth(ARFloat nWidth) {
        patt_width = nWidth;
    }

    /**
     * Provides access to ARToolKit' patt_trans matrix
     *  This method is primarily for compatibility issues with code previously using
     *  ARToolKit rather than ARToolKitPlus. patt_trans is the original transformation
     *  matrix ARToolKit calculates rather than the OpenGL style version of this matrix
     *  that can be retrieved via getModelViewMatrix().
     */
    virtual void getARMatrix(ARFloat nMatrix[3][4]) const;

    /**
     * Returns the confidence value of the currently best detected marker.
     */
    virtual float getConfidence() const {
        return (float)confidence;
    }

protected:
    ARFloat confidence;
    ARFloat patt_width;
    ARFloat patt_center[2];
    ARFloat patt_trans[3][4];

    // save the results of last calc call
    ARMarkerInfo *marker_info;
    int marker_num;
};

} // namespace ARToolKitPlus
