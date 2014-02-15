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
 * Defines a simple interface for multi-marker tracking with ARToolKitPlus
 *  ARToolKit::TrackerMultiMarker provides all methods to access ARToolKit for
 *  multi marker tracking without needing to mess around with it directly.
 *
 *  Per default the tracker searches for Id-based markers with normal border and uses
 *  the RPP algorithm for pose estimation.
 *  Furthermore it uses only 4 'good' points of the convex hull to do the pose estimation.
 *  You can override this using the according methods.
 */
class AR_EXPORT TrackerMultiMarker: public Tracker {
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
    TrackerMultiMarker(int imWidth, int imHeight, int maxImagePatterns = 8, int pattWidth = 6, int pattHeight = 6, int pattSamples = 6,
            int maxLoadPatterns = 0);

    virtual ~TrackerMultiMarker();

    /**
     * initializes ARToolKit
     *  nCamParamFile is the name of the camera parameter file
     *  nNearClip & nFarClip are near and far clipping values for the OpenGL projection matrix
     *  nLogger is an instance which implements the ARToolKit::Logger interface
     */
    virtual bool init(const char* const nCamParamFile, const char* const nMultiFile, ARFloat nNearClip, ARFloat nFarClip);

    /**
     * calculates the transformation matrix
     *	pass the image as RGBX (32-bits)
     */
    virtual int calc(const uint8_t* nImage);

    /*
     * Returns the number of detected markers used for multi-marker tracking
     */
    virtual int getNumDetectedMarkers() const {
        return numDetected;
    }

    /// Enables usage of arDetectMarkerLite. Otherwise arDetectMarker is used
    /**
     * Enables usage of arDetectMarkerLite. Otherwise arDetectMarker is used
     * In general arDetectMarker is more powerful since it keeps history about markers.
     * In some cases such as very low camera refresh rates it is advantegous to change this.
     * Using the non-lite version treats each image independent.
     */
    virtual void setUseDetectLite(bool nEnable) {
        useDetectLite = nEnable;
    }

    /**
     * Returns array of detected marker IDs
     * Only access the first getNumDetectedMarkers() markers
     */
    virtual void getDetectedMarkers(int*& nMarkerIDs);

    /*
     * Returns the ARMarkerInfo object for a found marker
     */
    virtual const ARMarkerInfo& getDetectedMarker(int nWhich) const {
        return detectedMarkers[nWhich];
    }

    /**
     * Returns the loaded ARMultiMarkerInfoT object
     *  If loading the multi-marker config file failed then this method
     *  returns NULL.
     */
    virtual const ARMultiMarkerInfoT* getMultiMarkerConfig() const {
        return config;
    }

    /**
     * Provides access to ARToolKit' internal version of the transformation matrix
     *  This method is primarily for compatibility issues with code previously using
     *  ARToolKit rather than ARToolKitPlus. This is the original transformation
     *  matrix ARToolKit calculates rather than the OpenGL style version of this matrix
     *  that can be retrieved via getModelViewMatrix().
     */
    virtual void getARMatrix(ARFloat nMatrix[3][4]) const;
protected:
    int numDetected;
    bool useDetectLite;

    ARMultiMarkerInfoT *config;
    int *detectedMarkerIDs;
    ARMarkerInfo *detectedMarkers;
};

} // namespace ARToolKitPlus
