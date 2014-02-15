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

#ifndef __ARTOOLKIT_TRACKERIMPL_HEADERFILE__
#define __ARTOOLKIT_TRACKERIMPL_HEADERFILE__

#include <ARToolKitPlus/ARToolKitPlus.h>
#include <ARToolKitPlus/ar.h>
#include <ARToolKitPlus/arMulti.h>
#include <ARToolKitPlus/vector.h>
#include <ARToolKitPlus/Camera.h>
#include <ARToolKitPlus/extra/BCH.h>
#include <ARToolKitPlus/extra/Hull.h>

#include <vector>


namespace ARToolKitPlus {

/**
 * Tracker is the vision core of ARToolKit.
 * Almost all original ARToolKit methods are included here.
 * Exceptions: matrix & vector.
 *
 * Tracker includes all methods that are needed to create a
 * basic ARToolKit application (e.g. the simple example
 * from the original ARToolKit package)
 *
 * Application developers should usually prefer using the
 * more high level classes:
 *  - TrackerSingleMarker
 *  - TrackerMultiMarker
 */
class AR_EXPORT Tracker {
public:
    Tracker(int imWidth, int imHeight, int maxImagePatterns = 8, int pattWidth = 6, int pattHeight = 6, int pattSamples = 6,
            int maxLoadPatterns = 0);
    virtual ~Tracker();

    /**
     * Sets the pixel format of the camera image
     *  Default format is RGB888 (PIXEL_FORMAT_RGB)
     */
    virtual bool setPixelFormat(PIXEL_FORMAT nFormat);

    /**
     * Loads a camera calibration file and stores data internally
     *  To prevent memory leaks, this method internally deletes an existing camera.
     *  If you want to use more than one camera, retrieve the existing camera using getCamera()
     *  and call setCamera(NULL); before loading another camera file.
     *  On destruction, ARToolKitPlus will only destroy the currently set camera. All other
     *  cameras have to be destroyed manually.
     */
    virtual bool loadCameraFile(const char* nCamParamFile, ARFloat nNearClip, ARFloat nFarClip);

    /**
     * Set to true to try loading camera undistortion table from a cache file
     *  On slow platforms (e.g. Smartphone) creation of the undistortion lookup-table
     *  can take quite a while. Consequently caching will speedup the start phase.
     *  If set to true and no cache file could be found a new one will be created.
     *  The cache file will get the same name as the camera file with the added extension '.LUT'
     */
    virtual void setLoadUndistLUT(bool nSet) {
        loadCachedUndist = nSet;
    }

    /// marker detection using tracking history
    virtual int arDetectMarker(const uint8_t *dataPtr, int thresh, ARMarkerInfo **marker_info, int *marker_num);

    /// marker detection without using tracking history
    virtual int arDetectMarkerLite(const uint8_t *dataPtr, int thresh, ARMarkerInfo **marker_info, int *marker_num);

    /// calculates the transformation matrix between camera and the given multi-marker config
    virtual ARFloat arMultiGetTransMat(ARMarkerInfo *marker_info, int marker_num, ARMultiMarkerInfoT *config);

    virtual ARFloat arMultiGetTransMatHull(ARMarkerInfo *marker_info, int marker_num, ARMultiMarkerInfoT *config);

    /// calculates the transformation matrix between camera and the given marker
    virtual ARFloat arGetTransMat(ARMarkerInfo *marker_info, ARFloat center[2], ARFloat width, ARFloat conv[3][4]);

    virtual ARFloat arGetTransMatCont(ARMarkerInfo *marker_info, ARFloat prev_conv[3][4], ARFloat center[2],
            ARFloat width, ARFloat conv[3][4]);

    // RPP integration -- [t.pintaric]
    virtual ARFloat rppMultiGetTransMat(ARMarkerInfo *marker_info, int marker_num, ARMultiMarkerInfoT *config);
    virtual ARFloat rppGetTransMat(ARMarkerInfo *marker_info, ARFloat center[2], ARFloat width, ARFloat conv[3][4]);

    /// loads a pattern from a file
    virtual int arLoadPatt(char *filename);

    /// frees a pattern from memory
    virtual int arFreePatt(int patno);

    virtual int arMultiFreeConfig(ARMultiMarkerInfoT *config);

    virtual ARMultiMarkerInfoT *arMultiReadConfigFile(const char *filename);

    /**
     * activates binary markers
     *  markers are converted to pure black/white during loading
     */
    virtual void activateBinaryMarker(int nThreshold) {
        binaryMarkerThreshold = nThreshold;
    }

    /**
     * activate the usage of id-based markers rather than template based markers
     *  Template markers are the classic marker type used in ARToolKit.
     *  Id-based markers directly encode the marker id in the image.
     *  Simple markers use 3-times redundancy to increase robustness, while
     *  BCH markers use an advanced CRC algorithm to detect and repair marker damages.
     *  See arBitFieldPattern.h for more information.
     *  In order to use id-based markers, the marker size has to be 6x6, 12x12 or 18x18.
     */
    virtual void setMarkerMode(MARKER_MODE nMarkerMode);

    /**
     * activates the complensation of brightness falloff in the corners of the camera image
     *  some cameras have a falloff in brightness at the border of the image, which creates
     *  problems with thresholding the image. use this function to set a (linear) adapted
     *  threshold value. the threshold value will stay exactly the same at the center but
     *  will deviate near to the border. all values specify a difference, not absolute values!
     *  nCorners define the falloff a all four corners. nLeftRight defines the falloff
     *  at the half y-position at the left and right side of the image. nTopBottom defines the falloff
     *  at the half x-position at the top and bottom side of the image.
     *  all values between these 9 points (center, 4 corners, left, right, top, bottom) will
     *  be interpolated.
     */
    virtual void activateVignettingCompensation(bool nEnable, int nCorners = 0, int nLeftRight = 0, int nTopBottom = 0);

    /**
     * Calculates the camera matrix from an ARToolKit camera file.
     * This method retrieves the OpenGL projection matrix that is stored
     * in an ARToolKit camera calibration file.
     * Returns true if loading of the camera file succeeded.
     */
    static bool calcCameraMatrix(const char* nCamParamFile, ARFloat nNear, ARFloat nFar, ARFloat *nMatrix);

    /// Changes the resolution of the camera after the camerafile was already loaded
    virtual void changeCameraSize(int nWidth, int nHeight);

    /**
     * Changes the undistortion mode
     * Default value is UNDIST_STD which means that
     * artoolkit's standard undistortion method is used.
     */
    virtual void setUndistortionMode(UNDIST_MODE nMode);

    /**
     * Changes the Pose Estimation Algorithm
     * POSE_ESTIMATOR_ORIGINAL (default): arGetTransMat()
     * POSE_ESTIMATOR_CONT: original pose estimator with "Cont"
     * POSE_ESTIMATOR_RPP: "Robust Pose Estimation from a Planar Target"
     */
    virtual bool setPoseEstimator(POSE_ESTIMATOR nMethod);

    /**
     * If true the alternative hull-algorithm will be used for multi-marker tracking
     *  Starting with version 2.2 ARToolKitPlus has a new mode for tracking multi-markers:
     *  Instead of using all points (as done by RPP multi-marker tracking)
     *  or tracking all markers independently and combine lateron
     *  (as done in ARToolKit's standard multi-marker pose estimator), ARToolKitPlus can now
     *  use only 4 'good' points of the convex hull to do the pose estimation.
     *  If the pose estimator is set to RPP then RPP will be used to track those 4 points.
     *  Otherwise, ARToolKit's standard single-marker pose estimator will be used to
     *  track the pose of these 4 points.
     */
    virtual void setHullMode(HULL_TRACKING_MODE nMode) {
        hullTrackingMode = nMode;
    }

    /**
     * Sets a new relative border width. ARToolKit's default value is 0.25
     * Take caution that the markers need of course really have thiner borders.
     * Values other than 0.25 have not been tested for regular pattern-based matching,
     * but only for id-encoded markers. It might be that the pattern creation process
     * needs to be updated too.
     */
    virtual void setBorderWidth(ARFloat nFraction) {
        relBorderWidth = nFraction;
    }

    /// Sets the threshold value that is used for black/white conversion
    virtual void setThreshold(int nValue) {
        thresh = nValue;
    }

    /// Returns the current threshold value.
    virtual int getThreshold() const {
        return thresh;
    }

    /// Turns automatic threshold calculation on/off
    virtual void activateAutoThreshold(bool nEnable) {
        autoThreshold.enable = nEnable;
    }

    /// Returns true if automatic threshold detection is enabled
    virtual bool isAutoThresholdActivated() const {
        return autoThreshold.enable;
    }

    /**
     * Sets the number of times the threshold is randomized in case no marker was visible (Minimum: 1, Default: 2)
     *  Autothreshold requires a visible marker to estime the optimal thresholding value. If
     *  no marker is visible ARToolKitPlus randomizes the thresholding value until a marker is
     *  found. This function sets the number of times ARToolKitPlus will randomize the threshold
     *  value and research for a marker per calc() invokation until it gives up.
     *  A value of 2 means that ARToolKitPlus will analyze the image a second time with an other treshold value
     *  if it does not find a marker the first time. Each unsuccessful try uses less processing power
     *  than a single full successful position estimation.
     */
    virtual void setNumAutoThresholdRetries(int nNumRetries) {
        autoThreshold.numRandomRetries = std::min(nNumRetries, 1);
    }

    /**
     * Sets an image processing mode (half or full resolution)
     *  Half resolution is faster but less accurate. When using
     *  full resolution smaller markers will be detected at a
     *  higher accuracy (or even detected at all).
     */
    virtual void setImageProcessingMode(IMAGE_PROC_MODE nMode) {
        arImageProcMode = (nMode == IMAGE_HALF_RES ? AR_IMAGE_PROC_IN_HALF : AR_IMAGE_PROC_IN_FULL);
    }

    /// Returns an opengl-style modelview transformation matrix
    virtual const ARFloat* getModelViewMatrix() const {
        return gl_para;
    }

    /// Returns an opengl-style projection transformation matrix
    virtual const ARFloat* getProjectionMatrix() const {
        return gl_cpara;
    }

    /// Returns the compiled-in pixel format
    virtual PIXEL_FORMAT getPixelFormat() const {
        return static_cast<PIXEL_FORMAT> (pixelFormat);
    }

    /// Returns the numbber of bits per pixel for the compiled-in pixel format
    virtual int getBitsPerPixel() const {
        return pixelSize * 8;
    }

    /**
     * Returns the maximum number of patterns that can be loaded
     *  This maximum number of loadable patterns can be set via the
     *  maxLoadPatterns parameter
     */
    virtual int getNumLoadablePatterns() const {
        return MAX_LOAD_PATTERNS;
    }

    /// Returns the current camera
    virtual Camera* getCamera() {
        return arCamera;
    }

    /// Sets a new camera without specifying new near and far clip values
    virtual void setCamera(Camera* nCamera);

    /// Sets a new camera including specifying new near and far clip values
    virtual void setCamera(Camera* nCamera, ARFloat nNearClip, ARFloat nFarClip);

    /// Calculates the OpenGL transformation matrix for a specific marker info
    virtual ARFloat calcOpenGLMatrixFromMarker(ARMarkerInfo* nMarkerInfo, ARFloat nPatternCenter[2],
            ARFloat nPatternSize, ARFloat *nOpenGLMatrix);

    /// Calls the pose estimator set with setPoseEstimator() for single marker tracking
    virtual ARFloat executeSingleMarkerPoseEstimator(ARMarkerInfo *marker_info, ARFloat center[2], ARFloat width,
            ARFloat conv[3][4]);

    /// Calls the pose estimator set with setPoseEstimator() for multi marker tracking
    virtual ARFloat executeMultiMarkerPoseEstimator(ARMarkerInfo *marker_info, int marker_num,
            ARMultiMarkerInfoT *config);

    /*
     * Returns a vector with screen coordinates of all corners
     * that were used for marker tracking for the last image
     */
    virtual const CornerPoints& getTrackedCorners() const {
        return trackedCorners;
    }

protected:
#ifdef SMALL_LUM8_TABLE
    static const int LUM_TABLE_SIZE = (0xffff >> 6) + 1;
#else
    static const int LUM_TABLE_SIZE = 0xffff + 1;
#endif
    const int PATTERN_WIDTH;
    const int PATTERN_HEIGHT;
    const int PATTERN_SAMPLE_NUM;
    const int MAX_LOAD_PATTERNS;
    const int MAX_IMAGE_PATTERNS;
    const int WORK_SIZE;

    bool checkPixelFormat();

    void checkImageBuffer();

    // converts an ARToolKit transformation matrix for usage with OpenGL
    void convertTransformationMatrixToOpenGLStyle(ARFloat para[3][4], ARFloat gl_para[16]);

    // converts an ARToolKit projection matrix for usage with OpenGL
    static bool convertProjectionMatrixToOpenGLStyle(Camera *param, ARFloat gnear, ARFloat gfar, ARFloat m[16]);
    static bool convertProjectionMatrixToOpenGLStyle2(ARFloat cparam[3][4], int width, int height, ARFloat gnear,
            ARFloat gfar, ARFloat m[16]);

    ARMarkerInfo2* arDetectMarker2(int16_t *limage, int label_num, int *label_ref, int *warea, ARFloat *wpos,
            int *wclip, int area_max, int area_min, ARFloat factor, int *marker_num);

    int arGetContour(int16_t *limage, int *label_ref, int label, int clip[4], ARMarkerInfo2 *marker_infoTWO);

    int check_square(int area, ARMarkerInfo2 *marker_infoTWO, ARFloat factor);

    int
    arGetCode(const uint8_t *image, int *x_coord, int *y_coord, int *vertex, int *code, int *dir, ARFloat *cf, int thresh);

    int arGetPatt(const uint8_t *image, int *x_coord, int *y_coord, int *vertex, uint8_t *ext_pat);

    int pattern_match(uint8_t *data, int *code, int *dir, ARFloat *cf);

    int downsamplePattern(uint8_t* data, unsigned char* imgPtr);

    int bitfield_check_simple(uint8_t *data, int *code, int *dir, ARFloat *cf, int thresh);

    int bitfield_check_BCH(uint8_t *data, int *code, int *dir, ARFloat *cf, int thresh);

    void gen_evec(void);

    ARMarkerInfo* arGetMarkerInfo(const uint8_t *image, ARMarkerInfo2 *marker_info2, int *marker_num, int thresh);

    ARFloat arGetTransMat5(ARFloat rot[3][3], ARFloat ppos2d[][2], ARFloat ppos3d[][3], int num, ARFloat conv[3][4],
            Camera *pCam);

    ARFloat arGetTransMatSub(ARFloat rot[3][3], ARFloat ppos2d[][2], ARFloat pos3d[][3], int num, ARFloat conv[3][4],
            Camera *pCam);

    ARFloat arModifyMatrix(ARFloat rot[3][3], ARFloat trans[3], ARFloat cpara[3][4], ARFloat vertex[][3],
            ARFloat pos2d[][2], int num);

    ARFloat arModifyMatrix2(ARFloat rot[3][3], ARFloat trans[3], ARFloat cpara[3][4], ARFloat vertex[][3],
            ARFloat pos2d[][2], int num);

    int arGetAngle(ARFloat rot[3][3], ARFloat *wa, ARFloat *wb, ARFloat *wc);

    int arGetRot(ARFloat a, ARFloat b, ARFloat c, ARFloat rot[3][3]);

    int arGetNewMatrix(ARFloat a, ARFloat b, ARFloat c, ARFloat trans[3], ARFloat trans2[3][4], ARFloat cpara[3][4],
            ARFloat ret[3][4]);

    int arGetInitRot(ARMarkerInfo *marker_info, ARFloat cpara[3][4], ARFloat rot[3][3]);

    int arGetInitRot2(ARMarkerInfo *marker_info, ARFloat rot[3][3], ARFloat center[2], ARFloat width);

    ARFloat arGetTransMatContSub(ARMarkerInfo *marker_info, ARFloat prev_conv[3][4], ARFloat center[2], ARFloat width,
            ARFloat conv[3][4]);

    int16_t* arLabeling(const uint8_t *image, int thresh, int *label_num, int **area, ARFloat **pos, int **clip,
            int **label_ref);

    int16_t* arLabeling_ABGR(const uint8_t *image, int thresh, int *label_num, int **area, ARFloat **pos, int **clip,
            int **label_ref);
    int16_t* arLabeling_BGR(const uint8_t *image, int thresh, int *label_num, int **area, ARFloat **pos, int **clip,
            int **label_ref);
    int16_t* arLabeling_RGB(const uint8_t *image, int thresh, int *label_num, int **area, ARFloat **pos, int **clip,
            int **label_ref);
    int16_t* arLabeling_RGB565(const uint8_t *image, int thresh, int *label_num, int **area, ARFloat **pos, int **clip,
            int **label_ref);
    int16_t* arLabeling_LUM(const uint8_t *image, int thresh, int *label_num, int **area, ARFloat **pos, int **clip,
            int **label_ref);

    int arActivatePatt(int patno);

    int arDeactivatePatt(int patno);

    int arMultiActivate(ARMultiMarkerInfoT *config);

    int arMultiDeactivate(ARMultiMarkerInfoT *config);

    int verify_markers(ARMarkerInfo *marker_info, int marker_num, ARMultiMarkerInfoT *config);

    int arInitCparam(Camera *pCam);

    int arGetLine(int x_coord[], int y_coord[], int vertex[], ARFloat line[4][3], ARFloat v[4][2]);

    static int arUtilMatMul(ARFloat s1[3][4], ARFloat s2[3][4], ARFloat d[3][4]);

    static int arUtilMatInv(ARFloat s[3][4], ARFloat d[3][4]);

    static int arMatrixPCA(ARMat *input, ARMat *evec, ARVec *ev, ARVec *mean);

    static int arMatrixPCA2(ARMat *input, ARMat *evec, ARVec *ev);

    static int arCameraDecomp(Camera *source, Camera *icpara, ARFloat trans[3][4]);

    static int arCameraDecompMat(ARFloat source[3][4], ARFloat cpara[3][4], ARFloat trans[3][4]);

    int arCameraObserv2Ideal_none(Camera* pCam, ARFloat ox, ARFloat oy, ARFloat *ix, ARFloat *iy);

    int arCameraObserv2Ideal_LUT(Camera* pCam, ARFloat ox, ARFloat oy, ARFloat *ix, ARFloat *iy);

    int arCameraObserv2Ideal_std(Camera* pCam, ARFloat ox, ARFloat oy, ARFloat *ix, ARFloat *iy);
    int arCameraIdeal2Observ_std(Camera* pCam, ARFloat ix, ARFloat iy, ARFloat *ox, ARFloat *oy);

    typedef int (Tracker::* ARPARAM_UNDIST_FUNC)(Camera* pCam, ARFloat ox, ARFloat oy, ARFloat *ix, ARFloat *iy);

    typedef ARFloat (Tracker::* POSE_ESTIMATOR_FUNC)(ARMarkerInfo *marker_info, ARFloat center[2], ARFloat width,
            ARFloat conv[3][4]);
    typedef ARFloat (Tracker::* MULTI_POSE_ESTIMATOR_FUNC)(ARMarkerInfo *marker_info, int marker_num,
            ARMultiMarkerInfoT *config);

    void buildUndistO2ITable(Camera* pCam);

    void checkRGB565LUT();

    // required for calib camera, should otherwise not be used directly
    void setFittingMode(int nWhich) {
        arFittingMode = nWhich;
    }

    ARFloat arGetTransMat3(ARFloat rot[3][3], ARFloat ppos2d[][2], ARFloat ppos3d[][2], int num, ARFloat conv[3][4],
            Camera *pCam);

    static int arCameraObserv2Ideal(Camera *pCam, ARFloat ox, ARFloat oy, ARFloat *ix, ARFloat *iy);
    static int arCameraIdeal2Observ(Camera *pCam, ARFloat ix, ARFloat iy, ARFloat *ox, ARFloat *oy);

protected:
    struct AutoThreshold {
        enum {
            MINLUM0 = 255, MAXLUM0 = 0
        };

        void reset() {
            minLum = MINLUM0;
            maxLum = MAXLUM0;
        }

        void addValue(int nRed, int nGreen, int nBlue, int nPixelFormat) {
            int lum;

            // in RGB565 and LUM8 all three values are simply the grey value...
            if (nPixelFormat == PIXEL_FORMAT_RGB565 || nPixelFormat == PIXEL_FORMAT_LUM)
                lum = nRed;
            else
                lum = (nRed + (nGreen << 1) + nBlue) >> 2;

            if (lum < minLum)
                minLum = lum;
            if (lum > maxLum)
                maxLum = lum;
        }

        int calc() {
            return (minLum + maxLum) / 2;
        }

        bool enable;
        int minLum, maxLum;
        int numRandomRetries;
    } autoThreshold;

    PIXEL_FORMAT pixelFormat;
    int pixelSize;

    int binaryMarkerThreshold;

    // arDetectMarker.cpp
    ARMarkerInfo2 *marker_info2;
    ARMarkerInfo *wmarker_info;
    int wmarker_num;

    arPrevInfo *prev_info;
    int prev_num;

    std::vector<std::vector<arPrevInfo> > sprev_info;
    int sprev_num[2];

    // arDetectMarker2.cpp
    ARMarkerInfo2 *marker_infoTWO; // CAUTION: this member has to be manually allocated!
    //          see TrackerSingleMarker for more info on this.

    int arGetContour_wx[AR_CHAIN_MAX];
    int arGetContour_wy[AR_CHAIN_MAX];

    // arGetCode.cpp
    int pattern_num;
    int *patf;
    std::vector<std::vector<std::vector<int> > > pat;
    ARFloat (*patpow)[4];
    std::vector<std::vector<std::vector<int> > > patBW;
    ARFloat (*patpowBW)[4];

    std::vector<std::vector<ARFloat> > evec;
    ARFloat (*epat)[4][EVEC_MAX];
    int evec_dim;
    int evecf;
    std::vector<std::vector<ARFloat> > evecBW;
    ARFloat (*epatBW)[4][EVEC_MAX];
    int evec_dimBW;
    int evecBWf;

    // arGetMarkerInfo.cpp
    //
    ARMarkerInfo *marker_infoL;

    // arGetTransMat.cpp
    //
    ARFloat pos2d[P_MAX][2];
    ARFloat pos3d[P_MAX][3];

    // arLabeling.cpp
    //
    int16_t *l_imageL; //[HARDCODED_BUFFER_WIDTH*HARDCODED_BUFFER_HEIGHT];		// dyna
    int16_t *l_imageR;
    int l_imageL_size;

    int *workL; //[WORK_SIZE];											// dyna
    int *work2L; //[WORK_SIZE*7];											// dyna

    int *workR;
    int *work2R;
    int *wareaR;
    int *wclipR;
    ARFloat *wposR;

    int wlabel_numL;
    int wlabel_numR;
    int *wareaL; //[WORK_SIZE];	dyna
    int *wclipL; //[WORK_SIZE*4]; dyna
    ARFloat *wposL; //[WORK_SIZE*2];dyna

    int arFittingMode;
    int arImageProcMode;
    Camera *arCamera;
    bool loadCachedUndist;
    int arImXsize, arImYsize;
    int arTemplateMatchingMode;
    int arMatchingPCAMode;

    uint8_t* arImageL;

    MARKER_MODE markerMode;

    unsigned char *RGB565_to_LUM8_LUT; // lookup table for RGB565 to LUM8 conversion


    // camera distortion addon by Daniel
    UNDIST_MODE undistMode;
    unsigned int *undistO2ITable;

    // used for Hull Tracking
    MarkerPoint hullInPoints[MAX_HULL_POINTS];
    MarkerPoint hullOutPoints[MAX_HULL_POINTS];

    CornerPoints trackedCorners;

    ARFloat relBorderWidth;

    ARPARAM_UNDIST_FUNC arCameraObserv2Ideal_func;

    // RPP integration -- [t.pintaric]
    POSE_ESTIMATOR poseEstimator;

    HULL_TRACKING_MODE hullTrackingMode;

    static int screenWidth;
	static int screenHeight;
    int thresh;

    ARFloat gl_para[16];
    ARFloat gl_cpara[16];

    char *descriptionString;

    struct {
        bool enabled;
        int corners, leftright, bottomtop;
    } vignetting;

    unsigned short *DIV_TABLE;

    BCH *bchProcessor;
};

} // namespace ARToolKitPlus

#endif //__ARTOOLKIT_TRACKERIMPL_HEADERFILE__
