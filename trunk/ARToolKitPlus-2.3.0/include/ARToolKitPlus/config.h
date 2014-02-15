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


/**
 * This file is a stripped down version of AR Toolkit original
 * config.h file. Only defines necessary for the core toolkit
 * routines have been left. I tried to comment all variables in a
 * meaningful way. Please extend the comments if you have any idea!
 */

#ifndef AR_CONFIG_H
#define AR_CONFIG_H

// may be changed to double, float is particularly useful for PDA's
#ifdef _USE_DOUBLE_
	typedef double ARFloat;
#else
	typedef float ARFloat;
#endif


/*------------------------------------------------------------
 * see 
 * http://www.hitl.washington.edu/people/grof/SharedSpace/Download/Doc/art240.html 
 * for an explanation of the next two define blocks
 */

// constants for variable arImageProcMode
// half mode is faster and useful for interlaced images
#define  AR_IMAGE_PROC_IN_FULL        0
#define  AR_IMAGE_PROC_IN_HALF        1
#define  DEFAULT_IMAGE_PROC_MODE     AR_IMAGE_PROC_IN_HALF

// constants for variable arFittingMode
#define  AR_FITTING_TO_IDEAL          0
#define  AR_FITTING_TO_INPUT          1
#define  DEFAULT_FITTING_MODE        AR_FITTING_TO_IDEAL

// constants for variable arTemplateMatchingMode
#define  AR_TEMPLATE_MATCHING_COLOR   0
#define  AR_TEMPLATE_MATCHING_BW      1
#define  DEFAULT_TEMPLATE_MATCHING_MODE     AR_TEMPLATE_MATCHING_COLOR

// constant for variable arMatchingPCAMode
#define  AR_MATCHING_WITHOUT_PCA      0
#define  AR_MATCHING_WITH_PCA         1
#define  DEFAULT_MATCHING_PCA_MODE          AR_MATCHING_WITHOUT_PCA


// constants influencing accuracy of arGetTransMat(...)
#define   AR_GET_TRANS_MAT_MAX_LOOP_COUNT         5
#define   AR_GET_TRANS_MAT_MAX_FIT_ERROR          1.0
// criterium for arGetTransMatCont(...) to call 
// arGetTransMat(...) instead
#define   AR_GET_TRANS_CONT_MAT_MAX_FIT_ERROR     1.0

// min/max area of fiducial interiors to be matched
// against templates, used in arDetectMarker.c
#define   AR_AREA_MAX      100000
#define   AR_AREA_MIN          70

// used in arDetectMarker2(...), this param controls the
// maximum number of potential markers evaluated further.
// Only the first AR_SQUARE_MAX patterns are examined.
//#define   AR_SQUARE_MAX        50
// plays some role in arDetectMarker2 I don't understand yet
#define   AR_CHAIN_MAX      10000

#define   EVEC_MAX     10
#define	  P_MAX       500

//#define SMALL_LUM8_TABLE

#ifdef SMALL_LUM8_TABLE
  #define getLUM8_from_RGB565(ptr)   RGB565_to_LUM8_LUT[ (*(unsigned short*)(ptr))>>6 ]
#else
  #define getLUM8_from_RGB565(ptr)   RGB565_to_LUM8_LUT[ (*(unsigned short*)(ptr))    ]
#endif //SMALL_LUM8_TABLE

// disable VisualStudio warnings 
#if defined(_MSC_VER) && !defined(AR_ENABLE_MSVC_WARNINGS)
    #pragma warning( disable : 4244 )
    #pragma warning( disable : 4251 )
    #pragma warning( disable : 4275 )
    #pragma warning( disable : 4512 )
    #pragma warning( disable : 4267 )
    #pragma warning( disable : 4702 )
    #pragma warning( disable : 4511 )
#endif

// Support for Visual Studio compilation
#if defined(AR_STATIC)
	#define AR_EXPORT 
#else
	#if defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__) || defined( __BCPLUSPLUS__)  || defined( __MWERKS__)
		#  if defined( AR_LIBRARY )
		#    define AR_EXPORT   __declspec(dllexport)
		#  else
		#    define AR_EXPORT   __declspec(dllimport)
		#  endif
	#else
		#  define AR_EXPORT
	#endif
#endif

#ifdef _MSC_VER
# if (_MSC_VER >= 1300)
#  define __STL_MEMBER_TEMPLATES
# endif
#endif

#endif //  AR_CONFIG_H

