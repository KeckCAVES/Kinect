/***********************************************************************
CornerExtractor - Helper class to extract the 2D center points of grid
corners from color images.
Copyright (c) 2015 Oliver Kreylos

This file is part of the Kinect 3D Video Capture Project (Kinect).

The Kinect 3D Video Capture Project is free software; you can
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

The Kinect 3D Video Capture Project is distributed in the hope that it
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Kinect 3D Video Capture Project; if not, write to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
02111-1307 USA
***********************************************************************/

#ifndef KINECT_CORNEREXTRACTOR_INCLUDED
#define KINECT_CORNEREXTRACTOR_INCLUDED

#include <vector>
#include <Threads/Thread.h>
#include <Threads/MutexCond.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}

namespace Kinect {

class CornerExtractor
	{
	/* Embedded classes: */
	public:
	typedef FrameSource::ColorPixel ColorPixel; // Type for color image pixels
	typedef float Scalar; // Type for scalar values
	typedef Geometry::Point<Scalar,2> Point; // Type for points in pixel space
	typedef Geometry::Vector<Scalar,2> Vector; // Type for vectors in pixel space
	
	struct Corner:public Point // Structure to represent extracted corners
		{
		/* Elements: */
		public:
		Vector bw,wb; // Separation line directions from black to white and white to black, respectively, forming a right-handed frame
		};
	
	typedef std::vector<Corner> CornerList; // Type for lists of extracted corners
	typedef Misc::FunctionCall<const CornerList&> ExtractionResultCallback; // Type for functions to be called when corners have been extracted from a color image
	
	private:
	struct RingPixel // Structure to store pixel offsets and angles around a ring
		{
		/* Elements: */
		public:
		int offset; // Frame buffer offset from the ring's center to this pixel
		Vector d; // Offset vector from ring's center to this pixel
		Scalar angle; // Angle of this pixel around the ring in radians
		
		/* Methods: */
		void init(int x,int y,int stride); // Initializes the ring pixel
		};
	
	/* Elements: */
	Scalar twoPi; // We end up needing this a lot...
	unsigned int frameSize[2]; // Size of incoming color images
	bool useGreenChannel; // Flag to only use the green channel to construct a greyscale image, to reduce chromatic aberration problems
	unsigned char* gammaCorrection; // A look-up table to gamma-correct incoming color images
	unsigned int maxNumRings; // Number of precomputed rings
	int minRingRadius; // Radius of first precomputed ring
	int* ringRadii; // Array of ring radii
	unsigned int* ringLengths; // Array of ring arc lengths in pixels
	RingPixel** rings; // Array of pointers to ring offsets and angles; each ring's array makes two full circles to avoid edge conditions
	unsigned int* integralImage; // Image containing sums of pixel values
	unsigned long* integral2Image; // Image containing sums of squared pixel values
	unsigned char* normalizedImage; // Normalized greyscale image
	unsigned int gridCellSize; // Grid line spacing of the crossing grid
	unsigned int gridSize[2]; // Number of horizontal and vertical grid lines
	unsigned int gridBaseX,gridBaseY; // x position of first vertical grid line and y position of first horizontal grid line
	unsigned char* gridX; // Array of horizontal grid lines with each pixel indicating a black/white or white/black crossing
	unsigned char* gridY; // Array of vertical grid lines with each pixel indicating a black/white or white/black crossing
	
	/* Corner extraction parameters: */
	unsigned int normalizationWindowSize; // Half-size of normalization window
	unsigned int regionThreshold; // Half size of "grey" area around central greyscale value
	unsigned int numRings; // Number of rings to test around each pixel
	unsigned int maxNonCornerRings; // Maximum number of non-corner rings accepted around a corner pixel
	Scalar maxBlackWhiteImbalance; // Maximum imbalance between black and white areas on corner rings
	Scalar maxAsymmetry; // Maximum asymmetry between the two black and the two white regions around a corner ring in radians
	Scalar maxBlackWhiteRatioSlope; // Maximum slope of the black-to-white ratio over ring number linear regression
	
	Threads::MutexCond newFrameCond; // Condition variable to signal the arrival of a new color image to the corner extractor thread
	volatile bool keepProcessing; // Flag to shut down the corner extractor thread
	FrameBuffer newFrame; // Buffer holding incoming color image for corner extraction
	bool ugc; // Background thread's copies of extraction parameters
	unsigned int nws,nr,mncr; // Ditto
	unsigned char greyMin,greyMax; // Ditto
	Scalar mbwi,ma,mbwrs; // Ditto
	Threads::Thread cornerExtractorThread; // Background thread extracting corners from color images
	ExtractionResultCallback* extractionResultCallback; // Function called with corner extraction results
	
	/* Private methods: */
	void normalizeFrame(const FrameBuffer& frame); // Normalizes the given color frame with the given sliding window size
	void calculateGrid(void); // Calculates a grid of b/w and w/b region crossings to speed up corner detection and grid construction
	bool checkPixel(const unsigned char* pixel,Vector& bwSeparator,Vector& wbSeparator) const; // Checks the pixel at the given address inside the normalized greyscale image for corner status
	void extractCorners(const FrameBuffer& frame,CornerList& corners); // Runs the corner extraction algorithm on the given color frame
	void* cornerExtractorThreadMethod(void); // Method implementing the corner extractor thread
	
	/* Constructors and destructors: */
	public:
	CornerExtractor(const unsigned int sFrameSize[2],unsigned int sMaxNumRings,int sMinRingRadius);
	private:
	CornerExtractor(const CornerExtractor& source); // Prohibit copy constructor
	CornerExtractor& operator=(const CornerExtractor& source); // Prohibit assignment operator
	public:
	~CornerExtractor(void);
	
	/* Methods: */
	int getTestRadius(void) const // Returns the radius of the largest tested ring around each corner candidate
		{
		return ringRadii[numRings-1];
		}
	const unsigned char* getNormalizedImage(void) const // Only for debugging purposes
		{
		return normalizedImage;
		}
	void setUseGreenChannel(bool newUseGreenChannel); // If set to true, uses only the green channel to create greyscale images
	void setInputGamma(float newInputGamma); // Sets a gamma correction value for incoming color images; e.g., JPG/MJPG sources usually require 2.2
	void setNormalizationWindowSize(unsigned int newNormalizationWindowSize);
	void setRegionThreshold(unsigned int newRegionThreshold);
	void setNumRings(unsigned int newNumRings);
	void setCornerTestRadius(int newCornerTestRadius);
	void setMaxNonCornerRings(unsigned int newMaxNonCornerRings);
	void setMaxBlackWhiteImbalance(Scalar newMaxBlackWhiteImbalance);
	void setMaxAsymmetry(Scalar newMaxAsymmetry);
	void setMaxBlackWhiteRatioSlope(Scalar newMaxBlackWhiteRatioSlope);
	CornerList processFrame(const FrameBuffer& frame); // Immediately processes the given frame
	void startStreaming(ExtractionResultCallback* newExtractionResultCallback); // Starts background processing; class takes ownership of new-allocated function object
	void stopStreaming(void); // Stops background processing
	void submitFrame(const FrameBuffer& newNewFrame) // Holds the given color frame for corner extraction
		{
		Threads::MutexCond::Lock newFrameLock(newFrameCond);
		
		/* Hold the new frame: */
		newFrame=newNewFrame;
		
		/* Wake up the corner extractor thread: */
		newFrameCond.signal();
		}
	};

}

#endif
