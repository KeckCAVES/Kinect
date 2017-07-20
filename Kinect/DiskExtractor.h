/***********************************************************************
DiskExtractor - Helper class to extract the 3D center points of disks
from depth images.
Copyright (c) 2015-2017 Oliver Kreylos

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

#ifndef KINECT_DISKEXTRACTOR_INCLUDED
#define KINECT_DISKEXTRACTOR_INCLUDED

#include <vector>
#include <Threads/Thread.h>
#include <Threads/MutexCond.h>
#include <Geometry/Point.h>
#include <Geometry/Vector.h>
#include <Geometry/ProjectiveTransformation.h>
#include <Geometry/ValuedPoint.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/FrameSource.h>

/* Forward declarations: */
namespace Misc {
template <class ParameterParam>
class FunctionCall;
}

namespace Kinect {

class DiskExtractor
	{
	/* Embedded classes: */
	private:
	typedef FrameSource::DepthPixel DepthPixel; // Type for depth image pixels
	typedef FrameSource::DepthCorrection::PixelCorrection PixelDepthCorrection; // Type for per-pixel depth correction factors
	typedef FrameSource::IntrinsicParameters::PTransform PTransform; // Type for projections from depth image space into camera space
	public:
	typedef PTransform::Scalar Scalar; // Type for scalar values
	typedef Geometry::ValuedPoint<Geometry::Point<Scalar,2>,Scalar> ImagePoint; // Type for image points in lens distortion-corrected depth image space with accumulation weights
	typedef PTransform::Point Point; // Type for points in depth image and camera space
	typedef PTransform::Vector Vector; // Type for vectors in depth image and camera space
	
	struct Disk // Structure for extracted disks
		{
		/* Elements: */
		public:
		Point center; // Disk's center point
		Vector normal; // Normal vector of plane containing Disk
		
		/* Extraction statistics: */
		unsigned int numPixels; // Number of pixels in the disk's blob
		Scalar radius; // Disk's radius
		Scalar flatness; // Disk's flatness factor
		};
	
	typedef std::vector<Disk> DiskList; // Type for lists of extracted disks
	typedef Misc::FunctionCall<const DiskList&> ExtractionResultCallback; // Type for functions to be called when disks have been extracted from a depth image
	typedef Misc::FunctionCall<const Disk&> TrackingCallback; // Type for functions to be called with the blob containing a tracked pixel
	
	/* Forward declarations of embedded classes: */
	#if 0 // Not used anymore
	struct DepthCentroidBlob;
	#endif
	struct DepthPCABlob;
	
	/* Elements: */
	private:
	unsigned int frameSize[2]; // Size of incoming depth images
	bool privateDepthCorrection; // Flag whether the 2D array of per-pixel depth correction factors was allocated by this disk extractor
	PixelDepthCorrection* depthCorrection; // 2D array of per-pixel depth correction factors
	ImagePoint* framePixels; // 2D array of lens distortion-corrected depth image pixels
	PTransform depthProjection; // Projection from depth image space into camera space
	
	/* Disk extraction parameters: */
	int maxBlobMergeDist; // Maximum depth value distance of neighboring pixels to join the same blob
	unsigned int minNumPixels; // Minimum number of pixels to consider a blob as a disk candidate
	Scalar diskRadius; // Radius of searched disk in camera space units
	Scalar diskRadiusMargin; // Maximum radius tolerance for disk radii
	Scalar diskFlatness; // Maximum along-axis extent of searched disks
	
	Threads::MutexCond newFrameCond; // Condition variable to signal the arrival of a new depth image to the disk extractor thread
	volatile bool keepProcessing; // Flag to shut down the disk extractor thread
	FrameBuffer newFrame; // Buffer holding incoming depth image for disk extraction
	Threads::Thread diskExtractorThread; // Background thread extracting disks from depth images
	ExtractionResultCallback* extractionResultCallback; // Function called with disk extraction results
	unsigned int trackingPixel; // Linear index of the tracking pixel
	TrackingCallback* trackingCallback; // Function called with the disk containing a tracked pixel
	
	/* Private methods: */
	void* diskExtractorThreadMethod(void); // Method implementing the disk extractor thread
	
	/* Constructors and destructors: */
	public:
	DiskExtractor(const unsigned int sFrameSize[2],const FrameSource::DepthCorrection* dc,const FrameSource::IntrinsicParameters& ips);
	DiskExtractor(const unsigned int sFrameSize[2],const PixelDepthCorrection* sDepthCorrection,const FrameSource::IntrinsicParameters& ips);
	private:
	DiskExtractor(const DiskExtractor& source); // Prohibit copy constructor
	DiskExtractor& operator=(const DiskExtractor& source); // Prohibit assignment operator
	public:
	~DiskExtractor(void);
	
	/* Methods: */
	int getMaxBlobMergeDist(void) const
		{
		return maxBlobMergeDist;
		}
	unsigned int getMinNumPixels(void) const
		{
		return minNumPixels;
		}
	Scalar getDiskRadius(void) const
		{
		return diskRadius;
		}
	Scalar getDiskRadiusMargin(void) const
		{
		return diskRadiusMargin;
		}
	Scalar getDiskFlatness(void) const
		{
		return diskFlatness;
		}
	const ImagePoint& getFramePixel(unsigned int x,unsigned int y) const // Returns the lens distortion-corrected position of the given depth frame pixel
		{
		return framePixels[y*frameSize[0]+x];
		}
	void setMaxBlobMergeDist(int newMaxBlobMergeDist); // Sets the blob merge limit for the next frame to be processed
	void setMinNumPixels(unsigned int newMinNumPixels); // Sets the minimum blob size
	void setDiskRadius(Scalar newDiskRadius); // Sets the radius of to-be-extracted disks
	void setDiskRadiusMargin(Scalar newDiskRadiusMargin); // Sets the maximum tolerance factor for disk radii
	void setDiskFlatness(Scalar newDiskFlatness); // Sets the maximum along-axis extent of to-be-extracted disks
	DiskList processFrame(const FrameBuffer& frame) const; // Immediately processes the given frame
	void startStreaming(ExtractionResultCallback* newExtractionResultCallback); // Starts background processing; class takes ownership of new-allocated function object
	void stopStreaming(void); // Stops background processing
	void startTracking(TrackingCallback* newTrackingCallback); // Starts tracking a specific pixel in the depth image
	void setTrackingPixel(unsigned int trackingX,unsigned int trackingY); // Sets the pixel to be tracked
	void stopTracking(void); // Stops pixel tracking
	void submitFrame(const FrameBuffer& newNewFrame) // Holds the given depth image frame for disk extraction
		{
		Threads::MutexCond::Lock newFrameLock(newFrameCond);
		
		/* Hold the new frame: */
		newFrame=newNewFrame;
		
		/* Wake up the disk extractor thread: */
		newFrameCond.signal();
		}
	};

}

#endif
