/***********************************************************************
SphereExtractor - Helper class to identify and extract spheres of known
radii in depth images.
Copyright (c) 2014 Oliver Kreylos

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

#include "SphereExtractor.h"

#include <Misc/FunctionCalls.h>
#include <Math/Math.h>
#include <Math/Matrix.h>
#define GEOMETRY_NONSTANDARD_TEMPLATES
#include <Geometry/LevenbergMarquardtMinimizer.h>
#include <Images/ExtractBlobs.h>

namespace {

/****************************
Helper classes and functions:
****************************/

typedef Kinect::FrameSource::DepthPixel DepthPixel; // Pixel type for Kinect depth images
typedef Kinect::FrameSource::DepthCorrection::PixelCorrection PixelDepthCorrection; // Type for Kinect per-pixel depth correction coefficients
typedef Kinect::FrameSource::ColorPixel ColorPixel; // Pixel type for Kinect color images
typedef Kinect::FrameSource::IntrinsicParameters::PTransform PTransform; // Type for depth unprojection transformations
typedef PTransform::Scalar Scalar; // Scalar type of depth unprojection transformation
typedef PTransform::Point Point; // Point type compatible with depth unprojection transformation
typedef Geometry::Sphere<Scalar,3> Sphere; // Type for extracted spheres

struct SphereBlob:public Images::Blob<DepthPixel> // Structure to fit spheres to unprojected depth image pixels
	{
	/* Embedded classes: */
	public:
	typedef DepthPixel Pixel;
	typedef Images::Blob<DepthPixel> Base;
	
	struct Creator:public Base::Creator
		{
		/* Elements: */
		public:
		unsigned int depthFrameSize[2];
		const PixelDepthCorrection* pixelDepthCorrection;
		PTransform depthProjection;
		unsigned int colorFrameSize[2];
		PTransform colorDepthProjection;
		const ColorPixel* colorFrame;
		ColorPixel::Component minWhite;
		ColorPixel::Component maxSpread;
		};
	
	/* Elements: */
	std::vector<Point> points; // List of all unprojected points belonging to this blob
	double system[14]; // Coefficients of the least-squares linear system fitting a sphere to the set of 3D points in the blob
	
	/* Constructors and destructors: */
	SphereBlob(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
		:Base(x,y,pixel,creator)
		{
		/* Calculate the pixel's depth-corrected depth image space position: */
		Point dp(Scalar(x),Scalar(y),Scalar(creator.pixelDepthCorrection[y*creator.depthFrameSize[0]+x].correct(float(pixel))));
		
		/* Unproject the depth image-space point: */
		Point cp=creator.depthProjection.transform(dp);
		
		/* Store the camera-space point: */
		points.push_back(cp);
		
		/* Initialize the least-squares system with the camera-space point: */
		double a[4];
		a[0]=2.0*cp[0];
		a[1]=2.0*cp[1];
		a[2]=2.0*cp[2];
		a[3]=1.0;
		double b=-Geometry::sqr(cp);
		system[0]=a[0]*a[0];
		system[1]=a[0]*a[1];
		system[2]=a[0]*a[2];
		system[3]=a[0]*a[3];
		system[4]=a[0]*b;
		system[5]=a[1]*a[1];
		system[6]=a[1]*a[2];
		system[7]=a[1]*a[3];
		system[8]=a[1]*b;
		system[9]=a[2]*a[2];
		system[10]=a[2]*a[3];
		system[11]=a[2]*b;
		system[12]=a[3]*a[3];
		system[13]=a[3]*b;
		}
	
	/* Methods: */
	void addPixel(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
		{
		Base::addPixel(x,y,pixel,creator);
		
		/* Calculate the pixel's depth-corrected depth image space position: */
		Point dp(Scalar(x),Scalar(y),Scalar(creator.pixelDepthCorrection[y*creator.depthFrameSize[0]+x].correct(float(pixel))));
		
		/* Unproject the depth image-space point: */
		Point cp=creator.depthProjection.transform(dp);
		
		/* Store the camera-space point: */
		points.push_back(cp);
		
		/* Add the camera-space point to the least-squares system: */
		double a[4];
		a[0]=2.0*cp[0];
		a[1]=2.0*cp[1];
		a[2]=2.0*cp[2];
		a[3]=1.0;
		double b=-Geometry::sqr(cp);
		system[0]+=a[0]*a[0];
		system[1]+=a[0]*a[1];
		system[2]+=a[0]*a[2];
		system[3]+=a[0]*a[3];
		system[4]+=a[0]*b;
		system[5]+=a[1]*a[1];
		system[6]+=a[1]*a[2];
		system[7]+=a[1]*a[3];
		system[8]+=a[1]*b;
		system[9]+=a[2]*a[2];
		system[10]+=a[2]*a[3];
		system[11]+=a[2]*b;
		system[12]+=a[3]*a[3];
		system[13]+=a[3]*b;
		}
	void merge(const SphereBlob& other,const Creator& creator)
		{
		Base::merge(other,creator);
		
		/* Merge the point lists: */
		points.insert(points.end(),other.points.begin(),other.points.end());
		
		/* Merge the least-squares linear systems: */
		for(int i=0;i<14;++i)
			system[i]+=other.system[i];
		}
	Sphere getSphere(void) const
		{
		/* Solve the least-squares linear system: */
		Math::Matrix ata(4,4);
		Math::Matrix atb(4,1);
		ata(0,0)=system[0];
		ata(0,1)=system[1];
		ata(0,2)=system[2];
		ata(0,3)=system[3];
		atb(0)=system[4];
		ata(1,0)=system[1];
		ata(1,1)=system[5];
		ata(1,2)=system[6];
		ata(1,3)=system[7];
		atb(1)=system[8];
		ata(2,0)=system[2];
		ata(2,1)=system[6];
		ata(2,2)=system[9];
		ata(2,3)=system[10];
		atb(2)=system[11];
		ata(3,0)=system[3];
		ata(3,1)=system[7];
		ata(3,2)=system[10];
		ata(3,3)=system[12];
		atb(3)=system[13];
		Math::Matrix x=atb.divideFullPivot(ata);
		
		/* Construct the result sphere: */
		Sphere::Point center(-x(0),-x(1),-x(2));
		Sphere::Scalar radius=Math::sqrt(Geometry::sqr(center)-Sphere::Scalar(x(3)));
		return Sphere(center,radius);
		}
	const std::vector<Point>& getPoints(void) const
		{
		return points;
		}
	};

class BlobForegroundSelector // Functor class to select foreground pixels in Kinect depth images
	{
	/* Elements: */
	private:
	const SphereBlob::Creator& creator; // Reference to the creator object for sphere blobs
	
	/* Constructors and destructors: */
	public:
	BlobForegroundSelector(const SphereBlob::Creator& sCreator)
		:creator(sCreator)
		{
		}
	
	/* Methods: */
	bool operator()(unsigned int x,unsigned int y,const DepthPixel& pixel) const
		{
		bool result=false;
		
		if(pixel<Kinect::FrameSource::invalidDepth)
			{
			/* Project the depth image pixel from depth image space to color image space: */
			Point colorPixel=creator.colorDepthProjection.transform(Point(Scalar(x),Scalar(y),Scalar(creator.pixelDepthCorrection[y*creator.depthFrameSize[0]+x].correct(float(pixel)))));
			
			/* Check if the color pixel is mostly white: */
			if(colorPixel[0]>=Scalar(0)&&colorPixel[0]<Scalar(creator.colorFrameSize[0])&&colorPixel[1]>=Scalar(0)&&colorPixel[1]<Scalar(creator.colorFrameSize[1]))
				{
				const ColorPixel& cp=creator.colorFrame[(unsigned int)(Math::floor(colorPixel[1]))*creator.colorFrameSize[0]+(unsigned int)(Math::floor(colorPixel[0]))];
				ColorPixel::Component min=Math::min(Math::min(cp.rgb[0],cp.rgb[1]),cp.rgb[2]);
				ColorPixel::Component max=Math::max(Math::max(cp.rgb[0],cp.rgb[1]),cp.rgb[2]);
				result=min>=creator.minWhite&&max-min<=creator.maxSpread;
				}
			}
		
		return result;
		}
	};

class BlobMergeChecker // Functor class to check whether two depth image pixels can belong to the same blob
	{
	/* Elements: */
	private:
	int maxDepthDist; // Maximum depth distance between two adjacent pixels
	
	/* Constructors and destructors: */
	public:
	BlobMergeChecker(int sMaxDepthDist)
		:maxDepthDist(sMaxDepthDist)
		{
		}
	
	/* Methods: */
	bool operator()(unsigned int x1,unsigned int y1,const DepthPixel& pixel1,unsigned int x2,unsigned int y2,const DepthPixel& pixel2) const
		{
		return Math::abs(int(pixel1)-int(pixel2))<=maxDepthDist;
		}
	};

class SphereLMFitter // Functor plug-in to fit a fixed-radius sphere to a set of camera-space points using Levenberg-Marquardt minimization
	{
	/* Embedded classes: */
	public:
	typedef ::Scalar Scalar;
	static const int dimension=3; // Dimension of the optimization space
	typedef Geometry::ComponentArray<Scalar,dimension> Derivative; // Type for distance function derivatives
	
	/* Elements: */
	private:
	const std::vector<Point>& points; // List of camera-space points to which to fit a sphere
	Scalar radius; // Desired sphere radius
	Point center; // Current estimated sphere center
	Point centerSave; // Saved estimated sphere center
	
	/* Constructors and destructors: */
	public:
	SphereLMFitter(const std::vector<Point>& sPoints,Scalar sRadius,const Point& sCenter)
		:points(sPoints),
		 radius(sRadius),
		 center(sCenter)
		{
		}
	
	/* Methods: */
	const Point& getCenter(void) const // Returns the estimated center
		{
		return center;
		};
	void save(void) // Saves the current estimate
		{
		centerSave=center;
		};
	void restore(void) // Restores the last saved estimate
		{
		center=centerSave;
		};
	size_t getNumPoints(void) const // Returns the number of target points
		{
		return points.size();
		};
	Scalar calcDistance(size_t index) const // Calculates the distance value for the current estimate and the given target point
		{
		return Geometry::dist(points[index],center)-radius;
		};
	Derivative calcDistanceDerivative(size_t index) const // Calculates the derivative of the distance function for the current estimate and the given target point
		{
		Derivative result;
		Scalar dist=Geometry::dist(points[index],center);
		for(int i=0;i<3;++i)
			result[i]=-(points[index][i]-center[i])/dist;
		return result;
		};
	Scalar calcMag(void) const // Returns the magnitude of the current estimate
		{
		return Math::sqrt(Geometry::sqr(center));
		};
	void increment(Derivative increment) // Increments the current estimate by the given difference vector
		{
		for(int i=0;i<3;++i)
			center[i]-=increment[i];
		};
	void normalize(void) // Normalizes the current estimate
		{
		};
	};

}

/********************************
Methods of class SphereExtractor:
********************************/

void* SphereExtractor::frameProcessingThreadMethod(void)
	{
	unsigned int depthFrameVersion=0;
	Kinect::FrameBuffer depthFrame;
	Kinect::FrameBuffer colorFrame;
	
	/* Prepare a blob creator object: */
	SphereBlob::Creator blobCreator;
	for(int i=0;i<2;++i)
		blobCreator.depthFrameSize[i]=depthFrameSize[i];
	blobCreator.pixelDepthCorrection=dcBuffer;
	blobCreator.depthProjection=depthProjection;
	PTransform shift=PTransform::identity;
	PTransform::Matrix& shm=shift.getMatrix();
	shm(0,3)=Scalar(0.5);
	shm(1,3)=Scalar(0.5);
	blobCreator.depthProjection*=shift;
	for(int i=0;i<2;++i)
		blobCreator.colorFrameSize[i]=colorFrameSize[i];
	blobCreator.colorDepthProjection=PTransform::identity;
	PTransform::Matrix& cdpm=blobCreator.colorDepthProjection.getMatrix();
	cdpm(0,0)=Scalar(colorFrameSize[0]);
	cdpm(1,1)=Scalar(colorFrameSize[1]);
	blobCreator.colorDepthProjection*=colorProjection;
	blobCreator.colorDepthProjection*=shift;
	
	while(true)
		{
		/* Get the next incoming depth frame: */
		{
		Threads::MutexCond::Lock inDepthFrameLock(inDepthFrameCond);
		
		/* Wait until a new depth frame arrives: */
		while(depthFrameVersion==inDepthFrameVersion)
			inDepthFrameCond.wait(inDepthFrameLock);
		
		/* Grab the new raw depth frame: */
		depthFrameVersion=inDepthFrameVersion;
		depthFrame=inDepthFrame;
		}
		
		/* Grab whatever the current color frame is: */
		{
		Threads::Mutex::Lock inColorFrameLock(inColorFrameMutex);
		colorFrame=inColorFrame;
		}
		if(colorFrame.getBuffer()==0)
			continue;
		
		/* Extract all foreground blobs from the raw depth frame: */
		blobCreator.colorFrame=static_cast<const ColorPixel*>(colorFrame.getBuffer());
		blobCreator.minWhite=minWhite;
		blobCreator.maxSpread=maxSpread;
		const DepthPixel* framePixels=static_cast<const DepthPixel*>(depthFrame.getBuffer());
		BlobForegroundSelector bfs(blobCreator);
		BlobMergeChecker bmc(1);
		std::vector<SphereBlob> blobs=Images::extractBlobs<SphereBlob>(depthFrameSize,framePixels,bfs,bmc,blobCreator);
		
		/* Find all large-enough blobs whose spheres match the desired radius and have low approximation residual: */
		SphereList& spheres=sphereLists.startNewValue();
		spheres.clear();
		Sphere bestSphere(Point::origin,Scalar(0));
		Scalar bestRms=sphereRadius*maxResidual;
		for(std::vector<SphereBlob>::iterator bIt=blobs.begin();bIt!=blobs.end();++bIt)
			if(bIt->numPixels>=minBlobSize)
				{
				try
					{
					/* Get the blob's sphere equation: */
					Sphere blobSphere=bIt->getSphere();
					
					if(Math::abs(blobSphere.getRadius()-sphereRadius)<=sphereRadius*radiusTolerance)
						{
						/* Fit a fixed-radius sphere to the blob via non-linear optimization: */
						Geometry::LevenbergMarquardtMinimizer<SphereLMFitter> minimizer;
						SphereLMFitter sphereFitter(bIt->getPoints(),sphereRadius,bIt->getSphere().getCenter());
						Scalar rms=Math::sqrt(Scalar(2)*minimizer.minimize(sphereFitter)/Scalar(bIt->numPixels));
						
						/* Check if this is the best sphere yet: */
						if(bestRms>rms)
							{
							bestSphere=Sphere(sphereFitter.getCenter(),sphereRadius);
							bestRms=rms;
							}
						}
					}
				catch(Math::Matrix::RankDeficientError)
					{
					/* Ignore this blob */
					}
				}
		
		/* Check if a matching sphere was found: */
		if(bestRms<sphereRadius*maxResidual)
			{
			/* Push the sphere to the main thread: */
			spheres.push_back(bestSphere);
			}
		
		/* Post the newly-extracted sphere list into the triple buffer: */
		sphereLists.postNewValue();
		
		/* Call the streaming callback with the list of found spheres: */
		if(streamingCallback!=0)
			(*streamingCallback)(spheres);
		}
	
	return 0;
	}

SphereExtractor::SphereExtractor(Kinect::FrameSource& frameSource,const SphereExtractor::PixelDepthCorrection* sDcBuffer)
	:dcBuffer(sDcBuffer),depthProjection(frameSource.getIntrinsicParameters().depthProjection),colorProjection(frameSource.getIntrinsicParameters().colorProjection),
	 sphereRadius(0),
	 minWhite(192),maxSpread(32),minBlobSize(10),radiusTolerance(0.2),maxResidual(0.1),
	 inDepthFrameVersion(0),
	 streamingCallback(0)
	{
	/* Copy the frame source's depth and color frame sizes: */
	for(int i=0;i<2;++i)
		{
		depthFrameSize[i]=frameSource.getActualFrameSize(Kinect::FrameSource::DEPTH)[i];
		colorFrameSize[i]=frameSource.getActualFrameSize(Kinect::FrameSource::COLOR)[i];
		}
	}

SphereExtractor::~SphereExtractor(void)
	{
	/* Stop background processing, just in case: */
	stopStreaming();
	}

void SphereExtractor::setSphereRadius(SphereExtractor::Scalar newSphereRadius)
	{
	sphereRadius=newSphereRadius;
	}

void SphereExtractor::setMatchLimits(unsigned int newMinWhite,unsigned int newMaxSpread,size_t newMinBlobSize,Scalar newRadiusTolerance,Scalar newMaxResidual)
	{
	minWhite=ColorPixel::Component(newMinWhite);
	maxSpread=ColorPixel::Component(newMaxSpread);
	minBlobSize=newMinBlobSize;
	radiusTolerance=newRadiusTolerance;
	maxResidual=newMaxResidual;
	}

void SphereExtractor::startStreaming(SphereExtractor::StreamingCallback* newStreamingCallback)
	{
	/* Delete the old streaming callback and install the new one: */
	delete streamingCallback;
	streamingCallback=newStreamingCallback;
	
	/* Start the depth frame processing thread: */
	frameProcessingThread.start(this,&SphereExtractor::frameProcessingThreadMethod);
	}

void SphereExtractor::setDepthFrame(const Kinect::FrameBuffer& newDepthFrame)
	{
	/* Put the new depth frame into the depth frame input slot and wake up the frame processing thread: */
	Threads::MutexCond::Lock inDepthFrameLock(inDepthFrameCond);
	++inDepthFrameVersion;
	inDepthFrame=newDepthFrame;
	inDepthFrameCond.signal();
	}

void SphereExtractor::setColorFrame(const Kinect::FrameBuffer& newColorFrame)
	{
	/* Put the new color frame into the color frame input slot: */
	Threads::Mutex::Lock inColorFrameLock(inColorFrameMutex);
	inColorFrame=newColorFrame;
	}

void SphereExtractor::stopStreaming(void)
	{
	if(!frameProcessingThread.isJoined())
		{
		/* Shut down the depth processing thread: */
		frameProcessingThread.cancel();
		frameProcessingThread.join();
		}
	
	/* Delete the streaming callback: */
	delete streamingCallback;
	streamingCallback=0;
	}
