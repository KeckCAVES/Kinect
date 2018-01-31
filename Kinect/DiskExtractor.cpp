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

#include <Kinect/DiskExtractor.h>

// DEBUGGING
#include <iostream>
#include <Realtime/Time.h>
#include <Geometry/OutputOperators.h>

#include <stdexcept>
#include <Misc/Utility.h>
#include <Misc/FunctionCalls.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Geometry/Matrix.h>
#include <Images/ExtractBlobs.h>

namespace Kinect {

/********************************
Declarations of embedded classes:
********************************/

#if 0 // Not used anymore

struct DiskExtractor::DepthCentroidBlob:public Images::Blob<DiskExtractor::DepthPixel> // Structure to calculate 3D centroids of blobs in depth image space
	{
	/* Embedded classes: */
	public:
	typedef DepthPixel Pixel;
	typedef Images::Blob<DepthPixel> Base;
	
	struct Creator:public Base::Creator
		{
		/* Elements: */
		public:
		unsigned int frameSize[2]; // Size of depth images
		const PixelDepthCorrection* depthCorrection; // 2D array of per-pixel depth correction factors
		const ImagePoint* framePixels; // 2D array of lens distortion-corrected depth image pixels
		PTransform depthProjection; // Projection from depth image space into camera space
		};
	
	/* Elements: */
	PTransform::HVector c; // Accumulated centroid components (x, y, z) and total weight in depth image space
	
	/* Constructors and destructors: */
	DepthCentroidBlob(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
		:Base(x,y,pixel,creator)
		{
		/* Calculate the pixel's linear index: */
		unsigned int index=y*creator.frameSize[0]+x;
		
		/* Calculate the pixel's depth image position: */
		PTransform::Point p(creator.framePixels[index][0],creator.framePixels[index][1],Scalar(pixel));
		if(creator.depthCorrection!=0)
			p[2]=Scalar(creator.depthCorrection[index].correct(float(pixel)));
		
		/* Unproject the pixel to calculate its centroid accumulation weight as undistortion function scale times camera-space z coordinate to the fourth: */
		const PTransform::Matrix& m=creator.depthProjection.getMatrix();
		Scalar weight=creator.framePixels[index].value*Math::sqr(Math::sqr((m(2,0)*p[0]+m(2,1)*p[1]+m(2,2)*p[2]+m(2,3))/(m(3,0)*p[0]+m(3,1)*p[1]+m(3,2)*p[2]+m(3,3))));
		
		/* Accumulate the pixel: */
		c[0]=p[0]*weight;
		c[1]=p[1]*weight;
		c[2]=p[2]*weight;
		c[3]=weight;
		}
	
	/* Methods: */
	void addPixel(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
		{
		Base::addPixel(x,y,pixel,creator);
		
		/* Calculate the pixel's linear index: */
		unsigned int index=y*creator.frameSize[0]+x;
		
		/* Calculate the pixel's depth image position: */
		PTransform::Point p(creator.framePixels[index][0],creator.framePixels[index][1],Scalar(pixel));
		if(creator.depthCorrection!=0)
			p[2]=Scalar(creator.depthCorrection[index].correct(float(pixel)));
		
		/* Unproject the pixel to calculate its centroid accumulation weight as undistortion function scale times camera-space z coordinate to the fourth: */
		const PTransform::Matrix& m=creator.depthProjection.getMatrix();
		Scalar weight=creator.framePixels[index].value*Math::sqr(Math::sqr((m(2,0)*p[0]+m(2,1)*p[1]+m(2,2)*p[2]+m(2,3))/(m(3,0)*p[0]+m(3,1)*p[1]+m(3,2)*p[2]+m(3,3))));
		
		/* Accumulate the pixel: */
		c[0]+=p[0]*weight;
		c[1]+=p[1]*weight;
		c[2]+=p[2]*weight;
		c[3]+=weight;
		}
	void merge(const DepthCentroidBlob& other,const Creator& creator)
		{
		Base::merge(other,creator);
		
		for(int i=0;i<4;++i)
			c[i]+=other.c[i];
		}
	PTransform::Point getCentroid(const PTransform& depthProjection) const // Returns the blob's centroid in camera space
		{
		return depthProjection.transform(c).toPoint();
		}
	};

#endif

struct DiskExtractor::DepthPCABlob:public Images::Blob<DiskExtractor::DepthPixel> // Structure to calculate 3D plane equations of blobs in depth image space
	{
	/* Embedded classes: */
	public:
	typedef DepthPixel Pixel;
	typedef Images::Blob<DepthPixel> Base;
	typedef Geometry::Matrix<double,3,3> Matrix; // Type for covariance matrices
	
	struct Creator:public Base::Creator
		{
		/* Elements: */
		public:
		unsigned int frameSize[2]; // Size of depth images
		const PixelDepthCorrection* depthCorrection; // 2D array of per-pixel depth correction factors
		const ImagePoint* framePixels; // 2D array of lens distortion-corrected depth image pixels
		PTransform depthProjection; // Projection from depth image space into camera space
		unsigned int trackingIndex; // Linear index of a pixel whose blob to track through the extraction process
		};
	
	/* Elements: */
	double pxpxs,pxpys,pxpzs,pypys,pypzs,pzpzs,pxs,pys,pzs; // Accumulated components of covariance matrix and centroid
	double weights; // Sum of all accumulated point weights
	bool tracked; // Flag whether this blob contains the tracking pixel
	
	/* Constructors and destructors: */
	DepthPCABlob(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
		:Base(x,y,pixel,creator)
		{
		/* Calculate the pixel's linear index: */
		unsigned int index=y*creator.frameSize[0]+x;
		
		/* Calculate the pixel's depth image position: */
		Point p(creator.framePixels[index][0],creator.framePixels[index][1],Scalar(pixel));
		if(creator.depthCorrection!=0)
			p[2]=Scalar(creator.depthCorrection[index].correct(float(pixel)));
		
		/* Unproject the pixel to calculate its centroid accumulation weight as undistortion function scale times camera-space z coordinate to the fourth: */
		const PTransform::Matrix& m=creator.depthProjection.getMatrix();
		double weight=double(creator.framePixels[index].value*Math::sqr(Math::sqr((m(2,0)*p[0]+m(2,1)*p[1]+m(2,2)*p[2]+m(2,3))/(m(3,0)*p[0]+m(3,1)*p[1]+m(3,2)*p[2]+m(3,3)))));
		
		/* Accumulate the pixel: */
		pxpxs=double(p[0])*double(p[0])*weight;
		pxpys=double(p[0])*double(p[1])*weight;
		pxpzs=double(p[0])*double(p[2])*weight;
		pypys=double(p[1])*double(p[1])*weight;
		pypzs=double(p[1])*double(p[2])*weight;
		pzpzs=double(p[2])*double(p[2])*weight;
		pxs=double(p[0])*weight;
		pys=double(p[1])*weight;
		pzs=double(p[2])*weight;
		weights=weight;
		
		/* Check if the new pixel is the tracking pixel: */
		tracked=index==creator.trackingIndex;
		}
	
	/* Methods: */
	void addPixel(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
		{
		Base::addPixel(x,y,pixel,creator);
		
		/* Calculate the pixel's linear index: */
		unsigned int index=y*creator.frameSize[0]+x;
		
		/* Calculate the pixel's depth image position: */
		Point p(creator.framePixels[index][0],creator.framePixels[index][1],Scalar(pixel));
		if(creator.depthCorrection!=0)
			p[2]=Scalar(creator.depthCorrection[index].correct(float(pixel)));
		
		/* Unproject the pixel to calculate its centroid accumulation weight as undistortion function scale times camera-space z coordinate to the fourth: */
		const PTransform::Matrix& m=creator.depthProjection.getMatrix();
		double weight=double(creator.framePixels[index].value*Math::sqr(Math::sqr((m(2,0)*p[0]+m(2,1)*p[1]+m(2,2)*p[2]+m(2,3))/(m(3,0)*p[0]+m(3,1)*p[1]+m(3,2)*p[2]+m(3,3)))));
		
		/* Accumulate the pixel: */
		pxpxs+=double(p[0])*double(p[0])*weight;
		pxpys+=double(p[0])*double(p[1])*weight;
		pxpzs+=double(p[0])*double(p[2])*weight;
		pypys+=double(p[1])*double(p[1])*weight;
		pypzs+=double(p[1])*double(p[2])*weight;
		pzpzs+=double(p[2])*double(p[2])*weight;
		pxs+=double(p[0])*weight;
		pys+=double(p[1])*weight;
		pzs+=double(p[2])*weight;
		weights+=weight;
		
		/* Check if the new pixel is the tracking pixel: */
		tracked=tracked||index==creator.trackingIndex;
		}
	void merge(const DepthPCABlob& other,const Creator& creator)
		{
		Base::merge(other,creator);
		
		pxpxs+=other.pxpxs;
		pxpys+=other.pxpys;
		pxpzs+=other.pxpzs;
		pypys+=other.pypys;
		pypzs+=other.pypzs;
		pzpzs+=other.pzpzs;
		pxs+=other.pxs;
		pys+=other.pys;
		pzs+=other.pzs;
		weights+=other.weights;
		
		/* Keep tracked of the tracking pixel: */
		tracked=tracked||other.tracked;
		}
	Point calcCentroid(void) const // Returns the blob's centroid in depth image space
		{
		return Point(Scalar(pxs/weights),Scalar(pys/weights),Scalar(pzs/weights));
		}
	Matrix calcCovariance(void) const // Returns the blob's covariance matrix for PCA calculation
		{
		Matrix cov;
		cov(0,0)=(pxpxs-(pxs*pxs)/weights)/weights;
		cov(0,1)=(pxpys-(pxs*pys)/weights)/weights;
		cov(0,2)=(pxpzs-(pxs*pzs)/weights)/weights;
		cov(1,0)=cov(0,1);
		cov(1,1)=(pypys-(pys*pys)/weights)/weights;
		cov(1,2)=(pypzs-(pys*pzs)/weights)/weights;
		cov(2,0)=cov(0,2);
		cov(2,1)=cov(1,2);
		cov(2,2)=(pzpzs-(pzs*pzs)/weights)/weights;
		return cov;
		}
	unsigned int calcEigenvalues(const Matrix& cov,double eigenvalues[3]) const // Calculates the eigenvalues of the covariance matrix in order of decreasing absolute value; returns the number of distinct real roots
		{
		/* Calculate the coefficients of the covariance matrix' characteristic polynomial: */
		double cp[3];
		cp[0]=-cov(0,0)-cov(1,1)-cov(2,2);
		cp[1]=cov(0,0)*cov(1,1)+cov(0,0)*cov(2,2)+cov(1,1)*cov(2,2)-cov(0,1)*cov(1,0)-cov(0,2)*cov(2,0)-cov(1,2)*cov(2,1);
		cp[2]=-cov(0,0)*(cov(1,1)*cov(2,2)-cov(1,2)*cov(2,1))+cov(0,1)*(cov(1,0)*cov(2,2)-cov(1,2)*cov(2,0))-cov(0,2)*(cov(1,0)*cov(2,1)-cov(1,1)*cov(2,0));
		
		/* Find all roots of the characteristic polynomial: */
		double q=(Math::sqr(cp[0])-3.0*cp[1])/9.0;
		double q3=Math::sqr(q)*q;
		double r=((2.0*Math::sqr(cp[0])-9.0*cp[1])*cp[0]+27.0*cp[2])/54.0;
		if(Math::sqr(r)<q3)
			{
			/* There are three real roots: */
			double theta=Math::acos(r/Math::sqrt(q3));
			eigenvalues[0]=-2.0*Math::sqrt(q)*Math::cos(theta/3.0)-cp[0]/3.0;
			eigenvalues[1]=-2.0*Math::sqrt(q)*Math::cos((theta+2.0*Math::Constants<double>::pi)/3.0)-cp[0]/3.0;
			eigenvalues[2]=-2.0*Math::sqrt(q)*Math::cos((theta-2.0*Math::Constants<double>::pi)/3.0)-cp[0]/3.0;
			
			/* Use Newton iteration to clean up the roots: */
			for(int i=0;i<3;++i)
				for(int j=0;j<5;++j)
					{
					double f=((eigenvalues[i]+cp[0])*eigenvalues[i]+cp[1])*eigenvalues[i]+cp[2];
					double fp=(3.0*eigenvalues[i]+2.0*cp[0])*eigenvalues[i]+cp[1];
					double s=f/fp;
					eigenvalues[i]-=s;
					}
			
			/* Sort the roots by descending absolute value: */
			if(Math::abs(eigenvalues[0])<Math::abs(eigenvalues[1]))
				Misc::swap(eigenvalues[0],eigenvalues[1]);
			if(Math::abs(eigenvalues[1])<Math::abs(eigenvalues[2]))
				Misc::swap(eigenvalues[1],eigenvalues[2]);
			if(Math::abs(eigenvalues[0])<Math::abs(eigenvalues[1]))
				Misc::swap(eigenvalues[0],eigenvalues[1]);
			
			return 3;
			}
		else
			{
			/* There is only one real root: */
			double a=Math::pow(Math::abs(r)+Math::sqrt(Math::sqr(r)-q3),1.0/3.0);
			if(r>0.0)
				a=-a;
			double b=a==0.0?0.0:q/a;
			eigenvalues[0]=a+b-cp[0]/3.0;
			
			/* Use Newton iteration to clean up the root: */
			for(int j=0;j<5;++j)
				{
				double f=((eigenvalues[0]+cp[0])*eigenvalues[0]+cp[1])*eigenvalues[0]+cp[2];
				double fp=(3.0*eigenvalues[0]+2.0*cp[0])*eigenvalues[0]+cp[1];
				double s=f/fp;
				eigenvalues[0]-=s;
				}
			
			/* Copy the eigenvalue twice: */
			eigenvalues[1]=eigenvalues[2]=eigenvalues[0];
			
			return 1;
			}
		}
	PTransform::Vector calcEigenvector(const Matrix& cov,double eigenvalue) const // Returns the eigenvector of the covariance matrix for the given eigenvalue
		{
		/* Create the modified covariance matrix: */
		Matrix c=cov;
		for(int i=0;i<3;++i)
			c(i,i)-=eigenvalue;
		
		/* Find the null space of the modified covariance matrix: */
		int rowIndices[3];
		for(int i=0;i<3;++i)
			rowIndices[i]=i;
		for(int step=0;step<3-1;++step)
			{
			/* Find the full pivot: */
			double pivot=Math::abs(c(step,step));
			int pivotRow=step;
			int pivotCol=step;
			for(int i=step;i<3;++i)
				for(int j=step;j<3;++j)
					{
					double val=Math::abs(c(i,j));
					if(pivot<val)
						{
						pivot=val;
						pivotRow=i;
						pivotCol=j;
						}
					}
			
			/* Swap current and pivot rows if necessary: */
			if(pivotRow!=step)
				{
				/* Swap rows step and pivotRow: */
				for(int j=0;j<3;++j)
					Misc::swap(c(step,j),c(pivotRow,j));
				}
			
			/* Swap current and pivot columns if necessary: */
			if(pivotCol!=step)
				{
				/* Swap columns step and pivotCol: */
				for(int i=0;i<3;++i)
					Misc::swap(c(i,step),c(i,pivotCol));
				Misc::swap(rowIndices[step],rowIndices[pivotCol]);
				}
			
			/* Combine all rows with the current row: */
			for(int i=step+1;i<3;++i)
				{
				/* Combine rows i and step: */
				double factor=-c(i,step)/c(step,step);
				for(int j=step+1;j<3;++j)
					c(i,j)+=c(step,j)*factor;
				}
			}
		
		/* Calculate the swizzled result using backsubstitution: */
		double x[3];
		x[3-1]=1.0;
		for(int i=3-2;i>=0;--i)
			{
			x[i]=0.0;
			for(int j=i+1;j<3;++j)
				x[i]-=c(i,j)*x[j];
			x[i]/=c(i,i);
			}
		
		/* Unswizzle and normalize the result: */
		PTransform::Vector result;
		for(int i=0;i<3;++i)
			result[rowIndices[i]]=Scalar(x[i]);
		result.normalize();
		return result;
		}
	bool isTracked(void) const // Returns true if this blob contains the tracking pixel
		{
		return tracked;
		}
	};

namespace {

/**************
Helper classes:
**************/

class BlobForegroundSelector // Functor class to select foreground pixels
	{
	/* Methods: */
	public:
	bool operator()(unsigned int x,unsigned int y,const Kinect::FrameSource::DepthPixel& pixel) const
		{
		return pixel<Kinect::FrameSource::invalidDepth;
		}
	};

class BlobMergeChecker // Functor class to check whether two pixels can belong to the same blob
	{
	/* Elements: */
	private:
	int maxDepthDist;
	
	/* Constructors and destructors: */
	public:
	BlobMergeChecker(int sMaxDepthDist)
		:maxDepthDist(sMaxDepthDist)
		{
		}
	
	/* Methods: */
	bool operator()(unsigned int x1,unsigned int y1,const Kinect::FrameSource::DepthPixel& pixel1,unsigned int x2,unsigned int y2,const Kinect::FrameSource::DepthPixel& pixel2) const
		{
		return Math::abs(int(pixel1)-int(pixel2))<=maxDepthDist;
		}
	};

}

/******************************
Methods of class DiskExtractor:
******************************/

void* DiskExtractor::diskExtractorThreadMethod(void)
	{
	while(true)
		{
		FrameBuffer frame;
		int bmd;
		unsigned int mnp;
		Scalar drMin,drMax;
		Scalar df;
		unsigned int tp;
		TrackingCallback* tc;
		{
		Threads::MutexCond::Lock newFrameLock(newFrameCond);
		
		/* Wait for the next incoming depth frame or for shutdown: */
		while(keepProcessing&&!newFrame.isValid())
			newFrameCond.wait(newFrameLock);
		if(!keepProcessing)
			break;
		
		/* Grab the new depth frame: */
		frame=newFrame;
		newFrame.invalidate();
		
		/* Grab the current disk extraction parameters: */
		bmd=maxBlobMergeDist;
		mnp=minNumPixels;
		drMin=diskRadius/diskRadiusMargin;
		drMax=diskRadius*diskRadiusMargin;
		df=diskFlatness;
		
		/* Grab the current pixel tracking parameters: */
		tp=trackingPixel;
		tc=trackingCallback;
		}
		
		// DEBUGGING
		// Realtime::TimePointMonotonic timer;
		
		/* Extract all foreground blobs from the raw depth frame: */
		const DepthPixel* depthFramePixels=frame.getData<DepthPixel>();
		BlobForegroundSelector bfs;
		BlobMergeChecker bmc(bmd);
		DepthPCABlob::Creator blobCreator;
		for(int i=0;i<2;++i)
			blobCreator.frameSize[i]=frameSize[i];
		blobCreator.depthCorrection=depthCorrection;
		blobCreator.framePixels=framePixels;
		blobCreator.depthProjection=depthProjection;
		blobCreator.trackingIndex=tp;
		std::vector<DepthPCABlob> blobs=Images::extractBlobs<DepthPCABlob>(frameSize,depthFramePixels,bfs,bmc,blobCreator);
		
		/* Create the result list: */
		DiskList extractionResult;
		extractionResult.reserve(blobs.size());
		for(std::vector<DepthPCABlob>::iterator bIt=blobs.begin();bIt!=blobs.end();++bIt)
			if(bIt->numPixels>=mnp||bIt->isTracked())
				{
				/* Calculate the blob's principal components: */
				Point centroid=bIt->calcCentroid();
				DepthPCABlob::Matrix cov=bIt->calcCovariance();
				double eigenvalues[3];
				bIt->calcEigenvalues(cov,eigenvalues);
				PTransform::Vector axes[3];
				for(int i=0;i<3;++i)
					axes[i]=bIt->calcEigenvector(cov,eigenvalues[i])*Math::sqrt(eigenvalues[i]);
				
				// DEBUGGING
				#if 0
				
				if(Math::abs(centroid[0]-245.0)<=10.0&&Math::abs(centroid[1]-232.0)<=10.0)
					{
					std::cout<<bIt->numPixels<<", "<<centroid<<std::endl;
					std::cout<<eigenvalues[0]<<", "<<eigenvalues[1]<<", "<<eigenvalues[2]<<std::endl;
					std::cout<<axes[0]<<", "<<axes[1]<<", "<<axes[2]<<std::endl;
					}
				
				#endif
				
				/* Calculate the blob's extents in camera space: */
				Scalar axisLengths[3];
				for(int i=0;i<3;++i)
					{
					axes[i]=depthProjection.transform(centroid+axes[i])-depthProjection.transform(centroid-axes[i]);
					axisLengths[i]=Geometry::mag(axes[i]);
					}
				
				// DEBUGGING
				#if 0
				
				if(Math::abs(centroid[0]-245.0)<=10.0&&Math::abs(centroid[1]-232.0)<=10.0)
					{
					std::cout<<axes[0]<<", "<<axes[1]<<", "<<axes[2]<<std::endl;
					std::cout<<axisLengths[0]<<", "<<axisLengths[1]<<", "<<axisLengths[2]<<std::endl;
					}
				
				#endif
				
				/* Check if the blob fits the search parameters: */
				bool blobValid=axisLengths[0]>=drMin&&axisLengths[0]<=drMax&&axisLengths[1]>=drMin&&axisLengths[1]<=drMax&&axisLengths[2]<=df;
				if(bIt->isTracked()||blobValid)
					{
					/* Store the extracted disk: */
					Disk disk;
					disk.center=depthProjection.transform(centroid);
					disk.normal=axes[0]^axes[1];
					Scalar nLen=Geometry::mag(disk.normal);
					if(disk.normal[2]>Scalar(0))
						nLen=-nLen;
					disk.normal/=nLen;
					
					disk.numPixels=bIt->numPixels;
					disk.radius=Math::sqrt(axisLengths[0]*axisLengths[1]);
					disk.flatness=axisLengths[2];
					disk.numPixels=bIt->numPixels;
					
					if(blobValid)
						extractionResult.push_back(disk);
					if(bIt->isTracked()&&tc!=0)
						{
						/* Call the tracking callback: */
						(*tc)(disk);
						}
					
					// DEBUGGING
					// std::cout<<"("<<centroid[0]<<", "<<centroid[1]<<", "<<centroid[2]<<"), "<<disk.radius<<std::endl;
					}
				}
		
		// DEBUGGING
		// double elapsed=timer.setAndDiff();
		// std::cout<<"Extracted "<<extractionResult.size()<<" disks in "<<elapsed*1000.0<<"ms"<<std::endl;
		
		if(extractionResultCallback!=0)
			{
			/* Call the result callback: */
			(*extractionResultCallback)(extractionResult);
			}
		}
	
	return 0;
	}

DiskExtractor::DiskExtractor(const unsigned int sFrameSize[2],const FrameSource::DepthCorrection* dc,const FrameSource::IntrinsicParameters& ips)
	:privateDepthCorrection(true),depthCorrection(0),framePixels(0),
	 maxBlobMergeDist(8),
	 minNumPixels(500),
	 diskRadius(60),diskRadiusMargin(1.1),diskFlatness(5.0),
	 keepProcessing(false),
	 extractionResultCallback(0),
	 trackingPixel(~0x0U),trackingCallback(0)
	{
	/* Copy the frame size: */
	for(int i=0;i<2;++i)
		frameSize[i]=sFrameSize[i];
	
	if(dc!=0)
		{
		/* Create an array of per-pixel depth correction factors: */
		depthCorrection=dc->getPixelCorrection(frameSize);
		}
	
	/* Pre-compute a 2D array of lens distortion-corrected image pixel positions: */
	framePixels=new ImagePoint[frameSize[1]*frameSize[0]];
	if(ips.depthLensDistortion.isIdentity())
		{
		/* Create uncorrected pixel positions: */
		ImagePoint* fpPtr=framePixels;
		for(unsigned int y=0;y<frameSize[1];++y)
			for(unsigned int x=0;x<frameSize[0];++x,++fpPtr)
				{
				(*fpPtr)[0]=Scalar(x)+Scalar(0.5);
				(*fpPtr)[1]=Scalar(y)+Scalar(0.5);
				fpPtr->value=LensDistortion::Scalar(1);
				}
		}
	else
		{
		/* Create lens distortion-corrected pixel positions: */
		ImagePoint* fpPtr=framePixels;
		for(unsigned int y=0;y<frameSize[1];++y)
			for(unsigned int x=0;x<frameSize[0];++x,++fpPtr)
				{
				/* Undistort the image point: */
				LensDistortion::Point dp(LensDistortion::Scalar(x)+LensDistortion::Scalar(0.5),LensDistortion::Scalar(y)+LensDistortion::Scalar(0.5));
				LensDistortion::Point up=ips.depthLensDistortion.undistortPixel(dp);
				
				/* Store the undistorted point: */
				(*fpPtr)[0]=Scalar(up[0]);
				(*fpPtr)[1]=Scalar(up[1]);
				
				/* Calculate the inverse distortion scale at the undistorted position: */
				fpPtr->value=LensDistortion::Scalar(1)/ips.depthLensDistortion.distortScalePixel(up);
				}
		}
	
	/* Copy the depth projection matrix: */
	depthProjection=ips.depthProjection;
	}

DiskExtractor::DiskExtractor(const unsigned int sFrameSize[2],const DiskExtractor::PixelDepthCorrection* sDepthCorrection,const FrameSource::IntrinsicParameters& ips)
	:privateDepthCorrection(false),depthCorrection(const_cast<PixelDepthCorrection*>(sDepthCorrection)),framePixels(0),
	 maxBlobMergeDist(8),
	 minNumPixels(500),
	 diskRadius(60),diskRadiusMargin(1.1),diskFlatness(5.0),
	 keepProcessing(false),
	 extractionResultCallback(0),
	 trackingPixel(~0x0U),trackingCallback(0)
	{
	/* Copy the frame size: */
	for(int i=0;i<2;++i)
		frameSize[i]=sFrameSize[i];
	
	/* Pre-compute a 2D array of lens distortion-corrected image pixel positions: */
	framePixels=new ImagePoint[frameSize[1]*frameSize[0]];
	if(ips.depthLensDistortion.isIdentity())
		{
		/* Create uncorrected pixel positions: */
		ImagePoint* fpPtr=framePixels;
		for(unsigned int y=0;y<frameSize[1];++y)
			for(unsigned int x=0;x<frameSize[0];++x,++fpPtr)
				{
				(*fpPtr)[0]=Scalar(x)+Scalar(0.5);
				(*fpPtr)[1]=Scalar(y)+Scalar(0.5);
				fpPtr->value=LensDistortion::Scalar(1);
				}
		}
	else
		{
		/* Create lens distortion-corrected pixel positions: */
		ImagePoint* fpPtr=framePixels;
		for(unsigned int y=0;y<frameSize[1];++y)
			for(unsigned int x=0;x<frameSize[0];++x,++fpPtr)
				{
				/* Undistort the image point: */
				LensDistortion::Point dp(LensDistortion::Scalar(x)+LensDistortion::Scalar(0.5),LensDistortion::Scalar(y)+LensDistortion::Scalar(0.5));
				LensDistortion::Point up=ips.depthLensDistortion.undistortPixel(dp);
				
				/* Store the undistorted point: */
				(*fpPtr)[0]=Scalar(up[0]);
				(*fpPtr)[1]=Scalar(up[1]);
				
				/* Calculate the inverse distortion scale at the undistorted position: */
				fpPtr->value=LensDistortion::Scalar(1)/ips.depthLensDistortion.distortScalePixel(up);
				}
		}
	
	/* Copy the depth projection matrix: */
	depthProjection=ips.depthProjection;
	}

DiskExtractor::~DiskExtractor(void)
	{
	/* Stop streaming if still active: */
	if(!diskExtractorThread.isJoined())
		{
		/* Shut down the disk extraction thread: */
		{
		Threads::MutexCond::Lock newFrameLock(newFrameCond);
		
		keepProcessing=false;
		
		/* Wake up the disk extractor thread: */
		newFrameCond.signal();
		}
		
		/* Wait until the disk extraction thread terminates: */
		diskExtractorThread.join();
		}
	
	if(privateDepthCorrection)
		delete[] depthCorrection;
	delete[] framePixels;
	delete extractionResultCallback;
	delete trackingCallback;
	}

void DiskExtractor::setMaxBlobMergeDist(int newMaxBlobMergeDist)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	maxBlobMergeDist=newMaxBlobMergeDist;
	}

void DiskExtractor::setMinNumPixels(unsigned int newMinNumPixels)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	minNumPixels=newMinNumPixels;
	}

void DiskExtractor::setDiskRadius(DiskExtractor::Scalar newDiskRadius)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	diskRadius=newDiskRadius;
	}

void DiskExtractor::setDiskRadiusMargin(DiskExtractor::Scalar newDiskRadiusMargin)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	diskRadiusMargin=newDiskRadiusMargin;
	}

void DiskExtractor::setDiskFlatness(DiskExtractor::Scalar newDiskFlatness)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	diskFlatness=newDiskFlatness;
	}

DiskExtractor::DiskList DiskExtractor::processFrame(const FrameBuffer& frame) const
	{
	/* Grab the current disk extraction parameters: */
	int bmd=maxBlobMergeDist;
	unsigned int mnp=minNumPixels;
	Scalar drMin=diskRadius/diskRadiusMargin;
	Scalar drMax=diskRadius*diskRadiusMargin;
	Scalar df=diskFlatness;
	unsigned int tp=trackingPixel;
	TrackingCallback* tc=trackingCallback;
	
	/* Extract all foreground blobs from the raw depth frame: */
	const DepthPixel* depthFramePixels=frame.getData<DepthPixel>();
	BlobForegroundSelector bfs;
	BlobMergeChecker bmc(bmd);
	DepthPCABlob::Creator blobCreator;
	for(int i=0;i<2;++i)
		blobCreator.frameSize[i]=frameSize[i];
	blobCreator.depthCorrection=depthCorrection;
	blobCreator.framePixels=framePixels;
	blobCreator.depthProjection=depthProjection;
	blobCreator.trackingIndex=tp;
	std::vector<DepthPCABlob> blobs=Images::extractBlobs<DepthPCABlob>(frameSize,depthFramePixels,bfs,bmc,blobCreator);
	
	/* Create the result list: */
	DiskList extractionResult;
	extractionResult.reserve(blobs.size());
	for(std::vector<DepthPCABlob>::iterator bIt=blobs.begin();bIt!=blobs.end();++bIt)
		if(bIt->numPixels>=mnp||bIt->isTracked())
			{
			/* Calculate the blob's principal components: */
			Point centroid=bIt->calcCentroid();
			DepthPCABlob::Matrix cov=bIt->calcCovariance();
			double eigenvalues[3];
			bIt->calcEigenvalues(cov,eigenvalues);
			PTransform::Vector axes[3];
			for(int i=0;i<3;++i)
				axes[i]=bIt->calcEigenvector(cov,eigenvalues[i])*Math::sqrt(Math::abs(eigenvalues[i]));
			
			// DEBUGGING
			#if 0
			if(Math::abs(centroid[0]-267.0)<5.0&&Math::abs(centroid[1]-221.0)<5.0)
				{
				std::cout<<bIt->numPixels<<", "<<centroid<<std::endl;
				std::cout<<eigenvalues[0]<<", "<<eigenvalues[1]<<", "<<eigenvalues[2]<<std::endl;
				std::cout<<axes[0]<<", "<<Geometry::mag(axes[0])<<std::endl;
				std::cout<<axes[1]<<", "<<Geometry::mag(axes[1])<<std::endl;
				std::cout<<axes[2]<<", "<<Geometry::mag(axes[2])<<std::endl;
				}
			#endif
			
			/* Calculate the blob's extents in camera space: */
			Scalar axisLengths[3];
			for(int i=0;i<3;++i)
				{
				axes[i]=depthProjection.transform(centroid+axes[i])-depthProjection.transform(centroid-axes[i]);
				axisLengths[i]=Geometry::mag(axes[i]);
				
				// DEBUGGING
				// std::cout<<axisLengths[i]<<std::endl;
				}
			
			// DEBUGGING
			#if 0
			if(Math::abs(centroid[0]-267.0)<5.0&&Math::abs(centroid[1]-221.0)<5.0)
				{
				std::cout<<axes[0]<<", "<<axisLengths[0]<<std::endl;
				std::cout<<axes[1]<<", "<<axisLengths[1]<<std::endl;
				std::cout<<axes[2]<<", "<<axisLengths[2]<<std::endl;
				}
			#endif
			
			/* Check if the blob fits the search parameters: */
			bool blobValid=axisLengths[0]>=drMin&&axisLengths[0]<=drMax&&axisLengths[1]>=drMin&&axisLengths[1]<=drMax&&axisLengths[2]<=df;
			if(bIt->isTracked()||blobValid)
				{
				/* Store the extracted disk: */
				Disk disk;
				disk.center=depthProjection.transform(centroid);
				disk.normal=axes[0]^axes[1];
				Scalar nLen=Geometry::mag(disk.normal);
				if(disk.normal[2]>Scalar(0))
					nLen=-nLen;
				disk.normal/=nLen;
				
				disk.numPixels=bIt->numPixels;
				disk.radius=Math::sqrt(axisLengths[0]*axisLengths[1]);
				disk.flatness=axisLengths[2];
				disk.numPixels=bIt->numPixels;
				
				if(blobValid)
					extractionResult.push_back(disk);
				if(bIt->isTracked()&&tc!=0)
					{
					/* Call the tracking callback: */
					(*tc)(disk);
					}
				}
			}
	
	return extractionResult;
	}

void DiskExtractor::startStreaming(DiskExtractor::ExtractionResultCallback* newExtractionResultCallback)
	{
	/* Bail out if already streaming: */
	if(!diskExtractorThread.isJoined())
		{
		delete newExtractionResultCallback;
		throw std::runtime_error("DiskExtractor::startStreaming: Streaming already in progress");
		}
	
	/* Remember the result callback: */
	extractionResultCallback=newExtractionResultCallback;
	
	/* Start the disk extraction thread: */
	keepProcessing=true;
	diskExtractorThread.start(this,&DiskExtractor::diskExtractorThreadMethod);
	}

void DiskExtractor::stopStreaming(void)
	{
	/* Bail out if not streaming: */
	if(diskExtractorThread.isJoined())
		return;
	
	/* Shut down the disk extraction thread: */
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	
	keepProcessing=false;
	
	/* Wake up the disk extractor thread: */
	newFrameCond.signal();
	}
	
	/* Wait until the disk extraction thread terminates: */
	diskExtractorThread.join();
	
	delete extractionResultCallback;
	extractionResultCallback=0;
	}

void DiskExtractor::startTracking(DiskExtractor::TrackingCallback* newTrackingCallback)
	{
	/* Store the callback: */
	trackingCallback=newTrackingCallback;
	}

void DiskExtractor::setTrackingPixel(unsigned int trackingX,unsigned int trackingY)
	{
	/* Update the tracking pixel index: */
	trackingPixel=trackingY*frameSize[0]+trackingX;
	}

void DiskExtractor::stopTracking(void)
	{
	/* Reset the tracking pixel index: */
	trackingPixel=~0x0U;
	
	/* Delete the tracking callback: */
	TrackingCallback* tc=trackingCallback;
	trackingCallback=0;
	delete tc;
	}

}
