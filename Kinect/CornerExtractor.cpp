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

#include <Kinect/CornerExtractor.h>

#include <Misc/FunctionCalls.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Geometry/ValuedPoint.h>
#include <Geometry/ArrayKdTree.h>

namespace Kinect {

/*******************************************
Methods of class CornerExtractor::RingPixel:
*******************************************/

void CornerExtractor::RingPixel::init(int x,int y,int stride)
	{
	/* Calculate the pixel's 2D frame buffer offset: */
	offset=y*stride+x;
	
	/* Store the offset vector: */
	d[0]=Scalar(x);
	d[1]=Scalar(y);
	
	/* Calculate the pixel's angle around the circle, starting on the right at 0 and ending at 2*pi: */
	angle=Math::atan2(d[1],d[0]);
	if(angle<Scalar(0))
		angle+=Scalar(2)*Math::Constants<Scalar>::pi;
	}

namespace {

/**************
Helper classes:
**************/

struct CornerCandidate // Structure to represent a pixel that is a candidate for a grid corner
	{
	/* Elements: */
	public:
	CornerCandidate* root; // Pointer to root corner of merged subset
	CornerExtractor::Scalar x,y; // Accumulated corner position
	Kinect::CornerExtractor::Vector bw,wb; // Accumulated separation line directions from black to white and white to black, respectively
	CornerExtractor::Scalar weight; // Accumulation weight of this corner
	};

typedef Geometry::ValuedPoint<CornerExtractor::Point,CornerCandidate*> CornerCandidatePoint;
typedef Geometry::ArrayKdTree<CornerCandidatePoint> CornerCandidateTree;

struct CornerCandidateMerger
	{
	/* Embedded classes: */
	public:
	typedef CornerExtractor::Scalar Scalar;
	
	/* Elements: */
	public:
	CornerCandidatePoint queryPoint;
	Scalar maxMergeDist;
	
	/* Methods: */
	const CornerCandidateTree::Point& getQueryPosition(void) const
		{
		return queryPoint;
		}
	bool operator()(const CornerCandidateTree::StoredPoint& node,int splitDimension)
		{
		if(Geometry::sqrDist(queryPoint,node)<Math::sqr(maxMergeDist))
			{
			/* Find the roots of both points: */
			CornerCandidate* root0=queryPoint.value;
			while(root0->root!=root0)
				root0=root0->root;
			CornerCandidate* root1=node.value;
			while(root1->root!=root1)
				root1=root1->root;
			
			if(root0!=root1)
				{
				/* Merge the two points: */
				root0->x+=root1->x;
				root0->y+=root1->y;
				if(root0->bw*root1->bw>=Scalar(0))
					root0->bw+=root1->bw;
				else
					root0->bw-=root1->bw;
				if(root0->wb*root1->wb>=Scalar(0))
					root0->wb+=root1->wb;
				else
					root0->wb-=root1->wb;
				root0->weight+=root1->weight;
				
				/* Shorten the root links: */
				root1->root=root0;
				queryPoint.value->root=root0;
				node.value->root=root0;
				}
			}
		
		return Math::abs(node[splitDimension]-queryPoint[splitDimension])<maxMergeDist;
		}
	};

}

/********************************
Methods of class CornerExtractor:
********************************/

void CornerExtractor::normalizeFrame(const FrameBuffer& frame)
	{
	/* Convert the incoming color image into a pair of integral images for normalization: */
	const ColorPixel* cPtr=frame.getData<ColorPixel>();
	int stride=int(frameSize[0]+1U);
	unsigned int* intImgPtr=integralImage+stride; // Skip row -1
	unsigned long* intImg2Ptr=integral2Image+stride; // Skip row -1
	unsigned char* imgPtr=normalizedImage;
	if(ugc)
		{
		/* Convert RGB pixels to greyscale using only the green channel: */
		for(unsigned int y=0;y<frameSize[1];++y)
			{
			++intImgPtr; // Skip column -1
			++intImg2Ptr; // Skip column -1
			for(unsigned int x=0;x<frameSize[0];++x,++cPtr,++intImgPtr,++intImg2Ptr,++imgPtr)
				{
				/* Convert the current color image pixel to greyscale: */
				unsigned int grey=(unsigned int)(gammaCorrection[(*cPtr)[1]]);
				
				/* Integrate the greyscale pixel: */
				intImgPtr[0]=grey+intImgPtr[-1]+intImgPtr[-stride]-intImgPtr[-stride-1];
				intImg2Ptr[0]=grey*grey+intImg2Ptr[-1]+intImg2Ptr[-stride]-intImg2Ptr[-stride-1];
				
				/* Store the unnormalized greyscale value: */
				*imgPtr=(unsigned char)(grey);
				}
			}
		}
	else
		{
		/* Convert RGB pixels to greyscale using all channels: */
		for(unsigned int y=0;y<frameSize[1];++y)
			{
			++intImgPtr; // Skip column -1
			++intImg2Ptr; // Skip column -1
			for(unsigned int x=0;x<frameSize[0];++x,++cPtr,++intImgPtr,++intImg2Ptr,++imgPtr)
				{
				/* Convert the current color image pixel to greyscale: */
				unsigned int grey=((unsigned int)gammaCorrection[(*cPtr)[0]]*306U+(unsigned int)gammaCorrection[(*cPtr)[1]]*601U+(unsigned int)gammaCorrection[(*cPtr)[2]]*117U+512U)>>10;
				
				/* Integrate the greyscale pixel: */
				intImgPtr[0]=grey+intImgPtr[-1]+intImgPtr[-stride]-intImgPtr[-stride-1];
				intImg2Ptr[0]=grey*grey+intImg2Ptr[-1]+intImg2Ptr[-stride]-intImg2Ptr[-stride-1];
				
				/* Store the unnormalized greyscale value: */
				*imgPtr=(unsigned char)(grey);
				}
			}
		}
	
	/* Shift the average of a sliding window to 128 grey: */
	imgPtr=normalizedImage;
	intImgPtr=integralImage+stride+1;
	intImg2Ptr=integral2Image+stride+1;
	for(unsigned int y=0;y<frameSize[1];++y)
		{
		int y0=Math::max(int(y)-int(nws),0)-1;
		int y1=Math::min(int(y)+int(nws),int(frameSize[1]-1));
		for(unsigned int x=0;x<frameSize[0];++x,++imgPtr)
			{
			/* Extract statistics from the integral images: */
			int x0=Math::max(int(x)-int(nws),0)-1;
			int x1=Math::min(int(x)+int(nws),int(frameSize[0]-1));
			double sum=intImgPtr[y1*stride+x1]+intImgPtr[y0*stride+x0]-intImgPtr[y0*stride+x1]-intImgPtr[y1*stride+x0];
			double sum2=intImg2Ptr[y1*stride+x1]+intImg2Ptr[y0*stride+x0]-intImg2Ptr[y0*stride+x1]-intImg2Ptr[y1*stride+x0];
			
			/* Calculate the sliding window's average and standard deviation: */
			double denominator=(y1-y0)*(x1-x0);
			double avg=sum; // /denominator;
			double stddev=Math::sqrt(sum2*denominator-Math::sqr(sum)); // /denominator;
			
			/* Normalize the image pixel: */
			if(stddev>0.0)
				*imgPtr=(unsigned char)(Math::clamp(int(Math::floor((double(*imgPtr)*denominator-avg)*128.0/stddev+128.0+0.5)),0,255));
			else
				*imgPtr=(unsigned char)(128U);
			}
		}
	}

void CornerExtractor::calculateGrid(void)
	{
	/* Calculate all horizontal grid crossings: */
	unsigned int y=gridBaseY;
	unsigned char* gPtr=gridX;
	while(y<frameSize[1])
		{
		const unsigned char* imgPtr=normalizedImage+(y*frameSize[0]);
		
		/* Find the first pixel in the grid line that is outside the center grey range: */
		unsigned int x;
		for(x=0;x<frameSize[0]&&*imgPtr>=greyMin&&*imgPtr<=greyMax;++x,++imgPtr,++gPtr)
			*gPtr=(unsigned char)(128U);
		
		/* Check if the grid line ever left the grey area: */
		if(x<frameSize[0])
			{
			/* Start the first region: */
			unsigned char currentRegion=*imgPtr>=128U?255U:0U;
			unsigned int lastRegionX=x;
			*gPtr=(unsigned char)(128U);
			
			/* Process the rest of the grid line: */
			for(++x,++imgPtr,++gPtr;x<frameSize[0];++x,++imgPtr,++gPtr)
				{
				*gPtr=(unsigned char)(128U);
				
				/* Get the current pixel's region: */
				unsigned char pixelRegion=*imgPtr<greyMin?(unsigned char)(0U):(*imgPtr>greyMax?(unsigned char)(255U):(unsigned char)(128U));
				
				/* Check for region change: */
				if(pixelRegion==currentRegion)
					lastRegionX=x;
				else if(pixelRegion!=(unsigned char)(128U))
					{
					/* Mark a region change halfway between the current pixel and the last region pixel: */
					*(gPtr-(x-lastRegionX)/2)=pixelRegion;
					
					/* Change the current region: */
					currentRegion=pixelRegion;
					lastRegionX=x;
					}
				}
			}
		
		/* Go to the next horizontal grid line: */
		y+=gridCellSize;
		}
	
	/* Calculate all vertical grid crossings: */
	unsigned int x=gridBaseX;
	gPtr=gridY;
	while(x<frameSize[0])
		{
		const unsigned char* imgPtr=normalizedImage+x;
		
		/* Find the first pixel in the grid line that is outside the center grey range: */
		unsigned int y;
		for(y=0;y<frameSize[1]&&*imgPtr>=greyMin&&*imgPtr<=greyMax;++y,imgPtr+=frameSize[0],++gPtr)
			*gPtr=(unsigned char)(128U);
		
		/* Check if the grid line ever left the grey area: */
		if(y<frameSize[1])
			{
			/* Start the first region: */
			unsigned char currentRegion=*imgPtr>=128U?255U:0U;
			unsigned int lastRegionY=y;
			*gPtr=(unsigned char)(128U);
			
			/* Process the rest of the grid line: */
			for(++y,imgPtr+=frameSize[0],++gPtr;y<frameSize[1];++y,imgPtr+=frameSize[0],++gPtr)
				{
				*gPtr=(unsigned char)(128U);
				
				/* Get the current pixel's region: */
				unsigned char pixelRegion=*imgPtr<greyMin?(unsigned char)(0U):(*imgPtr>greyMax?(unsigned char)(255U):(unsigned char)(128U));
				
				/* Check for region change: */
				if(pixelRegion==currentRegion)
					lastRegionY=y;
				else if(pixelRegion!=(unsigned char)(128U))
					{
					/* Mark a region change halfway between the current pixel and the last region pixel: */
					*(gPtr-(y-lastRegionY)/2)=pixelRegion;
					
					/* Change the current region: */
					currentRegion=pixelRegion;
					lastRegionY=y;
					}
				}
			}
		
		/* Go to the next vertical grid line: */
		x+=gridCellSize;
		}
	}

bool CornerExtractor::checkPixel(const unsigned char* pixel,CornerExtractor::Vector& bwSeparator,CornerExtractor::Vector& wbSeparator) const
	{
	/* Test a sequence of rings around the given pixel: */
	Vector bws=Vector::zero;
	Vector wbs=Vector::zero;
	Scalar xxs(0);
	Scalar xys(0);
	Scalar xs(0);
	Scalar ys(0);
	unsigned int numCornerRings=0;
	for(unsigned int ring=0;ring<nr&&ring-numCornerRings<=mncr;++ring)
		{
		/* Find the first pixel on the ring that is outside the center grey range: */
		unsigned int ringLength=ringLengths[ring];
		const RingPixel* rpPtr=rings[ring];
		unsigned int ringPos;
		for(ringPos=0;ringPos<ringLength&&pixel[rpPtr->offset]>=greyMin&&pixel[rpPtr->offset]<=greyMax;++ringPos,++rpPtr)
			;
		
		/* Check if the ring ever left the grey area: */
		if(ringPos<ringLength)
			{
			/* Go around the ring completely and count region crossings: */
			int currentRegion=pixel[rpPtr->offset]>=128U?1:-1;
			const RingPixel* lastRegionPixel=rpPtr;
			++rpPtr;
			unsigned int numRegionChanges=0;
			
			/* Keep track of region changes: */
			int regionChangeDirections[4];
			Vector regionChangeVectors[4];
			Scalar regionChangeAngles[4];
			for(ringPos=0;ringPos<ringLength;++ringPos,++rpPtr)
				{
				/* Get the current pixel's region: */
				int pixelRegion=pixel[rpPtr->offset]<greyMin?-1:(pixel[rpPtr->offset]>greyMax?1:0);
				
				/* Check for region change: */
				if(pixelRegion==currentRegion)
					lastRegionPixel=rpPtr;
				else if(pixelRegion!=0)
					{
					/* Bail out if there were already four region changes: */
					if(numRegionChanges==4)
						{
						++numRegionChanges;
						break;
						}
					
					/* Calculate the region change direction and angle: */
					regionChangeDirections[numRegionChanges]=pixelRegion;
					regionChangeVectors[numRegionChanges]=lastRegionPixel->d+rpPtr->d;
					regionChangeAngles[numRegionChanges]=Math::mid(lastRegionPixel->angle,rpPtr->angle);
					
					/* Change the current region: */
					++numRegionChanges;
					currentRegion=pixelRegion;
					lastRegionPixel=rpPtr;
					}
				}
			
			/* Start deciding whether this ring was compatible with a corner pixel: */
			bool isCornerRing=numRegionChanges==4;
			
			/* Check if the ring's black-to-white ratio matches the set limits: */
			Scalar region0,region2;
			Scalar whiteRatio;
			if(isCornerRing)
				{
				/* Calculate this ring's black-to-white ratio: */
				region0=regionChangeAngles[1]-regionChangeAngles[0];
				region2=regionChangeAngles[3]-regionChangeAngles[2];
				whiteRatio=(region0+region2)/twoPi;
				if(regionChangeDirections[0]==-1) // First change was actually white to black
					whiteRatio=Scalar(1)-whiteRatio;
				
				if(ring>=numRings/2&&Math::abs(whiteRatio-Scalar(0.5))>=mbwi) // Don't apply this criterion to the smaller rings
					isCornerRing=false;
				}
			
			/* Check if the ring's symmetry matches the set limits: */
			if(isCornerRing&&ring>=numRings/2) // Don't apply this criterion to the smaller rings
				{
				/* Calculate the ring's symmetry: */
				Scalar region1=regionChangeAngles[2]-regionChangeAngles[1];
				Scalar region3=regionChangeAngles[0]+twoPi-regionChangeAngles[3];
				
				if(Math::abs(region2-region0)>=ma||Math::abs(region3-region1)>=ma)
					isCornerRing=false;
				}
			
			if(isCornerRing)
				{
				/* Calculate the average black-to-white and white-to-black separator directions: */
				Vector bw,wb;
				if(regionChangeDirections[0]==1) // First region change was black-to-white
					{
					bw=regionChangeVectors[0]-regionChangeVectors[2];
					wb=regionChangeVectors[1]-regionChangeVectors[3];
					}
				else // First region change was white-to-black
					{
					wb=regionChangeVectors[0]-regionChangeVectors[2];
					bw=regionChangeVectors[1]-regionChangeVectors[3];
					}
				
				/* Accumulate the separator directions: */
				if(bws*bw>=Scalar(0))
					bws+=bw;
				else
					bws-=bw;
				if(wbs*wb>=Scalar(0))
					wbs+=wb;
				else
					wbs-=wb;
				
				/* Update the black-to-white ratio regression states: */
				xxs+=Scalar(ring)*Scalar(ring);
				xys+=Scalar(ring)*whiteRatio;
				xs+=Scalar(ring);
				ys+=whiteRatio;
				
				/* Mark this as a corner ring: */
				++numCornerRings;
				}
			}
		}
	
	/* Check if the pixel had enough corner rings: */
	bool result=nr-numCornerRings<=mncr;
	if(result)
		{
		/* Check if the black-to-white ratio stayed constant: */
		Scalar slope=(xys*Scalar(numCornerRings)-xs*ys)/(xxs*Scalar(numCornerRings)-xs*xs);
		if(Math::abs(slope)>=mbwrs)
			result=false;
		}
	
	if(result)
		{
		/* Normalize and store the separator directions: */
		bwSeparator=bws.normalize();
		wbSeparator=wbs.normalize();
		}
	
	return result;
	}

void CornerExtractor::extractCorners(const FrameBuffer& frame,CornerExtractor::CornerList& corners)
	{
	/* Normalize the given frame: */
	normalizeFrame(frame);
	
	/* Extract a list of corner candidate pixels by running the corner classifier on each pixel: */
	std::vector<CornerCandidatePoint> cornerCandidates;
	unsigned int border=ringRadii[nr-1]; // Radius of largest ring; mustn't process pixels closer to the edge than this
	for(unsigned int y=border;y<frameSize[1]-border;++y)
		{
		const unsigned char* imgPtr=normalizedImage+y*frameSize[0]+border;
		for(unsigned int x=border;x<frameSize[0]-border;++x,++imgPtr)
			{
			/* Run the corner classifier and check if the pixel is a corner candidate: */
			Vector bw,wb;
			if(checkPixel(imgPtr,bw,wb))
				{
				/* Create a new corner candidate structure: */
				CornerCandidate* newCornerCandidate=new CornerCandidate;
				newCornerCandidate->root=newCornerCandidate;
				newCornerCandidate->x=Scalar(x)+Scalar(0.5);
				newCornerCandidate->y=Scalar(y)+Scalar(0.5);
				newCornerCandidate->bw=bw;
				newCornerCandidate->wb=wb;
				newCornerCandidate->weight=Scalar(1);
				
				/* Add it to the list: */
				cornerCandidates.push_back(CornerCandidatePoint(CornerCandidatePoint::Point(newCornerCandidate->x,newCornerCandidate->y),newCornerCandidate));
				}
			}
		}
	
	/* Erect a kd-tree on top of the corner point vector: */
	Geometry::ArrayKdTree<CornerCandidatePoint> cornerCandidateTree;
	cornerCandidateTree.donatePoints(cornerCandidates.size(),&cornerCandidates.front());
	
	/* Merge all corner points closer than the maximum ring radius: */
	CornerCandidateMerger cm;
	cm.maxMergeDist=Scalar(ringRadii[nr-1]);
	for(std::vector<CornerCandidatePoint>::iterator cIt=cornerCandidates.begin();cIt!=cornerCandidates.end();++cIt)
		{
		/* Merge with all near points: */
		cm.queryPoint=*cIt;
		cornerCandidateTree.traverseTreeDirected(cm);
		}
	
	/* Clear the corner candidate tree: */
	cornerCandidateTree.detachPoints();
	
	/* Extract a list of corner candidate cluster roots: */
	corners.clear();
	for(std::vector<CornerCandidatePoint>::iterator cIt=cornerCandidates.begin();cIt!=cornerCandidates.end();++cIt)
		{
		CornerCandidate* c=cIt->value;
		
		/* Check if the corner candidate is the root of a cluster: */
		if(c->root==c)
			{
			/* Store the corner candidate cluster as a corner: */
			corners.push_back(Corner());
			Corner& newCorner=corners.back();
			newCorner[0]=c->x/c->weight;
			newCorner[1]=c->y/c->weight;
			newCorner.bw=Geometry::normalize(c->bw);
			newCorner.wb=Geometry::normalize(c->wb);
			
			/* Orient the white-to-black separation direction such that it forms a right-handed frame with the black-to-white separation direction: */
			if(newCorner.bw[0]*newCorner.wb[1]-newCorner.bw[1]*newCorner.wb[0]<Scalar(0))
				newCorner.wb=-newCorner.wb;
			}
		
		/* Delete the corner candidate: */
		delete c;
		}
	}

void* CornerExtractor::cornerExtractorThreadMethod(void)
	{
	while(true)
		{
		FrameBuffer frame;
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
		
		/* Grab current corner extraction parameters: */
		ugc=useGreenChannel;
		nws=normalizationWindowSize;
		nr=numRings;
		mncr=maxNonCornerRings;
		greyMin=(unsigned char)(128U-regionThreshold);
		greyMax=(unsigned char)(128U+regionThreshold);
		mbwi=maxBlackWhiteImbalance;
		ma=maxAsymmetry;
		mbwrs=maxBlackWhiteRatioSlope;
		}
		
		/* Run the corner extraction algorithm on the new frame: */
		CornerList extractionResult;
		extractCorners(frame,extractionResult);
		
		if(extractionResultCallback!=0)
			{
			/* Call the result callback: */
			(*extractionResultCallback)(extractionResult);
			}
		}
	
	return 0;
	}

CornerExtractor::CornerExtractor(const unsigned int sFrameSize[2],unsigned int sMaxNumRings,int sMinRingRadius)
	:twoPi(Scalar(2)*Math::Constants<Scalar>::pi),
	 useGreenChannel(false),
	 gammaCorrection(new unsigned char[256]),
	 maxNumRings(sMaxNumRings),minRingRadius(sMinRingRadius),
	 ringRadii(0),ringLengths(0),rings(0),
	 integralImage(0),integral2Image(0),normalizedImage(0),
	 gridX(0),gridY(0),
	 normalizationWindowSize(48),regionThreshold(80U),
	 numRings(maxNumRings),maxNonCornerRings(0),
	 maxBlackWhiteImbalance(Scalar(0.4)),maxAsymmetry(Math::rad(Scalar(20))),
	 maxBlackWhiteRatioSlope(Scalar(0.0125)),
	 keepProcessing(false),
	 extractionResultCallback(0)
	{
	/* Copy the frame size: */
	for(int i=0;i<2;++i)
		frameSize[i]=sFrameSize[i];
	
	/* Initialize gamma correction table: */
	setInputGamma(1.0f);
	
	/*********************************************************************
	Pre-compute pixel ring pointer offsets:
	*********************************************************************/
	
	/* Calculate the arc lengths of all rings to get total pointer offset array size: */
	ringRadii=new int[maxNumRings];
	ringLengths=new unsigned int[maxNumRings];
	unsigned int ringLengthSum=0;
	for(unsigned int ring=0;ring<maxNumRings;++ring)
		{
		/* Store the ring's radius: */
		ringRadii[ring]=minRingRadius+int(ring);
		
		/* Step through Bresenham's circle algorithm to calculate the length of an octant arc: */
		int numDs=0;
		int x=ringRadii[ring];
		int y=0;
		int decision=1-x;
		bool hitEnd=false;
		while(y<=x)
			{
			/* Count the current pixel: */
			++numDs;
			
			/* Check if the current pixel lies exactly on the arc's endpoint: */
			hitEnd=x==y;
			
			/* Step along the arc: */
			++y;
			if(decision<=0)
				decision+=2*y+1;
			else
				{
				--x;
				decision+=2*(y-x)+1;
				}
			}
		
		/* Calculate the total arc length by replicating the octant arc eight times, skipping shared pixels: */
		ringLengths[ring]=numDs*8-4;
		if(hitEnd)
			ringLengths[ring]-=4;
		
		ringLengthSum+=ringLengths[ring];
		}
	
	/* Allocate the complete ring offset array: */
	rings=new RingPixel*[maxNumRings];
	rings[0]=new RingPixel[ringLengthSum*2]; // Store two full circles per ring
	for(unsigned int ring=1;ring<maxNumRings;++ring)
		rings[ring]=rings[ring-1]+ringLengths[ring-1]*2;
	
	/* Calculate all ring pixel offsets: */
	int* dxs=new int[(ringLengths[maxNumRings-1]+7)/8+1];
	int* dys=new int[(ringLengths[maxNumRings-1]+7)/8+1];
	int stride=int(frameSize[0]);
	for(unsigned int ring=0;ring<maxNumRings;++ring)
		{
		int radius=ringRadii[ring];
		
		/* Calculate (x, y) offsets for one octant of the circle using Bresenham's circle algorithm: */
		int numDs=0;
		int x=radius;
		int y=0;
		int decision=1-x;
		bool hitEnd=false;
		while(y<=x)
			{
			/* Store the current pixel: */
			dxs[numDs]=x;
			dys[numDs]=y;
			++numDs;
			
			/* Check if the pixel exactly hit the arc endpoint: */
			hitEnd=x==y;
			
			/* Step along the arc: */
			++y;
			if(decision<=0)
				decision+=2*y+1;
			else
				{
				--x;
				decision+=2*(y-x)+1;
				}
			}
		
		/* Replicate the octant eight times, using each pixel exactly once: */
		RingPixel* rpPtr=rings[ring];
		
		/* First quadrant: x>0, y>0: */
		for(int d=0;d<numDs;++d,++rpPtr)
			rpPtr->init(dxs[d],dys[d],stride);
		for(int d=hitEnd?numDs-2:numDs-1;d>0;--d,++rpPtr)
			rpPtr->init(dys[d],dxs[d],stride);
		
		/* Second quadrant: x<0, y>0: */
		for(int d=0;d<numDs;++d,++rpPtr)
			rpPtr->init(-dys[d],dxs[d],stride);
		for(int d=hitEnd?numDs-2:numDs-1;d>0;--d,++rpPtr)
			rpPtr->init(-dxs[d],dys[d],stride);
		
		/* Third quadrant: x<0, y<0: */
		for(int d=0;d<numDs;++d,++rpPtr)
			rpPtr->init(-dxs[d],-dys[d],stride);
		for(int d=hitEnd?numDs-2:numDs-1;d>0;--d,++rpPtr)
			rpPtr->init(-dys[d],-dxs[d],stride);
		
		/* Fourth quadrant: x>0, y>0: */
		for(int d=0;d<numDs;++d,++rpPtr)
			rpPtr->init(dys[d],-dxs[d],stride);
		for(int d=hitEnd?numDs-2:numDs-1;d>0;--d,++rpPtr)
			rpPtr->init(dxs[d],-dys[d],stride);
		
		/* Copy the entire ring for a second circle, with increased angles, to avoid edge conditions: */
		RingPixel* rpPtr2=rings[ring];
		for(unsigned int i=0;i<ringLengths[ring];++i,++rpPtr,++rpPtr2)
			{
			rpPtr->offset=rpPtr2->offset;
			rpPtr->d=rpPtr2->d;
			rpPtr->angle=rpPtr2->angle+twoPi;
			}
		}
	delete[] dxs;
	delete[] dys;
	
	/* Allocate the integral and the normalized images: */
	integralImage=new unsigned int[(frameSize[1]+1)*(frameSize[0]+1)];
	integral2Image=new unsigned long[(frameSize[1]+1)*(frameSize[0]+1)];
	normalizedImage=new unsigned char[frameSize[1]*frameSize[0]];
	
	/* Initialize the -1 row and -1 column of the integral images to zero: */
	unsigned int* intImgPtr=integralImage;
	unsigned long* intImg2Ptr=integral2Image;
	stride=int(frameSize[0]+1U);
	*(intImgPtr++)=0;
	*(intImg2Ptr++)=0;
	for(unsigned int x=0;x<frameSize[0];++x,++intImgPtr,++intImg2Ptr)
		{
		*intImgPtr=0;
		*intImg2Ptr=0;
		}
	for(unsigned int y=0;y<frameSize[1];++y,intImgPtr+=stride,intImg2Ptr+=stride)
		{
		*intImgPtr=0;
		*intImg2Ptr=0;
		}
	
	/* Allocate the horizontal and vertical grid lines: */
	gridCellSize=ringRadii[maxNumRings-1];
	gridSize[0]=(frameSize[0]+gridCellSize-1)/gridCellSize;
	gridSize[1]=(frameSize[1]+gridCellSize-1)/gridCellSize;
	gridBaseX=((frameSize[0]-1)%gridCellSize+1)/2;
	gridBaseY=((frameSize[1]-1)%gridCellSize+1)/2;
	gridX=new unsigned char[gridSize[1]*frameSize[0]];
	gridY=new unsigned char[gridSize[0]*frameSize[1]];
	}

CornerExtractor::~CornerExtractor(void)
	{
	/* Stop streaming if still active: */
	if(!cornerExtractorThread.isJoined())
		{
		/* Shut down the corner extraction thread: */
		{
		Threads::MutexCond::Lock newFrameLock(newFrameCond);
		
		keepProcessing=false;
		
		/* Wake up the corner extractor thread: */
		newFrameCond.signal();
		}
		
		/* Wait until the disk extraction thread terminates: */
		cornerExtractorThread.join();
		}
	
	delete[] gammaCorrection;
	delete[] ringRadii;
	delete[] ringLengths;
	delete[] rings[0];
	delete[] rings;
	delete[] integralImage;
	delete[] integral2Image;
	delete[] normalizedImage;
	delete[] gridX;
	delete[] gridY;
	delete extractionResultCallback;
	}

void CornerExtractor::setUseGreenChannel(bool newUseGreenChannel)
	{
	useGreenChannel=newUseGreenChannel;
	}

void CornerExtractor::setInputGamma(float newInputGamma)
	{
	/* Calculate the gamma correction table: */
	for(unsigned int i=0;i<256;++i)
		gammaCorrection[i]=(unsigned char)(Math::floor(Math::pow(float(i)/255.0f,newInputGamma)*255.0f+0.5f));
	}

void CornerExtractor::setNormalizationWindowSize(unsigned int newNormalizationWindowSize)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	normalizationWindowSize=newNormalizationWindowSize;
	}

void CornerExtractor::setRegionThreshold(unsigned int newRegionThreshold)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	regionThreshold=newRegionThreshold;
	}

void CornerExtractor::setNumRings(unsigned int newNumRings)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	if(newNumRings<1U)
		numRings=1U;
	else if(newNumRings>maxNumRings)
		numRings=maxNumRings;
	else
		numRings=newNumRings;
	}

void CornerExtractor::setCornerTestRadius(int newCornerTestRadius)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	if(newCornerTestRadius<minRingRadius)
		numRings=1U;
	else if(newCornerTestRadius>minRingRadius+int(maxNumRings))
		numRings=maxNumRings;
	else
		numRings=(unsigned int)(newCornerTestRadius-minRingRadius)+1U;
	}

void CornerExtractor::setMaxNonCornerRings(unsigned int newMaxNonCornerRings)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	maxNonCornerRings=newMaxNonCornerRings;
	}

void CornerExtractor::setMaxBlackWhiteImbalance(CornerExtractor::Scalar newMaxBlackWhiteImbalance)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	maxBlackWhiteImbalance=newMaxBlackWhiteImbalance;
	}

void CornerExtractor::setMaxAsymmetry(CornerExtractor::Scalar newMaxAsymmetry)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	maxAsymmetry=newMaxAsymmetry;
	}

void CornerExtractor::setMaxBlackWhiteRatioSlope(CornerExtractor::Scalar newMaxBlackWhiteRatioSlope)
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	maxBlackWhiteRatioSlope=newMaxBlackWhiteRatioSlope;
	}

CornerExtractor::CornerList CornerExtractor::processFrame(const FrameBuffer& frame)
	{
	/* Grab current corner extraction parameters: */
	ugc=useGreenChannel;
	nws=normalizationWindowSize;
	nr=numRings;
	mncr=maxNonCornerRings;
	greyMin=(unsigned char)(128U-regionThreshold);
	greyMax=(unsigned char)(128U+regionThreshold);
	mbwi=maxBlackWhiteImbalance;
	ma=maxAsymmetry;
	mbwrs=maxBlackWhiteRatioSlope;
	
	/* Run the corner extraction algorithm on the given frame: */
	CornerList extractionResult;
	extractCorners(frame,extractionResult);
	
	/* Return the list of extracted corners: */
	return extractionResult;
	}

void CornerExtractor::startStreaming(CornerExtractor::ExtractionResultCallback* newExtractionResultCallback)
	{
	/* Bail out if already streaming: */
	if(!cornerExtractorThread.isJoined())
		{
		delete newExtractionResultCallback;
		throw std::runtime_error("CornerExtractor::startStreaming: Streaming already in progress");
		}
	
	/* Remember the result callback: */
	extractionResultCallback=newExtractionResultCallback;
	
	/* Start the disk extraction thread: */
	keepProcessing=true;
	cornerExtractorThread.start(this,&CornerExtractor::cornerExtractorThreadMethod);
	}

void CornerExtractor::stopStreaming(void)
	{
	/* Bail out if not streaming: */
	if(cornerExtractorThread.isJoined())
		return;
	
	/* Shut down the corner extraction thread: */
	{
	Threads::MutexCond::Lock newFrameLock(newFrameCond);
	
	keepProcessing=false;
	
	/* Wake up the corner extractor thread: */
	newFrameCond.signal();
	}
	
	/* Wait until the corner extraction thread terminates: */
	cornerExtractorThread.join();
	
	delete extractionResultCallback;
	extractionResultCallback=0;
	}

}
