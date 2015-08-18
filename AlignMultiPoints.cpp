#include <stddef.h>
#include <vector>
#include <iostream>
#include <IO/OpenFile.h>
#include <IO/ValueSource.h>
#include <Math/Math.h>
#include <Math/Constants.h>
#include <Math/Matrix.h>
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Vector.h>
#include <Geometry/OrthogonalTransformation.h>
#include <Geometry/ValuedPoint.h>
#include <Geometry/Random.h>
#include <Geometry/GeometryValueCoders.h>
#include <Geometry/OutputOperators.h>
#include <GL/gl.h>
#include <GL/GLColorTemplates.h>
#include <GL/GLColor.h>
#include <GL/GLGeometryWrappers.h>
#include <Vrui/Application.h>

namespace Geometry {

template <class FitterFunctor>
class LevenbergMarquardtMinimizer
	{
	/* Embedded classes: */
	public:
	typedef FitterFunctor Fitter; // Functor class for the fitting geometry
	
	/* Elements: */
	
	/* Minimization parameters (public because there are no invariants): */
	public:
	double tau;
	double epsilon1;
	double epsilon2;
	size_t maxNumIterations;
	
	/* Constructors and destructors: */
	public:
	LevenbergMarquardtMinimizer(void) // Sets default minimization parameters
		:tau(1.0e-3),
		 epsilon1(1.0e-20),
		 epsilon2(1.0e-20),
		 maxNumIterations(1000)
		{
		}
	LevenbergMarquardtMinimizer(double sTau,double sEpsilon1,double sEpsilon2,size_t sMaxNumIterations) // Sets given minimization parameters
		:tau(sTau),
		 epsilon1(sEpsilon1),
		 epsilon2(sEpsilon2),
		 maxNumIterations(sMaxNumIterations)
		{
		}
	
	/* Methods: */
	double minimize(Fitter& fitter) const; // Minimizes the target function by manipulating the given fitter
	};

/********************************************
Methods of class LevenbergMarquardtMinimizer:
********************************************/

template <class FitterParam>
inline
double
LevenbergMarquardtMinimizer<FitterParam>::minimize(
	typename LevenbergMarquardtMinimizer<FitterParam>::Fitter& fitter) const
	{
	/* Get the optimization space's dimension: */
	int dimension=fitter.getDimension();
	
	/* Compute the Jacobian matrix, the error vector, and the initial target function value: */
	Math::Matrix A(dimension,dimension);
	Math::Matrix g(dimension,1);
	for(int i=0;i<dimension;++i)
		{
		for(int j=0;j<dimension;++j)
			A(i,j)=0.0;
		g(i)=0.0;
		}
	double F=0.0;
	Math::Matrix dp(dimension,1);
	for(size_t index=0;index<fitter.getNumPoints();++index)
		{
		fitter.calcDistanceDerivative(index,dp);
		double d=fitter.calcDistance(index);
		for(int i=0;i<dimension;++i)
			{
			for(int j=0;j<dimension;++j)
				A(i,j)+=dp(i)*dp(j);
			g(i)+=dp(i)*d;
			}
		F+=Math::sqr(d);
		}
	F*=0.5;
	
	/* Compute the initial damping factor: */
	double maxA=A(0,0);
	for(int i=1;i<dimension;++i)
		if(maxA<A(i,i))
			maxA=A(i,i);
	double mu=tau*maxA;
	double nu=double(2);
	
	/* Check for convergence: */
	bool found=true;
	for(int i=0;i<dimension;++i)
		if(Math::abs(g(i))>epsilon1)
			found=false;
	size_t iteration;
	for(iteration=0;!found&&iteration<maxNumIterations;++iteration)
		{
		/* Calculate step direction: */
		Math::Matrix H(A);
		H.makePrivate();
		for(int i=0;i<dimension;++i)
			H(i,i)+=mu;
		Math::Matrix h=g;
		h.divideFullPivot(H); // h is actually the negative of hlm in the pseudo-code
		
		/* Check for convergence: */
		if(h.mag()<=epsilon2*(fitter.calcMag()+epsilon2))
			{
			std::cout<<h.mag()<<"<="<<epsilon2<<"*("<<fitter.calcMag()<<"+"<<epsilon2<<")"<<std::endl;
			break;
			}
		
		/* Try updating the current state: */
		fitter.save();
		fitter.increment(h); // Subtracts h instead of adding (h is negative, see above)
		fitter.normalize();
		
		/* Calculate the new target function value: */
		double newF=0.0;
		for(size_t index=0;index<fitter.getNumPoints();++index)
			newF+=Math::sqr(fitter.calcDistance(index));
		newF*=0.5;
		std::cout<<iteration<<": F="<<F<<std::endl;
		
		/* Calculate the gain value: */
		double denom=0.0;
		for(int i=0;i<dimension;++i)
			denom+=h(i)*(mu*h(i)+g(i)); // Adds g instead of subtracting (h is negative, see above)
		denom*=0.5;
		double rho=(F-newF)/denom;
		
		/* Accept or deny the step: */
		if(rho>0.0)
			{
			/* Compute the new Jacobian matrix and the new error vector: */
			A.makePrivate();
			g.makePrivate();
			for(int i=0;i<dimension;++i)
				{
				for(int j=0;j<dimension;++j)
					A(i,j)=0.0;
				g(i)=0.0;
				}
			for(size_t index=0;index<fitter.getNumPoints();++index)
				{
				fitter.calcDistanceDerivative(index,dp);
				double d=fitter.calcDistance(index);
				for(int i=0;i<dimension;++i)
					{
					for(int j=0;j<dimension;++j)
						A(i,j)+=dp(i)*dp(j);
					g(i)+=dp(i)*d;
					}
				}
			
			/* Update the target function value: */
			F=newF;
			
			/* Check for convergence: */
			found=true;
			for(int i=0;i<dimension;++i)
				if(Math::abs(g(i))>epsilon1)
					found=false;
			if(found)
				std::cout<<"Converged"<<std::endl;
			
			/* Update the damping factor: */
			double rhof=2.0*rho-1.0;
			double factor=1.0-rhof*rhof*rhof;
			if(factor<1.0/3.0)
				factor=1.0/3.0;
			mu*=factor;
			nu=2.0;
			}
		else
			{
			/* Deny the step: */
			fitter.restore();
			
			/* Update the damping factor: */
			mu*=nu;
			nu*=2.0;
			}
		}
	
	std::cout<<iteration<<std::endl;
	
	/* Clean up: */
	return F;
	}

}

class OGTransformFitter
	{
	/* Embedded classes: */
	public:
	typedef double Scalar; // Scalar type
	typedef Geometry::Point<Scalar,3> Point;
	typedef Geometry::ValuedPoint<Point,bool> TiePoint;
	typedef Geometry::Vector<Scalar,3> Vector;
	typedef Geometry::OrthogonalTransformation<Scalar,3> Transform;
	
	/* Elements: */
	private:
	size_t numPointSets; // Number of point sets to align
	size_t numPoints; // Number of source and target points
	const TiePoint* const* ps; // Array of point sets containing valid or invalid points
	
	/* Transient optimization state: */
	Transform* transforms; // Array of current transformation estimates from "world space" to all point sets
	Transform* transformSaves;
	
	/* Constructors and destructors: */
	public:
	OGTransformFitter(size_t sNumPointSets,size_t sNumPoints,const TiePoint* const* sPs)
		:numPointSets(sNumPointSets),numPoints(sNumPoints),ps(sPs),
		 transforms(new Transform[numPointSets]),
		 transformSaves(new Transform[numPointSets])
		{
		for(size_t i=0;i<numPointSets;++i)
			transforms[i]=Transform::identity;
		}
	~OGTransformFitter(void)
		{
		delete[] transforms;
		delete[] transformSaves;
		}
	
	/* Methods: */
	int getDimension(void) const // Returns the dimension of the optimization space
		{
		return int(numPointSets)*8;
		}
	const Transform& getTransform(size_t index) const // Returns the current transformation estimate
		{
		return transforms[index];
		}
	void setTransform(size_t index,const Transform& newTransform) // Sets the current transformation estimate
		{
		transforms[index]=newTransform;
		};
	
	/* Methods required by Levenberg-Marquardt optimizer: */
	void save(void) // Saves the current estimate
		{
		for(size_t i=0;i<numPointSets-1;++i)
			transformSaves[i]=transforms[i];
		};
	void restore(void) // Restores the last saved estimate
		{
		for(size_t i=0;i<numPointSets-1;++i)
			transforms[i]=transformSaves[i];
		};
	size_t getNumPoints(void) const // Returns the number of distance functions to minimize
		{
		return numPoints*numPointSets*(numPointSets-1)/2; // Fully connected comparison between each point tuple
		}
	double calcDistance(size_t index) const // Calculates the distance value for the current estimate and the given distance function index
		{
		/* Break down comparison index into point index, and point set indices: */
		size_t pointIndex=index%numPoints;
		index/=numPoints;
		size_t ps1;
		for(ps1=0;index>=numPointSets-1-ps1;++ps1)
			index-=numPointSets-1-ps1;
		size_t ps2=ps1+1+index;
		
		/* Return zero distance if either of the points is invalid: */
		if(ps[ps1][pointIndex].value&&ps[ps2][pointIndex].value)
			return Geometry::sqrDist(transforms[ps1].transform(ps[ps1][pointIndex]),transforms[ps2].transform(ps[ps2][pointIndex]));
		else
			return 0.0;
		}
	void calcDistanceDerivative(size_t index,Math::Matrix& derivative) const // Calculates the derivative of the distance value for the current estimate and the given distance function index
		{
		/* Break down comparison index into point index, and point set indices: */
		size_t pointIndex=index%numPoints;
		index/=numPoints;
		size_t ps1;
		for(ps1=0;index>=numPointSets-1-ps1;++ps1)
			index-=numPointSets-1-ps1;
		size_t ps2=ps1+1+index;
		
		/* Zero out all partial derivatives: */
		for(int i=0;i<int(numPointSets)*8;++i)
			derivative(i)=0.0;
		
		/* Bail out if either of the points is invalid: */
		if(!(ps[ps1][pointIndex].value&&ps[ps2][pointIndex].value))
			return;
		
		/*******************************************************************
		Calculate the distance vector between the two transformed points.
		The transformations are spelled out in order to reuse the
		intermediate results for the derivative calculation.
		*******************************************************************/
		
		/* Get the first point and transformation: */
		const Point& p1=ps[ps1][pointIndex];
		const Vector& t1=transforms[ps1].getTranslation();
		const Scalar* q1=transforms[ps1].getRotation().getQuaternion();
		Scalar s1=transforms[ps1].getScaling();
		
		/* Calculate the first transformation's first rotation part: */
		Scalar rX1=q1[1]*p1[2]-q1[2]*p1[1]+q1[3]*p1[0];
		Scalar rY1=q1[2]*p1[0]-q1[0]*p1[2]+q1[3]*p1[1];
		Scalar rZ1=q1[0]*p1[1]-q1[1]*p1[0]+q1[3]*p1[2];
		Scalar rW1=q1[0]*p1[0]+q1[1]*p1[1]+q1[2]*p1[2];
		
		/* Calculate the first transformation's scaling, second rotation, and translation parts: */
		Point tp1;
		tp1[0]=(rZ1*q1[1]-rY1*q1[2]+rW1*q1[0]+rX1*q1[3])*s1+t1[0];
		tp1[1]=(rX1*q1[2]-rZ1*q1[0]+rW1*q1[1]+rY1*q1[3])*s1+t1[1];
		tp1[2]=(rY1*q1[0]-rX1*q1[1]+rW1*q1[2]+rZ1*q1[3])*s1+t1[2];
		
		/* Get the second point and transformation: */
		const Point& p2=ps[ps2][pointIndex];
		const Vector& t2=transforms[ps2].getTranslation();
		const Scalar* q2=transforms[ps2].getRotation().getQuaternion();
		Scalar s2=transforms[ps2].getScaling();
		
		/* Calculate the second transformation's first rotation part: */
		Scalar rX2=q2[1]*p2[2]-q2[2]*p2[1]+q2[3]*p2[0];
		Scalar rY2=q2[2]*p2[0]-q2[0]*p2[2]+q2[3]*p2[1];
		Scalar rZ2=q2[0]*p2[1]-q2[1]*p2[0]+q2[3]*p2[2];
		Scalar rW2=q2[0]*p2[0]+q2[1]*p2[1]+q2[2]*p2[2];
		
		/* Calculate the second transformation's scaling, second rotation, and translation parts: */
		Point tp2;
		tp2[0]=(rZ2*q2[1]-rY2*q2[2]+rW2*q2[0]+rX2*q2[3])*s2+t2[0];
		tp2[1]=(rX2*q2[2]-rZ2*q2[0]+rW2*q2[1]+rY2*q2[3])*s2+t2[1];
		tp2[2]=(rY2*q2[0]-rX2*q2[1]+rW2*q2[2]+rZ2*q2[3])*s2+t2[2];
		
		/* Calculate the distance vector and difference magnitude: */
		Vector d=tp2-tp1;
		Scalar dist=Scalar(2); // Geometry::mag(d);
		
		/*******************************************************************
		Calculate the partial derivatives of the distance between the two
		transformed points.
		*******************************************************************/
		
		/* Calculate the first transformation's translational partial derivatives: */
		derivative(ps1*8+0)=-d[0]/dist;
		derivative(ps1*8+1)=-d[1]/dist;
		derivative(ps1*8+2)=-d[2]/dist;
		
		/* Calculate the first transformation's rotational partial derivatives: */
		derivative(ps1*8+3)=-Scalar(2)*(d[0]*(q1[1]*p1[1]+q1[2]*p1[2])+d[1]*(q1[1]*p1[0]-Scalar(2)*q1[0]*p1[1]-q1[3]*p1[2])+d[2]*(q1[2]*p1[0]+q1[3]*p1[1]-Scalar(2)*q1[0]*p1[2]))*s1/dist;
		derivative(ps1*8+4)=-Scalar(2)*(d[0]*(-Scalar(2)*q1[1]*p1[0]+q1[0]*p1[1]+q1[3]*p1[2])+d[1]*(q1[0]*p1[0]+q1[2]*p1[2])+d[2]*(-q1[3]*p1[0]+q1[2]*p1[1]-Scalar(2)*q1[1]*p1[2]))*s1/dist;
		derivative(ps1*8+5)=-Scalar(2)*(d[0]*(-Scalar(2)*q1[2]*p1[0]-q1[3]*p1[1]+q1[0]*p1[2])+d[1]*(q1[3]*p1[0]-Scalar(2)*q1[2]*p1[1]+q1[1]*p1[2])+d[2]*(q1[0]*p1[0]+q1[1]*p1[1]))*s1/dist;
		derivative(ps1*8+6)=-Scalar(2)*(d[0]*(-q1[2]*p1[1]+q1[1]*p1[2])+d[1]*(q1[2]*p1[0]-q1[0]*p1[2])+d[2]*(-q1[1]*p1[0]+q1[0]*p1[1]))*s1/dist;
		
		/* Calculate the first transformation's scaling partial derivatives: */
		derivative(ps1*8+7)=-((rZ1*q1[1]-rY1*q1[2]+rW1*q1[0]+rX1*q1[3])*d[0]
		                     +(rX1*q1[2]-rZ1*q1[0]+rW1*q1[1]+rY1*q1[3])*d[1]
		                     +(rY1*q1[0]-rX1*q1[1]+rW1*q1[2]+rZ1*q1[3])*d[2])/dist;
		
		/* Calculate the second transformation's translational partial derivatives: */
		derivative(ps2*8+0)=d[0]/dist;
		derivative(ps2*8+1)=d[1]/dist;
		derivative(ps2*8+2)=d[2]/dist;
		
		/* Calculate the second transformation's rotational partial derivatives: */
		derivative(ps2*8+3)=Scalar(2)*(d[0]*(q2[1]*p2[1]+q2[2]*p2[2])+d[1]*(q2[1]*p2[0]-Scalar(2)*q2[0]*p2[1]-q2[3]*p2[2])+d[2]*(q2[2]*p2[0]+q2[3]*p2[1]-Scalar(2)*q2[0]*p2[2]))*s2/dist;
		derivative(ps2*8+4)=Scalar(2)*(d[0]*(-Scalar(2)*q2[1]*p2[0]+q2[0]*p2[1]+q2[3]*p2[2])+d[1]*(q2[0]*p2[0]+q2[2]*p2[2])+d[2]*(-q2[3]*p2[0]+q2[2]*p2[1]-Scalar(2)*q2[1]*p2[2]))*s2/dist;
		derivative(ps2*8+5)=Scalar(2)*(d[0]*(-Scalar(2)*q2[2]*p2[0]-q2[3]*p2[1]+q2[0]*p2[2])+d[1]*(q2[3]*p2[0]-Scalar(2)*q2[2]*p2[1]+q2[1]*p2[2])+d[2]*(q2[0]*p2[0]+q2[1]*p2[1]))*s2/dist;
		derivative(ps2*8+6)=Scalar(2)*(d[0]*(-q2[2]*p2[1]+q2[1]*p2[2])+d[1]*(q2[2]*p2[0]-q2[0]*p2[2])+d[2]*(-q2[1]*p2[0]+q2[0]*p2[1]))*s2/dist;
		
		/* Calculate the second transformation's scaling partial derivatives: */
		derivative(ps2*8+7)=((rZ2*q2[1]-rY2*q2[2]+rW2*q2[0]+rX2*q2[3])*d[0]
		                    +(rX2*q2[2]-rZ2*q2[0]+rW2*q2[1]+rY2*q2[3])*d[1]
		                    +(rY2*q2[0]-rX2*q2[1]+rW2*q2[2]+rZ2*q2[3])*d[2])/dist;
		}
	Scalar calcMag(void) const // Returns the magnitude of the current estimate
		{
		Scalar mag2(0);
		for(size_t i=0;i<numPointSets;++i)
			mag2+=Geometry::sqr(transforms[i].getTranslation())+Scalar(1)+Math::sqr(transforms[i].getScaling());
		return Math::sqrt(mag2);
		}
	void increment(const Math::Matrix& increment) // Increments the current estimate by the given difference vector
		{
		for(size_t i=0;i<numPointSets;++i)
			{
			Vector newT;
			for(int j=0;j<3;++j)
				newT[j]=transforms[i].getTranslation()[j]-increment(i*8+j);
			Scalar newQ[4];
			for(int j=0;j<4;++j)
				newQ[j]=transforms[i].getRotation().getQuaternion()[j]-increment(i*8+3+j);
			Scalar newS=transforms[i].getScaling()-increment(i*8+7);
			transforms[i]=Transform(newT,Transform::Rotation::fromQuaternion(newQ),newS);
			}
		}
	void normalize(void) // Normalizes the current estimate
		{
		/* Fix the first point set's transformation to the identity transform: */
		Transform it0=Geometry::invert(transforms[0]);
		transforms[0]=Transform::identity;
		for(size_t i=1;i<numPointSets;++i)
			{
			transforms[i].leftMultiply(it0);
			transforms[i].renormalize();
			}
		}
	};

template <class TransformFitterParam>
inline
typename TransformFitterParam::Transform
findTransform2(size_t numPoints,
               const typename TransformFitterParam::TiePoint* points0,
               const typename TransformFitterParam::TiePoint* points1)
	{
	return TransformFitterParam::Transform::identity;
	}

template <>
inline
OGTransformFitter::Transform
findTransform2<OGTransformFitter>(size_t numPoints,
                                  const OGTransformFitter::TiePoint* points0,
                                  const OGTransformFitter::TiePoint* points1)
	{
	typedef OGTransformFitter::Point Point;
	typedef OGTransformFitter::Transform Transform;
	
	/* Calculate both point sets' centroids: */
	Point::AffineCombiner cc0;
	Point::AffineCombiner cc1;
	for(size_t i=0;i<numPoints;++i)
		if(points0[i].value&&points1[i].value)
			{
			cc0.addPoint(points0[i]);
			cc1.addPoint(points1[i]);
			}
	Point c0=cc0.getPoint();
	Point c1=cc1.getPoint();
	std::cout<<"Centroids: "<<c0<<", "<<c1<<", result translation = "<<c1-c0<<std::endl;
	
	/* Calculate both point sets' inner products: */
	double ip0=0.0;
	double ip1=0.0;
	for(size_t pi=0;pi<numPoints;++pi)
		if(points0[pi].value&&points1[pi].value)
			{
			Point::Vector d0=points0[pi]-c0;
			Point::Vector d1=points1[pi]-c1;
			ip0+=Math::sqr(d0[0])+Math::sqr(d0[1])+Math::sqr(d0[2]);
			ip1+=Math::sqr(d1[0])+Math::sqr(d1[1])+Math::sqr(d1[2]);
			}
	
	/* Calculate the normalizing scaling factors: */
	double scale0=Math::sqrt(ip0);
	double scale1=Math::sqrt(ip1);
	std::cout<<"Scales: "<<scale0<<", "<<scale1<<", result scale = "<<scale1/scale0<<std::endl;
	
	/* Move both point sets to their centroids and scale them to uniform size: */
	Transform centroidTransform0=Transform::translateToOriginFrom(c0);
	centroidTransform0.leftMultiply(Transform::scale(1.0/scale0));
	Transform centroidTransform1=Transform::translateToOriginFrom(c1);
	centroidTransform1.leftMultiply(Transform::scale(1.0/scale1));
	std::vector<Point> cPoints0;
	std::vector<Point> cPoints1;
	size_t cNumPoints=0;
	for(size_t i=0;i<numPoints;++i)
		if(points0[i].value&&points1[i].value)
			{
			cPoints0.push_back(centroidTransform0.transform(points0[i]));
			cPoints1.push_back(centroidTransform1.transform(points1[i]));
			++cNumPoints;
			}
	
	/* Calculate the inner product between the two point sets: */
	double m[3][3];
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			m[i][j]=0.0;
	for(size_t pi=0;pi<cNumPoints;++pi)
		for(int i=0;i<3;++i)
			for(int j=0;j<3;++j)
				m[i][j]+=cPoints0[pi][i]*cPoints1[pi][j];
	
	/* Calculate the coefficients of the quaternion-based characteristic polynomial of the quaternion key matrix: */
	double q4=1.0;
	double q3=0.0;
	double q2=0.0;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
			q2-=2.0*Math::sqr(m[i][j]);
	double q1=8.0*(m[0][0]*m[1][2]*m[2][1]+m[1][1]*m[2][0]*m[0][2]+m[2][2]*m[0][1]*m[1][0])
	         -8.0*(m[0][0]*m[1][1]*m[2][2]+m[1][2]*m[2][0]*m[0][1]+m[2][1]*m[1][0]*m[0][2]);
	double qd0=Math::sqr(Math::sqr(m[0][1])+Math::sqr(m[0][2])-Math::sqr(m[1][0])-Math::sqr(m[2][0]));
	double qd1=(-Math::sqr(m[0][0])+Math::sqr(m[1][1])+Math::sqr(m[2][2])+Math::sqr(m[1][2])+Math::sqr(m[2][1])-2.0*(m[1][1]*m[2][2]-m[1][2]*m[2][1]))
	          *(-Math::sqr(m[0][0])+Math::sqr(m[1][1])+Math::sqr(m[2][2])+Math::sqr(m[1][2])+Math::sqr(m[2][1])+2.0*(m[1][1]*m[2][2]-m[1][2]*m[2][1]));
	double qd2=(-(m[0][2]+m[2][0])*(m[1][2]-m[2][1])+(m[0][1]-m[1][0])*(m[0][0]-m[1][1]-m[2][2]))
	          *(-(m[0][2]-m[2][0])*(m[1][2]+m[2][1])+(m[0][1]-m[1][0])*(m[0][0]-m[1][1]+m[2][2]));
	double qd3=(-(m[0][2]+m[2][0])*(m[1][2]+m[2][1])-(m[0][1]+m[1][0])*(m[0][0]+m[1][1]-m[2][2]))
	          *(-(m[0][2]-m[2][0])*(m[1][2]-m[2][1])-(m[0][1]+m[1][0])*(m[0][0]+m[1][1]+m[2][2]));
	double qd4=((m[0][1]+m[1][0])*(m[1][2]+m[2][1])+(m[0][2]+m[2][0])*(m[0][0]-m[1][1]+m[2][2]))
	          *(-(m[0][1]-m[1][0])*(m[1][2]-m[2][1])+(m[0][2]+m[2][0])*(m[0][0]+m[1][1]+m[2][2]));
	double qd5=((m[0][1]+m[1][0])*(m[1][2]-m[2][1])+(m[0][2]-m[2][0])*(m[0][0]-m[1][1]-m[2][2]))
	          *(-(m[0][1]-m[1][0])*(m[1][2]+m[2][1])+(m[0][2]-m[2][0])*(m[0][0]+m[1][1]-m[2][2]));
	double q0=qd0+qd1+qd2+qd3+qd4+qd5;
	
	/* Calculate the optimal rotation: */
	double lambda=Math::mid(ip0,ip1);
	double lambda0;
	do
		{
		lambda0=lambda;
		double poly=(((q4*lambda+q3)*lambda+q2)*lambda+q1)*lambda+q0;
		double dPoly=((4.0*q4*lambda+3.0*q3)*lambda+2.0*q2)*lambda+q1;
		lambda-=poly/dPoly;
		}
	while(Math::abs(lambda-lambda0)<1.0e-8);
	std::cout<<"Largest eigenvalue of key matrix: "<<lambda<<std::endl;
	
	/* Find the eigenvector corresponding to the largest eigenvalue: */
	Math::Matrix k(4,4);
	k(0,0)=m[0][0]+m[1][1]+m[2][2];
	k(0,1)=m[1][2]-m[2][1];
	k(0,2)=m[2][0]-m[0][2];
	k(0,3)=m[0][1]-m[1][0];
	k(1,0)=m[1][2]-m[2][1];
	k(1,1)=m[0][0]-m[1][1]-m[2][2];
	k(1,2)=m[0][1]+m[1][0];
	k(1,3)=m[2][0]+m[0][2];
	k(2,0)=m[2][0]-m[0][2];
	k(2,1)=m[0][1]+m[1][0];
	k(2,2)=-m[0][0]+m[1][1]-m[2][2];
	k(2,3)=m[1][2]+m[2][1];
	k(3,0)=m[0][1]-m[1][0];
	k(3,1)=m[2][0]+m[0][2];
	k(3,2)=m[1][2]+m[2][1];
	k(3,3)=-m[0][0]-m[1][1]+m[2][2];
	
	#if 0
	
	/* Test the polynomial: */
	for(double x=-2.0;x<=2.0;x+=1.0/16.0)
		{
		double p1=(((q4*x+q3)*x+q2)*x+q1)*x+q0;
		
		Math::Matrix kp(4,4);
		for(int i=0;i<3;++i)
			for(int j=0;j<4;++j)
				kp(i,j)=i==j?x-k(i,j):-k(i,j);
		double p2=kp.determinant();
		std::cout<<x<<": "<<p1<<", "<<p2<<std::endl;
		}
	
	
	#endif
	
	std::pair<Math::Matrix,Math::Matrix> jacobi=k.jacobiIteration();
	std::cout<<"Eigenvalues of key matrix: ";
	for(int i=0;i<4;++i)
		std::cout<<", "<<jacobi.second(i);
	std::cout<<std::endl;
	double maxE=jacobi.second(0);
	int maxEIndex=0;
	for(int i=1;i<4;++i)
		if(maxE<jacobi.second(i))
			{
			maxE=jacobi.second(i);
			maxEIndex=i;
			}
	std::cout<<"Largest eigenvector: "<<jacobi.first(0,maxEIndex)<<", "<<jacobi.first(1,maxEIndex)<<", "<<jacobi.first(2,maxEIndex)<<", "<<jacobi.first(3,maxEIndex)<<std::endl;
	Transform::Rotation rotation=Transform::Rotation::fromQuaternion(jacobi.first(1,maxEIndex),jacobi.first(2,maxEIndex),jacobi.first(3,maxEIndex),jacobi.first(0,maxEIndex));
	std::cout<<"Result rotation: "<<rotation<<std::endl;
	
	Transform result=Geometry::invert(centroidTransform1);
	result*=Transform::rotate(rotation);
	result*=centroidTransform0;
	
	return result;
	}

template <class TransformFitterParam>
inline
typename TransformFitterParam::Transform*
findTransform(size_t numPointSets,
              size_t numPoints,
              typename TransformFitterParam::TiePoint** pointSets)
	{
	typedef typename TransformFitterParam::Point Point;
	typedef typename TransformFitterParam::TiePoint TiePoint;
	typedef typename TransformFitterParam::Transform Transform;
	
	/* Calculate initial transformation estimates: */
	Transform* preTransforms=new Transform[numPointSets];
	
	/* Calculate the first point set's centroid: */
	typename Point::AffineCombiner cc;
	for(size_t j=0;j<numPoints;++j)
		if(pointSets[0][j].value)
			cc.addPoint(pointSets[0][j]);
	preTransforms[0]=Transform::translateToOriginFrom(cc.getPoint());
	
	/* Calculate initial transformations from the first to all other point sets: */
	for(size_t i=1;i<numPointSets;++i)
		{
		preTransforms[i]=preTransforms[0];
		preTransforms[i]*=findTransform2<TransformFitterParam>(numPoints,pointSets[i],pointSets[0]);
		std::cout<<"Initial transform "<<i<<": "<<Misc::ValueCoder<Transform>::encode(preTransforms[i])<<std::endl;
		
		double rms2Sum=0.0;
		size_t rmsNp=0;
		for(size_t j=0;j<numPoints;++j)
			if(pointSets[0][j].value&&pointSets[i][j].value)
				{
				rms2Sum+=Geometry::sqrDist(preTransforms[0].transform(pointSets[0][j]),preTransforms[i].transform(pointSets[i][j]));
				++rmsNp;
				}
		
		std::cout<<"Initial RMS distance between point set "<<0<<" and "<<i<<": "<<Math::sqrt(rms2Sum/double(rmsNp))<<std::endl;
		}
	
	/* Transform all point sets by the initial transformations: */
	TiePoint** cPointSets=new TiePoint*[numPointSets];
	for(size_t i=0;i<numPointSets;++i)
		{
		cPointSets[i]=new TiePoint[numPoints];
		for(size_t j=0;j<numPoints;++j)
			{
			cPointSets[i][j]=preTransforms[i].transform(pointSets[i][j]);
			cPointSets[i][j].value=pointSets[i][j].value;
			}
		}
	
	/* Create a distance minimizer: */
	Geometry::LevenbergMarquardtMinimizer<TransformFitterParam> minimizer;
	minimizer.maxNumIterations=5000000;
	minimizer.epsilon2=1.0e-40;
	TransformFitterParam tf(numPointSets,numPoints,cPointSets);
	
	/* Minimize the fully-connected RMS distance between all point sets: */
	Transform* bestTransforms=new Transform[numPointSets];
	for(size_t i=0;i<numPointSets;++i)
		bestTransforms[i]=Transform::identity;
	double bestResidual=0.0; // Math::Constants<double>::max;
	for(int i=0;i<5;++i)
		{
		double residual=minimizer.minimize(tf);
		
		/* Print the residual: */
		std::cout<<"Alignment residual: "<<residual<<std::endl;
		
		if(bestResidual>residual)
			{
			for(size_t j=0;j<numPointSets;++j)
				bestTransforms[j]=tf.getTransform(j);
			bestResidual=residual;
			}
		}
	
	/* Clean up: */
	for(size_t i=0;i<numPointSets;++i)
		delete[] cPointSets[i];
	delete[] cPointSets;
	
	/* Undo all pretransformations: */
	bestTransforms[0]=Transform::identity;
	for(size_t i=1;i<numPointSets;++i)
		{
		Transform t=Geometry::invert(preTransforms[0]);
		t*=bestTransforms[i];
		t*=preTransforms[i];
		t.renormalize();
		bestTransforms[i]=t;
		}
	delete[] preTransforms;
	
	/* Calculate all pairwise RMS distances: */
	for(size_t i0=0;i0<numPointSets-1;++i0)
		for(size_t i1=i0+1;i1<numPointSets;++i1)
			{
			double rms2Sum=0.0;
			size_t rmsNp=0;
			for(size_t j=0;j<numPoints;++j)
				if(pointSets[i0][j].value&&pointSets[i1][j].value)
					{
					rms2Sum+=Geometry::sqrDist(bestTransforms[i0].transform(pointSets[i0][j]),bestTransforms[i1].transform(pointSets[i1][j]));
					++rmsNp;
					}
			
			std::cout<<"RMS distance between point set "<<i0<<" and "<<i1<<": "<<Math::sqrt(rms2Sum/double(rmsNp))<<std::endl;
			}
	
	/* Return the result transformations: */
	return bestTransforms;
	}

class AlignMultiPoints:public Vrui::Application
	{
	/* Embedded classes: */
	private:
	typedef OGTransformFitter::TiePoint TiePoint;
	typedef OGTransformFitter::Transform Transform;
	
	/* Elements: */
	private:
	size_t numPointSets;
	size_t numPoints;
	std::vector<std::vector<TiePoint> > pointSets;
	Transform* transforms;
	
	/* Constructors and destructors: */
	public:
	AlignMultiPoints(int& argc,char**& argv);
	virtual ~AlignMultiPoints(void);
	
	/* Methods from Vrui::Application: */
	virtual void display(GLContextData& contextData) const;
	};

AlignMultiPoints::AlignMultiPoints(int& argc,char**& argv)
	:Vrui::Application(argc,argv),
	 transforms(0)
	{
	/* Read all point sets: */
	for(int i=1;i<argc;++i)
		{
		/* Open the input file: */
		IO::ValueSource reader(IO::openFile(argv[i]));
		reader.setWhitespace(',',true);
		reader.setPunctuation('\n',true);
		reader.skipWs();
		
		/* Add a new point set: */
		pointSets.push_back(std::vector<TiePoint>());
		
		/* Read points until end of file: */
		while(!reader.eof())
			{
			TiePoint p=TiePoint::Point::origin;
			p.value=true;
			try
				{
				for(int i=0;i<3;++i)
					p[i]=TiePoint::Scalar(reader.readNumber());
				}
			catch(IO::ValueSource::NumberError)
				{
				p.value=false;
				}
			pointSets.back().push_back(p);
			reader.skipLine();
			reader.skipWs();
			}
		}
	
	#if 0
	/* Rotate and jitter the first point set: */
	Transform t=Transform::translate(Transform::Vector(10.0,-20.0,5.0));
	t*=Transform::rotate(Transform::Rotation::rotateAxis(Transform::Vector(0.4,-0.1,0.3),Math::rad(45.0)));
	for(size_t i=0;i<pointSets[1].size();++i)
		pointSets[0][i]=TiePoint(t.transform(pointSets[0][i])+Geometry::randVectorNormal<TiePoint::Scalar,3>(0.0),pointSets[0][i].value);
	#endif
	
	/* Make a C array of point sets: */
	numPointSets=pointSets.size();
	TiePoint** aPointSets=new TiePoint*[numPointSets];
	for(size_t i=0;i<numPointSets;++i)
		aPointSets[i]=&(pointSets[i].front());
	numPoints=pointSets[0].size();
	for(size_t i=1;i<pointSets.size();++i)
		if(numPoints>pointSets[i].size())
			numPoints=pointSets[i].size();
	
	/* Align the point sets: */
	transforms=findTransform<OGTransformFitter>(pointSets.size(),numPoints,aPointSets);
	for(size_t i=0;i<numPointSets;++i)
		std::cout<<Misc::ValueCoder<Transform>::encode(transforms[i])<<std::endl;
	
	/* Clean up: */
	delete[] aPointSets;
	}

AlignMultiPoints::~AlignMultiPoints(void)
	{
	delete[] transforms;
	}

void AlignMultiPoints::display(GLContextData& contextData) const
	{
	glPushAttrib(GL_ENABLE_BIT|GL_LINE_BIT|GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(3.0f);
	glLineWidth(1.0f);
	
	static const GLColor<GLfloat,3> colors[6]=
		{
		GLColor<GLfloat,3>(1.0f,0.0f,0.0f),
		GLColor<GLfloat,3>(1.0f,1.0f,0.0f),
		GLColor<GLfloat,3>(0.0f,1.0f,0.0f),
		GLColor<GLfloat,3>(0.0f,1.0f,1.0f),
		GLColor<GLfloat,3>(0.0f,0.0f,1.0f),
		GLColor<GLfloat,3>(1.0f,0.0f,1.0f)
		};
	
	glBegin(GL_POINTS);
	for(size_t i=0;i<numPointSets;++i)
		{
		glColor(colors[i%6]);
		for(size_t j=0;j<numPoints;++j)
			if(pointSets[i][j].value)
				glVertex(transforms[i].transform(pointSets[i][j]));
		}
	glEnd();
	
	glBegin(GL_LINES);
	for(size_t i0=0;i0<numPointSets-1;++i0)
		for(size_t i1=i0+1;i1<numPointSets;++i1)
			for(size_t j=0;j<numPoints;++j)
				if(pointSets[i0][j].value&&pointSets[i1][j].value)
					{
					glColor(colors[i0%6]);
					glVertex(transforms[i0].transform(pointSets[i0][j]));
					glColor(colors[i1%6]);
					glVertex(transforms[i1].transform(pointSets[i1][j]));
					}
	glEnd();
	
	glPopAttrib();
	}

VRUI_APPLICATION_RUN(AlignMultiPoints)
