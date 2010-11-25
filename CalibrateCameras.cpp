#include <iostream>
#include <Misc/File.h>
#include <Misc/FileCharacterSource.h>
#include <Misc/CSVSource.h>
#include <Math/Math.h>
#include <Math/Matrix.h>

int main(void)
	{
	/* Create the linear system: */
	Math::Matrix a(12,12,0.0);
	
	{
	/* Open the calibration data file: */
	Misc::FileCharacterSource dataSource("CalibrationData.csv");
	Misc::CSVSource data(dataSource);
	
	unsigned int numEntries=0;
	while(!data.eof())
		{
		/* Read a calibration entry from the data file: */
		double x=data.readField<double>();
		double y=data.readField<double>();
		double z=data.readField<double>();
		double s=data.readField<double>()/640.0;
		double t=data.readField<double>()/480.0;
		
		/* Insert the entry's two linear equations into the linear system: */
		double eq[2][12];
		eq[0][0]=x;
		eq[0][1]=y;
		eq[0][2]=z;
		eq[0][3]=1.0;
		eq[0][4]=0.0;
		eq[0][5]=0.0;
		eq[0][6]=0.0;
		eq[0][7]=0.0;
		eq[0][8]=-s*x;
		eq[0][9]=-s*y;
		eq[0][10]=-s*z;
		eq[0][11]=-s;
		
		eq[1][0]=0.0;
		eq[1][1]=0.0;
		eq[1][2]=0.0;
		eq[1][3]=0.0;
		eq[1][4]=x;
		eq[1][5]=y;
		eq[1][6]=z;
		eq[1][7]=1.0;
		eq[1][8]=-t*x;
		eq[1][9]=-t*y;
		eq[1][10]=-t*z;
		eq[1][11]=-t;
		
		for(int row=0;row<2;++row)
			{
			for(unsigned int i=0;i<12;++i)
				for(unsigned int j=0;j<12;++j)
					a.set(i,j,a(i,j)+eq[row][i]*eq[row][j]);
			}
		
		++numEntries;
		}
	std::cout<<numEntries<<" calibration data entries read from file"<<std::endl;
	}
	
	/* Find the linear system's smallest eigenvalue: */
	std::pair<Math::Matrix,Math::Matrix> qe=a.jacobiIteration();
	unsigned int minEIndex=0;
	double minE=Math::abs(qe.second(0,0));
	for(unsigned int i=1;i<12;++i)
		{
		if(minE>Math::abs(qe.second(i,0)))
			{
			minEIndex=i;
			minE=Math::abs(qe.second(i,0));
			}
		}
	
	/* Create the normalized homography: */
	Math::Matrix hom(3,4);
	double scale=qe.first(11,minEIndex);
	for(int i=0;i<3;++i)
		for(int j=0;j<4;++j)
			hom.set(i,j,qe.first(i*4+j,minEIndex)/scale);
	
	{
	/* Open the calibration data file: */
	Misc::FileCharacterSource dataSource("CalibrationData.csv");
	Misc::CSVSource data(dataSource);
	
	/* Test the homography on all calibration data entries: */
	while(!data.eof())
		{
		/* Read a calibration entry from the data file: */
		Math::Matrix world(4,1);
		for(unsigned int i=0;i<3;++i)
			world.set(i,data.readField<double>());
		world.set(3,1.0);
		double s=data.readField<double>();
		double t=data.readField<double>();
		
		/* Apply the homography: */
		Math::Matrix str=hom*world;
		std::cout<<"Result: s = "<<str(0)/str(2)<<", t = "<<str(1)/str(2)<<std::endl;
		}
	}
	
	/* Open the calibration file: */
	Misc::File matrixFile("CameraCalibrationMatrices.dat","wb",Misc::File::LittleEndian);
	
	/* Create the depth projection matrix: */
	Math::Matrix depthProjection(4,4,0.0);
	double cameraFov=560.0;
	depthProjection(0,0)=1.0/cameraFov;
	depthProjection(0,3)=-320.0/cameraFov;
	depthProjection(1,1)=1.0/cameraFov;
	depthProjection(1,3)=-240.0/cameraFov;
	depthProjection(2,3)=-1.0;
	depthProjection(3,2)=-1.0/34400.0;
	depthProjection(3,3)=1090.0/34400.0;
	
	/* Save the depth projection matrix: */
	for(unsigned int i=0;i<4;++i)
		for(unsigned int j=0;j<4;++j)
			matrixFile.write<double>(depthProjection(i,j));
	
	/* Create the color projection matrix by extending the homography: */
	Math::Matrix colorProjection(4,4);
	for(unsigned int i=0;i<2;++i)
		for(unsigned int j=0;j<4;++j)
			colorProjection(i,j)=hom(i,j);
	for(unsigned int j=0;j<4;++j)
		colorProjection(2,j)=j==2?1.0:0.0;
	for(unsigned int j=0;j<4;++j)
		colorProjection(3,j)=hom(2,j);
	
	/* Modify the color projection matrix by the depth projection matrix: */
	colorProjection*=depthProjection;
	
	/* Save the color projection matrix: */
	for(unsigned int i=0;i<4;++i)
		for(unsigned int j=0;j<4;++j)
			matrixFile.write<double>(colorProjection(i,j));
	
	return 0;
	}
