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
		double s=data.readField<double>();
		double t=data.readField<double>();
		
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
	
	/* Save the calibration matrix: */
	Misc::File matrixFile("CameraCalibrationMatrix.dat","wb",Misc::File::LittleEndian);
	for(unsigned int i=0;i<3;++i)
		for(unsigned int j=0;j<4;++j)
			matrixFile.write<double>(hom(i,j));
	
	return 0;
	}
