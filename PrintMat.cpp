#include <iostream>
#include <iomanip>
#include <IO/File.h>
#include <IO/OpenFile.h>
#include <Geometry/ProjectiveTransformation.h>

int main(int argc,char* argv[])
	{
	IO::FilePtr matFile(IO::openFile(argv[1],IO::File::ReadOnly));
	matFile->setEndianness(Misc::LittleEndian);
	matFile->skip<double>(4*4);
	double mat[4][4];
	for(int i=0;i<4;++i)
		for(int j=0;j<4;++j)
			mat[i][j]=matFile->read<double>();
	
	for(int i=0;i<4;++i)
		{
		std::cout<<std::setw(12)<<mat[i][0];
		for(int j=1;j<4;++j)
			std::cout<<"   "<<std::setw(12)<<mat[i][j];
		std::cout<<std::endl;
		}
	
	return 0;
	}
