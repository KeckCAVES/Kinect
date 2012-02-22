/***********************************************************************
MakeHuffmanTable - Utility to create encoding and decoding codebooks for
a Huffman coder based on a set of codes and frequencies read from an
input file.
Copyright (c) 2010-2011 Oliver Kreylos

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

#include <vector>
#include <iostream>
#include <iomanip>
#include <IO/OpenFile.h>
#include <IO/CSVSource.h>

struct CodeNode
	{
	/* Elements: */
	public:
	unsigned int children[2]; // Indices of the node's two child nodes
	size_t frequency; // Total number of occurrences of the node's descendants
	};

struct HuffmanCode
	{
	/* Elements: */
	public:
	unsigned int bits; // The code's bits, ending at the LSB
	unsigned int length; // The total number of bits in the code
	
	/* Constructors and destructors: */
	HuffmanCode(void) // Creates an empty Huffman code
		:bits(0x0U),length(0)
		{
		}
	};

HuffmanCode encode(unsigned int code,const std::vector<CodeNode>& nodes,unsigned int numLeaves,unsigned int node,const HuffmanCode& prefix)
	{
	/* Check if the node is a leaf: */
	if(node<numLeaves)
		{
		if(node==code)
			return prefix;
		else
			return HuffmanCode();
		}
	else
		{
		/* Recurse into the two children: */
		HuffmanCode childPrefix=prefix;
		childPrefix.bits<<=1;
		++childPrefix.length;
		HuffmanCode result0=encode(code,nodes,numLeaves,nodes[node].children[0],childPrefix);
		childPrefix.bits|=0x01U;
		HuffmanCode result1=encode(code,nodes,numLeaves,nodes[node].children[1],childPrefix);
		if(result0.length==0)
			result0=result1;
		return result0;
		}
	}

int main(int argc,char* argv[])
	{
	/* Read a list of codes and frequencies from the input file: */
	std::vector<CodeNode> nodes;
	size_t totalFreq=0;
	
	{
	/* Open the input file as a csv file: */
	IO::AutoFile fcs(IO::openFile(argv[1]));
	IO::CSVSource source(*fcs);
	
	/* Read all codes and their frequencies: */
	unsigned int nextCode=0;
	while(!source.eof())
		{
		/* Read the next code: */
		unsigned int code=source.readField<unsigned int>();
		if(code!=nextCode)
			std::cerr<<"Missing code "<<nextCode<<" in input file!"<<std::endl;
		++nextCode;
		CodeNode cn;
		cn.children[0]=cn.children[1]=0;
		cn.frequency=source.readField<unsigned int>();
		totalFreq+=cn.frequency;
		nodes.push_back(cn);
		}
	}
	
	/* Remember the number of leaf nodes: */
	unsigned int numLeaves=nodes.size();
	
	/* Combine leaf nodes until there is a single code tree: */
	bool* merged=new bool[numLeaves+numLeaves-1];
	for(unsigned int i=0;i<numLeaves+numLeaves-1;++i)
		merged[i]=false;
	for(unsigned int interior=0;interior<numLeaves-1;++interior)
		{
		/* Find the two smallest active nodes: */
		unsigned int index0;
		size_t freq0=totalFreq;
		unsigned int index1;
		size_t freq1=totalFreq;
		for(unsigned int i=0;i<numLeaves+interior;++i)
			if(!merged[i])
				{
				if(nodes[i].frequency<freq0)
					{
					index1=index0;
					freq1=freq0;
					index0=i;
					freq0=nodes[i].frequency;
					}
				else if(nodes[i].frequency<freq1)
					{
					index1=i;
					freq1=nodes[i].frequency;
					}
				}
		
		/* Merge the two nodes: */
		std::cout<<"Node "<<numLeaves+interior<<": Joining nodes "<<index0<<" and "<<index1<<" of frequencies "<<freq0<<" and "<<freq1<<std::endl;
		CodeNode cn;
		cn.children[0]=index0;
		cn.children[1]=index1;
		cn.frequency=freq0+freq1;
		nodes.push_back(cn);
		merged[index0]=true;
		merged[index1]=true;
		}
	
	/* Print the Huffman encoding table: */
	std::cout<<"static const unsigned int huffmanCodes["<<numLeaves<<"][2]="<<std::endl;
	std::cout<<"\t{"<<std::endl<<"\t";
	int column=0;
	for(unsigned int code=0;code<numLeaves;++code)
		{
		HuffmanCode result=encode(code,nodes,numLeaves,numLeaves+numLeaves-2,HuffmanCode());
		std::cout<<"{0x"<<std::hex<<result.bits<<"U,"<<std::dec<<result.length<<"},";
		++column;
		if(column==8)
			{
			std::cout<<std::endl<<"\t";
			column=0;
			}
		}
	if(column>0)
		std::cout<<std::endl<<"\t";
	std::cout<<"};"<<std::endl;
	
	/* Print the Huffman decoding table: */
	std::cout<<"static const unsigned int huffmanNodes["<<numLeaves-1<<"][2]="<<std::endl;
	std::cout<<"\t{"<<std::endl<<"\t";
	column=0;
	for(unsigned int node=numLeaves;node<numLeaves+numLeaves-1;++node)
		{
		std::cout<<'{'<<nodes[node].children[0]<<','<<nodes[node].children[1]<<"},";
		++column;
		if(column==8)
			{
			std::cout<<std::endl<<"\t";
			column=0;
			}
		}
	if(column>0)
		std::cout<<std::endl<<"\t";
	std::cout<<"};"<<std::endl;
	
	return 0;
	}
