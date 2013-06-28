#include "DLT.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <future>
#include <tuple>

using namespace std;

#define EPSILON .0001

/***************************************
 * Helper functions                    *
 ***************************************/

double CalcEntropy(Sample* data, int count) 
{
	vector<int> counts;
	for(int i=0;i<count;++i) 
	{
		Sample s = data[i];
		if(s.type >= counts.size()) 
		{
			counts.resize(s.type + 1);
		}
		++counts[s.type];
	}
	double entropy = 0;
	for(int i = 0; i < counts.size(); i++) {
		double prob = counts[i] / (double)count;
		if(prob > 0)
			entropy -= prob * log(prob)/log(2);
	}
	return entropy;
}

double CalcEntropy(const Sample* data, int count, int splitId, int splitVal) 
{
	vector<int> lowCount, highCount;
	int lowSize=0, highSize=0;
	for(int i=0;i<count;++i) 
	{
		Sample s = data[i];
		if(s.iAttr[splitId]>splitVal)
		{
			if(s.type >= highCount.size()) 
			{
				highCount.resize(s.type + 1);
			}
			++highCount[s.type];
			++highSize;
		}
		else
		{
			if(s.type >= lowCount.size()) 
			{
				lowCount.resize(s.type + 1);
			}
			++lowCount[s.type];
			++lowSize;
		}
	}
	double highEntropy = 0, lowEntropy=0;
	for(int i = 0; i < highCount.size(); i++) 
	{
		double prob = highCount[i] / (double)count;
		if(prob > 0)
		{
			highEntropy -= prob * log(prob)/log(2);
		}
	}
	for(int i=0;i<lowCount.size();++i)
	{
		double prob=lowCount[i]/(double)count;
		if(prob>0)
		{
			lowEntropy-=prob*log(prob)/log(2);
		}
	}
	return highEntropy*highSize/(double)count + lowEntropy*lowSize/(double)count;
}

Sample* splitOn(const Sample* data, int count, int id, int value, int &retSize, bool greater) 
{
	Sample* temp, *ret=new Sample[count];
	retSize=0;
	for(int i=0;i<count;++i) 
	{
		Sample s = data[i];
		if(s.iAttr[id] <= value ^ greater) //xor
		{
			ret[retSize]=s;
			++retSize;
		}
	}
	temp=new Sample[retSize];
	memcpy(temp, ret, sizeof(Sample)*retSize);
	delete ret;
	return temp;
}

/*
This getBestSplit does not use async, it is serial
*/

/*
void getBestSplit(const Sample* data, int attr, int count, int& id, int& value) 
{
	double lowestEntropyMeasure = 1000000.0;
	id = 0;
	value = 0;
	//Test each split
	for(int i = 0; i < attr; i++) 
	{
		int biggestValue = 0;
		for(int j=0;j<count;++j) 
		{
			if(biggestValue < data[j].iAttr[i])
			{
				biggestValue = data[j].iAttr[i];
			}
		}
		for(int curValue = 0; curValue < biggestValue; ++curValue) 
		{
			int highSideSize=0, lowSideSize=0;
			Sample* highSide = splitOn(data, count, i, curValue, highSideSize, true);
			Sample* lowSide = splitOn(data, count, i, curValue, lowSideSize, false);
			double entropy = CalcEntropy(highSide, highSideSize)*highSideSize/(double)count +
				             CalcEntropy(lowSide, lowSideSize)*lowSideSize/(double)count;
			if(entropy < lowestEntropyMeasure) 
			{
				lowestEntropyMeasure = entropy;
				id = i;
				value = curValue;
			}
			delete highSide;
			delete lowSide;
		}
	}
}
*/


void getBestSplit(const Sample* data, int attr, int count, int& id, int& value) 
{
	double lowestEntropyMeasure = 1000000.0;
	id = 0;
	value = 0;
	//Test each split
	vector<future<tuple<float, int, int>>> futs;
	for(int i = 0; i < attr; i++) 
	{
		futs.push_back(async(launch::async,
			[=]()->tuple<float, int, int>
			{
				vector<future<tuple<float, int, int>>> subFuts;
				//printf("Starting task %d of %d\n", i+1, attr);
				int biggestValue = 0;
				for(int j=0;j<count;++j) 
				{
					if(biggestValue < data[j].iAttr[i])
					{
						biggestValue = data[j].iAttr[i];
					}
				}
				for(int j=0;j<3;++j)
				{
					subFuts.push_back(async(launch::async,
						[=]()->tuple<float, int, int>
						{
							double lowestEnt=1000000;
							tuple<float, int, int> ret;
							for(int curValue = ((j*biggestValue)/3); curValue < (((j+1)*biggestValue)/3); ++curValue) 
							{
								//printf("Starting test %d of %d (task %d of %d)\n", curValue+1, biggestValue, i+1, attr);
								int highSideSize=0, lowSideSize=0;
								double entropy = CalcEntropy(data, count, i, curValue);
								if(entropy<lowestEnt)
								{
									lowestEnt=entropy;
									ret=make_tuple(entropy, i, curValue);
								}
							}
							return ret;
						}));
				}
				double ent=1000000;
				int index;
				tuple<float, int, int> ret;
				for(int i=0;i<subFuts.size();++i)
				{
					auto tup=subFuts[i].get();
					if(get<0>(tup) < ent)
					{
						ent=get<0>(tup);
						ret=make_tuple(get<0>(tup), get<1>(tup), get<2>(tup));	
					}
				}
				//printf("Done task %d of %d\n", i+1, attr);
				return ret;
			}));
	}
	double ent=1000000;
	int index;
	for(int i=0;i<futs.size();++i)
	{
		auto tup=futs[i].get();
		if(get<0>(tup) < ent)
		{
			ent=get<0>(tup);
			id=get<1>(tup);
			value=get<2>(tup);
		}
	}
}


/*
void getBestSplit(const Sample* data, int attr, int count, int& id, int& value) 
{
	double lowestEntropyMeasure = 1000000.0;
	id = 0;
	value = 0;
	//Test each split
	vector<future<tuple<float, int, int>>> futs;
	for(int i = 0; i < attr; i++) 
	{
		futs.push_back(async(launch::async,
			[=]()->tuple<float, int, int>
			{
				printf("Starting task %d of %d\n", i+1, attr);
				int biggestValue = 0;
				for(int j=0;j<count;++j) 
				{
					if(biggestValue < data[j].iAttr[i])
					{
						biggestValue = data[j].iAttr[i];
					}
				}
				double lowestEnt=1000000;
				tuple<float, int, int> ret;
				for(int curValue = 0; curValue < biggestValue; ++curValue) 
				{
					printf("Starting test %d of %d (task %d of %d)\n", curValue+1, biggestValue, i+1, attr);
					int highSideSize=0, lowSideSize=0;
					Sample* highSide = splitOn(data, count, i, curValue, highSideSize, true);
					Sample* lowSide = splitOn(data, count, i, curValue, lowSideSize, false);
					double entropy = CalcEntropy(highSide, highSideSize)*highSideSize/(double)count +
										CalcEntropy(lowSide, lowSideSize)*lowSideSize/(double)count;
					if(entropy<lowestEnt)
					{
						lowestEnt=entropy;
						ret=make_tuple(entropy, i, curValue);
					}
					delete highSide;
					delete lowSide;
				}
				return ret;
			}));
	}
	double ent=1000000;
	int index;
	for(int i=0;i<futs.size();++i)
	{
		auto tup=futs[i].get();
		if(get<0>(tup) < ent)
		{
			ent=get<0>(tup);
			id=get<1>(tup);
			value=get<2>(tup);
		}
	}
}
*/

/***************************************
 * Constructor                         *
 ***************************************/
DLT::DLT(Sample* data, int count, int attr, int depth) 
{
	double Entropy = CalcEntropy(data, count);
	if(depth <= 0 || Entropy < EPSILON) 
	{
		printf("\nConstructing leaf node at height %d\n", depth);
		vector<int> counts;
		for(int i=0;i<count;++i) 
		{
			Sample s = data[i];
			if(s.type >= counts.size()) 
			{
				counts.resize(s.type + 1);
			}
			++counts[s.type];
		}
		splitId = -1;
		lowSide = NULL;
		highSide = NULL;
		int max_count = 0;
		for(int i = 0; i < counts.size(); ++i) 
		{
			if(counts[i] > counts[max_count])
				max_count = i;
		}
		splitVal = max_count;
		delete data;
		return;
	} 
	else 
	{
		printf("\nConstructing node at height %d\n", depth);
		int highSideSize, lowSideSize;
		getBestSplit(data, attr, count, splitId, splitVal);
		Sample *highData=splitOn(data, count, splitId, splitVal, highSideSize, true);
		Sample *lowData=splitOn(data, count, splitId, splitVal, lowSideSize, false);
		delete data;
		highSide = new DLT(highData, highSideSize, attr, depth - 1);
		lowSide = new DLT(lowData, lowSideSize, attr, depth - 1);
	}
}

/***************************************
 * Member Functions                    *
 ***************************************/
int DLT::Classify(const Sample& s) {
	if(splitId < 0)
		return splitVal;
	if(s.iAttr[splitId] > splitVal)
		return highSide->Classify(s);
	else
		return lowSide->Classify(s);
}

void DLT::Save(ostream& fout) {
	if(splitId == -1) {
		fout << "Leaf " << splitVal << endl;
	} else {
		fout << "Branch " << splitId << " " << splitVal << endl;
		lowSide->Save(fout);
		highSide->Save(fout);
	}
}

DLT::DLT(istream& fin) {
	string type;
	fin >> type;
	if(type == "Leaf") {
		splitId = -1;
		fin >> splitVal;
	} else {
		fin >> splitId >> splitVal;
		lowSide = new DLT(fin);
		highSide = new DLT(fin);
	}
}
