#include "DLT.h"
#include "sqlite3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <ctime>
using namespace std;

//int DepthTest() {
//	list<Sample*> trainingSet;
//	/*for(int i = 0; i < 1000000; i++) {
//		trainingSet.push_back(genSample());
//	}*/
//	for(int depth = 0; depth < 9; depth++) {
//		printf("Creating Tree\n");
//		DLT myClassifier(trainingSet, depth);
//		printf("Testing\n");
//		int count = 0;
//		for(int i = 0; i < 100000; i++) {
//			//Sample* testSample = genSample();
//			int res = myClassifier.Classify(*testSample);
//			if(res == testSample->type)
//				count++;
//		}
//		printf("results(%d): %f/100\n",depth, count/1000.0);
//	}
//	return 0;
//}

void test(DLT tree, sqlite3 *db, int count)
{
	sqlite3_stmt *statement;
	sqlite3_prepare_v2(db, "select avgSat, avgBright, imageData.hue, imageData.saturation, imageData.brightness, imageData.tag from frameData join imageData where frameData.id=imageData.frame;", -1, &statement, 0);
	int cols = sqlite3_column_count(statement);
	int result = 0, progress=0, good=0;
	while(true)
	{
		result = sqlite3_step(statement);
		if(result == SQLITE_ROW)
		{
			if(progress%(count/100)==0)
			{
				printf("Testing tree: %d%%\n", (int)(100*(float)(progress+1)/(float)count));
			}
			Sample sample;
			for(int col = 0; col < cols; col++)
			{
				if(col==ATTR)
				{
					sample.type=atoi((char*)sqlite3_column_text(statement, col));
					continue;
				}
				sample.iAttr[col]=atoi((char*)sqlite3_column_text(statement, col));
			}
			if(tree.Classify(sample)==sample.type)
			{
				++good;
			}
			++progress;
		}
		else
		{
			break;   
		}
	}
	sqlite3_finalize(statement);
	printf("Results(%d): %f%%\n", DEPTH, (float)good*100/count);
	system("pause");
}

DLT SavingTest(Sample* data, int count) 
{
	printf("Training original tree\n");
	DLT original(data, count, ATTR , DEPTH);
	ofstream fout ("test.tree");
	printf("Saving tree\n");
	original.Save(fout);
	fout.close();
	return original;
}

int main(int argc, char* argv[]) 
{
	sqlite3 *db;
	sqlite3_stmt *statement;
	int count;
	Sample* data;
	if(sqlite3_open(argv[1], &db))
	{
		puts("Cannot open database\n");
		return 0;
	}
	if(sqlite3_prepare_v2(db, "select COUNT(*) from imageData;", -1, &statement, 0)==SQLITE_OK)
	{
		sqlite3_step(statement);
		count=atoi((char*)sqlite3_column_text(statement, 0));
		data=new Sample[count];
		sqlite3_prepare_v2(db, "select avgSat, avgBright, imageData.hue, imageData.saturation, imageData.brightness, imageData.tag from frameData join imageData where frameData.id=imageData.frame;", -1, &statement, 0);
		int cols = sqlite3_column_count(statement);
		int result = 0, progress=0;
		while(true)
		{
			result = sqlite3_step(statement);
			if(result == SQLITE_ROW)
			{
				if(progress%(count/100)==0)
				{
					printf("Reading Database: %d%%\n", (int)(100*(float)(progress+1)/(float)count));
				}
				Sample sample;
				for(int col = 0; col < cols; col++)
				{
					if(col==ATTR)
					{
						sample.type=atoi((char*)sqlite3_column_text(statement, col));
						continue;
					}
					sample.iAttr[col]=atoi((char*)sqlite3_column_text(statement, col));
				}
				data[progress]=sample;
				++progress;
			}
			else
			{
				break;   
			}
		}
		sqlite3_finalize(statement);
	}

	puts("Database read, starting tree generation");
	/*auto begin=time(0);
	auto tree=SavingTest(data, count);
	cout<<time(0)-begin<<endl;
	test(tree, db, count);*/
	test(SavingTest(data, count), db, count);
}
