#include <list>
#include <vector>
#include <string>
#include <fstream>

const int ATTR=5;
const int DEPTH=4;

class Sample
{
public:
	int iAttr[ATTR];
	int type;
};

class DLT {
	public:
		DLT(Sample* trainingSet, int count, int attr, int depth);
		DLT(std::istream& fin);
		int Classify(const Sample& s);
		void Save(std::ostream& fout);
	private:
		int splitId;
		int splitVal;
		DLT* lowSide;
		DLT* highSide;
};
