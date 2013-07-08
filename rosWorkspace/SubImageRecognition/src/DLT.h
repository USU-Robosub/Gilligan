#include <string>
#include <fstream>

const int ATTR=5;

class Sample
{
public:
	int iAttr[ATTR];
	int type;
};

class DLT {
	public:
	DLT(std::istream& fin);
		int Classify(const Sample& s);
	private:
		int splitId;
		int splitVal;
		DLT* lowSide;
		DLT* highSide;
};
