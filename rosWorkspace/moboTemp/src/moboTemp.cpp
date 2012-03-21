#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

using namespace std;

int main (void)
{
	string value;

	system("sensors | grep -o '+.*C\ '> temp.out");

	ifstream temperatureFile;

	temperatureFile.open("temp.out");

	while (!temperatureFile.eof())
	{
		temperatureFile >> value;
		cout << "value == " << value << endl;
	}

	temperatureFile.close();

	return 0;
}
