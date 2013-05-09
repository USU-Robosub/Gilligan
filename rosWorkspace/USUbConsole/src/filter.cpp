#include "filter.hpp"

Filter::Filter(int size) {
	_size = size;
	_buf = new double[size];
	_bufIndex=0;
}

void Filter::Update(double value) {
	_buf[_bufIndex] = value;
	_bufIndex++;
	_bufIndex %= _size;
}

double Filter::Value() {
	double sum = 0;
	for(int i = 0; i < _size; i++) {
		sum += _buf[i];	
	}
	return sum / _size;
}

