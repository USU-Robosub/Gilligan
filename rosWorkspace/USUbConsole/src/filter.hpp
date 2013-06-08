class Filter {
public:
	Filter(int size);
	void Update(double value);
	double Value();

private:
	int _size;
	double* _buf;
	int _bufIndex;
};
