all: DLTGen sqlite3 DLT

DLTGen: DLTTest.cpp sqlite3.o DLT.o
	gcc-4.8 -std=c++11 DLTTest.cpp sqlite3.o DLT.o -o DLTGen

sqlite3.o: sqlite3.c sqlite3.h
	gcc-4.8 -c sqlite3.c -o sqlite3.o

DLT.o: DLT.h DLT.cpp
	gcc-4.8 -std=c++11 DLT.cpp -o DLT.o