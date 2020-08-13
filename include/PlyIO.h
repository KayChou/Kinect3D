#ifndef PLYIO_H_
#define PLYIO_H_
#include <stdio.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "FIFO.h"

static const unsigned int MAX_COMMENT_SIZE = 256;

template <class T>
void bigLittleEndianSwap (T * v, unsigned int numOfElements);

bool parasePlyHeader(std::ifstream &in, const char* filename, unsigned int &vn, unsigned int &fn, 
                   PLYFormat &format, unsigned int &numVertProperties, bool &hasColor, int &headerSize);
bool readPlyFile(const char* filename, std::vector<Point3f> &verts, std::vector<RGB> &colors);
void savePlyFile(const char* filename, std::vector<Point3f> vertices, bool BINARY=true, std::vector<RGB> colors=std::vector<RGB>());

#endif
