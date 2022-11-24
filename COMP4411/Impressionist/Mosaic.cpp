#include "Mosaic.h"
#include "ImpressionistDoc.h"
#include "Bitmap.h"
#include <string>
#include <cmath>
#include <fstream>

Mosaic::Mosaic(ImpressionistDoc* pDoc) : m_pDoc(pDoc)
{
	for (int row = 0; row < macroSize; row++) {
		for (int col = 0; col < macroSize; col++) {
			unsigned char*** result = new unsigned char**[microSize];
			for (int i = 0; i < microSize; i++) {
				result[i] = new unsigned char*[microSize];
				for (int j = 0; j < microSize; j++) {
					result[i][j] = new unsigned char[3];
					for (int rgb = 0; rgb < 3; rgb++) {
						result[i][j][rgb] = m_pDoc->m_ucBitmap[3*(512 * microSize * row + 512 * i + microSize * col + j)+rgb];
					}
				}
			}
			orgImages[row][col] = result;
		}
	}
}

unsigned char* Mosaic::getResultBitmap()
{
	for (int row = 0; row < macroSize; row++) {
		cout << row << endl;
		for (int col = 0; col < macroSize; col++) {
			newImages[row][col] = getSimilarImage(orgImages[row][col]);
		}
	}

	unsigned char* result = new unsigned char[3 * 512 * 512];
	for (int row = 0; row < macroSize; row++) {
		for (int col = 0; col < macroSize; col++) {
			for (int i = 0; i < microSize; i++) {
				for (int j = 0; j < microSize; j++) {
					for (int rgb = 0; rgb < 3; rgb++) {
						result[3 * (512 * microSize * row + 512 * i + microSize * col + j) + rgb] = newImages[row][col][i][j][rgb];
					}
				}
			}
		}
	}

	return result;
}

unsigned char*** Mosaic::getSimilarImage(unsigned char*** orgImage)
{
	double similarity = 999999999;
	unsigned char*** result = nullptr;
	for (int i = 1; i <= 1500; i+=(14 + 2*rand()/RAND_MAX)) {
		string filename;
		if (i <= 9) { filename = "./Thumbnail8/C000" + to_string(i) + ".bmp"; }
		else if (i <= 99) { filename = "./Thumbnail8/C00" + to_string(i) + ".bmp"; }
		else if (i <= 999) { filename = "./Thumbnail8/C0" + to_string(i) + ".bmp"; }
		else { filename = "./Thumbnail8/C" + to_string(i) + ".bmp"; }
		char* fname = new char[strlen(filename.c_str()) + 1];
		strcpy(fname, strdup(filename.c_str()));
		fname[filename.size()] = '\0';
		int size = 8;
		unsigned char* sourceBitmap = readBMP(fname, size, size);
		delete[] fname;

		unsigned char*** testImage = new unsigned char**[size];
		for (int i = 0; i < size; i++) {
			testImage[i] = new unsigned char* [size];
			for (int j = 0; j < size; j++) {
				testImage[i][j] = new unsigned char[3];
				for (int rgb = 0; rgb < 3; rgb++) {
					testImage[i][j][rgb] = sourceBitmap[3 * (i * size + j) + rgb];
				}
			}
		}

		double temp = getSimilarityColor(orgImage, testImage);
		if (temp < similarity) {
			similarity = temp;
			if (result) {
				for (int i = 0; i < 8; i++) {
					for (int j = 0; j < 8; j++) {
						delete[] result[i][j];
					}
					delete[] result[i];
				}
				delete[] result;
			}
			result = testImage;
		}
		else {
			if (testImage) {
				for (int i = 0; i < 8; i++) {
					for (int j = 0; j < 8; j++) {
						delete[] testImage[i][j];
					}
					delete[] testImage[i];
				}
				delete[] testImage;
				testImage = nullptr;
			}
		}

	}
	return result;
}

double Mosaic::getSimilarityColor(unsigned char*** orgImage, unsigned char*** testImage)
{
	double oRGB[3], tRGB[3];
	for (int rgb = 0; rgb < 3; rgb++) {
		int orgSum = 0, testSum = 0;
		for (int i = 0; i < microSize; i++) {
			for (int j = 0; j < microSize; j++) {
				orgSum += (int)orgImage[i][j][rgb];
				testSum += (int)testImage[i][j][rgb];
			}
		}
		oRGB[rgb] = (double)(orgSum) / (microSize * microSize);
		tRGB[rgb] = (double)(testSum) / (microSize * microSize);
	}

	return sqrt(pow(oRGB[0] - tRGB[0], 2) + pow(oRGB[1] - tRGB[1], 2) + pow(oRGB[2] - tRGB[2], 2));
}

/*double Mosaic::getSimilarityCos(unsigned char*** orgImage, unsigned char*** testImage)
{
	double dotProduct = 0;
	double orgNormSq = 0;
	double testNormSq = 0;
	for (int i = 0; i < microSize; i++) {
		for (int j = 0; j < microSize; j++) {
			dotProduct += ((double)((int)orgImage[i][j][0] + (int)orgImage[i][j][1] + (int)orgImage[i][j][2]) / 3) *
						  ((double)((int)testImage[i][j][0] + (int)testImage[i][j][1] + (int)testImage[i][j][2]) / 3);
			 orgNormSq += ((double)((int)orgImage[i][j][0] + (int)orgImage[i][j][1] + (int)orgImage[i][j][2]) / 3) *
						  ((double)((int)orgImage[i][j][0] + (int)orgImage[i][j][1] + (int)orgImage[i][j][2]) / 3);
			testNormSq += ((double)((int)testImage[i][j][0] + (int)testImage[i][j][1] + (int)testImage[i][j][2]) / 3) *
						  ((double)((int)testImage[i][j][0] + (int)testImage[i][j][1] + (int)testImage[i][j][2]) / 3);
		}
	}
	return (1 - dotProduct / sqrt(orgNormSq * testNormSq)) / 2;
}*/

/*unsigned char*** Mosaic::getSimilarImageHash(unsigned char*** orgImage)
{
	int sum = 0;
	for (int i = 0; i < microSize; i++) {
		for (int j = 0; j < microSize; j++) {
			for (int rgb = 0; rgb < 3; rgb++) {
				sum += (int)orgImage[i][j][rgb];
			}
		}
	}
	double avg = ((double)sum) / (microSize * microSize * 3);

	int hash[64] = { 0 }; int index = 0;
	for (int row = 0; row < 8; row++) {
		for (int col = 0; col < 8; col++) {
			int PixelSum = 0;
			for (int i = 0; i < microSize / 8; i++) {
				for (int j = 0; j < microSize / 8; j++) {
					for (int rgb = 0; rgb < 3; rgb++) {
						PixelSum += (int)orgImage[microSize / 8 * row + i][microSize / 8 * col + j][rgb];
					}
				}
			}
			hash[index] = (double)PixelSum / (3 * microSize * microSize / 64) > avg ? 1 : 0;
			index++;
		}
	}

	int minhdist = 65; int fileindex = 0;
	for (int i = 1; i < testCount; i++) {
		string filename;
		if (i <= 9) { filename = "./Hash/C000" + to_string(i) + ".txt"; }
		else if (i <= 99) { filename = "./Hash/C00" + to_string(i) + ".txt"; }
		else if (i <= 999) { filename = "./Hash/C0" + to_string(i) + ".txt"; }
		else { filename = "./Hash/C" + to_string(i) + ".txt"; }
		ifstream ifs(filename);
		int testHash[64] = { 0 };
		int hdist = 0;
		for (int i = 0; i < 64; i++) {
			ifs >> testHash[i];
			if (testHash[i] != hash[i]) hdist++;
		}
		if (hdist <= 5) {
			fileindex = i;
			minhdist = hdist;
			break;
		}
		if (hdist < minhdist) {
			fileindex = i;
			minhdist = hdist;
		}
	}

	string filename;
	if (fileindex <= 9) { filename = "./Thumbnail" + to_string(microSize) + "/C000" + to_string(fileindex) + ".bmp"; }
	else if (fileindex <= 99) { filename = "./Thumbnail" + to_string(microSize) + "/C00" + to_string(fileindex) + ".bmp"; }
	else if (fileindex <= 999) { filename = "./Thumbnail" + to_string(microSize) + "/C0" + to_string(fileindex) + ".bmp"; }
	else { filename = "./Thumbnail" + to_string(microSize) + "/C" + to_string(fileindex) + ".bmp"; }
	char* fname = new char[strlen(filename.c_str()) + 1];
	strcpy(fname, strdup(filename.c_str()));
	fname[filename.size()] = '\0';
	int size = microSize;
	unsigned char* resultBitmap = readBMP(fname, size, size);
	delete[] fname;

	unsigned char*** resImage = new unsigned char** [size];
	for (int i = 0; i < size; i++) {
		resImage[i] = new unsigned char* [size];
		for (int j = 0; j < size; j++) {
			resImage[i][j] = new unsigned char[3];
			for (int rgb = 0; rgb < 3; rgb++) {
				resImage[i][j][rgb] = resultBitmap[3 * (i * size + j) + rgb];
			}
		}
	}
	delete[] resultBitmap;
	return resImage;
}*/