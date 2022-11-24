#ifndef MOSAIC_H
#define MOSAIC_H

#include <iostream>
using namespace std;

class ImpressionistDoc;

class Mosaic {
public:
	Mosaic(ImpressionistDoc* pDoc);

	unsigned char* getResultBitmap();

private:
	ImpressionistDoc* m_pDoc;

	static const int macroSize = 64; //How many thumbnail images in a row
	static const int microSize = 8; //Size of thumbnail images
	//static const int testCount = 100; //How many images do we check
	//static const int weightCos = 1, weightColor = 1; //weight of 2 measures of similarity; not normalized but this is OK.

	unsigned char*** orgImages[macroSize][macroSize];
	unsigned char*** newImages[macroSize][macroSize];

	//Helper Functions
	unsigned char*** getSimilarImage(unsigned char*** orgImage);
	double getSimilarityColor(unsigned char*** orgImage, unsigned char*** testImage);
	//double getSimilarityCos(unsigned char*** orgImage, unsigned char*** testImage);
	//unsigned char*** getSimilarImageHash(unsigned char*** orgImage);

};

#endif