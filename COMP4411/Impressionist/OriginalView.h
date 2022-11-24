//
// originalview.h
//
// The header file for original view of the input images
//

#ifndef ORIGINALVIEW_H
#define ORIGINALVIEW_H

#include <FL/Fl.H>

#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <stdlib.h>
#include <iostream>
using namespace std;

#include "ImpBrush.h"

class ImpressionistDoc;

class OriginalView : public Fl_Gl_Window
{
public:
	OriginalView(int x, int y, int w, int h, const char *l);
	
	void draw();
	void refresh();

	void resizeWindow(int width, int height);
	int getWindowHeight() { return m_nWindowHeight; }
	int getWindowWidth() { return m_nWindowWidth; }

	void updateMarker(Point source);//BW3

	void setRgbScale(double red, double green, double blue);//For BW5

	void setFilter(int** source, int size);
	void setFilterSize(int size);
	int getFilterSize();
	string getMatrixText() { return matrixText; }
	void setMatrixText(string text) { matrixText = text; }
	void switchFilter(bool switchedOn);
	void setNormalize(bool normalize);
	void convolution(unsigned char* inputBitmap, int width, int height);//BW15

	ImpressionistDoc*	m_pDoc;

private:
	int	m_nWindowWidth, 
		m_nWindowHeight;

	bool markerUpdate;
	Point markerPos;//For BW3

	double rgbScale[3];//For BW5

	struct Filter {
		bool isApplied;
		bool normalize;
		int size;
		int** matrix;
	};	Filter* filter;//For BW15
	string matrixText;
	unsigned char test = '\0';
};

#endif