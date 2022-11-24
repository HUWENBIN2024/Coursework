//
// CurvedBrush.h
//
// The header file for Point Brush. 
//

#ifndef CURVEDBRUCH_H
#define CURVEDBRUSH_H

#include "ImpBrush.h"
#include <vector>

class CurvedBrush : public ImpBrush
{
public:
	CurvedBrush(ImpressionistDoc* pDoc = NULL, char* name = NULL);

	void BrushBegin(const Point source, const Point target);
	void BrushMove(const Point source, const Point target);
	void BrushEnd(const Point source, const Point target);
	char* BrushName(void);
	vector<Point> CurvedBrush::FindPoints(int x, int y, int size);
	void BrushImplement(int x, int y, int size);
	int* getGradient(const Point source);
	void SetColor(GLubyte* color);

};

#endif