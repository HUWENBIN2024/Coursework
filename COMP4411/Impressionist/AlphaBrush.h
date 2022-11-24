//
// AlphaBrush.h
//
// The header file for Alpha Mapped Brush. 
//

#ifndef ALPHABRUSH_H
#define ALPHABRUSH_H

#include "ImpBrush.h"

class AlphaBrush : public ImpBrush
{
public:
	AlphaBrush(ImpressionistDoc* pDoc = NULL, char* name = NULL);

	void BrushBegin(const Point source, const Point target);
	void BrushMove(const Point source, const Point target);
	void BrushEnd(const Point source, const Point target);
	char* BrushName(void);
};

#endif