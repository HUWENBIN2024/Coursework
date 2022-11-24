//
// SctCircleBrush.h
//
// The header file for Scattered Circle Brush. 
//

#ifndef SCTCIRCLEBRUSH_H
#define SCTCIRCLEBRUSH_H

#include "ImpBrush.h"

class SctCircleBrush : public ImpBrush
{
public:
	SctCircleBrush(ImpressionistDoc* pDoc = NULL, char* name = NULL);

	void BrushBegin(const Point source, const Point target);
	void BrushMove(const Point source, const Point target);
	void BrushEnd(const Point source, const Point target);
	char* BrushName(void);
};

#endif