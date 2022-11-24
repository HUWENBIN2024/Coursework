//
// SctPointBrush.h
//
// The header file for Scattered Point Brush. 
//

#ifndef SCTPOINTBRUSH_H
#define SCTPOINTBRUSH_H

#include "ImpBrush.h"

class SctPointBrush : public ImpBrush
{
public:
	SctPointBrush(ImpressionistDoc* pDoc = NULL, char* name = NULL);

	void BrushBegin(const Point source, const Point target);
	void BrushMove(const Point source, const Point target);
	void BrushEnd(const Point source, const Point target);
	char* BrushName(void);
};

#endif