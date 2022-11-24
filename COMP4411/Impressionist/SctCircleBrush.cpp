//
// SctCircleBrush.cpp
//
// The implementation of Scattered Circle Brush.
//

#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "Sctcirclebrush.h"
#include "math.h"

extern float frand();

SctCircleBrush::SctCircleBrush(ImpressionistDoc* pDoc, char* name) :
	ImpBrush(pDoc, name)
{
}

void SctCircleBrush::BrushBegin(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	int size = pDoc->getSize();



	glPointSize(1);

	BrushMove(source, target);
}

void SctCircleBrush::BrushMove(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	if (pDoc == NULL) {
		printf("PointBrush::BrushMove  document is NULL\n");
		return;
	}

	int size = pDoc->getSize() / 2;//°ë¾¶

	glBegin(GL_POINTS);

	for (int i = 0; i < 4; i++) {
		int randx = 2 * size * frand();
		int randy = 2 * size * frand();
		Point SampleSourcePoint(source.x - size + randx, source.y - size + randy);
		Point SampleTargetPoint(target.x - size + randx, target.y - size + randy);

		SetColor(SampleSourcePoint);

		glVertex2d(SampleTargetPoint.x, SampleTargetPoint.y);

		for (int i = 0; i < 2 * size; i++) {
			for (int j = 0; j < sqrt(size * size - (size - i) * (size - i)); j++) {
				glVertex2d(SampleTargetPoint.x - size + i, SampleTargetPoint.y + j);
				if (j != 0) glVertex2d(SampleTargetPoint.x - size + i, SampleTargetPoint.y - j);
			}
		}

	}

	glEnd();
}

void SctCircleBrush::BrushEnd(const Point source, const Point target)
{
	// do nothing so far
}