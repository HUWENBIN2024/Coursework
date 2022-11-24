//
// SctPointBrush.cpp
//
// The implementation of Scattered Point Brush.
//

#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "SctPointbrush.h"
#include "math.h"

extern int irand(int);

SctPointBrush::SctPointBrush(ImpressionistDoc* pDoc, char* name) :
	ImpBrush(pDoc, name)
{
}

void SctPointBrush::BrushBegin(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	int size = pDoc->getSize();



	glPointSize(1);

	BrushMove(source, target);
}

void SctPointBrush::BrushMove(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	if (pDoc == NULL) {
		printf("PointBrush::BrushMove  document is NULL\n");
		return;
	}

	int size = pDoc->getSize();

	glBegin(GL_POINTS);

	for (int i = 0; i < size/2; i++) {
		for (int j = 0; j < size/2; j++) {
			if (!(irand(5))) {
				Point SamplePoint(source.x + i, source.y + j);
				SetColor(SamplePoint);
				glVertex2d(target.x + i, target.y + j);
			}
			if (!(irand(5))) {
				Point SamplePoint(source.x - i, source.y + j);
				SetColor(SamplePoint);
				glVertex2d(target.x - i, target.y + j);
			}
			if (!(irand(5))) {
				Point SamplePoint(source.x + i, source.y - j);
				SetColor(SamplePoint);
				glVertex2d(target.x + i, target.y - j);
			}
			if (!(irand(5))) {
				Point SamplePoint(source.x - i, source.y - j);
				SetColor(SamplePoint);
				glVertex2d(target.x - i, target.y - j);
			}
		}
	}

	glEnd();
}

void SctPointBrush::BrushEnd(const Point source, const Point target)
{
	// do nothing so far
}

