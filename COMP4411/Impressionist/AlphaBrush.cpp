//
// PointBrush.cpp
//
// The implementation of Point Brush. It is a kind of ImpBrush. All your brush implementations
// will look like the file with the different GL primitive calls.
//

#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "alphabrush.h"

extern float frand();

AlphaBrush::AlphaBrush(ImpressionistDoc* pDoc, char* name) :
	ImpBrush(pDoc, name)
{
}

void AlphaBrush::BrushBegin(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	if (pDoc == NULL) {
		printf("PointBrush::BrushMove  document is NULL\n");
		return;
	}
	ImpressionistUI* dlg = pDoc->m_pUI;
	if (pDoc->m_ucAlphaMap == NULL) {
		fl_message("Please load in an alpha map.");
		return;
	}

	glPointSize(1);

	glBegin(GL_POINTS);
	for (int i = 0; i < pDoc->m_nPaintHeight; i++) {
		for (int j = 0; j < pDoc->m_nPaintWidth; j++) {
			GLubyte color[4];
			Point pointOnAlphaMap(j, i);
			memcpy(color, pDoc->GetOriginalPixel(source), 3);
			color[3] = pDoc->getAlphaFromMap(pointOnAlphaMap);
			glColor4ubv(color);

			glVertex2d(j, i + dlg->m_origView->getWindowHeight() - pDoc->m_nPaintHeight);
		}
	}

	glEnd();
}

void AlphaBrush::BrushMove(const Point source, const Point target)
{
	// do nothing
}

void AlphaBrush::BrushEnd(const Point source, const Point target)
{
	// do nothing so far
}