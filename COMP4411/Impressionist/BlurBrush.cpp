//
// BlurBrush.cpp
//
// The implementation of Blur Brush.
//

#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "blurbrush.h"
#include "math.h"

extern float frand();

BlurBrush::BlurBrush(ImpressionistDoc* pDoc, char* name) :
	ImpBrush(pDoc, name)
{
}

void BlurBrush::BrushBegin(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	int size = pDoc->getSize();



	glPointSize(1);

	BrushMove(source, target);
}

void BlurBrush::BrushMove(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	if (pDoc == NULL) {
		printf("PointBrush::BrushMove  document is NULL\n");
		return;
	}

	int size = pDoc->getSize();

	glBegin(GL_POINTS);

	for (int i = - size / 2; i < size / 2; i++) {
		for (int j = - size / 2; j < size / 2; j++) {
			//Gaussian Filter 3x3
			GLubyte color[3];
			double result[3] = { 0 };
			int var[3] = { -1, 0, 1 };
			double Gaussian[3] = { 0.25, 0.5, 0.25 };
			for (int ii = 0; ii < 3; ii++) {
				for (int jj = 0; jj < 3; jj++) {
					GLubyte* thisPixelColor = pDoc->GetOriginalPixel(source.x + i + var[ii], source.y + j + var[jj]);
					for (int rgb = 0; rgb < 3; rgb++) {
						result[rgb] += static_cast<double>(thisPixelColor[rgb])* Gaussian[ii] * Gaussian[jj];
					}
				}
			}
			for (int rgb = 0; rgb < 3; rgb++) {
				color[rgb] = static_cast<int>(result[rgb]);
			}
			glColor3ubv(color);
			glVertex2d(target.x + i, target.y + j);
		}
	}

	glEnd();
}

void BlurBrush::BrushEnd(const Point source, const Point target)
{
	// do nothing so far
}

