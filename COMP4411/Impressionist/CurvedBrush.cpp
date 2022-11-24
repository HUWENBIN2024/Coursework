#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "CurvedBrush.h"
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;

extern float frand();

float colorBlend(unsigned char* p0, unsigned char* p1) {
	return sqrt(pow(p0[0] - p1[0], 2) + pow(p0[1] - p1[1], 2) + pow(p0[2] - p1[2], 2));
};


void draw(int x, int y, int size)// draw a circle
{
	glBegin(GL_POINTS);
	size /= 2;
	for (int i = 0; i < 2 * size; i++) {
		for (int j = 0; j < sqrt(size * size - (size - i) * (size - i)); j++) {
			glVertex2d(x - size + i, y + j);
			if (j != 0)
				glVertex2d(x - size + i, y - j);
		}
	}
	glEnd();
}

void CurvedBrush::SetColor(GLubyte* color)
{
	ImpressionistDoc* pDoc = GetDocument();


	GLubyte color_[4];

	memcpy(color_, color, 3);
	color_[3] = GLubyte(255 * pDoc->m_pUI->getAlpha());

	glColor4ubv(color_);
}


int* CurvedBrush::getGradient(const Point source)
{
	int G_x[3][3] = { {-1,0,1},{-2,0,2},{-1,0,1} };
	int G_y[3][3] = { {-1,-2,-1},{0,0,0},{1,2,1} };
	int gradient[2] = { };
	bool isAnotherGradient = GetDocument()->m_pUI->isAnotherGradient;	// whether use another bitmap for gradient

	// convolution to get the gradient using sobel operator
	for (int j = 0; j < 3; ++j)
	{
		for (int i = 0; i < 3; ++i)
		{
			GLubyte color[3];
			if (isAnotherGradient)
				memcpy(color, GetDocument()->GetAnotherPixel(source.x + i - 1, source.y + j - 1), 3);
			else
				memcpy(color, GetDocument()->GetOriginalPixel(source.x + i - 1, source.y + j - 1), 3);
			int gary_level = (static_cast<int>(color[0]) * 0.299 + static_cast<int>(color[1]) * 0.587 + static_cast<int>(color[2]) * 0.114);
			gradient[0] += gary_level * G_x[j][i];
			gradient[1] += gary_level * G_y[j][i];
		}
	}
	return gradient;
}

GLubyte* getColor(unsigned char* pic, int x, int y, int w, int h) {
	x = max(0, min(x, w - 1));
	y = max(0, min(y, h - 1));  // ·ÀÖ¹Ô½½ç
	return pic + (y * w + x) * 3;
};

CurvedBrush::CurvedBrush(ImpressionistDoc* pDoc, char* name) :
	ImpBrush(pDoc, name)
{
}

void CurvedBrush::BrushBegin(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	int size = pDoc->getSize();

	glPointSize(1);

	BrushMove(source, target);
}

void CurvedBrush::BrushMove(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	if (pDoc == NULL) {
		printf("CurvedBrush::BrushMove  document is NULL\n");
		return;
	}

	int size = pDoc->getSize() ;

	BrushImplement(source.x, source.y, size);
}

void CurvedBrush::BrushEnd(const Point source, const Point target)
{
	// do nothing so far
}

void CurvedBrush::BrushImplement(int x, int y, int size)
{
	vector<Point> points= FindPoints(x, y, size);
	for (int i = 0; i < points.size(); i++)
	{
		Point point_0 = points[i];
		draw(x, y, size);

		if (i + 1 < points.size())
		{
			Point point_1 = points[i + 1];
			int x0 = point_0.x;
			int y0 = point_0.y;
			int x1 = point_1.x;
			int y1 = point_1.y;
			float d = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
			float t = atan(double(point_1.y - point_0.y) / (point_1.x - point_0.x));
			for (int i = 1; i < size; i += size / 3)
			{
				draw(x0 + i * cos(t), y0 + i * sin(t), size);
			}
		}
	}
}

vector<Point> CurvedBrush::FindPoints(int x, int y, int size)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* pUI = pDoc->m_pUI;
	vector<Point> points= { Point(x,y)};

	float d_x_from_last = 0, d_y_from_last = 0;
	int max_len = 16;
	int min_len = 4;
	float curvature = 0.3;
	const int w = pDoc->m_nWidth;
	const int h = pDoc->m_nHeight;

	GLubyte* ColorOrg = getColor(pDoc->m_ucBitmap, x, y, w, h);
	SetColor(ColorOrg);

	for (int i = 0; i < max_len; i++) {
		if (x < 0 || y < 0 || x >= w || y >= h)
		{
			return points;
		}


		GLubyte* color_from_bitmap= getColor(pDoc->m_ucBitmap, x, y, w, h);
		GLubyte* color_from_painting = getColor(pDoc->m_ucPainting, x, y, w, h);
		int* grad = getGradient(Point(x, y));
		float norm = sqrt(grad[1] * grad[1] + grad[0] * grad[0]);

		if (i > min_len && colorBlend(color_from_bitmap, color_from_painting) < colorBlend(color_from_bitmap, ColorOrg)) {
			return points;
		}
		else if (norm == 0) {
			return points;	 
		}

		double g_x = grad[0] / norm;
		double g_y = grad[1] / norm;
		double d_x = -g_y;
		double d_y = g_x;

		if (d_x_from_last * d_x + d_y_from_last * d_y < 0) {
			d_x = -d_x;
			d_y = -d_y;
		}

		d_x = curvature * d_x + (1 - curvature) * d_x_from_last;
		d_y = curvature * d_y + (1 - curvature) * d_y_from_last;
		float norm_2 = sqrt(d_x * d_x + d_y * d_y);
		d_x /= norm_2;
		d_y /= norm_2;  // unit

		x += size * d_x;
		y += size * d_y;
		d_x_from_last = d_x;
		d_y_from_last = d_y;

		points.emplace_back(x, y);
	}
	return points;

}
