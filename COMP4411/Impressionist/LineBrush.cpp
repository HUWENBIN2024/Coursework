//
// LineBrush.cpp
//
// The implementation of Point Brush. It is a kind of ImpBrush. All your brush implementations
// will look like the file with the different GL primitive calls.
//

#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "LineBrush.h"
#include "math.h"
#include <iostream>
#define PI 3.14159265

using namespace std;

extern float frand();
int last_pixel_x = 0;
int last_pixel_y = 0;

int* LineBrush::getGradient(const Point source)
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
			int gary_level =( static_cast<int>(color[0]) * 0.299 + static_cast<int>(color[1]) * 0.587 + static_cast<int>(color[2]) * 0.114 );
			gradient[0] += gary_level * G_x[j][i];
			gradient[1] += gary_level * G_y[j][i];
		}
	}
	return gradient;
}


LineBrush::LineBrush(ImpressionistDoc* pDoc, char* name) :
	ImpBrush(pDoc, name)
{
}

void LineBrush::BrushBegin(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	int size = pDoc->getSize();
	glPointSize(1);

	BrushMove(source, target);
}

void LineBrush::BrushMove(const Point source, const Point target)
{
	ImpressionistDoc* pDoc = GetDocument();
	ImpressionistUI* dlg = pDoc->m_pUI;

	if (pDoc == NULL) {
		printf("LineBrush::BrushMove  document is NULL\n");
		return;
	}

	int size = pDoc->getSize();							// length of the rec
	int size_right = size / 2;
	int size_left = size / 2;
	double LineWidth = double(pDoc->getLineWidth());	// width of the rec

	int* grad = getGradient(source);
	switch (line_brush_type)
	{
	case SLIDER_RIGHT_MOUSE:
		LineAngle = double(pDoc->getLineAngle()) * (PI / 180);
		break;
	case GRADIENT:
		LineAngle = atan(-(double)grad[0] / (double)grad[1]);
		break;
	case BRUSH_DIRECTION:
		LineAngle = atan(double(target.y - last_pixel_y) / double(target.x - last_pixel_x));
		last_pixel_x = target.x;
		last_pixel_y = target.y;
		break;
	default:
		// LineAngle = double(pDoc->getLineAngle()) * (PI / 180);
		break;
	}

	double sin_theta = sin(LineAngle);
	double cos_theta = cos(LineAngle);
	double tan_theta = tan(LineAngle);

	if (pDoc->m_ucEdgeBitmap && dlg->isEdgeClipping)
	{
		for (int i = 0; i < (size / 2) * cos_theta; i++)
		{
			int y = i * tan_theta + target.y;
			int x = target.x + i;
			GLubyte color[3];
			memcpy(color, pDoc->GetEdgePixel(x, y), 3);
			if ((int)color[0] == 255 && (int)color[1] == 255 && (int)color[2] == 255)
			{
				size_right = i / cos_theta;
				break;
			}
		}
		for (int i = 0; i < (size / 2) * cos_theta; i++)
		{
			int y = -i * tan_theta + target.y;
			int x = target.x - i;
			GLubyte color[3];
			memcpy(color, pDoc->GetEdgePixel(x, y), 3);
			if ((int)color[0] == 255 && (int)color[1] == 255 && (int)color[2] == 255)
			{
				size_left = i / cos_theta;
				break;
			}
		}
	}
	
	glBegin(GL_POLYGON);
		SetColor(source);
		glVertex2d(target.x + (size_right) * cos_theta - (LineWidth / 2) * sin_theta, target.y + (size_right) * sin_theta + (LineWidth / 2) * cos_theta);
		glVertex2d(target.x + (size_right) * cos_theta + (LineWidth / 2) * sin_theta, target.y + (size_right) * sin_theta - (LineWidth / 2) * cos_theta);
		glVertex2d(target.x - (size_left) * cos_theta + (LineWidth / 2) * sin_theta, target.y - (size_left) * sin_theta - (LineWidth / 2) * cos_theta);
		glVertex2d(target.x - (size_left) * cos_theta - (LineWidth / 2) * sin_theta, target.y - (size_left) * sin_theta + (LineWidth / 2) * cos_theta);
	glEnd();
}

void LineBrush::BrushEnd(const Point source, const Point target)
{
	// do nothing so far
}

