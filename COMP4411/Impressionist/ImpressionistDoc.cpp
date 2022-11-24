// 
// impressionistDoc.cpp
//
// It basically maintain the bitmap for answering the color query from the brush.
// It also acts as the bridge between brushes and UI (including views)
//

#include <FL/fl_ask.H>

#include "impressionistDoc.h"
#include "impressionistUI.h"

#include "ImpBrush.h"

// Include individual brush headers here.
#include "PointBrush.h"
#include "CircleBrush.h"
#include "SctPointBrush.h"
#include "SctCircleBrush.h"
#include "BlurBrush.h"
#include "LineBrush.h"
#include "SctLineBrush.h"
#include "AlphaBrush.h"
#include "TriangleBrush.h"
#include "CurvedBrush.h"
#include <iostream>
using namespace std;

#define DESTROY(p)	{  if ((p)!=NULL) {delete [] p; p=NULL; } }

ImpressionistDoc::ImpressionistDoc()
{
	// Set NULL image name as init. 
	m_imageName[0] = '\0';

	m_nWidth = -1;
	m_ucBitmap = NULL;
	m_ucBitmapScaled = NULL;
	m_ucPainting = NULL;
	m_ucPaintingOrg = NULL;
	m_ucAlphaMap = NULL;
	m_pMosaic = NULL;

	m_ucOriginBitmap = NULL;
	m_ucEdgeBitmap = NULL;
	m_ucAnotherBitmap = NULL;

	m_ucLastBitmap = NULL;
	m_ucDissolveBitmap = NULL;

	// create one instance of each brush
	ImpBrush::c_nBrushCount = NUM_BRUSH_TYPE;
	ImpBrush::c_pBrushes = new ImpBrush * [ImpBrush::c_nBrushCount];

	ImpBrush::c_pBrushes[BRUSH_POINTS] = new PointBrush(this, "Points");

	// Note: You should implement these 5 brushes.  They are set the same (PointBrush) for now
	ImpBrush::c_pBrushes[BRUSH_LINES]				
		= new LineBrush( this, "Lines" );
	ImpBrush::c_pBrushes[BRUSH_CIRCLES]				
		= new CircleBrush( this, "Circles" );
	ImpBrush::c_pBrushes[BRUSH_SCATTERED_POINTS]	
		= new SctPointBrush( this, "Scattered Points" );
	ImpBrush::c_pBrushes[BRUSH_SCATTERED_LINES]		
		= new SctLineBrush( this, "Scattered Lines" );
	ImpBrush::c_pBrushes[BRUSH_SCATTERED_CIRCLES]	
		= new SctCircleBrush( this, "Scattered Circles" );
	ImpBrush::c_pBrushes[BRUSH_BLUR]
		= new BlurBrush(this, "Blur");
	ImpBrush::c_pBrushes[BRUSH_ALPHA]
		= new AlphaBrush(this, "Alpha Mapped");
	ImpBrush::c_pBrushes[BRUSH_TRIANGLE]
		= new TriangleBrush(this, "Triangle Brush");
	ImpBrush::c_pBrushes[BRUSH_CRUVED]
		= new CurvedBrush(this, "Curved Brush");

	// make one of the brushes current
	m_pCurrentBrush	= ImpBrush::c_pBrushes[0];

}


//---------------------------------------------------------
// Set the current UI 
//---------------------------------------------------------
void ImpressionistDoc::setUI(ImpressionistUI* ui) 
{
	m_pUI	= ui;
}

//---------------------------------------------------------
// Returns the active picture/painting name
//---------------------------------------------------------
char* ImpressionistDoc::getImageName() 
{
	return m_imageName;
}

void ImpressionistDoc::setStrokeDirectionType(StrokeDirectionType type)
{
	m_pCurrentBrush->line_brush_type = type;
}

void ImpressionistDoc::generateMosaic()
{
	if (m_nWidth != 512) {
		fl_message("Please load a picture from the specified directory. (../Mosaic/Cxxxx.bmp)");
		if (m_pMosaic) {
			delete m_pMosaic;
			m_pMosaic = NULL;
		}
		return;
	}

	m_pMosaic = new Mosaic(this);
	if (m_ucPainting) delete[] m_ucPainting;
	if (m_ucPaintingOrg) delete[] m_ucPaintingOrg;
	m_ucPainting = m_pMosaic->getResultBitmap();
	m_ucPaintingOrg = m_ucPainting;
}

//---------------------------------------------------------
// Called by the UI when the user changes the brush type.
// type: one of the defined brush types.
//---------------------------------------------------------
void ImpressionistDoc::setBrushType(int type)
{
	m_pCurrentBrush	= ImpBrush::c_pBrushes[type];
}


//---------------------------------------------------------
// Returns the size of the brush.
//---------------------------------------------------------
int ImpressionistDoc::getSize()
{
	return m_pUI->getSize();
}

void ImpressionistDoc::setSize(int size)
{
	m_pUI->setSize(size);
}

int ImpressionistDoc::getLineWidth()
{
	return m_pUI->getLineWidth();
}

int ImpressionistDoc::getLineAngle()
{
	return m_pUI->getLineAngle();
}

void ImpressionistDoc::setLineAngle(double angle)
{
	m_pCurrentBrush->LineAngle = angle;
	m_pUI->setLineAngle(angle);
}

//---------------------------------------------------------
// Returns the alpha value.
//---------------------------------------------------------
double ImpressionistDoc::getAlpha()
{
	return m_pUI->getAlpha();
}

unsigned char ImpressionistDoc::getAlphaFromMap(Point source)
{
	return m_ucAlphaMap[3 * m_nWidth * source.y + 3 * source.x];
}

double ImpressionistDoc::getBgdVal() {
	return m_pUI->getBgdVal();
}

//---------------------------------------------------------
// Load the specified image
// This is called by the UI when the load image button is 
// pressed.
//---------------------------------------------------------
int ImpressionistDoc::loadImage(char *iname) 
{
	// try to open the image to read
	unsigned char*	data;
	int				width, 
					height;

	if ( (data=readBMP(iname, width, height))==NULL ) 
	{
		fl_alert("Can't load bitmap file");
		return 0;
	}

	// reflect the fact of loading the new image
	m_nWidth		= width;
	m_nPaintWidth	= width;
	m_nHeight		= height;
	m_nPaintHeight	= height;

	// release old storage
	// if ( m_ucBitmap ) delete [] m_ucBitmap;
	if ( m_ucBitmapScaled ) delete[] m_ucBitmapScaled;
	if ( m_ucPainting ) delete [] m_ucPainting;
	if ( m_ucPaintingOrg ) delete [] m_ucPaintingOrg;

	if ( m_ucOriginBitmap ) delete[] m_ucOriginBitmap;

	if (m_ucEdgeBitmap) delete[] m_ucEdgeBitmap;
	m_ucEdgeBitmap = NULL;

	m_ucOriginBitmap = data;

	m_ucBitmap = m_ucOriginBitmap;

	// m_ucOriginBitmap = new unsigned char[3 * width * height];			// store bitmap to OriginBitmap
	// memcpy(m_ucOriginBitmap, m_ucBitmap, 3 * width * height);

	m_ucBitmapScaled = new unsigned char[3 * width * height];
	memcpy(m_ucBitmapScaled, m_ucBitmap, 3*width*height);

	m_ucAnotherBitmap = new unsigned char[3 * width * height];
	memcpy(m_ucAnotherBitmap, m_ucBitmap, 3 * width * height);

	// allocate space for draw view
	m_ucPainting = new unsigned char [width*height*3];
	memset(m_ucPainting, 0, width*height*3);
	m_ucPaintingOrg = new unsigned char[width * height * 3];//BW9
	memset(m_ucPaintingOrg, 0, width * height * 3);//BW9

	m_pUI->m_mainWindow->resize(m_pUI->m_mainWindow->x(), 
								m_pUI->m_mainWindow->y(), 
								width*2, 
								height+25);

	// display it on origView
	m_pUI->m_origView->resizeWindow(width, height);	
	m_pUI->m_origView->refresh();

	// refresh paint view as well
	m_pUI->m_paintView->resizeWindow(width, height);	
	m_pUI->m_paintView->refresh();


	return 1;
}

int ImpressionistDoc::loadEdgeImage(char* iname)
{
	// try to open the image to read
	unsigned char* data;
	int				width,
					height;
	data = readBMP(iname, width, height);
	if (width != m_nWidth || height != m_nHeight)
	{
		fl_alert("Different Dimension");
		return 0;
	}
	
	// release old storage
	if (m_ucEdgeBitmap) delete[] m_ucEdgeBitmap;	

	m_ucEdgeBitmap = data;

	return 1;
}

int ImpressionistDoc::loadAnotherImage(char* iname)
{
	// try to open the image to read
	unsigned char* data;
	int				width,
					height;
	data = readBMP(iname, width, height);
	if (width != m_nWidth || height != m_nHeight)
	{
		fl_alert("Different Dimension");
		return 0;
	}

	// release old storage
	if (m_ucAnotherBitmap) delete[] m_ucAnotherBitmap;

	m_ucAnotherBitmap = data;

	return 1;
		}

// display images(origin, edge, another)
void   ImpressionistDoc::displayOriginImage()
{
	// display it on origView
	m_ucBitmap = m_ucOriginBitmap;
	m_pUI->m_origView->refresh();
	
}
void   ImpressionistDoc::displayEdgeImage()
{
	unsigned char* temp = m_ucBitmap;

	if (m_ucEdgeBitmap)
	{
		m_ucBitmap = m_ucEdgeBitmap;
		m_pUI->m_origView->refresh();
		}
	else
	{
		displayEdgeImageFromThreshold();
	}
}
void   ImpressionistDoc::displayAnotherImage()
{
	m_ucBitmap = m_ucAnotherBitmap;
	m_pUI->m_origView->refresh();
}

void   ImpressionistDoc::displayEdgeImageFromThreshold()
{
	egdeDetection();
	m_ucBitmap = m_ucEdgeBitmap;
	m_pUI->m_origView->refresh();
}



//BW11
int ImpressionistDoc::loadAlphaMap(char* iname)
{
	// try to open the image to read
	unsigned char* data;
	int				width,
		height;

	if ((data = readBMP(iname, width, height)) == NULL)
	{
		if(m_ucAlphaMap) delete[] m_ucAlphaMap;  //
		m_ucAlphaMap = NULL;
		return 0;
	}

	if (width != m_nWidth || height != m_nHeight) {
		fl_message("Alpha Map Size Does Not Match the Original Image!");
		if(m_ucAlphaMap) delete[] m_ucAlphaMap;
		m_ucAlphaMap = NULL;
		return 0;
	}

	if (m_ucAlphaMap) delete[] m_ucAlphaMap;
	m_ucAlphaMap = new unsigned char[3 * m_nWidth * m_nHeight];
	memcpy(m_ucAlphaMap, data, 3 * m_nWidth * m_nHeight);
	return 1;
}

//BW10
int ImpressionistDoc::changeImage(char* iname)
{
	// try to open the image to read
	unsigned char* data;
	int		       width, prevWidth,
		height, prevHeight;

	if ((data = readBMP(iname, width, height)) == NULL)
	{
		fl_alert("Can't load bitmap file");
		return 0;
	}

	prevWidth = m_nWidth;
	prevHeight = m_nHeight;

	// reflect the fact of loading the new image
	m_nWidth = width;
	m_nPaintWidth = width;
	m_nHeight = height;
	m_nPaintHeight = height;

	// release old storage
	if (m_ucOriginBitmap) delete[] m_ucOriginBitmap;
	if (m_ucBitmapScaled) delete[] m_ucBitmapScaled;
	if (m_ucAnotherBitmap) delete[] m_ucAnotherBitmap;

	m_ucOriginBitmap = data;
	m_ucBitmap = m_ucOriginBitmap;
	m_ucBitmapScaled = new unsigned char[3 * width * height];

	m_ucAnotherBitmap = new unsigned char[width * height * 3];
	memcpy(m_ucAnotherBitmap, m_ucBitmap, width * height * 3);

	// update PaintView; BW10
	unsigned char* temp = new unsigned char[width * height * 3];
	memset(temp, 0, width * height * 3);
	for (int i = 0; i < min(height, prevHeight); i++) {
		for (int j = 0; j < 3 * min(width, prevWidth); j++) {
			temp[3 * width * i + j] = m_ucPainting[3 * prevWidth * i + j];
		}
	}
	if(m_ucPainting) delete[] m_ucPainting;
	m_ucPainting = temp;

	unsigned char* tempOrg = new unsigned char[width * height * 3];
	memset(tempOrg, 0, width * height * 3);
	for (int i = 0; i < min(height, prevHeight); i++) {
		for (int j = 0; j < 3 * min(width, prevWidth); j++) {
			tempOrg[3 * width * i + j] = m_ucPaintingOrg[3 * prevWidth * i + j];
		}
	}
	if(m_ucPaintingOrg) delete[] m_ucPaintingOrg;
	m_ucPaintingOrg = tempOrg;
	//May need to update attributes of OriginalView and PaintView. But this is not done here.


	m_pUI->m_mainWindow->resize(m_pUI->m_mainWindow->x(),
		m_pUI->m_mainWindow->y(),
		width * 2,
		height + 25);

	// display it on origView
	m_pUI->m_origView->resizeWindow(width, height);
	m_pUI->m_origView->refresh();

	// refresh paint view as well
	m_pUI->m_paintView->resizeWindow(width, height);
	m_pUI->m_paintView->refresh();


	return 1;
}


//----------------------------------------------------------------
// Save the specified image
// This is called by the UI when the save image menu button is 
// pressed.
//----------------------------------------------------------------
int ImpressionistDoc::saveImage(char *iname) 
{

	writeBMP(iname, m_nPaintWidth, m_nPaintHeight, m_ucPainting);

	return 1;
}

//BW4
void ImpressionistDoc::swap()
{
	if ((!m_ucBitmap) || (!m_ucPainting)) {
		fl_message("Cannot Swap!");
		return;
	}
	int size = 3 * m_nHeight * m_nWidth; //前提：OriginalView和PaintView的Size一样
	unsigned char* temp = new unsigned char[size];
	memcpy(temp, m_ucBitmapScaled, size);
	memcpy(m_ucBitmap, m_ucPainting, size);
	memcpy(m_ucPainting, temp, size);
	memcpy(m_ucPaintingOrg, temp, size); //交换后的PaintView均视作已被刷子刷过
	m_pUI->m_origView->refresh();
	m_pUI->m_paintView->refresh();
}

//----------------------------------------------------------------
// Clear the drawing canvas
// This is called by the UI when the clear canvas menu item is 
// chosen
//-----------------------------------------------------------------
int ImpressionistDoc::clearCanvas() 
{

	
	// Release old storage

		if (m_ucPainting) delete [] m_ucPainting;
		if (m_ucPaintingOrg) delete [] m_ucPaintingOrg;

		// allocate space for draw view
		m_ucPainting	= new unsigned char [m_nPaintWidth*m_nPaintHeight*3];
		memset(m_ucPainting, 0, m_nPaintWidth*m_nPaintHeight*3);
		m_ucPaintingOrg = new unsigned char[m_nPaintWidth * m_nPaintHeight * 3];
		memset(m_ucPaintingOrg, 0, m_nPaintWidth * m_nPaintHeight * 3);

		// refresh paint view as well	
		m_pUI->m_paintView->refresh();
	
	
	return 0;
}

//------------------------------------------------------------------
// Get the color of the pixel in the original image at coord x and y
//------------------------------------------------------------------
GLubyte* ImpressionistDoc::GetOriginalPixel( int x, int y )
{
	if ( x < 0 ) 
		x = 0;
	else if ( x >= m_nWidth ) 
		x = m_nWidth-1;

	if ( y < 0 ) 
		y = 0;
	else if ( y >= m_nHeight ) 
		y = m_nHeight-1;

	return (GLubyte*)(m_ucBitmapScaled + 3 * (y*m_nWidth + x));
}

// get map from m_ncOriginBitmap
GLubyte* ImpressionistDoc::GetPixelFromOriginBitmap(int x, int y)
{
	if (x < 0)
		x = 0;
	else if (x >= m_nWidth)
		x = m_nWidth - 1;

	if (y < 0)
		y = 0;
	else if (y >= m_nHeight)
		y = m_nHeight - 1;

	return (GLubyte*)(m_ucOriginBitmap + 3 * (y * m_nWidth + x));
}

GLubyte* ImpressionistDoc::GetEdgePixel(int x, int y)
{
	if (x < 0)
		x = 0;
	else if (x >= m_nWidth)
		x = m_nWidth - 1;

	if (y < 0)
		y = 0;
	else if (y >= m_nHeight)
		y = m_nHeight - 1;

	return (GLubyte*)(m_ucEdgeBitmap + 3 * (y * m_nWidth + x));
}


GLubyte* ImpressionistDoc::GetAnotherPixel(int x, int y)
{
	if (x < 0)
		x = 0;
	else if (x >= m_nWidth)
		x = m_nWidth - 1;

	if (y < 0)
		y = 0;
	else if (y >= m_nHeight)
		y = m_nHeight - 1;

	return (GLubyte*)(m_ucAnotherBitmap + 3 * (y * m_nWidth + x));
}

GLubyte* ImpressionistDoc::GetPixelFromOriginBitmap(const Point p)
{
	return GetPixelFromOriginBitmap(p.x, p.y);
}

GLubyte* ImpressionistDoc::GetAnotherPixel(const Point p)
{
	return GetAnotherPixel(p.x, p.y);
}

//----------------------------------------------------------------
// Get the color of the pixel in the original image at point p
//----------------------------------------------------------------
GLubyte* ImpressionistDoc::GetOriginalPixel( const Point p )
{
	return GetOriginalPixel( p.x, p.y );
}

GLubyte* ImpressionistDoc::GetEdgePixel(const Point p)
{
	return GetEdgePixel(p.x, p.y);
}


// get the edge of the whole picture
// renew the edge: trigered for display edge(without edge picture) or renew the threshold
void ImpressionistDoc::egdeDetection()
{
	if (!m_ucOriginBitmap)
		return;

	int threshold = m_pUI->getEdgeThreshold();
	int G_x[3][3] = { {-1,0,1},{-2,0,2},{-1,0,1} };			// conv kernal
	int G_y[3][3] = { {-1,-2,-1},{0,0,0},{1,2,1} };

	if(m_ucEdgeBitmap) delete[] m_ucEdgeBitmap;
	m_ucEdgeBitmap = new unsigned char[3 * m_nWidth * m_nHeight];

	for (int i = 0; i < m_nHeight; ++i)
	{
		for (int j = 0; j < m_nWidth; ++j)
		{
			int gradient[2] = { };						// grad_x, grad_y
			for (int k = 0; k < 3; k++)
			{
				for (int m = 0; m < 3; m++)
				{
					int pixel_x = j + m - 1;
					int pixel_y = i + k - 1;
					GLubyte color[3];     
					if (pixel_x< 0 || pixel_x > m_nWidth - 1 || pixel_y < 0 || pixel_y > m_nHeight - 1)
						color[0] = color[1] = color[2] = 0;
					else
						memcpy(color, GetPixelFromOriginBitmap(pixel_x, pixel_y), 3);
					int gary_level = (double)color[0] * 0.299 + (double)(color[1]) * 0.587 + (double)(color[2]) * 0.114;
					gradient[0] += gary_level * G_x[k][m];
					gradient[1] += gary_level * G_y[k][m];
				}
			}
			int grad_norm = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1]);
			for (int n = 0; n < 3; ++n)
			{
				if (i == 0 || j == 0 || i == m_nHeight-1 || j == m_nWidth-1)
					m_ucEdgeBitmap[3 * (i * m_nWidth + j) + n] = 0;
				else
					m_ucEdgeBitmap[3 * (i * m_nWidth + j) + n] = (grad_norm > threshold) ? 255 : 0;
			}
		}
	}
}

// for dissolving images:
void ImpressionistDoc::loadDissolveImage(char* iname)
{
	if (!m_ucOriginBitmap)
	{
		fl_alert("Original bitmap hasn't been loaded!");
		return;	
	}
		
	// try to open the image to read
	unsigned char* data;
	int				width,
					height;
	data = readBMP(iname, width, height);

	// release old storage
	if (m_ucDissolveBitmap) 
		delete[] m_ucEdgeBitmap;
	m_ucDissolveBitmap = new unsigned char[3 * m_nWidth * m_nHeight];
	memcpy(m_ucDissolveBitmap, m_ucOriginBitmap, 3 * m_nWidth * m_nHeight);

	int min_width = min(width, m_nWidth);
	int min_height = min(height, m_nHeight);

	double alpha_value = 0.5;

	for (int i = 0; i < min_height; i++) {
		for (int j = 0; j < min_width; j++)
		{
			m_ucDissolveBitmap[3 * (i * m_nWidth + j)] = data[3 * (i * width + j)] * alpha_value + m_ucOriginBitmap[3 * (i * m_nWidth + j)] * (1 - alpha_value);
			m_ucDissolveBitmap[3 * (i * m_nWidth + j) + 1] = data[3 * (i * width + j) + 1] * alpha_value + m_ucOriginBitmap[3 * (i * m_nWidth + j) + 1] * (1 - alpha_value);
			m_ucDissolveBitmap[3 * (i * m_nWidth + j) + 2] = data[3 * (i * width + j) + 2] * alpha_value + m_ucOriginBitmap[3 * (i * m_nWidth + j) + 2] * (1 - alpha_value);
		}
	}
	m_ucBitmap = m_ucDissolveBitmap;
	m_pUI->m_origView->refresh();
}

void ImpressionistDoc::autoPaint()
{
	m_pUI->m_paintView->willAutoPaint = true;
	m_pUI->m_paintView->redraw();
}

void ImpressionistDoc::undo()
{
	if (!m_ucLastBitmap)
		return;
	if (m_ucPaintingOrg) delete[] m_ucPaintingOrg;
	m_ucPaintingOrg = new unsigned char[3 * m_nPaintWidth * m_nPaintHeight];
	memcpy(m_ucPaintingOrg, m_ucLastBitmap, 3 * m_nPaintWidth * m_nPaintHeight);
	m_pUI->m_paintView->refresh();
}

