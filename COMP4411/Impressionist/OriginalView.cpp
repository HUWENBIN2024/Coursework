//
// originalview.cpp
//
// The code maintaining the original view of the input images
//

#include "impressionist.h"
#include "impressionistDoc.h"
#include "originalview.h"
#include <FL/Fl.H>

#ifndef WIN32
#define min(a, b)	( ( (a)<(b) ) ? (a) : (b) )
#endif

OriginalView::OriginalView(int			x, 
						   int			y, 
						   int			w, 
						   int			h, 
						   const char*	l)
							: Fl_Gl_Window(x,y,w,h,l)
{
	m_nWindowWidth	= w;
	m_nWindowHeight	= h;

	markerUpdate = false;
	markerPos.x = 0; markerPos.y = 0;

	rgbScale[0] = rgbScale[1] = rgbScale[2] = 1;

	filter = new Filter;
	filter->isApplied = false;
	filter->normalize = true;
	filter->size = 3;
	filter->matrix = NULL;
	matrixText = "";

}

void OriginalView::draw()
{
	if(!valid())
	{
		glClearColor(0.7f, 0.7f, 0.7f, 1.0);

		// We're only using 2-D, so turn off depth 
		glDisable( GL_DEPTH_TEST );

		// Tell openGL to read from the front buffer when capturing
		// out paint strokes 
		glReadBuffer( GL_FRONT );
		ortho();

	}

	glClear( GL_COLOR_BUFFER_BIT );

	if ( m_pDoc->m_ucBitmap ) 
	{
		// note that both OpenGL pixel storage and the Windows BMP format
		// store pixels left-to-right, BOTTOM-to-TOP!!  thus all the fiddling
		// around with startrow.

		m_nWindowWidth=w();
		m_nWindowHeight=h();

		int drawWidth, drawHeight;
		GLvoid* bitstart;

		// we are not using a scrollable window, so ignore it
		Point scrollpos;	// = GetScrollPosition();
		scrollpos.x=scrollpos.y=0;

		drawWidth	= min( m_nWindowWidth, m_pDoc->m_nWidth );
		drawHeight	= min( m_nWindowHeight, m_pDoc->m_nHeight );

		int	startrow	= m_pDoc->m_nHeight - (scrollpos.y + drawHeight);
		if ( startrow < 0 ) 
			startrow = 0;

		//BW15
		int width = m_pDoc->m_nWidth; int height = m_pDoc->m_nHeight;
		unsigned char* tempBitmapConv = new unsigned char[3 * height*width];
		for (int i = 0; i < 3 * width * height; i++) {
			tempBitmapConv[i] = m_pDoc->m_ucBitmap[i];
		}
		if (filter->isApplied) convolution(tempBitmapConv, m_pDoc->m_nWidth, m_pDoc->m_nHeight);

		//BW5
		for (int i = 0; i < m_pDoc->m_nHeight * m_pDoc->m_nWidth; i++) {
			m_pDoc->m_ucBitmapScaled[3 * i] = static_cast<unsigned char>(static_cast<int>(static_cast<int>(tempBitmapConv[3 * i]) * rgbScale[0]));
			m_pDoc->m_ucBitmapScaled[3 * i+1] = static_cast<unsigned char>(static_cast<int>(static_cast<int>(tempBitmapConv[3 * i+1]) * rgbScale[1]));
			m_pDoc->m_ucBitmapScaled[3 * i+2] = static_cast<unsigned char>(static_cast<int>(static_cast<int>(tempBitmapConv[3 * i+2]) * rgbScale[2]));
			
		}
		
		delete[] tempBitmapConv;

		//BW9
		for (int i = 0; i < 3 * m_pDoc->m_nHeight * m_pDoc->m_nWidth; i++) {
			if (m_pDoc->m_ucBitmapScaled[i] == static_cast<unsigned char>(0)) m_pDoc->m_ucBitmapScaled[i] = static_cast<unsigned char>(1);
		}

		bitstart = m_pDoc->m_ucBitmapScaled + 3 * ((m_pDoc->m_nWidth * startrow) + scrollpos.x);


		// just copy image to GLwindow conceptually
		glRasterPos2i( 0, m_nWindowHeight - drawHeight );
		glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
		glPixelStorei( GL_UNPACK_ROW_LENGTH, m_pDoc->m_nWidth );
		glDrawBuffer( GL_BACK );
		glDrawPixels( drawWidth, drawHeight, GL_RGB, GL_UNSIGNED_BYTE, bitstart );

		//BW3
		if (markerUpdate) {
			glPointSize(4);
			glBegin(GL_POINTS);
			glColor3d(1, 0, 0);
			glVertex2d(markerPos.x, markerPos.y + m_nWindowHeight - m_pDoc->m_nHeight);
			glEnd();
			markerUpdate = false;
		}

	}
			
	glFlush();
}

void OriginalView::refresh()
{
	redraw();
}

void OriginalView::resizeWindow(int	width, 
								int	height)
{
	resize(x(), y(), width, height);
}

void OriginalView::updateMarker(Point source)
{
	markerUpdate = true;
	markerPos = source;
	refresh();
}

void OriginalView::setRgbScale(double red, double green, double blue) {
	//-1 means do not change the value
	if (red != -1) rgbScale[0] = red;
	if (green != -1) rgbScale[1] = green;
	if (blue != -1) rgbScale[2] = blue;

	redraw();
}

void OriginalView::setFilter(int** source, int size)
{
	if (!source || size <= 0) return;

	if (filter->matrix) {
		for (int i = 0; i < filter->size; i++) {
			delete[] filter->matrix[i];
		}
		delete[] filter->matrix;
	}

	filter->matrix = new int* [size];
	for (int i = 0; i < size; i++) {
		filter->matrix[i] = new int[size];
		for (int j = 0; j < size; j++) {
			filter->matrix[i][j] = source[i][j];
		}
	}

	setFilterSize(size);
}

void OriginalView::setFilterSize(int size)
{
	filter->size = size;
}

int OriginalView::getFilterSize()
{
	return filter->size;
}

void OriginalView::switchFilter(bool switchedOn)
{
	filter->isApplied = switchedOn;
}

void OriginalView::setNormalize(bool normalize)
{
	filter->normalize = normalize;
}

void OriginalView::convolution(unsigned char* inputBitmap, int width, int height)
{
	if (!(filter->size == 3 || filter->size == 5 || filter->size == 7)) return;

	double normalize_factor = 1;
	if (filter->normalize) {
		int matrix_sum = 0;
		for (int i = 0; i < filter->size; i++) {
			for (int j = 0; j < filter->size; j++) {
				matrix_sum += filter->matrix[i][j];
			}
		}
		normalize_factor = 1 / (double)matrix_sum;
	}

	double* outputBitmap = new double[3 * width * height];
	for (int i = 0; i < 3 * width * height; i++) {//Cannot use memset() here to initialize. This bug takes me many hours to fix :(
		outputBitmap[i] = 0;
	}
	for (int height_i = 0; height_i < height; height_i++) {
		for (int width_i = 0; width_i < width; width_i++) {
			for (int size_h = filter->size - 1; size_h >= 0; size_h--) {
				for (int size_w = filter->size - 1; size_w >= 0; size_w--) {
					if ((height_i - (filter->size - 1) / 2 + filter->size - 1 - size_h) >= 0 &&
						(height_i - (filter->size - 1) / 2 + filter->size - 1 - size_h) < height &&
						(width_i - (filter->size - 1) / 2 + filter->size - 1 - size_w) >= 0 &&
						(width_i - (filter->size - 1) / 2 + filter->size - 1 - size_w) < width)//²»Ô½½ç
					{	
						for (int rgb = 0; rgb < 3; rgb++) {
							outputBitmap[3*width*height_i+3*width_i+rgb] += normalize_factor * filter->matrix[size_h][size_w] * 
								inputBitmap[3 * width * (height_i - (filter->size - 1) / 2 + filter->size - 1 - size_h) 
								+ 3 * (width_i - (filter->size - 1) / 2 + filter->size - 1 - size_w) + rgb];
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < 3 * width * height; i++) {
		if (outputBitmap[i] > 255) outputBitmap[i] = 255;
		if (outputBitmap[i] < 0) outputBitmap[i] = 0;
		inputBitmap[i] = static_cast<int>(outputBitmap[i]);
	}
	delete[] outputBitmap;
}