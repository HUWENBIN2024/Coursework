//
// paintview.cpp
//
// The code maintaining the painting view of the input images
//

#include "impressionist.h"
#include "impressionistDoc.h"
#include "impressionistUI.h"
#include "paintview.h"
#include "ImpBrush.h"
#include "math.h"
#include <iostream>
using namespace std;

extern float frand();

#define LEFT_MOUSE_DOWN		1
#define LEFT_MOUSE_DRAG		2
#define LEFT_MOUSE_UP		3
#define RIGHT_MOUSE_DOWN	4
#define RIGHT_MOUSE_DRAG	5
#define RIGHT_MOUSE_UP		6
#define PI 3.14159265

#ifndef WIN32
#define min(a, b)	( ( (a)<(b) ) ? (a) : (b) )
#define max(a, b)	( ( (a)>(b) ) ? (a) : (b) )
#endif

static int		eventToDo;
static int		isAnEvent=0;
static Point	coord;
int				red_line_start_point_x = 0;
int				red_line_start_point_y = 0;

PaintView::PaintView(int			x, 
					 int			y, 
					 int			w, 
					 int			h, 
					 const char*	l)
						: Fl_Gl_Window(x,y,w,h,l)
{
	m_nWindowWidth	= w;
	m_nWindowHeight	= h;

}


void PaintView::draw()
{
	#ifndef MESA
	// To avoid flicker on some machines.
	glDrawBuffer(GL_FRONT_AND_BACK);
	#endif // !MESA

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if(!valid())
	{

		glClearColor(0.7f, 0.7f, 0.7f, 1.0);

		// We're only using 2-D, so turn off depth 
		glDisable( GL_DEPTH_TEST );

		ortho();

		glClear( GL_COLOR_BUFFER_BIT );
	}

	Point scrollpos;// = GetScrollPosition();
	scrollpos.x = 0;
	scrollpos.y	= 0;

	m_nWindowWidth	= w();
	m_nWindowHeight	= h();

	int drawWidth, drawHeight;
	drawWidth = min( m_nWindowWidth, m_pDoc->m_nPaintWidth );
	drawHeight = min( m_nWindowHeight, m_pDoc->m_nPaintHeight );

	int startrow = m_pDoc->m_nPaintHeight - (scrollpos.y + drawHeight);
	if ( startrow < 0 ) startrow = 0;

	m_pPaintBitstart = m_pDoc->m_ucPainting + 
		3 * ((m_pDoc->m_nPaintWidth * startrow) + scrollpos.x);

	//BW9
	m_pPaintOrgBitstart = m_pDoc->m_ucPaintingOrg +
		3 * ((m_pDoc->m_nPaintWidth * startrow) + scrollpos.x);

	m_nDrawWidth	= drawWidth;
	m_nDrawHeight	= drawHeight;

	m_nStartRow		= startrow;
	m_nEndRow		= startrow + drawHeight;
	m_nStartCol		= scrollpos.x;
	m_nEndCol		= m_nStartCol + drawWidth;

	if ( m_pDoc->m_ucPainting && !isAnEvent) 
	{
		AddBgd();
		RestoreContent();
	}

	if ( m_pDoc->m_ucPainting && isAnEvent) 
	{

		// Clear it after processing.
		isAnEvent	= 0;	

		Point source( coord.x + m_nStartCol, m_nEndRow - coord.y );    // 取色部位
		Point target( coord.x, m_nWindowHeight - coord.y );
		
		// This is the event handler
		switch (eventToDo) 
		{
		case LEFT_MOUSE_DOWN:     // 按下鼠标
			if (m_pDoc->m_ucLastBitmap)			// 记录这一次绘画之前的bitmap, for "undo" function
				delete[] m_pDoc->m_ucLastBitmap;
			m_pDoc->m_ucLastBitmap = new unsigned char[3 * m_pDoc->m_nPaintWidth * m_pDoc->m_nPaintHeight];
			memcpy(m_pDoc->m_ucLastBitmap, m_pDoc->m_ucPainting, 3 * m_pDoc->m_nPaintWidth * m_pDoc->m_nPaintHeight);

			RestoreContentOrg();

			if (target.x >= 0 && target.x <= m_pDoc->m_nPaintWidth && target.y >= m_nWindowHeight - m_nDrawHeight)  // 边缘修剪
				m_pDoc->m_pCurrentBrush->BrushBegin(source, target);

			SaveCurrentContentOrg();
			AddBgd();
			RestoreContent();

			break;
		case LEFT_MOUSE_DRAG:	// 拖拽鼠标
			RestoreContentOrg();

			if (target.x >= 0 && target.x <= m_pDoc->m_nPaintWidth && target.y >= m_nWindowHeight - m_nDrawHeight)  // 边缘修剪
				m_pDoc->m_pCurrentBrush->BrushMove(source, target);

			SaveCurrentContentOrg();
			AddBgd();
			RestoreContent();

			if (target.x >= 0 && target.x <= m_pDoc->m_nPaintWidth && target.y >= m_nWindowHeight - m_nDrawHeight)  // 边缘修剪
				m_pDoc->m_pCurrentBrush->BrushMove(source, target);
			break;
		case LEFT_MOUSE_UP:		// 松开鼠标
			m_pDoc->m_pCurrentBrush->BrushEnd( source, target );

			SaveCurrentContent();
			RestoreContent();
			break;

		case RIGHT_MOUSE_DOWN:
			red_line_start_point_x = double(target.x);
			red_line_start_point_y = double(target.y);
			break;
		case RIGHT_MOUSE_DRAG:
			// do nothing so far
			break;
		case RIGHT_MOUSE_UP:
			// get the size and angle of lien brush using the right mouse
			m_pDoc->setLineAngle( atan( double(target.y - red_line_start_point_y) / double(target.x - red_line_start_point_x) ) );
			m_pDoc->setSize((int)sqrt( (target.y - red_line_start_point_y)* (target.y - red_line_start_point_y)
				+ (target.x - red_line_start_point_x) * (target.x - red_line_start_point_x) ) );
			break;

		default:
			printf("Unknown event!!\n");		
			break;
		}
	}

	if (willAutoPaint)
	{
		if (m_pDoc->m_ucLastBitmap)			// 记录这一次绘画之前的bitmap, for "undo" function
			delete[] m_pDoc->m_ucLastBitmap;
		m_pDoc->m_ucLastBitmap = new unsigned char[3 * m_pDoc->m_nPaintWidth * m_pDoc->m_nPaintHeight];
		memcpy(m_pDoc->m_ucLastBitmap, m_pDoc->m_ucPainting, 3 * m_pDoc->m_nPaintWidth * m_pDoc->m_nPaintHeight);

		autoPaint();
		willAutoPaint = false;
		SaveCurrentContentOrg();
		glFlush();
	}

	glFlush();

	#ifndef MESA
	// To avoid flicker on some machines.
	glDrawBuffer(GL_BACK);
	#endif // !MESA

}

void PaintView::autoPaint()
{
	const int w = m_pDoc->m_nPaintWidth;
	const int h = m_pDoc->m_nPaintHeight;

	for (int i = 0; i < w; i += 5)
	{
		for (int j = 0; j < h; j += 5)
		{
			const Point target(i , j + m_nWindowHeight - m_nDrawHeight );
			m_pDoc->m_pCurrentBrush->BrushBegin(target, target);
		}
	}
	
	
}

int PaintView::handle(int event)
{
	Point source(coord.x + m_nStartCol, m_nEndRow - coord.y);
	m_pDoc->m_pUI->m_origView->updateMarker(source);

	switch(event)
	{
	case FL_ENTER:
	    redraw();
		break;
	case FL_PUSH:
		coord.x = Fl::event_x();
		coord.y = Fl::event_y();
		if (Fl::event_button()>1)
			eventToDo=RIGHT_MOUSE_DOWN;
		else
			eventToDo=LEFT_MOUSE_DOWN;
		isAnEvent=1;
		redraw();
		break;
	case FL_DRAG:
		coord.x = Fl::event_x();
		coord.y = Fl::event_y();
		if (Fl::event_button()>1)
			eventToDo=RIGHT_MOUSE_DRAG;
		else
			eventToDo=LEFT_MOUSE_DRAG;
		isAnEvent=1;
		redraw();
		break;
	case FL_RELEASE:
		coord.x = Fl::event_x();
		coord.y = Fl::event_y();
		if (Fl::event_button()>1)
			eventToDo=RIGHT_MOUSE_UP;
		else
			eventToDo=LEFT_MOUSE_UP;
		isAnEvent=1;
		redraw();
		break;
	case FL_MOVE:
		coord.x = Fl::event_x();
		coord.y = Fl::event_y();
		break;
	default:
		return 0;
		break;

	}



	return 1;
}

void PaintView::refresh()
{
	redraw();
}

void PaintView::resizeWindow(int width, int height)
{
	resize(x(), y(), width, height);
}

void PaintView::SaveCurrentContent()
{
	// Tell openGL to read from the front buffer when capturing
	// out paint strokes
	glReadBuffer(GL_FRONT);

	glPixelStorei( GL_PACK_ALIGNMENT, 1 );
	glPixelStorei( GL_PACK_ROW_LENGTH, m_pDoc->m_nPaintWidth );
	
	glReadPixels( 0, 
				  m_nWindowHeight - m_nDrawHeight, 
				  m_nDrawWidth, 
				  m_nDrawHeight, 
				  GL_RGB, 
				  GL_UNSIGNED_BYTE, 
				  m_pPaintBitstart );
}

void PaintView::SaveCurrentContentOrg()
{
	glReadBuffer(GL_BACK);

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ROW_LENGTH, m_pDoc->m_nPaintWidth);

	glReadPixels(0,
		m_nWindowHeight - m_nDrawHeight,
		m_nDrawWidth,
		m_nDrawHeight,
		GL_RGB,
		GL_UNSIGNED_BYTE,
		m_pPaintOrgBitstart);
}


void PaintView::RestoreContent()
{
	glDrawBuffer(GL_BACK);

	glClear( GL_COLOR_BUFFER_BIT );

	glRasterPos2i( 0, m_nWindowHeight - m_nDrawHeight );
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1 );
	glPixelStorei( GL_UNPACK_ROW_LENGTH, m_pDoc->m_nPaintWidth );
	glDrawPixels( m_nDrawWidth, 
				  m_nDrawHeight, 
				  GL_RGB, 
				  GL_UNSIGNED_BYTE, 
				  m_pPaintBitstart);

//	glDrawBuffer(GL_FRONT);
}

void PaintView::RestoreContentOrg()
{
	glDrawBuffer(GL_BACK);

	glClear(GL_COLOR_BUFFER_BIT);

	glRasterPos2i(0, m_nWindowHeight - m_nDrawHeight);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, m_pDoc->m_nPaintWidth);
	glDrawPixels(m_nDrawWidth,
		m_nDrawHeight,
		GL_RGB,
		GL_UNSIGNED_BYTE,
		m_pPaintOrgBitstart);
}

void PaintView::AddBgd()
{
	for (int i = 0; i < 3 * m_nDrawWidth * m_nDrawHeight; i++) {
		if (m_pDoc->m_ucPaintingOrg[i] == 0) {
			m_pDoc->m_ucPainting[i] = static_cast<int>(m_pDoc->m_ucBitmapScaled[i] * m_pDoc->getBgdVal());
		}
		else {
			m_pDoc->m_ucPainting[i] = m_pDoc->m_ucPaintingOrg[i];
		}
	}
}