// 
// impressionistDoc.h
//
// header file for Doc 
//

#ifndef ImpressionistDoc_h
#define ImpressionistDoc_h

#include "impressionist.h"
#include "bitmap.h"
#include "Mosaic.h";

class ImpressionistUI;

class ImpressionistDoc 
{
public:
	ImpressionistDoc();

	void	setUI(ImpressionistUI* ui);		// Assign the UI to use

	int		loadImage(char *iname);			// called by the UI to load image
	int		loadAlphaMap(char* iname);      // BW11
	int		changeImage(char* iname);		// BW10
	int		saveImage(char *iname);			// called by the UI to save image
	int		loadEdgeImage(char* iname);
	int		loadAnotherImage(char* iname);
	void    displayOriginImage();
	void    displayEdgeImage();
	void    displayAnotherImage();
	void    displayEdgeImageFromThreshold();

	void	swap();							// BW4
	int     clearCanvas();                  // called by the UI to clear the drawing canvas
	void	setBrushType(int type);			// called by the UI to set the brushType
	void	setStrokeDirectionType(StrokeDirectionType type);			// called by the UI to set the StrokeDirectionType
	void	generateMosaic();				// BW20

	int		getSize();						// get the UI size
	double  getAlpha();                     // get the alpha value
	unsigned char getAlphaFromMap(Point source); // BW11
	double  getBgdVal();					// get the scale value of backgorund of paintview, for BW9
	int     getLineWidth();
	int     getLineAngle();
	void	setLineAngle(double angle);
	void	setSize(int size);				// set the UI size
	char*	getImageName();					// get the current image name
	
	void    egdeDetection();

	void    loadDissolveImage(char* iname);
	void    autoPaint();
	void	undo();

// Attributes
public:
	// Dimensions of original window.
	int				m_nWidth, 
					m_nHeight;
	// Dimensions of the paint window.
	int				m_nPaintWidth, 
					m_nPaintHeight;	
	// Bitmaps for original image and painting.
	unsigned char*	m_ucBitmap;
	unsigned char*  m_ucBitmapScaled; //For BW5
	unsigned char*	m_ucPainting;
	// bit map for edge and another
	unsigned char*  m_ucOriginBitmap;
	unsigned char*  m_ucEdgeBitmap;
	unsigned char*  m_ucAnotherBitmap;

	unsigned char*  m_ucLastBitmap;
	unsigned char* m_ucDissolveBitmap;

	unsigned char* m_ucPaintingOrg; //BW9
	unsigned char* m_ucAlphaMap; //BW11

	int				thereshold;				// thereshold of the edge
	// The Mosaic Doc
	Mosaic* m_pMosaic;

	// The current active brush.
	ImpBrush*			m_pCurrentBrush;	
	// Size of the brush.
	int m_nSize;		

	ImpressionistUI*	m_pUI;


// Operations
public:
	// Get the color of the original picture at the specified coord
	GLubyte* GetOriginalPixel( int x, int y );   
	// Get the color of the original picture at the specified point	
	GLubyte* GetOriginalPixel( const Point p );

	GLubyte* GetAnotherPixel(int x, int y);

	GLubyte* GetAnotherPixel(const Point p);

	GLubyte* GetPixelFromOriginBitmap(int x, int y);

	GLubyte* GetPixelFromOriginBitmap(const Point p);

	GLubyte* GetEdgePixel(int x, int y);

	GLubyte* GetEdgePixel(const Point p);

private:
	char			m_imageName[256];

};

extern void MessageBox(char *message);

#endif
