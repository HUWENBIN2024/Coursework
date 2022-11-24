//
// impressionistUI.h
//
// The header file for the UI part
//

#ifndef ImpressionistUI_h
#define ImpressionistUI_h

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/fl_file_chooser.H>		// FLTK file chooser
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Light_Button.H>
#include <iostream>
using namespace std;

#include "Impressionist.h"
#include "OriginalView.h"
#include "PaintView.h"

#include "ImpBrush.h"

class ImpressionistUI {
public:
	ImpressionistUI();

	// The FLTK widgets
	Fl_Window*			m_mainWindow;
	Fl_Menu_Bar*		m_menubar;
								
	PaintView*			m_paintView;
	OriginalView*		m_origView;

// for brush dialog
	Fl_Window*			m_brushDialog;
	Fl_Choice*			m_BrushTypeChoice;
	Fl_Choice*			m_AlphaMapChoice;//For BW11

	Fl_Slider*			m_BrushSizeSlider;

	Fl_Slider*          m_LineWidthSlider;			// this two slider are used for line brush width and angle, add by HU
	Fl_Slider*          m_LineAngleSlider;
	Fl_Slider*			m_EdgeThresholdSlider;

	Fl_Button*          m_ClearCanvasButton;
	Fl_Button*			m_DisplayEdgeButton;		// display the new edge map when changing the threshold
	Fl_Light_Button*	m_EdgeClipping;				// light button for edge clipping
	Fl_Light_Button*    m_AnotherGradient;				// light button for edge clipping

	Fl_Window*			m_manipulateColorDialog;//For BW5
	Fl_Window*			m_paintviewBgdDialog;//For BW9
	Fl_Window*			m_convDialog;//For BW15

	// Member functions
	void				setDocument(ImpressionistDoc* doc);
	ImpressionistDoc*	getDocument();

	void				show();
	void				resize_windows(int w, int h);

	// Interface to get attribute

	int					getSize();
	void				setSize(int size);
	int					getLineWidth();
	int					getLineAngle();
	void				setLineAngle(double angle); // unit: radian
	double              getAlpha();
	int					getEdgeThreshold();
	double				getBgdVal();

	bool isEdgeClipping = false;
	bool isAnotherGradient = false;

private:
	ImpressionistDoc*	m_pDoc;		// pointer to document to communicate with the document

	

	// All attributes here
	int		m_nSize;
	double  m_nAlpha;
	int		m_nLineWidth;
	int		m_nLineAngle;
	int     m_nThreshold;
	double  m_nBgdVal;
	
	// Static class members
	static Fl_Menu_Item		menuitems[];
	static Fl_Menu_Item		brushTypeMenu[NUM_BRUSH_TYPE+1];
	static Fl_Menu_Item		StrokeDirectionTypeMenu[4];
	static Fl_Menu_Item		filterSizeMenu[4];
	static Fl_Menu_Item		normalize_or_not_Menu[3];

	static ImpressionistUI*	whoami(Fl_Menu_* o);

	// All callbacks here.  Callbacks are declared 
	// static
	static void	cb_load_image(Fl_Menu_* o, void* v);
	static void	cb_save_image(Fl_Menu_* o, void* v);
	static void	cb_brushes(Fl_Menu_* o, void* v);
	static void	cb_clear_canvas(Fl_Menu_* o, void* v);
	static void	cb_exit(Fl_Menu_* o, void* v);
	static void	cb_about(Fl_Menu_* o, void* v);
	static void	cb_load_edge_image(Fl_Menu_* o, void* v);
	static void	cb_load_another_image(Fl_Menu_* o, void* v);
	static void	cb_dissolve_image(Fl_Menu_* o, void* v);

	static void	cb_original_image(Fl_Menu_* o, void* v);
	static void	cb_edge_image(Fl_Menu_* o, void* v);
	static void	cb_another_image(Fl_Menu_* o, void* v);

	static void	cb_auto_paint(Fl_Menu_* o, void* v);
	static void	cb_undo(Fl_Menu_* o, void* v);

	static void	cb_brushChoice(Fl_Widget* o, void* v);
	static void	cb_strokeDiretionChoice(Fl_Widget* o, void* v);

	static void	cb_clear_canvas_button(Fl_Widget* o, void* v);
	static void cb_display_edge_button(Fl_Widget* o, void* v); 

	static void cb_alphaChosen(Fl_Widget* o, void* v);

	static void	cb_sizeSlides(Fl_Widget* o, void* v);
	static void cb_alphaSlides(Fl_Widget* o, void* v);
	static void cb_rgbScaleSlides(Fl_Widget* o, void* v);
	static void cb_bgdScaleSlides(Fl_Widget* o, void* v);

	static void cb_filter_size(Fl_Widget* o, void* v);
	static void cb_filter_nm(Fl_Widget* o, void* v);
	static void cb_filter_apply(Fl_Widget* o, void* v);
	static void cb_filter_restore(Fl_Widget* o, void* v);
	static void cb_save_matrix_text(Fl_Widget* o, void* v);

	static void cb_change(Fl_Menu_* o, void* v);
	static void cb_swap(Fl_Menu_* o, void* v);
	static void cb_manipulate_color(Fl_Menu_* o, void* v);
	static void cb_paintview_bgd(Fl_Menu_* o, void* v);
	static void cb_convolution(Fl_Menu_* o, void* v);
	static void cb_mosaic(Fl_Menu_* o, void* v);

	static void	cb_LineWidthSlides(Fl_Widget* o, void* v);
	static void	cb_LineAngleSlides(Fl_Widget* o, void* v);

	static void cb_edge_clipping(Fl_Widget* o, void* v);
	static void cb_another_gradient(Fl_Widget* o, void* v);
	static void cb_edge_threshold_slides(Fl_Widget* o, void* v);
};

#endif
