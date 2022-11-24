//
// impressionistUI.h
//
// The user interface part for the program.
//


#include <FL/fl_ask.h>

#include <math.h>
#include <sstream>
#include <vector>

#include "impressionistUI.h"
#include "impressionistDoc.h"
#include <iostream>
using namespace std;

#define PI 3.1415926
/*
//------------------------------ Widget Examples -------------------------------------------------
Here is some example code for all of the widgets that you may need to add to the 
project.  You can copy and paste these into your code and then change them to 
make them look how you want.  Descriptions for all of the widgets here can be found 
in links on the fltk help session page.

//---------Window/Dialog and Menubar-----------------------------------
	
	//----To install a window--------------------------
	Fl_Window* myWindow = new Fl_Window(600, 300, "MyWindow");
		myWindow->user_data((void*)(this));	// record self to be used by static callback functions
		
		// install menu bar
		myMenubar = new Fl_Menu_Bar(0, 0, 600, 25);
		Fl_Menu_Item ImpressionistUI::myMenuItems[] = {
			{ "&File",		0, 0, 0, FL_SUBMENU },
				{ "&Load...",	FL_ALT + 'l', (Fl_Callback *)ImpressionistUI::cb_load },
				{ "&Save...",	FL_ALT + 's', (Fl_Callback *)ImpressionistUI::cb_save }.
				{ "&Quit",			FL_ALT + 'q', (Fl_Callback *)ImpressionistUI::cb_exit },
				{ 0 },
			{ "&Edit",		0, 0, 0, FL_SUBMENU },
				{ "&Copy",FL_ALT + 'c', (Fl_Callback *)ImpressionistUI::cb_copy, (void *)COPY },
				{ "&Cut",	FL_ALT + 'x', (Fl_Callback *)ImpressionistUI::cb_cut, (void *)CUT },
				{ "&Paste",	FL_ALT + 'v', (Fl_Callback *)ImpressionistUI::cb_paste, (void *)PASTE },
				{ 0 },
			{ "&Help",		0, 0, 0, FL_SUBMENU },
				{ "&About",	FL_ALT + 'a', (Fl_Callback *)ImpressionistUI::cb_about },
				{ 0 },
			{ 0 }
		};
		myMenubar->menu(myMenuItems);
    m_mainWindow->end();

	//----The window callback--------------------------
	// One of the callbacks
	void ImpressionistUI::cb_load(Fl_Menu_* o, void* v) 
	{	
		ImpressionistDoc *pDoc=whoami(o)->getDocument();

		char* newfile = fl_file_chooser("Open File?", "*.bmp", pDoc->getImageName() );
		if (newfile != NULL) {
			pDoc->loadImage(newfile);
		}
	}


//------------Slider---------------------------------------

	//----To install a slider--------------------------
	Fl_Value_Slider * mySlider = new Fl_Value_Slider(10, 80, 300, 20, "My Value");
	mySlider->user_data((void*)(this));	// record self to be used by static callback functions
	mySlider->type(FL_HOR_NICE_SLIDER);
    mySlider->labelfont(FL_COURIER);
    mySlider->labelsize(12);
	mySlider->minimum(1);
	mySlider->maximum(40);
	mySlider->step(1);
	mySlider->value(m_nMyValue);
	mySlider->align(FL_ALIGN_RIGHT);
	mySlider->callback(cb_MyValueSlides);

	//----The slider callback--------------------------
	void ImpressionistUI::cb_MyValueSlides(Fl_Widget* o, void* v)
	{
		((ImpressionistUI*)(o->user_data()))->m_nMyValue=int( ((Fl_Slider *)o)->value() ) ;
	}
	

//------------Choice---------------------------------------
	
	//----To install a choice--------------------------
	Fl_Choice * myChoice = new Fl_Choice(50,10,150,25,"&myChoiceLabel");
	myChoice->user_data((void*)(this));	 // record self to be used by static callback functions
	Fl_Menu_Item ImpressionistUI::myChoiceMenu[3+1] = {
	  {"one",FL_ALT+'p', (Fl_Callback *)ImpressionistUI::cb_myChoice, (void *)ONE},
	  {"two",FL_ALT+'l', (Fl_Callback *)ImpressionistUI::cb_myChoice, (void *)TWO},
	  {"three",FL_ALT+'c', (Fl_Callback *)ImpressionistUI::cb_myChoice, (void *)THREE},
	  {0}
	};
	myChoice->menu(myChoiceMenu);
	myChoice->callback(cb_myChoice);
	
	//-----The choice callback-------------------------
	void ImpressionistUI::cb_myChoice(Fl_Widget* o, void* v)
	{
		ImpressionistUI* pUI=((ImpressionistUI *)(o->user_data()));
		ImpressionistDoc* pDoc=pUI->getDocument();

		int type=(int)v;

		pDoc->setMyType(type);
	}


//------------Button---------------------------------------

	//---To install a button---------------------------
	Fl_Button* myButton = new Fl_Button(330,220,50,20,"&myButtonLabel");
	myButton->user_data((void*)(this));   // record self to be used by static callback functions
	myButton->callback(cb_myButton);

	//---The button callback---------------------------
	void ImpressionistUI::cb_myButton(Fl_Widget* o, void* v)
	{
		ImpressionistUI* pUI=((ImpressionistUI*)(o->user_data()));
		ImpressionistDoc* pDoc = pUI->getDocument();
		pDoc->startPainting();
	}


//---------Light Button------------------------------------
	
	//---To install a light button---------------------
	Fl_Light_Button* myLightButton = new Fl_Light_Button(240,10,150,25,"&myLightButtonLabel");
	myLightButton->user_data((void*)(this));   // record self to be used by static callback functions
	myLightButton->callback(cb_myLightButton);

	//---The light button callback---------------------
	void ImpressionistUI::cb_myLightButton(Fl_Widget* o, void* v)
	{
		ImpressionistUI *pUI=((ImpressionistUI*)(o->user_data()));

		if (pUI->myBool==TRUE) pUI->myBool=FALSE;
		else pUI->myBool=TRUE;
	}

//----------Int Input--------------------------------------

    //---To install an int input-----------------------
	Fl_Int_Input* myInput = new Fl_Int_Input(200, 50, 5, 5, "&My Input");
	myInput->user_data((void*)(this));   // record self to be used by static callback functions
	myInput->callback(cb_myInput);

	//---The int input callback------------------------
	void ImpressionistUI::cb_myInput(Fl_Widget* o, void* v)
	{
		((ImpressionistUI*)(o->user_data()))->m_nMyInputValue=int( ((Fl_Int_Input *)o)->value() );
	}

//------------------------------------------------------------------------------------------------
*/

//------------------------------------- Help Functions --------------------------------------------

//------------------------------------------------------------
// This returns the UI, given the menu item.  It provides a
// link from the menu items to the UI
//------------------------------------------------------------
ImpressionistUI* ImpressionistUI::whoami(Fl_Menu_* o)	
{
	return ( (ImpressionistUI*)(o->parent()->user_data()) );
}


//--------------------------------- Callback Functions --------------------------------------------

//------------------------------------------------------------------
// Brings up a file chooser and then loads the chosen image
// This is called by the UI when the load image menu item is chosen
//------------------------------------------------------------------
void ImpressionistUI::cb_load_image(Fl_Menu_* o, void* v) 
{
	ImpressionistDoc *pDoc=whoami(o)->getDocument();

	char* newfile = fl_file_chooser("Open File?", "*.bmp", pDoc->getImageName() );
	if (newfile != NULL) {
		pDoc->loadImage(newfile);
	}
}

//BW10
void ImpressionistUI::cb_change(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();

	char* newfile = fl_file_chooser("Choose Another Image", "*.bmp", pDoc->getImageName());
	if (newfile != NULL) {
		pDoc->changeImage(newfile);
	}
}
void ImpressionistUI::cb_load_edge_image(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();

	char* newfile = fl_file_chooser("Open File?", "*.bmp", pDoc->getImageName());
	if (newfile != NULL) {
		pDoc->loadEdgeImage(newfile);
	}
}
void ImpressionistUI::cb_load_another_image(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();

	char* newfile = fl_file_chooser("Open File?", "*.bmp", pDoc->getImageName());
	if (newfile != NULL) {
		pDoc->loadAnotherImage(newfile);
	}
}

void ImpressionistUI::cb_dissolve_image(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();

	char* newfile = fl_file_chooser("Open File?", "*.bmp", pDoc->getImageName());
	if (newfile != NULL) {
		pDoc->loadDissolveImage(newfile);
	}
}

//------------------------------------------------------------------
// Brings up a file chooser and then saves the painted image
// This is called by the UI when the save image menu item is chosen
//------------------------------------------------------------------
void ImpressionistUI::cb_save_image(Fl_Menu_* o, void* v) 
{
	ImpressionistDoc *pDoc=whoami(o)->getDocument();

	char* newfile = fl_file_chooser("Save File?", "*.bmp", "save.bmp" );
	if (newfile != NULL) {
		pDoc->saveImage(newfile);
	}
}

//-------------------------------------------------------------
// Brings up the paint dialog
// This is called by the UI when the brushes menu item
// is chosen
//-------------------------------------------------------------
void ImpressionistUI::cb_brushes(Fl_Menu_* o, void* v) 
{
	whoami(o)->m_brushDialog->show();
}

//------------------------------------------------------------
// Clears the paintview canvas.
// Called by the UI when the clear canvas menu item is chosen
//------------------------------------------------------------
void ImpressionistUI::cb_clear_canvas(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc=whoami(o)->getDocument();

	pDoc->clearCanvas();
}

//BW4
void ImpressionistUI::cb_swap(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();

	pDoc->swap();
}

//BW5
void ImpressionistUI::cb_manipulate_color(Fl_Menu_* o, void* v)
{
	whoami(o)->m_manipulateColorDialog->show();
}

//BW9
void ImpressionistUI::cb_paintview_bgd(Fl_Menu_* o, void* v)
{
	whoami(o)->m_paintviewBgdDialog->show();
}

//BW15
void ImpressionistUI::cb_convolution(Fl_Menu_* o, void* v)
{
	whoami(o)->m_convDialog->show();
}

//BW20
void ImpressionistUI::cb_mosaic(Fl_Menu_* o, void* v)
{
	whoami(o)->m_pDoc->generateMosaic();
}

//------------------------------------------------------------
// Causes the Impressionist program to exit
// Called by the UI when the quit menu item is chosen
//------------------------------------------------------------
void ImpressionistUI::cb_exit(Fl_Menu_* o, void* v) 
{
	whoami(o)->m_mainWindow->hide();
	whoami(o)->m_brushDialog->hide();

}



//-----------------------------------------------------------
// Brings up an about dialog box
// Called by the UI when the about menu item is chosen
//-----------------------------------------------------------
void ImpressionistUI::cb_about(Fl_Menu_* o, void* v) 
{
	fl_message("Impressionist FLTK version for CS341, Spring 2002");
}

// Display Menu
void ImpressionistUI::cb_original_image(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();
	pDoc->displayOriginImage();

}
void ImpressionistUI::cb_edge_image(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();
	pDoc->displayEdgeImage();
}
void ImpressionistUI::cb_another_image(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();
	pDoc->displayAnotherImage();
}


// function menus
void ImpressionistUI::cb_auto_paint(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();
	pDoc->autoPaint();
}

void ImpressionistUI::cb_undo(Fl_Menu_* o, void* v)
{
	ImpressionistDoc* pDoc = whoami(o)->getDocument();
	pDoc->undo();
}


//------- UI should keep track of the current for all the controls for answering the query from Doc ---------
//-------------------------------------------------------------
// Sets the type of brush to use to the one chosen in the brush 
// choice.  
// Called by the UI when a brush is chosen in the brush choice
//-------------------------------------------------------------
void ImpressionistUI::cb_brushChoice(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI=((ImpressionistUI *)(o->user_data()));
	ImpressionistDoc* pDoc=pUI->getDocument();

	int type=(int)v;

	pDoc->setBrushType(type);

	if (type == BRUSH_ALPHA) cb_alphaChosen(o, v);
}

void ImpressionistUI::cb_strokeDiretionChoice(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));
	ImpressionistDoc* pDoc = pUI->getDocument();

	int type = (int)v;
	//pDoc->setStrokeDirectionType(type);
	pDoc->setStrokeDirectionType((StrokeDirectionType)type);
}

void ImpressionistUI::cb_alphaChosen(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));
	ImpressionistDoc* pDoc = pUI->getDocument();

	char* newfile = fl_file_chooser("Open File?", "*.bmp", pDoc->getImageName());
	if (newfile != NULL) {
		pDoc->loadAlphaMap(newfile);
	}
	else {

		delete[] pDoc->m_ucAlphaMap;
		pDoc->m_ucAlphaMap = NULL;
	}
	
}

//------------------------------------------------------------
// Clears the paintview canvas.
// Called by the UI when the clear canvas button is pushed
//------------------------------------------------------------
void ImpressionistUI::cb_clear_canvas_button(Fl_Widget* o, void* v)
{
	ImpressionistDoc * pDoc = ((ImpressionistUI*)(o->user_data()))->getDocument();

	pDoc->clearCanvas();
}

void ImpressionistUI::cb_display_edge_button(Fl_Widget* o, void* v)
{
	ImpressionistDoc* pDoc = ((ImpressionistUI*)(o->user_data()))->getDocument();

	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));

	pDoc->displayEdgeImageFromThreshold();
}


//-----------------------------------------------------------
// Updates the brush size to use from the value of the size
// slider
// Called by the UI when the size slider is moved
//-----------------------------------------------------------
void ImpressionistUI::cb_sizeSlides(Fl_Widget* o, void* v)
{
	((ImpressionistUI*)(o->user_data()))->m_nSize=int( ((Fl_Slider *)o)->value() ) ;
}

//Update Alpha Value
void ImpressionistUI::cb_alphaSlides(Fl_Widget* o, void* v)
{
	((ImpressionistUI*)(o->user_data()))->m_nAlpha = ((Fl_Slider*)o)->value();
}

//BW5 Update R/G/B Scale Value
void ImpressionistUI::cb_rgbScaleSlides(Fl_Widget* o, void* v)
{
	const char* label = ((Fl_Slider*)o)->label();
	const char* red = "Red Scale";
	const char* green = "Green Scale";
	const char* blue = "Blue Scale";

	if (!strcmp(label, red)) //(((Fl_Slider*)o)->label() == "Red Scale")
	{
		((ImpressionistUI*)(o->user_data()))->m_origView->setRgbScale(((Fl_Slider*)o)->value(), -1, -1);	
	}
		
	if (!strcmp(label, green)) // (((Fl_Slider*)o)->label() == "Green Scale")
	{
		((ImpressionistUI*)(o->user_data()))->m_origView->setRgbScale(-1, ((Fl_Slider*)o)->value(), -1);
	}
	if (!strcmp(label, blue)) // (((Fl_Slider*)o)->label() == "Blue Scale")
	{
		((ImpressionistUI*)(o->user_data()))->m_origView->setRgbScale(-1, -1, ((Fl_Slider*)o)->value());
	}
}
// Updates Line width 
void ImpressionistUI::cb_LineWidthSlides(Fl_Widget* o, void* v)
{
	((ImpressionistUI*)(o->user_data()))->m_nLineWidth = int(((Fl_Slider*)o)->value());
}

// update Line angle
void ImpressionistUI::cb_LineAngleSlides(Fl_Widget* o, void* v)
{
	((ImpressionistUI*)(o->user_data()))->m_nLineAngle = int(((Fl_Slider*)o)->value());
}


// call function for light buttons
void ImpressionistUI::cb_edge_clipping(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));

	if (pUI->isEdgeClipping == TRUE) pUI->isEdgeClipping = FALSE;
	else pUI->isEdgeClipping = TRUE;
	}
void ImpressionistUI::cb_another_gradient(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));

	if (pUI->isAnotherGradient == TRUE) pUI->isAnotherGradient = FALSE;
	else pUI->isAnotherGradient = TRUE;
}

void ImpressionistUI::cb_edge_threshold_slides(Fl_Widget* o, void* v)
{
	((ImpressionistUI*)(o->user_data()))->m_nThreshold = int(((Fl_Slider*)o)->value());		// ¸üÐÂthreshold
}


//BW9 Update Background Scale Value
void ImpressionistUI::cb_bgdScaleSlides(Fl_Widget* o, void* v)
{
	((ImpressionistUI*)(o->user_data()))->m_nBgdVal = ((Fl_Slider*)o)->value();
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));
	pUI->m_paintView->refresh();
}

//BW15
void ImpressionistUI::cb_filter_size(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));

	int size = ((Fl_Choice*)o)->value() * 2 + 3;//0,1,2 to 3,5,7

	pUI->m_origView->setFilterSize(size);
}

void ImpressionistUI::cb_filter_nm(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));

	bool normalize = (bool)v;

	pUI->m_origView->setNormalize(normalize);
}

void ImpressionistUI::cb_filter_apply(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));
	int size = pUI->m_origView->getFilterSize();

	int** matrix = new int* [size];
	for (int i = 0; i < size; i++) {
		matrix[i] = new int[size];
	}

	//change matrix text to 2D array
	stringstream text_stream(pUI->m_origView->getMatrixText());
	string item;
	vector<int> matrixArray;
	while (std::getline(text_stream, item, ',')) {
		matrixArray.push_back(stoi(item));
	}
	if (matrixArray.size() != size * size) {
		fl_message("Invalid Input");
	}
	else {
		for (int i = 0; i < size; i++) {
			for (int j = 0; j < size; j++) {
				matrix[i][j] = matrixArray[i * size + j];
			}
		}
		pUI->m_origView->switchFilter(true);
		pUI->m_origView->setFilter(matrix, pUI->m_origView->getFilterSize());
	}

}

void ImpressionistUI::cb_filter_restore(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));
	pUI->m_origView->switchFilter(false);
}

void ImpressionistUI::cb_save_matrix_text(Fl_Widget* o, void* v)
{
	ImpressionistUI* pUI = ((ImpressionistUI*)(o->user_data()));
	pUI->m_origView->setMatrixText(((Fl_Input*)o)->value());
}


//---------------------------------- per instance functions --------------------------------------

//------------------------------------------------
// Return the ImpressionistDoc used
//------------------------------------------------
ImpressionistDoc* ImpressionistUI::getDocument()
{
	return m_pDoc;
}

//------------------------------------------------
// Draw the main window
//------------------------------------------------
void ImpressionistUI::show() {
	m_mainWindow->show();
	m_paintView->show();
	m_origView->show();
}

//------------------------------------------------
// Change the paint and original window sizes to 
// w by h
//------------------------------------------------
void ImpressionistUI::resize_windows(int w, int h) {
	m_paintView->size(w,h);
	m_origView->size(w,h);
}

//------------------------------------------------ 
// Set the ImpressionistDoc used by the UI to 
// communicate with the brushes 
//------------------------------------------------
void ImpressionistUI::setDocument(ImpressionistDoc* doc)
{
	m_pDoc = doc;

	m_origView->m_pDoc = doc;
	m_paintView->m_pDoc = doc;
}

//------------------------------------------------
// Return the brush size
//------------------------------------------------
int ImpressionistUI::getSize()
{
	return m_nSize;
}

//------------------------------------------------
// Return the alpha value
//------------------------------------------------
double ImpressionistUI::getAlpha()
{
	return m_nAlpha;
}

int ImpressionistUI::getLineWidth()
{
	return m_nLineWidth;
}

int ImpressionistUI::getLineAngle()
{
	return m_nLineAngle;
}

int	ImpressionistUI::getEdgeThreshold()
{
	return m_nThreshold;
}

void ImpressionistUI::setLineAngle(double angle)
{
	m_nLineAngle = int(round(angle * 180 / PI));
	m_LineAngleSlider->value(m_nLineAngle);

}

double ImpressionistUI::getBgdVal()
{
	return m_nBgdVal;
}

//-------------------------------------------------
// Set the brush size
//-------------------------------------------------
void ImpressionistUI::setSize( int size )
{
	m_nSize=size;

	if (size<=40) 
		m_BrushSizeSlider->value(m_nSize);
}

// Main menu definition
Fl_Menu_Item ImpressionistUI::menuitems[] = {
	{ "&File",		0, 0, 0, FL_SUBMENU },
		{ "&Load Image...",	FL_ALT + 'l', (Fl_Callback *)ImpressionistUI::cb_load_image },
 {"&Change an Image... (BW10)", FL_ALT + 'L', (Fl_Callback *)ImpressionistUI::cb_change },
		{ "&Save Image...",	FL_ALT + 's', (Fl_Callback *)ImpressionistUI::cb_save_image },
		{ "&Brushes...",	FL_ALT + 'b', (Fl_Callback *)ImpressionistUI::cb_brushes }, 
		{ "&Clear Canvas",  FL_ALT + 'c', (Fl_Callback *)ImpressionistUI::cb_clear_canvas, 0, FL_MENU_DIVIDER },
		{"Swap Views (BW4)", FL_ALT + 'S', (Fl_Callback*)ImpressionistUI::cb_swap },
 {"Manipulate Color (BW5)", FL_ALT + 'm', (Fl_Callback *)ImpressionistUI::cb_manipulate_color },
		{ "&Load Edge Image...",	FL_ALT + 'e', (Fl_Callback*)ImpressionistUI::cb_load_edge_image },
		{ "&Load Another Image...",	FL_ALT + 'q', (Fl_Callback*)ImpressionistUI::cb_load_another_image },
		{ "&Dissolve Image",	FL_ALT + 'k', (Fl_Callback*)ImpressionistUI::cb_dissolve_image },
		{"PaintView Background (BW9)", FL_ALT + 'g', (Fl_Callback*)ImpressionistUI::cb_paintview_bgd },
	{"Filter Design (BW15)", FL_ALT + 'd', (Fl_Callback*)ImpressionistUI::cb_convolution },
	{"Generate Mosaic (BW20)", FL_ALT + 'G', (Fl_Callback*)ImpressionistUI::cb_mosaic },

		{ "&Quit",			FL_ALT + 'q', (Fl_Callback *)ImpressionistUI::cb_exit },
		{ 0 },

	{ "&Display",		0, 0, 0, FL_SUBMENU },
		{ "&Original Image",	FL_ALT + 'o', (Fl_Callback*)ImpressionistUI::cb_original_image },
		{ "&Edge Image",	FL_ALT + 'e', (Fl_Callback*)ImpressionistUI::cb_edge_image },
		{ "&Another Image",	FL_ALT + 'a', (Fl_Callback*)ImpressionistUI::cb_another_image },
		{ 0 },
		
	{ "&Function",		0, 0, 0, FL_SUBMENU },
		{ "&Auto Paint",	FL_ALT + 'o', (Fl_Callback*)ImpressionistUI::cb_auto_paint },
		{ "&Undo",	FL_ALT + 'e', (Fl_Callback*)ImpressionistUI::cb_undo },
		{ 0 },

	{ "&Help",		0, 0, 0, FL_SUBMENU },
		{ "&About",	FL_ALT + 'a', (Fl_Callback *)ImpressionistUI::cb_about },
		{ 0 },

	{ 0 }
};

// Brush choice menu definition
Fl_Menu_Item ImpressionistUI::brushTypeMenu[NUM_BRUSH_TYPE+1] = {
  {"Points",			FL_ALT+'p', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_POINTS},
  {"Lines",				FL_ALT+'l', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_LINES},
  {"Circles",			FL_ALT+'c', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_CIRCLES},
  {"Scattered Points",	FL_ALT+'q', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_SCATTERED_POINTS},
  {"Scattered Lines",	FL_ALT+'m', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_SCATTERED_LINES},
  {"Scattered Circles",	FL_ALT+'d', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_SCATTERED_CIRCLES},
  {"Blur",              FL_ALT+'b', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_BLUR},
  {"Alpha Mapped",		FL_ALT+'a', (Fl_Callback *)ImpressionistUI::cb_brushChoice, (void *)BRUSH_ALPHA},
  {"Triangle Brush",		FL_ALT + 'c', (Fl_Callback*)ImpressionistUI::cb_brushChoice, (void*)BRUSH_TRIANGLE},
  {"Curved Brush",		FL_ALT + 'd', (Fl_Callback*)ImpressionistUI::cb_brushChoice, (void*)BRUSH_CRUVED},

  {0}
};

Fl_Menu_Item ImpressionistUI::StrokeDirectionTypeMenu[4] = {
  {"Slider/Right Mouse",			FL_ALT + 'p', (Fl_Callback*)ImpressionistUI::cb_strokeDiretionChoice, (void*)SLIDER_RIGHT_MOUSE},
  {"Gradient",						FL_ALT + 'l', (Fl_Callback*)ImpressionistUI::cb_strokeDiretionChoice, (void*)GRADIENT},
  {"Brush Direction",				FL_ALT + 'c', (Fl_Callback*)ImpressionistUI::cb_strokeDiretionChoice, (void*)BRUSH_DIRECTION},
  {0}
};

//BW15
Fl_Menu_Item ImpressionistUI::filterSizeMenu[3 + 1] = {
  {"3",FL_ALT + '3', (Fl_Callback*)ImpressionistUI::cb_filter_size, (void*)3},
  {"5",FL_ALT + '5', (Fl_Callback*)ImpressionistUI::cb_filter_size, (void*)5},
  {"7",FL_ALT + '7', (Fl_Callback*)ImpressionistUI::cb_filter_size, (void*)7},
  {0}
};

Fl_Menu_Item ImpressionistUI::normalize_or_not_Menu[2 + 1] = {
  {"Normalize",FL_ALT + 'Y', (Fl_Callback*)ImpressionistUI::cb_filter_nm, (void*)1},
  {"Do not Normalize",FL_ALT + 'N', (Fl_Callback*)ImpressionistUI::cb_filter_nm, (void*)0},
  {0}
};


//----------------------------------------------------
// Constructor.  Creates all of the widgets.
// Add new widgets here
//----------------------------------------------------
ImpressionistUI::ImpressionistUI() {
	// Create the main window
	m_mainWindow = new Fl_Window(600, 300, "Impressionist");
		m_mainWindow->user_data((void*)(this));	// record self to be used by static callback functions
		// install menu bar
		m_menubar = new Fl_Menu_Bar(0, 0, 600, 25);
		m_menubar->menu(menuitems);

		// Create a group that will hold two sub windows inside the main
		// window
		Fl_Group* group = new Fl_Group(0, 25, 600, 275);

			// install paint view window
			m_paintView = new PaintView(300, 25, 300, 275, "This is the paint view");//0jon
			m_paintView->box(FL_DOWN_FRAME);

			// install original view window
			m_origView = new OriginalView(0, 25, 300, 275, "This is the orig view");//300jon
			m_origView->box(FL_DOWN_FRAME);
			m_origView->deactivate();

		group->end();
		Fl_Group::current()->resizable(group);
    m_mainWindow->end();

	// init values

	m_nSize = 10;
	m_nAlpha = 1.00;
	m_nLineAngle = 0;
	m_nLineWidth = 1;
	m_nThreshold = 200;
	m_nBgdVal = 0;

	// brush dialog definition
	m_brushDialog = new Fl_Window(400, 325, "Brush Dialog");
		// Add a brush type choice to the dialog
		m_BrushTypeChoice = new Fl_Choice(50,10,150,25,"&Brush");
		m_BrushTypeChoice->user_data((void*)(this));	// record self to be used by static callback functions
		m_BrushTypeChoice->menu(brushTypeMenu);
		m_BrushTypeChoice->callback(cb_brushChoice);

		// Add a stroke direction choice to the dialog
		m_BrushTypeChoice = new Fl_Choice(114, 40, 150, 25, "&Stroke Direction");
		m_BrushTypeChoice->user_data((void*)(this));	// record self to be used by static callback functions
		m_BrushTypeChoice->menu(StrokeDirectionTypeMenu);
		m_BrushTypeChoice->callback(cb_strokeDiretionChoice);

		m_ClearCanvasButton = new Fl_Button(240,10,150,25,"&Clear Canvas");
		m_ClearCanvasButton->user_data((void*)(this));
		m_ClearCanvasButton->callback(cb_clear_canvas_button);

		m_DisplayEdgeButton = new Fl_Button(325, 230, 50, 25, "&Do It");
		m_DisplayEdgeButton->user_data((void*)(this));
		m_DisplayEdgeButton->callback(cb_display_edge_button);

		// add a light button to brush dialog
		m_EdgeClipping= new Fl_Light_Button(10, 200, 150, 25, "&Edge Clipping");
		m_EdgeClipping->user_data((void*)(this));   // record self to be used by static callback functions
		m_EdgeClipping->callback(cb_edge_clipping);

		// add a light button to brush dialog
		m_AnotherGradient = new Fl_Light_Button(240, 200, 150, 25, "&Another Gradient");
		m_AnotherGradient->user_data((void*)(this));   // record self to be used by static callback functions
		m_AnotherGradient->callback(cb_another_gradient);


		// Add brush size slider to the dialog 
		m_BrushSizeSlider = new Fl_Value_Slider(10, 80, 300, 20, "Size");
		m_BrushSizeSlider->user_data((void*)(this));	// record self to be used by static callback functions
		m_BrushSizeSlider->type(FL_HOR_NICE_SLIDER);
        m_BrushSizeSlider->labelfont(FL_COURIER);
        m_BrushSizeSlider->labelsize(12);
		m_BrushSizeSlider->minimum(1);
		m_BrushSizeSlider->maximum(40);
		m_BrushSizeSlider->step(1);
		m_BrushSizeSlider->value(m_nSize);
		m_BrushSizeSlider->align(FL_ALIGN_RIGHT);
		m_BrushSizeSlider->callback(cb_sizeSlides);

		//Alpha Blending Slider
		Fl_Value_Slider* alphaSlider = new Fl_Value_Slider(10, 170, 300, 20, "Alpha");
		alphaSlider->user_data((void*)(this));	// record self to be used by static callback functions
		alphaSlider->type(FL_HOR_NICE_SLIDER);
		alphaSlider->labelfont(FL_COURIER);
		alphaSlider->labelsize(12);
		alphaSlider->minimum(0);
		alphaSlider->maximum(1);
		alphaSlider->step(0.01);
		alphaSlider->value(m_nAlpha);
		alphaSlider->align(FL_ALIGN_RIGHT);
		alphaSlider->callback(cb_alphaSlides);

		m_LineWidthSlider = new Fl_Value_Slider(10, 110, 300, 20, "Line Width");
		m_LineWidthSlider->user_data((void*)(this));	// record self to be used by static callback functions
		m_LineWidthSlider->type(FL_HOR_NICE_SLIDER);
		m_LineWidthSlider->labelfont(FL_COURIER);
		m_LineWidthSlider->labelsize(12);
		m_LineWidthSlider->minimum(1);
		m_LineWidthSlider->maximum(40);
		m_LineWidthSlider->step(1);
		m_LineWidthSlider->value(m_nLineWidth);     
		m_LineWidthSlider->align(FL_ALIGN_RIGHT);
		m_LineWidthSlider->callback(cb_LineWidthSlides);

		m_LineAngleSlider = new Fl_Value_Slider(10, 140, 300, 20, "Line Angle");
		m_LineAngleSlider->user_data((void*)(this));	// record self to be used by static callback functions
		m_LineAngleSlider->type(FL_HOR_NICE_SLIDER);
		m_LineAngleSlider->labelfont(FL_COURIER);
		m_LineAngleSlider->labelsize(12);
		m_LineAngleSlider->minimum(1);
		m_LineAngleSlider->maximum(359);
		m_LineAngleSlider->step(1);
		m_LineAngleSlider->value(m_nLineAngle);    
		m_LineAngleSlider->align(FL_ALIGN_RIGHT);
		m_LineAngleSlider->callback(cb_LineAngleSlides);

		m_EdgeThresholdSlider = new Fl_Value_Slider(10, 230, 200, 20, "Edge Threshold");
		m_EdgeThresholdSlider->user_data((void*)(this));	// record self to be used by static callback functions
		m_EdgeThresholdSlider->type(FL_HOR_NICE_SLIDER);
		m_EdgeThresholdSlider->labelfont(FL_COURIER);
		m_EdgeThresholdSlider->labelsize(12);
		m_EdgeThresholdSlider->minimum(0);
		m_EdgeThresholdSlider->maximum(500);
		m_EdgeThresholdSlider->step(1);
		m_EdgeThresholdSlider->value(m_nThreshold);
		m_EdgeThresholdSlider->align(FL_ALIGN_RIGHT);
		m_EdgeThresholdSlider->callback(cb_edge_threshold_slides);
    m_brushDialog->end();	

	//BW5; Manipulate Color Dialog Definition
	m_manipulateColorDialog = new Fl_Window(400, 100, "Manipulate Color Dialog for BW5");
		//Red
		Fl_Value_Slider* redSlider = new Fl_Value_Slider(10, 10, 300, 20, "Red Scale");
		redSlider->user_data((void*)(this));	// record self to be used by static callback functions
		redSlider->type(FL_HOR_NICE_SLIDER);
		redSlider->labelfont(FL_COURIER);
		redSlider->labelsize(12);
		redSlider->minimum(0);
		redSlider->maximum(1);
		redSlider->step(0.01);
		redSlider->value(1.00);
		redSlider->align(FL_ALIGN_RIGHT);
		redSlider->callback(cb_rgbScaleSlides);
		//Red
		Fl_Value_Slider* greenSlider = new Fl_Value_Slider(10, 40, 300, 20, "Green Scale");
		greenSlider->user_data((void*)(this));	// record self to be used by static callback functions
		greenSlider->type(FL_HOR_NICE_SLIDER);
		greenSlider->labelfont(FL_COURIER);
		greenSlider->labelsize(12);
		greenSlider->minimum(0);
		greenSlider->maximum(1);
		greenSlider->step(0.01);
		greenSlider->value(1.00);
		greenSlider->align(FL_ALIGN_RIGHT);
		greenSlider->callback(cb_rgbScaleSlides);
		//Red
		Fl_Value_Slider* blueSlider = new Fl_Value_Slider(10, 70, 300, 20, "Blue Scale");
		blueSlider->user_data((void*)(this));	// record self to be used by static callback functions
		blueSlider->type(FL_HOR_NICE_SLIDER);
		blueSlider->labelfont(FL_COURIER);
		blueSlider->labelsize(12);
		blueSlider->minimum(0);
		blueSlider->maximum(1);
		blueSlider->step(0.01);
		blueSlider->value(1.00);
		blueSlider->align(FL_ALIGN_RIGHT);
		blueSlider->callback(cb_rgbScaleSlides);
	m_manipulateColorDialog->end();

	//BW9; PaintView Background Dialog Definition
	m_paintviewBgdDialog = new Fl_Window(400, 40, "PaintView Backgroud Dialog for BW9");
		Fl_Value_Slider* bgdSlider = new Fl_Value_Slider(10, 10, 300, 20, "Background Blending Scale");
		bgdSlider->user_data((void*)(this));	// record self to be used by static callback functions
		bgdSlider->type(FL_HOR_NICE_SLIDER);
		bgdSlider->labelfont(FL_COURIER);
		bgdSlider->labelsize(12);
		bgdSlider->minimum(0);
		bgdSlider->maximum(1);
		bgdSlider->step(0.01);
		bgdSlider->value(0.00);
		bgdSlider->align(FL_ALIGN_RIGHT);
		bgdSlider->callback(cb_bgdScaleSlides);
	m_paintviewBgdDialog->end();

	m_convDialog = new Fl_Window(350, 100, "Filter Kernel Design for BW15");
		//Choose Filter Size
		Fl_Choice* filterSizeChoice = new Fl_Choice(80, 10, 40, 20, "&Filter Size");
		filterSizeChoice->user_data((void*)(this));	 // record self to be used by static callback functions
		filterSizeChoice->menu(filterSizeMenu);
		filterSizeChoice->callback(cb_filter_size);
		//Choose: Normalize or not
		Fl_Choice* nmChoice = new Fl_Choice(220, 10, 100, 20, "&Normalize?");
		nmChoice->user_data((void*)(this));	 // record self to be used by static callback functions
		nmChoice->menu(normalize_or_not_Menu);
		nmChoice->callback(cb_filter_nm);
		//Apply Button
		Fl_Button* applyButton = new Fl_Button(10, 70, 50, 20, "&Apply");
		applyButton->user_data((void*)(this));   // record self to be used by static callback functions
		applyButton->callback(cb_filter_apply);
		//Restore Button
		Fl_Button* restoreButton = new Fl_Button(80, 70, 90, 20, "&Restore");
		restoreButton->user_data((void*)(this));   // record self to be used by static callback functions
		restoreButton->callback(cb_filter_restore);
		//Integer Input
		Fl_Input* matrixInput = new Fl_Input(10, 40, 150, 20);
		matrixInput->user_data((void*)(this));
		matrixInput->callback(cb_save_matrix_text);

	m_convDialog->end();
}