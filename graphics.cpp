/************************************************************
Grafika OpenGL
*************************************************************/
#include <windows.h>
#include <gl\gl.h>
#include <gl\glu.h>
#include <iterator> 
#include <map>
#include <time.h>

#include "graphics.h"
//#include "vector3D.h"
//#include "quaternion.h"
#ifndef _OBJECTS__H
#include "objects.h"
#endif
using namespace std;

extern int druzyny[9][3];
extern FILE *f;
extern MovableObject *my_vehicle;               // obiekt przypisany do tej aplikacji 
extern map<int, MovableObject*> network_vehicles;
extern CRITICAL_SECTION m_cs;
extern Terrain terrain;

extern long czas_istnienia_grupy;
extern long czas_start;

int g_GLPixelIndex = 0;
HGLRC g_hGLContext = NULL;
unsigned int font_base;

ParametryWidoku par_wid;

extern void TworzListyWyswietlania();		// definiujemy listy tworzące labirynt
extern void RysujGlobalnyUkladWsp();

void UstawienieStandardowychParWidoku(ParametryWidoku *p)
{
	p->pocz_kierunek_kamery = Wektor3(0, -3, -11);   // kierunek patrzenia
	p->pocz_pol_kamery = Wektor3(30, 3, 0);          // po³o¿enie kamery
	p->pocz_pion_kamery = Wektor3(0, 1, 0);           // kierunek pionu kamery             

	// Zmienne - ustawiane przez użytkownika
	p->sledzenie = 1;                             // tryb œledzenia obiektu przez kamerê
	p->widok_z_gory = 0;                          // tryb widoku z gory
	p->oddalenie = 14.0;                          // oddalenie lub przybli¿enie kamery
	p->zoom = 1.0;                               // zmiana kąta widzenia
	p->kat_kam_z = 0;                            // obrót kamery góra-dół

	p->przes_w_prawo = 0;                        // przesunięcie kamery w prawo (w lewo o wart. ujemnej) - chodzi głównie o tryb edycji
	p->przes_w_dol = 0;                          // przesunięcie do dołu (w górę o wart. ujemnej)          i widok z góry (klawisz Q)  
}

int InicjujGrafike(HDC g_context)
{

	if (SetWindowPixelFormat(g_context) == FALSE)
		return FALSE;

	if (CreateViewGLContext(g_context) == FALSE)
		return 0;
	BuildFont(g_context);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	UstawienieStandardowychParWidoku(&par_wid);

	TworzListyWyswietlania();		// definiujemy listy tworzące różne elementy sceny
	terrain.PoczatekGrafiki();
}

// Ustwienia kamery w zależności od wartości początkowych (w interakcji), wartości ustawianych
// przez użytkowika oraz stanu obiektu (np. gdy tryb śledzenia)
void UstawieniaKamery(Wektor3 *polozenie, Wektor3 *kierunek, Wektor3 *pion, ParametryWidoku pw)
{
	if (pw.sledzenie)  // kamera ruchoma - porusza się wraz z obiektem
	{
		(*kierunek) = my_vehicle->qOrient.obroc_wektor(Wektor3(1, 0, 0));
		(*pion) = my_vehicle->qOrient.obroc_wektor(Wektor3(0, 1, 0));
		Wektor3 prawo_kamery = my_vehicle->qOrient.obroc_wektor(Wektor3(0, 0, 1));

		(*pion) = (*pion).obrot(pw.kat_kam_z, prawo_kamery.x, prawo_kamery.y, prawo_kamery.z);
		(*kierunek) = (*kierunek).obrot(pw.kat_kam_z, prawo_kamery.x, prawo_kamery.y, prawo_kamery.z);
		(*polozenie) = my_vehicle->wPol - (*kierunek)*my_vehicle->dlugosc * 0 +
			(*pion).znorm()*my_vehicle->wysokosc * 5;
		if (pw.widok_z_gory)
		{
			(*pion) = (*kierunek);
			(*kierunek) = Wektor3(0, -1, 0);
			(*polozenie) = (*polozenie) + Wektor3(0, 100, 0) + (*pion)*pw.przes_w_dol + (*pion)*(*kierunek)*pw.przes_w_prawo;
		}
	}
	else // bez śledzenia - kamera nie podąża wraz z pojazdem
	{
		(*pion) = pw.pocz_pion_kamery;
		(*kierunek) = pw.pocz_kierunek_kamery;
		(*polozenie) = pw.pocz_pol_kamery;
		Wektor3 prawo_kamery = ((*kierunek)*(*pion)).znorm();
		(*pion) = (*pion).obrot(pw.kat_kam_z / 20, prawo_kamery.x, prawo_kamery.y, prawo_kamery.z);
		(*kierunek) = (*kierunek).obrot(pw.kat_kam_z / 20, prawo_kamery.x, prawo_kamery.y, prawo_kamery.z);
		if (pw.widok_z_gory)
		{
			(*pion) = Wektor3(0, 0, -1);
			(*kierunek) = Wektor3(0, -1, 0.02);
			(*polozenie) = pw.pocz_pol_kamery + Wektor3(0, 100, 0) + (*pion)*pw.przes_w_dol + (*pion)*(*kierunek)*pw.przes_w_prawo;
		}
	}
}


void RysujScene()
{
	float czas = (float)(czas_istnienia_grupy + clock() - czas_start) / CLOCKS_PER_SEC;  // czas od uruchomienia w [s]
	GLfloat OwnObjectColor[] = { 0.9f, 0.2f, 0.5f, 0.9f };
	GLfloat BlueSurfaceTr[] = { 0.9f, 0.2f, 0.5f, 0.6f };

	GLfloat Kolory[][4] = { { 0.0f, 0.0f, 0.0f, 0.5f },{ 1.0f, 1.0f, 1.0f, 0.5f },{ 1.0f, 1.0f, 0.0f, 0.5f },{ 0.0f, 1.0f, 0.0f, 0.5f }
						   ,{ 1.0f, 0.0f, 0.0f, 0.5f },{ 0.0f, 1.0f, 1.0f, 0.5f },{ 1.0f, 0.0f, 1.0f, 0.5f },{ 1.0f, 1.0f, 0.5f, 0.5f } 
						   ,{ 1.0f, 0.5f, 0.0f, 0.5f },{ 0.5f, 0.5f, 0.5f, 0.5f } };

	GLfloat NetworkVehiclesColor[] = { 0.8f, 0.2f, 0.8f, 0.7f };
	GLfloat RedSurface[] = { 0.8f, 0.2f, 0.1f, 0.5f };
	GLfloat OrangeSurface[] = { 1.0f, 0.8f, 0.0f, 0.7f };
	GLfloat GroundSurface[] = { 0.55f, 0.42f, 0.3f, 1.0f };
	GLfloat YellowSurface[] = { 0.75f, 0.75f, 0.0f, 1.0f };
	GLfloat YellowLight[] = { 2.0f, 2.0f, 1.0f, 1.0f };

	GLfloat LightAmbient[] = { 0.1f, 0.1f, 0.1f, 0.1f };
	GLfloat LightDiffuse[] = { 0.7f, 0.7f, 0.7f, 0.7f };

	GLfloat LightPosition[] = { 10.0*cos(czas / 5), 5.0f, 10.0*sin(czas / 5), 0.0f };


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);		//1 składowa: światło otaczające (bezkierunkowe)
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);		//2 składowa: światło rozproszone (kierunkowe)
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT0);

	//glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, YellowLight);


	glLoadIdentity();
	glClearColor(0.3, 0.3, 0.7, 0.8);   // ustawienie nieczarnego koloru tła
	glTranslatef(-24, 24, -40);
	glRasterPos2f(4.0, -4.0);
	glPrint("%s", par_wid.napis1);
	glPrint("%s", par_wid.napis2);
	glLoadIdentity();


	Wektor3 pol_k, kierunek_k, pion_k;

	UstawieniaKamery(&pol_k, &kierunek_k, &pion_k, par_wid);

	gluLookAt(pol_k.x - par_wid.oddalenie*kierunek_k.x,
		pol_k.y - par_wid.oddalenie*kierunek_k.y, pol_k.z - par_wid.oddalenie*kierunek_k.z,
		pol_k.x + kierunek_k.x, pol_k.y + kierunek_k.y, pol_k.z + kierunek_k.z,
		pion_k.x, pion_k.y, pion_k.z);

	//glRasterPos2f(0.30,-0.27);
	//glPrint("MojObiekt->iID = %d",my_vehicle->iID ); 

	RysujGlobalnyUkladWsp();

	int dw = 0, dk = 0;
	if (terrain.czy_toroidalnosc)
	{
		if (terrain.granica_x > 0) dk = 1;
		if (terrain.granica_z > 0) dw = 1;
	}

	for (int w = -dw; w < 1 + dw; w++)
		for (int k = -dk; k < 1 + dk; k++)
		{
			glPushMatrix();

			glTranslatef(terrain.granica_x*k, 0, terrain.granica_z*w);

			GLfloat *kolor;
			kolor = Kolory[9];
			for (int i = 0; i < 9; i++)
				if (druzyny[i][0] != 0)
					for (int j = 0; j < 3; j++)
						if (druzyny[i][j] == my_vehicle->iID)
							kolor = Kolory[i];
			kolor[3] = 1.0f;
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, kolor);
			glEnable(GL_BLEND);

			my_vehicle->Rysuj();

			// Lock the Critical section
			EnterCriticalSection(&m_cs);
			for (map<int, MovableObject*>::iterator it = network_vehicles.begin(); it != network_vehicles.end(); ++it)
			{
				kolor = Kolory[9];
				if (it->second)
				{
					for (int i = 0; i < 9; i++)
						if (druzyny[i][0] != 0)
							for (int j = 0; j < 3; j++)
								if (druzyny[i][j] == it->second->iID)
									kolor = Kolory[i];

					it->second->iID;
					glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, kolor);
					it->second->Rysuj();
				}
			}
			//Release the Critical section
			LeaveCriticalSection(&m_cs);
			
			glDisable(GL_BLEND);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GroundSurface);
			
			terrain.Rysuj();
			glPopMatrix();
		}

	//glPopMatrix();	
	glFlush();
}




void ZmianaRozmiaruOkna(int cx, int cy)
{
	GLsizei width, height;
	GLdouble aspect;
	width = cx;
	height = cy;

	if (cy == 0)
		aspect = (GLdouble)width;
	else
		aspect = (GLdouble)width / (GLdouble)height;

	glViewport(0, 0, width, height);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(55 * par_wid.zoom, aspect, 1, 1000000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDrawBuffer(GL_BACK);

	glEnable(GL_LIGHTING);

	glEnable(GL_DEPTH_TEST);

}

Wektor3 WspolrzedneKursora3D(int x, int y) // współrzędne 3D punktu na obrazie 2D
{
	//  pobranie macierz modelowania
	GLdouble model[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model);

	// pobranie macierzy rzutowania
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX, proj);

	// pobranie obszaru renderingu
	GLint view[4];
	glGetIntegerv(GL_VIEWPORT, view);

	// tablice ze odczytanymi współrzędnymi w przestrzeni widoku
	GLdouble wsp[3];

	//RECT rc;
	//GetClientRect (okno, &rc);

	GLdouble wsp_z;

	int wynik = gluProject(0, 0, 0, model, proj, view, wsp + 0, wsp + 1, &wsp_z);

	gluUnProject(x, y, wsp_z, model, proj, view, wsp + 0, wsp + 1, wsp + 2);
	//gluUnProject (x,rc.bottom - y,wsp_z ,model,proj,view,wsp+0,wsp+1,wsp+2);
	return Wektor3(wsp[0], wsp[1], wsp[2]);
}

Wektor3 WspolrzedneKursora3D(int x, int y, float wysokosc) // współrzędne 3D punktu na obrazie 2D
{
	glTranslatef(0, wysokosc, 0);
	//  pobranie macierz modelowania
	GLdouble model[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model);

	// pobranie macierzy rzutowania
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX, proj);

	// pobranie obszaru renderingu
	GLint view[4];
	glGetIntegerv(GL_VIEWPORT, view);

	// tablice ze odczytanymi współrzędnymi w przestrzeni widoku
	GLdouble wsp[3];

	//RECT rc;
	//GetClientRect (okno, &rc);

	GLdouble wsp_z;

	int wynik = gluProject(0, 0, 0, model, proj, view, wsp + 0, wsp + 1, &wsp_z);
	gluUnProject(x, y, wsp_z, model, proj, view, wsp + 0, wsp + 1, wsp + 2);
	glTranslatef(0, -wysokosc, 0);
	//gluUnProject (x,rc.bottom - y,wsp_z ,model,proj,view,wsp+0,wsp+1,wsp+2);
	return Wektor3(wsp[0], wsp[1], wsp[2]);
}

void WspolrzedneEkranu(float *xx, float *yy, float *zz, Wektor3 Punkt3D) // współrzędne punktu na ekranie na podstawie wsp 3D
{
	//  pobranie macierz modelowania
	GLdouble model[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model);
	// pobranie macierzy rzutowania
	GLdouble proj[16];
	glGetDoublev(GL_PROJECTION_MATRIX, proj);
	// pobranie obszaru renderingu
	GLint view[4];
	glGetIntegerv(GL_VIEWPORT, view);
	// tablice ze odczytanymi współrzędnymi w przestrzeni widoku
	GLdouble wsp[3], wsp_okn[3];
	GLdouble liczba;
	int wynik = gluProject(Punkt3D.x, Punkt3D.y, Punkt3D.z, model, proj, view, wsp_okn + 0, wsp_okn + 1, wsp_okn + 2);
	gluUnProject(wsp_okn[0], wsp_okn[1], wsp_okn[2], model, proj, view, wsp + 0, wsp + 1, wsp + 2);
	//fprintf(f,"   Wsp. punktu 3D = (%f, %f, %f), wspolrzedne w oknie = (%f, %f, %f), wsp. punktu = (%f, %f, %f)\n",
	//  Punkt3D.x,Punkt3D.y,Punkt3D.z,  wsp_okn[0],wsp_okn[1],wsp_okn[2],  wsp[0],wsp[1],wsp[2]);

	(*xx) = wsp_okn[0]; (*yy) = wsp_okn[1]; (*zz) = wsp_okn[2];
}

void ZakonczenieGrafiki()
{
	if (wglGetCurrentContext() != NULL)
	{
		// dezaktualizacja kontekstu renderującego
		wglMakeCurrent(NULL, NULL);
	}
	if (g_hGLContext != NULL)
	{
		wglDeleteContext(g_hGLContext);
		g_hGLContext = NULL;
	}
	glDeleteLists(font_base, 96);
}

BOOL SetWindowPixelFormat(HDC hDC)
{
	PIXELFORMATDESCRIPTOR pixelDesc;

	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pixelDesc.nVersion = 1;
	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER | PFD_STEREO_DONTCARE;
	pixelDesc.iPixelType = PFD_TYPE_RGBA;
	pixelDesc.cColorBits = 32;
	pixelDesc.cRedBits = 8;
	pixelDesc.cRedShift = 16;
	pixelDesc.cGreenBits = 8;
	pixelDesc.cGreenShift = 8;
	pixelDesc.cBlueBits = 8;
	pixelDesc.cBlueShift = 0;
	pixelDesc.cAlphaBits = 0;
	pixelDesc.cAlphaShift = 0;
	pixelDesc.cAccumBits = 64;
	pixelDesc.cAccumRedBits = 16;
	pixelDesc.cAccumGreenBits = 16;
	pixelDesc.cAccumBlueBits = 16;
	pixelDesc.cAccumAlphaBits = 0;
	pixelDesc.cDepthBits = 32;
	pixelDesc.cStencilBits = 8;
	pixelDesc.cAuxBuffers = 0;
	pixelDesc.iLayerType = PFD_MAIN_PLANE;
	pixelDesc.bReserved = 0;
	pixelDesc.dwLayerMask = 0;
	pixelDesc.dwVisibleMask = 0;
	pixelDesc.dwDamageMask = 0;
	g_GLPixelIndex = ChoosePixelFormat(hDC, &pixelDesc);

	if (g_GLPixelIndex == 0)
	{
		g_GLPixelIndex = 1;

		if (DescribePixelFormat(hDC, g_GLPixelIndex, sizeof(PIXELFORMATDESCRIPTOR), &pixelDesc) == 0)
		{
			return FALSE;
		}
	}

	if (SetPixelFormat(hDC, g_GLPixelIndex, &pixelDesc) == FALSE)
	{
		return FALSE;
	}

	return TRUE;
}
BOOL CreateViewGLContext(HDC hDC)
{
	g_hGLContext = wglCreateContext(hDC);

	if (g_hGLContext == NULL)
	{
		return FALSE;
	}

	if (wglMakeCurrent(hDC, g_hGLContext) == FALSE)
	{
		return FALSE;
	}

	return TRUE;
}

GLvoid BuildFont(HDC hDC)								// Build Our Bitmap Font
{
	HFONT	font;										// Windows Font ID
	HFONT	oldfont;									// Used For Good House Keeping

	font_base = glGenLists(96);								// Storage For 96 Characters

	font = CreateFont(-14,							// Height Of Font
		0,								// Width Of Font
		0,								// Angle Of Escapement
		0,								// Orientation Angle
		FW_NORMAL,						// Font Weight
		FALSE,							// Italic
		FALSE,							// Underline
		FALSE,							// Strikeout
		ANSI_CHARSET,					// Character Set Identifier
		OUT_TT_PRECIS,					// Output Precision
		CLIP_DEFAULT_PRECIS,			// Clipping Precision
		ANTIALIASED_QUALITY,			// Output Quality
		FF_DONTCARE | DEFAULT_PITCH,		// Family And Pitch
		"Courier New");					// Font Name

	oldfont = (HFONT)SelectObject(hDC, font);           // Selects The Font We Want
	wglUseFontBitmaps(hDC, 31, 96, font_base);				// Builds 96 Characters Starting At Character 32
	SelectObject(hDC, oldfont);							// Selects The Font We Want
	DeleteObject(font);									// Delete The Font
}

// Napisy w OpenGL
GLvoid glPrint(const char *fmt, ...)	// Custom GL "Print" Routine
{
	char		text[256];	// Holds Our String
	va_list		ap;		// Pointer To List Of Arguments

	if (fmt == NULL)		// If There's No Text
		return;			// Do Nothing

	va_start(ap, fmt);		// Parses The String For Variables
	vsprintf(text, fmt, ap);	// And Converts Symbols To Actual Numbers
	va_end(ap);			// Results Are Stored In Text

	glPushAttrib(GL_LIST_BIT);	// Pushes The Display List Bits
	glListBase(font_base - 31);		// Sets The Base Character to 32
	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text);	// Draws The Display List Text
	glPopAttrib();			// Pops The Display List Bits
}


void TworzListyWyswietlania()
{
	glNewList(Wall1, GL_COMPILE);	// GL_COMPILE - lista jest kompilowana, ale nie wykonywana

	glBegin(GL_QUADS);		// inne opcje: GL_POINTS, GL_LINES, GL_LINE_STRIP, GL_LINE_LOOP
	// GL_TRIANGLES, GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_QUAD_STRIP, GL_POLYGON
	glNormal3f(-1.0, 0.0, 0.0);
	glVertex3f(-1.0, -1.0, 1.0);
	glVertex3f(-1.0, 1.0, 1.0);
	glVertex3f(-1.0, 1.0, -1.0);
	glVertex3f(-1.0, -1.0, -1.0);
	glEnd();
	glEndList();

	glNewList(Wall2, GL_COMPILE);
	glBegin(GL_QUADS);
	glNormal3f(1.0, 0.0, 0.0);
	glVertex3f(1.0, -1.0, 1.0);
	glVertex3f(1.0, 1.0, 1.0);
	glVertex3f(1.0, 1.0, -1.0);
	glVertex3f(1.0, -1.0, -1.0);
	glEnd();
	glEndList();

	/*glNewList(Floor,GL_COMPILE);
	glBegin(GL_POLYGON);
	glNormal3f( 0.0, 1.0, 0.0);
	glVertex3f( -100, 0, -300.0);
	glVertex3f( -100, 0, 100.0);
	glVertex3f( 100, 0, 100.0);
	glVertex3f( 100, 0, -300.0);
	glEnd();
	glEndList();*/

	glNewList(Cube, GL_COMPILE);
	glBegin(GL_QUADS);
	// przod
	glNormal3f(0.0, 0.0, 1.0);
	glVertex3f(0, 0, 1);
	glVertex3f(0, 1, 1);
	glVertex3f(1, 1, 1);
	glVertex3f(1, 0, 1);
	// tyl
	glNormal3f(0.0, 0.0, -1.0);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(0, 1, 0);
	// gora
	glNormal3f(0.0, 1.0, 0.0);
	glVertex3f(0, 1, 0);
	glVertex3f(0, 1, 1);
	glVertex3f(1, 1, 1);
	glVertex3f(1, 1, 0);
	// dol
	glNormal3f(0.0, -1.0, 0.0);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(1, 0, 1);
	glVertex3f(0, 0, 1);
	// prawo
	glNormal3f(1.0, 0.0, 0.0);
	glVertex3f(1, 0, 0);
	glVertex3f(1, 0, 1);
	glVertex3f(1, 1, 1);
	glVertex3f(1, 1, 0);
	// lewo
	glNormal3f(-1.0, 0.0, 0.0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(0, 1, 1);
	glVertex3f(0, 0, 1);

	glEnd();
	glEndList();

	glNewList(Cube_skel, GL_COMPILE);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(0, 0, 1);
	glVertex3f(1, 0, 1);
	glVertex3f(0, 1, 1);
	glVertex3f(1, 1, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(0, 0, 1);
	glVertex3f(0, 1, 1);
	glVertex3f(1, 0, 1);
	glVertex3f(1, 1, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glVertex3f(1, 0, 0);
	glVertex3f(1, 0, 1);
	glVertex3f(0, 1, 0);
	glVertex3f(0, 1, 1);
	glVertex3f(1, 1, 0);
	glVertex3f(1, 1, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(0, 0, 1);
	glVertex3f(1, 0, 1);
	glVertex3f(0, 1, 1);
	glVertex3f(1, 1, 1);

	glEnd();
	glEndList();
}


void RysujGlobalnyUkladWsp(void)
{

	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(2, 0, 0);
	glVertex3f(2, -0.25, 0.25);
	glVertex3f(2, 0.25, -0.25);
	glVertex3f(2, -0.25, -0.25);
	glVertex3f(2, 0.25, 0.25);

	glEnd();
	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 2, 0);
	glVertex3f(0, 2, 0);
	glVertex3f(0.25, 2, 0);
	glVertex3f(0, 2, 0);
	glVertex3f(-0.25, 2, 0.25);
	glVertex3f(0, 2, 0);
	glVertex3f(-0.25, 2, -0.25);

	glEnd();
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 2);
	glVertex3f(-0.25, -0.25, 2);
	glVertex3f(0.25, 0.25, 2);
	glVertex3f(-0.25, -0.25, 2);
	glVertex3f(0.25, -0.25, 2);
	glVertex3f(-0.25, 0.25, 2);
	glVertex3f(0.25, 0.25, 2);

	glEnd();

	glColor3f(1, 1, 1);
}

