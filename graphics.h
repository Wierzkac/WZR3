#include <gl\gl.h>
#include <gl\glu.h>
#ifndef _WEKTOR__H
  #include "vector3D.h"
#endif


enum GLDisplayListNames
{
	Wall1 = 1,
	Wall2 = 2,
	Floor = 3,
	Cube = 4,
	Cube_skel = 5,
	PowierzchniaTerrainu = 6
};

// Stale (wartości początkowe)

struct ParametryWidoku
{
	Wektor3 pocz_kierunek_kamery;
	Wektor3 pocz_pol_kamery;
	Wektor3 pocz_pion_kamery;
	bool sledzenie;
	bool widok_z_gory;
	float oddalenie;
	float zoom;
	float kat_kam_z;
	float przes_w_prawo;                        // przesunięcie kamery w prawo (w lewo o wart. ujemnej) - chodzi głównie o tryb edycji
	float przes_w_dol;                          // przesunięcie do dołu (w górę o wart. ujemnej)          i widok z góry (klawisz Q)  
	char napis1[512], napis2[512];
};

void UstawienieStandardowychParWidoku(ParametryWidoku *p);

// Ustwienia kamery w zależności od wartości początkowych (w interakcji), wartości ustawianych
// przez użytkowika oraz stanu obiektu (np. gdy tryb śledzenia)
void UstawieniaKamery(Wektor3 *polozenie, Wektor3 *kierunek, Wektor3 *pion, ParametryWidoku pw);

int InicjujGrafike(HDC g_context);
void RysujScene();
void ZmianaRozmiaruOkna(int cx,int cy);

Wektor3 WspolrzedneKursora3D(int x, int y); // współrzędne 3D punktu na obrazie 2D
Wektor3 WspolrzedneKursora3D(int x, int y, float wysokosc); // współrzędne 3D punktu na obrazie 2D
void WspolrzedneEkranu(float *xx, float *yy, float *zz, Wektor3 Punkt3D); // współrzędne punktu na ekranie na podstawie wsp 3D

void ZakonczenieGrafiki();

BOOL SetWindowPixelFormat(HDC hDC);
BOOL CreateViewGLContext(HDC hDC);
GLvoid BuildFont(HDC hDC);
GLvoid glPrint(const char *fmt, ...);