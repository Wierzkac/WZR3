/*********************************************************************
Symulacja obiektów fizycznych ruchomych np. samochody, statki, roboty, itd.
+ obs³uga obiektów statycznych np. terrain.
**********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <windows.h>
#include <gl\gl.h>
#include <gl\glu.h>
#include <iterator> 
#include <map>
using namespace std;
#ifndef _OBJECTS__H
#include "objects.h"
#endif
#include "graphics.h"


//#include "vector3D.h"
extern FILE *f;
extern HWND okno;

extern ParametryWidoku par_wid;
extern map<int, MovableObject*> network_vehicles;
extern CRITICAL_SECTION m_cs;


extern bool tryb_edycji_terrainu;
//extern Terrain terrain;
//extern int iLiczbaCudzychOb;
//extern MovableObject *CudzeObiekty[1000]; 

//enum TYpyPrzedm { PRZ_MONETA, PRZ_BECZKA, PRZ_DRZEWO, PRZ_BUDYNEK, PRZ_PUNKT, PRZ_KRAWEDZ };
char *PRZ_nazwy[32] = { "moneta", "beczka", "drzewo", "punkt", "krawedz" };
//enum PodtypyDrzew { DRZ_TOPOLA, DRZ_SWIERK, DRZ_BAOBAB, DRZ_FANTAZJA };
char *DRZ_nazwy[32] = { "topola", "swierk", "baobab", "fantazja" };

unsigned long __log2(unsigned long x)  // w starszej wersji Visuala (< 2013) nie ma funkcji log2() w bibliotece math.h  !!!
{
	long i = -1;
	long y = x;
	while (y > 0){
	 	y = y >> 1;
		i++;
	}
	return i;
}

MovableObject::MovableObject(Terrain *t)             // konstruktor                   
{

	terrain = t;

	//iID = (unsigned int)(clock() % 1000);  // identyfikator obiektu
	iID = (unsigned int)(rand() % 1000);  // identyfikator obiektu
	fprintf(f, "Nowy obiekt: iID = %d\n", iID);
	iID_wlasc = iID;           // identyfikator w³aœciciela obiektu
	czy_autonom = 0;

	pieniadze = 1000;    // np. dolarów
	ilosc_paliwa = 10.0;   // np. kilogramów paliwa 

	czas_symulacji = 0;    // symulowany czas rzeczywisty od pocz¹tku istnienia obiektu 

	F = alfa = 0;	      // si³y dzia³aj¹ce na obiekt 
	ham = 0;			      // stopieñ hamowania

	F_max = 6000;
	alfa_max = PI*45.0 / 180;
	masa_wlasna = 800.0;     // masa w³asna obiektu [kg] (bez paliwa)
	masa_calk = masa_wlasna + ilosc_paliwa;  // masa ca³kowita
	Fy = masa_wlasna*9.81;    // si³a nacisku na podstawê obiektu (na ko³a pojazdu)
	dlugosc = 6.0;
	szerokosc = 2.7;
	wysokosc = 1.0;
	przeswit = 0.0;           // wysokoœæ na której znajduje siê podstawa obiektu
	dl_przod = 1.0;           // odleg³oœæ od przedniej osi do przedniego zderzaka 
	dl_tyl = 0.2;             // odleg³oœæ od tylniej osi do tylniego zderzaka

	iID_kolid = -1;           // na razie brak kolizji 

	//wPol.y = przeswit+wysokosc/2 + 10;
	wPol = Wektor3(220, przeswit + wysokosc / 2 + 10, -50);
	promien = sqrt(dlugosc*dlugosc + szerokosc*szerokosc + wysokosc*wysokosc) / 2 / 1.15;
	//wV_kat = Wektor3(0,1,0)*40;  // pocz¹tkowa prêdkoœæ k¹towa (w celach testowych)

	//moment_wziecia = 0;            // czas ostatniego wziecia przedmiotu
	//czas_oczekiwania = 1000000000; // 
	//nr_przedmiotu = -1;

	nr_wzietego_przedm = -1;
	wartosc_wzieta = 0;
	nr_odnowionego_przedm = -1;

	// obrót obiektu o k¹t 30 stopni wzglêdem osi y:
	kwaternion qObr = AsixToQuat(Wektor3(0, 1, 0), -40 * PI / 180.0);
	qOrient = qObr*qOrient;

	// losowanie umiejêtnoœci tak by nie by³o bardzo s³abych i bardzo silnych:
	umiejetn_sadzenia = 0.2 + (float)(rand() % 5) / 5;
	umiejetn_zb_paliwa = 0.2 + (float)(rand() % 5) / 5;
	umiejetn_zb_monet = 0.2 + (float)(rand() % 5) / 5;
	//float suma_um = umiejetn_sadzenia + umiejetn_zb_paliwa + umiejetn_zb_monet;
	//float suma_um_los = 0.7 + 0.8*(float)rand()/RAND_MAX;    // losuje umiejetnoœæ sumaryczn¹
	//umiejetn_sadzenia *= suma_um_los/suma_um;
	//umiejetn_zb_paliwa *= suma_um_los/suma_um;
	//umiejetn_zb_monet *= suma_um_los/suma_um;

	czy_zazn = false;
}

MovableObject::~MovableObject()            // destruktor
{
}

void MovableObject::ZmienStan(StanObiektu stan)  // przepisanie podanego stanu 
{                                                // w przypadku obiektów, które nie s¹ symulowane
	this->iID = stan.iID;

	this->wPol = stan.wPol;
	this->qOrient = stan.qOrient;
	this->wV = stan.wV;
	this->wA = stan.wA;
	this->wV_kat = stan.wV_kat;
	this->wA_kat = stan.wA_kat;

	this->masa_calk = stan.masa_calk;
	this->ilosc_paliwa = stan.ilosc_paliwa;
	this->pieniadze = stan.pieniadze;
	this->iID_wlasc = stan.iID_wlasc;
	this->czy_autonom = stan.czy_autonom;
}

StanObiektu MovableObject::Stan()                // metoda zwracaj¹ca stan obiektu ³¹cznie z iID
{
	StanObiektu stan;

	stan.iID = iID;
	stan.qOrient = qOrient;
	stan.wA = wA;
	stan.wA_kat = wA_kat;
	stan.wPol = wPol;
	stan.wV = wV;
	stan.wV_kat = wV_kat;

	stan.masa_calk = masa_calk;
	stan.ilosc_paliwa = ilosc_paliwa;
	stan.pieniadze = pieniadze;
	stan.iID_wlasc = iID_wlasc;
	stan.czy_autonom = czy_autonom;
	return stan;
}


void MovableObject::Symulacja(float dt)          // obliczenie nowego stanu na podstawie dotychczasowego,
{                                                // dzia³aj¹cych si³ i czasu, jaki up³yn¹³ od ostatniej symulacji

	if (dt == 0) return;

	czas_symulacji += dt;          // sumaryczny czas wszystkich symulacji obiektu od jego powstania

	float tarcie = 0.8;            // wspó³czynnik tarcia obiektu o pod³o¿e 
	float tarcie_obr = tarcie;     // tarcie obrotowe (w szczególnych przypadkach mo¿e byæ inne ni¿ liniowe)
	float tarcie_toczne = 0.30;    // wspó³czynnik tarcia tocznego
	float sprezystosc = 0.8;       // wspó³czynnik sprê¿ystoœci (0-brak sprê¿ystoœci, 1-doskona³a sprê¿ystoœæ) 
	float g = 9.81;                // przyspieszenie grawitacyjne
	float m = masa_wlasna + ilosc_paliwa;   // masa calkowita

	Wektor3 wPol_pop = wPol;       // zapisanie poprzedniego po³o¿enia

	// obracam uk³ad wspó³rzêdnych lokalnych wed³ug kwaterniona orientacji:
	Wektor3 w_przod = qOrient.obroc_wektor(Wektor3(1, 0, 0)); // na razie oœ obiektu pokrywa siê z osi¹ x globalnego uk³adu wspó³rzêdnych (lokalna oœ x)
	Wektor3 w_gora = qOrient.obroc_wektor(Wektor3(0, 1, 0));  // wektor skierowany pionowo w górê od podstawy obiektu (lokalna oœ y)
	Wektor3 w_prawo = qOrient.obroc_wektor(Wektor3(0, 0, 1)); // wektor skierowany w prawo (lokalna oœ z)

	//fprintf(f,"w_przod = (%f, %f, %f)\n",w_przod.x,w_przod.y,w_przod.z);
	//fprintf(f,"w_gora = (%f, %f, %f)\n",w_gora.x,w_gora.y,w_gora.z);
	//fprintf(f,"w_prawo = (%f, %f, %f)\n",w_prawo.x,w_prawo.y,w_prawo.z);

	//fprintf(f,"|w_przod|=%f,|w_gora|=%f,|w_prawo|=%f\n",w_przod.dlugosc(),w_gora.dlugosc(),w_prawo.dlugosc()  );
	//fprintf(f,"ilo skalar = %f,%f,%f\n",w_przod^w_prawo,w_przod^w_gora,w_gora^w_prawo  );
	//fprintf(f,"w_przod = (%f, %f, %f) w_gora = (%f, %f, %f) w_prawo = (%f, %f, %f)\n",
	//           w_przod.x,w_przod.y,w_przod.z,w_gora.x,w_gora.y,w_gora.z,w_prawo.x,w_prawo.y,w_prawo.z);


	// rzutujemy wV na sk³adow¹ w kierunku przodu i pozosta³e 2 sk³adowe
	// sk³adowa w bok jest zmniejszana przez si³ê tarcia, sk³adowa do przodu
	// przez si³ê tarcia tocznego
	Wektor3 wV_wprzod = w_przod*(wV^w_przod),
		wV_wprawo = w_prawo*(wV^w_prawo),
		wV_wgore = w_gora*(wV^w_gora);

	// dodatkowa normalizacja likwidujaca blad numeryczny:
	if (wV.dlugosc() > 0)
	{
		float blad_dlugosci = (wV_wprzod + wV_wprawo + wV_wgore).dlugosc() / wV.dlugosc();
		wV_wprzod = wV_wprzod / blad_dlugosci;
		wV_wprawo = wV_wprawo / blad_dlugosci;
		wV_wgore = wV_wgore / blad_dlugosci;
	}

	// rzutujemy prêdkoœæ k¹tow¹ wV_kat na sk³adow¹ w kierunku przodu i pozosta³e 2 sk³adowe
	Wektor3 wV_kat_wprzod = w_przod*(wV_kat^w_przod),
		wV_kat_wprawo = w_prawo*(wV_kat^w_prawo),
		wV_kat_wgore = w_gora*(wV_kat^w_gora);


	// ograniczenia 
	if (F > F_max) F = F_max;
	if (F < -F_max / 2) F = -F_max / 2;
	if (alfa > alfa_max) alfa = alfa_max;
	if (alfa < -alfa_max) alfa = -alfa_max;

	// obliczam promien skrêtu pojazdu na podstawie k¹ta skrêtu kó³, a nastêpnie na podstawie promienia skrêtu
	// obliczam prêdkoœæ k¹tow¹ (UPROSZCZENIE! pomijam przyspieszenie k¹towe oraz w³aœciw¹ trajektoriê ruchu)
	if (Fy > 0)
	{
		float V_kat_skret = 0;
		if (alfa != 0)
		{
			float Rs = sqrt(dlugosc*dlugosc / 4 + (fabs(dlugosc / tan(alfa)) + szerokosc / 2)*(fabs(dlugosc / tan(alfa)) + szerokosc / 2));
			V_kat_skret = wV_wprzod.dlugosc()*(1.0 / Rs);
		}
		Wektor3 wV_kat_skret = w_gora*V_kat_skret*(alfa > 0 ? 1 : -1);
		Wektor3 wV_kat_wgore2 = wV_kat_wgore + wV_kat_skret;
		if (wV_kat_wgore2.dlugosc() <= wV_kat_wgore.dlugosc()) // skrêt przeciwdzia³a obrotowi
		{
			if (wV_kat_wgore2.dlugosc() > V_kat_skret)
				wV_kat_wgore = wV_kat_wgore2;
			else
				wV_kat_wgore = wV_kat_skret;
		}
		else
		{
			if (wV_kat_wgore.dlugosc() < V_kat_skret)
				wV_kat_wgore = wV_kat_skret;

		}

		// tarcie zmniejsza prêdkoœæ obrotow¹ (UPROSZCZENIE! zamiast masy winienem wykorzystaæ moment bezw³adnoœci)     
		float V_kat_tarcie = Fy*tarcie_obr*dt / m / 1.0;      // zmiana pr. k¹towej spowodowana tarciem
		float V_kat_wgore = wV_kat_wgore.dlugosc() - V_kat_tarcie;
		if (V_kat_wgore < V_kat_skret) V_kat_wgore = V_kat_skret;        // tarcie nie mo¿e spowodowaæ zmiany zwrotu wektora pr. k¹towej
		wV_kat_wgore = wV_kat_wgore.znorm()*V_kat_wgore;
	}


	Fy = m*g*w_gora.y;                      // si³a docisku do pod³o¿a 
	if (Fy < 0) Fy = 0;
	// ... trzeba j¹ jeszcze uzale¿niæ od tego, czy obiekt styka siê z pod³o¿em!
	float Fh = Fy*tarcie*ham;                  // si³a hamowania (UP: bez uwzglêdnienia poœlizgu)

	float V_wprzod = wV_wprzod.dlugosc();// - dt*Fh/m - dt*tarcie_toczne*Fy/m;
	if (V_wprzod < 0) V_wprzod = 0;

	float V_wprawo = wV_wprawo.dlugosc();// - dt*tarcie*Fy/m;
	if (V_wprawo < 0) V_wprawo = 0;


	// wjazd lub zjazd: 
	//wPol.y = terrain.WysokoscGruntu(wPol.x,wPol.z);   // najprostsze rozwi¹zanie - obiekt zmienia wysokoœæ bez zmiany orientacji

	// 1. gdy wjazd na wklês³oœæ: wyznaczam wysokoœci terrainu pod naro¿nikami obiektu (ko³ami), 
	// sprawdzam która trójka
	// naro¿ników odpowiada najni¿ej po³o¿onemu œrodkowi ciê¿koœci, gdy przylega do terrainu
	// wyznaczam prêdkoœæ podbicia (wznoszenia œrodka pojazdu spowodowanego wklês³oœci¹) 
	// oraz prêdkoœæ k¹tow¹
	// 2. gdy wjazd na wypuk³oœæ to si³a ciê¿koœci wywo³uje obrót przy du¿ej prêdkoœci liniowej

	// punkty zaczepienia kó³ (na wysokoœci pod³ogi pojazdu):
	Wektor3 P = wPol + w_przod*(dlugosc / 2 - dl_przod) - w_prawo*szerokosc / 2 - w_gora*wysokosc / 2,
		Q = wPol + w_przod*(dlugosc / 2 - dl_przod) + w_prawo*szerokosc / 2 - w_gora*wysokosc / 2,
		R = wPol + w_przod*(-dlugosc / 2 + dl_tyl) - w_prawo*szerokosc / 2 - w_gora*wysokosc / 2,
		S = wPol + w_przod*(-dlugosc / 2 + dl_tyl) + w_prawo*szerokosc / 2 - w_gora*wysokosc / 2;

	// pionowe rzuty punktów zacz. kó³ pojazdu na powierzchniê terrainu:  
	Wektor3 Pt = P, Qt = Q, Rt = R, St = S;
	//Pt.y = terrain->WysokoscGruntu(P.x, P.z); Qt.y = terrain->WysokoscGruntu(Q.x, Q.z);
	//Rt.y = terrain->WysokoscGruntu(R.x, R.z); St.y = terrain->WysokoscGruntu(S.x, S.z);
	Pt.y = terrain->Wysokosc(P); Qt.y = terrain->Wysokosc(Q);
	Rt.y = terrain->Wysokosc(R); St.y = terrain->Wysokosc(S);
	Wektor3 normPQR = normalna(Pt, Rt, Qt), normPRS = normalna(Pt, Rt, St), normPQS = normalna(Pt, St, Qt),
		normQRS = normalna(Qt, Rt, St);   // normalne do p³aszczyzn wyznaczonych przez trójk¹ty

	//fprintf(f,"P.y = %f, Pt.y = %f, Q.y = %f, Qt.y = %f, R.y = %f, Rt.y = %f, S.y = %f, St.y = %f\n",
	//  P.y, Pt.y, Q.y, Qt.y, R.y,Rt.y, S.y, St.y);

	float sryPQR = ((Qt^normPQR) - normPQR.x*wPol.x - normPQR.z*wPol.z) / normPQR.y, // wys. œrodka pojazdu
		sryPRS = ((Pt^normPRS) - normPRS.x*wPol.x - normPRS.z*wPol.z) / normPRS.y, // po najechaniu na skarpê 
		sryPQS = ((Pt^normPQS) - normPQS.x*wPol.x - normPQS.z*wPol.z) / normPQS.y, // dla 4 trójek kó³
		sryQRS = ((Qt^normQRS) - normQRS.x*wPol.x - normQRS.z*wPol.z) / normQRS.y;
	float sry = sryPQR; Wektor3 norm = normPQR;
	if (sry > sryPRS) { sry = sryPRS; norm = normPRS; }
	if (sry > sryPQS) { sry = sryPQS; norm = normPQS; }
	if (sry > sryQRS) { sry = sryQRS; norm = normQRS; }  // wybór trójk¹ta o œrodku najni¿ej po³o¿onym    



	Wektor3 wV_kat_wpoziomie = Wektor3(0, 0, 0);
	// jesli któreœ z kó³ jest poni¿ej powierzchni terrainu
	if ((P.y <= Pt.y + wysokosc / 2 + przeswit) || (Q.y <= Qt.y + wysokosc / 2 + przeswit) ||
		(R.y <= Rt.y + wysokosc / 2 + przeswit) || (S.y <= St.y + wysokosc / 2 + przeswit))
	{
		// obliczam powsta³¹ prêdkoœæ k¹tow¹ w lokalnym uk³adzie wspó³rzêdnych:      
		Wektor3 wobrot = -norm.znorm()*w_gora*0.6;
		wV_kat_wpoziomie = wobrot / dt;
	}

	Wektor3 wAg = Wektor3(0, -1, 0)*g;    // przyspieszenie grawitacyjne

	// jesli wiecej niz 2 kola sa na ziemi, to przyspieszenie grawitacyjne jest rownowazone przez opor gruntu:
	if ((P.y <= Pt.y + wysokosc / 2 + przeswit) + (Q.y <= Qt.y + wysokosc / 2 + przeswit) +
		(R.y <= Rt.y + wysokosc / 2 + przeswit) + (S.y <= St.y + wysokosc / 2 + przeswit) > 2)
	{
		wAg = wAg +
			w_gora*(w_gora^wAg)*-1; //przyspieszenie wynikaj¹ce z si³y oporu gruntu
	}
	else   // w przeciwnym wypadku brak sily docisku 
		Fy = 0;



	// sk³adam z powrotem wektor prêdkoœci k¹towej: 
	//wV_kat = wV_kat_wgore + wV_kat_wprawo + wV_kat_wprzod;  
	wV_kat = wV_kat_wgore + wV_kat_wpoziomie;


	float h = sry + wysokosc / 2 + przeswit - wPol.y;  // ró¿nica wysokoœci jak¹ trzeba pokonaæ  
	float V_podbicia = 0;
	if ((h > 0) && (wV.y <= 0.01))
		V_podbicia = 0.5*sqrt(2 * g*h);  // prêdkoœæ spowodowana podbiciem pojazdu przy wje¿d¿aniu na skarpê 
	if (h > 0) wPol.y = sry + wysokosc / 2 + przeswit;

	// lub  w przypadku zag³êbienia siê 
	//fprintf(f,"sry = %f, wPol.y = %f, dt = %f\n",sry,wPol.y,dt);  
	//fprintf(f,"normPQR.y = %f, normPRS.y = %f, normPQS.y = %f, normQRS.y = %f\n",normPQR.y,normPRS.y,normPQS.y,normQRS.y); 


	Wektor3 dwPol = wV*dt;//wA*dt*dt/2; // czynnik bardzo ma³y - im wiêksza czêstotliwoœæ symulacji, tym mniejsze znaczenie 
	wPol = wPol + dwPol;

	// korekta po³o¿enia w przypadku terrainu cyklicznego (toroidalnego) z uwzglêdnieniem granic:
	if (terrain->czy_toroidalnosc)
	{
		if (terrain->granica_x > 0)
			if (wPol.x < -terrain->granica_x) wPol.x += terrain->granica_x*2;
			else if (wPol.x > terrain->granica_x) wPol.x -= terrain->granica_x*2;
		if (terrain->granica_z > 0)
			if (wPol.z < -terrain->granica_z) wPol.z += terrain->granica_z*2;
			else if (wPol.z > terrain->granica_z) wPol.z -= terrain->granica_z*2;
	}
	else 
	{
		if (terrain->granica_x > 0)
			if (wPol.x < -terrain->granica_x) wPol.x = -terrain->granica_x;
			else if (wPol.x > terrain->granica_x) wPol.x = terrain->granica_x;
		if (terrain->granica_z > 0)
			if (wPol.z < -terrain->granica_z) wPol.z = -terrain->granica_z;
			else if (wPol.z > terrain->granica_z) wPol.z = terrain->granica_z;
	}

	// Sprawdzenie czy obiekt mo¿e siê przemieœciæ w zadane miejsce: Jeœli nie, to 
	// przemieszczam obiekt do miejsca zetkniêcia, wyznaczam nowe wektory prêdkoœci
	// i prêdkoœci k¹towej, a nastêpne obliczam nowe po³o¿enie na podstawie nowych
	// prêdkoœci i pozosta³ego czasu. Wszystko powtarzam w pêtli (pojazd znowu mo¿e 
	// wjechaæ na przeszkodê). Problem z zaokr¹glonymi przeszkodami - konieczne 
	// wyznaczenie minimalnego kroku.


	Wektor3 wV_pop = wV;

	// sk³adam prêdkoœci w ró¿nych kierunkach oraz efekt przyspieszenia w jeden wektor:    (problem z przyspieszeniem od si³y tarcia -> to przyspieszenie 
	//      mo¿e dzia³aæ krócej ni¿ dt -> trzeba to jakoœ uwzglêdniæ, inaczej poazd bêdzie wê¿ykowa³)
	wV = wV_wprzod.znorm()*V_wprzod + wV_wprawo.znorm()*V_wprawo + wV_wgore +
		Wektor3(0, 1, 0)*V_podbicia + wA*dt;
	// usuwam te sk³adowe wektora prêdkoœci w których kierunku jazda nie jest mo¿liwa z powodu
	// przeskód:
	// np. jeœli pojazd styka siê 3 ko³ami z nawierzchni¹ lub dwoma ko³ami i œrodkiem ciê¿koœci to
	// nie mo¿e mieæ prêdkoœci w dó³ pod³ogi
	if ((P.y <= Pt.y + wysokosc / 2 + przeswit) || (Q.y <= Qt.y + wysokosc / 2 + przeswit) ||
		(R.y <= Rt.y + wysokosc / 2 + przeswit) || (S.y <= St.y + wysokosc / 2 + przeswit))    // jeœli pojazd styka siê co najm. jednym ko³em
	{
		Wektor3 dwV = wV_wgore + w_gora*(wA^w_gora)*dt;
		if ((w_gora.znorm() - dwV.znorm()).dlugosc() > 1)  // jeœli wektor skierowany w dó³ pod³ogi
			wV = wV - dwV;
	}

	/*fprintf(f," |wV_wprzod| %f -> %f, |wV_wprawo| %f -> %f, |wV_wgore| %f -> %f |wV| %f -> %f\n",
	wV_wprzod.dlugosc(), (wV_wprzod.znorm()*V_wprzod).dlugosc(),
	wV_wprawo.dlugosc(), (wV_wprawo.znorm()*V_wprawo).dlugosc(),
	wV_wgore.dlugosc(), (wV_wgore.znorm()*wV_wgore.dlugosc()).dlugosc(),
	wV_pop.dlugosc(), wV.dlugosc()); */

	// sk³adam przyspieszenia liniowe od si³ napêdzaj¹cych i od si³ oporu: 
	wA = (w_przod*F) / m*(Fy > 0)*(ilosc_paliwa > 0)  // od si³ napêdzaj¹cych
		- wV_wprzod.znorm()*(Fh / m + tarcie_toczne*Fy / m)*(V_wprzod > 0.01) // od hamowania i tarcia tocznego (w kierunku ruchu)
		- wV_wprawo.znorm()*tarcie*Fy / m*(V_wprawo > 0.01)    // od tarcia w kierunku prost. do kier. ruchu
		+ wAg;           // od grawitacji


	// utrata paliwa:
	ilosc_paliwa -= (fabs(F))*(Fy > 0)*dt / 20000;
	if (ilosc_paliwa < 0) ilosc_paliwa = 0;
	masa_calk = masa_wlasna + ilosc_paliwa;


	// obliczenie nowej orientacji:
	Wektor3 w_obrot = wV_kat*dt;// + wA_kat*dt*dt/2;    
	kwaternion q_obrot = AsixToQuat(w_obrot.znorm(), w_obrot.dlugosc());
	//fprintf(f,"w_obrot = (x=%f, y=%f, z=%f) \n",w_obrot.x, w_obrot.y, w_obrot.z );
	//fprintf(f,"q_obrot = (w=%f, x=%f, y=%f, z=%f) \n",q_obrot.w, q_obrot.x, q_obrot.y, q_obrot.z );
	qOrient = q_obrot*qOrient;
	//fprintf(f,"Pol = (%f, %f, %f) V = (%f, %f, %f) A = (%f, %f, %f) V_kat = (%f, %f, %f) ID = %d\n",
	//  wPol.x,wPol.y,wPol.z,wV.x,wV.y,wV.z,wA.x,wA.y,wA.z,wV_kat.x,wV_kat.y,wV_kat.z,iID);

	Przedmiot **wsk_prz = NULL;
	long liczba_prz_w_prom = terrain->Przedmioty_w_promieniu(&wsk_prz, wPol, this->promien * 2);
	// Generujê listê wskaŸników przedmiotów w s¹siedztwie symulowanego o promieniu 2.2, gdy¿ 
	// w przypadku obiektów mniejszych wystarczy 2.0 (zak³adaj¹c, ¿e inny obiekt ma promieñ co najwy¿ej taki sam),
	// a w przypadku wiêkszych to symulator tego wiêkszego powinien wczeœniej wykryæ kolizjê 
	MovableObject **wsk_ob = NULL;
	long liczba_ob_w_promieniu = terrain->Obiekty_w_promieniu(&wsk_ob, wPol, this->promien * 2.2 + wV.dlugosc()*dt);

	// wykrywanie kolizji z drzewami:
	// wykrywanie kolizji z drzewami z u¿yciem tablicy obszarów:
	Wektor3 wWR = wPol - wPol_pop;  // wektor jaki przemierzy³ pojazd w tym cyklu
	Wektor3 wWR_zn = wWR.znorm();
	float fWR = wWR.dlugosc();
	for (long i = 0; i < liczba_prz_w_prom; i++)
	{
		Przedmiot *prz = wsk_prz[i];
		if (prz->typ == PRZ_DRZEWO)
		{
			// bardzo duze uproszczenie -> traktuje pojazd jako kulê
			Wektor3 wPolDrz = prz->wPol;
			wPolDrz.y = (wPolDrz.y + prz->wartosc > wPol.y ? wPol.y : wPolDrz.y + prz->wartosc);
			float promien_drzewa = prz->param_f[0] * prz->srednica / 2;
			if ((wPolDrz - wPol).dlugosc() < promien*0.8 + promien_drzewa)  // jesli kolizja 
			{
				// od wektora predkosci odejmujemy jego rzut na kierunek od punktu styku do osi drzewa:
				// jesli pojazd juz wjechal w PRZ_DRZEWO, to nieco zwiekszamy poprawke     
				// punkt styku znajdujemy laczac krawedz pojazdu z osia drzewa odcinkiem
				// do obu prostopadlym      
				Wektor3 dP = (wPolDrz - wPol).znorm();   // wektor, w ktorego kierunku ruch jest niemozliwy
				float k = wV^dP;
				if (k > 0)     // jesli jest skladowa predkosci w strone drzewa
				{
					Wektor3 wV_pocz = wV;
					//wV = wV_pocz - dP*k*(1 + sprezystosc);  // odjecie skladowej + odbicie sprezyste 
					


					// Uwzglêdnienie obrotu:
					// pocz¹tkowa energia kinetyczna ruchu zamienia siê w energiê kinetyczn¹ ruchu postêpowego po 
					// kolizji i energiê ruchu obrotowego:
					float cos_alfa = wV.znorm() ^ dP;       // kosinus k¹ta pomiêdzy kierunkiem pojazda-drzewo a wektorem prêdkoœci poj.  
					
					// przyjmujê, ¿e im wiêkszy k¹t, tym wiêcej energii idzie na obrót
					wV = wV_pocz - dP*k*(1 + sprezystosc)*cos_alfa;
					Wektor3 wV_spr = wV_pocz - dP*k * 2*cos_alfa;                          // wektor prêdkoœci po odbiciu w pe³ni sprê¿ystym
					float fV_spr = wV_spr.dlugosc(), fV_pocz = wV_pocz.dlugosc();
					//float fV = wV.dlugosc();
					float dE = (fV_pocz*fV_pocz - fV_spr*fV_spr)*masa_calk / 2;
					if (dE > 0)     // w³aœciwie nie powinno byæ inaczej!
					{
						float I = (dlugosc*dlugosc + szerokosc*szerokosc)*masa_calk / 12;  // moment bezw³adnoœci prostok¹ta
						float omega = sqrt(2 * dE / I);     // modu³ prêdkoœci k¹towej wynikaj¹cy z reszty energii
						wV_kat = wV_kat + dP.znorm()*wV_pocz.znorm()*omega*sprezystosc;
					}
					
				} // jeœli wektor prêdkoœci w kierunku drzewa
			} // jeœli kolizja
		} // jeœli drzewo
		else if (prz->typ == PRZ_MUR)
		{			
			// bardzo duze uproszczenie -> traktuje pojazd jako kulê
			Wektor3 A = terrain->p[prz->param_i[0]].wPol, B = terrain->p[prz->param_i[1]].wPol;       // punkty tworz¹ce krawêdŸ
			A.y += terrain->p[prz->param_i[0]].wartosc;
			B.y += terrain->p[prz->param_i[1]].wartosc;

			Wektor3 AB = B - A;
			//Wektor3 AB_zn = AB.znorm();		
			Wektor3 m_pion = Wektor3(0,1,0);     // pion muru
			float m_wysokosc = prz->wartosc;       // od prostej AB do góry jest polowa wysokoœci, druga polowa idzie w dó³ 

			// zanim policzymy to co poni¿ej, mo¿na wykonaæ prostszy test np. odleg³oœci pomiêdzy spoziomowanymi odcinkami AB i odcinkiem wPol_pop,wPol
			// jeœli ta odleg³oœæ nie bêdzie wiêksza ni¿ szerokoœæ muru/2+promien to mo¿liwa kolizja:






			int liczba_scian = 4;     // œciany boczne
			Wektor3 m_wlewo;          // wektor prostopad³y do œciany wg³¹b muru
			Wektor3 m_wprzod;         // wektor po d³ugoœci œciany (ApBp) 
			float m_dlugosc;          // d³ugoœæ œciany
			float m_szerokosc;        // szerokoœæ muru poprzecznie do œciany  
			Wektor3 Ap, Bp;           // pocz¹tek i koniec œciany
			float Ap_y, Bp_y;           // wysokoœci w punktach pocz¹tkowym i koñcowym œciany
			bool czy_kolizja_zreal = false;   // czy kolizja zosta³a zrealizowana 

			for (int sciana = 0; sciana < liczba_scian; sciana++)
			{
				switch (sciana)
				{
				case 0:   // œciana z prawej strony patrz¹c od punktu A do B
					m_wlewo = (m_pion*AB).znorm();          // wektor jednostkowy w lewo od wektora AB i prostopadle do pionu
					m_wprzod = m_wlewo*m_pion;
					m_dlugosc = AB^m_wprzod;
					m_szerokosc = prz->param_f[0];

					Ap = A - m_wlewo*m_szerokosc / 2;
					Bp = B - m_wlewo*m_szerokosc / 2; // rzut odcinka AB na praw¹ œcianê muru
					Ap_y = A.y;
					Bp_y = B.y;
					break;
				case 1:  // œciana kolejna w kier. przeciwnym do ruchu wskazówek zegara (prostopad³a do AB, przechodz¹ca przez punkt B)
					m_wlewo = -AB.znorm();          // wektor jednostkowy w lewo od wektora AB i prostopadle do pionu
					m_wprzod = m_wlewo*m_pion;
					m_dlugosc = prz->param_f[0];
					m_szerokosc = -AB^m_wlewo;

					Ap = B - m_wprzod*m_dlugosc / 2;
					Bp = B + m_wprzod*m_dlugosc / 2; 
					Ap_y = Bp_y = B.y;

					break;
				case 2:  
					m_wlewo = -(m_pion*AB).znorm();          // wektor jednostkowy w lewo od wektora AB i prostopadle do pionu
					m_wprzod = m_wlewo*m_pion;
					m_dlugosc = -AB^m_wprzod;
					m_szerokosc = prz->param_f[0];

					Ap = B - m_wlewo*m_szerokosc / 2;
					Bp = A - m_wlewo*m_szerokosc / 2; 
					Ap_y = B.y;
					Bp_y = A.y;
					break;
				case 3:
					m_wlewo = AB.znorm();          // wektor jednostkowy w lewo od wektora AB i prostopadle do pionu
					m_wprzod = m_wlewo*m_pion;
					m_dlugosc = prz->param_f[0];
					m_szerokosc = AB^m_wlewo;

					Ap = A - m_wprzod*m_dlugosc / 2;
					Bp = A + m_wprzod*m_dlugosc / 2;
					Ap_y = Bp_y = A.y;
					break;
				}

				//Wektor3 Al = A + m_wlewo*m_szerokosc / 2, Bl = B + m_wlewo*m_szerokosc / 2; // rzut odcinka AB na lew¹ œcianê muru
				Wektor3 RR = punkt_przec_prostej_z_plaszcz(wPol_pop, wPol, -m_wlewo, Ap);  // punkt przeciêcia wektora ruchu ze œcian¹ praw¹
				Wektor3 QQ = rzut_punktu_na_pl(wPol, m_wlewo, Ap);  // rzut prostopad³y œrodka pojazdu na p³aszczyznê prawej œciany
				float odl_zezn_prawa = (QQ - wPol) ^ m_wlewo;        // odleg³oœæ ze znakiem œr.pojazdu od p³. prawej: dodatnia jeœli wPol nie przekracza œciany
				//float odl_dokladna = odleglosc_pom_punktem_a_odcinkiem(wPol, Ap, Bp);
				float cos_kata_pad = wWR_zn^m_wlewo;       // kosinus k¹ta wekt.ruchu w stos. do normalnej œciany (dodatni, gdy pojazd zmierza do œciany od prawej strony)
				//Wektor3 Ap_granica_kol = Ap - m_wprzod*promien / cos_kata_pad, Bp_granica_kol = Bp + m_wprzod*promien / cos_kata_pad; // odcinek ApBp powiêkszony z obu stron 

				bool czy_poprzednio_obiekt_przed_sciana = (((RR - wPol_pop) ^ m_wlewo) > 0); // czy w poprzednim cyklu œrodek poj. znajdowa³ siê przed œcian¹
				bool czy_QQ_na_scianie = ((QQ - Ap).x*(QQ - Bp).x + (QQ - Ap).z*(QQ - Bp).z < 0);
				bool czy_RR_na_scianie = ((RR - Ap).x*(RR - Bp).x + (RR - Ap).z*(RR - Bp).z < 0);

				if ((czy_poprzednio_obiekt_przed_sciana) && (cos_kata_pad > 0) &&
					((odl_zezn_prawa >= 0) && (odl_zezn_prawa < promien*0.8) && (czy_QQ_na_scianie) ||   // gdy obiekt lekko nachodzi na œcianê
					(odl_zezn_prawa < 0) && (czy_RR_na_scianie))                                     // gdy obiekt ,,przelecia³'' przez œcianê w ci¹gu cyklu
					)
				{
					float czy_w_przod = ((wWR_zn^m_wprzod) > 0);                    // czy sk³¹dowa pozioma wektora ruchu skierowana w przód 
					Wektor3 SS = RR - m_wprzod*(2 * czy_w_przod - 1)*promien / cos_kata_pad;   // punkt styku obiektu ze œcian¹
					float odl_od_A = fabs((SS - Ap) ^ m_wprzod);     // odleg³oœæ od punktu A
					float wysokosc_muru_wS = Ap_y + (Bp_y - Ap_y)*odl_od_A / m_dlugosc + m_wysokosc / 2;  // wysokoœæ muru w punkcie styku
					//bool czy_wysokosc_kol = ((SS - Ap).y - this->wysokosc / 2 < wysokosc_muru_wS);  // test wysokoœci - czy kolizja
					bool czy_wysokosc_kol = ((SS.y - this->wysokosc / 2 < wysokosc_muru_wS) &&  // test wysokoœci od góry
						(SS.y + this->wysokosc / 2 > wysokosc_muru_wS - m_wysokosc));  // od do³u
					if (czy_wysokosc_kol)
					{
						float V_prost_pop = wV^m_wlewo;
						wV = wV - m_wlewo*V_prost_pop*(1 + sprezystosc);       // zamiana kierunku sk³adowej prêdkoœci prostopad³ej do œciany 

						// mo¿emy te¿ czêœæ energii kinetycznej 

						// cofamy te¿ obiekt przed œcianê, tak jakby siê od niej odbi³ z ustalon¹ sprê¿ystoœci¹:
						float poprawka_polozenia = promien*0.8 - odl_zezn_prawa;
						wPol = wPol - m_wlewo* poprawka_polozenia*(1 + sprezystosc);

						//fprintf(f, "poj. w scianie, cos_kat = %f, w miejscu %f, Vpop = %f, V = %f\n", cos_kata_pad, odl_od_A / m_dlugosc,
						//	V_prost_pop, wV^m_wlewo);
						czy_kolizja_zreal = true;
						break;    // wyjœcie z pêtli po œcianach, by nie traciæ czasu na dalsze sprawdzanie
					}
					else
					{
						int x = 1;
					}
				} // jeœli wykryto kolizjê ze œcian¹ w przestrzeni 2D
			} // po œcianach

			if (czy_kolizja_zreal == false)  // jeœli kolizja nie zosta³a jeszcze zrealizowana, sprawdzam kolizjê ze œcian¹ doln¹
			{
				m_wlewo = (m_pion*AB).znorm();
				m_szerokosc = prz->param_f[0];
				Wektor3 N = (AB*m_wlewo).znorm();    // normalna do dolnej œciany (zwrot do wewn¹trz)
				Wektor3 Ad = A - m_pion*(m_wysokosc / 2), Bd = B - m_pion*(m_wysokosc / 2); // rzuty pionowe punktów A i B na doln¹ p³aszczyznê 
				Wektor3 RR = punkt_przec_prostej_z_plaszcz(wPol_pop, wPol, -N, Ad);  // punkt przeciêcia wektora ruchu ze œcian¹ praw¹
				Wektor3 RRR = rzut_punktu_na_prosta(RR, Ad, Bd);
				float odl_RR_od_RRR = (RR - RRR).dlugosc();
				Wektor3 QQ = rzut_punktu_na_pl(wPol, -N, Ad);  // rzut prostopad³y œrodka pojazdu na p³aszczyznê prawej œciany
				float odl_zezn_prawa = (QQ - wPol) ^ N;        // odleg³oœæ ze znakiem œr.pojazdu od p³aszczyzny: dodatnia jeœli wPol nie przekracza œciany
				float odl_praw = (QQ - wPol).dlugosc();
				float x = (Ad - wPol) ^ N;
				float xx = (RR - wPol) ^ wWR_zn;
				Wektor3 QQQ = rzut_punktu_na_prosta(QQ, Ad, Bd);  
				float odl_QQ_od_QQQ = (QQ - QQQ).dlugosc();
				//float odl_dokladna = odleglosc_pom_punktem_a_odcinkiem(wPol, Ap, Bp);
				float cos_kata_pad = wWR_zn^N;       // kosinus k¹ta wekt.ruchu w stos. do normalnej œciany (dodatni, gdy pojazd zmierza do œciany od prawej strony)
				//Wektor3 Ap_granica_kol = Ap - m_wprzod*promien / cos_kata_pad, Bp_granica_kol = Bp + m_wprzod*promien / cos_kata_pad; // odcinek ApBp powiêkszony z obu stron 

				bool czy_poprzednio_obiekt_przed_sciana = (((RR - wPol_pop) ^ N) > 0); // czy w poprzednim cyklu œrodek poj. znajdowa³ siê przed œcian¹

				bool czy_QQ_na_scianie = ((QQ - Ad).x*(QQ - Bd).x + (QQ - Ad).z*(QQ - Bd).z < 0) && (odl_QQ_od_QQQ < m_szerokosc / 2);
				bool czy_RR_na_scianie = ((RR - Ad).x*(RR - Bd).x + (RR - Ad).z*(RR - Bd).z < 0) && (odl_RR_od_RRR < m_szerokosc / 2);


				//fprintf(f, "nr. cyklu = %d, odl_zezn_prawa = %f, x = %f, QQ = (%f,%f,%f), N = (%f,%f,%f), Ad = (%f,%f,%f), wPol = (%f,%f,%f)\n", 
				//	terrain->liczba_wyswietlen, odl_zezn_prawa, x, QQ.x, QQ.y, QQ.z, N.x, N.y, N.z, Ad.x, Ad.y, Ad.z, wPol.x, wPol.y, wPol.z);
				//fprintf(f, "nr. cyklu = %d, odl_zezn_prawa = %f, odl do RR = %f, cos_kata_pad = %f\n",
				//	terrain->liczba_wyswietlen, odl_zezn_prawa, xx, cos_kata_pad);
				//fprintf(f, "wPol.y = %f, Ad.y = %f\n", wPol.y, Ad.y);


				if ((czy_poprzednio_obiekt_przed_sciana) && (cos_kata_pad > 0) &&
					((odl_zezn_prawa >= 0) && (odl_zezn_prawa < promien*0.8) && (czy_QQ_na_scianie) ||   // gdy obiekt lekko nachodzi na œcianê
					(odl_zezn_prawa < 0) && (czy_RR_na_scianie))                                     // gdy obiekt ,,przelecia³'' przez œcianê w ci¹gu cyklu
					)
				{
					float V_prost_pop = wV^N;
					wV = wV - N*V_prost_pop*(1 + sprezystosc);       // zamiana kierunku sk³adowej prêdkoœci prostopad³ej do œciany 
					float poprawka_polozenia = promien*0.8 - odl_zezn_prawa;
					wPol = wPol - N* poprawka_polozenia*(1 + sprezystosc);
					//fclose(f);
					czy_kolizja_zreal = true;
				}
				

			}

		}  // jeœli mur
	} // for po przedmiotach w promieniu


	// sprawdzam, czy nie najecha³em na monetê lub beczkê z paliwem. 
	// zak³adam, ¿e w jednym cylku symulacji mogê wzi¹æ maksymalnie jeden przedmiot
	for (long i = 0; i < liczba_prz_w_prom; i++)
	{
		Przedmiot *prz = wsk_prz[i];

		if ((prz->do_wziecia == 1) &&
			((prz->wPol - wPol + Wektor3(0, wPol.y - prz->wPol.y, 0)).dlugosc() < promien))
		{
			float odl_nasza = (prz->wPol - wPol + Wektor3(0, wPol.y - prz->wPol.y, 0)).dlugosc();

			long wartosc = prz->wartosc;
			wartosc_wzieta = -1;

			if (prz->typ == PRZ_MONETA)
			{
				bool mozna_wziac = false;
				// przy du¿ej wartoœci nie mogê samodzielnie podnieœæ pieni¹¿ka bez bratniej pomocy innego pojazdu
				// odleg³oœæ bratniego pojazdu od pieni¹dza nie mo¿e byæ mniejsza od naszej odleg³oœci, gdy¿ wtedy
				// to ten inny pojazd zgarnie monetê:
				if (wartosc >= 1000)
				{
					bool bratnia_pomoc = false;
					int kto_pomogl = -1;
					for (long k = 0; k < liczba_ob_w_promieniu; k++)
					{
						MovableObject *inny = wsk_ob[k];
						float odl_bratnia = (inny->wPol - prz->wPol).dlugosc();
						if ((inny->iID != iID) && (odl_bratnia < inny->promien * 2) && (odl_nasza < odl_bratnia))
						{
							bratnia_pomoc = true;
							kto_pomogl = inny->iID;
						}
					}

					if (!bratnia_pomoc)
					{
						sprintf(par_wid.napis1, "nie_mozna_wziac_tak_ciezkiego_pieniazka,_chyba_ze_ktos_podjedzie_i_pomoze.");
						mozna_wziac = false;
					}
					else
					{
						sprintf(par_wid.napis1, "pojazd_o_ID_%d_pomogl_wziac_monete_o_wartosci_%d", kto_pomogl, wartosc);
						mozna_wziac = true;
					}
				}
				else
					mozna_wziac = true;

				if (mozna_wziac)
				{
					wartosc_wzieta = (float)wartosc*umiejetn_zb_monet;
					pieniadze += (long)wartosc_wzieta;
				}

				//sprintf(napis2,"Wziecie_gotowki_o_wartosci_ %d",wartosc);
			} // jeœli to PRZ_MONETA
			else if (prz->typ == PRZ_BECZKA)
			{
				wartosc_wzieta = (float)wartosc*umiejetn_zb_paliwa;
				ilosc_paliwa += wartosc_wzieta;
				//sprintf(napis2,"Wziecie_paliwa_w_ilosci_ %d",wartosc);
			}

			if (wartosc_wzieta > 0)
			{
				prz->do_wziecia = 0;
				prz->czy_ja_wzialem = 1;
				prz->czas_wziecia = czas_symulacji;

				// zapis informacji, by przekazaæ j¹ innym aplikacjom:
				nr_wzietego_przedm = prz->indeks;
			}
		}
	} // po przedmiotach w promieniu


	// Zamiast listowaæ ca³¹ tablicê przedmiotów mo¿na by zrobiæ listê wziêtych przedmiotów
	for (long i = 0; i < terrain->liczba_przedmiotow; i++)
	{
		Przedmiot *prz = &terrain->p[i];
		if ((prz->do_wziecia == 0) && (prz->czy_ja_wzialem) && (prz->czy_odnawialny) &&
			(czas_symulacji - prz->czas_wziecia >= terrain->czas_odnowy_przedm))
		{                              // jeœli min¹³ pewnien okres czasu przedmiot mo¿e zostaæ przywrócony
			prz->do_wziecia = 1;
			prz->czy_ja_wzialem = 0;
			nr_odnowionego_przedm = i;
		}
	}



	// kolizje z innymi obiektami
	if (iID_kolid == iID) // ktoœ o numerze iID_kolid wykry³ kolizjê z naszym pojazdem i poinformowa³ nas o tym

	{
		//fprintf(f,"ktos wykryl kolizje - modyf. predkosci\n",iID_kolid);
		wV = wV + wdV_kolid;   // modyfikuje prêdkoœæ o wektor obliczony od drugiego (¿yczliwego) uczestnika
		iID_kolid = -1;

	}
	else
	{
		for (long i = 0; i < liczba_ob_w_promieniu; i++)
		{
			MovableObject *inny = wsk_ob[i];

			if ((wPol - inny->wPol).dlugosc() < promien + inny->promien)  // jeœli kolizja 
			{
				// zderzenie takie jak w symulacji kul 
				Wektor3 norm_pl_st = (wPol - inny->wPol).znorm();    // normalna do p³aszczyzny stycznej - kierunek odbicia
				float m1 = masa_calk, m2 = inny->masa_calk;          // masy obiektów
				float W1 = wV^norm_pl_st, W2 = inny->wV^norm_pl_st;  // wartosci prêdkoœci
				if (W2 > W1)      // jeœli obiekty siê przybli¿aj¹
				{

					float Wns = (m1*W1 + m2*W2) / (m1 + m2);        // prêdkoœæ po zderzeniu ca³kowicie niesprê¿ystym
					float W1s = ((m1 - m2)*W1 + 2 * m2*W2) / (m1 + m2), // prêdkoœæ po zderzeniu ca³kowicie sprê¿ystym
						W2s = ((m2 - m1)*W2 + 2 * m1*W1) / (m1 + m2);
					float W1sp = Wns + (W1s - Wns)*sprezystosc;    // prêdkoœæ po zderzeniu sprê¿ysto-plastycznym
					float W2sp = Wns + (W2s - Wns)*sprezystosc;

					wV = wV + norm_pl_st*(W1sp - W1);    // poprawka prêdkoœci (zak³adam, ¿e inny w przypadku drugiego obiektu zrobi to jego w³asny symulator) 
					iID_kolid = inny->iID;
					wdV_kolid = norm_pl_st*(W2sp - W2);
					//fprintf(f,"wykryto i zreal. kolizje z %d W1=%f,W2=%f,W1s=%f,W2s=%f,m1=%f,m2=%f\n",iID_kolid,W1,W2,W1s,W2s,m1,m2); 
				}
				//if (fabs(W2 - W1)*dt < (wPol - inny->wPol).dlugosc() < 2*promien) wV = wV + norm_pl_st*(W1sp-W1)*2;
			}
		}
	} // do else
	delete wsk_prz;
	delete wsk_ob;

}



void MovableObject::Rysuj()
{
	glPushMatrix();

	glTranslatef(wPol.x, wPol.y + przeswit, wPol.z);

	kwaternion k = qOrient.AsixAngle();
	//fprintf(f,"kwaternion = [%f, %f, %f], w = %f\n",qOrient.x,qOrient.y,qOrient.z,qOrient.w);
	//fprintf(f,"os obrotu = [%f, %f, %f], kat = %f\n",k.x,k.y,k.z,k.w*180.0/PI);

	glRotatef(k.w*180.0 / PI, k.x, k.y, k.z);
	glPushMatrix();
	glTranslatef(-dlugosc / 2, -wysokosc / 2, -szerokosc / 2);

	glScalef(dlugosc, wysokosc, szerokosc);
	glCallList(Cube);
	glPopMatrix();
	if (this->czy_zazn)
	{
		float w = 1.1;
		glTranslatef(-dlugosc / 2 * w, -wysokosc / 2 * w, -szerokosc / 2 * w);
		glScalef(dlugosc*w, wysokosc*w, szerokosc*w);
		GLfloat Surface[] = { 2.0f, 0.0f, 0.0f, 1.0f };
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
		glCallList(Cube_skel);
	}

	GLfloat Surface[] = { 2.0f, 2.0f, 1.0f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
	glRasterPos2f(0.30, 1.20);
	glPrint("%d", iID);
	glPopMatrix();
}





Sektor::Sektor(int _loczek, long _w, long _k, bool czy_mapa)
{
	w = _w; k = _k;
	liczba_obiektow_ruch = 0;
	liczba_obiektow_ruch_max = 10;
	wob = new MovableObject*[liczba_obiektow_ruch_max];
	liczba_przedmiotow = 0;
	liczba_przedmiotow_max = 10;
	wp = new Przedmiot*[liczba_przedmiotow_max];
	liczba_oczek = _loczek;
	if (czy_mapa)
	{
		pamiec_dla_mapy(liczba_oczek,false);  // mapa wysokoœci + normalne + typy_nawierzchni
		this->liczba_oczek_wyswietlana = liczba_oczek;
		liczba_oczek_wyswietlana_pop = liczba_oczek;
	}
	else
	{
		typy_naw = NULL;
		mapa_wysokosci = NULL;
		Norm = NULL;
		poziom_wody = NULL;
		this->liczba_oczek_wyswietlana = 1;
		liczba_oczek_wyswietlana_pop = 1;
	}
	typ_naw_sek = 0;              // parametry standardowe dla ca³ego sektora brane pod uwagê gdy nie ma dok³adnej mapy
	wysokosc_gruntu_sek = 0;
	poziom_wody_sek = -1e10;

	mapa_wysokosci_edycja = NULL;
	Norm_edycja = NULL;
	typy_naw_edycja = NULL;
	poziom_wody_edycja = NULL;
	//fprintf(f, "Konstruktor: utworzono sektor w = %d, k = %d, mapa = %d\n", w, k, mapa_wysokosci);
	this->wysokosc_max = -1e10;
}

Sektor::~Sektor()
{
	delete wob;  // to zawsze jest
	delete wp;   // to te¿

	// natomiast to ju¿ nie zawsze:
	if (mapa_wysokosci)
		zwolnij_pamiec_dla_mapy();
	//fprintf(f, "Destruktor: Usunieto sektor w = %d, k = %d\n", w, k);
}

void Sektor::pamiec_dla_mapy(int __liczba_oczek, bool czy_edycja)
{
	float **__mapa_wysokosci = new float*[__liczba_oczek * 2 + 1];
	for (int n = 0; n < __liczba_oczek * 2 + 1; n++)
	{
		__mapa_wysokosci[n] = new float[__liczba_oczek + 1];
	}
	int liczba_rozdzielczosci = 1+log2(__liczba_oczek);         // liczba map o ró¿nych rozdzielczoœciach a¿ do mapy 1x1 (jedno oczko, 4 normalne)
	Wektor3**** __Norm = new Wektor3***[liczba_rozdzielczosci];
	for (int rozdz = 0; rozdz < liczba_rozdzielczosci; rozdz++)
	{
		long loczek = __liczba_oczek / (1 << rozdz);
		__Norm[rozdz] = new Wektor3**[loczek];
		for (int k = 0; k < loczek; k++)
		{
			__Norm[rozdz][k] = new Wektor3*[loczek];
			for (int n = 0; n < loczek; n++)
				__Norm[rozdz][k][n] = new Wektor3[4];
		}
	}
	int **__typy_naw = new int*[__liczba_oczek];
	for (int k = 0; k < __liczba_oczek; k++)
	{
		__typy_naw[k] = new int[__liczba_oczek];
		for (int n = 0; n < __liczba_oczek; n++)
			__typy_naw[k][n] = 0;
	}

	float **__poziom_wody = new float*[__liczba_oczek];
	for (int k = 0; k < __liczba_oczek; k++)
	{
		__poziom_wody[k] = new float[__liczba_oczek];
		for (int n = 0; n < __liczba_oczek; n++)
			__poziom_wody[k][n] = -1e10;
	}
	

	if (czy_edycja)
	{
		mapa_wysokosci_edycja = __mapa_wysokosci;
		Norm_edycja = __Norm;
		typy_naw_edycja = __typy_naw;
		poziom_wody_edycja = __poziom_wody;
		liczba_oczek_edycja = __liczba_oczek;
	}
	else
	{
		mapa_wysokosci = __mapa_wysokosci;
		Norm = __Norm;
		typy_naw = __typy_naw;
		poziom_wody = __poziom_wody;
		liczba_oczek = __liczba_oczek;
	}
}

void Sektor::zwolnij_pamiec_dla_mapy(bool czy_edycja)
{
	float **mapa = mapa_wysokosci;
	Wektor3 ****__N = Norm;
	int **__typy_naw = typy_naw;
	float **__poziom_wody = poziom_wody;
	long loczek = liczba_oczek;

	if (czy_edycja)
	{
		mapa = mapa_wysokosci_edycja;
		__N = Norm_edycja;
		__typy_naw = typy_naw_edycja;
		__poziom_wody = poziom_wody_edycja;
		loczek = liczba_oczek_edycja;
		mapa_wysokosci_edycja = NULL;
		Norm_edycja = NULL;
		typy_naw_edycja = NULL;
		poziom_wody_edycja = NULL;
	}
	else
	{
		mapa_wysokosci = NULL;
		Norm = NULL;
		typy_naw = NULL;
		poziom_wody = NULL;
	}

	if (mapa)
	{
		for (int ww = 0; ww < loczek * 2 + 1; ww++)
			delete mapa[ww];
		delete mapa;

		long liczba_rozdzielczosci = 1 + log2(loczek);
		for (int rozdz = 0; rozdz < liczba_rozdzielczosci; rozdz++)
		{
			for (int i = 0; i < loczek / (1 << rozdz); i++)
			{
				for (int j = 0; j < loczek / (1 << rozdz); j++)
					delete __N[rozdz][i][j];
				delete __N[rozdz][i];
			}
			delete __N[rozdz];
		}
		delete __N;

		for (int i = 0; i < loczek; i++)
			delete __typy_naw[i];
		delete __typy_naw;
		for (int i = 0; i < loczek; i++)
			delete __poziom_wody[i];
		delete __poziom_wody;
		
	}
}

void Sektor::wstaw_przedmiot(Przedmiot *p)
{
	if (liczba_przedmiotow == liczba_przedmiotow_max) // powiekszenie tablicy
	{
		Przedmiot **wp_nowe = new Przedmiot*[2 * liczba_przedmiotow_max];
		for (long i = 0; i < liczba_przedmiotow; i++) wp_nowe[i] = wp[i];
		delete wp;
		wp = wp_nowe;
		liczba_przedmiotow_max = 2 * liczba_przedmiotow_max;
	}
	wp[liczba_przedmiotow] = p;
	liczba_przedmiotow++;
}

void Sektor::usun_przedmiot(Przedmiot *p)
{
	for (long i = 0; i < this->liczba_przedmiotow;i++)
		if (this->wp[i] == p){
			wp[i] = wp[liczba_przedmiotow-1];
			liczba_przedmiotow--;
			break;
		}
}

void Sektor::wstaw_obiekt_ruchomy(MovableObject *o)
{
	if (liczba_obiektow_ruch == liczba_obiektow_ruch_max)
	{
		MovableObject **wob_nowe = new MovableObject*[2 * liczba_obiektow_ruch_max];
		for (long i = 0; i < liczba_obiektow_ruch; i++) wob_nowe[i] = wob[i];
		delete wob;
		wob = wob_nowe;
		liczba_obiektow_ruch_max = 2 * liczba_obiektow_ruch_max;
	}
	wob[liczba_obiektow_ruch] = o;
	liczba_obiektow_ruch++;
}
void Sektor::usun_obiekt_ruchomy(MovableObject *o)
{
	for (long i = liczba_obiektow_ruch - 1; i >= 0; i--)
		if (wob[i] == o){
			wob[i] = wob[liczba_obiektow_ruch - 1];
			liczba_obiektow_ruch--;
			break;
		}
}
// obliczenie wektorów normalnych N do p³aszczyzn trójk¹tów, by nie robiæ tego ka¿dorazowo przy odrysowywaniu
void Sektor::oblicz_normalne(float rozmiar_sektora, bool czy_edycja)
{
	float **mapa = mapa_wysokosci;
	Wektor3 ****__Norm = Norm;
	long loczek = liczba_oczek;

	if (czy_edycja){
		mapa = mapa_wysokosci_edycja;
		__Norm = Norm_edycja;
		loczek = liczba_oczek_edycja;
	}

	if (mapa)
	{
		long liczba_rozdzielczosci = 1 + log2(loczek);              // gdy podst. rozdzielczoœæ 16x16, mamy 5 rozdzielczoœci:
		                                                            // 16x16, 8x8, 4x4, 2x2, 1x1
		for (int rozdz = 0; rozdz < liczba_rozdzielczosci; rozdz++)
		{

			//fprintf(f, "znaleziono sektor o w = %d, k = %d\n", sek->w, sek->k);
			long zmn = (1 << rozdz);             // zmniejszenie rozdzielczoœci (potêga dwójki)
			long loczek_rozdz = loczek / zmn;    // liczba oczek w danym wariancie rozdzielczoœci
			float rozmiar_pola = rozmiar_sektora / loczek;        // rozmiar pola w danym wariancie rozdzielczoœci

			// tworze listê wyœwietlania rysuj¹c poszczególne pola mapy za pomoc¹ trójk¹tów 
			// (po 4 trójk¹ty na ka¿de pole):
			enum tr{ ABC = 0, ADB = 1, BDE = 2, CBE = 3 };

			Wektor3 A, B, C, D, E, N;

			for (long w = 0; w < loczek_rozdz; w++)
				for (long k = 0; k < loczek_rozdz; k++)
				{
					A = Wektor3(k*rozmiar_pola, mapa[w * 2 * zmn][k*zmn], w*rozmiar_pola);
					B = Wektor3((k + 0.5)*rozmiar_pola, mapa[(w * 2 + 1)*zmn][k*zmn+(zmn>1)*zmn/2], (w + 0.5)*rozmiar_pola);
					C = Wektor3((k + 1)*rozmiar_pola, mapa[w * 2 * zmn][(k + 1)*zmn], w*rozmiar_pola);
					D = Wektor3(k*rozmiar_pola, mapa[(w + 1) * 2 * zmn][k*zmn], (w + 1)*rozmiar_pola);
					E = Wektor3((k + 1)*rozmiar_pola, mapa[(w + 1) * 2 * zmn][(k + 1)*zmn], (w + 1)*rozmiar_pola);

					// tworzê trójk¹t ABC w górnej czêœci kwadratu: 
					//  A o_________o C
					//    |.       .|
					//    |  .   .  | 
					//    |    o B  | 
					//    |  .   .  |
					//    |._______.|
					//  D o         o E

					Wektor3 AB = B - A;
					Wektor3 BC = C - B;
					N = (AB*BC).znorm();
					//d[w][k][ABC] = -(B^N);          // dodatkowo wyznaczam wyraz wolny z równania plaszyzny trójk¹ta
					__Norm[rozdz][w][k][ABC] = N;          // dodatkowo zapisujê normaln¹ do p³aszczyzny trójk¹ta
					// trójk¹t ADB:
					Wektor3 AD = D - A;
					N = (AD*AB).znorm();
					//d[w][k][ADB] = -(B^N);
					__Norm[rozdz][w][k][ADB] = N;
					// trójk¹t BDE:
					Wektor3 BD = D - B;
					Wektor3 DE = E - D;
					N = (BD*DE).znorm();
					//d[w][k][BDE] = -(B^N);
					__Norm[rozdz][w][k][BDE] = N;
					// trójk¹t CBE:
					Wektor3 CB = B - C;
					Wektor3 BE = E - B;
					N = (CB*BE).znorm();
					//d[w][k][CBE] = -(B^N);
					__Norm[rozdz][w][k][CBE] = N;
				}
		} // po rozdzielczoœciach (od najwiêkszej do najmniiejszej
	} // jeœli znaleziono mapê sektora


	// dodatkowo wyznaczana jest maksymalna wysokoœæ:
	if (mapa)
	{
		wysokosc_max = -1e10;
		for (long w = 0; w < loczek; w++)
			for (long k = 0; k < loczek; k++)
				if (mapa[w * 2][k] > wysokosc_max)
					wysokosc_max = mapa[w * 2][k];
	}
	else
		wysokosc_max = 0;
}

TabSektorow::TabSektorow()
{
	liczba_komorek = 1000;
	ogolna_liczba_sektorow = 0;
	komorki = new KomorkaTablicy[liczba_komorek];
	for (long i = 0; i < liczba_komorek; i++)
	{
		komorki[i].liczba_sektorow = 0;
		komorki[i].rozmiar_pamieci = 0;
		komorki[i].sektory = NULL;
	}
	// zainicjowanie indeksów skrajnych sektorów (w celu u³atwienia wyszukiwania istniej¹cych sektorów): 
	w_min = 1000000;
	w_max = -1000000;
	k_min = 1000000;
	k_max = -1000000;
}

TabSektorow::~TabSektorow()
{
	for (long i = 0; i < liczba_komorek; i++)
		delete komorki[i].sektory;
	delete komorki;
}

 // wyznaczanie indeksu komórki na podstawie wspó³rzêdnych sektora
unsigned long TabSektorow::wyznacz_klucz(long w, long k)                 
{
	unsigned long kl = abs((w^k + w*(w + 1)*k + k*(k - 1)*w)) % liczba_komorek;
	//if (kl < 0) kl = -kl;

	return kl;
}

// znajdowanie sektora (zwraca NULL jeœli nie znaleziono)
Sektor *TabSektorow::znajdz(long w, long k)       
{
	unsigned long klucz = wyznacz_klucz(w, k);
	Sektor *sektor = NULL;
	//fprintf(f, "  znajdowanie sektora - klucz = %d, jest to %d sektorow\n", klucz, komorki[klucz].liczba_sektorow);
	for (long i = 0; i < komorki[klucz].liczba_sektorow; i++)
	{
		Sektor *s = komorki[klucz].sektory[i];
		//fprintf(f, "    i=%d, w=%d, k=%d\n", i, s->w, s->k);
		if ((s->w == w) && (s->k == k)) {
			sektor = s;
			break;
		}
	}
	return sektor;
}

// wstawianie sektora do tablicy
Sektor *TabSektorow::wstaw(Sektor *s)      
{
	ogolna_liczba_sektorow++;

	if (ogolna_liczba_sektorow > liczba_komorek) // reorganizacja tablicy (by nie by³o zbyt du¿o konfliktów)
	{
		long stara_liczba_komorek = liczba_komorek;
		KomorkaTablicy *stare_komorki = komorki;
		liczba_komorek = stara_liczba_komorek * 2;
		komorki = new KomorkaTablicy[liczba_komorek];
		for (long i = 0; i < liczba_komorek; i++)
		{
			komorki[i].liczba_sektorow = 0;
			komorki[i].rozmiar_pamieci = 0;
			komorki[i].sektory = NULL;
		}
		for (long i = 0; i < stara_liczba_komorek; i++)
		{
			for (long j = 0; j < stare_komorki[i].liczba_sektorow; j++)
			{
				ogolna_liczba_sektorow--;
				wstaw(stare_komorki[i].sektory[j]);
			}
			delete stare_komorki[i].sektory;
		}
		delete stare_komorki;
	}

	long w = s->w, k = s->k;
	long klucz = wyznacz_klucz(w, k);
	long liczba_sekt = komorki[klucz].liczba_sektorow;
	if (liczba_sekt >= komorki[klucz].rozmiar_pamieci)     // jesli brak pamieci, to nale¿y j¹ powiêkszyæ
	{
		long nowy_rozmiar = (komorki[klucz].rozmiar_pamieci == 0 ? 1 : komorki[klucz].rozmiar_pamieci * 2);
		Sektor **sektory2 = new Sektor*[nowy_rozmiar];
		for (long i = 0; i < komorki[klucz].liczba_sektorow; i++) sektory2[i] = komorki[klucz].sektory[i];
		delete komorki[klucz].sektory;
		komorki[klucz].sektory = sektory2;
		komorki[klucz].rozmiar_pamieci = nowy_rozmiar;
	}
	komorki[klucz].sektory[liczba_sekt] = s;
	komorki[klucz].liczba_sektorow++;

	if (w_min > w) w_min = w;
	if (w_max < w) w_max = w;
	if (k_min > k) k_min = k;
	if (k_max < k) k_max = k;

	return komorki[klucz].sektory[liczba_sekt];           // zwraca wskaŸnik do nowoutworzonego sektora
}

void TabSektorow::usun(Sektor *s)
{
	long w = s->w, k = s->k;
	long klucz = wyznacz_klucz(w, k);
	long liczba_sekt = komorki[klucz].liczba_sektorow;
	long indeks = -1;
	for (long i = 0; i < liczba_sekt; i++)
	{
		Sektor *ss = komorki[klucz].sektory[i];
		if ((ss->w == w) && (ss->k == k)) {
			indeks = i;
			break;
		}
	}
	if (indeks > -1){
		komorki[klucz].sektory[indeks] = komorki[klucz].sektory[liczba_sekt - 1];
		komorki[klucz].liczba_sektorow--;
	}
}
//**********************
//   Obiekty nieruchome
//**********************
Terrain::Terrain()
{
	bool czy_z_pliku = 1;
    szczeg = 0.5;                 // stopieñ szczegó³owoœci wyœwietlania przedmiotów i powierzchni terrainu (1 - pe³na, 0 - minimalna)
	liczba_wyswietlen = 0;
	ts = new TabSektorow();

	liczba_zazn_przedm_max = 100;  // to nie oznacza, ¿e tyle tylko mo¿na zaznaczyæ przedmiotów, ale ¿e tyle jest miejsca w tablicy, która jest automatycznie powiêkszana
	zazn_przedm = new long[liczba_zazn_przedm_max];     // tablica zaznaczonych przedmiotów (dodatkowo ka¿dy przedmiot posiada pole z informacj¹ o zaznaczeniu)
	liczba_zazn_przedm = 0;
	
	if (czy_z_pliku)
	{
		char filename[] = "teren1.map";
		int result = Odczyt(filename);
		if (result == -1)
		{
			char path[1024];
			sprintf(path, "..\\%s", filename);
			int result = Odczyt(path);
			if (result == -1)
			{
				sprintf(path, "..\\..\\%s", filename);
				int result = Odczyt(path);
				if (result == -1)
				{
					fprintf(f, "Cannot open terrain map!");
				}
			}
		}
		//Odczyt("terrain.map");
	}
	else
	{
		rozmiar_sektora = 420;         // d³ugoœæ boku kwadratu w [m]
		czas_odnowy_przedm = 120;
		czy_toroidalnosc = false;
		granica_x = -1;
		granica_z = -1;
		liczba_przedmiotow = 0;
		liczba_przedmiotow_max = 10;
		
		p = new Przedmiot[liczba_przedmiotow_max];
	}
	for (long i = 0; i < liczba_przedmiotow; i++)
		if (p[i].typ == PRZ_DRZEWO)
		{
			//p[i].wartosc = 20;
			//this->UmieszczeniePrzedmiotuWterrainie(&p[i]);
		}

	//Zapis("mapa4");
	//czas_odnowy_przedm = 120;   // czas [s] po jakim przedmiot jest odnawiany
}



int Terrain::Zapis(char nazwa_pliku[])
{
	FILE *pl = fopen(nazwa_pliku, "wb");
	if (!pl)
	{
		printf("Nie dalo sie otworzyc pliku %s do zapisu!\n", nazwa_pliku);
		return 0;
	}
	long liczba_sekt = 0;
	for (long i = 0; i < ts->liczba_komorek; i++)
	{
		for (long j = 0; j < ts->komorki[i].liczba_sektorow; j++)
			if (ts->komorki[i].sektory[j]->mapa_wysokosci) liczba_sekt++;
	}

	fwrite(&rozmiar_sektora, sizeof(float), 1, pl);
	fwrite(&czas_odnowy_przedm, sizeof(float), 1, pl);
	fwrite(&czy_toroidalnosc, sizeof(bool), 1, pl);
	fwrite(&granica_x, sizeof(float), 1, pl);
	fwrite(&granica_z, sizeof(float), 1, pl);
	fwrite(&liczba_sekt, sizeof(long), 1, pl);
	fprintf(f, "jest %d sektorow z mapami wysokosciowych + opisem nawierzchni + poz.wody lub innymi informacjami \n", liczba_sekt);
	for (long i = 0; i < ts->liczba_komorek; i++)
	{
		fprintf(f, "  w komorce %d tab.hash jest %d sektorow:\n", i, ts->komorki[i].liczba_sektorow);
		for (long j = 0; j < ts->komorki[i].liczba_sektorow; j++)
		{
			Sektor *s = ts->komorki[i].sektory[j];
			int czy_mapa = (s->mapa_wysokosci != 0 ? 1 : 0);
			bool czy_ogolne_wartosci = (s->typ_naw_sek != 0) && (s->wysokosc_gruntu_sek != 0) && (s->poziom_wody_sek > -1e10);
			if (czy_mapa || czy_ogolne_wartosci)
			{
				fwrite(&ts->komorki[i].sektory[j]->w, sizeof(long), 1, pl);
				fwrite(&ts->komorki[i].sektory[j]->k, sizeof(long), 1, pl);
				int loczek = ts->komorki[i].sektory[j]->liczba_oczek;
				fwrite(&loczek, sizeof(int), 1, pl);
				fwrite(&czy_mapa, sizeof(int), 1, pl);
				if (czy_mapa)
				{

					for (int wx = 0; wx < loczek * 2 + 1; wx++)
						fwrite(ts->komorki[i].sektory[j]->mapa_wysokosci[wx], sizeof(float), loczek + 1, pl);
					for (int w = 0; w < loczek; w++)
						fwrite(ts->komorki[i].sektory[j]->typy_naw[w], sizeof(int), loczek, pl);
					for (int w = 0; w < loczek; w++)
						fwrite(ts->komorki[i].sektory[j]->poziom_wody[w], sizeof(float), loczek, pl);

					fprintf(f, "    sekt.%d z mapa, w = %d, k = %d, rozmiar = %d\n", j, ts->komorki[i].sektory[j]->w,
						ts->komorki[i].sektory[j]->k, loczek);
				}
				else
				{
					fwrite(&ts->komorki[i].sektory[j]->typ_naw_sek, sizeof(int), 1, pl);
					fwrite(&ts->komorki[i].sektory[j]->wysokosc_gruntu_sek, sizeof(float), 1, pl);
					fwrite(&ts->komorki[i].sektory[j]->poziom_wody_sek, sizeof(float), 1, pl);
					fprintf(f, "    sekt.%d bez mapy, w = %d, k = %d, rozmiar = %d\n", j, ts->komorki[i].sektory[j]->w,
						ts->komorki[i].sektory[j]->k, loczek);
				}
			} // jeœli jakaœ informacja jest w sektorze poza przedmiotami i obiektami ruchomymi
		} // po sektorach
	} // po komórkach tabl.hash
	fwrite(&liczba_przedmiotow, sizeof(long), 1, pl);
	for (long i = 0; i < liczba_przedmiotow; i++)
		fwrite(&p[i], sizeof(Przedmiot), 1, pl);
	long liczba_par_edycji_fald = 10;
	fwrite(&liczba_par_edycji_fald, sizeof(long), 1, pl);
	for (long i = 0; i < liczba_par_edycji_fald; i++)
		fwrite(&pf_rej[i], sizeof(ParamFaldy), 1,pl);

	fclose(pl);


	//fstream fbin;
	//fbin.open(filename.c_str(), ios::binary | ios::in | ios::out);

	return 1;
}



int Terrain::Odczyt(char nazwa_pliku[])
{
	FILE *pl = fopen(nazwa_pliku, "rb");
	if (!pl)
	{
		printf("Nie dalo sie otworzyc pliku %s do odczytu!\n", nazwa_pliku);
		fprintf(f, "Nie dalo sie otworzyc pliku %s do odczytu!\n", nazwa_pliku);
		return -1;
	}

	fread(&rozmiar_sektora, sizeof(float), 1, pl);
	fread(&czas_odnowy_przedm, sizeof(float), 1, pl);
	fread(&czy_toroidalnosc, sizeof(bool), 1, pl);
	fread(&granica_x, sizeof(float), 1, pl);
	fread(&granica_z, sizeof(float), 1, pl);

	long _liczba_sekt = 0;        // czyli liczba sektorów 
	fread(&_liczba_sekt, sizeof(long), 1, pl);

	fprintf(f, "\nOdczyt danych o terrainie z pliku %s\n\n", nazwa_pliku);
	fprintf(f, "  rozmiar sektora = %f\n", rozmiar_sektora);
	fprintf(f, "  czas odnowy przedmiotu = %f\n", czas_odnowy_przedm);
	fprintf(f, "  czy_toroidalnosc = %d\n", czy_toroidalnosc);
	fprintf(f, "  granica_x = %f\n", granica_x);
	fprintf(f, "  granica_z = %f\n", granica_z);
	fprintf(f, "  liczba sekt. = %d\n", _liczba_sekt);

	for (long i = 0; i < _liczba_sekt; i++)
	{
		long w, k;
		int loczek, czy_mapa = true;
		fread(&w, sizeof(long), 1, pl);
		fread(&k, sizeof(long), 1, pl);
		fread(&loczek, sizeof(int), 1, pl);
		fread(&czy_mapa, sizeof(int), 1, pl);      // czy dok³adna mapa
		fprintf(f, "    mapa w=%d, k=%d, l.oczek = %d\n", w, k, loczek);

		Sektor *o = new Sektor(loczek, w, k, czy_mapa);
		ts->wstaw(o);
		if (czy_mapa)
		{
			for (int wx = 0; wx < loczek * 2 + 1; wx++)
				fread(o->mapa_wysokosci[wx], sizeof(float), loczek + 1, pl);
			for (int w = 0; w < loczek; w++)
				fread(o->typy_naw[w], sizeof(int), loczek, pl);
			for (int w = 0; w < loczek; w++)
				fread(o->poziom_wody[w], sizeof(float), loczek, pl);
			o->oblicz_normalne(rozmiar_sektora);
		}
		else
		{		
			fread(&o->typ_naw_sek, sizeof(int), 1, pl);
			fread(&o->wysokosc_gruntu_sek, sizeof(float), 1, pl);
			fread(&o->poziom_wody_sek, sizeof(float), 1, pl);
		}
	}
	long _liczba_przedm = 0;
	fread(&_liczba_przedm, sizeof(long), 1, pl);
	fprintf(f, "Znaleziono w pliku %s przedmioty w liczbie %d\n", nazwa_pliku, _liczba_przedm);
	fclose(f);
	f = fopen("wzr_log.txt", "a");
	p = new Przedmiot[_liczba_przedm];
	for (long i = 0; i < _liczba_przedm; i++)
	{
		fread(&p[i], sizeof(Przedmiot), 1, pl);
		fprintf(f, "  przedm.%d, typ = %d, wartosc = %f\n", i,p[i].typ,p[i].wartosc);
		fclose(f);
		f = fopen("wzr_log.txt", "a");
		p[i].indeks = i;
		WstawPrzedmiotWsektory(&p[i]);
		UmieszczeniePrzedmiotuWterrainie(&p[i]);
		if (p[i].czy_zazn) {
			p[i].czy_zazn = 0;
			ZaznaczOdznaczPrzedmiotLubGrupe(i);
		}
	}
	this->liczba_przedmiotow_max = this->liczba_przedmiotow = _liczba_przedm;

	long liczba_par_edycji_fald;
	fread(&liczba_par_edycji_fald, sizeof(long), 1, pl);
	for (long i = 0; i < liczba_par_edycji_fald; i++)
		fread(&pf_rej[i], sizeof(ParamFaldy), 1, pl);

	fprintf(f, "Koniec wczytywania danych o terrainie.\n");
	fclose(pl);
	return 1;
}

void Terrain::WspSektora(long *w, long *k, float x, float z)  // na podstawie wsp. po³o¿enia punktu (x,z) zwraca wsp. sektora
{
	// punkt (x,z) = (0,0) odpowiada œrodkowi sektora (w,k) = (0,0)
	long _k, _w;
	if (x > 0)
	_k = (long)(x / rozmiar_sektora + 0.5);
	else
	{
		float _kf = x / rozmiar_sektora - 0.5;
		_k = (long)(_kf); // przy dodatnich rzutowanie obcina w dó³, przy ujemnych w górê!
		if ((long)_kf == _kf) _k++;
	}
	if (z > 0)
	_w = (long)(z / rozmiar_sektora + 0.5);
	else
	{
		float _wf = z / rozmiar_sektora - 0.5;
		_w = (long)(_wf);
		if ((long)_wf == _wf) _w++;
	}

	*w = _w; *k = _k;
}

void Terrain::PolozeniePoczSektora(float *x, float *z, long w, long k) // na podstawie wspó³rzêdnych sektora (w,k) 
{												              // zwraca po³o¿enie punktu pocz¹tkowego sektora (x,z)
	float _x = (k - 0.5)*rozmiar_sektora,
		_z = (w - 0.5)*rozmiar_sektora;

	(*x) = _x;
	(*z) = _z;
}

Terrain::~Terrain()
{
	for (long i = 0; i < ts->liczba_komorek; i++)
		for (long j = 0; j < ts->komorki[i].liczba_sektorow; j++)
			delete ts->komorki[i].sektory[j];

	delete ts;
	delete p;
}

// punkt przeciêcia prostej przechodz¹cej przez punkt pol z górn¹ p³aszczyzn¹ najwy¿ej po³o¿onego przedmiotu spoœród
// zaznaczonych. Jeœli ¿aden nie le¿y na linii przeciêcia, to zwracana jest wartoœæ -1e10.  
float Terrain::wysokosc_na_najw_zazn_przedm(Wektor3 pol)
{
	// szukamy przedmiotów w promieniu
	Przedmiot **wsk_prz = NULL;
	long liczba_prz_w_prom = Przedmioty_w_promieniu(&wsk_prz, pol, 0);
	float wysokosc_maks = -1e10;
	Przedmiot *wsk_min = NULL;  // wskaŸnik do przedmiotu, od którego liczona jest minimalna wysokoœæ
	for (long i = 0; i < liczba_prz_w_prom; i++)
		if (wsk_prz[i]->czy_zazn)
		{
			float wys_na_p = WysokoscNaPrzedmiocie(pol, wsk_prz[i]);
			//if (wsk_prz[i]->typ == PRZ_MUR)
			{
				if (wysokosc_maks < wys_na_p)
				{
					wysokosc_maks = wys_na_p;
					wsk_min = wsk_prz[i];
				}
			}
		}

	delete wsk_prz;
	return wysokosc_maks;
}

// iteracyjne wyznaczenie kursora 3D z uwazglêdnieniem wysokoœci terrainu i zaznaczonych przedmiotów, na których
// mo¿e le¿eæ nowo-tworzony przedmiot
Wektor3 Terrain::wspolrzedne_kursora3D_bez_paralaksy(int X, int Y)
{
	Wektor3 w = Wektor3(0, 0, 0);
	float wys_na_prz;
	for (int i = 0; i < 7; i++)
	{
		w = WspolrzedneKursora3D(X, Y, w.y);
		wys_na_prz = wysokosc_na_najw_zazn_przedm(w);
		if (wys_na_prz > -1e9)
			w.y = wys_na_prz;
		else
			w.y = WysokoscGruntu(w.x, w.z);
	}
	return w;
}

void Terrain::UmieszczeniePrzedmiotuWterrainie(Przedmiot *prz) // wyzn. par. przedmiotu w odniesieniu do terrainu
{
	// uzupe³nienie wartoœci niektórych parametrów przedmiotu:
	prz->czy_odnawialny = 1;
	float grubosc = 0;
	switch (prz->typ)
	{
	case PRZ_MONETA:
	{
		prz->srednica = powf(prz->wartosc / 100, 0.4);
		grubosc = 0.2*prz->srednica;
		
		prz->do_wziecia = 1;
		prz->srednica_widocz = sqrt(prz->srednica*prz->srednica + grubosc*grubosc);
		break;
	}
	case PRZ_BECZKA:
	{
		prz->srednica = powf((float)prz->wartosc / 50, 0.4);
		grubosc = 2 * prz->srednica;
		//prz->wPol.y = grubosc + WysokoscGruntu(prz->wPol.x, prz->wPol.z);
		prz->do_wziecia = 1;
		prz->srednica_widocz = sqrt(prz->srednica*prz->srednica + grubosc*grubosc);
		break;
	}
	case PRZ_DRZEWO:
		prz->do_wziecia = 0;

		switch (prz->podtyp)
		{
		case DRZ_TOPOLA:
		{
			prz->srednica = 0.65 + prz->wartosc / 40;
			break;
		}
		case DRZ_SWIERK:
		{
			prz->srednica = prz->wartosc / 10;
			break;
		}
		case DRZ_BAOBAB:
		{
			prz->srednica = prz->wartosc / 5;
			break;
		}
		case DRZ_FANTAZJA:
		{
			prz->srednica = prz->wartosc / 10;
			break;
		}
		}
		//prz->wPol.y = prz->wartosc + WysokoscGruntu(prz->wPol.x, prz->wPol.z);
		prz->srednica_widocz = prz->wartosc;
		grubosc = prz->wartosc;
		break;
	case PRZ_PUNKT:
	{
		//prz->wPol.y = 0.0 + WysokoscGruntu(prz->wPol.x, prz->wPol.z);
		break;
	}
	} // switch typ przedmiotu

	// obliczenie wysokoœci na jakiej le¿y przedmiot 
	if (prz->param_f[1] > -1e10)      // gdy le¿y on na jakimœ innym przedmiocie prz->param_f[1] jest wysokoœci¹ od której zaczynamy poszukiwania
	{                                 // jeœli schodz¹c w dó³ ¿adnego przedmiotu nie napotkamy, to przedmiot znajdzie siê na gruncie
		prz->wPol.y = grubosc + prz->param_f[1];
		prz->wPol.y = grubosc + Wysokosc(prz->wPol);
	}
	else                              // gdy le¿y bezpoœrednio na gruncie
		prz->wPol.y = grubosc + WysokoscGruntu(prz->wPol.x, prz->wPol.z);
}

long Terrain::WstawPrzedmiot(Przedmiot prz)
{
	long ind = liczba_przedmiotow;
	if (liczba_przedmiotow >= liczba_przedmiotow_max) // gdy nowy przedmiot nie mieœci siê w tablicy przedmiotów
	{
		long nowa_liczba_przedmiotow_max = 2 * liczba_przedmiotow_max;
		Przedmiot *p_nowe = new Przedmiot[nowa_liczba_przedmiotow_max];
		for (long i = 0; i < liczba_przedmiotow; i++) {
			p_nowe[i] = p[i];
			UsunPrzedmiotZsektorow(&p[i]);   // trzeba to zrobiæ gdy¿ zmieniaj¹ siê wskaŸniki !
			WstawPrzedmiotWsektory(&p_nowe[i]);
		}
		delete p;
		p = p_nowe;
		liczba_przedmiotow_max = nowa_liczba_przedmiotow_max;
	}
	prz.indeks = ind;
	p[ind] = prz;
	
	fprintf(f, "Utworzono przedmiot %d indeks = %d, typ = %d (%s)\n", &p[ind], ind, p[ind].typ, PRZ_nazwy[p[ind].typ]);
	UmieszczeniePrzedmiotuWterrainie(&p[ind]);
	WstawPrzedmiotWsektory(&p[ind]);
	//fprintf(f,"wstawiam przedmiot %d w miejsce (%f, %f, %f)\n",i,terrain.p[ind].wPol.x,terrain.p[ind].wPol.y,terrain.p[ind].wPol.z);
	

	liczba_przedmiotow++;
	return ind;
}

void Terrain::ZaznaczOdznaczPrzedmiotLubGrupe(long nr_prz)
{
	if (p[nr_prz].grupa > -1)       // zaznaczanie/ odznaczanie grupy
	{ 
		long grupa = p[nr_prz].grupa;
		if (p[nr_prz].czy_zazn)     // odznaczanie grupy
		{
			for (long i = 0; i < liczba_przedmiotow; i++)
				if (p[i].grupa == grupa)
				{
					p[i].czy_zazn = 0;
					for (long j = 0; j < liczba_zazn_przedm; j++)  // przeszukujemy listê zaznaczonych przedmiotów by usun¹æ zaznaczenie
					{
						if (zazn_przedm[j] == i)
						{
							zazn_przedm[j] = zazn_przedm[liczba_zazn_przedm - 1];
							liczba_zazn_przedm--;
						}
					}
				}
		}
		else                        // zaznaczanie grupy
		{
			for (long i = 0; i < liczba_przedmiotow; i++)
				if (p[i].grupa == grupa)
				{
					p[i].czy_zazn = 1;
					if (liczba_zazn_przedm == liczba_zazn_przedm_max) // trzeba powiêkszyæ tablicê
					{
						long *__zazn_przedm = new long[liczba_zazn_przedm_max * 2];
						for (long j = 0; j < liczba_zazn_przedm; j++) __zazn_przedm[j] = zazn_przedm[j];
						delete zazn_przedm;
						zazn_przedm = __zazn_przedm;
						liczba_zazn_przedm_max *= 2;
					}
					zazn_przedm[liczba_zazn_przedm] = i;   // wstawiam na koniec listy
					liczba_zazn_przedm++;
				}
		}
	}
	else                            // zaznaczanie/odznaczanie przedmiotu
	{
		p[nr_prz].czy_zazn = 1 - p[nr_prz].czy_zazn;
		char lan[128];
		if (p[nr_prz].czy_zazn)
			sprintf(lan, "zaznaczono_przedmiot_%d,_razem_%d_zaznaczonych",
			nr_prz, liczba_zazn_przedm + 1);
		else
			sprintf(lan, "odznaczono_przedmiot_%d__wcisnij_SHIFT+RMB_by_odznaczyc_wszystkie", nr_prz);
		SetWindowText(okno, lan);
		if (p[nr_prz].czy_zazn)   // dodaje do osobnej listy zaznaczonych przedmiotów
		{
			if (liczba_zazn_przedm == liczba_zazn_przedm_max) // trzeba powiêkszyæ tablicê
			{
				long *__zazn_przedm = new long[liczba_zazn_przedm_max * 2];
				for (long i = 0; i < liczba_zazn_przedm; i++) __zazn_przedm[i] = zazn_przedm[i];
				delete zazn_przedm;
				zazn_przedm = __zazn_przedm;
				liczba_zazn_przedm_max *= 2;
			}
			zazn_przedm[liczba_zazn_przedm] = nr_prz;   // wstawiam na koniec listy
			liczba_zazn_przedm++;
		}
		else                             // usuwam z osobnej listy zaznaczonych przedmiotów
		{
			for (long i = 0; i < liczba_zazn_przedm; i++)  // przeszukujemy listê zaznaczonych przedmiotów by usun¹æ zaznaczenie
			{
				if (zazn_przedm[i] == nr_prz)
				{
					zazn_przedm[i] = zazn_przedm[liczba_zazn_przedm - 1];
					liczba_zazn_przedm--;
				}
			}
		}
	}
}

// UWAGA! Na razie procedura s³aba pod wzglêdem obliczeniowym (pêtla po przedmiotach x po zaznaczonych przedmiotach)
// z powodu wystêpowania krawêdzi, które s¹ zale¿ne od punktów
void Terrain::UsunZaznPrzedmioty()
{
	// sprawdzenie czy wœród przeznaczonych do usuniêcia s¹ punkty, które mogê byæ powi¹zane z krawêdziami. 
	// Jeœli tak, to krawêdzie równie¿ nale¿y usun¹æ - mo¿na to zrobiæ poprzez do³¹czenie ich do listy zaznaczonych:
	for (long i = 0; i < liczba_zazn_przedm; i++)
	{
		long nr_prz = zazn_przedm[i];
		if (p[nr_prz].typ == PRZ_PUNKT)
			for (long j = 0; j < liczba_przedmiotow; j++)				
				if (((p[j].typ == PRZ_KRAWEDZ) || (p[j].typ == PRZ_MUR)) && (p[j].czy_zazn == 0) && ((p[j].param_i[0] == nr_prz) || (p[j].param_i[1] == nr_prz)))
					ZaznaczOdznaczPrzedmiotLubGrupe(j);
	}
	

	fprintf(f, "\n\nUsu.prz.: liczba przedm = %d, liczba zazn = %d\n", liczba_przedmiotow,liczba_zazn_przedm);
	fprintf(f, "  lista przedmiotow:\n");
	for (long i = 0; i < liczba_przedmiotow; i++){
		fprintf(f, "%d: %d (%s) o wartosci %f\n", i, p[i].typ,PRZ_nazwy[p[i].typ], p[i].wartosc);
		if (p[i].typ == PRZ_KRAWEDZ)
			fprintf(f, "         krawedz pomiedzy punktami %d a %d\n", p[i].param_i[0], p[i].param_i[1]);
	}

	for (long i = 0; i < liczba_zazn_przedm; i++)
	{
		long nr_prz = zazn_przedm[i];

		fprintf(f, "  usuwam prz %d: %d (%s) o wartosci %f\n", nr_prz, p[nr_prz].typ,PRZ_nazwy[p[nr_prz].typ], p[nr_prz].wartosc);

		UsunPrzedmiotZsektorow(&p[nr_prz]);
		
		// usuniêcie przedmiotu z listy przedmiotów:
		if (nr_prz != liczba_przedmiotow - 1)
		{
			UsunPrzedmiotZsektorow(&p[liczba_przedmiotow - 1]);
			p[nr_prz] = p[liczba_przedmiotow - 1];         // przepisuje w miejsce usuwanego ostatni przedmiot na liœcie
			p[nr_prz].indeks = nr_prz;

			// przedmiot ostatni na liœcie uzyskuje nowy indeks i jeœli jest to punkt, to trzeba poprawiæ te¿ odwo³ania do niego w krawêdziach:
			if (p[liczba_przedmiotow - 1].typ == PRZ_PUNKT)
				for (long j = 0; j < liczba_przedmiotow;j++)
					if ((p[j].typ == PRZ_KRAWEDZ) || (p[j].typ == PRZ_MUR))
					{
						if (p[j].param_i[0] == liczba_przedmiotow - 1) p[j].param_i[0] = nr_prz;
						if (p[j].param_i[1] == liczba_przedmiotow - 1) p[j].param_i[1] = nr_prz;
					}
				

			WstawPrzedmiotWsektory(&p[nr_prz]);                  // wstawiam go w sektory
			// zamiana numerów na liœcie zaznaczonych jeœli ostatnmi na niej wystêpuje:
			if (p[nr_prz].czy_zazn)
				for (long k = i + 1; k < liczba_zazn_przedm; k++)
					if (zazn_przedm[k] == liczba_przedmiotow - 1) zazn_przedm[k] = nr_prz;
		}
		// byæ mo¿e trzeba by jeszcze pozamieniaæ wskaŸniki przedmiotów w sektorach

		liczba_przedmiotow--;
	}
	liczba_zazn_przedm = 0;

}

void Terrain::NowaMapa()
{
	for (long i = 0; i < ts->liczba_komorek; i++)
		for (long j = 0; j < ts->komorki[i].liczba_sektorow; j++)
			delete ts->komorki[i].sektory[j];

	delete ts;
	delete p;
	this->liczba_przedmiotow = 0;
	this->liczba_przedmiotow_max = 10;
	p = new Przedmiot[liczba_przedmiotow_max];
	ts = new TabSektorow();

	liczba_zazn_przedm = 0;
}

float Terrain::WysokoscGruntu(float x, float z)      // okreœlanie wysokoœci dla punktu o wsp. (x,z) 
{
	float y = 0;    // wysokoœæ standardowa;

	long w, k;
	this->WspSektora(&w, &k, x, z);               // w,k - wspó³rzêdne sektora, w którym znajduje siê punkt (x,z)

	Sektor *s = ts->znajdz(w, k);

	float **mapa = NULL;
	Wektor3 ****__Norm = NULL;
	long loczek = 0;
	if (s)
	{
		if (s->mapa_wysokosci_edycja)
		{
			mapa = s->mapa_wysokosci_edycja;
			__Norm = s->Norm_edycja;
			loczek = s->liczba_oczek_edycja;
		}
		else
		{
			mapa = s->mapa_wysokosci;
			__Norm = s->Norm;
			loczek = s->liczba_oczek;
		}
	}

	if (mapa)
	{
		float rozmiar_pola = (float)this->rozmiar_sektora / loczek;
		float x_pocz_sek, z_pocz_sek;             // wspó³rzêdne po³o¿enia pocz¹tku sektora
		this->PolozeniePoczSektora(&x_pocz_sek, &z_pocz_sek, w, k);
		float x_lok = x - x_pocz_sek, z_lok = z - z_pocz_sek;   // wspó³rzêdne lokalne wewn¹trz sektora
		long k_lok = (long)(x_lok / rozmiar_pola), w_lok = (long)(z_lok / rozmiar_pola);  // lokalne wspó³rzêdne pola w sektorze


		if ((k_lok < 0) || (w_lok < 0))
		{
			fprintf(f,"procedura Terrain::Wysokosc: k i w lokalne nie moga byc ujemne!\n");
			fclose(f);
			exit(1);
		}

		if ((k_lok > loczek - 1) || (w_lok > loczek - 1))
		{
			fprintf(f, "procedura Terrain::Wysokosc: k i w lokalne nie moga byc wieksze od liczby oczek - 1!\n");
			fclose(f);
			exit(1);
		}

		// tworzê trójk¹t ABC w górnej czêœci kwadratu: 
		//  A o_________o C
		//    |.       .|
		//    |  .   .  | 
		//    |    o B  | 
		//    |  .   .  |
		//    |._______.|
		//  D o         o E
		// wyznaczam punkt B - œrodek kwadratu oraz trójk¹t, w którym znajduje siê punkt
		Wektor3 B = Wektor3((k_lok + 0.5)*rozmiar_pola, mapa[w_lok * 2 + 1][k_lok], (w_lok + 0.5)*rozmiar_pola);
		enum tr{ ABC = 0, ADB = 1, BDE = 2, CBE = 3 };       // trójk¹t w którym znajduje siê punkt 
		int trojkat = 0;
		if ((B.x > x_lok) && (fabs(B.z - z_lok) < fabs(B.x - x_lok))) trojkat = ADB;
		else if ((B.x < x_lok) && (fabs(B.z - z_lok) < fabs(B.x - x_lok))) trojkat = CBE;
		else if (B.z > z_lok) trojkat = ABC;
		else trojkat = BDE;

		// wyznaczam normaln¹ do p³aszczyzny a nastêpnie wspó³czynnik d z równania p³aszczyzny
		Wektor3 N = __Norm[0][w_lok][k_lok][trojkat];
		float dd = -(B^N);          // wyraz wolny z równania plaszyzny trójk¹ta
		//float y;
		if (N.y > 0) y = (-dd - N.x*x_lok - N.z*z_lok) / N.y;
		else y = 0;

	} // jeœli sektor istnieje i zawiera mapê wysokoœci

	return y;
}

// wysokoœæ na jakiej znajdzie siê przeciêcie pionowej linii przech. przez pol na górnej powierzchni przedmiotu
float Terrain::WysokoscNaPrzedmiocie(Wektor3 pol, Przedmiot *prz)
{
	float wys_na_p = -1e10;
	if (prz->typ == PRZ_MUR)
	{
		// sprawdzamy, czy rzut pionowy pol mieœci siê w prostok¹cie muru:
		Wektor3 A = p[prz->param_i[0]].wPol, B = p[prz->param_i[1]].wPol;       // punkty tworz¹ce krawêdŸ
		A.y += p[prz->param_i[0]].wartosc;
		B.y += p[prz->param_i[1]].wartosc;

		Wektor3 AB = B - A;
		Wektor3 m_pion = Wektor3(0, 1, 0);     // pion muru
		float m_wysokosc = prz->wartosc;       // od prostej AB do góry jest polowa wysokoœci, druga polowa idzie w dó³ 
		Wektor3 m_wlewo = (m_pion*AB).znorm();          // wektor jednostkowy w lewo od wektora AB i prostopadle do pionu
		float m_szerokosc = prz->param_f[0];

		Wektor3 N = (AB*m_wlewo).znorm();            // normalna do p³aszczyzny górnej muru
		Wektor3 R = punkt_przec_prostej_z_plaszcz(pol, pol - m_pion, N, A); // punkt spadku punktu pol na górn¹ p³. muru
		Wektor3 RR = rzut_punktu_na_prosta(R, A, B);                        // rzut punktu R na prost¹ AB
		bool czy_RR_na_odcAB = (((RR - A) ^ (RR - B)) < 0);                 // czy punkt ten le¿y na odcinku AB (mieœci siê na d³ugoœci muru)
		Wektor3 R_RR = RR - R;
		float odl = fabs(R_RR.x*m_wlewo.x + R_RR.z*m_wlewo.z);              // odleg³oœæ R od osi muru (niewymagaj¹ca pierwiastkowania)
		if ((odl < m_szerokosc / 2) && (czy_RR_na_odcAB))  // warunek tego, ¿e rzut pionowy pol mieœci siê w prostok¹cie muru
			wys_na_p = (R.y + m_wysokosc / 2);               // wysokoœæ punktu pol nad górn¹ p³. muru
	} // jeœli mur
	return wys_na_p;
}

// Okreœlenie wysokoœci nad najbli¿szym przedmiotem (obiektem ruchomym) lub gruntem patrz¹c w dó³ od punktu pol
// wysokoœæ mo¿e byæ ujemna, co oznacza zag³êbienie w gruncie lub przedmiocie
float Terrain::Wysokosc(Wektor3 pol)                 
{
	// szukamy przedmiotów w promieniu
	Przedmiot **wsk_prz = NULL;
	long liczba_prz_w_prom = Przedmioty_w_promieniu(&wsk_prz, pol, 0);
	
	float wysokosc_gruntu = WysokoscGruntu(pol.x, pol.z);
	float roznica_wys_min = pol.y-wysokosc_gruntu;

	Przedmiot *wsk_min = NULL;  // wskaŸnik do przedmiotu, od którego liczona jest minimalna wysokoœæ
	for (long i = 0; i < liczba_prz_w_prom; i++)
	{		
		float wys_na_p = this->WysokoscNaPrzedmiocie(pol, wsk_prz[i]);
		if (wys_na_p > -1e10)
			if (wsk_prz[i]->typ == PRZ_MUR)
			{
				float roznica_wys = pol.y - wys_na_p;               // wysokoœæ punktu pol nad górn¹ p³. muru
				float m_wysokosc = wsk_prz[i]->wartosc;
				//bool czy_wewn_muru = ((roznica_wys < 0) && (-roznica_wys < m_wysokosc));  // czy punkt pol znajduje siê wewn¹trz muru
				if ((roznica_wys < roznica_wys_min) && 
					((roznica_wys > 0) || (-roznica_wys < m_wysokosc)))  // ponad murem lub wewn¹trz muru (nie pod spodem!)
				{
					roznica_wys_min = roznica_wys;
					wsk_min = wsk_prz[i];
				}		
			} // jeœli mur
	}
	delete wsk_prz;

	// Tutaj nale¿a³oby jeszcze sprawdziæ wysokoœæ nad obiektami ruchomymi


	return pol.y - roznica_wys_min;
}

void Terrain::PoczatekGrafiki()
{


}


void Terrain::WstawPrzedmiotWsektory(Przedmiot *prz)
{
	// przypisanie przedmniotu do okreœlonego obszaru -> umieszczenie w tablicy wskaŸników 
	// wspó³rzêdne kwadratu opisuj¹cego przedmiot:
	float x1 = prz->wPol.x - prz->srednica / 2, x2 = prz->wPol.x + prz->srednica / 2,
		z1 = prz->wPol.z - prz->srednica / 2, z2 = prz->wPol.z + prz->srednica / 2;

	long k1, w1, k2, w2;
	WspSektora(&w1, &k1, x1, z1);
	WspSektora(&w2, &k2, x2, z2);

	//fprintf(f, "wsp. kwadratu opisujacego przedmiot: w1=%d,w2=%d,k1=%d,k2=%d\n", w1, w2, k1, k2);
	

	// wstawiamy przedmiot w te sektory, w których powinien byæ widoczny:
	for (long w = w1; w <= w2; w++)
		for (long k = k1; k <= k2; k++)
		{
			fprintf(f, "wstawiam przedmiot %d: %s o wartosci %f i srednicy %f w sektor w = %d, k = %d\n", prz,PRZ_nazwy[prz->typ],prz->wartosc, prz->srednica, w, k);
			Sektor *ob = ts->znajdz(w, k);
			if (ob == NULL) {                 // tworzymy sektor, gdy nie istnieje (mo¿na nie tworzyæ mapy)
				ob = new Sektor(0, w, k, false);
				ts->wstaw(ob);
			}
			ob->wstaw_przedmiot(prz);

			//fprintf(f, "  wstawiono przedmiot w sektor (%d, %d) \n", w, k);
		}
}

// Wyszukiwanie wszystkich przedmiotow le¿¹cych w okrêgu o œrodku wyznaczonym
// sk³adowymi x,z wekrora pol i zadanym promieniu. Przedmioty umieszczane s¹ na 
// liœcie wskaŸników wsk_prz. PóŸniej nale¿y zwolniæ pamiêæ: 
//     delete wsk_prz;
// Funkcja zwraca liczbê znalezionych przedmiotów
long Terrain::Przedmioty_w_promieniu(Przedmiot*** wsk_prz, Wektor3 pol, float promien)
{
	Wektor3 pol2D = pol;
	pol2D.y = 0;
	float x1 = pol2D.x - promien, z1 = pol2D.z - promien, x2 = pol2D.x + promien, z2 = pol2D.z + promien;
	long kp1, wp1, kp2, wp2;
	this->WspSektora(&wp1, &kp1, x1, z1);
	this->WspSektora(&wp2, &kp2, x2, z2);
	//fprintf(f,"kp1 = %d, wp1 = %d, kp2 = %d, wp2 = %d\n",kp1,wp1,kp2,wp2);
	float promien_kw = promien*promien;

	Sektor **oby = new Sektor*[(wp2 - wp1 + 1)*(kp2 - kp1 + 1)];
	long liczba_obsz = 0;

	// wyznaczenie sektorów w otoczeniu: 
	long liczba_przedm = 0, liczba_prz_max = 0;            // wyznaczenie maks. liczby przedmiotów
	for (long w = wp1; w <= wp2; w++)
		for (long k = kp1; k <= kp2; k++)
		{
			Sektor *ob = ts->znajdz(w, k);
			if (ob)
			{
				oby[liczba_obsz++] = ob;   // wpicuje obszar do tablicy by ponownie go nie wyszukiwaæ
				liczba_prz_max += ob->liczba_przedmiotow;
			}
		}

	//fprintf(f,"liczba_przedm_max = %d\n",liczba_przedm_max);
	//fclose(f);
	//exit(1);

	if (liczba_prz_max > 0)
	{
		(*wsk_prz) = new Przedmiot*[liczba_prz_max];              // alokacja pamieci dla wskaŸników przedmiotów
		for (long n = 0; n < liczba_obsz; n++)
		{
			Sektor *ob = oby[n];
			for (long i = 0; i < ob->liczba_przedmiotow; i++)
			{
				float odl_kw = 0;
				if ((ob->wp[i]->typ == PRZ_KRAWEDZ) || (ob->wp[i]->typ == PRZ_MUR))   // przedmioty odcinkowe
				{
					Wektor3 A = p[ob->wp[i]->param_i[0]].wPol, B = p[ob->wp[i]->param_i[1]].wPol;
					A.y = 0; B.y = 0;
					float odl = odleglosc_pom_punktem_a_odcinkiem(pol2D, A, B) - ob->wp[i]->param_f[0] / 2;
					if (odl < 0) odl = 0;
					odl_kw = odl*odl;
				}
				else
					odl_kw = (ob->wp[i]->wPol.x - pol2D.x)*(ob->wp[i]->wPol.x - pol2D.x) +
					(ob->wp[i]->wPol.z - pol2D.z)*(ob->wp[i]->wPol.z - pol2D.z) - ob->wp[i]->srednica*ob->wp[i]->srednica / 4;
				if (odl_kw <= promien_kw)
				{
					(*wsk_prz)[liczba_przedm] = ob->wp[i];
					liczba_przedm++;
				}
			}
		}
	} // if maks. liczba przedmiotów > 0
	delete oby;
	return liczba_przedm;
}


void Terrain::WstawObiektWsektory(MovableObject *ob)
{
	float x1 = ob->wPol.x - ob->promien, x2 = ob->wPol.x + ob->promien,
		z1 = ob->wPol.z - ob->promien, z2 = ob->wPol.z + ob->promien;

	long k1, w1, k2, w2;
	WspSektora(&w1, &k1, x1, z1);
	WspSektora(&w2, &k2, x2, z2);

	//fprintf(f, "wsp. kwadratu opisujacego przedmiot: w1=%d,w2=%d,k1=%d,k2=%d\n", w1, w2, k1, k2);

	// wstawiamy obiekt ruchomy w te sektory, w których powinien byæ widoczny:
	for (long w = w1; w <= w2; w++)
		for (long k = k1; k <= k2; k++)
		{
			Sektor *obsz = ts->znajdz(w, k);
			if (obsz == NULL) {                 // tworzymy sektor, gdy nie istnieje (mo¿na nie tworzyæ mapy)
				obsz = new Sektor(0, w, k, false);
				ts->wstaw(obsz);
			}
			obsz->wstaw_obiekt_ruchomy(ob);

			//fprintf(f, "  wstawiono obiekt ruchomy w sektor (%d, %d) \n", w, k);
		}

}
void Terrain::UsunObiektZsektorow(MovableObject *ob)
{
	float x1 = ob->wPol.x - ob->promien, x2 = ob->wPol.x + ob->promien,
		z1 = ob->wPol.z - ob->promien, z2 = ob->wPol.z + ob->promien;

	long k1, w1, k2, w2;
	WspSektora(&w1, &k1, x1, z1);
	WspSektora(&w2, &k2, x2, z2);

	for (long w = w1; w <= w2; w++)
		for (long k = k1; k <= k2; k++)
		{
			Sektor *obsz = ts->znajdz(w, k);
			if (obsz == NULL) {                 // to nie powinno siê zdarzaæ!

			}
			else
			{
				obsz->usun_obiekt_ruchomy(ob);
				//fprintf(f, "  usuniêto obiekt ruchomy z sektora (%d, %d) \n", w, k);

				// mo¿na jeszcze sprawdziæ czy sektor jest pusty (brak mapy, przedmiotów i ob. ruchomych) i usun¹æ sektor:
				if ((obsz->liczba_obiektow_ruch == 0) && (obsz->liczba_przedmiotow == 0) &&
					(obsz->typy_naw == NULL) && (obsz->mapa_wysokosci == NULL) && (obsz->mapa_wysokosci_edycja == NULL))
				{
					ts->usun(obsz);
					//delete obsz->wob;  // to jest usuwane w destruktorze ( dwie linie dalej!)
					//delete obsz->wp;
					delete obsz;
				}
			}
			//fprintf(f, "  usunieto obiekt ruchomy w sektora (%d, %d) \n", w, k);
		}

}

void Terrain::UsunPrzedmiotZsektorow(Przedmiot *prz)
{
	float x1 = prz->wPol.x - prz->srednica / 2, x2 = prz->wPol.x + prz->srednica / 2,
		z1 = prz->wPol.z - prz->srednica / 2, z2 = prz->wPol.z + prz->srednica / 2;

	long k1, w1, k2, w2;
	WspSektora(&w1, &k1, x1, z1);
	WspSektora(&w2, &k2, x2, z2);

	for (long w = w1; w <= w2; w++)
		for (long k = k1; k <= k2; k++)
		{
			Sektor *sek = ts->znajdz(w, k);
			if (sek == NULL) {                 // to nie powinno siê zdarzaæ!

			}
			else
			{
				sek->usun_przedmiot(prz);
				fprintf(f, "  usuniêto przedmiot %d: %s o wartosci %f z sektora (%d, %d) \n", prz, PRZ_nazwy[prz->typ],prz->wartosc,w, k);

				// mo¿na jeszcze sprawdziæ czy sektor jest pusty (brak mapy, przedmiotów i ob. ruchomych) i usun¹æ sektor:
				if ((sek->liczba_obiektow_ruch == 0) && (sek->liczba_przedmiotow == 0) &&
					(sek->typy_naw == NULL) && (sek->mapa_wysokosci == NULL))
				{
					ts->usun(sek);
					//delete obsz->wob;  // to jest usuwane w destruktorze ( dwie linie dalej!)
					//delete obsz->wp;
					delete sek;
				}
			}
		}

}

long Terrain::Obiekty_w_promieniu(MovableObject*** wsk_ob, Wektor3 pol, float promien)
{
	float x1 = pol.x - promien, z1 = pol.z - promien, x2 = pol.x + promien, z2 = pol.z + promien;
	long kp1, wp1, kp2, wp2;
	this->WspSektora(&wp1, &kp1, x1, z1);
	this->WspSektora(&wp2, &kp2, x2, z2);
	//fprintf(f,"kp1 = %d, wp1 = %d, kp2 = %d, wp2 = %d\n",kp1,wp1,kp2,wp2);
	float promien_kw = promien*promien;

	Sektor **oby = new Sektor*[(wp2 - wp1 + 1)*(kp2 - kp1 + 1)];
	long liczba_obsz = 0;

	long liczba_obiektow = 0, liczba_obiektow_max = 0;            // wyznaczenie maks. liczby przedmiotów
	for (long w = wp1; w <= wp2; w++)
		for (long k = kp1; k <= kp2; k++)
		{
			Sektor *ob = ts->znajdz(w, k);
			if (ob)
			{
				oby[liczba_obsz++] = ob;   // wpicuje obszar do tablicy by ponownie go nie wyszukiwaæ
				liczba_obiektow_max += ob->liczba_obiektow_ruch;
			}
		}

	//fprintf(f,"liczba_przedm_max = %d\n",liczba_przedm_max);
	//fclose(f);
	//exit(1);

	if (liczba_obiektow_max > 0)
	{
		(*wsk_ob) = new MovableObject*[liczba_obiektow_max];              // alokacja pamieci dla wskaŸników obiektów ruchomych
		for (long n = 0; n < liczba_obsz; n++)
		{
			Sektor *ob = oby[n];
			for (long i = 0; i < ob->liczba_obiektow_ruch; i++)
			{
				float odl_kw = (ob->wob[i]->wPol.x - pol.x)*(ob->wob[i]->wPol.x - pol.x) +
					(ob->wob[i]->wPol.z - pol.z)*(ob->wob[i]->wPol.z - pol.z);
				if (odl_kw <= promien_kw)
				{
					(*wsk_ob)[liczba_obiektow] = ob->wob[i];
					liczba_obiektow++;
				}
			}
		}
	} // if maks. liczba przedmiotów > 0
	delete oby;
	return liczba_obiektow;

}



/*
maks. wielkoœæ obiektu w pikselach, widoczna na ekranie, -1 jeœli obiekt w ogóle nie jest widoczny
*/
float ilosc_pikseli_widocznosci(Wektor3 WPol, float srednica)
{
	// Sprawdzenie, czy jakikolwiek z kluczowych punktów sfery opisujacej przedmiot nie jest widoczny
	// (rysowanie z pominiêciem niewidocznych przedmiotów jest znacznie szybsze!)
	bool czy_widoczny = 1;// 0;
	RECT clr;
	GetClientRect(okno, &clr);
	Wektor3 pol_kam = par_wid.pocz_pol_kamery - par_wid.pocz_kierunek_kamery*par_wid.oddalenie;     // po³o¿enie kamery
	//Wektor3 wPol = p.wPol;
	Wektor3 pkt_klucz_sfery[] = { Wektor3(1, 0, 0), Wektor3(-1, 0, 0), Wektor3(0, 1, 0), // kluczowe punkty sfery widocznoœci
		Wektor3(0, -1, 0), Wektor3(0, 0, 1), Wektor3(0, 0, -1) };
	const int liczba_pkt_klucz = sizeof(pkt_klucz_sfery) / sizeof(Wektor3);
	Wektor3 ePol[liczba_pkt_klucz];   // wspolrzedne punktów kluczowych na ekranie (tylko skladowe x,y) 

	for (int pkt = 0; pkt<liczba_pkt_klucz; pkt++)
	{
		Wektor3 wPol2 = WPol + pkt_klucz_sfery[pkt] * srednica;   // po³o¿enie wybranego punktu pocz¹tkowego  
		WspolrzedneEkranu(&ePol[pkt].x, &ePol[pkt].y, &ePol[pkt].z, wPol2);        // wyznaczenie wspó³rzêdnych ekranu
		float ilo_skal = (wPol2 - pol_kam) ^ par_wid.pocz_kierunek_kamery;                      // iloczyn skalarny kierunku od kam do pojazdu i kierunku patrzenia kam
		if ((ePol[pkt].x > clr.left) && (ePol[pkt].x < clr.right) &&               // jeœli wspó³rzêdne ekranu któregokolwiek punktu mieszcz¹ siê
			(ePol[pkt].y > clr.top) && (ePol[pkt].y < clr.bottom) && (ilo_skal > 0))  // w oknie i punkt z przodu kamery to mo¿e on byæ widoczny
			czy_widoczny = 1;                                                        // (mo¿na jeszcze sprawdziæ sto¿ek widocznoœci) 
	}

	// sprawdzenie na ile widoczny jest przedmiot - w tym celu wybierana jest jedna z 3 osi sfery o najwiêkszej 
	// ekranowej odleg³oœci pomiêdzy jej punktami na sferze. Im wiêksza ekranowa odleg³oœæ, tym wiêcej szczegó³ów, 
	// jeœli < 1 piksel, to mo¿na w ogóle przedmiotu nie rysowaæ, jeœli < 10 piks. to mo¿na nie rysowaæ napisu
	float liczba_pix_wid = 0;          // szczegó³owoœæ w pikselach maksymalnie widocznego rozmiaru przedmiotu
	if (czy_widoczny)
	{
		float odl_x = fabs(ePol[0].x - ePol[1].x), odl_y = fabs(ePol[2].y - ePol[3].y),
			odl_z = fabs(ePol[4].z - ePol[5].z);
		liczba_pix_wid = (odl_x > odl_y ? odl_x : odl_y);
		liczba_pix_wid = (odl_z > liczba_pix_wid ? odl_z : liczba_pix_wid);
	}

	if (czy_widoczny == 0) liczba_pix_wid = -1;

	return liczba_pix_wid;
}


void Terrain::Rysuj()
{
	float promien_widnokregu = 8000;

	// rysowanie powierzchni terrainu:
	//glCallList(PowierzchniaTerrainu);
	GLfloat GreySurface[] = { 0.7f, 0.7f, 0.7f, 0.3f };
	GLfloat OrangeSurface[] = { 1.0f, 0.8f, 0.0f, 0.7f };
	GLfloat GreenSurface[] = { 0.45f, 0.42f, 0.3f, 1.0f };

	//GLfloat RedSurface[] = { 0.8f, 0.2f, 0.1f, 0.5f };
	//glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, RedSurface);
	//glEnable(GL_BLEND);
	// rysujemy wszystko co jest w okrêgu o srednicy 1000 m:
	
	long liczba_sekt = promien_widnokregu / this->rozmiar_sektora;  // liczba widocznych sektorów
	long wwx, kkx;                // wspó³rzêdne sektora, w którym znajduje siê kamera
	//Wektor3 pol_kam = pocz_pol_kamery - pocz_kierunek_kamery*oddalenie;     // po³o¿enie kamery
	Wektor3 pol_kam, kierunek_kam, pion_kam;
	UstawieniaKamery(&pol_kam, &kierunek_kam, &pion_kam, par_wid);             // wyznaczenie po³o¿enia kamery i innych jej ustawieñ

	this->WspSektora(&wwx, &kkx, pol_kam.x, pol_kam.z);               // wspó³rzêdne sektora w którym znajduje siê kamera
	//fprintf(f, "pol. kamery = (%f,%f,%f), sektor (%d, %d)\n", pol_kam.x, pol_kam.y, pol_kam.z, wwx, kkx);

	// kwadratury do rysowania przedmiotów:
	GLUquadricObj *Qcyl = gluNewQuadric();
	GLUquadricObj *Qdisk = gluNewQuadric();
	GLUquadricObj *Qsph = gluNewQuadric();

	for (int ww = wwx - liczba_sekt; ww <= wwx + liczba_sekt; ww++)       // po sektorach w otoczeniu kamery
		for (int kk = kkx - liczba_sekt; kk <= kkx + liczba_sekt; kk++)
		{
			//fprintf(f, "szukam sektora o w =%d, k = %d, liczba_sekt = %d\n", ww, kk,liczba_sekt);
			Sektor *sek = ts->znajdz(ww, kk);
			//float pocz_x = -rozmiar_pola*lkolumn / 2,     // wspó³rzêdne lewego górnego krañca terrainu
			//	pocz_z = -rozmiar_pola*lwierszy / 2;
			float pocz_x = (kk-0.5)*rozmiar_sektora;
			float pocz_z = (ww-0.5)*rozmiar_sektora;
			if ((sek) && ((sek->mapa_wysokosci)||(sek->mapa_wysokosci_edycja)))
			{
				//fprintf(f, "znaleziono sektor o w = %d, k = %d\n", sek->w, sek->k);
				long loczek = 0;
				float **mapa;
				Wektor3 ****__Norm;
				if (sek->mapa_wysokosci_edycja)
				{
					mapa = sek->mapa_wysokosci_edycja;
					__Norm = sek->Norm_edycja;
					loczek = sek->liczba_oczek_edycja;
					glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, OrangeSurface);
				}
				else
				{
					mapa = sek->mapa_wysokosci;
					__Norm = sek->Norm;
					loczek = sek->liczba_oczek;
					glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreenSurface);
				}

				// Tutaj nale¿y ustaliæ na ile widoczny (blisko obserwatora) jest sektor i w zal. od tego wybraæ rozdzielczoœæ
				float rozmiar_pola_r0 = rozmiar_sektora / loczek;
				Wektor3 pol = Wektor3(kk*rozmiar_sektora, sek->wysokosc_max, ww*rozmiar_sektora);

				long nr_rozdz = 0;             // nr_rozdzielczoœæ: 0 - podstawowa, kolejne numery odpowiadaj¹ coraz mniejszym rozdzielczoœciom

				float liczba_pix_wid = ilosc_pikseli_widocznosci(pol, rozmiar_pola_r0);
				if (liczba_pix_wid < 20/(szczeg + 0.05))
					nr_rozdz = 1 + (int)log2(20/(szczeg + 0.05) / liczba_pix_wid);			
				
				long zmn = (1<<nr_rozdz);     // zmiejszenie rozdzielczoœci (jeœli sektor daleko od obserwatora)
				if (zmn > loczek)
				{
					zmn = loczek;
					nr_rozdz = log2(loczek);
				}
				//long loczek_nr_rozdz = loczek / zmn;                          // zmniejszona liczba oczek
				
				//sek->liczba_oczek_wyswietlana_pop = sek->liczba_oczek_wyswietlana;
				sek->liczba_oczek_wyswietlana_pop = sek->liczba_oczek_wyswietlana;
				sek->liczba_oczek_wyswietlana = loczek / zmn;                          // rozdzielczoœæ do wyœwietlenia
				long loczek_nr_rozdz = sek->liczba_oczek_wyswietlana_pop;
				float rozmiar_pola = rozmiar_sektora / loczek_nr_rozdz;       // rozmiar pola dla aktualnej rozdzielczoœci
				zmn = loczek / loczek_nr_rozdz;
				nr_rozdz = log2(zmn);

				Sektor *s_lewy = ts->znajdz(ww, kk - 1), *s_prawy = ts->znajdz(ww, kk + 1),   // sektory s¹siednie
					*s_dolny = ts->znajdz(ww + 1, kk), *s_gorny = ts->znajdz(ww - 1, kk);

				
				// tworze listê wyœwietlania rysuj¹c poszczególne pola mapy za pomoc¹ trójk¹tów 
				// (po 4 trójk¹ty na ka¿de pole):
				enum tr{ ABC = 0, ADB = 1, BDE = 2, CBE = 3 };

				Wektor3 A, B, C, D, E, N;
				//glNewList(PowierzchniaTerrainu, GL_COMPILE);
				glBegin(GL_TRIANGLES);
				for (long w = 0; w < loczek_nr_rozdz; w++)
					for (long k = 0; k < loczek_nr_rozdz; k++)
					{						
						if (k > 0)
						{
							A = C;     // dziêki temu, ¿e poruszamy siê z oczka na oczko w okreœlonym kierunku, mo¿emy zaoszczêdziæ
							D = E;     // trochê obliczeñ
						}
						else
						{
							A = Wektor3(pocz_x + k*rozmiar_pola, mapa[w * 2 * zmn][k*zmn], pocz_z + w*rozmiar_pola);
							D = Wektor3(pocz_x + k*rozmiar_pola, mapa[(w + 1) * 2 * zmn][k*zmn], pocz_z + (w + 1)*rozmiar_pola);
						}
						B = Wektor3(pocz_x + (k + 0.5)*rozmiar_pola, mapa[(w * 2 + 1)*zmn][k*zmn + (zmn>1)*zmn / 2], pocz_z + (w + 0.5)*rozmiar_pola);
						C = Wektor3(pocz_x + (k + 1)*rozmiar_pola, mapa[w * 2 * zmn][(k + 1)*zmn], pocz_z + w*rozmiar_pola);
						E = Wektor3(pocz_x + (k + 1)*rozmiar_pola, mapa[(w + 1) * 2 * zmn][(k + 1)*zmn], pocz_z + (w + 1)*rozmiar_pola);
						

						// dopasowanie brzegów sektora do nr_rozdzielczoœci sektorów s¹siednich (o ile maj¹ mniejsz¹ rozdzielczoœæ):
						if ((k==0)&&(s_lewy) && (s_lewy->liczba_oczek_wyswietlana < loczek_nr_rozdz))
						{
							//if ((s_lewy->mapa_wysokosci == NULL) && (s_lewy->mapa_wysokosci_edycja == NULL)) s_lewy->liczba_oczek_wyswietlana = 1;
							int iloraz = loczek_nr_rozdz / s_lewy->liczba_oczek_wyswietlana;
							int reszta = w % iloraz;
							if (reszta > 0)
								A.y = mapa[(w - reszta) * 2 * zmn][0] + 
								(mapa[(w - reszta + iloraz) * 2 * zmn][0] - mapa[(w - reszta) * 2 * zmn][0])*(float)reszta / iloraz;
							reszta = (w+1) % iloraz;
							if (reszta > 0)
								D.y = mapa[(w + 1 - reszta) * 2 * zmn][0] + 
								(mapa[(w + 1 - reszta + iloraz) * 2 * zmn][0] - mapa[(w + 1 - reszta) * 2 * zmn][0])*(float)reszta / iloraz;
						}
						if ((k == loczek_nr_rozdz - 1) && (s_prawy) && (s_prawy->liczba_oczek_wyswietlana < loczek_nr_rozdz))
						{
							int iloraz = loczek_nr_rozdz / s_prawy->liczba_oczek_wyswietlana;
							int reszta = w % iloraz;
							if (reszta > 0)
								C.y = mapa[(w - reszta) * 2 * zmn][loczek_nr_rozdz*zmn] + 
								(mapa[(w - reszta + iloraz) * 2 * zmn][loczek_nr_rozdz*zmn] - mapa[(w - reszta) * 2 * zmn][loczek_nr_rozdz*zmn])*(float)reszta / iloraz;
							reszta = (w + 1) % iloraz;
							if (reszta > 0)
								E.y = mapa[(w + 1 - reszta) * 2 * zmn][loczek_nr_rozdz*zmn] + 
								(mapa[(w + 1 - reszta + iloraz) * 2 * zmn][loczek_nr_rozdz*zmn] - mapa[(w + 1 - reszta) * 2 * zmn][loczek_nr_rozdz*zmn])*(float)reszta / iloraz;
						}
						
						if ((w == 0) && (s_gorny) && (s_gorny->liczba_oczek_wyswietlana < loczek_nr_rozdz))
						{
							int iloraz = loczek_nr_rozdz / s_gorny->liczba_oczek_wyswietlana;
							int reszta = k % iloraz;
							if (reszta > 0)
								A.y = mapa[0][(k - reszta)*zmn] + (mapa[0][(k - reszta + iloraz)*zmn] - mapa[0][(k - reszta)*zmn])*(float)reszta / iloraz;
							reszta = (k+1) % iloraz;
							if (reszta > 0)
								C.y = mapa[0][(k + 1 - reszta)*zmn] + (mapa[0][(k + 1 - reszta + iloraz)*zmn] - mapa[0][(k + 1 - reszta)*zmn])*(float)reszta / iloraz;
						}
						
						if ((w == loczek_nr_rozdz - 1) && (s_dolny) && (s_dolny->liczba_oczek_wyswietlana < loczek_nr_rozdz))
						{
							int iloraz = loczek_nr_rozdz / s_dolny->liczba_oczek_wyswietlana;
							int reszta = k % iloraz;
							if (reszta > 0)
								D.y = mapa[loczek_nr_rozdz*zmn*2][(k - reszta)*zmn] + 
								(mapa[loczek_nr_rozdz*zmn * 2][(k - reszta + iloraz)*zmn] - mapa[loczek_nr_rozdz*zmn * 2][(k - reszta)*zmn])*(float)reszta / iloraz;
							reszta = (k + 1) % iloraz;
							if (reszta > 0)
								E.y = mapa[loczek_nr_rozdz*zmn * 2][(k + 1 - reszta)*zmn] + 
								(mapa[loczek_nr_rozdz*zmn * 2][(k + 1 - reszta + iloraz)*zmn] - mapa[loczek_nr_rozdz*zmn * 2][(k + 1 - reszta)*zmn])*(float)reszta / iloraz;
						}
						// tworzê trójk¹t ABC w górnej czêœci kwadratu: 
						//  A o_________o C
						//    |.       .|
						//    |  .   .  | 
						//    |    o B  | 
						//    |  .   .  |
						//    |._______.|
						//  D o         o E

						//Wektor3 AB = B - A;
						//Wektor3 BC = C - B;
						//N = (AB*BC).znorm();
						N = __Norm[nr_rozdz][w][k][ABC];
						glNormal3f(N.x, N.y, N.z);
						glVertex3f(A.x, A.y, A.z);
						glVertex3f(B.x, B.y, B.z);
						glVertex3f(C.x, C.y, C.z);
						//d[w][k][ABC] = -(B^N);          // dodatkowo wyznaczam wyraz wolny z równania plaszyzny trójk¹ta
						//Norm[w][k][ABC] = N;          // dodatkowo zapisujê normaln¹ do p³aszczyzny trójk¹ta
						// trójk¹t ADB:
						//Wektor3 AD = D - A;
						//N = (AD*AB).znorm();
						N = __Norm[nr_rozdz][w][k][ADB];
						glNormal3f(N.x, N.y, N.z);
						glVertex3f(A.x, A.y, A.z);
						glVertex3f(D.x, D.y, D.z);
						glVertex3f(B.x, B.y, B.z);
						//d[w][k][ADB] = -(B^N);
						//Norm[w][k][ADB] = N;
						// trójk¹t BDE:
						//Wektor3 BD = D - B;
						//Wektor3 DE = E - D;
						//N = (BD*DE).znorm();
						N = __Norm[nr_rozdz][w][k][BDE];
						glNormal3f(N.x, N.y, N.z);
						glVertex3f(B.x, B.y, B.z);
						glVertex3f(D.x, D.y, D.z);
						glVertex3f(E.x, E.y, E.z);
						//d[w][k][BDE] = -(B^N);
						//Norm[w][k][BDE] = N;
						// trójk¹t CBE:
						//Wektor3 CB = B - C;
						//Wektor3 BE = E - B;
						//N = (CB*BE).znorm();
						N = __Norm[nr_rozdz][w][k][CBE];
						glNormal3f(N.x, N.y, N.z);
						glVertex3f(C.x, C.y, C.z);
						glVertex3f(B.x, B.y, B.z);
						glVertex3f(E.x, E.y, E.z);
						//d[w][k][CBE] = -(B^N);
						//Norm[w][k][CBE] = N;
					}
				glEnd();
			} // jeœli znaleziono mapê sektora
			else if (sek)
			{
				// ustawienie tekstury:
				// ....
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreenSurface);
				glBegin(GL_QUADS);
				glNormal3f(0, 1, 0);
				glVertex3f(pocz_x, 0.0, pocz_z);
				glVertex3f(pocz_x, 0.0, pocz_z + rozmiar_sektora);
				glVertex3f(pocz_x + rozmiar_sektora, 0.0, pocz_z + rozmiar_sektora);
				glVertex3f(pocz_x + rozmiar_sektora, 0.0, pocz_z);
				glEnd();
			} // jeœli nie znaleziono mapy ale znaleziono sektor ze standardow¹ nawierzchni¹
			else
			{
				glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreenSurface);
				glBegin(GL_QUADS);
				glNormal3f(0, 1, 0);
				glVertex3f(pocz_x, 0.0, pocz_z);
				glVertex3f(pocz_x, 0.0, pocz_z + rozmiar_sektora);
				glVertex3f(pocz_x + rozmiar_sektora, 0.0, pocz_z + rozmiar_sektora);
				glVertex3f(pocz_x + rozmiar_sektora, 0.0, pocz_z);
				glEnd();

			} // jeœli nie znaleziono sektora
		} // po sektorach


	// RYSOWANIE PRZEDMIOTÓW
	for (int ww = wwx - liczba_sekt; ww <= wwx + liczba_sekt; ww++)       // po sektorach w otoczeniu kamery
		for (int kk = kkx - liczba_sekt; kk <= kkx + liczba_sekt; kk++)
		{
			//fprintf(f, "szukam sektora o w =%d, k = %d, liczba_sekt = %d\n", ww, kk,liczba_sekt);
			Sektor *sek = ts->znajdz(ww, kk);
		
			long liczba_prz = 0;
			if (sek) liczba_prz = sek->liczba_przedmiotow;
			
			for (long prz = 0; prz < liczba_prz; prz++)
			{
				Przedmiot *p = sek->wp[prz];
				if (p->nr_wyswietlenia != this->liczba_wyswietlen)   // o ile jeszcze nie zosta³ wyœwietlony w tym cyklu rysowania
				{
					p->nr_wyswietlenia = liczba_wyswietlen;          // by nie wyœwietlaæ wielokrotnie, np. gdy przedmiot jest w wielu sektorach
					float liczba_pix_wid = ilosc_pikseli_widocznosci(p->wPol, p->srednica_widocz);
					bool czy_widoczny = (liczba_pix_wid >= 0 ? 1 : 0);

					glPushMatrix();

					glTranslatef(p->wPol.x, p->wPol.y, p->wPol.z);
					//glRotatef(k.w*180.0/PI,k.x,k.y,k.z);
					//glScalef(dlugosc,wysokosc,szerokosc);
					switch (p->typ)
					{
					case PRZ_MONETA:
						//gluCylinder(Qcyl,promien1,promien2,wysokosc,10,20);
						if (p->do_wziecia)
						{
							GLfloat Surface[] = { 2.0f, 2.0f, 1.0f, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
							glRotatef(90, 1, 0, 0);
							float grubosc = 0.2*p->srednica;
							//gluDisk(Qdisk,0,p->srednica,10,10);
							//gluCylinder(Qcyl,p->srednica,p->srednica,grubosc,10,20);
							if (liczba_pix_wid < (int)3.0 / (szczeg + 0.001))        // przy ma³ych widocznych rozmiarach rysowanie punktu
							{
								glPointSize((int)liczba_pix_wid*0.3);
								glBegin(GL_POINTS);
								glVertex3f(0, 0, 0);
								glEnd();
							}
							else
							{
								int liczba = (int)(liczba_pix_wid*szczeg / 3);
								if (liczba < 5) liczba = 5;
								if (liczba > 15) liczba = 15;
								gluDisk(Qdisk, 0, p->srednica, liczba, liczba * 2);
								gluCylinder(Qcyl, p->srednica, p->srednica, grubosc, liczba, liczba * 2);
							}
							if (liczba_pix_wid > 5)
							{
								glRasterPos2f(0.30, 1.20);
								glPrint("%d", (int)p->wartosc);
							}
						}
						break;
					case PRZ_BECZKA:
						//gluCylinder(Qcyl,promien1,promien2,wysokosc,10,20);
						if (p->do_wziecia)
						{
							GLfloat Surface[] = { 0.50f, 0.45f, 0.0f, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
							if (liczba_pix_wid < (int)3.0 / (szczeg + 0.001))        // przy ma³ych widocznych rozmiarach rysowanie punktu
							{
								glPointSize((int)liczba_pix_wid*0.3);
								glBegin(GL_POINTS);
								glVertex3f(0, 0, 0);
								glEnd();
							}
							else
							{
								int liczba = (int)(liczba_pix_wid*szczeg / 3);
								if (liczba < 5) liczba = 5;
								if (liczba > 15) liczba = 15;
								glRotatef(90, 1, 0, 0);

								float grubosc = 2 * p->srednica;
								gluDisk(Qdisk, 0, p->srednica, liczba, liczba);
								gluCylinder(Qcyl, p->srednica, p->srednica, grubosc, liczba, liczba * 2);
							}
							if (liczba_pix_wid > 5)
							{
								glRasterPos2f(0.30, 1.20);
								glPrint("%d", (int)p->wartosc);
							}
						}
						break;
					case PRZ_DRZEWO:
					{
						float wariancja_ksztaltu = 0.2;     // poprawka wielkoœci i koloru zale¿¹ od po³o¿enia (wsp. x,z)
						float wariancja_koloru = 0.2;        // wszêdzie wiêc (u ró¿nych klientów) drzewa bêd¹ wygl¹da³y tak samo
						float poprawka_kol1 = (2 * fabs(p->wPol.x / 600 - floor(p->wPol.x /600)) - 1)*wariancja_koloru,
							poprawka_kol2 = (2 * fabs(p->wPol.z /600 - floor(p->wPol.z /600)) - 1)*wariancja_koloru;
						switch (p->podtyp)
						{
						case DRZ_TOPOLA:
						{
							GLfloat Surface[] = { 0.5f, 0.5f, 0.0f, 1.0f };
							GLfloat KolorPnia[] = { 0.4 + poprawka_kol1 / 3, 0.6 + poprawka_kol2 / 3, 0.0f, 1.0f };
							GLfloat KolorLisci[] = { 2*poprawka_kol1, 0.9 + 2*poprawka_kol2, 0.0f, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorPnia);
							if (liczba_pix_wid < (int)10.0 / (szczeg + 0.001))
							{
								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								glPointSize((int)liczba_pix_wid*0.3);
								glBegin(GL_POINTS);
								glVertex3f(0, 0, 0);
								glEnd();
							}
							else
							{
								int liczba = (int)(liczba_pix_wid*szczeg / 20);
								if (liczba < 3) liczba = 3;
								if (liczba > 10) liczba = 10;

								glPushMatrix();
								glRotatef(90, 1, 0, 0);
								float poprawka_ksz1 = (2 * fabs(p->wPol.x / 35 - floor(p->wPol.x / 35)) - 1)*wariancja_ksztaltu;
								float poprawka_ksz2 = (2 * fabs(p->wPol.z / 15 - floor(p->wPol.z / 15)) - 1)*wariancja_ksztaltu;
								float wysokosc = p->wartosc;
								float sredn = p->param_f[0] * p->srednica*(1 + poprawka_ksz1);
								gluCylinder(Qcyl, sredn / 3, sredn, wysokosc*1.1, liczba, liczba * 2);
								glPopMatrix();

								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								//glTranslatef(0,wysokosc,0);
								
								glScalef(sredn, 3 * (1 + poprawka_ksz2), sredn);
								
								gluSphere(Qsph, 3.0, liczba * 2, liczba * 2);
							}
							break;
						}
						case DRZ_SWIERK:
						{
							GLfloat KolorPnia[] = { 0.65f, 0.3f, 0.0f, 1.0f };
							GLfloat KolorLisci[] = { 0.0f, 0.70 + poprawka_kol1, 0.2 + poprawka_kol2, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorPnia);
							if (liczba_pix_wid < (int)10.0 / (szczeg + 0.001))
							{
								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								glPointSize((int)liczba_pix_wid*0.3);
								glBegin(GL_POINTS);
								glVertex3f(0, 0, 0);
								glEnd();

							}
							else
							{
								int liczba = (int)(liczba_pix_wid*szczeg / 20);
								if (liczba < 3) liczba = 3;
								if (liczba > 10) liczba = 10;

								glPushMatrix();
								glRotatef(90, 1, 0, 0);
								float wysokosc = p->wartosc;
								float promien = p->param_f[0] * p->srednica / 2;
								gluCylinder(Qcyl, promien/3, promien * 2, wysokosc*1.1, liczba, liczba * 2);

								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								glTranslatef(0, 0, wysokosc * 2 / 4);
								gluCylinder(Qcyl, promien * 2, promien * 8, wysokosc / 4, liczba, liczba * 2);
								glTranslatef(0, 0, -wysokosc * 1 / 4);
								gluCylinder(Qcyl, promien * 2, promien * 6, wysokosc / 4, liczba, liczba * 2);
								glTranslatef(0, 0, -wysokosc * 1 / 3);
								gluCylinder(Qcyl, 0, promien * 4, wysokosc / 3, liczba, liczba * 2);

								glPopMatrix();
							}
							break;
						}
						case DRZ_BAOBAB:
						{
							GLfloat KolorPnia[] = { 0.5f, 0.9f, 0.2f, 1.0f };
							GLfloat KolorLisci[] = { 0.0f, 0.9 + poprawka_kol2, 0.2 + poprawka_kol1, 1.0f };

							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorPnia);

							if (liczba_pix_wid < (int)10.0 / (szczeg + 0.001))
							{
								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								glPointSize((int)liczba_pix_wid*0.3);
								glBegin(GL_POINTS);
								glVertex3f(0, 0, 0);
								glEnd();
							}
							else
							{
								int liczba = (int)(liczba_pix_wid*szczeg / 20);
								if (liczba < 3) liczba = 3;
								if (liczba > 10) liczba = 10;
								glPushMatrix();
								glRotatef(90, 1, 0, 0);
								float wysokosc = p->wartosc;
								float sredn = p->param_f[0] * p->srednica;
								gluCylinder(Qcyl, p->srednica / 2, sredn, wysokosc*1.1, liczba, liczba * 2);
								glPopMatrix();

								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								glPushMatrix();
								float s = 2 + wysokosc / 20;
								glScalef(s, s / 2, s);
								gluSphere(Qsph, 3, liczba * 2, liczba * 2);
								glPopMatrix();
								glTranslatef(0, -s / 1.5, 0);
								glScalef(s*2.2, s / 2, s*2.2);
								gluSphere(Qsph, 3, liczba, liczba);
							}
							break;
						}
						case DRZ_FANTAZJA:
						{
							GLfloat KolorPnia[] = { 0.65f, 0.3f, 0.0f, 1.0f };                    // kolor pnia i ga³êzi
							GLfloat KolorLisci[] = { 0.5f, 0.65 + poprawka_kol1, 0.2f, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorPnia);

							if (liczba_pix_wid < (int)10.0 / (szczeg + 0.001))
							{
								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								glPointSize((int)liczba_pix_wid*0.3);
								glBegin(GL_POINTS);
								glVertex3f(0, 0, 0);
								glEnd();
							}
							else
							{
								int liczba = (int)(liczba_pix_wid*szczeg / 20);
								if (liczba < 3) liczba = 3;
								if (liczba > 10) liczba = 10;
								glPushMatrix();
								glRotatef(90, 1, 0, 0);
								float wysokosc = p->wartosc, promien_podst = p->param_f[0] * p->srednica / 2,
									promien_wierzch = p->param_f[0] * p->srednica / 5;
								gluCylinder(Qcyl, promien_wierzch * 2, promien_podst * 2, wysokosc*1.1, liczba, liczba * 2);  // rysowanie pnia
								glPopMatrix();
								glPushMatrix();
								// kolor liœci
								if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
								else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);
								//glTranslatef(0,wysokosc,0);
								float s = 0.5 + wysokosc / 15;    // œrednica listowia      
								glScalef(s, s, s);
								gluSphere(Qsph, 3, liczba * 2, liczba * 2);        // rysowanie pêku liœci na wierzcho³ku
								glPopMatrix();

								int liczba_gal = (int)(wysokosc / 3.5) + (int)(wysokosc * 10) % 3;             // liczba ga³êzi
								float prop_podz = 0.25 + (float)((int)(wysokosc * 13) % 10) / 100;           // proporcja wysokoœci na której znajduje siê ga³¹Ÿ od poprzedniej galezi lub gruntu
								float prop = 1, wys = 0;

								for (int j = 0; j < liczba_gal; j++)
								{
									glPushMatrix();
									prop *= prop_podz;
									wys += (wysokosc - wys)*prop_podz;              // wysokoœæ na której znajduje siê i-ta ga³¹Ÿ od poziomu gruntu

									float kat = 3.14 * 2 * (float)((int)(wys * 107) % 10) / 10;    // kat w/g pnia drzewa
									float sredn = (promien_podst * 2 - wys / wysokosc*(promien_podst * 2 - promien_wierzch * 2)) / 2;
									float dlug = sredn * 2 + sqrt(wysokosc - wys);
									if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
									else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorPnia);

									glTranslatef(0, wys - wysokosc, 0);
									glRotatef(kat * 180 / 3.14, 0, 1, 0);
									gluCylinder(Qcyl, sredn, sredn / 2, dlug, liczba, liczba);
									if (p->czy_zazn) glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
									else glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, KolorLisci);

									float ss = s*sredn / promien_podst * 2 * 1.5;
									glTranslatef(0, 0, dlug + ss * 2);
									glScalef(ss, ss, ss);
									gluSphere(Qsph, 3, liczba, liczba);        // rysowanie pêku liœci na wierzcho³ku

									glPopMatrix();
								}
							}

							break;
						} // case Fantazja

						} // switch po typach drzew
						break;
					} // drzewo
					case PRZ_PUNKT:
					{
						//gluCylinder(Qcyl,promien1,promien2,wysokosc,10,20);
						if (tryb_edycji_terrainu)
						{
							GLfloat Surface[] = { 2.0f, 0.0f, 0.0f, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
							//glRotatef(90, 1, 0, 0);
							glTranslatef(0, p->wartosc, 0);
							glPointSize(3);
							glBegin(GL_POINTS);
							glVertex3f(0, 0, 0);
							glEnd();
							glTranslatef(0, -p->wartosc, 0);
							if (liczba_pix_wid > 5)
							{
								glRasterPos2f(0.30, 1.20);
								glPrint("punkt");
							}
						}
						break;
					}
					case PRZ_KRAWEDZ:
					{
						//gluCylinder(Qcyl,promien1,promien2,wysokosc,10,20);
						if (tryb_edycji_terrainu)
						{
							GLfloat Surface[] = { 2.0f, 0.0f, 0.0f, 1.0f };
							if (p->czy_zazn)
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
							else
								glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
							//glRotatef(90, 1, 0, 0);
							long nr_punktu1 = p->param_i[0], nr_punktu2 = p->param_i[1];
							Wektor3 pol = (this->p[nr_punktu1].wPol + this->p[nr_punktu2].wPol) / 2;
							glTranslatef(-p->wPol.x, -p->wPol.y, -p->wPol.z);
							//glTranslatef(-pol.x, -pol.y, -pol.z);
							glLineWidth(2);
							
							glBegin(GL_LINES);					
							glVertex3f(this->p[nr_punktu1].wPol.x, this->p[nr_punktu1].wPol.y + this->p[nr_punktu1].wartosc, this->p[nr_punktu1].wPol.z);
							glVertex3f(this->p[nr_punktu2].wPol.x, this->p[nr_punktu2].wPol.y + this->p[nr_punktu2].wartosc, this->p[nr_punktu2].wPol.z);
							glEnd();

							//glTranslatef(p->wPol.x, p->wPol.y, p->wPol.z);
							glTranslatef(pol.x, pol.y, pol.z);
							if (liczba_pix_wid > 5)
							{
								glRasterPos2f(0.30, 1.20);
								glPrint("krawedz");
							}
							glTranslatef(-pol.x, -pol.y, -pol.z);
						}
						break;
					}
					case PRZ_MUR:
					{
						GLfloat Surface[] = { 0.8f, 0.8f, 0.4f, 1.0f };
						if (p->czy_zazn)
							glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, GreySurface);
						else
							glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, Surface);
						//glRotatef(90, 1, 0, 0);
						long nr_punktu1 = p->param_i[0], nr_punktu2 = p->param_i[1];
						Wektor3 pol1 = this->p[nr_punktu1].wPol, pol2 = this->p[nr_punktu2].wPol, pion = Wektor3(0, 1, 0);
						pol1.y += this->p[nr_punktu1].wartosc;
						pol2.y += this->p[nr_punktu2].wartosc;

						Wektor3 pol = (pol1 + pol2) / 2;
						Wektor3 wprzod = (pol2 - pol1).znorm();             // kierunek od punktu 1 do punktu 2
						Wektor3 wlewo = pion*wprzod;                        // kierunek w lewo, gdyby patrzeæ w kierunku wprzod
						float grubosc = p->param_f[0];     // gruboœæ muru
						float wysokosc = p->wartosc;       // wysokoœæ muru (tzn. gruboœæ w pionie) niezale¿nie od pionowego umiejscowienia
							
						//glTranslatef(-pol.x, -pol.y, -pol.z);							
						glTranslatef(-p->wPol.x, -p->wPol.y, -p->wPol.z);

			            // punkty kluczowe:
						Wektor3 A = pol1 - wlewo*grubosc / 2 + pion*wysokosc/2;
						Wektor3 B = A + wlewo*grubosc;
						Wektor3 C = B - pion*wysokosc;
						Wektor3 D = A - pion*wysokosc;
						Wektor3 Ap = A + pol2 - pol1, Bp = B + pol2 - pol1,
							Cp = C + pol2 - pol1, Dp = D + pol2 - pol1;

						glBegin(GL_QUADS);
						// sciana ABCD:
						Wektor3 N = pion*wlewo;
						glNormal3f(N.x, N.y, N.z);
						glVertex3f(A.x, A.y, A.z);
						glVertex3f(B.x, B.y, B.z);
						glVertex3f(C.x, C.y, C.z);
						glVertex3f(D.x, D.y, D.z);

						glNormal3f(-N.x, -N.y, -N.z);
						glVertex3f(Ap.x, Ap.y, Ap.z);
						glVertex3f(Dp.x, Dp.y, Dp.z);
						glVertex3f(Cp.x, Cp.y, Cp.z);
						glVertex3f(Bp.x, Bp.y, Bp.z);

						glNormal3f(-wlewo.x, -wlewo.y, -wlewo.z);
						glVertex3f(A.x, A.y, A.z);
						glVertex3f(D.x, D.y, D.z);
						glVertex3f(Dp.x, Dp.y, Dp.z);
						glVertex3f(Ap.x, Ap.y, Ap.z);

						glNormal3f(wlewo.x, wlewo.y, wlewo.z);
						glVertex3f(B.x, B.y, B.z);
						glVertex3f(Bp.x, Bp.y, Bp.z);
						glVertex3f(Cp.x, Cp.y, Cp.z);
						glVertex3f(C.x, C.y, C.z);

						N = wprzod*wlewo;
						glNormal3f(N.x, N.y, N.z);
						glVertex3f(A.x, A.y, A.z);
						glVertex3f(Ap.x, Ap.y, Ap.z);
						glVertex3f(Bp.x, Bp.y, Bp.z);
						glVertex3f(B.x, B.y, B.z);

						glNormal3f(-N.x, -N.y, -N.z);
						glVertex3f(D.x, D.y, D.z);
						glVertex3f(C.x, C.y, C.z);
						glVertex3f(Cp.x, Cp.y, Cp.z);
						glVertex3f(Dp.x, Dp.y, Dp.z);

						glEnd();
						//glTranslatef(p->wPol.x, p->wPol.y, p->wPol.z);

						if (tryb_edycji_terrainu)
						{
							glTranslatef(pol.x, pol.y+wysokosc/2, pol.z);
							if (liczba_pix_wid > 5)
							{
								glRasterPos2f(0.30, 1.20);
								glPrint("mur");
							}
							glTranslatef(-pol.x, -pol.y-wysokosc/2, -pol.z);
						}
						//fprintf(f, "Narysowano mur pomiedzy punktami %d (%f,%f,%f) a %d (%f,%f,%f)\n", nr_punktu1, pol1.x, pol1.y, pol1.z,
						//	nr_punktu2, pol2.x, pol2.y, pol2.z);
						break;
					}
					} // switch  po typach przedmiotów

					glPopMatrix();
				} // jeœli przedmiot nie zosta³ ju¿ wyœwietlony w tej funkcji
			} // po przedmiotach z sektora
		
		} // po sektorach
	gluDeleteQuadric(Qcyl); gluDeleteQuadric(Qdisk); gluDeleteQuadric(Qsph);
	//glCallList(Floor);     
	this->liczba_wyswietlen++;
}

