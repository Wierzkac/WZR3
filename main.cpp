/****************************************************
	Wirtualne zespoly robocze - przykladowy projekt w C++
	Do zadań dotyczących współpracy, ekstrapolacji i
	autonomicznych obiektów
	****************************************************/

#include <windows.h>
#include <math.h>
#include <time.h>

#include <gl\gl.h>
#include <gl\glu.h>
#include <iterator> 
#include <map>
using namespace std;

#include "objects.h"

#include "graphics.h"
#include "net.h"


bool if_different_skills = true;          // czy zró¿nicowanie umiejêtnoœci (dla ka¿dego pojazdu losowane s¹ umiejêtnoœci
// zbierania gotówki i paliwa)
bool if_autonomous_control = false;       // sterowanie autonomiczne pojazdem


FILE *f = fopen("wzrlog.txt", "w");     // plik do zapisu informacji testowych

MovableObject *my_vehicle;             // obiekt przypisany do tej aplikacji
//MovableObject *MojeObiekty[1000];        // obiekty przypisane do tej aplikacji max. 1000 (mogą być zarówno sterowalne, jak i poza kontrolą lub wrogie)
//int iLiczbaWlasnychOb = 5;
//int iAkt = 0;                            // numer obiektu spośród przypisanych do tej aplikacji, który jest w danym momencie aktywny - sterowalny


Terrain terrain;
map<int, MovableObject*> network_vehicles;

float fDt;                          // sredni czas pomiedzy dwoma kolejnymi cyklami symulacji i wyswietlania
long czas_cyklu_WS, licznik_sym;     // zmienne pomocnicze potrzebne do obliczania fDt
float sr_czestosc;                  // srednia czestosc wysylania ramek w [ramkach/s] 
long czas_start = clock();          // czas od poczatku dzialania aplikacji  
long czas_istnienia_grupy = clock();    // czas od pocz¹tku istnienia grupy roboczej (czas od uruchom. pierwszej aplikacji)      

multicast_net *multi_reciv;         // wsk do obiektu zajmujacego sie odbiorem komunikatow
multicast_net *multi_send;          //   -||-  wysylaniem komunikatow

HANDLE threadReciv;                 // uchwyt w¹tku odbioru komunikatów
extern HWND okno;
CRITICAL_SECTION m_cs;               // do synchronizacji wątków

bool SHIFTwcisniety = 0;
bool CTRLwcisniety = 0;
bool ALTwcisniety = 0;
bool Lwcisniety = 0;
//bool rejestracja_uczestnikow = true;   // rejestracja trwa do momentu wziêcia przedmiotu przez któregokolwiek uczestnika,
// w przeciwnym razie trzeba by przesy³aæ ca³y stan œrodowiska nowicjuszowi

// Parametry widoku:
extern ParametryWidoku par_wid;

bool sterowanie_myszkowe = 0;                   // sterowanie pojazdem za pomoc¹ myszki
int kursor_x, kursor_y;                         // polo¿enie kursora myszki w chwili w³¹czenia sterowania
//char napis1[300], napis2[300];                  // napisy wyœwietlane w trybie graficznym 
long nr_miejsca_przedm = terrain.liczba_przedmiotow;  // numer miejsca, w którym mo¿na umieœciæ przedmiot

int opoznienia = 0;


extern float WyslaniePrzekazu(int ID_adresata, int typ_przekazu, float wartosc_przekazu);

enum typy_ramek {
	STAN_OBIEKTU, WZIECIE_PRZEDMIOTU, ODNOWIENIE_SIE_PRZEDMIOTU, KOLIZJA, PRZEKAZ
};

enum typy_przekazu { PIENIADZE, PALIWO };

struct Ramka
{
	int iID;
	int typ_ramki;
	StanObiektu stan;
	
	long czas_wyslania;
	int iID_adresata;      // nr ID adresata wiadomoœci (pozostali uczestnicy powinni wiadomoœæ zignorowaæ)

	int nr_przedmiotu;     // nr przedmiotu, który zosta³ wziêty lub odzyskany
	Wektor3 wdV_kolid;     // wektor prêdkoœci wyjœciowej po kolizji (uczestnik o wskazanym adresie powinien 
	// przyj¹æ t¹ prêdkoœæ)  

	int typ_przekazu;        // gotówka, paliwo
	float wartosc_przekazu;  // iloœæ gotówki lub paliwa 
	int nr_druzyny;

	long czas_istnienia;        // czas jaki uplyn¹³ od uruchomienia programu
};


//******************************************
// Funkcja obs³ugi w¹tku odbioru komunikatów 
DWORD WINAPI WatekOdbioru(void *ptr)
{
	multicast_net *pmt_net = (multicast_net*)ptr;  // wskaŸnik do obiektu klasy multicast_net
	int rozmiar;                                 // liczba bajtów ramki otrzymanej z sieci
	Ramka ramka;
	StanObiektu stan;

	while (1)
	{
		rozmiar = pmt_net->reciv((char*)&ramka, sizeof(Ramka));   // oczekiwanie na nadejœcie ramki 
		// Lock the Critical section
		EnterCriticalSection(&m_cs);               // wejście na ścieżkę krytyczną - by inne wątki (np. główny) nie współdzielił 

		switch (ramka.typ_ramki)
		{
		case STAN_OBIEKTU:           // podstawowy typ ramki informuj¹cej o stanie obiektu              
		{
			stan = ramka.stan;
			//fprintf(f,"odebrano stan iID = %d, ID dla mojego obiektu = %d\n",stan.iID,my_vehicle->iID);
			if ((ramka.iID != my_vehicle->iID))          // jeœli to nie mój w³asny obiekt
			{

				if ((network_vehicles.size() == 0) || (network_vehicles[ramka.iID] == NULL))         // nie ma jeszcze takiego obiektu w tablicy -> trzeba go stworzyæ
				{
					MovableObject *ob = new MovableObject(&terrain);
					ob->iID = ramka.iID;
					network_vehicles[ramka.iID] = ob;
					if (ramka.czas_istnienia > czas_istnienia_grupy) czas_istnienia_grupy = ramka.czas_istnienia;
					ob->ZmienStan(stan);   // aktualizacja stanu obiektu obcego 
					terrain.WstawObiektWsektory(ob);
					// wys³anie nowemu uczestnikowi informacji o wszystkich wziêtych przedmiotach:
					for (long i = 0; i < terrain.liczba_przedmiotow; i++)
						if ((terrain.p[i].do_wziecia == 0) && (terrain.p[i].czy_ja_wzialem))
						{
							Ramka ramka;
							ramka.typ_ramki = WZIECIE_PRZEDMIOTU;
							ramka.nr_przedmiotu = i;
							ramka.stan = my_vehicle->Stan();
							ramka.iID = my_vehicle->iID;
							int iRozmiar = multi_send->send((char*)&ramka, sizeof(Ramka));
						}

				}
				else if ((network_vehicles.size() > 0) && (network_vehicles[ramka.iID] != NULL))
				{
					terrain.UsunObiektZsektorow(network_vehicles[ramka.iID]);
					network_vehicles[ramka.iID]->ZmienStan(stan);   // aktualizacja stanu obiektu obcego 	
					terrain.WstawObiektWsektory(network_vehicles[ramka.iID]);
				}
				
			}
			break;
		}
		case WZIECIE_PRZEDMIOTU:            // ramka informuj¹ca, ¿e ktoœ wzi¹³ przedmiot o podanym numerze
		{
			stan = ramka.stan;
			if ((ramka.nr_przedmiotu < terrain.liczba_przedmiotow) && (ramka.iID != my_vehicle->iID))
			{
				terrain.p[ramka.nr_przedmiotu].do_wziecia = 0;
				terrain.p[ramka.nr_przedmiotu].czy_ja_wzialem = 0;
			}
			break;
		}
		case ODNOWIENIE_SIE_PRZEDMIOTU:       // ramka informujaca, ¿e przedmiot wczeœniej wziêty pojawi³ siê znowu w tym samym miejscu
		{
			if (ramka.nr_przedmiotu < terrain.liczba_przedmiotow)
				terrain.p[ramka.nr_przedmiotu].do_wziecia = 1;
			break;
		}
		case KOLIZJA:                       // ramka informuj¹ca o tym, ¿e obiekt uleg³ kolizji
		{
			if (ramka.iID_adresata == my_vehicle->iID)  // ID pojazdu, który uczestniczy³ w kolizji zgadza siê z moim ID 
			{
				my_vehicle->wdV_kolid = ramka.wdV_kolid; // przepisuje poprawkê w³asnej prêdkoœci
				my_vehicle->iID_kolid = my_vehicle->iID; // ustawiam nr. kolidujacego jako w³asny na znak, ¿e powinienem poprawiæ prêdkoœæ
			}
			break;
		}
		case PRZEKAZ:                       // ramka informuj¹ca o przelewie pieniê¿nym lub przekazaniu towaru    
		{
			if (ramka.iID_adresata == my_vehicle->iID)  // ID pojazdu, ktory otrzymal przelew zgadza siê z moim ID 
			{
				if (ramka.typ_przekazu == PIENIADZE)
					my_vehicle->pieniadze += ramka.wartosc_przekazu;
				else if (ramka.typ_przekazu == PALIWO)
					my_vehicle->ilosc_paliwa += ramka.wartosc_przekazu;
				sprintf(par_wid.napis2, "");
				// nale¿a³oby jeszcze przelew potwierdziæ (w UDP ramki mog¹ byæ gubione!)
			}
			break;
		}
		
		} // switch po typach ramek
		//Release the Critical section
		LeaveCriticalSection(&m_cs);               // wyjście ze ścieżki krytycznej
	}  // while(1)
	return 1;
}

// *****************************************************************
// ****    Wszystko co trzeba zrobiæ podczas uruchamiania aplikacji
// ****    poza grafik¹   
void PoczatekInterakcji()
{
	DWORD dwThreadId;

	my_vehicle = new MovableObject(&terrain);    // tworzenie wlasnego obiektu
	if (if_different_skills == false)
		my_vehicle->umiejetn_sadzenia = my_vehicle->umiejetn_zb_monet = my_vehicle->umiejetn_zb_paliwa = 1.0;


	czas_cyklu_WS = clock();             // pomiar aktualnego czasu

	// obiekty sieciowe typu multicast (z podaniem adresu WZR oraz numeru portu)
	multi_reciv = new multicast_net("224.10.12.190", 10001);      // obiekt do odbioru ramek sieciowych
	multi_send = new multicast_net("224.10.12.190", 10001);       // obiekt do wysy³ania ramek

	// uruchomienie watku obslugujacego odbior komunikatow
	threadReciv = CreateThread(
		NULL,                        // no security attributes
		0,                           // use default stack size
		WatekOdbioru,                // thread function
		(void *)multi_reciv,               // argument to thread function
		0,                           // use default creation flags
		&dwThreadId);                // returns the thread identifier
		
}


// *****************************************************************
// ****    Wszystko co trzeba zrobiæ w ka¿dym cyklu dzia³ania 
// ****    aplikacji poza grafik¹ 
void Cykl_WS()
{
	licznik_sym++;

	// obliczenie œredniego czasu pomiêdzy dwoma kolejnnymi symulacjami po to, by zachowaæ  fizycznych 
	if (licznik_sym % 50 == 0)          // jeœli licznik cykli przekroczy³ pewn¹ wartoœæ, to
	{                                   // nale¿y na nowo obliczyæ œredni czas cyklu fDt
		char text[200];
		long czas_pop = czas_cyklu_WS;
		czas_cyklu_WS = clock();
		float fFps = (50 * CLOCKS_PER_SEC) / (float)(czas_cyklu_WS - czas_pop);
		if (fFps != 0) fDt = 1.0 / fFps; else fDt = 1;

		sprintf(par_wid.napis1, " %0.0f_fps, paliwo = %0.2f, gotowka = %d,", fFps, my_vehicle->ilosc_paliwa, my_vehicle->pieniadze);
		if (licznik_sym % 500 == 0) sprintf(par_wid.napis2, "");
	}


	terrain.UsunObiektZsektorow(my_vehicle);
	my_vehicle->Symulacja(fDt);                    // symulacja w³asnego obiektu
	terrain.WstawObiektWsektory(my_vehicle);


	if ((my_vehicle->iID_kolid > -1) &&             // wykryto kolizjê - wysy³am specjaln¹ ramkê, by poinformowaæ o tym drugiego uczestnika
		(my_vehicle->iID_kolid != my_vehicle->iID)) // oczywiœcie wtedy, gdy nie chodzi o mój pojazd
	{
		Ramka ramka;
		ramka.typ_ramki = KOLIZJA;
		ramka.iID_adresata = my_vehicle->iID_kolid;
		ramka.wdV_kolid = my_vehicle->wdV_kolid;
		ramka.iID = my_vehicle->iID;
		int iRozmiar = multi_send->send((char*)&ramka, sizeof(Ramka));

		char text[128];
		sprintf(par_wid.napis2, "Kolizja_z_obiektem_o_ID = %d", my_vehicle->iID_kolid);
		//SetWindowText(okno,text);

		my_vehicle->iID_kolid = -1;
	}

	// wyslanie komunikatu o stanie obiektu przypisanego do aplikacji (my_vehicle):    

	Ramka ramka;
	ramka.typ_ramki = STAN_OBIEKTU;
	ramka.stan = my_vehicle->Stan();         // stan w³asnego obiektu 
	ramka.iID = my_vehicle->iID;
	ramka.czas_istnienia = clock() - czas_start;
	int iRozmiar = multi_send->send((char*)&ramka, sizeof(Ramka));



	// wziêcie przedmiotu -> wysy³anie ramki 
	if (my_vehicle->nr_wzietego_przedm > -1)
	{
		Ramka ramka;
		ramka.typ_ramki = WZIECIE_PRZEDMIOTU;
		ramka.nr_przedmiotu = my_vehicle->nr_wzietego_przedm;
		ramka.stan = my_vehicle->Stan();
		ramka.iID = my_vehicle->iID;
		int iRozmiar = multi_send->send((char*)&ramka, sizeof(Ramka));

		sprintf(par_wid.napis2, "Wziecie_przedmiotu_o_wartosci_ %f", my_vehicle->wartosc_wzieta);

		my_vehicle->nr_wzietego_przedm = -1;
		my_vehicle->wartosc_wzieta = 0;
		//rejestracja_uczestnikow = 0;     // koniec rejestracji nowych uczestników
	}

	// odnawianie siê przedmiotu -> wysy³anie ramki
	if (my_vehicle->nr_odnowionego_przedm > -1)
	{                             // jeœli min¹³ pewnien okres czasu przedmiot mo¿e zostaæ przywrócony
		Ramka ramka;
		ramka.typ_ramki = ODNOWIENIE_SIE_PRZEDMIOTU;
		ramka.nr_przedmiotu = my_vehicle->nr_odnowionego_przedm;
		ramka.iID = my_vehicle->iID;
		int iRozmiar = multi_send->send((char*)&ramka, sizeof(Ramka));


		my_vehicle->nr_odnowionego_przedm = -1;
	}


}

// *****************************************************************
// ****    Wszystko co trzeba zrobiæ podczas zamykania aplikacji
// ****    poza grafik¹ 
void ZakonczenieInterakcji()
{
	fprintf(f, "Koniec interakcji\n");
	fclose(f);
}

// Funkcja wysylajaca ramke z przekazem, zwraca zrealizowan¹ wartoœæ przekazu
float WyslaniePrzekazu(int ID_adresata, int typ_przekazu, float wartosc_przekazu)
{
	Ramka ramka;
	ramka.typ_ramki = PRZEKAZ;
	ramka.iID_adresata = ID_adresata;
	ramka.typ_przekazu = typ_przekazu;
	ramka.wartosc_przekazu = wartosc_przekazu;
	ramka.iID = my_vehicle->iID;

	// tutaj nale¿a³oby uzyskaæ potwierdzenie przekazu zanim sumy zostan¹ odjête
	if (typ_przekazu == PIENIADZE)
	{
		if (my_vehicle->pieniadze < wartosc_przekazu)
			ramka.wartosc_przekazu = my_vehicle->pieniadze;
		my_vehicle->pieniadze -= ramka.wartosc_przekazu;
		sprintf(par_wid.napis2, "Przelew_sumy_ %f _na_rzecz_ID_ %d", wartosc_przekazu, ID_adresata);
	}
	else if (typ_przekazu == PALIWO)
	{
		if (my_vehicle->ilosc_paliwa < wartosc_przekazu)
			ramka.wartosc_przekazu = my_vehicle->ilosc_paliwa;
		my_vehicle->ilosc_paliwa -= ramka.wartosc_przekazu;
		sprintf(par_wid.napis2, "Przekazanie_paliwa_w_iloœci_ %f _na_rzecz_ID_ %d", wartosc_przekazu, ID_adresata);
	}

	if (ramka.wartosc_przekazu > 0)
		int iRozmiar = multi_send->send((char*)&ramka, sizeof(Ramka));

	return ramka.wartosc_przekazu;
}





//deklaracja funkcji obslugi okna
LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);


HWND okno;                   // uchwyt do okna aplikacji
HDC g_context = NULL;        // uchwyt kontekstu graficznego

bool tryb_edycji_terrainu = 0;

//funkcja Main - dla Windows
int WINAPI WinMain(HINSTANCE hInstance,
	HINSTANCE hPrevInstance,
	LPSTR     lpCmdLine,
	int       nCmdShow)
{
	//Initilize the critical section:
	InitializeCriticalSection(&m_cs);

	MSG meldunek;		  //innymi slowy "komunikat"
	WNDCLASS nasza_klasa; //klasa głównego okna aplikacji

	static char nazwa_klasy[] = "Basic";

	//Definiujemy klase głównego okna aplikacji
	//Okreslamy tu wlasciwosci okna, szczegoly wygladu oraz
	//adres funkcji przetwarzajacej komunikaty
	nasza_klasa.style = CS_HREDRAW | CS_VREDRAW;
	nasza_klasa.lpfnWndProc = WndProc; //adres funkcji realizującej przetwarzanie meldunków 
	nasza_klasa.cbClsExtra = 0;
	nasza_klasa.cbWndExtra = 0;
	nasza_klasa.hInstance = hInstance; //identyfikator procesu przekazany przez MS Windows podczas uruchamiania programu
	nasza_klasa.hIcon = 0;
	nasza_klasa.hCursor = LoadCursor(0, IDC_ARROW);
	nasza_klasa.hbrBackground = (HBRUSH)GetStockObject(GRAY_BRUSH);
	nasza_klasa.lpszMenuName = "Menu";
	nasza_klasa.lpszClassName = nazwa_klasy;

	//teraz rejestrujemy klasę okna głównego
	RegisterClass(&nasza_klasa);

	/*tworzymy okno główne
	okno będzie miało zmienne rozmiary, listwę z tytułem, menu systemowym
	i przyciskami do zwijania do ikony i rozwijania na cały ekran, po utworzeniu
	będzie widoczne na ekranie */
	okno = CreateWindow(nazwa_klasy, "WZR 2018/19, temat 3, wersja d ", WS_OVERLAPPEDWINDOW | WS_VISIBLE | WS_CLIPCHILDREN | WS_CLIPSIBLINGS,
		100, 100, 1200, 800, NULL, NULL, hInstance, NULL);


	ShowWindow(okno, nCmdShow);

	//odswiezamy zawartosc okna
	UpdateWindow(okno);



	// GŁÓWNA PĘTLA PROGRAMU

	// pobranie komunikatu z kolejki jeśli funkcja PeekMessage zwraca wartość inną niż FALSE,
	// w przeciwnym wypadku symulacja wirtualnego świata wraz z wizualizacją
	ZeroMemory(&meldunek, sizeof(meldunek));
	while (meldunek.message != WM_QUIT)
	{
		if (PeekMessage(&meldunek, NULL, 0U, 0U, PM_REMOVE))
		{
			TranslateMessage(&meldunek);
			DispatchMessage(&meldunek);
		}
		else
		{
			Cykl_WS();    // Cykl wirtualnego świata
			InvalidateRect(okno, NULL, FALSE);
		}
	}

	return (int)meldunek.wParam;
}

// ************************************************************************
// ****    Obs³uga klawiszy s³u¿¹cych do sterowania obiektami lub
// ****    widokami 
void KlawiszologiaSterowania(UINT kod_meldunku, WPARAM wParam, LPARAM lParam)
{

	int LCONTROL = GetKeyState(VK_LCONTROL);
	int RCONTROL = GetKeyState(VK_RCONTROL);
	int LALT = GetKeyState(VK_LMENU);
	int RALT = GetKeyState(VK_RMENU);


	switch (kod_meldunku)
	{

	case WM_LBUTTONDOWN: //reakcja na lewy przycisk myszki
	{
		int x = LOWORD(lParam);
		int y = HIWORD(lParam);
		if (sterowanie_myszkowe)
			my_vehicle->F = my_vehicle->F_max;        // si³a pchaj¹ca do przodu

		break;
	}
	case WM_RBUTTONDOWN: //reakcja na prawy przycisk myszki
	{
		int x = LOWORD(lParam);
		int y = HIWORD(lParam);
		int LSHIFT = GetKeyState(VK_LSHIFT);   // sprawdzenie czy lewy Shift wciśnięty, jeśli tak, to LSHIFT == 1
		int RSHIFT = GetKeyState(VK_RSHIFT);

		if (sterowanie_myszkowe)
			my_vehicle->F = -my_vehicle->F_max / 2;        // si³a pchaj¹ca do tylu
		else if (wParam & MK_SHIFT)                    // odznaczanie wszystkich obiektów   
		{
			for (long i = 0; i < terrain.liczba_zazn_przedm; i++)
				terrain.p[terrain.zazn_przedm[i]].czy_zazn = 0;
			terrain.liczba_zazn_przedm = 0;
		}
		else                                          // zaznaczenie obiektów
		{
			RECT r;
			//GetWindowRect(okno,&r);
			GetClientRect(okno, &r);
			//Wektor3 w = WspolrzedneKursora3D(x, r.bottom - r.top - y);
			Wektor3 w = terrain.wspolrzedne_kursora3D_bez_paralaksy(x, r.bottom - r.top - y);


			//float promien = (w - punkt_klik).dlugosc();
			float odl_min = 1e10;
			long index_min = -1;
			bool czy_ob_ruch;
			for (map<int, MovableObject*>::iterator it = network_vehicles.begin(); it != network_vehicles.end(); ++it)
			{
				if (it->second)
				{
					MovableObject *ob = it->second;
					float xx, yy, zz;
					WspolrzedneEkranu(&xx, &yy, &zz, ob->wPol);
					yy = r.bottom - r.top - yy;
					float odl_kw = (xx - x)*(xx - x) + (yy - y)*(yy - y);
					if (odl_min > odl_kw)
					{
						odl_min = odl_kw;
						index_min = ob->iID;
						czy_ob_ruch = 1;
					}
				}
			}
		

			// trzeba to przerobić na wersję sektorową, gdyż przedmiotów może być dużo!
			// niestety nie jest to proste. 

			//Przedmiot **wsk_prz = NULL;
			//long liczba_prz_w_prom = terrain.Przedmioty_w_promieniu(&wsk_prz, w,100);

			for (long i = 0; i < terrain.liczba_przedmiotow; i++)
			{
				float xx, yy, zz;
				Wektor3 polozenie;
				if ((terrain.p[i].typ == PRZ_KRAWEDZ) || (terrain.p[i].typ == PRZ_MUR))
				{
					polozenie = (terrain.p[terrain.p[i].param_i[0]].wPol + terrain.p[terrain.p[i].param_i[1]].wPol) / 2;
				}
				else
					polozenie = terrain.p[i].wPol;
				WspolrzedneEkranu(&xx, &yy, &zz, polozenie);
				yy = r.bottom - r.top - yy;
				float odl_kw = (xx - x)*(xx - x) + (yy - y)*(yy - y);
				if (odl_min > odl_kw)
				{
					odl_min = odl_kw;
					index_min = i;
					czy_ob_ruch = 0;
				}
			}

			if (index_min > -1)
			{
				//fprintf(f,"zaznaczono przedmiot %d pol = (%f, %f, %f)\n",ind_min,terrain.p[ind_min].wPol.x,terrain.p[ind_min].wPol.y,terrain.p[ind_min].wPol.z);
				//terrain.p[ind_min].czy_zazn = 1 - terrain.p[ind_min].czy_zazn;
				if (czy_ob_ruch)
				{
					network_vehicles[index_min]->czy_zazn = 1 - network_vehicles[index_min]->czy_zazn;

					if (network_vehicles[index_min]->czy_zazn)
						sprintf(par_wid.napis2, "zaznaczono_ obiekt_ID_%d", network_vehicles[index_min]->iID);
				}
				else
				{
					terrain.ZaznaczOdznaczPrzedmiotLubGrupe(index_min);
				}
				//char lan[256];
				//sprintf(lan, "klikniêto w przedmiot %d pol = (%f, %f, %f)\n",ind_min,terrain.p[ind_min].wPol.x,terrain.p[ind_min].wPol.y,terrain.p[ind_min].wPol.z);
				//SetWindowText(okno,lan);
			}
			Wektor3 punkt_klik = WspolrzedneKursora3D(x, r.bottom - r.top - y);

		}

		break;
	}
	case WM_MBUTTONDOWN: //reakcja na œrodkowy przycisk myszki : uaktywnienie/dezaktywacja sterwania myszkowego
	{
		sterowanie_myszkowe = 1 - sterowanie_myszkowe;
		kursor_x = LOWORD(lParam);
		kursor_y = HIWORD(lParam);
		break;
	}
	case WM_LBUTTONUP: //reakcja na puszczenie lewego przycisku myszki
	{
		if (sterowanie_myszkowe)
			my_vehicle->F = 0.0;        // si³a pchaj¹ca do przodu
		break;
	}
	case WM_RBUTTONUP: //reakcja na puszczenie lewy przycisk myszki
	{
		if (sterowanie_myszkowe)
			my_vehicle->F = 0.0;        // si³a pchaj¹ca do przodu
		break;
	}
	case WM_MOUSEMOVE:
	{
		int x = LOWORD(lParam);
		int y = HIWORD(lParam);
		if (sterowanie_myszkowe)
		{
			float kat_skretu = (float)(kursor_x - x) / 20;
			if (kat_skretu > 45) kat_skretu = 45;
			if (kat_skretu < -45) kat_skretu = -45;
			my_vehicle->alfa = PI*kat_skretu / 180;
		}
		break;
	}
	case WM_MOUSEWHEEL:     // ruch kó³kiem myszy -> przybli¿anie, oddalanie widoku
	{
		int zDelta = GET_WHEEL_DELTA_WPARAM(wParam);  // dodatni do przodu, ujemny do ty³u
		//fprintf(f,"zDelta = %d\n",zDelta);          // zwykle +-120, jak siê bardzo szybko zakrêci to czasmi wyjdzie +-240
		if (zDelta > 0){
			if (par_wid.oddalenie > 0.5) par_wid.oddalenie /= 1.2;
			else par_wid.oddalenie = 0;
		}
		else {
			if (par_wid.oddalenie > 0) par_wid.oddalenie *= 1.2;
			else par_wid.oddalenie = 0.5;
		}

		break;
	}
	case WM_KEYDOWN:
	{

		switch (LOWORD(wParam))
		{
		case VK_SHIFT:
		{
			SHIFTwcisniety = 1;
			break;
		}
		case VK_CONTROL:
		{
			CTRLwcisniety = 1;
			break;
		}
		case VK_MENU:
		{
			ALTwcisniety = 1;
			break;
		}

		case VK_SPACE:
		{
			my_vehicle->ham = 1.0;       // stopieñ hamowania (reszta zale¿y od si³y docisku i wsp. tarcia)
			break;                       // 1.0 to maksymalny stopieñ (np. zablokowanie kó³)
		}
		case VK_UP:
		{
			if (CTRLwcisniety && par_wid.widok_z_gory)
				par_wid.przes_w_dol += par_wid.oddalenie / 2;       // przesunięcie widoku z kamery w górę
			else
				my_vehicle->F = my_vehicle->F_max;        // si³a pchaj¹ca do przodu
			break;
		}
		case VK_DOWN:
		{
			if (CTRLwcisniety && par_wid.widok_z_gory)
				par_wid.przes_w_dol -= par_wid.oddalenie / 2;       // przesunięcie widoku z kamery w dół 
			else
				my_vehicle->F = -my_vehicle->F_max / 2;        // sila pchajaca do tylu
			break;
		}
		case VK_LEFT:
		{
			if (CTRLwcisniety && par_wid.widok_z_gory)
				par_wid.przes_w_prawo += par_wid.oddalenie / 2;       // przesunięcie widoku z kamery w lewo
			else
			{                           // skręt pojazdu w lewo
				if (wParam & SHIFTwcisniety) my_vehicle->alfa = PI * 35 / 180;
				else
					my_vehicle->alfa = PI * 15 / 180;
			}
			break;
		}
		case VK_RIGHT:
		{
			if (CTRLwcisniety && par_wid.widok_z_gory)
			{
				par_wid.przes_w_prawo -= par_wid.oddalenie / 2;       // przesunięcie widoku z kamery w prawo
			}
			else
			{
				if (wParam & SHIFTwcisniety) my_vehicle->alfa = -PI * 35 / 180;
				else
					my_vehicle->alfa = -PI * 15 / 180;
			}
			break;
		}
		case VK_HOME:
		{
			if (CTRLwcisniety && par_wid.widok_z_gory)
				par_wid.przes_w_prawo = par_wid.przes_w_dol = 0;

			break;
		}
		case 'W':   // przybli¿enie widoku
		{
			//pocz_pol_kamery = pocz_pol_kamery - pocz_kierunek_kamery*0.3;
			if (par_wid.oddalenie > 0.5)par_wid.oddalenie /= 1.2;
			else par_wid.oddalenie = 0;
			break;
		}
		case 'S':   // oddalenie widoku
		{
			//pocz_pol_kamery = pocz_pol_kamery + pocz_kierunek_kamery*0.3; 
			if (par_wid.oddalenie > 0) par_wid.oddalenie *= 1.2;
			else par_wid.oddalenie = 0.5;
			break;
		}
		case 'Q':   // widok z góry
		{
			par_wid.widok_z_gory = 1 - par_wid.widok_z_gory;
			if (par_wid.widok_z_gory)
				SetWindowText(okno, "Włączono widok z góry!");
			else
				SetWindowText(okno, "Wyłączono widok z góry.");
			break;
		}
		case 'E':   // obrót kamery ku górze (wzglêdem lokalnej osi z)
		{
			par_wid.kat_kam_z += PI * 5 / 180;
			break;
		}
		case 'D':   // obrót kamery ku do³owi (wzglêdem lokalnej osi z)
		{
			par_wid.kat_kam_z -= PI * 5 / 180;
			break;
		}
		case 'A':   // w³¹czanie, wy³¹czanie trybu œledzenia obiektu
		{
			par_wid.sledzenie = 1 - par_wid.sledzenie;
			break;
		}
		case 'Z':   // zoom - zmniejszenie k¹ta widzenia
		{
			par_wid.zoom /= 1.1;
			RECT rc;
			GetClientRect(okno, &rc);
			ZmianaRozmiaruOkna(rc.right - rc.left, rc.bottom - rc.top);
			break;
		}
		case 'X':   // zoom - zwiêkszenie k¹ta widzenia
		{
			par_wid.zoom *= 1.1;
			RECT rc;
			GetClientRect(okno, &rc);
			ZmianaRozmiaruOkna(rc.right - rc.left, rc.bottom - rc.top);
			break;
		}
		

		case 'F':  // przekazanie 10 kg paliwa pojazdom zaznaczonym
		{
			for (map<int, MovableObject*>::iterator it = network_vehicles.begin(); it != network_vehicles.end(); ++it)
			{
				if (it->second)
				{
					MovableObject *ob = it->second;
					if (ob->czy_zazn)
						float ilosc_p = WyslaniePrzekazu(ob->iID, PALIWO, 10);
				}
			}
			break;
		}
		case 'G':  // przekazanie 100 jednostek gotowki pojazdom zaznaczonym
		{
			for (map<int, MovableObject*>::iterator it = network_vehicles.begin(); it != network_vehicles.end(); ++it)
			{
				if (it->second)
				{
					MovableObject *ob = it->second;
					if (ob->czy_zazn)
						float ilosc_p = WyslaniePrzekazu(ob->iID, PIENIADZE, 100);
				}
			}
			break;
		}
		
		case 'L':     // rozpoczęcie zaznaczania metodą lasso
			Lwcisniety = true;
			break;
		
		

		} // switch po klawiszach

		break;
	}

	case WM_KEYUP:
	{
		switch (LOWORD(wParam))
		{
		case VK_SHIFT:
		{
			SHIFTwcisniety = 0;
			break;
		}
		case VK_CONTROL:
		{
			CTRLwcisniety = 0;
			break;
		}
		case VK_MENU:
		{
			ALTwcisniety = 0;
			break;
		}
		case 'L':     // zakonczenie zaznaczania metodą lasso
			Lwcisniety = false;
			break;
		case VK_SPACE:
		{
			my_vehicle->ham = 0.0;
			break;
		}
		case VK_UP:
		{
			my_vehicle->F = 0.0;

			break;
		}
		case VK_DOWN:
		{
			my_vehicle->F = 0.0;
			break;
		}
		case VK_LEFT:
		{
			my_vehicle->alfa = 0;
			break;
		}
		case VK_RIGHT:
		{
			my_vehicle->alfa = 0;
			break;
		}

		}

		break;
	}

	} // switch po komunikatach
}

/********************************************************************
FUNKCJA OKNA realizujaca przetwarzanie meldunków kierowanych do okna aplikacji*/
LRESULT CALLBACK WndProc(HWND okno, UINT kod_meldunku, WPARAM wParam, LPARAM lParam)
{

	// PONIŻSZA INSTRUKCJA DEFINIUJE REAKCJE APLIKACJI NA POSZCZEGÓLNE MELDUNKI 


	KlawiszologiaSterowania(kod_meldunku, wParam, lParam);

	switch (kod_meldunku)
	{
	case WM_CREATE:  //meldunek wysyłany w momencie tworzenia okna
	{

		g_context = GetDC(okno);

		srand((unsigned)time(NULL));
		int wynik = InicjujGrafike(g_context);
		if (wynik == 0)
		{
			printf("nie udalo sie otworzyc okna graficznego\n");
			//exit(1);
		}

		PoczatekInterakcji();

		SetTimer(okno, 1, 10, NULL);

		return 0;
	}
	case WM_KEYDOWN:
	{
		switch (LOWORD(wParam))
		{
		case VK_F1:  // wywolanie systemu pomocy
		{
			char lan[1024], lan_bie[1024];
			//GetSystemDirectory(lan_sys,1024);
			GetCurrentDirectory(1024, lan_bie);
			strcpy(lan, "C:\\Program Files\\Internet Explorer\\iexplore ");
			strcat(lan, lan_bie);
			strcat(lan, "\\pomoc.htm");
			int wyni = WinExec(lan, SW_NORMAL);
			if (wyni < 32)  // proba uruchominia pomocy nie powiodla sie
			{
				strcpy(lan, "C:\\Program Files\\Mozilla Firefox\\firefox ");
				strcat(lan, lan_bie);
				strcat(lan, "\\pomoc.htm");
				wyni = WinExec(lan, SW_NORMAL);
				if (wyni < 32)
				{
					char lan_win[1024];
					GetWindowsDirectory(lan_win, 1024);
					strcat(lan_win, "\\notepad pomoc.txt ");
					wyni = WinExec(lan_win, SW_NORMAL);
				}
			}
			break;
		}
		
		case VK_ESCAPE:   // wyjście z programu
		{
			SendMessage(okno, WM_DESTROY, 0, 0);
			break;
		}
		}
		return 0;
	}

	case WM_PAINT:
	{
		PAINTSTRUCT paint;
		HDC kontekst;
		kontekst = BeginPaint(okno, &paint);

		RysujScene();
		SwapBuffers(kontekst);

		EndPaint(okno, &paint);



		return 0;
	}

	case WM_TIMER:

		return 0;

	case WM_SIZE:
	{
		int cx = LOWORD(lParam);
		int cy = HIWORD(lParam);

		ZmianaRozmiaruOkna(cx, cy);

		return 0;
	}

	case WM_DESTROY: //obowiązkowa obsługa meldunku o zamknięciu okna
		if (lParam == 100)
			MessageBox(okno, "Jest zbyt późno na dołączenie do wirtualnego świata. Trzeba to zrobić zanim inni uczestnicy zmienią jego stan.", "Zamknięcie programu", MB_OK);

		ZakonczenieInterakcji();
		ZakonczenieGrafiki();

		ReleaseDC(okno, g_context);
		KillTimer(okno, 1);

		PostQuitMessage(0);
		return 0;

	default: //standardowa obsługa pozostałych meldunków
		return DefWindowProc(okno, kod_meldunku, wParam, lParam);
	}

}

