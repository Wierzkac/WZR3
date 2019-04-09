#define _OBJECTS__H
#include <stdio.h>
#include "quaternion.h"

#define PI 3.1416
extern class Terrain;

struct StanObiektu
{
	int iID;                  // identyfikator obiektu
	Wektor3 wPol;             // polozenie obiektu (œrodka geometrycznego obiektu) 
	kwaternion qOrient;       // orientacja (polozenie katowe)
	Wektor3 wV, wA;            // predkosc, przyspiesznie liniowe
	Wektor3 wV_kat, wA_kat;   // predkosc i przyspeszenie liniowe
	float masa_calk;          // masa ca³kowita (wa¿na przy kolizjach)
	long pieniadze;
	float ilosc_paliwa;
	int iID_wlasc;
	int czy_autonom;
};


class MovableObject
{
public:
	int iID;                  // identyfikator obiektu

	// parametry stanu:
	Wektor3 wPol;             // polozenie obiektu
	kwaternion qOrient;       // orientacja (polozenie katowe)
	Wektor3 wV, wA;            // predkosc, przyspiesznie liniowe
	Wektor3 wV_kat, wA_kat;   // predkosc i przyspeszenie liniowe
	long pieniadze;
	float ilosc_paliwa;

	// parametry akcji:
	float F;                  // F - si³a pchajaca do przodu (do ty³u ujemna)
	float ham;                // stopieñ hamowania Fh_max = tarcie*Fy*ham
	float alfa;               // kat skretu kol w radianach (w lewo - dodatni)



	// decyzje zwi¹zane z trwa³¹ wspó³prac¹ polegaj¹c¹ na dzieleniu siê
	// paliwem i gotówk¹ pó³ na pó³:
	bool wyslanie_zaproszenia_do_dzielenia;
	bool zgoda_na_dzielenie;
	bool koniec_dzielenia;
	int ID_koalicjanta;       // identyfikator obiektu, z którym chcemy nawi¹zaæ wspó³pracê (-1 z nikim)

	// decyzje zwi¹zane z kupnem 10 kg paliwa:
	long cena_od_kupca_paliwa;     // cena zaproponowana przez chc¹cego kupiæ paliwo
	bool chec_sprzedazy_paliwa;    // chêæ sprzeda¿y paliwa po zaproponowanej cenie 
	int ID_kupca_paliwa;           // identyfikator kupca paliwa, gdy podejmiemy decyzjê o sprzeda¿y (-1 brak takiej decyzji)
	int ID_przedawcy_paliwa;       // identyfikator sprzedawcy paliwa, gdy podejmiemy decyzjê o jego kupnie

	// decyzje zwi¹zane z podniesieniem ciê¿kiej monety:
	int nr_przedm_do_podniesienia;        // numer przedmiotu do podniesienia
	bool czy_przelew_polowy_wartosci;     // decyzja o przelewie w zwi¹zku z pomoc¹


	// parametry sta³e:
	float umiejetn_sadzenia,    // umiejêtnoœæ sadzenia drzew (1-pe³na, 0- brak)
		umiejetn_zb_paliwa,   // umiejêtnoœæ zbierania paliwa
		umiejetn_zb_monet;

	bool czy_zazn;            // czy obiekt zaznaczony  

	// parametry fizyczne pojazdu:
	float F_max;
	float alfa_max;
	float Fy;                 // si³a nacisku na podstawê pojazdu - gdy obiekt styka siê z pod³o¿em (od niej zale¿y si³a hamowania)
	float masa_wlasna;		  // masa w³asna obiektu (bez paliwa)	
	float masa_calk;          // masa ca³kowita (wa¿na przy kolizjach)
	float dlugosc, szerokosc, wysokosc; // rozmiary w kierunku lokalnych osi x,y,z
	float promien;            // promien minimalnej sfery, w ktora wpisany jest obiekt
	float przeswit;           // wysokoœæ na której znajduje siê podstawa obiektu
	float dl_przod;           // odleg³oœæ od przedniej osi do przedniego zderzaka 
	float dl_tyl;             // odleg³oœæ od tylniej osi do tylniego zderzaka 

	// inne: 
	int iID_kolid;            // identyfikator pojazdu z którym nast¹pi³a kolizja (-1  brak kolizji)
	Wektor3 wdV_kolid;        // poprawka prêdkoœci pojazdu koliduj¹cego
	int iID_wlasc;
	bool czy_autonom;         // czy obiekt jest autonomiczny
	long nr_wzietego_przedm;   // numer wzietego przedmiotu
	float wartosc_wzieta;   // wartosc wzietego przedmiotu
	long nr_odnowionego_przedm;


	float czas_symulacji;     // czas sumaryczny symulacji obiektu   
	Terrain *terrain;             // wskaŸnik do terrainu, do którego przypisany jest obiekt

public:
	MovableObject(Terrain *t);          // konstruktor
	~MovableObject();
	void ZmienStan(StanObiektu stan);          // zmiana stanu obiektu
	StanObiektu Stan();        // metoda zwracajaca stan obiektu
	void Symulacja(float dt);  // symulacja ruchu obiektu w oparciu o biezacy stan, przylozone sily
	// oraz czas dzialania sil. Efektem symulacji jest nowy stan obiektu 
	void Rysuj();			   // odrysowanie obiektu					
};

enum TYpyPrzedm { PRZ_MONETA, PRZ_BECZKA, PRZ_DRZEWO, PRZ_BUDYNEK, PRZ_PUNKT, PRZ_KRAWEDZ, PRZ_MIEJSCE_STARTOWE, PRZ_MUR, PRZ_BRAMKA };
//char *PRZ_nazwy[] = { "moneta", "beczka", "drzewo", "punkt", "krawedz", "miejsce startowe", "mur", "bramka" };
enum PodtypyDrzew { DRZ_TOPOLA, DRZ_SWIERK, DRZ_BAOBAB, DRZ_FANTAZJA };
//char *DRZ_nazwy[] = { "topola", "swierk", "baobab", "fantazja" };
enum PodtypyPunktow {PUN_ZWYKLY, PUN_KRAWEDZI, PUN_WODY};

struct Przedmiot
{
	Wektor3 wPol;             // polozenie obiektu
	kwaternion qOrient;       // orientacja (polozenie katowe)

	int typ;
	int podtyp;
	long indeks;                 // identyfikator - numer przedmiotu (jest potrzebny przy przesy³aniu danych z tego wzglêdu,
	//    ¿e wyszukiwanie przedmiotów (np. Przedmioty_w_promieniu()) zwraca wskaŸniki)

	float wartosc;            // w zal. od typu nomina³ monety /ilosc paliwa, itd.
	float srednica;           // np. grubosc pnia u podstawy, srednica monety
	float srednica_widocz;          // œrednica widocznoœci - sfery, która opisuje przedmiot
	float param_f[3];        // dodatkowe parametry roznych typow
	long param_i[3];
	bool czy_zazn;            // czy przedmiot jest zaznaczony przez uzytkownika
	long grupa;               // numer grupy do której nale¿y przedmiot;

	bool do_wziecia;          // czy przedmiot mozna wziac
	bool czy_ja_wzialem;      // czy przedmiot wziety przeze mnie
	bool czy_odnawialny;      // czy mo¿e siê odnawiaæ w tym samym miejscu po pewnym czasie
	long czas_wziecia;        // czas wziêcia (potrzebny do przywrócenia)

	
	unsigned long nr_wyswietlenia;   // zgodny z liczba wyœwietleñ po to, by nie powtarzaæ rysowania przedmiotów
};

struct PunktLasso
{
	float x;
	float y;
};

class Sektor
{
public:
	long w, k;                    // wiersz i kolumna okreœlaj¹ce po³o¿enie obszaru na nieograniczonej p³aszczyŸnie
	int liczba_oczek;             // liczba oczek na mapie w poziomie i pionie (potêga dwójki!) 
	//float rozmiar_sektora;        // potrzebny do obliczenia normalnych
	Przedmiot **wp;               // jednowymiarowa tablica wskaŸników do przedmiotów znajduj¹cych siê w pewnym obszarze
	long liczba_przedmiotow;      // liczba przedmiotów w obszarze 
	long liczba_przedmiotow_max;  // rozmiar tablicy przedmiotów z przydzielon¹ pamiêci¹

	float **mapa_wysokosci;         // wysokoœci wierzcho³ków
	int **typy_naw;                 // typy nawierzchni
	float **poziom_wody;            // poziom wody w ka¿dym oczku 
	Wektor3 ****Norm;                // wektory normalne do poszczególnych p³aszczyzn (rozdzielczoœæ x wiersz x kolumna x 4 p³aszczyzny);

	float **mapa_wysokosci_edycja;    // mapa w trakcie edycji - powinna byæ widoczna ³¹cznie z dotychczasow¹ map¹
	Wektor3 ****Norm_edycja;   
	int **typy_naw_edycja;
	float **poziom_wody_edycja;
	int liczba_oczek_edycja;

	// podstawowe w³aœciwoœci dla ca³ego sektora, gdyby nie by³o mapy
	int typ_naw_sek;                              // typ nawierzchni gdyby nie by³o mapki
	float wysokosc_gruntu_sek;
	float poziom_wody_sek;

	MovableObject **wob;                          // tablica obiektów ruchomych, które mog¹ znaleŸæ siê w obszarze w ci¹gu
	// kroku czasowego symulacji (np. obiekty b. szybkie mog¹ znaleŸæ siê w wielu obszarach)
	long liczba_obiektow_ruch;
	long liczba_obiektow_ruch_max;

	float wysokosc_max;             // maksymalna wysokoœæ terrainu w sektorze -> istotna dla wyznaczenia rozdzielczoœci np. przy widoku z góry
	int liczba_oczek_wyswietlana;   // liczba oczek (rozdzielczoœæ) aktualnie wyœwietlana, w zal. od niej mo¿na dopasowaæ s¹siednie sektory
	                                // by nie by³o dziur
	int liczba_oczek_wyswietlana_pop;  // poprzednia liczba oczek, by uzyskaæ pa³ne dopasowanie rozdzielczoœci s¹siaduj¹cych sektorów 
	//float promien;                // promien sektora zwykle d³ugoœæ po³owy przek¹tnej kwadratu, mo¿e byæ wiêkszy, gdy w
	                              // œrodku du¿e ró¿nice wysokoœci lub du¿e przedmioty lub obiekty

	Sektor(){};
	Sektor(int _loczek, long _w, long _k, bool czy_mapa);
	~Sektor();
	void pamiec_dla_mapy(int __liczba_oczek, bool czy_edycja=0);
	void zwolnij_pamiec_dla_mapy(bool czy_edycja = 0);
	void wstaw_przedmiot(Przedmiot *p);
	void usun_przedmiot(Przedmiot *p);
	void wstaw_obiekt_ruchomy(MovableObject *o);
	void usun_obiekt_ruchomy(MovableObject *o);
	void oblicz_normalne(float rozmiar_sektora, bool czy_edycja=0);         // obliczenie wektorów normalnych N do p³aszczyzn trójk¹tów, by nie robiæ tego ka¿dorazowo przy odrysowywaniu   
};

struct KomorkaTablicy  // komórka tablicy rozproszonej (te¿ mog³aby byæ tablic¹ rozproszon¹)
{
	Sektor **sektory;  // tablica wskaŸników do sektorów (mog¹ zdarzaæ siê konflikty -> w jednej komórce wiele sektorów) 
	long liczba_sektorow; // liczba sektorów w komórce
	long rozmiar_pamieci; // liczba sektorów, dla której zarezerwowano pamiêæ
};

class TabSektorow      // tablica rozproszona przechowuj¹ca sektory
{
	//private:
public:
	long liczba_komorek;                                 // liczba komórek tablicy
	long ogolna_liczba_sektorow;
	long w_min, w_max, k_min, k_max;                     // minimalne i maksymalne wspó³rzêdne sektorów (przyspieszaj¹ wyszukiwanie)  
	KomorkaTablicy *komorki;
	unsigned long wyznacz_klucz(long w, long k);          // wyznaczanie indeksu komórki

public:

	TabSektorow();        // na wejœciu konstruktora liczba komórek tablicy
	~TabSektorow();

	Sektor *znajdz(long w, long k);       // znajdowanie sektora (zwraca NULL jeœli nie znaleziono)
	Sektor *wstaw(Sektor *s);              // wstawianie sektora do tablicy
	void usun(Sektor *s);              // usuwanie sektora
};

struct ParamFaldy
{
	float srednica_faldy;
	float wysokosc_faldy;
	int liczba_oczek_faudy;           // liczba oczek w ka¿dym sektorze, w którym tworzona jest fa³da 
	bool czy_wypukla;                 // 1-wypuk³oœæ w górê, 0 - w dó³
	float wariancja_wysokosci;        // losowa wariancja wysokoœci ka¿dego wierzcho³ka fa³dy
	int ksztalt_przekroju;
	float srednica_wewnetrzna;        // pozwala na uzyskanie p³askiego miejsca na górze lub dole fa³dy w zal. od wypuk³oœci
	bool czy_stale_nachylenie;        // czy stoki wa³u maj¹ sta³e pochylenie -> wówczas maj¹ one sta³y k¹t nachylenia, niezale¿nie
	// od wysokoœci (np. jak nasyp drogowy) -> nie jest brana pod uwagê œrednica fa³dy ale k¹t nachylenia
	float kat_nachylenia;             // k¹t nachylenia stoku w radianach np. pi/4 == 45 stopni

	int sposob_laczenia_faudy;        // sposób ³¹czenia nowoutworzonej fa³dy z obecn¹ powierzchni¹ terrainu 
};

class Terrain
{
public:
	void WstawPrzedmiotWsektory(Przedmiot *prz);
	void UsunPrzedmiotZsektorow(Przedmiot *prz);
public:

	long liczba_przedmiotow;      // liczba przedmiotów na planszy
	long liczba_przedmiotow_max;  // rozmiar tablicy przedmiotów

	TabSektorow *ts;

	

	// inne istotne w³aœciwoœci terrainu:
	float rozmiar_sektora;        // w [m] zamiast rozmiaru pola
	float czas_odnowy_przedm;     // czas w [s] po którym nastêpuje odnowienie siê przedmiotu 
	bool czy_toroidalnosc;        // inaczej cyklicznosc -> po dojsciu do granicy przeskakujemy na
	                              // pocz¹tek terrainu, zarówno w pionie, jak i w poziomie (gdy nie ma granicy, terrain nie mo¿e byæ cykliczny!)
	float granica_x, granica_z;   // granica terrainu (jeœli -1 - terrain nieskoñczenie wielki)

	// parametry wyœwietlania:
	float szczeg;                 // stopieñ szczegó³owoœci w przedziale 0,1 
	unsigned long liczba_wyswietlen;   // potrzebne do unikania podwójnego rysowania przedmiotów
	long *zazn_przedm;     // tablica numerów zaznaczonych przedmiotów (dodatkowo ka¿dy przedmiot posiada pole z informacj¹ o zaznaczeniu)
	long liczba_zazn_przedm,      // liczba aktualnie zaznaczonych przedmiotów
		liczba_zazn_przedm_max;   // liczba miejsc w tablicy 

	ParamFaldy pf_rej[10];        // parametry edycji fa³d (s¹ tu tylko ze wzglêdu na koniecznoœæ zapisu do pliku

	Przedmiot *p;          // tablica przedmiotow roznego typu

	Terrain();
	~Terrain();
	float WysokoscGruntu(float x, float z);      // okreœlanie wysokoœci gruntu dla punktu o wsp. (x,z)
	float WysokoscNaPrzedmiocie(Wektor3 pol, Przedmiot *prz); // wysokoœæ na jakiej znajdzie siê przeciêcie pionowej linii przech. przez pol na górnej powierzchni przedmiotu
	float Wysokosc(Wektor3 pol);                 // okreœlenie wysokoœci nad najbli¿szym przedmiotem lub gruntem patrz¹c w dó³ od punktu pol
	void WspSektora(long *w, long *k, float x, float z);  // na podstawie wsp. po³o¿enia punktu (x,z) zwraca wsp. sektora 
	void PolozeniePoczSektora(float *x, float *z, long w, long k); // na podstawie wspó³rzêdnych sektora (w,k) zwraca po³o¿enie punktu pocz¹tkowego
	float wysokosc_na_najw_zazn_przedm(Wektor3 pol);
	Wektor3 wspolrzedne_kursora3D_bez_paralaksy(int X, int Y);
	void Rysuj();	      // odrysowywanie terrainu 
	void PoczatekGrafiki();               // tworzenie listy wyœwietlania
	int Zapis(char nazwa_pliku[]);        // zapis mapy w pliku o podanej nazwie
	int Odczyt(char nazwa_pliku[]);       // odczyt mapy z pliku
	void UmieszczeniePrzedmiotuWterrainie(Przedmiot *prz);     // m.i. ustalenie wysokoœci, na której le¿y przedmiot w zal. od ukszta³t. terrainu
	long WstawPrzedmiot(Przedmiot prz);   // wstawia przedmiot do tablic, zwraca numer przedmiotu w tablicy p[]
	void ZaznaczOdznaczPrzedmiotLubGrupe(long nr_prz);// wpisywanie w pole ¿e jest zaznaczony lub nie, dodawanie/usuwanie do/z listy zaznaczonych 
	void UsunZaznPrzedmioty();
	void NowaMapa();                      // tworzenie nowej mapy (zwolnienie pamiêci dla starej)
	
	long Przedmioty_w_promieniu(Przedmiot*** wsk_prz, Wektor3 pol, float promien);
	void WstawObiektWsektory(MovableObject *ob);
	void UsunObiektZsektorow(MovableObject *ob);
	long Obiekty_w_promieniu(MovableObject*** wsk_ob, Wektor3 pol, float promien);
};