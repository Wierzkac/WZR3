

#define _WEKTOR__H
//#define float double

class Wektor3
{
public:      
  float x,y,z;
  Wektor3();
  Wektor3(float _x,float _y, float _z);
  Wektor3 operator* (float value);
  
  Wektor3 operator/ (float value);
  Wektor3 operator+=(Wektor3 v);    // dodanie vec+vec
  Wektor3 operator+(Wektor3 v);     // dodanie vec+vec
  Wektor3 operator+=(float stala);    // dodanie vec+liczba
  Wektor3 operator+(float stala);     // dodanie vec+liczba
  Wektor3 operator-=(Wektor3 v);    // dodanie vec+vec
  Wektor3 operator-(Wektor3 v);     // odejmowanie vec-vec
  Wektor3 operator*(Wektor3 v);     // iloczyn wektorowy
  Wektor3 operator- ();              // odejmowanie vec-vec
  bool operator==(Wektor3 v2); // porownanie
  Wektor3 obrot(float alfa,float vn0,float vn1,float vn2);
  Wektor3 obrot(float alfa,Wektor3 os);
  Wektor3 znorm();
  Wektor3 znorm2D();
  float operator^(Wektor3 v);        // iloczyn scalarny
  float dlugosc(); // dlugosc aktualnego wektora    
};

Wektor3 normalna(Wektor3 A,Wektor3 B, Wektor3 C);  // normalna do trojkata ABC

// rzut punktu A na plaszczyzne o normalnej N i punkcie P:
Wektor3 rzut_punktu_na_pl(Wektor3 A, Wektor3 N, Wektor3 P);

// rzut prostopadly punktu P na prosta AB
Wektor3 rzut_punktu_na_prosta(Wektor3 P, Wektor3 A, Wektor3 B);

// najbli¿szy punkt le¿¹cy na odcinku AB do punktu P:
Wektor3 najblizszy_punkt_na_odcinku(Wektor3 P, Wektor3 A, Wektor3 B);

// odleglosc punktu A od plaszczyzny o normalnej N i punkcie P:
float odleglosc_punktu_od_pl(Wektor3 A, Wektor3 N, Wektor3 P);

// odleglosc pomiêdzy punktem P a prost¹ wyznaczana przez punkty A,B
float odleglosc_pom_punktem_a_prosta(Wektor3 P, Wektor3 A, Wektor3 B);

// ró¿ni siê od poprzedniej funkcji tym, ¿e jest ograniczona do odcinka, czyli jeœli
// rzut punktu le¿y poza odcinkiem to wynikiem jest odleg³oœæ do najbli¿szego punktu ze zb. {A,B}
float odleglosc_pom_punktem_a_odcinkiem(Wektor3 P, Wektor3 A, Wektor3 B);

// Punkt przeciecia wektora AB z plaszczyzna o normalnej N i punkcie P
Wektor3 punkt_przec_prostej_z_plaszcz(Wektor3 A, Wektor3 B, Wektor3 N, Wektor3 P);

// Punkt przeciecia dwoch prostych przy zalozeniu, ze obie proste
// leza na jednej plaszczyznie. Proste dane w postaci wektorow W i punktow P (19 mnozen)
Wektor3 punkt_przec_dwoch_prostych(Wektor3 W1, Wektor3 P1, Wektor3 W2, Wektor3 P2);

// odleg³oœæ pomiêdzy prostymi wyznaczonymi przez punkty AB i CD + punkty le¿¹ce najbli¿ej na ka¿dej z prostych
float odleglosc_pom_prostymi(Wektor3 A, Wektor3 B, Wektor3 C, Wektor3 D, 
             Wektor3 *Xab, Wektor3 *Xcd);

// Sprawdzenie czy punkt lezacy na plaszczyznie ABC znajduje sie
// tez w trojkacie ABC:
bool czy_w_trojkacie(Wektor3 A,Wektor3 B, Wektor3 C, Wektor3 P);

float kat_pom_wekt2D(Wektor3 Wa, Wektor3 Wb);  // zwraca kat pomiedzy wektorami

/*
   wyznaczanie punktu przeciecia sie 2 odcinkow AB i CD lub ich przedluzen
   zwraca 1 jesli odcinki sie przecinaja 
*/
bool punkt_przeciecia2D(float *x,float *y,float xA,float yA, float xB, float yB,
                        float xC,float yC, float xD, float yD);

/*
    Minimalna odleglosc pomiedzy odcinkami AB i CD obliczana jako odleglosc 2 plaszczyzn
    wzajemnie rownoleglych , z ktorych kazda zawiera jeden z odcinkow (jesli rzuty odcinkow
    sie przecinaja) lub minimalna odleglosc ktoregos z punktow A,B,C lub D do drugiego odcinka 
    
    dodatkowo na wejsciu punkty na prostych AB i CD ktore mozna polaczyc 
    odcinkiem o minimalnej dlugosci oraz info czy rzuty odcinkow sie przecinaja
    (wowczas oba punkty Xab oraz Xcd leza na odcinkach AB i CD)
*/
//float odleglosc_pom_odcinkami(Wektor3 A, Wektor3 B, Wektor3 C, Wektor3 D, 
//             Wektor3 *Xab, Wektor3 *Xcd, bool *czy_przeciecie);



void wektory_sprawdzenie_dodatkow();


