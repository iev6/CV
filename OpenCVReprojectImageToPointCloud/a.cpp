#include<iostream>
#include<fstream>
using namespace std;

struct Normals
{
 double r,g,b,l;
 void out()
 {
	 cout<<r<<' '<<g<<' '<<b<<' '<<l;
 }
 };
 
 int main()

{

  fstream f;
  f.open("normals.clr", ios::in|ios::out);
  Normals n;
  f.read((char*)&n,sizeof(n));
  n.out();
  
  return 0;
  
  }
