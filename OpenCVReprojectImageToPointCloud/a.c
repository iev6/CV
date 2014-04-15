#include<iostream>
using namespace std;

struct Normals
{
 double r,g,b,l;
 void out()
 {
	 cout<<r<<g<<b<<l;
 }
 };
 
 int main()

{
cv::Mat img_norm;
  fstream f;
  f.open("normals.clr", ios::in|ios::out);
  normals n;
  f.read((char*)&n,sizeof(n));
  
  
  }
