#include<fstream>
#include<iostream>
#include<string>
#include<cstdlib>
#include<ctime>
using namespace std;
#define NUM 20
#define MINX 20
#define MAXX 90
#define MINY 20
#define MAXY 120
int main()
{
    ifstream fin1("const.txt");
    ofstream fout("maze.world");
    while(!fin1.eof())
    {
        char buff[500];
        fin1.getline(buff,500);
        fout<<buff<<endl;
    }
    srand(time(0));
    for (int i=1;i<NUM;i++)
    {
        double x=(rand()% ((MAXX-MINX)*10) )/10.0 +MINX;
        double y=(rand()% ((MAXY-MINY)*10) )/10.0 +MINY;
        fout<<"turtlebot"<<endl;
        fout<<"("<<endl;
        fout<<"pose [ "<<x<<" "<<y<<" 0 0 ] "<<endl;
        fout<<"name \"turtlebot"<<i<<"\" "<<endl;
        switch (i%4){
            case 0:
                fout<<"color \"blue\")"<<endl;break;
            case 1:
                fout<<"color \"yellow\")"<<endl;break;
            case 2:
                fout<<"color \"red\")"<<endl;break;
            case 3:
                fout<<"color \"green\")"<<endl;break;
         }
    }
}
