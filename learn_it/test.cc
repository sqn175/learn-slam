#include <iostream>
#include <string.h>
#include <vector>

using namespace std;

int main (int argv, char** argc)
{
    vector<int> v;
    for (int i = 0; i < 1000; ++i) 
        v.push_back(i);
    
    return 0;
}