#include <iostream>
#include <string.h>
#include <vector>

using namespace std;

struct TCB {
    int state;
    string name;
};

void output(TCB* tcb)
{
    cout << "state: " <<  tcb->state << " name:" << tcb->name << endl;
}

int main (int argv, char** argc)
{
    TCB tcb;
    tcb.state = 0;
    tcb.name = "blah";
    output(&tcb);

    tcb.name = "test";
    tcb.state = 128;
    output(&tcb);

    tcb.state = 100;

    vector<int> a;
    a.push_back(1);
    a.push_back(3);
    return 0;
}