#include "iostream"
#include<typeinfo>
// #include "include/skeletonService.h"
// #include "include/myList.h"
// #include "include/manager.h"

using namespace std;

class A
{
    public:
        int cnt;
    private:
        int num;
};

int main()
{
    A a;
    cout << typeid(a).name() << endl;
}