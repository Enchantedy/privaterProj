#include "iostream"
#include<typeinfo>
// #include "include/skeletonService.h"
#include "include/singlyList.h"
// #include "include/manager.h"

using namespace std;

int main()
{
    struct list_node* head = init_list();
    tail_insert_list(1, head);
    tail_insert_list(2, head);
    tail_insert_list(3, head);
    tail_insert_list(4, head);
    tail_insert_list(5, head);
    show_list(head);
}