#include <iostream>
#include <stdio.h>
#include <stdlib.h>
struct list_node
{
    int num;
    struct list_node* next;
};

struct list_node* init_list();
int front_insert_list(int num, struct list_node* head);
int tail_insert_list(int num, struct list_node* head);
int delete_list(int num, struct list_node* head);
void show_list(struct list_node* head);