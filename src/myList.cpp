#include "include/myList.h"

struct list_node* init_list() {
    struct list_node* head = (struct list_node*)malloc(sizeof(struct list_node));
    if(head == NULL) {
        perror("list node malloc failed");
        return NULL;
    }
    head->next = NULL;
    return head;
}

int front_insert_list(int num, struct list_node* head) {
    struct list_node* tmp = init_list();
    if(tmp == NULL) {
        perror("malloc failed");
        return 0;
    }
    tmp->num = num;
    tmp->next = head->next;
    head->next = tmp;
    return 1;
}

int tail_insert_list(int num, struct list_node* head) {
    
}

int delete_list(int num, struct list_node* head) {

}

void show_list(struct list_node* head) {
    struct list_node* tmp = head->next;
    for(tmp; tmp != NULL; tmp = tmp->next) {
        std::cout << tmp->num << " ";
    }
    std::cout << std::endl;
}