#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <string>
#include <fstream>
#include <map>
#include <stdlib.h>
#include <string>
#include <list>
#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <math.h>
#include <string.h>
void swap(int &a, int &b)
{
    int temp;
    temp = a;
    a = b;
    b = temp;
}
void choose_sort(int* arr, int len)
{
    for(int i = 0; i < len; i++)
    {
        for(int j = i+1; j < len; j++)
        {
            if(arr[i] > arr[j])
            {
                swap(arr[i], arr[j]);
            }
        }
    }
}
void bubble_sort(int* arr, int len)
{
        int i = 0;
        int j = 0;
        int tmp = 0;
        for (i = 0; i < len - 1; i++)
        {	
            int flag = 0;
            for (j = 0; j < len - i - 1; j++)//确定比较次数
            {
                if (arr[j]>arr[j + 1])
                {
                    swap(arr[j], arr[j + 1]);
                    flag = 1;
                }
            }
            if (flag == 0)//如果没有交换过元素，则已经有序
            {
                return;
            }
        }
}

void quickSort(int *arr, int begin, int end)
{
    if(end > begin) {
        int i = begin;
        int j = end;
        int tmp = arr[begin];
        while(i < j)
        {
            while(i < j && tmp < arr[j])
                j--;
            arr[i] = arr[j];
            while(i < j && tmp >= arr[i])
                i++;
            arr[j] = arr[i];
            arr[i] = tmp;
            quickSort(arr, begin, i-1);
            quickSort(arr, i+1, end);
        }
    }
    else 
        return;
}

