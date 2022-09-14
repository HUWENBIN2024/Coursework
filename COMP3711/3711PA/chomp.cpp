// Your Name: HU,Wenbin
// Your Student ID: 20760747
// Your email address: whuak@connect.ust.hk
// Your COMP3711 lecture  (L1 or L2): L2

#include <array>
#include <iostream>
#include <vector>
using namespace std;

bool setTrue(int a, int b, int c, int d, int e);
bool setFalse(int a, int b, int c, int d, int e);
bool report(int a, int b, int c, int d, int e);

//helper function: 吃掉某块时巧克力的状态
void bite(int *next, int x, int y)
{
    const int size = 5;
    for (int i = size; i >= y; i--)
    {
        next[i] = min(x - 1, next[i]);
    }
}

void create(int n)
{
    setFalse(1, 0, 0, 0, 0);
    for (int a = 1; a <= n; a++)
    {
        for (int b = 0; b <= a; b++)
        {
            for (int c = 0; c <= b; c++)
            {
                for (int d = 0; d <= c; d++)
                {
                    for (int e = 0; e <= d; e++)
                    {
                        bool winning_position = 0;
                        const int arr[6] = {0, a, b, c, d, e};
                        for (int y = 1; y <= 5; y++)
                        {
                            for (int x = 1; x <= arr[y]; x++)
                            {
                                if (x == 1 && y == 1)
                                    continue;
                                int next[6] = {0, a, b, c, d, e};
                                bite(next, x, y);
                                if (report(next[1], next[2], next[3], next[4], next[5]))
                                    winning_position = 0;
                                else
                                {
                                    winning_position = 1;
                                    break;
                                }
                            }
                            if (winning_position)
                                break;
                        }
                        if (winning_position)
                            setTrue(a, b, c, d, e);
                        else
                            setFalse(a, b, c, d, e);
                    }
                }
            }
        }
    }
}