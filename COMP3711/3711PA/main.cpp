// #include <array>
// #include <iostream>
// #include <vector>
// constexpr int M = 5;
// #include "chomp.cpp"
// using namespace std;

// int N=12;   //Max size of a row.  Can be changed later
// enum Status { True, False, Invalid };
// template <typename T> using vec5D = vector<vector<vector<vector<vector<T>>>>>;
// template <typename T> using vec4D = vector<vector<vector<vector<T>>>>;
// template <typename T> using vec3D = vector<vector<vector<T>>>;
// template <typename T> using vec2D = vector<vector<T>>;
// vec5D<Status> WIN = vec5D<Status>(
//     	11, vec4D<Status>(11, vec3D<Status>(11, vec2D<Status>(11, vector<Status>(11, Invalid)))));;

// bool isValid(int a, int b, int c, int d, int e) {
//     array<int, M + 1> rows = {N, a, b, c, d, e};
//     for (int i = 1; i <= M; i++) {
//         if (rows[i] > rows[i - 1])
//             return false;
//     }
//     return true;
// }

// bool setTrue(int a, int b, int c, int d, int e) {
//     if (!isValid(a, b, c, d, e))
//         return false;
//     WIN[a][b][c][d][e] = True;
//     return true;
// }

// bool setFalse(int a, int b, int c, int d, int e) {
//     if (!isValid(a, b, c, d, e))
//         return false;
//     WIN[a][b][c][d][e] = False;
//     return true;
// }

// bool report(int a, int b, int c, int d, int e) {
//     if (!isValid(a, b, c, d, e)) {
//         cerr << "ERROR: Reporting invalid position!!" << endl;
//         exit(1);
//     }
//     if (WIN[a][b][c][d][e] == True)
//         return true;
//     else
//         return false;
// }

// int main() {
// 	int n = 10;
//     create(n);
//     return 0;
// }

#include <array>
#include <iostream>
#include <vector>
constexpr int M = 5;
#include "chomp.cpp"
using namespace std;

int N = 25; //Max size of a row.  Can be changed later
enum Status
{
    True,
    False,
    Invalid
};
template <typename T>
using vec5D = vector<vector<vector<vector<vector<T>>>>>;
template <typename T>
using vec4D = vector<vector<vector<vector<T>>>>;
template <typename T>
using vec3D = vector<vector<vector<T>>>;
template <typename T>
using vec2D = vector<vector<T>>;
vec5D<Status> WIN = vec5D<Status>(
    21, vec4D<Status>(21, vec3D<Status>(21, vec2D<Status>(21, vector<Status>(21, Invalid)))));
;

bool isValid(int a, int b, int c, int d, int e)
{
    array<int, M + 1> rows = {N, a, b, c, d, e};
    for (int i = 1; i <= M; i++)
    {
        if (rows[i] > rows[i - 1])
            return false;
    }
    return true;
}

bool setTrue(int a, int b, int c, int d, int e)
{
    if (!isValid(a, b, c, d, e))
        return false;
    WIN[a][b][c][d][e] = True;
    return true;
}

bool setFalse(int a, int b, int c, int d, int e)
{
    if (!isValid(a, b, c, d, e))
        return false;
    WIN[a][b][c][d][e] = False;
    return true;
}

bool report(int a, int b, int c, int d, int e)
{
    if (!isValid(a, b, c, d, e))
    {
        cerr << "ERROR: Reporting invalid position!!" << endl;
        exit(1);
    }
    if (WIN[a][b][c][d][e] == True)
        return true;
    else
        return false;
}

void check2(int n)
{ //checks correctness of legal positions in form (a,b,0,0,0)
    for (int a = 1; a <= n; a++)
    {
        for (int b = 0; b <= a; b++)
        {
            if (b == (a - 1))
            {
                if (report(a, b, 0, 0, 0) == true)
                    cout << "WIN(" << a << "," << b << ",0,0,0) = true  is incorrect" << endl;
                else
                    cout << "WIN(" << a << "," << b << ",0,0,0) = false is correct" << endl;
            };
            if (b != (a - 1))
            {
                if (report(a, b, 0, 0, 0) == true)
                    cout << "WIN(" << a << "," << b << ",0,0,0) = true  is correct" << endl;
                else
                    cout << "WIN(" << a << "," << b << ",0,0,0) = false is incorrect" << endl;
            };
        }
    }
}

void print(int &running_times)
{
    for (int a = 1; a <= 20; a++)
    {
        for (int b = 0; b <= a; b++)
        {
            for (int c = 0; c <= b; c++)
            {
                for (int d = 0; d <= c; d++)
                {
                    for (int e = 0; e <= d; e++)
                    {
                        if (!report(a, b, c, d, e))
                        {
                            cout << "(" << a << "," << b << "," << c << "," << d << "," << e <<") : " << report(a, b, c, d, e) << endl;
                            running_times++;
                        }
                    }
                }
            }
        }
    }
}

int main()
{
    int n = 20;
    create(n);
    // check2(n);
    int running_times = 0;
    // print(running_times);
    // while(1);
    return 0;
}