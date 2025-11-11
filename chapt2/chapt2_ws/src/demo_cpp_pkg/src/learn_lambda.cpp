#include <iostream>
#include <algorithm>

using namespace std;

int main()
{
    auto add = [](int a, int b) -> int 
    {
        return a + b;
    };
    
    int sum = add(200, 50);
    auto print_sum = [sum]() -> void
    {
        cout << sum << endl;
    };

    print_sum();
    
    return 0;
}