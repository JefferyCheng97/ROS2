#include <iostream>
#include <memory>

using namespace std;

int main()
{
    auto p1 = make_shared<string>("This is a str.");
    cout << "p1的引用计数: " << p1.use_count() << ", 指向内存地址: " << p1.get() << endl;

    auto p2 = p1;
    cout << "p1的引用计数: " << p1.use_count() << ", 指向内存地址: " << p1.get() << endl;
    cout << "p2的引用计数: " << p2.use_count() << ", 指向内存地址: " << p2.get() << endl;

    p1.reset();
    cout << "p1的引用计数: " << p1.use_count() << ", 指向内存地址: " << p1.get() << endl;
    cout << "p2的引用计数: " << p2.use_count() << ", 指向内存地址: " << p2.get() << endl;

    cout << "p2中的数据: " << p2->c_str() << endl;

    return 0;
}