#include <iostream>
#include <functional>

using namespace std;

void save_with_free_fun(const string& file_name)
{
    cout << "自由函数: " << file_name << endl;
}

class FileSave
{
private:    

public:
    void save_with_member_fun(const string& file_name)
    {
        cout << "成员函数: " << file_name << endl;
    }
};

int main()
{
    FileSave file_save;

    auto save_with_lambda_fun = [](const string& file_name) -> void
    {
        cout << "Lambda 函数: " << file_name << endl;
    };

    save_with_free_fun("file.txt");
    file_save.save_with_member_fun("file.txt");
    save_with_lambda_fun("file.txt");

    // 使用函数包装器来包装三种函数
    function<void(const string&)> save1 = save_with_free_fun;
    function<void(const string&)> save2 = save_with_lambda_fun;
    function<void(const string&)> save3 = bind(&FileSave::save_with_member_fun, 
        &file_save, placeholders::_1);

    save1("file.txt");
    save2("file.txt");
    save3("file.txt");
    
    return 0;
}