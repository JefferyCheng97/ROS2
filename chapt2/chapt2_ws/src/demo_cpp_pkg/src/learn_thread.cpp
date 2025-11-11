#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <cpp-httplib/httplib.h>

using namespace std;

class Download
{
private:
public:
    void download(const string& host, 
        const string& path, 
        const function<void(const string&, const string&)> callback_wc)
    {
        cout << "线程: " << this_thread::get_id() << endl;
        httplib::Client client(host);
        auto response = client.Get(path.c_str());

        if (response && response->status == 200)
        {
            callback_wc(path, response->body);
        }
        else
        {
            cout << "下载失败: " << path << endl;
        }
    }

    void start_download(const string& host, 
        const string& path, 
        const function<void(const string&, const string&)> callback_wc)
    {
        auto download_fun = bind(&Download::download, this,
            placeholders::_1, placeholders::_2, placeholders::_3);
            
        thread thread(download_fun, host, path, callback_wc);
        thread.detach();
    }
};

int main()
{
    auto d = Download();

    // 类型也可以直接写auto
    function<void(const string&, const string&)> wc = [](const string& path, const string& result) -> void
    {
        cout << "下载完成" << path 
            << "长度为 " << result.length() 
            << "->" << (result.size() >= 5 ? result.substr(0, 5) : result)
            <<endl;
    };

    d.start_download("http://127.0.0.1:8080", "/novel1.txt", wc);
    d.start_download("http://127.0.0.1:8080", "/novel2.txt", wc);
    d.start_download("http://127.0.0.1:8080", "/novel3.txt", wc);
    this_thread::sleep_for(chrono::seconds(5));

    return 0;
}
