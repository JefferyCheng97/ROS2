import threading
import requests

class Download:
    def download(self, url, callback_wc):
        print(f"线程: {threading.get_ident()} 开始下载 {url}")
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback_wc(url, response.text)

    def start_download(self, url, callback_wc):
        thread = threading.Thread(target=self.download, args=(url, callback_wc))
        thread.start()
        

def wc(url, result):
    print(f"{url}: {len(result)} -> {result[:5]}")

def main():
    download = Download()
    download.start_download("http://localhost:8000/novel1.txt", wc)
    download.start_download("http://localhost:8000/novel2.txt", wc)
    download.start_download("http://localhost:8000/novel3.txt", wc)
