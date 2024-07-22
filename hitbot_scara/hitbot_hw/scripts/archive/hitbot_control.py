import queue
import threading
import time


def producer(data, q):
    for item in data:
        q.put(item)
        time.sleep(0.1)  # 模擬數據更新的時間


def consumer(q):
    while True:
        item = q.get()
        print("消費者獲得:", item)


# 創建一個隊列
my_queue = queue.Queue()

# 要實時更新的數據
data_to_update = [1, 2, 3, 4, 5]

# 創建一個生產者執行緒
producer_thread = threading.Thread(target=producer, args=(data_to_update, my_queue))
producer_thread.start()

# 創建一個消費者執行緒
consumer_thread = threading.Thread(target=consumer, args=(my_queue,))
consumer_thread.start()

# 主執行緒等待生產者執行緒和消費者執行緒完成
producer_thread.join()
consumer_thread.join()
