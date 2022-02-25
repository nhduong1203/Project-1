# Project-1

## Hướng dẫn biên dịch

Chương trình được viết bằng **python 3** nên trước khi bắt đầu, bạn hãy cài đặt **python** phiên bản 3. trên máy tính của bạn.

### Bắt đầu Python Interpreter

Cách đơn giản nhất để khởi động trình thông dịch là sử dụng trình thông dịch từ command-line.

Để mở command-line:

* Trên Windows, command-line được gọi là Command Prompt hoặc MS-DOS consoler. Một cách nhanh hơn để truy cập nó là vào **Start menu → Run** và nhập **cmd**.
* Trên GNU / Linux, command-line có thể được truy cập bởi một số ứng dụng như xterm, Gnome Terminal hoặc Konsole.
* Trên MAC OS X, thiết bị đầu cuối hệ thống được truy cập thông qua **Application → Utilities → Terminal**.

### Chạy một chương trình python sử dụng lệnh python

Cách cơ bản và dễ dàng nhất để chạy một file code python là sử dụng lệnh python. Bạn cần mở command-line và nhập **python3**, theo sau là đường dẫn đến một file code python như sau:

```
python3 first_script.py
```

Sau đó, bạn nhấn nút ENTER từ bàn phím và thế là xong. Tuy nhiên, nếu trình biên dịch báo lỗi, bạn có thể muốn kiểm tra PATH hệ thống về python và nơi bạn đã lưu trình biên dịch python của mình. Nếu nó vẫn không hoạt động, hãy cài đặt lại Python trong hệ thống của bạn và thử lại.

### Cài đặt các package 

Các tên của các package python được liệt kê trong `requirements.txt`.
Để cài đặt dùng lệnh sau trên command-line.

```
pip3 install -r requirements.txt
```

## Hướng dẫn sử dụng

### Dữ liệu đầu vào

* Dữ liệu đầu vào được miêu tả trong hình dưới đây. Trong đó, có các thông tin:
* Cust No: Số thứ tự của Node, Node 0 sẽ là kho hàng.
* XCoord : Tọa độ x của Node
* YCoord : Tọa đọ y của Node
* Demand : Nhu cầu giao hàng ở Node 
* Ready Time : Thời gian Node bắt đầu mở cửa
* Due Time   : Thời gian Node đóng cửa
* Service time : Thời gian xe phục vụ tại Node

![alt text](https://github.com/nhduong1203/Project-1/blob/main/data/data.png)

### Truyền file dữ liệu đầu vào

Đường dẫn chứa file dữ liệu đầu vào được lưu vào biến file_path trong hàm main

```
file_path = './data/200_customer/C1_2_2.txt'
```

Sau đó có thể thực biện biên dịch chương trình.

### Kết quả

Sau khi biên dịch chương trình, kết quả của bài toán sẽ được ghi vào file result.txt trong cùng thư mục.
