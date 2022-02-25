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

File kết quả chứa thông tin về thời gian chạy chương trình, tuyến đường di chuyển của đội xe, số lượng xe sử dụng và tổng quãng đường di chuyển.

```
**************************************************
Time is up: solution in given time(1 minutes)
It takes 60.008 second from multiple_ant_colony_system running
The best path have found is:
Vehicle 1: 0 113 118 83 143 176 128 4 72 82 84 191 60 0

Vehicle 2: 0 62 131 44 192 146 97 14 96 130 15 198 0

Vehicle 3: 0 101 159 38 150 22 151 16 140 187 142 111 63 0

Vehicle 4: 0 30 48 50 156 112 168 79 29 87 42 123 175 0

Vehicle 5: 0 45 178 27 173 154 24 61 100 64 162 74 28 196 0

Vehicle 6: 0 190 5 180 35 126 71 9 1 99 144 166 0

Vehicle 7: 0 20 85 41 80 31 25 172 77 110 109 0

Vehicle 8: 0 164 66 12 129 11 6 122 139 73 116 147 155 0

Vehicle 9: 0 93 119 193 46 125 33 121 165 188 108 34 10 0

Vehicle 10: 0 57 78 58 184 199 37 81 138 55 183 56 0

Vehicle 11: 0 148 103 197 200 141 69 51 174 136 189 124 0

Vehicle 12: 0 170 134 152 40 153 169 89 105 59 133 149 68 0

Vehicle 13: 0 21 23 182 75 163 194 145 195 52 92 0

Vehicle 14: 0 177 18 54 104 132 7 181 117 49 137 135 0

Vehicle 15: 0 161 3 88 8 186 127 98 157 185 53 114 0

Vehicle 16: 0 32 65 86 94 115 171 91 95 158 106 67 0

Vehicle 17: 0 76 120 26 13 43 2 90 17 19 102 179 0

Vehicle 18: 0 36 160 47 70 167 39 107 0

Best path distance is 3846.528864, best vehicle_num is 18
**************************************************
```
