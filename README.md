# 事前準備

## 更改 esp-idf

![image](https://user-images.githubusercontent.com/1200981/117580273-ce6da600-b129-11eb-94ec-f9f23b02a33c.png)

![image](https://user-images.githubusercontent.com/1200981/117580286-d9283b00-b129-11eb-856b-5678c996cbe1.png)

![image](https://user-images.githubusercontent.com/1200981/117580297-e513fd00-b129-11eb-95e3-e8ddf6091ccd.png)

# serial

index.js 記得改對連接電腦的 ESP32 的 port number

# 常用指令

> port number 改你們自己的

idf.py -p /dev/cu.SLAB_USBtoUART flash
idf.py -p /dev/cu.SLAB_USBtoUART monitor
idf.py -p /dev/cu.SLAB_USBtoUART erase_flash
