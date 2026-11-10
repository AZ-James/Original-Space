# ZDT通信协议：当前项目已确认部分

## 串口参数

```text
115200 baud, 8 data bits, no parity, 1 stop bit
address = 0x01
fixed checksum/end byte = 0x6B
```

## 控制命令

```text
使能：01 F3 AB 01 00 6B
失能：01 F3 AB 00 00 6B
急停：01 FE 98 00 6B
速度模式：01 F6 DIR SPEED_H SPEED_L ACC 00 6B
位置模式：01 FD DIR SPEED_H SPEED_L ACC P3 P2 P1 P0 MODE 00 6B
```

方向字节：`01=CW`，`00=CCW`。当前电机可以执行控制命令，但不返回ACK。

## 只读命令

```text
版本：01 1F 6B，预期5字节
状态：01 3A 6B，预期4字节
编码器：01 31 6B，预期5字节
速度：01 35 6B，预期6字节
位置：01 36 6B，预期8字节
```

有效响应必须至少满足：首字节为地址`01`，长度符合命令，末字节为`6B`。

## 当前硬件限制

PL2303和FTDI模块已经双向交叉验证正常，但连接电机后所有只读命令均无返回。
在更换电机侧通信小板前，程序必须使用纯发送控制路径，不能把超时当作控制失败。

## 电气安全

- 当前电机安装RS485小板，`R/A/H`和`T/B/L`对应A/B；
- `UART_FUN`不改变物理接口类型；
- RS485连接不接隔离侧GND；
- 树莓派GPIO不能直接连接A/B；
- 只有安装正确TTL小板且确认3.3V兼容后，才能使用GPIO UART并连接COM共地。

