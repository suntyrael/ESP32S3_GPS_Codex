#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ubx_probe.py - UBX 配置能力探针（面向 NEO-M8N-0-01 / PROTVER 15.00）

用途：把"UBX 无法配置"拆成三种可区分的情况：
  1) 接收机根本不认识该消息  -> 完全无应答（UBX 对未知 msgId 静默丢弃）
  2) 认识但参数非法/不支持  -> 回 UBX-NAK 0x05 0x00，payload 第 2 字节是被拒消息 id
  3) 配置成功              -> 回 UBX-ACK 0x05 0x01

用法：
    python3 tools/ubx_probe.py COM5 --baud 9600
    python3 tools/ubx_probe.py /dev/ttyUSB0 --baud 115200
    python3 tools/ubx_probe.py COM5 --polls-only        # 只读，不写任何配置
    python3 tools/ubx_probe.py COM5 --skip-gnss         # 跳过 CFG-GNSS 写入

依赖：pip install pyserial
注意：请先确保 ESP32 处于 GNSS_DISABLE_FOR_UCENTER=1（UART 引脚已释放），
      否则双主机会互相抢线，探针结果不可信。
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial：pip install pyserial")

SYNC = b"\xB5\x62"


def checksum(frame_body: bytes) -> bytes:
    ca = cb = 0
    for b in frame_body:
        ca = (ca + b) & 0xFF
        cb = (cb + ca) & 0xFF
    return bytes((ca, cb))


def ubx(cls: int, mid: int, payload: bytes = b"") -> bytes:
    body = bytes((cls, mid, len(payload) & 0xFF, len(payload) >> 8)) + payload
    return SYNC + body + checksum(body)


def read_frames(ser, timeout=1.0):
    """读串口并按 UBX 帧切分，返回 [(cls, mid, payload)]；NMEA 被丢弃。"""
    frames = []
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            continue
        time.sleep(0.01)
        while True:
            i = buf.find(SYNC)
            if i < 0:
                buf.clear()
                break
            if i > 0:
                del buf[:i]
            if len(buf) < 8:
                break
            plen = buf[4] | (buf[5] << 8)
            total = 8 + plen
            if len(buf) < total:
                break
            body = bytes(buf[2:4 + 2 + plen])
            if checksum(body) == bytes(buf[total - 2:total]):
                frames.append((buf[2], buf[3], bytes(buf[6:6 + plen])))
                del buf[:total]
            else:
                del buf[:2]          # 校验失败：扔掉一个 sync 重新对齐
    return frames


def describe(frames, expect_cls, expect_mid, verbose=True):
    """从一帧集合里判定 ACK / NAK / 是否收到目标消息。"""
    acked = naked = got = None
    for cls, mid, pl in frames:
        if cls == 0x05 and mid == 0x01 and len(pl) >= 2:
            if pl[0] == expect_cls and pl[1] == expect_mid:
                acked = True
        elif cls == 0x05 and mid == 0x00 and len(pl) >= 2:
            if pl[0] == expect_cls and pl[1] == expect_mid:
                naked = True
        elif cls == expect_cls and mid == expect_mid:
            got = pl
    if verbose:
        if acked:
            print("    -> ACK   （消息被接受）")
        elif naked:
            print("    -> NAK   （消息存在，但参数/能力被拒）")
        else:
            print("    -> 无应答（该固件可能根本没有这条消息，或链路不通）")
    return acked, naked, got


def send_and_wait(ser, cls, mid, payload=b"", timeout=0.6, label=""):
    if label:
        print("  [发送] %s  cls=0x%02X id=0x%02X len=%d" % (label, cls, mid, len(payload)))
    ser.reset_input_buffer()
    ser.write(ubx(cls, mid, payload))
    ser.flush()
    return describe(read_frames(ser, timeout), cls, mid)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--polls-only", action="store_true", help="只读，不写配置")
    ap.add_argument("--skip-gnss", action="store_true", help="跳过 CFG-GNSS 写入")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        print("== 端口 %s @ %d ==" % (args.port, args.baud))
        time.sleep(0.3)
        ser.reset_input_buffer()

        # ---------- 1. 身份 ----------
        print("\n[1] MON-VER")
        acked, naked, pl = send_and_wait(ser, 0x0A, 0x04, label="poll MON-VER")
        if pl:
            sw = pl[:30].rstrip(b"\x00").decode("ascii", "replace")
            hw = pl[30:40].rstrip(b"\x00").decode("ascii", "replace") if len(pl) > 30 else "?"
            print("    swVersion=%r hwVersion=%r" % (sw, hw))
            exts = pl[40:].split(b"\x00")
            print("    extensions=%s" % [e.decode("ascii", "replace") for e in exts if e])
        else:
            sys.exit("    MON-VER 无应答：波特率不对，或 UBX 输入被关闭，先解决链路")

        # ---------- 2. 各 CFG 消息是否被认识（poll 无 payload）----------
        print("\n[2] CFG 消息存在性探测（poll，不改变任何配置）")
        polls = [
            ("CFG-PRT",   0x06, 0x00),
            ("CFG-MSG",   0x06, 0x01),
            ("CFG-RATE",  0x06, 0x08),
            ("CFG-CFG",   0x06, 0x09),
            ("CFG-NAV5",  0x06, 0x24),
            ("CFG-GNSS",  0x06, 0x3E),
            ("CFG-ANT",   0x06, 0x13),
            ("CFG-SBAS",  0x06, 0x16),
            ("CFG-PM2",   0x06, 0x3B),
            ("CFG-ODO",   0x06, 0x1E),
        ]
        supported = {}
        for name, cls, mid in polls:
            print("  [poll] %s" % name)
            acked, naked, pl = send_and_wait(ser, cls, mid)
            supported[name] = pl is not None
            if pl is not None:
                print("    payload(%d)=%s" % (len(pl), pl.hex()))

        if args.polls_only:
            print("\n--polls-only：结束，未做任何写入")
            return

        # ---------- 3. 最小写测试：CFG-RATE ----------
        print("\n[3] CFG-RATE 1000ms（最小写测试）")
        send_and_wait(ser, 0x06, 0x08, bytes((0xE8, 0x03, 0x01, 0x00, 0x01, 0x00)),
                      label="CFG-RATE meas=1000 nav=1 timeRef=1")

        # ---------- 4. CFG-MSG：NMEA 与"本固件可能不存在"的 NAV-SAT ----------
        print("\n[4] CFG-MSG 写测试")
        send_and_wait(ser, 0x06, 0x01, bytes((0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00)),
                      label="CFG-MSG NMEA-GGA rate(UART1)=1")
        send_and_wait(ser, 0x06, 0x01, bytes((0xF0, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00)),
                      label="CFG-MSG NMEA-GSV rate(UART1)=1")
        send_and_wait(ser, 0x06, 0x01, bytes((0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00)),
                      label="CFG-MSG NAV-PVT rate(UART1)=1")
        send_and_wait(ser, 0x06, 0x01, bytes((0x01, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)),
                      label="CFG-MSG NAV-SAT（PROTVER15 可能不存在，用于验证）")

        # ---------- 5. CFG-GNSS 三变体 ----------
        if not args.skip_gnss:
            print("\n[5] CFG-GNSS 变体测试（GPS+GLO，逐个换参数找被拒的那一项）")

            def gnss_payload(blocks, trk_hw=32, trk_use=32):
                p = bytearray((0x00, trk_hw & 0xFF, trk_use & 0xFF, len(blocks)))
                for gid, res, mx, flags in blocks:
                    p += bytes((gid, res, mx, 0x00))
                    p += flags.to_bytes(4, "little")
                return bytes(p)

            variants = [
                ("A: res=8 max=16 flags=0x00010001（当前固件写法）",
                 gnss_payload([(0, 8, 16, 0x00010001), (6, 8, 14, 0x00010001)])),
                ("B: flags 只保留 enable=0x00000001",
                 gnss_payload([(0, 8, 16, 0x00000001), (6, 8, 14, 0x00000001)])),
                ("C: 只使能 GPS（单星座最小集）",
                 gnss_payload([(0, 8, 16, 0x00000001)])),
            ]
            for label, pl in variants:
                acked, naked, _ = send_and_wait(ser, 0x06, 0x3E, pl, label=label)
                if acked:
                    print("    ** 该变体被接受，后续以它为准 **")
                    break

            print("\n[5b] CFG-GNSS 回读")
            acked, naked, pl = send_and_wait(ser, 0x06, 0x3E, label="poll CFG-GNSS")
            if pl and len(pl) >= 4:
                names = ["GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO", "IRNSS"]
                print("    numTrkChHw=%d numTrkChUse=%d blocks=%d" % (pl[1], pl[2], pl[3]))
                for i in range(pl[3]):
                    b = 4 + i * 8
                    if b + 8 > len(pl):
                        break
                    gid = pl[b]
                    flags = int.from_bytes(pl[b + 4:b + 8], "little")
                    print("    %-5s id=%d res=%d max=%d flags=0x%08X enable=%d"
                          % (names[gid] if gid < 8 else "?", gid, pl[b + 1], pl[b + 2],
                             flags, flags & 1))

        # ---------- 6. 保存 ----------
        print("\n[6] CFG-CFG 保存到 BBR+Flash")
        send_and_wait(ser, 0x06, 0x09, bytes((0, 0, 0, 0, 0xFF, 0x03, 0, 0, 0, 0, 0, 0, 0x03)),
                      label="CFG-CFG saveMask=0x03FF deviceMask=BBR|Flash")

        print("\n完成。判读：\n"
              "  · 某消息 poll 无应答      -> 该固件不支持这条消息（PROTVER 15 常见：NAV-SAT、MON-RF、CFG-VALSET）\n"
              "  · 变体 A NAK、B/C ACK     -> 是 flags/maxTrkCh 参数问题，按通过的变体改固件\n"
              "  · 全部 NAK                -> CFG-GNSS 在该固件被禁用/不支持，只能依赖 OTP 默认星座\n"
              "  · 有 ACK 但回读没变        -> Flash 写入失败或未真正生效，需查 FIS/掉电")


if __name__ == "__main__":
    main()
