#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ubx_probe.py - UBX 配置能力探针（面向 NEO-M8N-0-01 / PROTVER 15.00）

用途：把"UBX 无法配置"拆成三种可区分的情况：
  1) 接收机根本不认识该消息  -> 完全无应答（UBX 对未知 msgId 静默丢弃）
  2) 认识但参数非法/不支持  -> 回 UBX-NAK 0x05 0x00，payload 第 2 字节是被拒消息 id
  3) 配置成功              -> 回 UBX-ACK 0x05 0x01

重要说明（避免误判）：
  · poll（0 长度请求）的正确回包是"该消息本体"，不是 ACK。本脚本对 poll 只要求
    "有本体应答"，不要求 ACK。
  · CFG-MSG / CFG-CFG 没有 0 长度 poll 形式（CFG-MSG 需 2 字节 cls/id），
    空 poll 被 NAK 属**正常**，不代表不支持。

用法：
    python3 tools/ubx_probe.py COM5 --baud 9600
    python3 tools/ubx_probe.py /dev/ttyUSB0 --baud 115200
    python3 tools/ubx_probe.py COM5 --polls-only        # 只读，不写任何配置
    python3 tools/ubx_probe.py COM5 --write-only        # 跳过只读段，直接写测试

依赖：pip install pyserial
前置：ESP32 处于 GNSS_DISABLE_FOR_UCENTER=1（UART 引脚已释放），u-center 已断开。
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial：pip install pyserial")

SYNC = b"\xB5\x62"
GNSS_NAMES = ["GPS", "SBAS", "GAL", "BDS", "IMES", "QZSS", "GLO", "IRNSS"]


def checksum(body: bytes) -> bytes:
    ca = cb = 0
    for b in body:
        ca = (ca + b) & 0xFF
        cb = (cb + ca) & 0xFF
    return bytes((ca, cb))


def ubx(cls: int, mid: int, payload: bytes = b"") -> bytes:
    body = bytes((cls, mid, len(payload) & 0xFF, len(payload) >> 8)) + payload
    return SYNC + body + checksum(body)


def read_frames(ser, timeout=1.0):
    frames, buf = [], bytearray()
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
            body = bytes(buf[2:6 + plen])
            if checksum(body) == bytes(buf[total - 2:total]):
                frames.append((buf[2], buf[3], bytes(buf[6:6 + plen])))
                del buf[:total]
            else:
                del buf[:2]
    return frames


def describe(frames, cls, mid, is_poll=False):
    acked = naked = got = None
    for fcls, fmid, pl in frames:
        if fcls == 0x05 and fmid == 0x01 and len(pl) >= 2 and pl[0] == cls and pl[1] == mid:
            acked = True
        elif fcls == 0x05 and fmid == 0x00 and len(pl) >= 2 and pl[0] == cls and pl[1] == mid:
            naked = True
        elif fcls == cls and fmid == mid:
            got = pl

    if is_poll:
        if got is not None:
            print("    -> 有应答（本体 %d 字节）" % len(got))
        else:
            print("    -> 无应答（该固件可能没有这条消息，或链路不通）")
    else:
        if acked:
            print("    -> ACK")
        elif naked:
            print("    -> NAK（消息存在，参数/能力被拒）")
        elif got is not None:
            print("    -> 有本体应答，但无 ACK/NAK")
        else:
            print("    -> 无应答")
    return acked, naked, got


def send_and_wait(ser, cls, mid, payload=b"", timeout=0.6, label="", is_poll=False):
    if label:
        print("  [%s] %s  cls=0x%02X id=0x%02X len=%d"
              % ("poll" if is_poll else "send", label, cls, mid, len(payload)))
    ser.reset_input_buffer()
    ser.write(ubx(cls, mid, payload))
    ser.flush()
    return describe(read_frames(ser, timeout), cls, mid, is_poll)


# ---------------- 回读解码 ----------------
def decode_prt(pl):
    if len(pl) < 20:
        return
    port = pl[0]
    mode = int.from_bytes(pl[4:8], "little")
    baud = int.from_bytes(pl[8:12], "little")
    inp = int.from_bytes(pl[12:14], "little")
    out = int.from_bytes(pl[14:16], "little")
    charlen = {0: 5, 1: 6, 2: 7, 3: 8}.get((mode >> 6) & 0x3, "?")
    parity = {0: "Even", 1: "Odd", 4: "None"}.get((mode >> 9) & 0x7, "?")
    stop = {0: 1, 1: 1.5, 2: 2, 3: 0.5}.get((mode >> 12) & 0x3, "?")
    print("    portID=%d baud=%d mode=0x%08X (%s%s%s) inProto=0x%04X outProto=0x%04X"
          % (port, baud, mode, charlen, parity, stop, inp, out))
    print("    inProto 位: %s" % " ".join(
        n for b, n in ((0x01, "UBX"), (0x02, "NMEA"), (0x04, "RTCM2"), (0x08, "RTCM3")) if inp & b))
    print("    outProto 位: %s" % " ".join(
        n for b, n in ((0x01, "UBX"), (0x02, "NMEA"), (0x04, "RTCM2"), (0x08, "RTCM3")) if out & b))


def decode_rate(pl):
    if len(pl) < 6:
        return
    meas = int.from_bytes(pl[0:2], "little")
    nav = int.from_bytes(pl[2:4], "little")
    tref = int.from_bytes(pl[4:6], "little")
    print("    meas=%dms (%s Hz) navRate=%d timeRef=%d"
          % (meas, ("%.1f" % (1000.0 / meas)) if meas else "?", nav, tref))


def decode_nav5(pl):
    if len(pl) < 36:
        return
    print("    mask=0x%04X dynModel=%d fixMode=%d fixedAlt=%dm"
          % (int.from_bytes(pl[0:2], "little"), pl[2], pl[3],
             int.from_bytes(pl[4:8], "little")))


def decode_gnss(pl):
    if len(pl) < 4:
        return
    print("    numTrkChHw=%d numTrkChUse=%d numConfigBlocks=%d" % (pl[1], pl[2], pl[3]))
    for i in range(pl[3]):
        b = 4 + i * 8
        if b + 8 > len(pl):
            break
        gid = pl[b]
        flags = int.from_bytes(pl[b + 4:b + 8], "little")
        print("      %-5s id=%d resTrkCh=%-2d maxTrkCh=%-2d flags=0x%08X enable=%d"
              % (GNSS_NAMES[gid] if gid < 8 else "?", gid, pl[b + 1], pl[b + 2],
                 flags, flags & 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--polls-only", action="store_true", help="只读，不写配置")
    ap.add_argument("--write-only", action="store_true", help="跳过只读段")
    ap.add_argument("--skip-gnss", action="store_true", help="跳过 CFG-GNSS 写入")
    ap.add_argument("--no-save", action="store_true", help="不执行 CFG-CFG 保存（只改 RAM/BBR，掉电即恢复）")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        print("== 端口 %s @ %d ==" % (args.port, args.baud))
        time.sleep(0.3)
        ser.reset_input_buffer()

        if not args.write_only:
            print("\n[1] MON-VER")
            _, _, pl = send_and_wait(ser, 0x0A, 0x04, label="MON-VER", is_poll=True)
            if pl:
                print("    swVersion=%r hwVersion=%r"
                      % (pl[:30].rstrip(b"\x00").decode("ascii", "replace"),
                         pl[30:40].rstrip(b"\x00").decode("ascii", "replace")))
                print("    extensions=%s" % [e.decode("ascii", "replace")
                                             for e in pl[40:].split(b"\x00") if e])
            else:
                sys.exit("    MON-VER 无应答：波特率不对，或 UBX 输出被关闭")

            print("\n[2] 只读回读（不改变任何配置）")
            plan = [
                ("CFG-PRT",  0x06, 0x00, decode_prt),
                ("CFG-RATE", 0x06, 0x08, decode_rate),
                ("CFG-NAV5", 0x06, 0x24, decode_nav5),
                ("CFG-GNSS", 0x06, 0x3E, decode_gnss),
                ("CFG-ANT",  0x06, 0x13, None),
                ("CFG-SBAS", 0x06, 0x16, None),
                ("CFG-PM2",  0x06, 0x3B, None),
                ("CFG-ODO",  0x06, 0x1E, None),
            ]
            for name, cls, mid, dec in plan:
                _, _, pl = send_and_wait(ser, cls, mid, label=name, is_poll=True)
                if pl is None:
                    continue
                if dec:
                    dec(pl)
                else:
                    print("    payload(%d)=%s" % (len(pl), pl.hex()))

            print("\n    注：CFG-MSG / CFG-CFG 无 0 长度 poll 形式，空 poll 被 NAK 属正常，故不在此探测")

        if args.polls_only:
            print("\n--polls-only：结束，未做任何写入")
            return

        print("\n[3] CFG-RATE 1000ms（最小写测试）")
        send_and_wait(ser, 0x06, 0x08, bytes((0xE8, 0x03, 0x01, 0x00, 0x01, 0x00)),
                      label="CFG-RATE meas=1000 nav=1 timeRef=GPS")

        print("\n[4] CFG-MSG 写测试")
        for label, pl in (
            ("NMEA-GGA=1Hz(UART1)", bytes((0xF0, 0x00, 0x00, 0x01, 0, 0, 0, 0))),
            ("NMEA-GSV=1Hz(UART1)", bytes((0xF0, 0x03, 0x00, 0x01, 0, 0, 0, 0))),
            ("NAV-PVT=1Hz(UART1)", bytes((0x01, 0x07, 0x00, 0x01, 0, 0, 0, 0))),
            ("NAV-SAT=0（PROTVER15 可能不存在）", bytes((0x01, 0x35, 0x00, 0x00, 0, 0, 0, 0))),
        ):
            send_and_wait(ser, 0x06, 0x01, pl, label=label)

        if not args.skip_gnss:
            print("\n[5] CFG-GNSS 变体测试（逐个换参数，找出被拒的那一项）")

            def gnss_payload(blocks, trk_hw=32, trk_use=32):
                p = bytearray((0x00, trk_hw & 0xFF, trk_use & 0xFF, len(blocks)))
                for gid, res, mx, flags in blocks:
                    p += bytes((gid, res, mx, 0x00)) + flags.to_bytes(4, "little")
                return bytes(p)

            GPS, GLO, BDS = 0, 6, 3
            variants = [
                # flags 取模块自身回读值 0x01010001（不是 0x00010001）
                ("A: GPS+GLO flags=0x01010001（照模块回读值）",
                 gnss_payload([(GPS, 8, 16, 0x01010001), (GLO, 8, 14, 0x01010001)])),
                ("B: GPS+GLO+BDS flags=0x01010001",
                 gnss_payload([(GPS, 8, 16, 0x01010001), (GLO, 8, 14, 0x01010001),
                               (BDS, 8, 16, 0x01010001)])),
                ("C: GPS+BDS flags=0x01010001（固件默认模式）",
                 gnss_payload([(GPS, 8, 16, 0x01010001), (BDS, 8, 16, 0x01010001)])),
                ("D: GPS+GLO flags=0x00010001（固件当前写法）",
                 gnss_payload([(GPS, 8, 16, 0x00010001), (GLO, 8, 14, 0x00010001)])),
                ("E: GPS 单星座 flags=0x01010001",
                 gnss_payload([(GPS, 8, 16, 0x01010001)])),
            ]
            accepted = []
            for label, pl in variants:
                acked, naked, _ = send_and_wait(ser, 0x06, 0x3E, pl, label=label)
                if acked:
                    accepted.append(label.split(":")[0])
            print("    ** ACK 的变体：%s **" % (", ".join(accepted) if accepted else "无（全部 NAK/超时）"))

            print("\n[5b] CFG-GNSS 回读（确认是否真的写进去了）")
            _, _, pl = send_and_wait(ser, 0x06, 0x3E, label="CFG-GNSS", is_poll=True)
            if pl:
                decode_gnss(pl)

        if args.no_save:
            print("\n[6] 跳过 CFG-CFG 保存（--no-save，掉电后恢复原配置）")
        else:
            print("\n[6] CFG-CFG 保存到 BBR+Flash")
            send_and_wait(ser, 0x06, 0x09,
                          bytes((0, 0, 0, 0, 0xFF, 0x03, 0, 0, 0, 0, 0, 0, 0x03)),
                          label="CFG-CFG saveMask=0x03FF deviceMask=BBR|Flash")

        print("\n完成。判读：\n"
              "  · poll 无应答        -> 该固件没有这条消息（PROTVER 15 常见：NAV-SAT/MON-RF/CFG-VALSET）\n"
              "  · 某变体 ACK         -> 以该变体的 flags/块组合为准改固件\n"
              "  · 全部 NAK           -> 该固件禁止改星座，只能吃 OTP 默认\n"
              "  · ACK 但回读没变      -> Flash/BBR 写入未生效")


if __name__ == "__main__":
    main()
