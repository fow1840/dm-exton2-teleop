#!/usr/bin/env python3
"""以 pre-T23 现场版为基底，仅叠加 T03 lock-step（movej_canfd 逐帧应答解锁）。"""
import sys

src, dst = sys.argv[1], sys.argv[2]
text = open(src, encoding='utf-8').read()

pairs = [
    # ① _feed：status 解锁条件扩到 canfd
    (
        "                if msg.get('command') == 'movej' and 'status' in msg:",
        "                if 'status' in msg and msg.get('command') in (\n"
        "                        'movej', 'movej_canfd'):",
    ),
    # ② _tx_loop：is_movej → is_motion（canfd 同样在途等待）
    (
        "                    is_movej = payload.get('command') == 'movej'\n"
        "                    if is_movej and waiting_movej_seq is not None:",
        "                    # movej 与 movej_canfd 都走 lock-step（T03）：status 应答\n"
        "                    # 到达前不发下一条运动帧，杜绝程序重叠被拒。\n"
        "                    is_motion = payload.get('command') in ('movej', 'movej_canfd')\n"
        "                    if is_motion and waiting_movej_seq is not None:",
    ),
    (
        "                    else:\n"
        "                        if is_movej:\n"
        "                            with self._movej_state_lock:\n"
        "                                sent_after_seq = self._movej_ack_seq\n"
        "                        self._send_now(payload)\n"
        "                        if is_movej:",
        "                    else:\n"
        "                        if is_motion:\n"
        "                            with self._movej_state_lock:\n"
        "                                sent_after_seq = self._movej_ack_seq\n"
        "                        self._send_now(payload)\n"
        "                        if is_motion:",
    ),
    # ③ 启动日志文案
    (
        "        if self.control_mode == 'movej':\n"
        "            self.get_logger().info(\n"
        "                'movej lock-step 已启用：仅 movej status 应答解锁，'\n"
        "                '300ms 超时兜底'\n"
        "            )",
        "        if self.control_mode in ('movej', 'movej_canfd'):\n"
        "            self.get_logger().info(\n"
        "                'lock-step 已启用：仅运动命令 status 应答解锁，'\n"
        "                '300ms 超时兜底'\n"
        "            )",
    ),
]

for old, new in pairs:
    if text.count(old) != 1:
        sys.exit(f'补丁锚点不唯一或缺失: {old[:60]!r} count={text.count(old)}')
    text = text.replace(old, new)

open(dst, 'w', encoding='utf-8').write(text)
print('patched ->', dst)
