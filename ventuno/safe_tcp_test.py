"""
Prova segura del pont TCP del Ventuno Q (ventuno/motor_daemon.cpp), sense l'app.

    1. Es connecta per TCP al daemon.
    2. Envia un PING i comprova que torna el PONG amb les mateixes dades.
    3. Envia UNA sola trama de RESET al motor (per defecte 0x06).
    4. Espera la resposta CAN_RX amb fb_type 0xB0 d'aquest motor i n'imprimeix
       la versió i la data de firmware (igual que safe_test.cpp).

No envia cap ordre de mode ni de posició: el motor no s'ha de moure. Si el
motor estava actiu (POSITION_MODE...), el RESET el deixa lliure (LED blau).

Ús (amb el venv del projecte o qualsevol Python 3):
    python ventuno/safe_tcp_test.py
    python ventuno/safe_tcp_test.py --host 192.168.1.140 --port 5000 --id 6

Opcional, per comprovar l'allowlist --ids del daemon:
    python ventuno/safe_tcp_test.py --test-filter 99
envia també un RESET a un motor_id que NO ha de ser a --ids (ni existir al bus).
El daemon l'ha de descartar: al seu log ha de sortir
"filtering motor_id=99 (not in allowlist)" i tx_filtered ha de pujar.
"""

import argparse
import socket
import struct
import sys
import time

MAGIC = 0xA5
T_CAN_TX, T_CAN_RX, T_PING, T_ERROR = 0x01, 0x02, 0x03, 0x04
FRAME = 12
CMD_RESET = 0x00
FB_RESET_REPLY = 0xB0
PING_PERIOD_S = 0.2                                                             # El daemon talla si no rep res en 1 s
ERROR_NAMES = {0x01: "port sèrie caigut al Ventuno", 0x02: "error d'escriptura sèrie al Ventuno"}


def frame(kind, motor_id, cmd, data):
    assert len(data) == 8
    return bytes([MAGIC, kind, motor_id, cmd]) + bytes(data)


class Link:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = bytearray()
        self.last_tx = time.monotonic()
        self.rx_counts = {}                                                     # (motor_id, fb_type) -> nombre de trames rebudes

    def send(self, data):
        self.sock.sendall(data)
        self.last_tx = time.monotonic()

    def wait_for(self, match, timeout):
        """Llegeix trames fins que match(kind, id, cmd, data) és True o passa el timeout.
        Mentre espera envia PINGs perquè el watchdog del daemon no talli."""
        deadline = time.monotonic() + timeout
        while True:
            while len(self.buf) >= FRAME:
                if self.buf[0] != MAGIC:
                    del self.buf[0]
                    continue
                kind, mid, cmd, data = self.buf[1], self.buf[2], self.buf[3], bytes(self.buf[4:12])
                del self.buf[:FRAME]
                if kind == T_CAN_RX:
                    self.rx_counts[(mid, cmd)] = self.rx_counts.get((mid, cmd), 0) + 1
                elif kind == T_ERROR:
                    print(f"  !! ERROR del daemon: codi 0x{data[0]:02X} ({ERROR_NAMES.get(data[0], '?')})")
                if match(kind, mid, cmd, data):
                    return kind, mid, cmd, data

            now = time.monotonic()
            if now >= deadline:
                return None
            if now - self.last_tx >= PING_PERIOD_S:
                self.send(frame(T_PING, 0, 0, bytes(8)))
            self.sock.settimeout(min(0.05, deadline - now))
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("el daemon ha tancat la connexió")
            self.buf += chunk


def main():
    ap = argparse.ArgumentParser(description="Prova segura del pont TCP del Ventuno")
    ap.add_argument("--host", default="192.168.1.140")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--id", type=lambda s: int(s, 0), default=0x06, help="motor a qui s'envia el RESET (defecte 6)")
    ap.add_argument("--timeout", type=float, default=1.0, help="segons d'espera per cada resposta")
    ap.add_argument("--test-filter", type=lambda s: int(s, 0), metavar="ID",
                    help="envia també un RESET a aquest motor_id, que ha d'estar FORA de --ids")
    args = ap.parse_args()
    ok = True

    print(f"[1] Connectant a {args.host}:{args.port} ...")
    try:
        link = Link(args.host, args.port)
    except OSError as e:
        print(f"  FALLA: no es pot connectar: {e}")
        print("  Comprova que motor_daemon corre, la IP, --allow i el firewall (ufw).")
        return 1
    print("  OK connectat")

    print("[2] PING ...")
    marker = bytes([0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04])
    t0 = time.monotonic()
    link.send(frame(T_PING, 0x42, 0x24, marker))
    pong = link.wait_for(lambda k, m, c, d: k == T_PING and m == 0x42 and c == 0x24 and d == marker, args.timeout)
    if pong:
        print(f"  OK pong en {(time.monotonic() - t0) * 1000:.1f} ms")
    else:
        print("  FALLA: cap pong. Si el daemon diu 'connexió rebutjada', revisa --allow.")
        return 1

    print(f"[3] RESET al motor 0x{args.id:02X} (una sola trama: A5 01 {args.id:02X} 00 55x8) ...")
    t0 = time.monotonic()
    link.send(frame(T_CAN_TX, args.id, CMD_RESET, b"\x55" * 8))
    reply = link.wait_for(lambda k, m, c, d: k == T_CAN_RX and m == args.id and c == FB_RESET_REPLY, args.timeout)

    print(f"[4] Resposta B0 del motor 0x{args.id:02X} ...")
    if reply:
        data = reply[3]
        version, date = struct.unpack(">ii", data)
        print(f"  OK resposta en {(time.monotonic() - t0) * 1000:.1f} ms: {data.hex(' ')}")
        print(f"     versió firmware = {version}   data = {date}")
    else:
        ok = False
        print(f"  FALLA: cap trama B0 del motor 0x{args.id:02X} en {args.timeout} s.")
        print("  Mira al daemon: si tx_frames ha pujat però bytes_read no, el convertidor/motor no respon;")
        print(f"  si tx_filtered ha pujat, el motor 0x{args.id:02X} no és a --ids.")

    if args.test_filter is not None:
        fid = args.test_filter
        print(f"[5] Prova d'allowlist: RESET a motor_id={fid} (ha de ser descartat) ...")
        link.send(frame(T_CAN_TX, fid, CMD_RESET, b"\x55" * 8))
        leaked = link.wait_for(lambda k, m, c, d: k == T_CAN_RX and m == fid and c == FB_RESET_REPLY, args.timeout)
        if leaked:
            ok = False
            print(f"  FALLA: el motor {fid} ha respost, el filtre NO l'ha aturat!")
        else:
            print(f"  OK cap resposta. Al log del daemon hi ha d'haver 'filtering motor_id={fid} (not in allowlist)'.")

    if link.rx_counts:
        resum = ", ".join(f"id{m}/0x{c:02X}×{n}" for (m, c), n in sorted(link.rx_counts.items()))
        print(f"Feedback rebut durant la prova: {resum}")
    link.sock.close()
    print("RESULTAT:", "OK ✅" if ok else "FALLA ❌")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
