# Pont TCP Mac ↔ Ventuno Q ↔ motors

Des del 18-10-2026, el Mac ja no parla directament amb el convertidor USB-CAN. Abans ho feia per `/dev/cu.usbmodem…`, però el driver USB de macOS afegia massa latència. Ara el convertidor TA_CAN està connectat a un **Arduino Ventuno Q**. Al Ventuno hi corre un daemon que rep les ordres del Mac per TCP i les envia al bus CAN.

Estat validat: pont complet amb el motor 06, moviment fluid, ping de 7–9 ms per WiFi. El daemon corre sota systemd amb `--ids 1,2,3,4,5,6`. **Pendent: provar els 6 motors físics junts.**

## Arquitectura

```
 Mac (app NeuroSpine6)                         Ventuno Q (Ubuntu 24.04)              Bus CAN
 ┌──────────────────────────────┐   TCP 5000   ┌─────────────────────────────┐  USB   ┌─────────┐
 │ viewer_cobot / MotorController│  trames de  │ motor_daemon (systemd)      │ 13 B   │ TA_CAN  │──▶ motors 1..6
 │   └ MotorBus (sitodrive.py)   │  12 bytes   │  · allowlist --ids          │◀──────▶│ convert.│
 │      └ subprocés _daemon.py   │◀───────────▶│  · watchdog 1 s             │ttyACM0 └─────────┘
 │          └ _TcpLink           │  WiFi/Eth   │  · reconnexió sèrie         │
 └──────────────────────────────┘              └─────────────────────────────┘
```

- **L'app no ha canviat.** `send_q`, `MotorBus`, `MotorController` i la GUI són els mateixos. Només ha canviat el port del robot: si comença per `tcp://`, el subprocés `_daemon.py` fa servir `_TcpLink` en lloc de `serial.Serial`.
- **D'on surt el port:** `robot_port` de Preferències. Si no hi ha preferències desades, es fa servir `tcp://192.168.1.140:5000`. El port es llegeix en obrir l'app, així que si el canvies cal reiniciar-la. Si poses `/dev/cu.usbmodem…`, es torna a fer servir el convertidor USB directament al Mac.
- **Format de les trames:** són les mateixes trames del fabricant (`88 05 06 id cmd + 8 dades`). Per TCP viatgen sense la capçalera fixa `88 05 06`.

| Fitxer | On corre | Què fa |
|---|---|---|
| [motor_daemon.cpp](motor_daemon.cpp) | Ventuno | Pont TCP ↔ sèrie (`serial::Serial`, la mateixa capa que la llibreria TA_CAN) |
| [safe_tcp_test.py](safe_tcp_test.py) | Mac | Prova sense app: ping + un sol RESET al motor 06 + resposta B0 |
| [../src/hardware/_daemon.py](../src/hardware/_daemon.py) | Mac | Classe `_TcpLink`: converteix les trames, envia un ping cada 200 ms i es reconnecta sola |

### Ethernet amb fallback a WiFi

El Ventuno és accessible per dues adreces: Ethernet directe (`10.42.0.2`) i WiFi (`192.168.1.140`). Si el port de l'app és `tcp://10.42.0.2:5000`, `_TcpLink` es connecta així:

1. Prova primer l'adreça del port de l'app (Ethernet), amb un timeout d'**1,5 s**.
2. Si no respon (cable desconnectat, host unreachable o timeout), prova en ordre les adreces de `TCP_FALLBACK_HOSTS`, al principi de [_daemon.py](../src/hardware/_daemon.py). Ara la llista només té `tcp://192.168.1.140:5000` (WiFi), amb un timeout de **3 s**.
3. Al log del Mac s'hi veu per on s'ha connectat: `[tcp] connectat per Ethernet (…)` o `[tcp] fallback a WiFi (…)`. Si no respon cap adreça, l'app dona un error amb la llista de totes les adreces provades.

Per afegir-hi més adreces, afegeix-les a `TCP_FALLBACK_HOSTS`. Si vols que surtin amb nom al log, afegeix-les també a `TCP_LINK_NAMES`. La suma dels timeouts ha de quedar per sota dels 8 s de `READY_TIMEOUT_S` (a `sitodrive.py`).

Si l'Ethernet cau amb l'app en marxa, el Mac es reconnecta sol en ~1 s i torna a provar les adreces en el mateix ordre: primer l'Ethernet, després el WiFi. Un cop connectat pel WiFi, **no torna automàticament a l'Ethernet**, ni que reconnectis el cable. Hi torna en la propera reconnexió o quan reinicies l'app. Mentre dura la reconnexió, els motors mantenen l'última ordre.

### Protocol TCP (12 bytes fixos, en tots dos sentits)

| byte | camp | valors |
|---|---|---|
| 0 | magic | `0xA5` |
| 1 | tipus | `0x01` CAN_TX (Mac→Ventuno) · `0x02` CAN_RX (feedback) · `0x03` PING/PONG · `0x04` ERROR |
| 2 | motor_id | 1..6 |
| 3 | cmd / fb_type | `Mode` (0x00–0x45) o `FeedbackType` (0xB0–0xB3) |
| 4–11 | dades | 8 bytes big-endian, igual que els `encode_*` de `sitodrive.py` |

Codis d'error (byte 4 d'una trama ERROR): `0x01` el port sèrie del Ventuno ha caigut · `0x02` error d'escriptura al port sèrie.

### Comportament de seguretat del daemon

- **`--ids` és una llista d'ids permesos.** Les ordres per a un motor que no hi és es descarten (`tx_filtered` al diag). La primera vegada que passa amb cada motor, surt al log `filtering motor_id=X (not in allowlist)`.
- **`--allow IP`**: si es posa, el daemon només accepta connexions d'aquesta IP. **Ara no es fa servir** (des del 18-10-2026), perquè el Mac s'hi connecta per Ethernet o per WiFi, i en cada cas té una IP diferent. Sense `--allow`, qualsevol equip de la xarxa pot enviar ordres als motors: tingues el Ventuno en una xarxa de confiança.
- **Watchdog:** si el Mac no envia res en 1 s, el daemon tanca la connexió. El Mac envia un ping cada 200 ms.
- **`--failsafe none` (per defecte, i és el que volem):** si es perd el Mac, els motors **mantenen l'última ordre**. `--failsafe reset` existeix, però en RESET el braç pot quedar sense força i caure per gravetat: **no l'activis sense haver-ho provat abans**.
- **Si el port sèrie cau** (cable, convertidor reiniciat), el daemon el torna a obrir cada segon.

## Gestió del daemon al Ventuno

```bash
ssh arduino@192.168.1.140

systemctl status motor-daemon          # ha de dir active (running)
sudo systemctl restart motor-daemon    # reiniciar
sudo systemctl stop motor-daemon       # aturar (p. ex. per fer servir safe_test.cpp directe)
sudo systemctl start motor-daemon
systemctl cat motor-daemon             # veure la línia ExecStart amb els arguments actuals
```

`ExecStart` actual (18-10-2026):

```ini
ExecStart=/home/arduino/neurospine6/motor_daemon/motor_daemon --port /dev/ttyACM0 --baud 921600 --tcp-port 5000 --ids 1,2,3,4,5,6
```

(Abans hi havia `--allow 192.168.1.144`. L'hem tret perquè el Mac s'hi pugui connectar també per Ethernet.)

Per canviar els arguments (`--ids`, `--allow`, `--failsafe`…):

```bash
sudo systemctl edit --full motor-daemon   # editar ExecStart
sudo systemctl daemon-reload
sudo systemctl restart motor-daemon
```

### Logs (journalctl)

```bash
journalctl -u motor-daemon -f                      # en directe
journalctl -u motor-daemon -n 200                  # últimes 200 línies
journalctl -u motor-daemon --since "10 min ago"
journalctl -u motor-daemon -b                      # des de l'últim arrencament
journalctl -u motor-daemon -f | grep -v diag:      # sense les línies diag
```

En arrencar ha de sortir `escoltant TCP al port 5000 (failsafe=none, ids=1,2,3,4,5,6)`. Cada 5 s surt una línia `diag:`:

| Comptador | Què vol dir |
|---|---|
| `tx_frames` | Ordres enviades al bus. Amb l'app activa a 100 Hz i 6 motors, unes 3000 per cada 5 s |
| `tx_filtered` | Ordres descartades per estar fora de `--ids` |
| `packets_parsed` / `bytes_read` | Feedback rebut dels motors |
| `bytes_skipped` | Soroll a la línia sèrie (uns quants en arrencar és normal) |
| `write_errors` | Errors en escriure al convertidor (ha de ser 0) |
| `tcp_bad` | Trames TCP mal formades (ha de ser 0) |
| `pings` | Pings rebuts del Mac (~5/s) |

Al Mac, el subprocés escriu una línia `[motor-daemon] diag:` semblant, amb `bytes_read`, `packets_parsed`…

### Actualitzar el daemon

```bash
# Mac
scp ~/Desktop/Neurospine6/ventuno/motor_daemon.cpp arduino@192.168.1.140:~/neurospine6/motor_daemon/
# Ventuno
cd ~/neurospine6/motor_daemon
g++ -O2 -std=c++17 -pthread motor_daemon.cpp -o motor_daemon \
    -I/usr/local/include -L/usr/local/lib -lserial -Wl,-rpath,/usr/local/lib
sudo systemctl restart motor-daemon
```

Si `g++` dona l'error `Text file busy`, atura el servei abans de compilar. Si surt un error de símbol amb `setBaudrate`, és el conflicte amb `libserial-dev` del sistema: el `-Wl,-rpath,/usr/local/lib` el resol.

### Diagnòstic ràpid

| Símptoma | Causa probable |
|---|---|
| L'app no connecta / `Connection refused` | Daemon aturat (`systemctl status`), IP incorrecta o firewall (`ufw`) |
| El daemon diu `connexió rebutjada de X` | Només passa amb `--allow`: la IP del Mac no hi coincideix |
| Al Mac surt `[tcp] fallback a WiFi` amb el cable posat | L'Ethernet no respon: revisa el cable, la connexió compartida al Mac i `ping 10.42.0.2` |
| `tx_frames` puja però `bytes_read` no | El convertidor o els motors no responen (alimentació, cable CAN) |
| Puja `tx_filtered` | El motor no és a `--ids` |
| `watchdog: cap trama del client` | El Mac s'ha penjat o el WiFi ha perdut paquets més d'1 s |
| Moviment a trossos amb `tx_frames` regular | Jitter del WiFi o temporització de l'app: prova-ho per Ethernet |

## Checklist de seguretat per a la primera connexió (o després de canvis)

1. **Ventuno:** `systemctl status motor-daemon` diu active, i el log diu `failsafe=none` i les `ids` que esperes.
2. **Prova sense app:** `venv/bin/python ventuno/safe_tcp_test.py` ha d'acabar amb `RESULTAT: OK ✅`. El RESET deixa el motor 06 lliure, així que no el facis amb el braç sota càrrega.
3. **Robot:** alimentació correcta, **STOP físic a mà**, espai de treball lliure i ningú dins l'abast del braç.
4. **⚠️ En connectar, tots els motors de `--ids` van directament a la posició que té l'app.** En obrir l'app, aquesta posició és zero (`self.q = 0`), i el moviment es fa sense rampa a `default_current_mA`. Abans, posa el braç a prop de la posició zero.
5. **Si cau la connexió** (WiFi, Mac), els motors mantenen l'última ordre. L'aturada d'emergència és sempre l'**STOP físic**, no l'app.
6. **Els primers segons:** al `journalctl -f`, comprova que `tx_frames` i `packets_parsed` pugen i que `write_errors=0`.
7. **Abans de fer proves de velocitat serioses:** connecta el Ventuno per **Ethernet**. El WiFi (`wlp3s0`) pot provocar pics de latència.
8. **Per provar només alguns motors:** canvia `--ids` (p. ex. `--ids 6`) i reinicia el servei. Els motors que no hi són no reben res, encara que l'app els enviï ordres.

## Dades de referència

- Ventuno Q: `arduino@192.168.1.140` (WiFi) / `10.42.0.2` (Ethernet directe), Ubuntu 24.04, convertidor a `/dev/ttyACM0` @ 921600
- Mac: 192.168.1.144 per WiFi (el daemon ja no filtra per IP)
- Codi del fabricant: `~/neurospine6/TA_CAN-1d11e8524cd6c597b0be2fa36e8bfaf1/TA_CAN/C++/Linux/`; llibreria serial a `~/serial_vendor`, instal·lada a `/usr/local/lib`
- Firmware del motor 06 (resposta B0): versió 1757708975, data 20250724
