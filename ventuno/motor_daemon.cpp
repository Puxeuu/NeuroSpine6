/*
 * motor_daemon.cpp — Pont TCP <-> convertidor TA_CAN per al NeuroSpine6.
 *
 * Corre al Ventuno Q. El Mac s'hi connecta per TCP (port 5000) i li envia
 * trames; el daemon les reenvia al convertidor USB-CAN (/dev/ttyACM0) i
 * retorna al Mac tot el feedback dels motors (B0/B1/B2/B3).
 *
 * Fa servir serial::Serial (~/serial_vendor), la mateixa capa que la classe
 * MotorControl del fabricant, i el mateix format de trama sèrie:
 *
 *     [0x88, 0x05, 0x06, motor_id, cmd, d0..d7]            = 13 bytes
 *
 * PROTOCOL TCP (mida fixa, 12 bytes, en tots dos sentits):
 *
 *     byte 0      magic     0xA5
 *     byte 1      tipus     0x01 CAN_TX  Mac -> Ventuno, envia al bus
 *                           0x02 CAN_RX  Ventuno -> Mac, feedback d'un motor
 *                           0x03 PING    Mac -> Ventuno, el daemon el retorna igual (pong)
 *                           0x04 ERROR   Ventuno -> Mac, d0 = codi d'error
 *     byte 2      motor_id
 *     byte 3      cmd       Mode (0x00..0x45) o FeedbackType (0xB0..0xB3)
 *     byte 4..11  dades     8 bytes, sense tocar (big-endian com ara)
 *
 * Watchdog: si el client no envia res durant --watchdog-ms (per defecte
 * 1000 ms), es considera perdut i es tanca la connexió. El Mac envia PING
 * cada 200 ms per mantenir-la viva.
 *
 * --ids és una ALLOWLIST: només les trames CAN_TX amb motor_id dins la llista
 * arriben al bus. La resta es descarten i es compten a tx_filtered (diag);
 * la primera vegada que es filtra cada motor_id es fa un log. Així es pot
 * provar un sol motor (--ids 6) encara que l'app enviï ordres als 6.
 * El feedback (CAN_RX) no es filtra.
 *
 * Failsafe en perdre el client: --failsafe none (per defecte, els motors
 * mantenen l'última ordre, com passa ara si cau el USB) o --failsafe reset
 * (envia RESET a tots els --ids).
 *
 * Compilació:
 *     g++ -O2 -std=c++17 -pthread motor_daemon.cpp -o motor_daemon \
 *         -I/usr/local/include -L/usr/local/lib -lserial -Wl,-rpath,/usr/local/lib
 *
 * Ús:
 *     ./motor_daemon --port /dev/ttyACM0 --baud 921600 --tcp-port 5000 \
 *                    --ids 1,2,3,4,5,6 --allow 192.168.1.144
 */

#include <serial/serial.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;


// ==============================================================================
// CONSTANTS DEL PROTOCOL
// ==============================================================================

constexpr uint8_t TCP_MAGIC  = 0xA5;
constexpr size_t  TCP_FRAME  = 12;
constexpr uint8_t T_CAN_TX   = 0x01;
constexpr uint8_t T_CAN_RX   = 0x02;
constexpr uint8_t T_PING     = 0x03;
constexpr uint8_t T_ERROR    = 0x04;

constexpr uint8_t SER_HEADER = 0x88;                                            // Igual que Header i ID de motor_control.h
constexpr uint8_t SER_ID0    = 0x05;
constexpr uint8_t SER_ID1    = 0x06;
constexpr size_t  SER_FRAME  = 13;

constexpr uint8_t E_SERIAL_DOWN  = 0x01;                                        // Codis d'error (d0 de T_ERROR)
constexpr uint8_t E_SERIAL_WRITE = 0x02;

constexpr uint8_t CMD_RESET  = 0x00;


// ==============================================================================
// CONFIGURACIÓ I ESTAT GLOBAL
// ==============================================================================

struct Config {
    std::string port = "/dev/ttyACM0";
    uint32_t baud = 921600;
    uint16_t tcp_port = 5000;
    int watchdog_ms = 1000;
    bool failsafe_reset = false;
    std::vector<uint8_t> ids = {1, 2, 3, 4, 5, 6};
    std::array<bool, 256> allowed{};                                            // allowed[id] = id és a --ids (s'omple a main)
    std::string allow_ip;                                                       // Buit = accepta qualsevol IP
};

struct Stats {
    std::atomic<uint64_t> bytes_read{0}, rx_packets{0}, bytes_skipped{0};
    std::atomic<uint64_t> tx_frames{0}, tx_filtered{0}, write_errors{0}, tcp_bad{0}, pings{0};
};

static std::atomic<bool> g_running{true};
static Stats g_stats;
static std::array<bool, 256> g_filter_logged{};                                 // Només el fil principal hi toca

static void log(const std::string& msg) {
    std::cerr << "[motor-daemon] " << msg << std::endl;                         // stderr -> journalctl si corre com a servei
}


// ==============================================================================
// PORT SÈRIE: només el fil lector obre/tanca; les escriptures van amb mutex.
// ==============================================================================

class SerialLink {
public:
    SerialLink(const std::string& port, uint32_t baud) : port_(port), baud_(baud) {}

    bool open() {
        std::lock_guard<std::mutex> lk(wmtx_);
        try {
            if (ser_.isOpen()) ser_.close();
            ser_.setPort(port_);
            ser_.setBaudrate(baud_);
            serial::Timeout to = serial::Timeout::simpleTimeout(20);            // waitReadable() espera com a màxim 20 ms
            ser_.setTimeout(to);
            ser_.open();
            ser_.flushInput();
            ok_ = true;
        } catch (std::exception& e) {
            log(std::string("no s'ha pogut obrir ") + port_ + ": " + e.what());
            ok_ = false;
        }
        return ok_;
    }

    bool ok() const { return ok_; }

    bool write_frame(uint8_t motor_id, uint8_t cmd, const uint8_t* data) {      // Una trama de 13 bytes per write(), igual que el fabricant
        uint8_t f[SER_FRAME] = {SER_HEADER, SER_ID0, SER_ID1, motor_id, cmd};
        std::memcpy(f + 5, data, 8);
        std::lock_guard<std::mutex> lk(wmtx_);
        if (!ok_) return false;
        try {
            return ser_.write(f, SER_FRAME) == SER_FRAME;
        } catch (std::exception& e) {
            log(std::string("error d'escriptura sèrie: ") + e.what());
            ok_ = false;                                                        // El fil lector el tornarà a obrir
            return false;
        }
    }

    size_t read_some(uint8_t* buf, size_t cap) {                                // Només des del fil lector
        if (!ok_) return 0;
        try {
            size_t n = ser_.available();
            if (n == 0) {
                if (!ser_.waitReadable()) return 0;
                n = ser_.available();
                if (n == 0) return 0;
            }
            return ser_.read(buf, std::min(n, cap));
        } catch (std::exception& e) {
            log(std::string("error de lectura sèrie: ") + e.what());
            ok_ = false;
            return 0;
        }
    }

private:
    std::string port_;
    uint32_t baud_;
    serial::Serial ser_;
    std::mutex wmtx_;
    std::atomic<bool> ok_{false};
};


// ==============================================================================
// CLIENT TCP: un sol client alhora. Hi escriuen el fil lector (feedback) i el
// fil principal (pong, errors), per això l'enviament va amb mutex.
// ==============================================================================

class Client {
public:
    void set(int fd) { std::lock_guard<std::mutex> lk(m_); fd_ = fd; }
    void clear()     { std::lock_guard<std::mutex> lk(m_); fd_ = -1; }

    bool send_frame(uint8_t type, uint8_t motor_id, uint8_t cmd, const uint8_t* data) {
        uint8_t f[TCP_FRAME] = {TCP_MAGIC, type, motor_id, cmd};
        std::memcpy(f + 4, data, 8);
        std::lock_guard<std::mutex> lk(m_);
        if (fd_ < 0) return false;
        size_t off = 0;
        while (off < TCP_FRAME) {
            ssize_t r = ::send(fd_, f + off, TCP_FRAME - off, MSG_NOSIGNAL);
            if (r < 0 && errno == EINTR) continue;
            if (r <= 0) {                                                       // Error o SO_SNDTIMEO: el Mac no llegeix
                ::shutdown(fd_, SHUT_RDWR);                                     // El fil principal veurà recv()=0 i tancarà
                fd_ = -1;
                return false;
            }
            off += static_cast<size_t>(r);
        }
        return true;
    }

    void send_error(uint8_t code) {
        uint8_t d[8] = {code, 0, 0, 0, 0, 0, 0, 0};
        send_frame(T_ERROR, 0, 0, d);
    }

private:
    std::mutex m_;
    int fd_ = -1;
};


// ==============================================================================
// FIL LECTOR DEL PORT SÈRIE: extreu trames de 13 bytes i les passa al client.
// També reobre el port si cau (cable, convertidor reiniciat...).
// ==============================================================================

static void serial_reader(SerialLink& ser, Client& cli) {
    std::vector<uint8_t> buf;
    buf.reserve(8192);
    uint8_t chunk[1024];
    bool reported_down = false;
    auto next_retry = Clock::now();

    while (g_running) {
        if (!ser.ok()) {
            if (!reported_down) {
                log("port sèrie caigut, reintentant cada 1 s");
                cli.send_error(E_SERIAL_DOWN);
                reported_down = true;
            }
            if (Clock::now() >= next_retry) {
                if (ser.open()) {
                    log("port sèrie reobert");
                    reported_down = false;
                    buf.clear();
                } else {
                    next_retry = Clock::now() + 1s;
                }
            } else {
                std::this_thread::sleep_for(50ms);
            }
            continue;
        }

        size_t n = ser.read_some(chunk, sizeof(chunk));
        if (n == 0) continue;
        g_stats.bytes_read += n;
        buf.insert(buf.end(), chunk, chunk + n);

        size_t i = 0;
        while (i + SER_FRAME <= buf.size()) {
            if (buf[i] == SER_HEADER && buf[i + 1] == SER_ID0 && buf[i + 2] == SER_ID1) {
                cli.send_frame(T_CAN_RX, buf[i + 3], buf[i + 4], &buf[i + 5]);
                g_stats.rx_packets++;
                i += SER_FRAME;
            } else {
                g_stats.bytes_skipped++;                                        // Soroll: busquem el següent 0x88 05 06
                i++;
            }
        }
        if (i > 0) buf.erase(buf.begin(), buf.begin() + static_cast<long>(i));
        if (buf.size() > 4096) buf.erase(buf.begin(), buf.end() - SER_FRAME * 4);
    }
}


// ==============================================================================
// DIAGNÒSTIC cada 5 s (mateixos noms que el _daemon.py del Mac)
// ==============================================================================

static void maybe_diag(Clock::time_point& next_diag) {
    auto now = Clock::now();
    if (now < next_diag) return;
    std::ostringstream os;
    os << "diag: bytes_read=" << g_stats.bytes_read
       << " packets_parsed=" << g_stats.rx_packets
       << " bytes_skipped=" << g_stats.bytes_skipped
       << " tx_frames=" << g_stats.tx_frames
       << " tx_filtered=" << g_stats.tx_filtered
       << " write_errors=" << g_stats.write_errors
       << " tcp_bad=" << g_stats.tcp_bad
       << " pings=" << g_stats.pings;
    log(os.str());
    next_diag = now + 5s;
}


// ==============================================================================
// SERVEI D'UN CLIENT: llegeix trames de 12 bytes i les executa.
// ==============================================================================

static void serve_client(int fd, const Config& cfg, SerialLink& ser, Client& cli, Clock::time_point& next_diag) {
    std::vector<uint8_t> buf;
    buf.reserve(4096);
    uint8_t rx[2048];
    auto last_rx = Clock::now();

    while (g_running) {
        pollfd p{fd, POLLIN, 0};
        int r = ::poll(&p, 1, 50);
        if (r < 0 && errno != EINTR) { log(std::string("poll: ") + std::strerror(errno)); return; }

        if (r > 0) {
            ssize_t n = ::recv(fd, rx, sizeof(rx), 0);
            if (n == 0) { log("el client ha tancat la connexió"); return; }
            if (n < 0) {
                if (errno == EINTR) continue;
                log(std::string("recv: ") + std::strerror(errno));
                return;
            }
            last_rx = Clock::now();
            buf.insert(buf.end(), rx, rx + n);

            size_t i = 0;
            while (i + TCP_FRAME <= buf.size()) {
                if (buf[i] != TCP_MAGIC) { g_stats.tcp_bad++; i++; continue; } // Desincronitzat: busquem el següent 0xA5
                uint8_t type = buf[i + 1], motor_id = buf[i + 2], cmd = buf[i + 3];
                const uint8_t* data = &buf[i + 4];
                if (type == T_CAN_TX) {
                    if (!cfg.allowed[motor_id]) {                               // Fora de l'allowlist --ids: no arriba al bus
                        g_stats.tx_filtered++;
                        if (!g_filter_logged[motor_id]) {
                            g_filter_logged[motor_id] = true;
                            log("filtering motor_id=" + std::to_string(motor_id) + " (not in allowlist)");
                        }
                    } else if (ser.write_frame(motor_id, cmd, data)) {
                        g_stats.tx_frames++;
                    } else {
                        g_stats.write_errors++;
                        cli.send_error(E_SERIAL_WRITE);
                    }
                } else if (type == T_PING) {
                    g_stats.pings++;
                    cli.send_frame(T_PING, motor_id, cmd, data);                // Pong idèntic
                } else {
                    g_stats.tcp_bad++;
                }
                i += TCP_FRAME;
            }
            if (i > 0) buf.erase(buf.begin(), buf.begin() + static_cast<long>(i));
        }

        if (cfg.watchdog_ms > 0 &&
            Clock::now() - last_rx > std::chrono::milliseconds(cfg.watchdog_ms)) {
            log("watchdog: cap trama del client en " + std::to_string(cfg.watchdog_ms) + " ms");
            return;
        }
        maybe_diag(next_diag);
    }
}

static void on_client_lost(const Config& cfg, SerialLink& ser) {
    if (!cfg.failsafe_reset) return;
    log("failsafe: RESET a tots els motors");
    uint8_t d[8];
    std::memset(d, 0x55, sizeof(d));
    for (uint8_t id : cfg.ids) ser.write_frame(id, CMD_RESET, d);
}


// ==============================================================================
// ARGUMENTS
// ==============================================================================

static bool parse_args(int argc, char** argv, Config& cfg) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](std::string& out) {
            if (i + 1 >= argc) return false;
            out = argv[++i];
            return true;
        };
        std::string v;
        try {
            if (a == "--port" && next(v))             cfg.port = v;
            else if (a == "--baud" && next(v))        cfg.baud = static_cast<uint32_t>(std::stoul(v));
            else if (a == "--tcp-port" && next(v))    cfg.tcp_port = static_cast<uint16_t>(std::stoul(v));
            else if (a == "--watchdog-ms" && next(v)) cfg.watchdog_ms = std::stoi(v);
            else if (a == "--allow" && next(v))       cfg.allow_ip = v;
            else if (a == "--failsafe" && next(v)) {
                if (v == "reset") cfg.failsafe_reset = true;
                else if (v == "none") cfg.failsafe_reset = false;
                else return false;
            } else if (a == "--ids" && next(v)) {
                cfg.ids.clear();
                std::stringstream ss(v);
                std::string tok;
                while (std::getline(ss, tok, ',')) cfg.ids.push_back(static_cast<uint8_t>(std::stoul(tok, nullptr, 0)));
            } else return false;
        } catch (std::exception&) {
            return false;
        }
    }
    return true;
}

static void on_signal(int) { g_running = false; }


// ==============================================================================
// MAIN
// ==============================================================================

int main(int argc, char** argv) {
    Config cfg;
    if (!parse_args(argc, argv, cfg)) {
        std::cerr << "ús: " << argv[0] << " [--port /dev/ttyACM0] [--baud 921600] [--tcp-port 5000]"
                     " [--watchdog-ms 1000] [--failsafe none|reset] [--ids 1,2,3,4,5,6] [--allow IP]\n";
        return 2;
    }
    std::string ids_txt;
    for (uint8_t id : cfg.ids) {
        cfg.allowed[id] = true;
        ids_txt += (ids_txt.empty() ? "" : ",") + std::to_string(id);
    }

    struct sigaction sa{};
    sa.sa_handler = on_signal;                                                  // Sense SA_RESTART: poll()/accept() tornen amb EINTR
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    SerialLink ser(cfg.port, cfg.baud);
    if (ser.open()) log("port obert: " + cfg.port + " @ " + std::to_string(cfg.baud));
    Client cli;
    std::thread reader(serial_reader, std::ref(ser), std::ref(cli));

    int lfd = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(cfg.tcp_port);
    if (::bind(lfd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0 || ::listen(lfd, 1) < 0) {
        log(std::string("no es pot escoltar al port TCP: ") + std::strerror(errno));
        g_running = false;
        reader.join();
        return 1;
    }
    log("escoltant TCP al port " + std::to_string(cfg.tcp_port) +
        (cfg.failsafe_reset ? " (failsafe=reset" : " (failsafe=none") + ", ids=" + ids_txt + ")");

    auto next_diag = Clock::now() + 5s;
    while (g_running) {
        pollfd p{lfd, POLLIN, 0};
        if (::poll(&p, 1, 500) <= 0) { maybe_diag(next_diag); continue; }

        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int fd = ::accept(lfd, reinterpret_cast<sockaddr*>(&caddr), &clen);
        if (fd < 0) continue;

        char ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &caddr.sin_addr, ip, sizeof(ip));
        if (!cfg.allow_ip.empty() && cfg.allow_ip != ip) {
            log(std::string("connexió rebutjada de ") + ip);
            ::close(fd);
            continue;
        }

        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));            // Cap retard de Nagle: cada trama surt de seguida
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
        timeval snd_to{0, 200000};                                              // send() bloquejat >200 ms = client mort
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));

        log(std::string("client connectat: ") + ip);
        cli.set(fd);
        if (!ser.ok()) cli.send_error(E_SERIAL_DOWN);
        serve_client(fd, cfg, ser, cli, next_diag);
        cli.clear();
        ::close(fd);
        log(std::string("client desconnectat: ") + ip);
        on_client_lost(cfg, ser);
    }

    log("aturant");
    ::close(lfd);
    reader.join();
    return 0;
}
