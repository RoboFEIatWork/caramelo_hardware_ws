#include <iostream>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <pigpiod_if2.h>
#include <pigpio.h>

struct EncPins { int A; int B; };
static inline uint32_t bit(int gpio) { return (1u << gpio); }

int main() {
    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        std::cerr << "Falha ao conectar no pigpiod. Veja: sudo systemctl status pigpiod\n";
        return 1;
    }

    EncPins enc[4] = {
        { 5,  6},
        {12, 13},
        {16, 19},
        {20, 21}
    };

    for (int i = 0; i < 4; ++i) {
        set_mode(pi, enc[i].A, PI_INPUT);
        set_mode(pi, enc[i].B, PI_INPUT);

        set_pull_up_down(pi, enc[i].A, PI_PUD_OFF);
        set_pull_up_down(pi, enc[i].B, PI_PUD_OFF);

        set_glitch_filter(pi, enc[i].A, 1);
        set_glitch_filter(pi, enc[i].B, 1);
    }

    int handle = notify_open(pi);
    if (handle < 0) {
        std::cerr << "notify_open falhou.\n";
        pigpio_stop(pi);
        return 1;
    }

    uint32_t mask = 0;
    for (int i = 0; i < 4; ++i) {
        mask |= bit(enc[i].A);
        mask |= bit(enc[i].B);
    }

    if (notify_begin(pi, handle, mask) < 0) {
        std::cerr << "notify_begin falhou.\n";
        notify_close(pi, handle);
        pigpio_stop(pi);
        return 1;
    }

    char devname[64];
    snprintf(devname, sizeof(devname), "/dev/pigpio%d", handle);

    int fd = open(devname, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Falha ao abrir " << devname << ".\n";
        notify_close(pi, handle);
        pigpio_stop(pi);
        return 1;
    }

    long long count[4] = {0,0,0,0};
    long long lastCount[4] = {0,0,0,0};

    const double cpt = 1024.0;
    const double gear = 28.0;
    const double counts_per_wheel_rev = cpt * gear; // 28672

    const double dt = 0.1;
    const double rad_per_count = (2.0 * M_PI) / counts_per_wheel_rev;
    const double rad_per_sec_per_count = rad_per_count / dt;

    uint32_t lastLevel = 0;
    uint32_t lastTick = 0;

    gpioReport_t rep{};
    ssize_t n = read(fd, &rep, sizeof(rep));
    if (n == (ssize_t)sizeof(rep)) {
        lastLevel = rep.level;
        lastTick  = rep.tick;
    } else {
        uint32_t lvl = 0;
        for (int i = 0; i < 4; ++i) {
            if (gpio_read(pi, enc[i].A)) lvl |= bit(enc[i].A);
            if (gpio_read(pi, enc[i].B)) lvl |= bit(enc[i].B);
        }
        lastLevel = lvl;
        lastTick = get_current_tick(pi);   // ✅ aqui
    }

    while (true) {
        const uint32_t startTick = lastTick;

        while ((uint32_t)(lastTick - startTick) < 100000) {
            gpioReport_t r;
            ssize_t nr = read(fd, &r, sizeof(r));

            if (nr == (ssize_t)sizeof(r)) {
                uint32_t level = r.level;
                uint32_t changed = (lastLevel ^ level) & mask;

                for (int i = 0; i < 4; ++i) {
                    uint32_t aBit = bit(enc[i].A);
                    if (changed & aBit) {
                        bool aNow = (level & aBit);
                        if (aNow) {
                            bool bNow = (level & bit(enc[i].B));
                            count[i] += (bNow ? -1 : +1);
                        }
                    }
                }

                lastLevel = level;
                lastTick  = r.tick;
            } else {
                usleep(50);
                lastTick = get_current_tick(pi);  // ✅ aqui
            }
        }

        long long delta[4];
        double w[4];

        for (int i = 0; i < 4; ++i) {
            delta[i] = count[i] - lastCount[i];
            lastCount[i] = count[i];
            w[i] = (double)delta[i] * rad_per_sec_per_count;
        }

        std::cout
            << "100ms | "
            << "M1 d=" << delta[0] << " w=" << w[0] << " rad/s | "
            << "M2 d=" << delta[1] << " w=" << w[1] << " rad/s | "
            << "M3 d=" << delta[2] << " w=" << w[2] << " rad/s | "
            << "M4 d=" << delta[3] << " w=" << w[3] << " rad/s\n";
        std::cout.flush();
    }

    close(fd);
    notify_close(pi, handle);
    pigpio_stop(pi);
    return 0;
}