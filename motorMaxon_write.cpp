#include <iostream>
#include <unistd.h>
#include <pigpiod_if2.h>

int main() {
    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        std::cerr << "Falha ao conectar no pigpiod. Veja: sudo systemctl status pigpiod\n";
        return 1;
    }

    const int pwm[4] = {18, 23, 24, 25};

    // 90% de 255 = 229.5 -> 230
    const int duty90 = 230;

    for (int i = 0; i < 4; ++i) {
        set_mode(pi, pwm[i], PI_OUTPUT);
        set_PWM_frequency(pi, pwm[i], 500); // 1 kHz (ok pra driver/motor; pode mudar)
        set_PWM_range(pi, pwm[i], 255);
        set_PWM_dutycycle(pi, pwm[i], duty90);
    }

    std::cout << "PWM fixo em 90% nos GPIOs 18,23,24,25. Ctrl+C para parar.\n";
    std::cout.flush();

    while (true) {
        usleep(500000);
    }

    pigpio_stop(pi);
    return 0;
}
