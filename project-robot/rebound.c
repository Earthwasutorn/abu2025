// cd /home/pi-4/project-robot
// cc -o rebound rebound.c -lm
// sudo systemctl restart project-robot.service
// sudo ./rebound

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stdbool.h>
#include <dirent.h>
#include <sys/stat.h>

#define SERIAL_BY_ID_PATH "/dev/serial/by-id/"
#define BAUDRATE B115200
#define MAX_PATH 256

uint8_t data [9];
int L_STICK_X, L_STICK_Y;
int R_STICK_X, R_STICK_Y;
int fd_usb = -1;
int written;

// ฟังก์ชันส่งข้อมูลผ่าน USB --------------------------------------------------------
int try_open_serial(const char *device) {
    int fd_usb = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_usb == -1)
        return -1;

    fcntl(fd_usb, F_SETFL, 0);  // blocking mode

    struct termios options;
    tcgetattr(fd_usb, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd_usb, TCSANOW, &options);

    return fd_usb;
}

int find_arduino_mega(char *found_path, size_t size) {
    DIR *dir = opendir(SERIAL_BY_ID_PATH);
    if (!dir) return 0;

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, "arduino")) {
            snprintf(found_path, size, "%s%s", SERIAL_BY_ID_PATH, entry->d_name);
            closedir(dir);
            return 1;
        }
    }

    closedir(dir);
    return 0;
}

int is_disconnected(int fd_usb) {
    struct stat st;
    if (fstat(fd_usb, &st) == -1 || tcdrain(fd_usb) == -1) {
        return 1;
    }
    return 0;
}

int send_9_bytes(int fd_usb, uint8_t *data) {
    written = write(fd_usb, data, 9);
    return (written == 9) ? 1 : 0;
}

// ฟังก์ชัน MAP -----------------------------------------------------------------
int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ฟังก์ชันแปลงข้อมูลเป็น PWM ------------------------------------------------------
int stick_limit = 1000;
int pwm_motor (int stick) {
    int pwm_m = map_value(stick, 0, stick_limit, 0, 255);
    pwm_m = pwm_m > 255 ? 255 : pwm_m;
    return pwm_m;
}
int pwm_motor_x (int l_stick_x, int l_stick_y) {
    int pwm_x = map_value(abs(l_stick_x) - abs(l_stick_y), 0, stick_limit, 0, 255);
    pwm_x = pwm_x > 255 ? 255 : pwm_x;
    return pwm_x;
}
int pwm_motor_y (int l_stick_x, int l_stick_y) {
    int pwm_y = map_value(abs(l_stick_y) - abs(l_stick_x), 0, stick_limit, 0, 255);
    pwm_y = pwm_y > 255 ? 255 : pwm_y;
    return pwm_y;
}
int r_pwm_motor_x (int r_stick_x) {
    int r_pwm_x = map_value(abs(r_stick_x), stick_limit, 0, 0, 255);
    r_pwm_x = r_pwm_x > 255 ? 255 : r_pwm_x;
    return r_pwm_x;
}


// ฟังก์ชันส่งข้อมูลไปยังที่ต่าง ๆ -----------------------------------------------------
// pwm_motor_control (fll, flr, frl, frr, bll, blr, brl, brr);
void pwm_motor_control (uint8_t fll, uint8_t flr, uint8_t frl, uint8_t frr, uint8_t bll, uint8_t blr, uint8_t brl, uint8_t brr) {
    uint8_t mega_pwm_m [9];
    
    mega_pwm_m[0] = 0x01;
    mega_pwm_m[1] = fll;
    mega_pwm_m[2] = flr;
    mega_pwm_m[3] = frl;
    mega_pwm_m[4] = frr;
    mega_pwm_m[5] = bll;
    mega_pwm_m[6] = blr;
    mega_pwm_m[7] = brl;
    mega_pwm_m[8] = brr;
    printf("mega_pwm_m : %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        mega_pwm_m[0], mega_pwm_m[1], mega_pwm_m[2], mega_pwm_m[3],mega_pwm_m[4], mega_pwm_m[5], mega_pwm_m[6], mega_pwm_m[7], mega_pwm_m[8]);
    // send_9_bytes(fd_usb, mega_pwm_m);
    if (fd_usb > 0 && send_9_bytes(fd_usb, mega_pwm_m)) {}
}

// ฟังก์ชันหลัก ------------------------------------------------------------------
void process_event(struct js_event *js, int fd_usb) {
    js->type &= ~JS_EVENT_INIT;

    if (js->type == JS_EVENT_BUTTON) {
        switch (js->number) {
            case 0: {
                if (js->value == 1) {
                    printf("Button A : 🟢 Press\n");
                    data[0] = 0x02;
                    data[1] = 0x02;
                    if (fd_usb > 0 && send_9_bytes(fd_usb, data)) {}
                } else {
                    printf("Button A : 🔴 Release\n");
                    data[0] = 0x02;
                    data[1] = 0x00;
                    if (fd_usb > 0 && send_9_bytes(fd_usb, data)) {}
                }
                break;
            } case 1: {
                if (js->value == 1) {
                    // printf("Button B : 🟢 Press\n");
                } else {
                    // printf("Button B : 🔴 Release\n");
                }
                break;
            } case 3: {
                if (js->value == 1) {
                    printf("Button X : 🟢 Press\n");
                    data[0] = 0x02;
                    data[5] = 0x05;
                    data[6] = 0x06;
                    data[7] = 0x07;
                    data[8] = 0x08;
                    if (fd_usb > 0 && send_9_bytes(fd_usb, data)) {}
                } else {
                    printf("Button X : 🔴 Release\n");
                    data[0] = 0x02;
                    data[5] = 0x00;
                    data[6] = 0x00;
                    data[7] = 0x00;
                    data[8] = 0x00;
                    if (fd_usb > 0 && send_9_bytes(fd_usb, data)) {}
                }
                break;
            } case 4: {
                if (js->value == 1) {
                    printf("Button Y : 🟢 Press\n");
                    data[0] = 0x02;
                    data[1] = 0x01;
                    if (fd_usb > 0 && send_9_bytes(fd_usb, data)) {}
                } else {
                    printf("Button Y : 🔴 Release\n");
                    data[0] = 0x02;
                    data[1] = 0x00;
                    if (fd_usb > 0 && send_9_bytes(fd_usb, data)) {}
                }
                break;
            } case 6: {
                if (js->value == 1) {
                    // printf("Button LB : 🟢 Press\n");
                } else {
                    // printf("Button LB : 🔴 Release\n");
                }
                break;
            } case 7: {
                if (js->value == 1) {
                    // printf("Button RB : 🟢 Press\n");
                } else {
                    // printf("Button RB : 🔴 Release\n");
                }
                break;
            } case 10: {
                if (js->value == 1) {
                    // printf("Button VIEW : 🟢 Press\n");
                    printf("🔄 reboot\n");
                    system("sudo reboot");
                } else {
                    // printf("Button VIEW : 🔴 Release\n");
                }
                break;
            } case 11: {
                if (js->value == 1) {
                    // printf("Button MENU : 🟢 Press\n");
                    printf("🛑 poweroff\n");
                    system("sudo poweroff");
                } else {
                    // printf("Button MENU : 🔴 Release\n");
                }
                break;
            } case 12: {
                if (js->value == 1) {
                    // printf("Button XBOX : 🟢 Press\n");
                } else {
                    // printf("Button XBOX : 🔴 Release\n");
                }
                break;
            } case 13: {
                if (js->value == 1) {
                    // printf("Button L-STICK CLICK : 🟢 Press\n");
                } else {
                    // printf("Button L-STICK CLICK : 🔴 Release\n");
                }
                break;
            } case 14: {
                if (js->value == 1) {
                    // printf("Button R-STICK CLICK : 🟢 Press\n");
                } else {
                    // printf("Button R-STICK CLICK : 🔴 Release\n");
                }
                break;
            } default: {
                // printf("UNKNOWN\n");
                break;
            }
        }
    } else if (js->type == JS_EVENT_AXIS) {
        switch (js->number) {
            case 0: {
                L_STICK_X = map_value(js->value, -32767, 32767, -1000, 1000);
                // printf("L_STICK_X = %d\n",L_STICK_X);
                break;
            } case 1: {
                L_STICK_Y = map_value(js->value, 32767, -32767, -1000, 1000);
                // printf("L_STICK_Y = %d\n",L_STICK_Y);
                break;
            } case 2: {
                R_STICK_X = map_value(js->value, -32767, 32767, -1000, 1000);
                // printf("R_STICK_X = %d\n",R_STICK_X);
                break;
            } case 3: {
                R_STICK_Y = map_value(js->value, 32767, -32767, -1000, 1000);
                // printf("R_STICK_Y = %d\n",R_STICK_Y);
                break;
            } case 4: {
                int RT = map_value(js->value, -32767, 32767, 0, 1000);
                int pwm_m = pwm_motor(RT);
                // printf("RT = %d\n",RT);
                pwm_motor_control (0, pwm_m, pwm_m, 0, 0, pwm_m, pwm_m, 0);
                break;
            } case 5: {
                int LT = map_value(js->value, -32767, 32767, 0, 1000);
                int pwm_m = pwm_motor(LT);
                // printf("LT = %d\n",LT);
                pwm_motor_control (pwm_m, 0, 0, pwm_m, pwm_m, 0, 0, pwm_m);
                break;
            } case 6: {
                if (js->value == -32767) {
                    int D_PAD_X = -1;
                    // printf("⬅️ D_PAD_X = %d\n",D_PAD_X);
                } else if (js->value == 32767) {
                    int D_PAD_X = 1;
                    // printf("➡️ D_PAD_X = %d\n",D_PAD_X);
                } else {
                    int D_PAD_X = 0;
                    // printf("D_PAD_X = %d\n",D_PAD_X);
                }
                break;
            } case 7: {
                if (js->value == -32767) {
                    int D_PAD_Y = 1;
                    // printf("⬆️ D_PAD_Y = %d\n",D_PAD_Y);
                } else if (js->value == 32767) {
                    int D_PAD_Y = -1;
                    // printf("⬇️ D_PAD_Y = %d\n",D_PAD_Y);
                } else {
                    int D_PAD_Y = 0;
                    // printf("D_PAD_Y = %d\n",D_PAD_Y);
                }
                break;
            } default: {
		        // printf("%d : %d\n",js->number,js->value);
                break;
            }
        }
        if (js->number == 0 || js->number == 1) {
            int L_STICK = sqrt(pow(L_STICK_X, 2) + pow(L_STICK_Y, 2));
            int pwm_m = pwm_motor(L_STICK);
            int pwm_x = pwm_motor_x(L_STICK_X, L_STICK_Y);
            int pwm_y = pwm_motor_y(L_STICK_X, L_STICK_Y);
            printf("X = %d , Y = %d : L_STICK = %d\n", L_STICK_X, L_STICK_Y, L_STICK);
            
            if (L_STICK_X == 0 && L_STICK_Y > 0) {
                pwm_motor_control (0, pwm_m, 0, pwm_m, 0, pwm_m, 0, pwm_m);     //Forward
            } else if (L_STICK_X == 0 && L_STICK_Y < 0) {
                pwm_motor_control (pwm_m, 0, pwm_m, 0, pwm_m, 0, pwm_m, 0);     //Backward
            } else if (L_STICK_X < 0 && L_STICK_Y == 0) {
                pwm_motor_control (pwm_m, 0, 0, pwm_m, 0, pwm_m, pwm_m, 0);     //Left
            } else if (L_STICK_X > 0 && L_STICK_Y == 0) {
                pwm_motor_control (0, pwm_m, pwm_m, 0, pwm_m, 0, 0, pwm_m);     //Right
            } else if (L_STICK_X > 0 && L_STICK_Y > 0) {
                if (abs(L_STICK_X) < abs(L_STICK_Y)) {
                    pwm_motor_control (0, pwm_m, 0, pwm_y, 0, pwm_y, 0, pwm_m); //pwm_y control
                } else if (abs(L_STICK_X) == abs(L_STICK_Y)) {
                    pwm_motor_control (0, pwm_m, 0, 0, 0, 0, 0, pwm_m);         //Forward Right
                } else if (abs(L_STICK_X) > abs(L_STICK_Y)) {
                    pwm_motor_control (0, pwm_m, pwm_x, 0, pwm_x, 0, 0, pwm_m); //pwm_x control
                }
            } else if (L_STICK_X < 0 && L_STICK_Y < 0) {
                if (abs(L_STICK_X) < abs(L_STICK_Y)) {
                    pwm_motor_control (pwm_m, 0, pwm_y, 0, pwm_y, 0, pwm_m, 0); //pwm_y control
                } else if (abs(L_STICK_X) == abs(L_STICK_Y)) {
                    pwm_motor_control (pwm_m, 0, 0, 0, 0, 0, pwm_m, 0);         //Backward Left
                } else if (abs(L_STICK_X) > abs(L_STICK_Y)) {
                    pwm_motor_control (pwm_m, 0, 0, pwm_x, 0, pwm_x, pwm_m, 0); //pwm_x control
                }
            } else if (L_STICK_X < 0 && L_STICK_Y > 0) {
                if (abs(L_STICK_X) < abs(L_STICK_Y)) {
                    pwm_motor_control (0, pwm_y, 0, pwm_m, 0, pwm_m, 0, pwm_y); //pwm_y control
                } else if (abs(L_STICK_X) == abs(L_STICK_Y)) {
                    pwm_motor_control (0, 0, 0, pwm_m, 0, pwm_m, 0, 0);         //Forward Left
                } else if (abs(L_STICK_X) > abs(L_STICK_Y)) {
                    pwm_motor_control (pwm_x, 0, 0, pwm_m, 0, pwm_m, pwm_x, 0); //pwm_x control
                }
            } else if (L_STICK_X > 0 && L_STICK_Y < 0) {
                if (abs(L_STICK_X) < abs(L_STICK_Y)) {
                    pwm_motor_control (pwm_y, 0, pwm_m, 0, pwm_m, 0, pwm_y, 0); //pwm_y control
                } else if (abs(L_STICK_X) == abs(L_STICK_Y)) {
                    pwm_motor_control (0, 0, pwm_m, 0, pwm_m, 0, 0, 0);         //Backward Right
                } else if (abs(L_STICK_X) > abs(L_STICK_Y)) {
                    pwm_motor_control (0, pwm_x, pwm_m, 0, pwm_m, 0, 0, pwm_x); //pwm_x control
                }
            } else {
                pwm_motor_control (0, 0, 0, 0, 0, 0, 0, 0);                     //Stop
            }
        }

        if (js->number == 2 || js->number == 3) {
            int R_STICK = sqrt(pow(R_STICK_X, 2) + pow(R_STICK_Y, 2));
            int pwm_m = pwm_motor(R_STICK);
            int r_pwm_x = r_pwm_motor_x(R_STICK_X);
            printf("X = %d , Y = %d : R_STICK = %d\n", R_STICK_X, R_STICK_Y, R_STICK);
            
            if (R_STICK_X == 0 && R_STICK_Y > 0) {
                pwm_motor_control (0, pwm_m, 0, pwm_m, 0, pwm_m, 0, pwm_m);     //Forward
            } 
            else if (R_STICK_X == 0 && R_STICK_Y < 0) {
                pwm_motor_control (pwm_m, 0, pwm_m, 0, pwm_m, 0, pwm_m, 0);     //Backward
            } 
            // else if (R_STICK_X < 0 && R_STICK_Y == 0) {
            //     pwm_motor_control (0, 0, 0, pwm_m, 0, 0, 0, pwm_m);     //Left
            // } 
            // else if (R_STICK_X > 0 && R_STICK_Y == 0) {
            //     pwm_motor_control (0, pwm_m, 0, 0, 0, 0, 0, pwm_m);     //Right
            // } 
            else if (R_STICK_X > 0 && R_STICK_Y > 0) {
                pwm_motor_control (0, pwm_m, 0, r_pwm_x, 0, pwm_m, 0, r_pwm_x); //Right
            } else if (R_STICK_X < 0 && R_STICK_Y > 0) {
                pwm_motor_control (0, r_pwm_x, 0, pwm_m, 0, r_pwm_x, 0, pwm_m); //Left
            } else if (R_STICK_X > 0 && R_STICK_Y < 0) {
                if (abs(R_STICK_X) < abs(R_STICK_Y)) {
                    pwm_motor_control (pwm_m, 0, r_pwm_x, 0, pwm_m, 0, r_pwm_x, 0); //Right
                } else if (abs(R_STICK_X) == abs(R_STICK_Y)) {
                    pwm_motor_control (pwm_m, 0, r_pwm_x, 0, pwm_m, 0, r_pwm_x, 0); //Right
                }
            } else if (R_STICK_X < 0 && R_STICK_Y < 0) {
                if (abs(R_STICK_X) < abs(R_STICK_Y)) {
                    pwm_motor_control (r_pwm_x, 0, pwm_m, 0, r_pwm_x, 0, pwm_m, 0); //Left
                } else if (abs(R_STICK_X) == abs(R_STICK_Y)) {
                    pwm_motor_control (r_pwm_x, 0, pwm_m, 0, r_pwm_x, 0, pwm_m, 0); //Left
                }
            } else {
                pwm_motor_control (0, 0, 0, 0, 0, 0, 0, 0);                     //Stop
            }
        }
    }
}

int open_joystick(const char *device_path) {
    int fd;
    printf("⚠️ รอการเชื่อมต่อ joystick... (%s)\n", strerror(errno));
    while ((fd = open(device_path, O_RDONLY)) < 0) {
        sleep(1);
    }
    printf("🎮 เชื่อมต่อกับอุปกรณ์ %s\n", device_path);
    return fd;
}

int main() {
    const char *device_path = "/dev/input/js0";
    int fd = open_joystick(device_path);
    struct js_event js;

    L_STICK_X = 0;
    L_STICK_Y = 0;
    R_STICK_X = 0;
    R_STICK_Y = 0;

    char found_path[MAX_PATH];

    while (1) {
        if (is_disconnected(fd_usb)) {
            printf("🔌 USB disconnected... attempting reconnect\n");
            close(fd_usb);
            fd_usb = -1;

            while (fd_usb < 0) {
                char device_path[MAX_PATH];
                if (find_arduino_mega(device_path, sizeof(device_path))) {
                    fd_usb = try_open_serial(device_path);
                }
                sleep(1); // รอ 1 วินาทีก่อนลองใหม่
            }
            printf("✅ Reconnected to %s\n", device_path);
        }

        ssize_t bytes = read(fd, &js, sizeof(js));
        if (bytes == sizeof(js)) {
            process_event(&js, fd_usb);
        } else {
            printf("❌ การเชื่อมต่อขาด! รอการเชื่อมต่อใหม่...\n");
            close(fd);
            fd = open_joystick(device_path);
        }
    }

    close(fd);
    return 0;
}
