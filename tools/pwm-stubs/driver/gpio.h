#ifndef TEST_GPIO_H
#define TEST_GPIO_H
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULLDOWN_ONLY 2
int gpio_set_level(int pin,int level);
int gpio_set_pull_mode(int pin,int pull);
int gpio_set_direction(int pin,int mode);
#endif
