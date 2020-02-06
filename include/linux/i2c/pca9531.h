#ifndef __LINUX_PCA9531_H
#define __LINUX_PCA9531_H

#define PCA9531_LED0   0
#define PCA9531_LED1   1
#define PCA9531_LED2   2
#define PCA9531_LED3   3
#define PCA9531_LED4   4
#define PCA9531_LED5   5
#define PCA9531_LED6   6
#define PCA9531_LED7   7

#define PCA9531_ONOFF  0
#define PCA9531_PWM0   1
#define PCA9531_PWM1   2

struct pca9531_platform_data {
       char *name;
       char *trigger;                  /* see available led triggers */
       unsigned int brightness;        /* use values in [0:31] */
       unsigned int inverted:1;        /* inverted polarity led */
       unsigned int pwm:2;             /* 0 = pwm0, 1 = pwm1, 2 = none */
};

#endif /* __LINUX_PCA9531_H */
