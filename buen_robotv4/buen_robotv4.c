// buen_robotv4.c — EK-TM4C1294XL + L298N + UART (V4: duty reescalado alto en bajos + kick corto) + HC-SR04
// Niveles perceptibles:
//   5%  -> ~52%  (muy lento pero ARRANCA seguro)
//   25% -> ~58%  (lento)
//   50% -> ~70%  (normal)
//   75% -> ~85%  (rápido)
//   100%-> 100%  (máximo)
//
// WASD: movimiento; ' ' o '0' = stop; 'k'/'l' bajan/suben duty; 'b' buzzer PB5
// HC-SR04: TRIG=PP5, ECHO=PA7 (divisor a 3V3). Autostop ≤ 3 cm.

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"

// ===== L298N pins =====
#define ENA_PORT   GPIO_PORTF_BASE
#define ENA_PIN    GPIO_PIN_1          // PF1 M0PWM1
#define ENB_PORT   GPIO_PORTF_BASE
#define ENB_PIN    GPIO_PIN_2          // PF2 M0PWM2
#define A_IN_FWD   GPIO_PORTK_BASE
#define A_IN_FWD_PIN GPIO_PIN_4        // PK4
#define A_IN_REV   GPIO_PORTK_BASE
#define A_IN_REV_PIN GPIO_PIN_5        // PK5
#define B_IN_FWD   GPIO_PORTM_BASE
#define B_IN_FWD_PIN GPIO_PIN_0        // PM0
#define B_IN_REV   GPIO_PORTM_BASE
#define B_IN_REV_PIN GPIO_PIN_1        // PM1

// ===== Buzzer =====
#define BUZZ_PORT  GPIO_PORTB_BASE
#define BUZZ_PIN   GPIO_PIN_5          // PB5
#define BUZZ_MS    2000u

// ===== HC-SR04 =====
#define TRIG_PORT  GPIO_PORTP_BASE
#define TRIG_PIN   GPIO_PIN_5          // PP5 (TRIG)
#define ECHO_PORT  GPIO_PORTA_BASE
#define ECHO_PIN   GPIO_PIN_7          // PA7 (ECHO con divisor a 3V3)
#define STOP_CM    3
#define LOOP_MS    60u

// ===== PWM/system =====
#define PWM_HZ     1000u               // si quieres más par a baja, prueba 500 o 250
static uint32_t gSysClk;
static uint32_t gLoad;

// ===== Duty steps (usuario) — inicia en 25% =====
static const uint8_t duty_user_steps[] = {5, 25, 50, 75, 100};
static volatile int8_t duty_idx = 1;   // 25%

// ===== Reescala “alto en bajos” =====
static const uint8_t duty_apply_map[] = {55, 61, 67, 73, 79}; // 5/25/50/75/100

// ===== Patada corta al salir de 0 =====
#define START_KICK_DUTY   80u   // golpe
#define START_KICK_MS     30u   // ~30 ms

// ===== Estado movimiento =====
typedef enum { M_STOP=0, M_FWD, M_REV, M_LEFT, M_RIGHT } motion_t;
static volatile motion_t g_motion = M_STOP;

// Buzzer
static volatile uint32_t buzzer_ms_left = 0;

// Último PWM aplicado (para decidir si venimos de 0)
static uint8_t lastA = 0, lastB = 0;

// ===== UART helpers =====
static inline void uart_putc(char c){ UARTCharPut(UART0_BASE, c); }
static void uart_puts(const char *s){ while(*s) uart_putc(*s++); }
static void uart_putu(uint32_t v){ char b[11]; int i=10; b[i--]='\0';
  if(!v){ uart_putc('0'); return; } while(v&&i>=0){ b[i--]='0'+(v%10); v/=10; } uart_puts(&b[i+1]); }
static inline void msg_duty(uint8_t user, uint8_t applied){
    uart_puts("DUTY,"); uart_putu(user); uart_puts(","); uart_putu(applied); uart_puts("\r\n");
}
static inline void msg_dist(int32_t cm){
    uart_puts("DIST,"); if(cm<0) uart_puts("-1"); else uart_putu((uint32_t)cm); uart_puts("\r\n");
}

static inline void delay_us(uint32_t us){ while(us--) MAP_SysCtlDelay(40); }

// ===== Direcciones y PWM =====
static inline void A_coast(void){ MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, 0); MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, 0); }
static inline void B_coast(void){ MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, 0); MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, 0); }
static inline void A_forward(void){ MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, A_IN_FWD_PIN); MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, 0); }
static inline void A_reverse(void){ MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, 0); MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, A_IN_REV_PIN); }
static inline void B_forward(void){ MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, B_IN_FWD_PIN); MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, 0); }
static inline void B_reverse(void){ MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, 0); MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, B_IN_REV_PIN); }
static inline uint32_t duty_to_ticks(uint32_t d){ if(d>100) d=100; return (gLoad * d) / 100u; }
static inline void set_pwm_A(uint8_t d){ MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_to_ticks(d)); lastA = d; }
static inline void set_pwm_B(uint8_t d){ MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, duty_to_ticks(d)); lastB = d; }

// Patada corta condicional
static inline void kick_if_needed(uint8_t targetA, uint8_t targetB){
    bool kickA = (lastA == 0 && targetA > 0 && targetA < START_KICK_DUTY);
    bool kickB = (lastB == 0 && targetB > 0 && targetB < START_KICK_DUTY);
    if(!(kickA || kickB)) return;
    if(kickA) set_pwm_A(START_KICK_DUTY);
    if(kickB) set_pwm_B(START_KICK_DUTY);
    MAP_SysCtlDelay(gSysClk/(3u * (1000u/START_KICK_MS))); // ≈ START_KICK_MS
}

// Aplica movimiento directo (sin rampas), con patada corta si hace falta
static void apply_motion(void){
    uint8_t user  = duty_user_steps[duty_idx];
    uint8_t dutyA = 0, dutyB = 0;
    uint8_t applied = duty_apply_map[duty_idx];

    switch(g_motion){
        case M_STOP:
            A_coast(); B_coast(); set_pwm_A(0); set_pwm_B(0);
            msg_duty(user, 0);
            return;
        case M_FWD:
            A_forward(); B_forward(); dutyA = applied; dutyB = applied; break;
        case M_REV:
            A_reverse(); B_reverse(); dutyA = applied; dutyB = applied; break;
        // Invertido para tu chasis: 'a' = izquierda física, 'd' = derecha física
        case M_LEFT:   B_coast(); dutyB = 0; A_forward(); dutyA = applied; break;
        case M_RIGHT:  A_coast(); dutyA = 0; B_forward(); dutyB = applied; break;
    }

    kick_if_needed(dutyA, dutyB);
    set_pwm_A(dutyA);
    set_pwm_B(dutyB);
    msg_duty(user, applied);
}

// ===== HC-SR04 =====
static void hcsr04_init(void){
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));
    MAP_GPIOPinTypeGPIOOutput(TRIG_PORT, TRIG_PIN);
    MAP_GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
    MAP_GPIOPinTypeGPIOInput(ECHO_PORT, ECHO_PIN);
    MAP_GPIOPadConfigSet(ECHO_PORT, ECHO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
}

static int32_t hcsr04_read_cm(void){
    MAP_GPIOPinWrite(TRIG_PORT, TRIG_PIN, TRIG_PIN); delay_us(10);
    MAP_GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
    uint32_t wait = 30000;
    while((MAP_GPIOPinRead(ECHO_PORT, ECHO_PIN)==0) && wait--) delay_us(1);
    if(!wait) return -1;
    uint32_t width_us=0; wait=40000;
    while((MAP_GPIOPinRead(ECHO_PORT, ECHO_PIN)!=0) && wait--){ delay_us(1); width_us++; }
    if(!wait) return -1;
    return (int32_t)(width_us/58u);
}

// ===== UART0 ISR =====
void UART0IntHandler(void){
    uint32_t st = MAP_UARTIntStatus(UART0_BASE, true);
    MAP_UARTIntClear(UART0_BASE, st);
    while(MAP_UARTCharsAvail(UART0_BASE)){
        int c = MAP_UARTCharGetNonBlocking(UART0_BASE);
        if(c < 0) break;
        char ch = (char)c;
        switch(ch){
            case 'w': g_motion = M_FWD;   apply_motion(); break;
            case 's': g_motion = M_REV;   apply_motion(); break;
            case 'a': g_motion = M_LEFT; apply_motion(); break; // invertido para tu chasis
            case 'd': g_motion = M_RIGHT;  apply_motion(); break; // invertido para tu chasis
            case '0': case ' ': g_motion = M_STOP;  apply_motion(); break;
            case 'k': case 'K': if(duty_idx > 0) duty_idx--; apply_motion(); break;
            case 'l': case 'L': if(duty_idx < (int)(sizeof(duty_user_steps)/sizeof(duty_user_steps[0]))-1) duty_idx++; apply_motion(); break;
            case 'b': buzzer_ms_left = BUZZ_MS; break;
            default: break;
        }
    }
}

// Stubs por si startup referencia estas IRQ
void GPIOJIntHandler(void){ MAP_GPIOIntClear(GPIO_PORTJ_BASE, 0xFF); }
void Timer0AIntHandler(void){ MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); }

int main(void){
    // Reloj 120 MHz
    gSysClk = MAP_SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240, 120000000);

    // Periféricos
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

    // DIR salidas y coast inicial
    MAP_GPIOPinTypeGPIOOutput(A_IN_FWD, A_IN_FWD_PIN);
    MAP_GPIOPinTypeGPIOOutput(A_IN_REV, A_IN_REV_PIN);
    MAP_GPIOPinTypeGPIOOutput(B_IN_FWD, B_IN_FWD_PIN);
    MAP_GPIOPinTypeGPIOOutput(B_IN_REV, B_IN_REV_PIN);
    A_coast(); B_coast();

    // Buzzer
    MAP_GPIOPinTypeGPIOOutput(BUZZ_PORT, BUZZ_PIN);
    MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);

    // PWM 1 kHz en PF1/PF2
    MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    uint32_t pwmclk = gSysClk / 64u;
    gLoad = (pwmclk / PWM_HZ) - 1u;
    MAP_GPIOPinConfigure(GPIO_PF1_M0PWM1);
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(ENA_PORT, ENA_PIN);
    MAP_GPIOPinTypePWM(ENB_PORT, ENB_PIN);
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, gLoad);
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, gLoad);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // UART0 115200 (PA0/PA1 por ICDI)
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_UARTConfigSetExpClk(UART0_BASE, gSysClk, 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    MAP_UARTFIFOEnable(UART0_BASE);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    MAP_IntEnable(INT_UART0);

    // HC-SR04
    hcsr04_init();

    // IRQ global
    MAP_IntMasterEnable();

    // Estado inicial
    g_motion = M_STOP;
    lastA = lastB = 0;
    msg_duty(duty_user_steps[duty_idx], duty_apply_map[duty_idx]);

    while(1){
        // Buzzer
        if(buzzer_ms_left > 0){
            MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, BUZZ_PIN);
            buzzer_ms_left = (buzzer_ms_left >= LOOP_MS) ? (buzzer_ms_left - LOOP_MS) : 0;
        }else{
            MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);
        }

        // Distancia + autostop duro
        int32_t d = hcsr04_read_cm();
        msg_dist(d);
        if(d > 0 && d <= STOP_CM){
            g_motion = M_STOP;
            A_coast(); B_coast();
            set_pwm_A(0); set_pwm_B(0);
        }

        delay_us(LOOP_MS * 1000u);
    }
}
