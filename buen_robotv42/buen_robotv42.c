// buen_robot_ordenado.c
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

// ============================================================================
//                                CONFIGURACIÓN
// ============================================================================

// Giro 180° (tuning)
// Giro 180° (tuning)
#define TURN_TIME_MS     1230u   // antes 650u → ~x2 para ~180° esto cambio para tiempo de giro 
#define TURN_COOLDOWN_MS 300u
// PWM por motor durante el giro (permite compensar asimetrías)
#define TURN_A_DUTY_PCT  60u     // motor A (derecha) durante el giro  esto cambio para motores 
#define TURN_B_DUTY_PCT  60u     // motor B (izquierda) durante el giro  esto cambio para motores 


// ---- L298N (dos motores) ----
#define ENA_PORT        GPIO_PORTF_BASE
#define ENA_PIN         GPIO_PIN_1          // PF1 → M0PWM1 (Motor A)
#define ENB_PORT        GPIO_PORTF_BASE
#define ENB_PIN         GPIO_PIN_2          // PF2 → M0PWM2 (Motor B)

#define A_IN_FWD        GPIO_PORTK_BASE
#define A_IN_FWD_PIN    GPIO_PIN_4          // PK4
#define A_IN_REV        GPIO_PORTK_BASE
#define A_IN_REV_PIN    GPIO_PIN_5          // PK5

#define B_IN_FWD        GPIO_PORTM_BASE
#define B_IN_FWD_PIN    GPIO_PIN_0          // PM0
#define B_IN_REV        GPIO_PORTM_BASE
#define B_IN_REV_PIN    GPIO_PIN_1          // PM1

// ---- Buzzer (opcional) ----
#define BUZZ_PORT       GPIO_PORTB_BASE
#define BUZZ_PIN        GPIO_PIN_5          // PB5
#define BUZZ_MS         2000u

// ---- Ultrasonido HC-SR04 ----
#define TRIG_PORT       GPIO_PORTP_BASE
#define TRIG_PIN        GPIO_PIN_5          // PP5 (TRIG)
#define ECHO_PORT       GPIO_PORTA_BASE
#define ECHO_PIN        GPIO_PIN_7          // PA7 (ECHO)
#define STOP_CM         12      // <-- antes 3 AQUI PARA EL SENSOR DISTANCIA 
#define LOOP_MS         40u     // <-- opcional: más rápido (20 ms) para reaccionar antes
             // periodo de ciclo de fondo (ms)

// ---- PWM / Sistema ----
#define PWM_HZ          1000u               // 500–1000 Hz dan par decente a bajas

// --- Slowdown (approach) ---
#define SLOW_START_CM   STOP_CM // = 10 → no hay rampa, todo es “zona de stop”
#define SLOW_STOP_CM    STOP_CM 
#define SLOW_MIN_PCT    100u    // % mínimo mientras se aproxima (evita paradas bruscas)
static volatile uint8_t g_slow_pct = 100u;  // 100% = sin recorte

#define TURN_MAX_DEG      360u  // limitar por seguridad

// ms necesarios para un giro 'deg' en base a tu calibración de 180°
static inline uint32_t ms_for_degrees(uint16_t deg){
    if(deg > TURN_MAX_DEG) deg = TURN_MAX_DEG;
    return ((uint32_t)TURN_TIME_MS * (uint32_t)deg) / 180u;
}


static volatile bool g_auto_run = false;   // arranca avanzando “solo” hasta SPACE/0    //PARA 180





static uint32_t gSysClk;                    // 120 MHz
static uint32_t gLoad;                      // periodo de PWM (ticks)
static uint32_t TICKS_PER_US; 

// ---- Escalones de “velocidad usuario” (teclas k/l) ----
static const uint8_t duty_user_steps[] = { 5, 25, 50, 75, 100 };
static volatile int8_t duty_idx = 1;        // arranca en 25%

// ---- Reescala para “alto en bajos” (aplicado a ENx) ----
static const uint8_t duty_apply_map[] = { 55, 61, 67, 73, 79 }; // mapea 5/25/50/75/100 → 55/61/67/73/79%

// ---- Patada de arranque (si venimos de 0) ----
#define START_KICK_DUTY 80u
#define START_KICK_MS   30u

// ---- Trims / compensaciones por motor y sentido ----
//   100 = 100% (sin cambio), 98 = -2%, 105 = +5%
#define TRIM_A_FWD      110u   // motor derecha, adelante
#define TRIM_A_REV      111u   // motor A, atrás
#define TRIM_B_FWD      112u   // motor B, adelante
#define TRIM_B_REV      109u   // motor B, atrás
// Ganancia global por sentido (adelante/atrás)
#define DIR_GAIN_FWD    100u
#define DIR_GAIN_REV    100u

// ============================================================================
//                                 ESTADO
// ============================================================================

typedef enum { M_STOP=0, M_FWD, M_REV, M_LEFT, M_RIGHT } motion_t;
static volatile motion_t g_motion = M_STOP;

static volatile uint32_t buzzer_ms_left = 0;
static uint8_t lastA = 0, lastB = 0;        // último duty aplicado (para “patada”)

// ---- Ultrasonido (compartido entre ISR y main) ----
static volatile uint32_t us_echo_rise = 0;
static volatile uint32_t us_last_width = 0;
static volatile int32_t  us_last_cm = -1;
static volatile bool     us_new_sample = false;
static volatile bool     us_hard_stop  = false;

// GIRO 180 GRADOS 
static volatile bool g_need_turn = false;     // pedir giro 180° desde la ISR
static volatile uint32_t us_ignore_ms = 0;    // ignorar hard-stop durante este tiempo
// --- Modo interactivo de giro ---
static volatile bool     g_wait_turn    = false;   // esperando que el usuario ingrese L/R + grados
static volatile bool     g_have_turn_cmd= false;   // ya hay comando completo para ejecutar
static volatile char     g_turn_side    = '?';     // 'L' o 'R'
static volatile uint16_t g_turn_deg     = 0;       // 1..360
static char              g_angle_buf[4] = {0};     // hasta "360"
static volatile uint8_t  g_angle_len    = 0;       // # dígitos leídos




// ============================================================================
/*                               UTILIDADES */
// ============================================================================

static inline void delay_us(uint32_t us) { while (us--) MAP_SysCtlDelay(40); }

static inline void uart_putc(char c) { UARTCharPut(UART0_BASE, c); }
static void uart_puts(const char *s){ while(*s) uart_putc(*s++); }
static void uart_putu(uint32_t v){ char b[11]; int i=10; b[i--]='\0';
  if(!v){ uart_putc('0'); return; } while(v&&i>=0){ b[i--]='0'+(v%10); v/=10; } uart_puts(&b[i+1]); }

static inline void msg_duty(uint8_t user, uint8_t applied){
    uart_puts("DUTY,"); uart_putu(user); uart_puts(","); uart_putu(applied); uart_puts("\r\n");
}
static inline void msg_dist(int32_t cm){
    uart_puts("DIST,"); if(cm<0) uart_puts("-1"); else uart_putu((uint32_t)cm); uart_puts("\r\n");
}
static inline uint32_t ticks_to_us(uint32_t ticks){
    // con resta unsigned, el wrap se maneja solo: (now - rise)
    return ticks / TICKS_PER_US;   // a 120 MHz: us = ticks / 120
}
static inline int32_t us_to_cm(uint32_t us){
    return (int32_t)(us / 58u);
}
// ============================================================================
//                       GPIO: DIRECCIONES + PWM HELPERS
// ============================================================================

static inline void A_coast(void){  // rueda libre
    MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, 0);
    MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, 0);
}
static inline void B_coast(void){
    MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, 0);
    MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, 0);
}
static inline void A_forward(void){
    MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, A_IN_FWD_PIN);
    MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, 0);
}
static inline void A_reverse(void){
    MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, 0);
    MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, A_IN_REV_PIN);
}
static inline void B_forward(void){
    MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, B_IN_FWD_PIN);
    MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, 0);
}
static inline void B_reverse(void){
    MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, 0);
    MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, B_IN_REV_PIN);
}
static inline void A_brake(void){  // freno activo (IN=1/1)
    MAP_GPIOPinWrite(A_IN_FWD, A_IN_FWD_PIN, A_IN_FWD_PIN);
    MAP_GPIOPinWrite(A_IN_REV, A_IN_REV_PIN, A_IN_REV_PIN);
}
static inline void B_brake(void){
    MAP_GPIOPinWrite(B_IN_FWD, B_IN_FWD_PIN, B_IN_FWD_PIN);
    MAP_GPIOPinWrite(B_IN_REV, B_IN_REV_PIN, B_IN_REV_PIN);
}

static inline uint32_t duty_to_ticks(uint32_t d){
    if(d > 100u) d = 100u;
    return (gLoad * d) / 100u;
}
static inline void set_pwm_A(uint8_t d){
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, duty_to_ticks(d));
    lastA = d;
}
static inline void set_pwm_B(uint8_t d){
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, duty_to_ticks(d));
    lastB = d;
}

// “Patada” corta si venimos de 0 y el duty objetivo es bajo
static inline void kick_if_needed(uint8_t targetA, uint8_t targetB){
    bool kickA = (lastA == 0 && targetA > 0 && targetA < START_KICK_DUTY);
    bool kickB = (lastB == 0 && targetB > 0 && targetB < START_KICK_DUTY);
    if(!(kickA || kickB)) return;
    if(kickA) set_pwm_A(START_KICK_DUTY);
    if(kickB) set_pwm_B(START_KICK_DUTY);
    MAP_SysCtlDelay(gSysClk/(3u * (1000u/START_KICK_MS))); // ≈ START_KICK_MS
}

// ============================================================================
//                               APLICAR MOVIMIENTO
// ============================================================================



// Devuelve % de velocidad permitido según distancia.
// >=SLOW_START_CM → 100% ; entre (STOP_CM, SLOW_START_CM) baja lineal a SLOW_MIN_PCT ; <=STOP_CM → 0%
static inline uint8_t speed_limit_from_cm(int32_t cm){
    if(cm < 0)            return 100u; // lectura inválida → no recortar
    if(cm <= SLOW_STOP_CM) return 0u;  // se encargará el hard-stop
    if(cm >= SLOW_START_CM) return 100u;
    // interpolación lineal de (STOP_CM, SLOW_START_CM) → [SLOW_MIN_PCT, 100]
    uint32_t span = (uint32_t)(SLOW_START_CM - SLOW_STOP_CM);  // e.g. 7
    uint32_t pos  = (uint32_t)(cm - SLOW_STOP_CM);             // 1..6..7
    uint32_t pct  = SLOW_MIN_PCT + (pos * (100u - SLOW_MIN_PCT)) / span;
    if(pct > 100u) pct = 100u;
    return (uint8_t)pct;
}



static void apply_motion(void){
    uint8_t user    = duty_user_steps[duty_idx];
    uint8_t applied = duty_apply_map[duty_idx];
    uint8_t dutyA   = 0, dutyB = 0;

    switch(g_motion){
        case M_STOP:
            A_coast(); B_coast(); set_pwm_A(0); set_pwm_B(0);
            msg_duty(user, 0);
            return;

        case M_FWD:
            A_forward(); B_forward(); dutyA = applied; dutyB = applied;
            break;

        case M_REV:
            A_reverse(); B_reverse(); dutyA = applied; dutyB = applied;
            break;

        case M_LEFT:   // solo avanza la rueda contraria
            B_coast(); dutyB = 0; A_forward(); dutyA = applied;
            break;

        case M_RIGHT:
            A_coast(); dutyA = 0; B_forward(); dutyB = applied;
            break;
    }

    // ---- TRIMS / GANANCIAS POR SENTIDO ----
    
    
    // Bloqueo de avance si estamos en zona de STOP
if ((g_motion == M_FWD || g_motion == M_LEFT || g_motion == M_RIGHT) &&
    (us_last_cm > 0 && us_last_cm <= STOP_CM)) {
    A_brake(); B_brake();
    set_pwm_A(0); set_pwm_B(0);
    g_motion = M_STOP;
    msg_duty(duty_user_steps[duty_idx], 0);
    return;
}

    if(g_motion == M_FWD || g_motion == M_LEFT || g_motion == M_RIGHT){
        uint32_t a = (uint32_t)dutyA * TRIM_A_FWD / 100u;
        uint32_t b = (uint32_t)dutyB * TRIM_B_FWD / 100u;
        a = a * DIR_GAIN_FWD / 100u;
        b = b * DIR_GAIN_FWD / 100u;
        dutyA = (a > 100u) ? 100u : (uint8_t)a;
        dutyB = (b > 100u) ? 100u : (uint8_t)b;
    }else if(g_motion == M_REV){
        uint32_t a = (uint32_t)dutyA * TRIM_A_REV / 100u;
        uint32_t b = (uint32_t)dutyB * TRIM_B_REV / 100u;
        a = a * DIR_GAIN_REV / 100u;
        b = b * DIR_GAIN_REV / 100u;
        dutyA = (a > 100u) ? 100u : (uint8_t)a;
        dutyB = (b > 100u) ? 100u : (uint8_t)b;
    }

    // ---- SLOWDOWN POR DISTANCIA (solo al avanzar/curvar) ----
    if (g_motion == M_FWD || g_motion == M_LEFT || g_motion == M_RIGHT) {
        uint32_t a = ((uint32_t)dutyA * g_slow_pct) / 100u;
        uint32_t b = ((uint32_t)dutyB * g_slow_pct) / 100u;
        dutyA = (a > 100u) ? 100u : (uint8_t)a;
        dutyB = (b > 100u) ? 100u : (uint8_t)b;
    }

    // Patada corta si hace falta y aplicar
    kick_if_needed(dutyA, dutyB);
    set_pwm_A(dutyA);
    set_pwm_B(dutyB);
    msg_duty(user, applied);
}


// ============================================================================
//                       ULTRASONIDO (TRIG + ECHO por IRQ)
// ============================================================================

static inline void hcsr04_trigger(void){
    // Pulso de 10us en TRIG
    MAP_GPIOPinWrite(TRIG_PORT, TRIG_PIN, TRIG_PIN);
    delay_us(10);  
    MAP_GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
}

static void hcsr04_init_irq(void){
    // TRIG salida
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    MAP_GPIOPinTypeGPIOOutput(TRIG_PORT, TRIG_PIN);
    MAP_GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);

    // ECHO entrada con pulldown
    MAP_GPIOPinTypeGPIOInput(ECHO_PORT, ECHO_PIN);
    MAP_GPIOPadConfigSet(ECHO_PORT, ECHO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);

    // Timer1 como contador a ~1 MHz (para medir ancho)
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC_UP);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFFFFFF);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);

    // IRQ por ambos flancos en PA7 (ECHO)
    MAP_GPIOIntDisable(ECHO_PORT, ECHO_PIN);
    MAP_GPIOIntTypeSet(ECHO_PORT, ECHO_PIN, GPIO_BOTH_EDGES);
    MAP_GPIOIntClear(ECHO_PORT, ECHO_PIN);
    MAP_GPIOIntEnable(ECHO_PORT, ECHO_PIN);

    // Prioridades: ECHO más alta
    MAP_IntPrioritySet(INT_GPIOA,   0x00);
    MAP_IntPrioritySet(INT_UART0,   0x80);
    MAP_IntPrioritySet(INT_TIMER0A, 0xA0);

    MAP_IntEnable(INT_GPIOA);
}

// IRQ de ECHO (PA7)
void GPIOAIntHandler(void){
    uint32_t status = MAP_GPIOIntStatus(ECHO_PORT, true);
    MAP_GPIOIntClear(ECHO_PORT, status);
    if(!(status & ECHO_PIN)) return;

    uint32_t pin = MAP_GPIOPinRead(ECHO_PORT, ECHO_PIN);
    uint32_t now = MAP_TimerValueGet(TIMER1_BASE, TIMER_A);

    if(pin){
        // Flanco de subida → inicio de pulso
        us_echo_rise = now;
    }else{
        // Flanco de bajada → fin de pulso, medir
        uint32_t ticks = now - us_echo_rise;         // wrap ok (unsigned)
        uint32_t width_us = ticks_to_us(ticks);
        us_last_width = width_us;

        // Filtros
        if(width_us < 100u || width_us > 25000u){
            us_last_cm = -1;
        }else{
            us_last_cm = us_to_cm(width_us);
        }
        us_new_sample = true;

        // Disparar modo interactivo SÓLO si no estamos ignorando ni ya esperando
        if (us_ignore_ms == 0 && !g_wait_turn && us_last_cm > 0 && us_last_cm <= STOP_CM) {
            // Freno + PWM=0
            A_brake(); B_brake();
            MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
            MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
            g_motion = M_STOP;

            // Entrar en "esperar comando"
            g_wait_turn     = true;
            g_have_turn_cmd = false;
            g_turn_side     = '?';
            g_turn_deg      = 0;
            g_angle_len     = 0;
            g_angle_buf[0]  = '\0';

            // Evitar múltiples disparos mientras esperas
            us_ignore_ms    = 500u;

            uart_puts("TURN? Ingresa L/R y grados, ej: L90 o R45. ENTER para confirmar\r\n");
        }
    }
}



// ============================================================================
//                                 UART0 (ISR)
// ============================================================================

void UART0IntHandler(void){
while(MAP_UARTCharsAvail(UART0_BASE)){
    int c = MAP_UARTCharGetNonBlocking(UART0_BASE);
    if(c < 0) break;
    char ch = (char)c;

    // === Modo "esperando comando de giro" ===
    if (g_wait_turn) {
        if (ch == 'l' || ch == 'L') {
            g_turn_side = 'L';
            uart_puts("L\r\n");
        } else if (ch == 'r' || ch == 'R') {
            g_turn_side = 'R';
            uart_puts("R\r\n");
        } else if (ch >= '0' && ch <= '9') {
            if (g_angle_len < 3) {               // hasta 3 dígitos (360 máx)
                g_angle_buf[g_angle_len++] = ch;
                g_angle_buf[g_angle_len] = '\0';
                uart_putc(ch);
            }
        } else if (ch == '\r' || ch == '\n') {
            // Confirmar
            uint16_t deg = 0;
            for (uint8_t i=0;i<g_angle_len;i++) deg = deg*10 + (uint16_t)(g_angle_buf[i]-'0');
            if (deg == 0) deg = 90;              // por si no escribes grados, gira 90 por defecto
            if (deg > TURN_MAX_DEG) deg = TURN_MAX_DEG;

            if (g_turn_side == 'L' || g_turn_side == 'R') {
                g_turn_deg       = deg;
                g_have_turn_cmd  = true;         // el lazo principal ejecuta el giro
                g_wait_turn      = false;        // salimos del modo entrada
                uart_puts("\r\nOK\r\n");
            } else {
                uart_puts("\r\nFalta lado L/R\r\n");
            }
        } else if (ch == ' ') {
            // Cancelar
            g_wait_turn = false;
            uart_puts("\r\nCANCEL\r\n");
        }
        // Mientras esperamos comando, ignoramos el resto de teclas (w/a/s/d/k/l/etc)
        continue;
    }

    // === Modo normal (no esperando comando) ===
    switch(ch){
        case 'w':
            g_auto_run = true;
            g_motion = M_FWD;
            apply_motion();
            break;

        case 's': g_motion = M_REV;   apply_motion(); break;
        case 'a': g_motion = M_LEFT;  apply_motion(); break;
        case 'd': g_motion = M_RIGHT; apply_motion(); break;

        case '0':
        case ' ':
            g_auto_run = false;
            g_motion = M_STOP;
            apply_motion();
            break;

        case 'k': case 'K': if(duty_idx > 0) duty_idx--; apply_motion(); break;
        case 'l': case 'L': if(duty_idx < (int)(sizeof(duty_user_steps)/sizeof(duty_user_steps[0]))-1) duty_idx++; apply_motion(); break;
        case 'b': buzzer_ms_left = BUZZ_MS; break;
        default: break;
    }
}

}

// Stubs por si el startup referencia estas IRQ (no usadas aquí)
void GPIOJIntHandler(void){ MAP_GPIOIntClear(GPIO_PORTJ_BASE, 0xFF); }
void Timer0AIntHandler(void){ MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); }

// ============================================================================
//                                    MAIN
// ============================================================================

int main(void){
    // Reloj 120 MHz
    gSysClk = MAP_SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240, 120000000);

TICKS_PER_US = gSysClk / 1000000u;

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

    // DIR: salidas + coast inicial
    MAP_GPIOPinTypeGPIOOutput(A_IN_FWD, A_IN_FWD_PIN);
    MAP_GPIOPinTypeGPIOOutput(A_IN_REV, A_IN_REV_PIN);
    MAP_GPIOPinTypeGPIOOutput(B_IN_FWD, B_IN_FWD_PIN);
    MAP_GPIOPinTypeGPIOOutput(B_IN_REV, B_IN_REV_PIN);
    A_coast(); B_coast();

    // Buzzer
    MAP_GPIOPinTypeGPIOOutput(BUZZ_PORT, BUZZ_PIN);
    MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);

    // PWM 1 kHz en PF1 / PF2
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

    // UART0 115200 (PA0/PA1)
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_UARTConfigSetExpClk(UART0_BASE, gSysClk, 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    MAP_UARTFIFOEnable(UART0_BASE);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    MAP_IntEnable(INT_UART0);

    // Ultrasonido por interrupciones
    hcsr04_init_irq();

    // IRQ global
    MAP_IntMasterEnable();

    // Estado inicial / log
    g_motion = M_STOP;
    lastA = lastB = 0;
    msg_duty(duty_user_steps[duty_idx], duty_apply_map[duty_idx]);

    // Bucle principal: disparo TRIG, manejo buzzer, log DIST
    while(1){
        // Disparo TRIG periódico
        hcsr04_trigger();

        // Buzzer
        if(buzzer_ms_left > 0){
            MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, BUZZ_PIN);
            buzzer_ms_left = (buzzer_ms_left >= LOOP_MS) ? (buzzer_ms_left - LOOP_MS) : 0;
        }else{
            MAP_GPIOPinWrite(BUZZ_PORT, BUZZ_PIN, 0);
        }


// Enfriamiento del ultrasonido (si está activo)
if (us_ignore_ms > 0) {
    us_ignore_ms = (us_ignore_ms >= LOOP_MS) ? (us_ignore_ms - LOOP_MS) : 0;
}

// Ejecutar el giro pedido por el usuario
if (g_have_turn_cmd) {
    g_have_turn_cmd = false;

    uint32_t turn_ms = ms_for_degrees(g_turn_deg);
    us_ignore_ms = (uint32_t)turn_ms + TURN_COOLDOWN_MS;

    // Freno corto
    A_brake(); B_brake();
    delay_us(80 * 1000u);

    // Lado del giro
    if (g_turn_side == 'R') {
        A_forward(); B_reverse();
    } else { // 'L'
        A_reverse(); B_forward();
    }
    set_pwm_A(TURN_A_DUTY_PCT);
    set_pwm_B(TURN_B_DUTY_PCT);

    // Mantener el giro
    delay_us(turn_ms * 1000u);

    // Freno y PWM=0
    A_brake(); B_brake();
    set_pwm_A(0);
    set_pwm_B(0);
    delay_us(50 * 1000u);

    // Reanudar según auto-run
    if (g_auto_run) { g_motion = M_FWD; } else { g_motion = M_STOP; }
    apply_motion();
}


}

// AQUI EL AUTO GIRA Y AVANZA
// Auto-run: si está habilitado y no estamos girando ni ignorando eco, avanza
// Reanuda avance solo si g_auto_run está activo
// Auto-run: si está habilitado y no estamos girando ni ignorando eco, avanza
if (g_auto_run && us_ignore_ms == 0 && !g_wait_turn && !g_have_turn_cmd) {
    if (g_motion != M_FWD && (us_last_cm < 0 || us_last_cm > STOP_CM)) {
        g_motion = M_FWD;
        apply_motion();
    }


//HASTA AQUI


        // Reporte de distancia no bloqueante
if (us_new_sample) {
    us_new_sample = false;
    msg_dist(us_last_cm);
    g_slow_pct = speed_limit_from_cm(us_last_cm);   // <-- AÑADIR
}

        if(us_hard_stop){
            us_hard_stop = false;
            uart_puts("STOP,HARD\r\n");
        }

        // ~60 ms
        delay_us(LOOP_MS * 1000u);
    }
}
