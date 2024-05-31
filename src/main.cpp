/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */
//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"
#include "arm_math_types.h"
#include "control_factory.h"
#include "transform.h"
#include "trigo.h" 
#include "ScopeMimicry.h"
#include "zephyr/console/console.h"
#include <cstdint>
// #include "shield_channels.h"
//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//--------------LOOP FUNCTIONS DECLARATION--------------------
void loop_background_task();   // Code to be executed in the background task
void loop_critical_task();     // Code to be executed in real time in the critical task
void application_task();
//--------------USER fARIABLES DECLARATIONS-------------------
#define HALL1 PA7
#define HALL2 PD2
#define HALL3 PC6

static const float32_t AC_CURRENT_LIMIT = 10.0;
static const float32_t DC_CURRENT_LIMIT = 5.0;
static const float32_t MIN_DC_VOLTAGE = 35.0; // for control
static const float32_t V_HIGH_MIN = 5.0;      // to start the POWER
static const float32_t Ts = 100.e-6F;
static const uint32_t control_task_period = (uint32_t) (Ts * 1.e6F);

/* Hall effect sensors */
static uint8_t HALL1_value;
static uint8_t HALL2_value;
static uint8_t HALL3_value;
static uint8_t angle_index;
static float32_t hall_angle;
static PllAngle pllangle = controlLibFactory.pllAngle(Ts, 10.0F, 0.03F);
static PllAngle pll_ab = controlLibFactory.pllAngle(Ts, 10.0F, 0.08F);
static PllDatas pllDatas;
static float32_t angle_filtered;
static float32_t w_meas;
static int16_t sector[] = {-1, 4, 2, 3, 0, 5, 1};
static float32_t k_calage = 10.0;

// LEG meas
static float32_t meas_data;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I1_offset;
static float32_t I2_offset;
static float32_t tmpI1_offset;
static float32_t tmpI2_offset;
static float32_t nb_offset;

// DC meas
static float32_t I_high;
static float32_t V_high;
static float32_t Tq_meas;
static float32_t assist_torque_coef;

/* Pedal Incremental encoder */
static float32_t ab_sector[] = {0.F, 1.F, 3.F, 2.F};
static uint8_t PA2_value;
static uint8_t PA3_value;
static float32_t ab_value;
static float32_t ab_pulsation;
static float32_t ab_angle;
static PllDatas ab_pll_datas;

/* Three phase system and dqo */
static three_phase_t Vabc;
static three_phase_t duty_abc;
static float32_t duty_ramp = 0.1;
static three_phase_t Iabc;
static dqo_t Vdq;
static dqo_t Idq;
static dqo_t Idq_ref;
static float32_t angle_4_control;

/* variables used to get static value for scopemimicry */
// static float32_t comp_dt = 0.01;
static float32_t duty_a, duty_b;
static float32_t Va;
static float32_t Iq_meas;
static float32_t Iq_ref;
static float32_t Iq_max;
static float32_t Vd, Vq;

static float32_t manual_Iq_ref;

enum regulation_mode {
    TORQUE_MODE=0,
    ASSIST_MODE=1
} regulation_asked;


static LowPassFirstOrderFilter w_ref_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static LowPassFirstOrderFilter vHigh_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);
static LowPassFirstOrderFilter w_mes_filter = controlLibFactory.lowpassfilter(Ts, 1.0e-3F);
static LowPassFirstOrderFilter tq_mes_filter = controlLibFactory.lowpassfilter(Ts, 1000.0e-3F); // 1s to filter a lot.
static LowPassFirstOrderFilter ab_pulse_filter = controlLibFactory.lowpassfilter(Ts, 100.0e-3F);
static float32_t V_high_filtered;
static float32_t inverse_Vhigh;

/*--- PID ---*/
static float32_t Kp = 15*0.035;
static float32_t Ti = 0.005029;
static float32_t Td = 0.0;
static float32_t N = 1.0;
static float32_t lower_bound = -MIN_DC_VOLTAGE * 0.5;
static float32_t upper_bound = MIN_DC_VOLTAGE * 0.5;
static Pid pi_d = controlLibFactory.pid(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
static Pid pi_q = controlLibFactory.pid(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);

// transform electrical pulsation to linear speed in km/h
// to_kmh = (1.0/45.0*26*0.0254)/1000.0*3600.0; 
// 45 pair of poles
// 1 wheel of 26 inches (1 inch = 2.54cm)

const float32_t to_kmh = 0.02641;
const static uint32_t decimation = 20;
static uint32_t counter_time;

static float32_t w_estimate;
uint8_t received_serial_char;
enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

enum control_state_mode {
    OFFSET_ST = 0,
    IDLE_ST = 1,
    STARTUP_ST = 2,
    POWER_ST = 3,
    ERROR_ST = 4
};

enum control_state_mode control_state;

static uint16_t error_counter;
static bool pwm_enable;
uint8_t asked_mode = IDLEMODE;

void init_filt_and_reg(void) {
    pllangle.reset(0.F);
    pll_ab.reset(ab_value);
    vHigh_filter.reset(V_HIGH_MIN);
    w_ref_filter.reset(0.0);
    tq_mes_filter.reset(1500.0);
    pi_d.reset();
    pi_q.reset();
    error_counter = 0;
}

ScopeMimicry scope(512, 11);
static bool is_downloading;

bool mytrigger() {
    return ((ab_pulsation > 2.0 || ab_pulsation < -2.0) && control_state == POWER_ST);
}

void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    uint16_t buffer_size = scope.get_buffer_size() >> 2; // we divide by 4 (4 bytes per float data) 
    printk("begin record\n");
    printk("#");
    for (uint16_t k=0;k < scope.get_nb_channel(); k++) {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k=0;k < buffer_size; k++) {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

/* 
 * pulsation_estimator(sector, time)
 *
 * assume sector in integer in [0, 5] 
 */

float32_t pulsation_estimator(int16_t sector, float32_t time) {
    static float32_t w_estimate_intern = 0.0;
    static int16_t prev_sector;
    static float32_t prev_time = 0.0;
    int16_t delta_sector;
    float32_t sixty_degre_step_time;
    delta_sector = sector - prev_sector;
    prev_sector = sector; 
    if (delta_sector == 1 || delta_sector == -5) {
        // positive speed
        sixty_degre_step_time = (time - prev_time);
        w_estimate_intern = (PI/3.0) / sixty_degre_step_time;   
        prev_time = time;
    } 
    if (delta_sector == -1 || delta_sector == 5) {
        sixty_degre_step_time = (time - prev_time);
        w_estimate_intern = (-PI/3.0) / sixty_degre_step_time;
        prev_time = time;
    }
    return w_estimate_intern;
}

static __inline__ float32_t dead_time_comp(float32_t I, float32_t comp_value)
{
    float32_t dt;
    dt = comp_value * I;
    if (dt > comp_value) 
    {
        dt = comp_value;
    } 
    else if (I < -comp_value)
    {
        dt = -comp_value;
    }
    return dt;
}

inline void retrieve_analog_datas() {
    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE) I1_low_value = meas_data + I1_offset;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE) I2_low_value = meas_data + I2_offset;

    if (control_state == OFFSET_ST && counter_time < 2000) {
        tmpI1_offset += I1_low_value; 
        tmpI2_offset += I2_low_value; 
    }

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE) V_high = meas_data;

    meas_data = data.getLatest(I_HIGH);
    if (meas_data != NO_VALUE) I_high = -meas_data; // normal when we see the kicad

    meas_data = data.getLatest(ANALOG_SIN);
    // if (meas_data != NO_VALUE) Tq_meas = meas_data;

    meas_data = data.getLatest(ANALOG_COS);
    if (meas_data != NO_VALUE) Tq_meas = meas_data;
    
    Tq_meas = tq_mes_filter.calculateWithReturn(Tq_meas);
    // Mesure de Vhigh
    V_high_filtered = vHigh_filter.calculateWithReturn(V_high);
}

inline void get_position_and_speed() {
    //reconstitution de l'index à partir de la lecture
    HALL1_value = spin.gpio.readPin(HALL1);
    HALL2_value = spin.gpio.readPin(HALL2);
    HALL3_value = spin.gpio.readPin(HALL3);
    angle_index = HALL3_value*4 + HALL2_value*2 + HALL1_value*1;

    hall_angle = ot_modulo_2pi(PI / 3.0 * sector[angle_index] + PI * k_calage / 12.0);
    w_estimate = pulsation_estimator(sector[angle_index], counter_time*Ts);
    pllDatas = pllangle.calculateWithReturn(hall_angle);

    angle_filtered = pllDatas.angle;
    w_meas = w_mes_filter.calculateWithReturn(pllDatas.w);
}

inline void get_pedal_speed() {

    PA2_value = spin.gpio.readPin(PA2);
    PA3_value = spin.gpio.readPin(PA3);
    ab_value = 2.0F * PI / 4.0F * ab_sector[(PA2_value * 2 + PA3_value * 1)];
    ab_pll_datas = pll_ab.calculateWithReturn(ab_value);
    ab_pulsation = ab_pulse_filter.calculateWithReturn(ab_pll_datas.w);
}

inline void overcurrent_mngt() {
    if (I1_low_value > AC_CURRENT_LIMIT || I1_low_value < -AC_CURRENT_LIMIT || 
        I2_low_value > AC_CURRENT_LIMIT || I2_low_value < -AC_CURRENT_LIMIT || 
        I_high > DC_CURRENT_LIMIT) 
    {
            error_counter++;
    }
    if (error_counter > 2) 
    {
        control_state = ERROR_ST;
    }
}

inline void stop_pwm_and_reset_regulation() {
    if (pwm_enable == true) {
        twist.stopAll();
        // reset filters and pid
        init_filt_and_reg();
        pwm_enable = false;
    }
}

inline void control_torque() {
    angle_4_control = angle_filtered;
    if (regulation_asked == TORQUE_MODE)
        Idq_ref.q = manual_Iq_ref;
    else
        Idq_ref.q = 0.001 * (Tq_meas - 1600.0) * assist_torque_coef;

    if (Idq_ref.q > Iq_max) {
        Idq_ref.q = Iq_max;
    }
    if (Idq_ref.q < 0.0)
    {
        Idq_ref.q = 0.0;
    } 
    if (ab_pulsation < 5.0 && regulation_asked == ASSIST_MODE) 
    {
        Idq_ref.q = 0.0;
    }
    Idq_ref.d = 0.0;
    Iabc.a = I2_low_value; 
    Iabc.b = I1_low_value;
    Iabc.c = -(Iabc.a + Iabc.b);
    Idq = Transform::to_dqo(Iabc, angle_4_control);
    Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
    Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);
    Vdq.o = 0.0F;
    Vabc = Transform::to_threephase(Vdq, angle_4_control);

}

inline void compute_duties() {
    inverse_Vhigh = 1.0/ MIN_DC_VOLTAGE;
    duty_abc.a = (Vabc.a * 0.5 * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc.a, comp_dt); 
    duty_abc.b = (Vabc.b * 0.5 * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc.b, comp_dt);
    duty_abc.c = (Vabc.c * 0.5 * inverse_Vhigh + 0.5); // + dead_time_comp(Iabc.c, comp_dt);

}

inline void apply_duties() {
    twist.setLegDutyCycle(LEG1, duty_abc.a);
    twist.setLegDutyCycle(LEG2, duty_abc.b);
    twist.setLegDutyCycle(LEG3, duty_abc.c);
}

void start_pwms() {
    if (!pwm_enable)
    {
        pwm_enable = true;
        twist.startAll();
    }
}

void make_duty_ramp() {
    duty_ramp +=  0.5F / 1000.0F;
    if (duty_ramp > 0.5)
        duty_ramp = 0.5;
    if (duty_ramp < 0.1)
        duty_ramp = 0.1;
    duty_abc.a = duty_ramp; 
    duty_abc.b = duty_ramp; 
    duty_abc.c = duty_ramp;
}

void config_adcs() {
    spin.adc.configureTriggerSource(1, hrtim_ev1);
    spin.adc.configureTriggerSource(2, hrtim_ev3);
    spin.adc.configureTriggerSource(3, software);
    spin.adc.configureTriggerSource(4, software);
    spin.adc.configureTriggerSource(5, software);

    spin.adc.configureDiscontinuousMode(1,1);
    spin.adc.configureDiscontinuousMode(2, 1);

    data.enableShieldChannel(1, I1_LOW);
    data.enableShieldChannel(1, I_HIGH);
    data.enableShieldChannel(1, V_HIGH);

    data.enableShieldChannel(2, I2_LOW);
    data.enableShieldChannel(2, ANALOG_SIN);
    data.enableShieldChannel(2, ANALOG_COS);
}

void init_constant() {
    /* to count the time */
    counter_time = 0; 
    /* variable for measurements */
    I1_low_value = 0.0;
    I2_low_value = 0.0;
    I_high = 0.0;
    V_high = 0.0;
    /* state view of the pwm */
    pwm_enable = false;
    /* idle or power mode*/
    asked_mode = IDLEMODE;
    /* we begin to measure the current offset before all */
    control_state = OFFSET_ST;
    I1_offset = 0.0;
    I2_offset = 0.0;
    tmpI1_offset = 0.0;
    tmpI2_offset = 0.0;
    /* number of data used to make offset */
    nb_offset = 2000;
    /* */
    assist_torque_coef = 3.0;
    Iq_max = 2.0;
    manual_Iq_ref = 0.0;
    regulation_asked = ASSIST_MODE;
}
//--------------SETUP FUNCTIONS-------------------------------
/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    // Setup the hardware first
    // spin.version.setBoardVersion(TWIST_v_1_1_3);
    twist.setVersion(shield_ownverter);
    twist.initAllBuck();
    config_adcs();
    data.setParameters(I1_LOW, 0.005, -10);
    data.setParameters(I2_LOW, 0.005, -10.0);
    data.setParameters(I_HIGH, 0.005, -10.0);
    data.setParameters(V_HIGH, 0.0666, 0.0);

    spin.gpio.configurePin(HALL1, INPUT);
    spin.gpio.configurePin(HALL2, INPUT);
    spin.gpio.configurePin(HALL3, INPUT);
    spin.gpio.configurePin(PA3, INPUT);
    spin.gpio.configurePin(PA2, INPUT);

    scope.connectChannel(V_high, "V_high");
    scope.connectChannel(V_high_filtered, "V_high_filt");
    scope.connectChannel(ab_value, "ab_value");
    scope.connectChannel(ab_pulsation, "V_ab_pulse");
    scope.connectChannel(ab_angle, "ab_angle");
    scope.connectChannel(Iq_ref, "I_qref");
    scope.connectChannel(Iq_meas, "I_qmeas");
    scope.connectChannel(angle_filtered, "angle_filtered");
    scope.connectChannel(hall_angle, "hall_angle");
    scope.connectChannel(Vq, "V_q");
    scope.connectChannel(Vd, "V_d");
    scope.set_trigger(&mytrigger);
    scope.set_delay(0.0);
    scope.start();
    init_filt_and_reg();
    init_constant();
    spin.led.turnOn();
    // Then declare tasks
    uint32_t background_task_number = task.createBackground(loop_background_task);
    uint32_t app_task_number = task.createBackground(application_task);
    task.createCritical(loop_critical_task, control_task_period); // Uncomment if you use the critical task
    // Finally, start tasks
    task.startBackground(background_task_number);
    task.startBackground(app_task_number);
    task.startCritical(); // Uncomment if you use the critical task
}
//--------------LOOP FUNCTIONS--------------------------------
void loop_background_task()
{
    // Task content
    received_serial_char = console_getchar();
    switch (received_serial_char) {
        case 'h':
            printk("help");
            break;
        case 'p':
            printk("power asked");
            asked_mode = POWERMODE;
            scope.start();
            break;
        case 'i':
            printk("idle asked");
            asked_mode = IDLEMODE;
            break;
        case 'r':
            is_downloading = true;
        case 'u':
            assist_torque_coef += 1.0F;
            break;
        case 'd':
            assist_torque_coef -= 1.0F;
            break;
        case 'j':
            Iq_max += 0.5;
            manual_Iq_ref = Iq_max;
            break;
        case 'k':
            Iq_max -= 0.5;
            manual_Iq_ref = Iq_max;
            break;

        case 'a': 
            // TO SWITCH BETWEEN ASSIST AND TORQUE MODE.
            // IN TORQUE MODE IQ_MAX DRIVE
            if (regulation_asked == TORQUE_MODE) {
                regulation_asked = ASSIST_MODE;
                printk("ASSIST\n");
            }
            else {
                regulation_asked = TORQUE_MODE;
                Iq_max = 0.0;
                manual_Iq_ref = 0.0;
                printk("TORQUE\n");
            }

            break;
        case 't':
            k_calage += 1.0;
            break;

        case 'g':
            k_calage -= 1.0;
            break;
    }

    // Pause between two runs of the task
}

void application_task() {

    printk("%.2f:", I_high);
    printk("%.2f:", V_high_filtered);
    printk("%.2f:", ab_pulsation);
    printk("%.2f:", w_estimate);
    printk("%.2f:", Iq_ref);
    printk("%d:", scope.has_trigged());
    printk("%.0f:", k_calage);
    printk("%.2f:", Iq_meas);
    printk("%.2f:", Iq_max);
    printk("%d\n", control_state);

    if (is_downloading) {
        dump_scope_datas(scope);
        is_downloading = false;
    }
    switch (control_state) {
        case OFFSET_ST:
            if (counter_time > 2000) {
                spin.led.turnOff();
                I1_offset = - tmpI1_offset / 2000.0;
                I2_offset = - tmpI2_offset / 2000.0;
                control_state = IDLE_ST;
            }
        break;

        case IDLE_ST:
            if ((asked_mode == POWERMODE) && (V_high_filtered > V_HIGH_MIN)) {
                control_state = STARTUP_ST;
            }
        break;

        case STARTUP_ST:
            if (duty_ramp == 0.5) control_state = POWER_ST;
        break;

        case POWER_ST:
            if (asked_mode == IDLEMODE) {
                control_state = IDLE_ST;
            }
        break;

        case ERROR_ST:
            if (asked_mode == IDLEMODE)
                error_counter = 0;
            control_state = IDLE_ST;
            break;
    }

    task.suspendBackgroundMs(250);
}

void loop_critical_task()
{
    counter_time++;

    retrieve_analog_datas();

    get_position_and_speed();

    get_pedal_speed(); 

    overcurrent_mngt();

    switch (control_state) {
        case OFFSET_ST:
            stop_pwm_and_reset_regulation();
        break;
        case IDLE_ST:
            stop_pwm_and_reset_regulation();
            break;
        case ERROR_ST:
            stop_pwm_and_reset_regulation();
            break;
        case POWER_ST:
            control_torque();
            compute_duties();
            apply_duties();
            break;
        case STARTUP_ST:
            start_pwms();
            make_duty_ramp();
            apply_duties();
            break;
    }

    if (counter_time%decimation == 0)
    {
        Va = Vabc.a;
        duty_a = duty_abc.a;
        duty_b = duty_abc.b;
        Iq_ref = Idq_ref.q;
        Iq_meas = Idq.q;
        Vd = Vdq.d;
        Vq = Vdq.q;
        ab_angle = ab_pll_datas.angle;
        scope.acquire();
    }
}

int main(void)
{
    setup_routine();

    return 0;
}
