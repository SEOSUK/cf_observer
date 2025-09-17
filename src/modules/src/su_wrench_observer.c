#include "su_wrench_observer.h"
#include "su_params.h"
#include "platform_defaults.h"   // ★ THRUST_MAX, THRUST2TORQUE, ARM_LENGTH
#include "log.h"
#include "param.h"
#include "debug.h"
#include "config.h"
#include "pm.h"                  // ★ 배터리 전압 읽기
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"



// ==============================
// SU: 내부 상태 (로그 대상)
// ==============================
static float su_body_input_Force[3];    // [Fx, Fy, Fz] (Body)
static float su_world_input_Force[3];   // [Fx, Fy, Fz] (World)
static float su_body_input_Torque[3];   // [Tx, Ty, Tz] (Body)


// === MOB 추정치 (로그 대상) ===
static float su_world_F_hat_ext[3];     // [Fx, Fy, Fz] (World)
static float su_body_Tau_hat_ext[3];     // [Tx, Ty, Tz] (body)
static float EE_world_F_hat[3];

static float r6[6], wext_hat_raw6[6];
static float wrench_conversion_checker;   // RMSE of torque-consistency
// for logging/inspection (world frame)
static float tau_world_meas[3] = {0}, tau_world_pred[3] = {0};
// EWMA(=IIR) for RMSE^2
static float wrench_mse_state = 0.0f;
static const float WRENCH_RMSE_ALPHA = 0.01f;  // 느린 평균(필요시 su_params로 뺄 수 있음)
// === 6D MOB 내부상태: p̂ = [m v_W ; J ω_B] ===
static float su_p6_hat[6] = {0};        // p̂ = [px,py,pz,Lx,Ly,Lz]


// === 실제 전압 / LPF 상태 ===
static float su_vbat_filt = 4.0f;       // 초기값: 적당한 시작 전압


// 로깅: 필터된 전압
static float su_vbat_log = 0.0f;


// === 시간 측정 & dt LPF ===           // ★★★ [ADD] dt LPF 관련
static TickType_t su_prev_tick;
static float su_dt_filt = 0.004f;     // [s] 250 Hz 기준, 모니터링용 필터된 dt
static float su_dt_mon  = 0.004f;     // 로깅용(필터된 dt)
static float su_dt_raw  = 0.004f;     // 로깅용(생 dt)
// 제어/적분용 고정 dt
static uint8_t su_dt_fixed_enable = 1;     // 1=고정 dt 사용
static float   su_dt_fixed_value  = 0.004f; // [s] 250 Hz


// === MOB 내부 상태 (추정량) 초기값 권장 ===   // ★★★ [ADD] 초기화용 (이미 선언된 p_hat, L_hat은 그대로 사용)
static inline float lpf1(float y_prev, float x, float alpha) {
    return y_prev + alpha * (x - y_prev);
}


void suWrenchObserverInit(void) {
    su_body_input_Force[0] = su_body_input_Force[1] = su_body_input_Force[2] = 0.0f;
    su_world_input_Force[0] = su_world_input_Force[1] = su_world_input_Force[2] = 0.0f;
    su_body_input_Torque[0] = su_body_input_Torque[1] = su_body_input_Torque[2] = 0.0f;

    // 초기 전압 샘플로 LPF 초기화
    float v0 = pmGetBatteryVoltage();           // [V]
    // 만약 pmGetBatteryVoltageMV()만 있다면: v0 = pmGetBatteryVoltageMV() * 0.001f;
    if (isfinite(v0) && v0 > 0.5f) {
        su_vbat_filt = v0;
    }
    su_vbat_log = su_vbat_filt;


    // ★★★ [ADD] MOB 내부상태/출력 0으로
    for (int i=0; i<3; ++i) {
        su_world_F_hat_ext[i] = 0.0f;
        su_body_Tau_hat_ext[i] = 0.0f;
    }

    for (int i = 0; i < 6; ++i) {
        su_p6_hat[i] = 0.0f;
    }

    // ★★★ [ADD] dt LPF용 초기 타임스탬프/값
    su_prev_tick = xTaskGetTickCount();
    su_dt_filt = su_dt_fixed_value;   // 모니터 초기값
    su_dt_mon  = su_dt_filt;

    su_dt_raw  = su_dt_filt;
    DEBUG_PRINT("SU Wrench observer initialized (using su_params)\n");
}


void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_pwm_t *motorPwm,
                            const Axis3f *gyro_deg_s){


    // ---- (-1) dt 측정 + 1차 LPF
    // 기존 (usecTimestamp / usecElapsedSince 사용) 블록을 아래로 교체
    TickType_t now = xTaskGetTickCount();   // 1초에 1000씩 오름.
    TickType_t dt_ticks = now - su_prev_tick;
    su_prev_tick = now;

    // FreeRTOS tick -> seconds (모니터링용)
    float dt_meas = (float)dt_ticks * ((float)portTICK_PERIOD_MS * 1e-3f);
    if (isfinite(dt_meas) && dt_meas > 0.0f && dt_meas < 0.05f) {
        su_dt_filt = lpf1(su_dt_filt, dt_meas, su_dt_alpha);  // su_dt_alpha from su_params
        if (su_dt_filt < 1e-4f) su_dt_filt = 1e-4f;
        if (su_dt_filt > 5e-2f) su_dt_filt = 5e-2f;
        su_dt_raw = dt_meas;
    }
    su_dt_mon = su_dt_filt; // 로그에만 사용
    

    // ---- 0) 실제 전압 읽기 + LPF
    float v_meas = pmGetBatteryVoltage();       // [V]
    // 만약 pmGetBatteryVoltageMV()만 있다면: v_meas = pmGetBatteryVoltageMV() * 0.001f;

    // 1차 IIR LPF
    su_vbat_filt += su_vbat_alpha * (v_meas - su_vbat_filt);  // su_vbat_alpha from su_params
    su_vbat_log = su_vbat_filt;
    // ---- 1) PWM → Force[N]  (power_distribution와 동일 스케일)
    const float pwm2N = (THRUST_MAX / (float)UINT16_MAX);

    // ★ 실제 전압을 반영한 모델
    const float voltage_model = su_voltage_model_a * su_vbat_filt - su_voltage_model_b; // from su_params
    const float f1 = voltage_model * pwm2N * (float)motorPwm->motors.m1;
    const float f2 = voltage_model * pwm2N * (float)motorPwm->motors.m2;
    const float f3 = voltage_model * pwm2N * (float)motorPwm->motors.m3;
    const float f4 = voltage_model * pwm2N * (float)motorPwm->motors.m4;

    // ---- 2) Body-frame Force
    const float Fz = f1 + f2 + f3 + f4;
    su_body_input_Force[0] = 0.0f;
    su_body_input_Force[1] = 0.0f;
    su_body_input_Force[2] = Fz;

    // ---- 3) Body-frame Torque (정확한 역-할당)
    const float arm = 0.707106781f * ARM_LENGTH; // r / sqrt(2)

    const float Tx = arm * ((f3 + f4) - (f1 + f2));
    const float Ty = arm * ((f2 + f3) - (f1 + f4));
    const float Tz = THRUST2TORQUE * (-f1 + f2 - f3 + f4);

    su_body_input_Torque[0] = Tx;
    su_body_input_Torque[1] = Ty;
    su_body_input_Torque[2] = Tz;

    // ---- 4) Body → World (자세 쿼터니언 → 회전행렬 R_B2W)
    float R11,R12,R13,R21,R22,R23,R31,R32,R33;
    {
        float qx = state->attitudeQuaternion.x;
        float qy = state->attitudeQuaternion.y;
        float qz = state->attitudeQuaternion.z;
        float qw = state->attitudeQuaternion.w;

        const float n = sqrtf(qx*qx + qy*qy + qz*qz + qw*qw);
        if (n > 1e-6f) { qx/=n; qy/=n; qz/=n; qw/=n; }

        const float xx = qx*qx, yy = qy*qy, zz = qz*qz;
        const float xy = qx*qy, xz = qx*qz, yz = qy*qz;
        const float xw = qx*qw, yw = qy*qw, zw = qz*qw;

        R11 = 1.0f - 2.0f*(yy + zz);
        R12 = 2.0f*(xy - zw);
        R13 = 2.0f*(xz + yw);
        R21 = 2.0f*(xy + zw);
        R22 = 1.0f - 2.0f*(xx + zz);
        R23 = 2.0f*(yz - xw);
        R31 = 2.0f*(xz - yw);
        R32 = 2.0f*(yz + xw);
        R33 = 1.0f - 2.0f*(xx + yy);

        // body F = [0,0,Fz]_B → world
        su_world_input_Force[0] = R13 * Fz;
        su_world_input_Force[1] = R23 * Fz;
        su_world_input_Force[2] = R33 * Fz;
    }




    // =========================
    // 6D MCG-MOB (World force + Body torque, 슬라이드 식 (a)~(i))
    // p := M v,   p̂˙ = u - C(q,v) - G(q) + K*(p - p̂)
    // \hat{w}_ext = K*(p - p̂)
    // =========================
    // v_W (m/s)
    float vx = state->velocity.x, vy = state->velocity.y, vz = state->velocity.z;
    if (!isfinite(vx)) vx = 0.0f;
    if (!isfinite(vy)) vy = 0.0f;
    if (!isfinite(vz)) vz = 0.0f;    
    
    // ω_B (rad/s)
    const float DEG2RAD = 0.017453292519943295f;
    float wx = gyro_deg_s ? (gyro_deg_s->x * DEG2RAD) : 0.0f;
    float wy = gyro_deg_s ? (gyro_deg_s->y * DEG2RAD) : 0.0f;
    float wz = gyro_deg_s ? (gyro_deg_s->z * DEG2RAD) : 0.0f;
    if (!isfinite(wx)) wx = 0.0f;
    if (!isfinite(wy)) wy = 0.0f;
    if (!isfinite(wz)) wz = 0.0f;


    // p = M v
    const float Ix = Jxx, Iy = Jyy, Iz = Jzz;  // inertias from su_params
    float p6[6] = { su_mass*vx, su_mass*vy, su_mass*vz, Ix*wx, Iy*wy, Iz*wz }; // mass from su_params

    // u = [F_W ; τ_B]
    float u6[6] = {
    su_world_input_Force[0], su_world_input_Force[1], su_world_input_Force[2],
    su_body_input_Torque[0], su_body_input_Torque[1], su_body_input_Torque[2]
    };

    // C(q,v) = [0; ω×(Jω)]   (이름 충돌 방지: C6/G6 -> cori6/grav6)
    const float Iw_x = Ix*wx, Iw_y = Iy*wy, Iw_z = Iz*wz;
    float cori6[6] = { 0,0,0,
    (wy*Iw_z - wz*Iw_y), (wz*Iw_x - wx*Iw_z), (wx*Iw_y - wy*Iw_x)
    };

    // G(q) = [0,0, m g, 0,0,0]
    const float GRAV = 9.80665f;
    float grav6[6] = { 0,0, su_mass*GRAV, 0,0,0 };

    // r = p - p̂,   \hat{w}_ext = K * r
    for (int i=0;i<6;i++) r6[i] = p6[i] - su_p6_hat[i];
    wext_hat_raw6[0] = su_Kf   * r6[0];   // Kf from su_params
    wext_hat_raw6[1] = su_Kf   * r6[1];
    wext_hat_raw6[2] = su_Kf   * r6[2];
    wext_hat_raw6[3] = su_Ktau * r6[3];  // Ktau from su_params
    wext_hat_raw6[4] = su_Ktau * r6[4];
    wext_hat_raw6[5] = su_Ktau * r6[5];

    // deadzone(옵션)
    for (int i=0;i<3;i++)
    {
        float v=wext_hat_raw6[i],a=su_deadzone_F;  
        wext_hat_raw6[i]=(fabsf(v)<=a)?0.0f:(v>0?v-a:v+a); 
    }

    for (int i=3;i<6;i++)
    { 
        float v=wext_hat_raw6[i],a=su_deadzone_T;  
        wext_hat_raw6[i]=(fabsf(v)<=a)?0.0f:(v>0?v-a:v+a); 
    }


    float p6_hat_dot[6];
    // 적분용 dt: 고정 사용(기본 0.004s) — 루프 밖에서 한 번만 결정
    const float dt = su_dt_fixed_enable ? su_dt_fixed_value : su_dt_filt;
    for (int i = 0; i < 6; i++) {
      p6_hat_dot[i] = u6[i] - cori6[i] - grav6[i] + wext_hat_raw6[i];
      su_p6_hat[i] += dt * p6_hat_dot[i];
    }

    // 출력 LPF
    su_world_F_hat_ext[0] = lpf1(su_world_F_hat_ext[0], wext_hat_raw6[0], su_mob_alpha); // from su_params
    su_world_F_hat_ext[1] = lpf1(su_world_F_hat_ext[1], wext_hat_raw6[1], su_mob_alpha);
    su_world_F_hat_ext[2] = lpf1(su_world_F_hat_ext[2], wext_hat_raw6[2], su_mob_alpha);
    su_body_Tau_hat_ext[0]= lpf1(su_body_Tau_hat_ext[0], wext_hat_raw6[3], su_mob_alpha);
    su_body_Tau_hat_ext[1]= lpf1(su_body_Tau_hat_ext[1], wext_hat_raw6[4], su_mob_alpha);
    su_body_Tau_hat_ext[2]= lpf1(su_body_Tau_hat_ext[2], wext_hat_raw6[5], su_mob_alpha);    




    // ===== Wrench conversion & consistency (EE torque = 0 가정) =====
    // EE가 환경에 "가하는" 힘(로그용): 드론이 받은 외력의 반대 부호
    EE_world_F_hat[0] = -su_world_F_hat_ext[0];
    EE_world_F_hat[1] = -su_world_F_hat_ext[1];
    EE_world_F_hat[2] = -su_world_F_hat_ext[2];

    // 모델 일관성 검증은 환경->EE 힘 f_E^W 를 사용 (드론이 받은 외력과 동일 부호)
    const float fEw_x = su_world_F_hat_ext[0];
    const float fEw_y = su_world_F_hat_ext[1];
    const float fEw_z = su_world_F_hat_ext[2];

    // 레버암 r_B (body) -> r_W = R * r_B   (Offset_*는 su_params.c에서 제공)
    const float rBx = Offset_x, rBy = Offset_y, rBz = Offset_z;
    const float rWx = R11*rBx + R12*rBy + R13*rBz;
    const float rWy = R21*rBx + R22*rBy + R23*rBz;
    const float rWz = R31*rBx + R32*rBy + R33*rBz;

    // 예측 토크(월드): tau_pred = r_W × f_E^W
    tau_world_pred[0] = rWy*fEw_z - rWz*fEw_y;
    tau_world_pred[1] = rWz*fEw_x - rWx*fEw_z;
    tau_world_pred[2] = rWx*fEw_y - rWy*fEw_x;

    // 측정 토크(바디) -> 월드: tau_meas^W = R * tau_meas^B
    const float tauBx = su_body_Tau_hat_ext[0];
    const float tauBy = su_body_Tau_hat_ext[1];
    const float tauBz = su_body_Tau_hat_ext[2];
    tau_world_meas[0] = R11*tauBx + R12*tauBy + R13*tauBz;
    tau_world_meas[1] = R21*tauBx + R22*tauBy + R23*tauBz;
    tau_world_meas[2] = R31*tauBx + R32*tauBy + R33*tauBz;

    // 잔차 및 EWMA RMSE
    const float ex = tau_world_meas[0] - tau_world_pred[0];
    const float ey = tau_world_meas[1] - tau_world_pred[1];
    const float ez = tau_world_meas[2] - tau_world_pred[2];
    const float err2 = ex*ex + ey*ey + ez*ez;     // [N^2·m^2]
    wrench_mse_state = (1.0f - WRENCH_RMSE_ALPHA) * wrench_mse_state
                     +        WRENCH_RMSE_ALPHA  * err2;
    wrench_conversion_checker = sqrtf(wrench_mse_state); // [N·m]

}


// 로그 그룹
LOG_GROUP_START(suWrenchObs)
LOG_ADD(LOG_FLOAT, suFx,  &su_body_input_Force[0])
LOG_ADD(LOG_FLOAT, suFy,  &su_body_input_Force[1])
LOG_ADD(LOG_FLOAT, suFz,  &su_body_input_Force[2])
LOG_ADD(LOG_FLOAT, suTx,  &su_body_input_Torque[0])
LOG_ADD(LOG_FLOAT, suTy,  &su_body_input_Torque[1])
LOG_ADD(LOG_FLOAT, suTz,  &su_body_input_Torque[2])
LOG_ADD(LOG_FLOAT, suWFx, &su_world_input_Force[0])
LOG_ADD(LOG_FLOAT, suWFy, &su_world_input_Force[1])
LOG_ADD(LOG_FLOAT, suWFz, &su_world_input_Force[2])

LOG_ADD(LOG_FLOAT, suV,   &su_vbat_log)                  // 필터링 된 전압 로깅 (로패필 거침)


// ★★★ [ADD] dt 로깅 & MOB 출력
LOG_ADD(LOG_FLOAT, suDt,     &su_dt_mon)                 // 모니터링용 필터된 dt
LOG_ADD(LOG_FLOAT, suDtraw,  &su_dt_raw)                 // 생 dt

LOG_ADD(LOG_FLOAT, suFextX_raw, &wext_hat_raw6[0])      // World 힘 추정 (데드존 없음)
LOG_ADD(LOG_FLOAT, suFextY_raw, &wext_hat_raw6[1])
LOG_ADD(LOG_FLOAT, suFextZ_raw, &wext_hat_raw6[2])

LOG_ADD(LOG_FLOAT, suFextX, &su_world_F_hat_ext[0])     // World 힘 추정 (데드존 있음)
LOG_ADD(LOG_FLOAT, suFextY, &su_world_F_hat_ext[1])
LOG_ADD(LOG_FLOAT, suFextZ, &su_world_F_hat_ext[2])

LOG_ADD(LOG_FLOAT, suTextX_raw, &wext_hat_raw6[3])   // Body 토크 추정 (데드존 없음)
LOG_ADD(LOG_FLOAT, suTextY_raw, &wext_hat_raw6[4])
LOG_ADD(LOG_FLOAT, suTextZ_raw, &wext_hat_raw6[5])

LOG_ADD(LOG_FLOAT, suTextX, &su_body_Tau_hat_ext[0])     // body 토크 (데드존 있음)
LOG_ADD(LOG_FLOAT, suTextY, &su_body_Tau_hat_ext[1])
LOG_ADD(LOG_FLOAT, suTextZ, &su_body_Tau_hat_ext[2])


LOG_ADD(LOG_FLOAT, EE_world_Fx, &EE_world_F_hat[0])    // "엔드이펙터가 환경에: 내고 있는 힘 !!!!" (데드존 없음)
LOG_ADD(LOG_FLOAT, EE_world_Fy, &EE_world_F_hat[1])
LOG_ADD(LOG_FLOAT, EE_world_Fz, &EE_world_F_hat[2])

LOG_ADD(LOG_FLOAT, wrench_conversion_checker, &wrench_conversion_checker)     // 외란관측 신뢰도: 엔드이펙터에 가해진 토크가 정말 0인지 확인.

LOG_GROUP_STOP(suWrenchObs)
