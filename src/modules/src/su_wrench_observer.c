#include "su_wrench_observer.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "config.h"
#include "platform_defaults.h"   // ★ THRUST_MAX, THRUST2TORQUE, ARM_LENGTH
#include <math.h>

// ==============================
// SU: 내부 상태 (로그 대상)
// ==============================
static float su_body_input_Force[3];    // [Fx, Fy, Fz] (Body)
static float su_world_input_Force[3];   // [Fx, Fy, Fz] (World)
static float su_body_input_Torque[3];   // [Tx, Ty, Tz] (Body)

// 옵셔널 미세보정 (기본 1.0)
static float su_k_force_scale = 1.0f;   // MJSEUK TODO. 이거 바꿔야할수도있음.

void suWrenchObserverInit(void) {
    su_body_input_Force[0] = su_body_input_Force[1] = su_body_input_Force[2] = 0.0f;
    su_world_input_Force[0] = su_world_input_Force[1] = su_world_input_Force[2] = 0.0f;
    su_body_input_Torque[0] = su_body_input_Torque[1] = su_body_input_Torque[2] = 0.0f;
    DEBUG_PRINT("SU Wrench observer initialized\n");
}

void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_pwm_t *motorPwm) {
    // ---- 1) PWM → Force[N]  (power_distribution와 동일 스케일)
    const float pwm2N = (THRUST_MAX / (float)UINT16_MAX) * su_k_force_scale;

    const float f1 = pwm2N * (float)motorPwm->motors.m1;
    const float f2 = pwm2N * (float)motorPwm->motors.m2;
    const float f3 = pwm2N * (float)motorPwm->motors.m3;
    const float f4 = pwm2N * (float)motorPwm->motors.m4;

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

    // ---- 4) Body → World (자세 쿼터니언)
    {
        float qx = state->attitudeQuaternion.x;
        float qy = state->attitudeQuaternion.y;
        float qz = state->attitudeQuaternion.z;
        float qw = state->attitudeQuaternion.w;

        const float n = sqrtf(qx*qx + qy*qy + qz*qz + qw*qw);
        if (n > 1e-6f) { qx/=n; qy/=n; qz/=n; qw/=n; }

        // 필요한 항만 계산 (unused 방지)
        const float xx = qx*qx;
        const float yy = qy*qy;
        const float xz = qx*qz;
        const float yz = qy*qz;
        const float xw = qx*qw;
        const float yw = qy*qw;

        const float R13 = 2.0f*(xz + yw);
        const float R23 = 2.0f*(yz - xw);
        const float R33 = 1.0f - 2.0f*(xx + yy);

        su_world_input_Force[0] = R13 * Fz;
        su_world_input_Force[1] = R23 * Fz;
        su_world_input_Force[2] = R33 * Fz;
    }
}

// 파라미터 (필요시 튜닝)
PARAM_GROUP_START(su_wrench)
PARAM_ADD(PARAM_FLOAT, k_force_scale, &su_k_force_scale)
PARAM_GROUP_STOP(su_wrench)

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
LOG_GROUP_STOP(suWrenchObs)
