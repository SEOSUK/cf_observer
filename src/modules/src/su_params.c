#include "platform_defaults.h"
#include "param.h"
#include "su_params.h"

// ========= 전역 공유 파라미터 정의 (단일 소스) =========
// 플랫폼/모델
float su_mass            = CF_MASS;      // [kg] 원래는 0.393
float Jxx                = 1.9e-5f;      // [kg·m^2]
float Jyy                = 1.9e-5f;      // [kg·m^2]
float Jzz                = 3.0e-5f;      // [kg·m^2]


// Wrench observer / MOB 관련
float su_Kf              = 5.0f;         // [1/s] 선운동량 관측 이득
float su_Ktau            = 5.0f;         // [1/s] 각운동량 관측 이득
float su_deadzone_F      = 0.000f;       // [N]   힘 deadzone
float su_deadzone_T      = 0.0000f;      // [N·m] 토크 deadzone
float su_voltage_model_a = 0.320569f;    // 전압-추력 모델 a
float su_voltage_model_b = 0.150439f;    // 전압-추력 모델 b
float su_vbat_alpha      = 0.005f;       // 배터리 전압 LPF 알파
float su_dt_alpha        = 0.01f;        // dt 모니터링용 LPF 알파
float su_mob_alpha       = 1.0f;         // MOB 출력 LPF 알파   [안하겟단소리]

// ========= PARAM 등록 =========
PARAM_GROUP_START(su_platform)
// Platform / model parameters
PARAM_ADD(PARAM_FLOAT, mass, &su_mass)
PARAM_ADD(PARAM_FLOAT, Jxx,  &Jxx)
PARAM_ADD(PARAM_FLOAT, Jyy,  &Jyy)
PARAM_ADD(PARAM_FLOAT, Jzz,  &Jzz)
PARAM_GROUP_STOP(su_platform)

// Wrench/MOB 파라미터: 기존 su_wrench 그룹명 유지(로그/툴 호환성)
PARAM_GROUP_START(su_wrench)
// 관측 이득
PARAM_ADD(PARAM_FLOAT, Kf,              &su_Kf)
PARAM_ADD(PARAM_FLOAT, Ktau,            &su_Ktau)
// Deadzone
PARAM_ADD(PARAM_FLOAT, deadzone_F,      &su_deadzone_F)
PARAM_ADD(PARAM_FLOAT, deadzone_T,      &su_deadzone_T)
// 전압-추력 모델
PARAM_ADD(PARAM_FLOAT, voltage_model_a, &su_voltage_model_a)
PARAM_ADD(PARAM_FLOAT, voltage_model_b, &su_voltage_model_b)
// 필터 게인
PARAM_ADD(PARAM_FLOAT, vbat_alpha,      &su_vbat_alpha)
PARAM_ADD(PARAM_FLOAT, dt_alpha,        &su_dt_alpha)
PARAM_ADD(PARAM_FLOAT, mob_alpha,       &su_mob_alpha)
PARAM_GROUP_STOP(su_wrench)
