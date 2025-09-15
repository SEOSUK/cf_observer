#pragma once
#include "stabilizer_types.h"
#include "motors.h"      // motors_thrust_pwm_t

void suWrenchObserverInit(void);

// state = 추정된 상태 (stateEstimator 결과)
// motorPwm = 실제 ESC로 전송된 PWM 값 (캡핑 후)
void suWrenchObserverUpdate(const state_t *state,
                            const motors_thrust_pwm_t *motorPwm);
