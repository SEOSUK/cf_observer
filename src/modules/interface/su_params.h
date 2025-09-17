#ifndef SU_PARAMS_H_
#define SU_PARAMS_H_
/*
 * Centralized shared parameters for SU modules
 *
 * These are defined (storage allocated) in su_params.c and registered
 * to the Crazyflie PARAM system there. Include this header from any
 * module (estimator / controller / trajectory / observers ...) that
 * needs to read/write the same parameters at runtime.
 *
 * Units:
 *  - mass:            [kg]
 *  - Kf, Ktau:        [1/s]
 *  - deadzone_F:      [N]
 *  - deadzone_T:      [N·m]
 *  - voltage_model_*: [-] (model coefficients)
 *  - *_alpha:         [0..1] IIR (LPF) gain
 */

#ifdef __cplusplus
extern "C" {
#endif

// -------- Platform / common --------
extern float su_mass;             // [kg]
extern float Jxx;                 // [kg·m^2]
extern float Jyy;                 // [kg·m^2]
extern float Jzz;                 // [kg·m^2]

extern float Offset_x;            // [m]
extern float Offset_y;            // [m]
extern float Offset_z;            // [m]

// -------- Wrench observer / MOB --------
extern float su_Kf;               // [1/s] linear momentum observer gain
extern float su_Ktau;             // [1/s] angular momentum observer gain

extern float su_deadzone_F;       // [N]   force deadzone
extern float su_deadzone_T;       // [N·m] torque deadzone

extern float su_voltage_model_a;  // [-] thrust-voltage model 'a'
extern float su_voltage_model_b;  // [-] thrust-voltage model 'b'

extern float su_vbat_alpha;       // [0..1] battery voltage LPF alpha
extern float su_dt_alpha;         // [0..1] dt monitor LPF alpha
extern float su_mob_alpha;        // [0..1] MOB output LPF alpha

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* SU_PARAMS_H_ */
