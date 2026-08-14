#ifndef DYMOTOR_BRIDGE_H
#define DYMOTOR_BRIDGE_H

#include <stdint.h>

#if defined(_WIN32)
#define ARM_API __declspec(dllexport)
#else
#define ARM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum ArmJointIndex {
    ARM_JOINT_SHOULDER_FLEXION = 0,
    ARM_JOINT_SHOULDER_ABDUCTION = 1,
    ARM_JOINT_ELBOW_FLEXION = 2,
    ARM_JOINT_UPPER_ARM_ROTATION = 3,
    ARM_JOINT_COUNT = 4
};

typedef struct ArmOpenConfig {
    uint16_t device_id;
    const char *local_ip;
    int local_port;
    const char *remote_ip;
    int remote_port;
    int fast_mode;
    uint16_t motor_ids[ARM_JOINT_COUNT];
    uint16_t can_ids[ARM_JOINT_COUNT];
} ArmOpenConfig;

/* All functions return 0 on success and a negative value on failure. */
ARM_API int arm_open(const ArmOpenConfig *config);
ARM_API int arm_enable(void);
ARM_API int arm_disable(void);
ARM_API int arm_get_joint_state(
    int joint,
    float *position,
    float *velocity,
    float *current,
    float *torque,
    uint32_t *state,
    uint32_t *bus,
    uint32_t *error
);
ARM_API void arm_set_diagnostics(int enabled);
ARM_API int arm_set_joint_position(int joint, float position);
ARM_API int arm_set_joint_positions(const float positions[ARM_JOINT_COUNT], uint32_t mask);
ARM_API int arm_set_three_joint_positions(
    float shoulder_flexion,
    float shoulder_abduction,
    float elbow_flexion
);
ARM_API void arm_close(void);
ARM_API const char *arm_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
