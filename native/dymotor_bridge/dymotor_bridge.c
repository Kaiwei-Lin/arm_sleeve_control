#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include "dymotor_bridge.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#include "Common.h"
#include "RobotControl.h"

#define ARM_ALL_JOINTS_MASK ((1u << ARM_JOINT_COUNT) - 1u)
#define ARM_PREFILL_COUNT 45
#define ARM_PREFILL_NS 2000000L

typedef struct ArmSession {
    RobotCtx *ctx;
    RobotMotor *motors[ARM_JOINT_COUNT];
    int open;
    int enabled;
} ArmSession;

static ArmSession g_arm;
static char g_last_error[256];
static int g_diagnostics;

typedef struct JointFeedback {
    float position;
    float velocity;
    float current;
    float torque;
    uint32_t state;
    uint32_t bus;
    uint32_t error;
} JointFeedback;

static int fail(int code, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(g_last_error, sizeof(g_last_error), format, args);
    va_end(args);
    return code;
}

static int valid_joint(int joint)
{
    return joint >= 0 && joint < ARM_JOINT_COUNT;
}

static void sleep_prefill_cycle(void)
{
#ifdef _WIN32
    Sleep(2);
#else
    const struct timespec delay = {0, ARM_PREFILL_NS};
    (void)nanosleep(&delay, NULL);
#endif
}

static void wait_for_fast_feedback(int period_ms)
{
    uint64_t remaining_ms = (uint64_t)(unsigned int)period_ms * 2U;
    while (remaining_ms > 0U) {
        const unsigned int chunk_ms = remaining_ms > 1000U ? 1000U : (unsigned int)remaining_ms;
#ifdef _WIN32
        Sleep((DWORD)chunk_ms);
#else
        const struct timespec delay = {
            (time_t)(chunk_ms / 1000U),
            (long)(chunk_ms % 1000U) * 1000000L
        };
        (void)nanosleep(&delay, NULL);
#endif
        remaining_ms -= chunk_ms;
    }
}

static int servo_off_all(void)
{
    int failed = 0;
    int joint;

    if (g_arm.ctx == NULL) {
        return 0;
    }
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if (g_arm.motors[joint] != NULL &&
            !robot_motor_set_control_world(g_arm.motors[joint], CTRL_SERVO_OFF)) {
            failed = 1;
        }
    }
    g_arm.enabled = 0;
    return failed ? fail(-6, "Servo Off failed for one or more motors") : 0;
}

static void release_session(int request_servo_off)
{
    int joint;

    if (request_servo_off) {
        (void)servo_off_all();
    }
    if (g_arm.ctx != NULL) {
        for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
            if (g_arm.motors[joint] != NULL) {
                robot_destroy_motor(g_arm.ctx, g_arm.motors[joint]);
                g_arm.motors[joint] = NULL;
            }
        }
        robot_destroy(g_arm.ctx);
    }
    memset(&g_arm, 0, sizeof(g_arm));
    g_diagnostics = 0;
}

static int validate_open_config(const ArmOpenConfig *config)
{
    int i;
    int j;

    if (config == NULL || config->local_ip == NULL || config->remote_ip == NULL) {
        return fail(-1, "arm_open received a null config or IP address");
    }
    if (sizeof(unsigned int) != sizeof(uint32_t)) {
        return fail(-1, "SDK unsigned int is not compatible with uint32_t outputs");
    }
    if (config->local_port <= 0 || config->local_port > 65535 ||
        config->remote_port <= 0 || config->remote_port > 65535 ||
        config->fast_mode <= 0) {
        return fail(-1, "invalid network port or fast_mode");
    }
    for (i = 0; i < ARM_JOINT_COUNT; ++i) {
        for (j = i + 1; j < ARM_JOINT_COUNT; ++j) {
            if (config->motor_ids[i] == config->motor_ids[j] &&
                config->can_ids[i] == config->can_ids[j]) {
                return fail(-1, "duplicate motor/CAN mapping in arm_open config");
            }
        }
    }
    return 0;
}

static int discover_required_motors(const ArmOpenConfig *config)
{
    RobotMotorListHandle list = NULL;
    MotorArray array;
    int matches[ARM_JOINT_COUNT] = {0};
    int i;
    int joint;
    int object_list_created = 0;
    int result = 0;

    memset(&array, 0, sizeof(array));
    list = motorlist_create();
    if (list == NULL) {
        return fail(-3, "motorlist_create failed");
    }
    if (!get_robot_motorlist(g_arm.ctx, list)) {
        result = fail(-3, "get_robot_motorlist failed");
        goto done;
    }

    robot_create_motorObjectList(g_arm.ctx, list, &array);
    object_list_created = 1;
    if (array.count < 0 || array.count > 16) {
        result = fail(-3, "SDK returned invalid motor count: %d", array.count);
        if (array.count < 0) {
            array.count = 0;
        } else {
            array.count = 16;
        }
        goto done;
    }
    for (i = 0; i < array.count; ++i) {
        unsigned short motor_id = 0;
        unsigned short can_id = 0;
        if (array.robotmotors[i] == NULL) {
            result = fail(-3, "SDK returned a null motor in the discovery list");
            goto done;
        }
        robot_motor_get_motor_id(array.robotmotors[i], &motor_id, &can_id);
        for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
            if (motor_id == config->motor_ids[joint] && can_id == config->can_ids[joint]) {
                ++matches[joint];
            }
        }
    }
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if (matches[joint] != 1) {
            result = fail(
                -3,
                "required motor id=%u CAN=%u was discovered %d time(s)",
                (unsigned)config->motor_ids[joint],
                (unsigned)config->can_ids[joint],
                matches[joint]
            );
            goto done;
        }
    }

done:
    if (object_list_created) {
        motorObjectlist_destroy(g_arm.ctx, &array);
    }
    motorlist_destroy(list);
    return result;
}

int arm_open(const ArmOpenConfig *config)
{
    int joint;
    int result;

    g_last_error[0] = '\0';
    if (g_arm.open) {
        return fail(-2, "arm is already open");
    }
    result = validate_open_config(config);
    if (result != 0) {
        return result;
    }

    g_arm.ctx = robot_create(config->device_id);
    if (g_arm.ctx == NULL) {
        return fail(-2, "robot_create failed");
    }
    if (!robot_config_net(
            g_arm.ctx,
            config->local_ip,
            config->local_port,
            config->remote_port,
            config->remote_ip)) {
        fail(-2, "robot_config_net failed");
        release_session(0);
        return -2;
    }

    /* Match the vendor examples exactly: configure fast feedback immediately
       after the network connection, before creating any motor objects. */
    if (!robot_set_fast_mode(g_arm.ctx, config->fast_mode)) {
        fail(-2, "robot_set_fast_mode failed");
        release_session(0);
        return -2;
    }

    result = discover_required_motors(config);
    if (result != 0) {
        release_session(0);
        return result;
    }

    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        g_arm.motors[joint] = robot_create_motor(
            g_arm.ctx,
            config->motor_ids[joint],
            config->can_ids[joint]
        );
        if (g_arm.motors[joint] == NULL) {
            fail(
                -3,
                "robot_create_motor failed for id=%u CAN=%u",
                (unsigned)config->motor_ids[joint],
                (unsigned)config->can_ids[joint]
            );
            release_session(0);
            return -3;
        }
    }

    /* Opening is motion-command-free: no NMT transition and no Servo command.
       PVCTFast reads an SDK cache and has no validity return, so allow it time
       to populate before reporting the session as open. */
    wait_for_fast_feedback(config->fast_mode);

    g_arm.open = 1;
    return 0;
}

static int read_joint_feedback(int joint, JointFeedback *feedback)
{
    float sdk_position = 0.0f;
    float sdk_velocity = 0.0f;
    float sdk_torque = 0.0f;
    unsigned int sdk_state = 0U;
    unsigned int sdk_mos_temperature = 0U;
    unsigned int sdk_winding_temperature = 0U;
    unsigned int sdk_bus = 0U;
    unsigned int sdk_error = 0U;
    unsigned short motor_id = 0U;
    unsigned short can_id = 0U;

    if (!g_arm.open || g_arm.ctx == NULL) {
        return fail(-2, "arm is not open");
    }
    if (!valid_joint(joint) || feedback == NULL) {
        return fail(-1, "invalid joint index or state output");
    }
    if (g_arm.motors[joint] == NULL) {
        return fail(-3, "motor object is unavailable for joint %d", joint);
    }

    robot_motor_get_PVCTFast(
        g_arm.motors[joint],
        &sdk_position,
        &sdk_velocity,
        &sdk_torque,
        &sdk_state,
        &sdk_mos_temperature,
        &sdk_winding_temperature,
        &sdk_bus,
        &sdk_error
    );

    feedback->position = sdk_position;
    feedback->velocity = sdk_velocity;
    feedback->current = NAN; /* The SDK's PVCTFast declaration has no current output. */
    feedback->torque = sdk_torque;
    feedback->state = (uint32_t)sdk_state;
    feedback->bus = (uint32_t)sdk_bus;
    feedback->error = (uint32_t)sdk_error;

    if (g_diagnostics) {
        robot_motor_get_motor_id(g_arm.motors[joint], &motor_id, &can_id);
        fprintf(
            stderr,
            "[dymotor_bridge raw] motor=%u can=%u position=%.9g velocity=%.9g "
            "current=unavailable torque=%.9g state=%u bus=%u error=%u\n",
            (unsigned)motor_id,
            (unsigned)can_id,
            (double)sdk_position,
            (double)sdk_velocity,
            (double)sdk_torque,
            sdk_state,
            sdk_bus,
            sdk_error
        );
        fflush(stderr);
    }
    if (!isfinite(sdk_position) || !isfinite(sdk_velocity) || !isfinite(sdk_torque) ||
        sdk_bus == 0U) {
        return fail(
            -4,
            "PVCT feedback is unavailable or invalid for joint %d "
            "(position=%.9g velocity=%.9g torque=%.9g state=%u bus=%u error=%u)",
            joint,
            (double)sdk_position,
            (double)sdk_velocity,
            (double)sdk_torque,
            sdk_state,
            sdk_bus,
            sdk_error
        );
    }
    return 0;
}

int arm_get_joint_state(
    int joint,
    float *position,
    float *velocity,
    float *current,
    float *torque,
    uint32_t *state,
    uint32_t *bus,
    uint32_t *error)
{
    JointFeedback feedback;
    int status;

    if (position == NULL || velocity == NULL || current == NULL || torque == NULL ||
        state == NULL || bus == NULL || error == NULL) {
        return fail(-1, "arm_get_joint_state received a null output pointer");
    }
    memset(&feedback, 0, sizeof(feedback));
    status = read_joint_feedback(joint, &feedback);
    *position = feedback.position;
    *velocity = feedback.velocity;
    *current = feedback.current;
    *torque = feedback.torque;
    *state = feedback.state;
    *bus = feedback.bus;
    *error = feedback.error;
    return status;
}

void arm_set_diagnostics(int enabled)
{
    g_diagnostics = enabled != 0;
}

int arm_enable(void)
{
    JointFeedback states[ARM_JOINT_COUNT];
    int sample;
    int joint;

    if (!g_arm.open) {
        return fail(-2, "arm is not open");
    }
    if (g_arm.enabled) {
        return 0;
    }

    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if (read_joint_feedback(joint, &states[joint]) != 0) {
            return -4;
        }
        if (states[joint].error != 0) {
            return fail(
                -4,
                "motor fault prevents Servo On for joint %d: error=%u",
                joint,
                (unsigned)states[joint].error
            );
        }
    }

    /* This is the first state-changing boundary and is only reached after an
       explicit enable request and valid, fault-free feedback from all joints. */
    robot_StateMachine(g_arm.ctx, 0x80);
    robot_StateMachine(g_arm.ctx, 1);
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if (!robot_motor_set_control_mode(g_arm.motors[joint], MOTOR_CTRL_MODE_POSITION)) {
            (void)servo_off_all();
            return fail(-5, "failed to set position mode for joint %d", joint);
        }
    }

    /* Seed every disabled motor with its measured position before Servo On. */
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        robot_motor_set_pos(g_arm.motors[joint], states[joint].position, 0.0f, 0.0f);
        robot_motor_set_position(g_arm.motors[joint], states[joint].position);
    }
    /* Match the vendor position example's bounded 45-cycle prefill. */
    for (sample = 0; sample < ARM_PREFILL_COUNT; ++sample) {
        robot_motor_set_big_pose(g_arm.ctx);
        sleep_prefill_cycle();
    }

    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if (!robot_motor_set_control_world(g_arm.motors[joint], CTRL_SERVO_ON)) {
            (void)servo_off_all();
            return fail(-5, "Servo On failed for joint %d", joint);
        }
    }
    g_arm.enabled = 1;
    return 0;
}

int arm_disable(void)
{
    if (!g_arm.open || !g_arm.enabled) {
        return 0;
    }
    return servo_off_all();
}

int arm_set_joint_positions(const float positions[ARM_JOINT_COUNT], uint32_t mask)
{
    JointFeedback feedback[ARM_JOINT_COUNT];
    int joint;

    if (!g_arm.open || !g_arm.enabled) {
        return fail(-5, "arm must be open and enabled before sending positions");
    }
    if (positions == NULL || mask == 0 || (mask & ~ARM_ALL_JOINTS_MASK) != 0) {
        return fail(-1, "invalid position array or joint mask");
    }
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if ((mask & (1u << joint)) != 0 && !isfinite(positions[joint])) {
            return fail(-1, "non-finite position for joint %d", joint);
        }
    }
    /* Defense in depth: never rely on the Python safety layer alone. */
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        if (read_joint_feedback(joint, &feedback[joint]) != 0) {
            char feedback_error[sizeof(g_last_error)];
            snprintf(feedback_error, sizeof(feedback_error), "%s", g_last_error);
            (void)servo_off_all();
            return fail(-4, "feedback check before motion failed: %s", feedback_error);
        }
        if (feedback[joint].error != 0) {
            (void)servo_off_all();
            return fail(
                -4,
                "motor error before motion for joint %d: %u",
                joint,
                (unsigned)feedback[joint].error
            );
        }
    }
    for (joint = 0; joint < ARM_JOINT_COUNT; ++joint) {
        const float target = (mask & (1u << joint)) != 0
            ? positions[joint]
            : feedback[joint].position;
        robot_motor_set_position(g_arm.motors[joint], target);
    }
    /* The SDK queues per-motor targets; one big-pose call sends the batch. */
    robot_motor_set_big_pose(g_arm.ctx);
    return 0;
}

int arm_set_joint_position(int joint, float position)
{
    float positions[ARM_JOINT_COUNT] = {0};
    if (!valid_joint(joint)) {
        return fail(-1, "invalid joint index: %d", joint);
    }
    positions[joint] = position;
    return arm_set_joint_positions(positions, 1u << joint);
}

int arm_set_three_joint_positions(
    float shoulder_flexion,
    float shoulder_abduction,
    float elbow_flexion)
{
    const float positions[ARM_JOINT_COUNT] = {
        shoulder_flexion,
        shoulder_abduction,
        elbow_flexion
    };
    /* Compatibility API: never target the fourth joint implicitly. */
    return arm_set_joint_positions(
        positions,
        (1u << ARM_JOINT_UPPER_ARM_ROTATION) - 1u
    );
}

void arm_close(void)
{
    if (g_arm.ctx != NULL) {
        release_session(g_arm.enabled);
    }
}

const char *arm_last_error(void)
{
    return g_last_error;
}
