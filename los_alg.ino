#include <math.h>

// ===== CONSTANT =====
#define PI 3.141592653589793

// ===== PARAM (ปรับได้) =====
float Kp_roll = 1.5;
float max_roll  = 20.0 * PI/180.0;
float max_pitch = 15.0 * PI/180.0;

// LOS parameter
float L = 5.0;   // lookahead distance (เมตร)

// ===== STATE =====
float x, y, z;                 // position
float yaw_current, pitch_current;

// waypoint
float x_start, y_start;
float x_target, y_target, z_target;

// ===== HELPER =====
float wrap_angle(float angle) {
    while (angle > PI)  angle -= 2*PI;
    while (angle < -PI) angle += 2*PI;
    return angle;
}

float constrain(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// ===== OUTPUT STRUCT =====
typedef struct {
    float roll;
    float pitch;
    float yaw;
} AttitudeCmd;

// ===== AUTOPILOT (LOS) =====
AttitudeCmd autopilot_update() {

    AttitudeCmd cmd;

    // ===== 1. path heading =====
    float dx_path = x_target - x_start;
    float dy_path = y_target - y_start;
    float path_heading = atan2(dy_path, dx_path);

    // ===== 2. cross-track error =====
    float dx = x - x_start;
    float dy = y - y_start;

    float e = -sin(path_heading)*dx + cos(path_heading)*dy;

    // ===== 3. LOS heading =====
    float yaw_target = path_heading - atan2(e, L);
    yaw_target = wrap_angle(yaw_target);

    // ===== 4. roll (bank to turn) =====
    float yaw_error = wrap_angle(yaw_target - yaw_current);
    float roll_target = Kp_roll * yaw_error;

    // ===== 5. pitch (altitude control) =====
    float dx_t = x_target - x;
    float dy_t = y_target - y;
    float dz_t = z_target - z;

    float dist_xy = sqrt(dx_t*dx_t + dy_t*dy_t);
    float pitch_target = atan2(dz_t, dist_xy);

    // ===== 6. limit =====
    roll_target  = constrain(roll_target,  -max_roll,  max_roll);
    pitch_target = constrain(pitch_target, -max_pitch, max_pitch);

    // ===== 7. deadband (กันสั่น) =====
    if (fabs(yaw_error) < 0.03) {
        roll_target = 0;
    }

    // ===== OUTPUT =====
    cmd.roll  = roll_target;
    cmd.pitch = pitch_target;
    cmd.yaw   = yaw_target;

    return cmd;
}
void loop() {

    // ===== อ่าน sensor =====
    x = get_x();
    y = get_y();
    z = get_z();

    yaw_current   = get_yaw();
    pitch_current = get_pitch();

    // ===== autopilot =====
    AttitudeCmd cmd = autopilot_update();

    // ===== ส่งเข้า inner loop ของคุณ =====
    set_roll_target(cmd.roll);
    set_pitch_target(cmd.pitch);
    set_yaw_target(cmd.yaw);
}