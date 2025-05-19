#ifndef __VISION_ALIGN_H__
#define __VISION_ALIGN_H__

#ifdef __cplusplus
extern "C" {
#endif

void vision_alignment_update(void);

void vision_alignment_set_x_params(float kp, float ki, float kd, int setpoint, int limit, int deadband);
void vision_alignment_set_y_params(float kp, float ki, float kd, int setpoint, int limit, int deadband);

#ifdef __cplusplus
}
#endif

#endif // __VISION_ALIGN_H__
