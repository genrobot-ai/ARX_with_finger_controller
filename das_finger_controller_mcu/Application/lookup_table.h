#ifndef __LOOKUP_TABLE_H__
#define __LOOKUP_TABLE_H__

typedef struct {
    float L;      // 夹爪张开距离（mm）
    float theta;  // 电机旋转圈数
} GripperLookupEntry;

GripperLookupEntry gripper_lookup_table[207];

#define GRIPPER_LOOKUP_TABLE_SIZE 207
#define GRIPPER_L_MIN 0.0f
#define GRIPPER_L_MAX 103.0f
#define GRIPPER_L_STEP 0.5f



float gripper_lookup_theta(float L);

float gripper_distance_to_revolutions(float L);

float gripper_revolutions_to_distance(float revolutions);
float gripper_distance_to_revolutions_tactile(float L);
float gripper_revolutions_to_distance_tactile(float revolutions);


#endif





