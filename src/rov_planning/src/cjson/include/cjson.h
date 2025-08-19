#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <cjson/cJSON.h>

// 定义运动指令结构体
struct MotionCommand {
    char type[20];  // "forward", "yaw", "throttle"等
    int duty;       // PWM占空比（如1600）
};

// 定义路径段结构体
struct PathSegment {
    MotionCommand* commands;  // 该段包含的指令列表
    int command_count;       // 指令数量
    int time;                // 执行时间（秒）
};

// 解析JSON文件
void parse_json(const char* filename, PathSegment** segments, int* segment_count) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open file");
        exit(1);
    }

    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    char* json_data = (char*)malloc(file_size + 1);
    fread(json_data, 1, file_size, file);
    fclose(file);
    json_data[file_size] = '\0';

    cJSON* root = cJSON_Parse(json_data);
    if (!root) {
        printf("Error parsing JSON: %s\n", cJSON_GetErrorPtr());
        free(json_data);
        exit(1);
    }

    cJSON* auto_path = cJSON_GetObjectItem(root, "auto_path");
    *segment_count = cJSON_GetArraySize(auto_path);
    *segments = (PathSegment*)malloc(*segment_count * sizeof(PathSegment));

    for (int i = 0; i < *segment_count; i++) {
        cJSON* segment = cJSON_GetArrayItem(auto_path, i);
        cJSON* type_list = cJSON_GetObjectItem(segment, "type_list");
        int command_count = cJSON_GetArraySize(type_list);
        (*segments)[i].commands = (MotionCommand*)malloc(command_count * sizeof(MotionCommand));
        (*segments)[i].command_count = command_count;
        (*segments)[i].time = cJSON_GetObjectItem(segment, "time")->valueint;

        for (int j = 0; j < command_count; j++) {
            cJSON* cmd = cJSON_GetArrayItem(type_list, j);
            strcpy((*segments)[i].commands[j].type, cJSON_GetObjectItem(cmd, "type")->valuestring);
            (*segments)[i].commands[j].duty = cJSON_GetObjectItem(cmd, "duty")->valueint;
        }
    }

    cJSON_Delete(root);
    free(json_data);
}

// 执行运动控制（模拟函数，需替换为实际硬件控制）
void execute_command(const MotionCommand* cmd) {
    printf("Executing: type=%s, duty=%d\n", cmd->type, cmd->duty);
    // 实际控制代码示例（伪代码）：
    // if (strcmp(cmd->type, "forward") == 0) {
    //     set_motor_pwm(MOTOR_FORWARD, cmd->duty);
    // } else if (strcmp(cmd->type, "yaw") == 0) {
    //     set_servo_angle(SERVO_YAW, cmd->duty);
    // }
}

int main() {
    PathSegment* segments;
    int segment_count;

    // 解析JSON文件
    parse_json("auto_path.json", &segments, &segment_count);

    // 执行自动循迹
    for (int i = 0; i < segment_count; i++) {
        printf("Segment %d: Time=%ds\n", i + 1, segments[i].time);
        for (int j = 0; j < segments[i].command_count; j++) {
            execute_command(&segments[i].commands[j]);
        }
        sleep(segments[i].time);  // 等待该段执行完成
    }

    // 释放内存
    for (int i = 0; i < segment_count; i++) {
        free(segments[i].commands);
    }
    free(segments);

    return 0;
}