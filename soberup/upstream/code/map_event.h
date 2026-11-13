#ifndef CODE_MAP_EVENT_H_
#define CODE_MAP_EVENT_H_

#include "zf_common_headfile.h"
#include "inertial_navigation.h"

// ==============================================================
// 事件类型
// ==============================================================
typedef enum
{
    MAP_EVENT_NONE = 0,
    MAP_EVENT_FORK,
    MAP_EVENT_ANGLE
} MapEventType;

// ==============================================================
// 事件结构体: Path_Memory 中连续相同 element_code 的片段
// ==============================================================
typedef struct
{
    MapEventType type; // NONE / FORK / ANGLE
    int flag;          // 原始 element_code，保留 int 以兼容现有辅助接口
    int id;            // 由 trajectory_policy 统一解码的元素编号

    int start_index; // Path_Memory 起始索引
    int mid_index;   // Path_Memory 中点索引
    int end_index;   // Path_Memory 结束索引

    int start_mileage; // 起点里程 (ticks)
    int mid_mileage;   // 中点里程
    int end_mileage;   // 终点里程

    int event_point_count; // 含多少个 Path_Memory 点
} MapEvent;

#define MAX_MAP_EVENTS 128

extern MapEvent g_map_events[MAX_MAP_EVENTS];
extern int g_map_event_count;
extern int map_event_overflow;

// ==============================================================
// API
// ==============================================================

// 清空事件和溢出状态
void map_event_clear(void);

// 扫描 Path_Memory, 将连续相同 element_code 聚合为 MapEvent 序列
void map_event_rebuild(void);

// 事件总数
int map_event_count(void);

// 按索引获取事件, 越界返回 NULL
const MapEvent *map_event_get(int index);

// 按 flag 查找事件 (返回第一个匹配, 同 flag 有多个时不适用)
const MapEvent *map_event_find_flag(int flag);

// 按 flag 查找事件, 返回 mid_mileage 最接近 mileage 的那个 (处理同 flag 断裂)
const MapEvent *map_event_find_flag_near(int flag, int mileage);

// 按 flag 查找索引 (返回第一个匹配)
int map_event_find_index_by_flag(int flag);

// 指针反查索引: 传入 g_map_events 中的元素指针, 返回其下标
int map_event_find_index_of_event(const MapEvent *ev);

// 找 flag 之后的下一个 fork (从第一个匹配的 flag 之后开始找, 可能找到同 flag 断裂段)
const MapEvent *map_event_find_next_fork_after_flag(int flag);

// 从指定索引之后找下一个不同 flag 的 fork (跳过同 flag 断裂片段)
const MapEvent *map_event_find_next_fork_after_index(int event_index);

// 统计 event_index 之前的 angle 事件片段数量（兼容旧调试接口）。
int map_event_count_angles_before_index(int event_index);

// 地图是否有效 (事件数 > 0)
int map_event_is_valid(void);

#endif /* CODE_MAP_EVENT_H_ */
