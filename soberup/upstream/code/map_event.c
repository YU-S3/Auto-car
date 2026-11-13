#include "map_event.h"
#include "trajectory_policy.h"

MapEvent g_map_events[MAX_MAP_EVENTS];
int g_map_event_count = 0;
int map_event_overflow = 0;

void map_event_clear(void)
{
    g_map_event_count = 0;
    map_event_overflow = 0;
}

// 将一个已结束的连续元素片段写入事件表。
// 返回 1 表示写入成功，0 表示事件表已满。
static int map_event_finalize(int event_start, int event_end, uint8_t element_code)
{
    MapEvent *ev;
    TrajectoryElementType element_type;

    if (event_end < event_start)
    {
        return 1;
    }

    if (g_map_event_count >= MAX_MAP_EVENTS)
    {
        map_event_overflow = 1;
        return 0;
    }

    ev = &g_map_events[g_map_event_count];
    ev->flag = (int)element_code;
    ev->id = trajectory_element_id(element_code);
    ev->start_index = event_start;
    ev->end_index = event_end;
    ev->mid_index = (event_start + event_end) / 2;
    ev->start_mileage = (int)Path_Memory[event_start].mileage_ticks;
    ev->end_mileage = (int)Path_Memory[event_end].mileage_ticks;
    ev->mid_mileage = (int)Path_Memory[ev->mid_index].mileage_ticks;
    ev->event_point_count = event_end - event_start + 1;

    element_type = trajectory_element_type(element_code);
    if (element_type == TRAJECTORY_ELEMENT_TYPE_FORK)
    {
        ev->type = MAP_EVENT_FORK;
    }
    else if (element_type == TRAJECTORY_ELEMENT_TYPE_ANGLE)
    {
        ev->type = MAP_EVENT_ANGLE;
    }
    else
    {
        ev->type = MAP_EVENT_NONE;
    }

    g_map_event_count++;
    return 1;
}

// 扫描 Path_Memory，将连续相同 element_code 的片段聚合成 MapEvent。
// 普通路线不产生事件，元素类型和编号由 trajectory_policy 解码。
void map_event_rebuild(void)
{
    int in_event = 0;
    int event_start = 0;
    uint8_t current_code = TRAJECTORY_ELEMENT_NONE;

    map_event_clear();
    if (Total_Path_Points < 2)
    {
        return;
    }

    for (int i = 0; i < Total_Path_Points; i++)
    {
        uint8_t element_code = Path_Memory[i].element_code;

        if (!in_event)
        {
            if (element_code != TRAJECTORY_ELEMENT_NONE)
            {
                in_event = 1;
                event_start = i;
                current_code = element_code;
            }
        }
        else if (element_code == TRAJECTORY_ELEMENT_NONE ||
                 element_code != current_code)
        {
            if (!map_event_finalize(event_start, i - 1, current_code))
            {
                return;
            }

            if (element_code == TRAJECTORY_ELEMENT_NONE)
            {
                in_event = 0;
                current_code = TRAJECTORY_ELEMENT_NONE;
            }
            else
            {
                event_start = i;
                current_code = element_code;
            }
        }
    }

    if (in_event)
    {
        (void)map_event_finalize(
            event_start,
            Total_Path_Points - 1,
            current_code
        );
    }
}

int map_event_count(void)
{
    return g_map_event_count;
}

const MapEvent *map_event_get(int index)
{
    if (index < 0 || index >= g_map_event_count)
    {
        return 0;
    }
    return &g_map_events[index];
}

const MapEvent *map_event_find_flag(int flag)
{
    int i;

    for (i = 0; i < g_map_event_count; i++)
    {
        if (g_map_events[i].flag == flag)
        {
            return &g_map_events[i];
        }
    }
    return 0;
}

// 按 flag 查找, 返回 mid_mileage 最接近 mileage 的事件
// 同 flag 在 Path_Memory 中可能断裂为多段, 此函数选最近的那段
// 没有匹配时返回 NULL
const MapEvent *map_event_find_flag_near(int flag, int mileage)
{
    const MapEvent *best = 0;
    int best_dist = 0x7FFFFFFF;
    int i;

    for (i = 0; i < g_map_event_count; i++)
    {
        if (g_map_events[i].flag == flag)
        {
            int dist = g_map_events[i].mid_mileage - mileage;
            if (dist < 0)
            {
                dist = -dist;
            }
            if (dist < best_dist)
            {
                best_dist = dist;
                best = &g_map_events[i];
            }
        }
    }
    return best;
}

int map_event_find_index_by_flag(int flag)
{
    int i;

    for (i = 0; i < g_map_event_count; i++)
    {
        if (g_map_events[i].flag == flag)
        {
            return i;
        }
    }
    return -1;
}

const MapEvent *map_event_find_next_fork_after_flag(int flag)
{
    int found_current = 0;
    int i;

    for (i = 0; i < g_map_event_count; i++)
    {
        if (found_current)
        {
            if (g_map_events[i].type == MAP_EVENT_FORK)
            {
                return &g_map_events[i];
            }
        }
        if (g_map_events[i].flag == flag)
        {
            found_current = 1;
        }
    }
    return 0;
}

// 指针反查: 通过 ev - g_map_events 计算索引, 越界返回 -1
int map_event_find_index_of_event(const MapEvent *ev)
{
    int idx;

    if (ev == 0)
    {
        return -1;
    }

    idx = (int)(ev - g_map_events);
    if (idx >= 0 && idx < g_map_event_count)
    {
        return idx;
    }
    return -1;
}

// 从 event_index 之后找下一个不同 flag 的 fork
// 跳过与当前事件同 flag 的断裂片段 (避免同一物理三叉被当成"下一个")
// 返回 NULL 表示后面没有 fork 了
const MapEvent *map_event_find_next_fork_after_index(int event_index)
{
    int current_flag = (event_index >= 0 && event_index < g_map_event_count)
                       ? g_map_events[event_index].flag : 0;
    int i;

    for (i = event_index + 1; i < g_map_event_count; i++)
    {
        if (g_map_events[i].type == MAP_EVENT_FORK
            && g_map_events[i].flag != current_flag)
        {
            return &g_map_events[i];
        }
    }
    return 0;
}

int map_event_count_angles_before_index(int event_index)
{
    int count = 0;
    int i;

    for (i = 0; i < event_index && i < g_map_event_count; i++)
    {
        if (g_map_events[i].type == MAP_EVENT_ANGLE)
        {
            count++;
        }
    }
    return count;
}

int map_event_is_valid(void)
{
    return (g_map_event_count > 0);
}
