#ifndef CODE_MAZE_H_
#define CODE_MAZE_H_

#define SAFE_MARGIN_Y 10

#define GEOMETRIC_BACKTRACK_ADVANCE_MIN    20
#define GEOMETRIC_BACKTRACK_RETURN_MARGIN   5
#define GEOMETRIC_BACKTRACK_CONFIRM_POINTS  4

#include "stdint.h"
#include "tool.h"
typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    float   slope;          /* 赛道边缘的斜率（高度/宽度） */
    float   avg_dist;       /* 平均距离（像素）          */
    float   first_dist;     /* 起点距离（像素）          */
    float   last_dist;      /* 终点距离（像素）          */
    int     valid_pairs;    /* 有效匹配点数量            */
} DistTrend;

typedef struct {
    int fx;
    int fy;
    int fsx;
    int fsy;
    int step;
    int dir;
    int turn;
}maze;

// 出口点信息（爬线过程中记录）
typedef struct {
    Point pt;        // 出口点坐标
    int   step_idx;  // 该点在爬线路径中是第几步（-1 表示未记录）
} ExitPoint;

// 单层边界出口记录
typedef struct {
    uint8_t   L_hit;      // 左爬虫是否经过此边界层
    uint8_t   R_hit;      // 右爬虫是否经过此边界层
    uint8_t   L_entries;  // 左爬虫进入此边界层的次数 (>1 说明反复钻进钻出)
    uint8_t   R_entries;  // 右爬虫进入此边界层的次数
    ExitPoint L;          // 左爬虫在此边界层的第一个出口点
    ExitPoint R;          // 右爬虫在此边界层的第一个出口点
} ExitLayer;

// 三边出口汇总
typedef struct {
    ExitLayer left[3];   // 左边界三层: x <= 11, 13, 15
    ExitLayer right[3];  // 右边界三层: x >= W-11, W-13, W-15
    ExitLayer top;       // 顶部: y <= 15
} ExitRecord;

#define MAX_MATCH_POINTS  250    /* 与 MAX_step 保持一致     */
#define MIN_VALID_PAIRS     3    /* 有效匹配点最小阈值        */
#define TOUCH_DISTANCE_PX   2    /* 双爬虫距离不大于此值时相遇 */


extern Point Side_L[300],Side_R[300],StarM;
extern int L_point,R_point;
extern Point Max_Lx,Max_Ly,Max_Rx,Max_Ry;
extern DistTrend trend;
extern ExitRecord exit_record;
extern int cross_rebuild_result;

void search_line_main(void);
int cross_rebuild_edges(void);
void findline_twoside(Point L, Point R, int *Lnum, int *Rnum, int MAX_step);
int calc_side_distance_trend(int Lnum, int Rnum, DistTrend *result);
int find_line_edges_at_row(int y, Point *L, Point *R);
int find_outermost_edges_at_row(int y, Point *L, Point *R);

#endif /* CODE_MAZE_H_ */
