# Agent 任务：识别小车上的发光正方形环

## 代码路径
- `project/code/find.c` — 连通域计算、形状检测、新增 ring 检测函数
- `project/code/find.h` — 头文件，新增结构体和函数声明
- `project/code/image.c` — 二值化、Sobel、形态学操作
- `project/code/image.h` — 图像处理头文件

## 环境背景
- 嵌入式平台：Infineon TRAVEO T2G (CYT4BB7)，ARM Cortex-M7
- 摄像头：MT9V03X 灰度摄像头
- 图像分辨率：`MT9V03X_W = 188`，`MT9V03X_H = 120`，总像素 22560
- 原始图像存储在全局变量 `mt9v03x_image[MT9V03X_H][MT9V03X_W]`（0-255 灰度）
- 二值图像存储在 `mt9v03x_image_BandW[MT9V03X_H][MT9V03X_W]`
- 帧完成标志：`mt9v03x_finish_flag == 1`

## 目标特征：发光正方形环

### 物理特征
小车上的发光正方形环在摄像头画面中呈现为：
- **高亮度**：环本身是发光体（LED 灯带构成的正方形边框），在灰度图中像素值远高于背景
- **中空结构**：环中间是暗的（无发光体），形成"回"字形
- **正方形轮廓**：外轮廓接近正方形，边框宽度相对均匀
- **尺寸范围**：环在画面中的大小取决于距离，预估外边长 15~60 像素范围

### 二值化后的理想形态
```
  00000000000000000000000000000
  00000000000000000000000000000
  00000111111111111111110000000
  00000100000000000000010000000
  00000100000000000000010000000
  00000100000000000000010000000
  00000100000000000000010000000
  00000100000000000000010000000
  00000111111111111111110000000
  00000000000000000000000000000
```
- 白色(1) = 环的边框（发光部分）
- 黑色(0) = 背景和环内部空洞

---

## 检测算法设计

### 总体流程

```
帧捕获 → 二值化(OTSU) → 去噪 → 形态学闭运算(可选) → 连通域标记 → 环形筛选 → 最优候选锁定
```

### 阶段 1：二值化

发光环比背景亮，OTSU 自适应阈值天然会把环分为前景(白)。

**关键注意**：现有 `Get_mt9v03x_image_BandW(mode=0)` 输出 0/255，但 `ConnectedComponentLabeling` 检查 `bitmap[index] == 1`。需要一个新的二值化函数，输出 **0 和 1**。

**方案 A（推荐）**：写一个新函数 `Threshold_OTSU_01(void)`，内部调用 `GetOSTU()` 获取阈值，然后逐像素判断 `>= threshold ? 1 : 0`，写入 `mt9v03x_image_BandW`。

```c
// 新增函数：OTSU 二值化，输出 0/1
void Threshold_OTSU_01(void)
{
    short thr = GetOSTU(mt9v03x_image);
    for (int i = 0; i < MT9V03X_H; i++) {
        for (int j = 0; j < MT9V03X_W; j++) {
            mt9v03x_image_BandW[i][j] = (mt9v03x_image[i][j] >= thr) ? 1 : 0;
        }
    }
}
```

**方案 B（备选）**：如果环境光照可控，使用固定阈值。发光环通常是最亮的物体，阈值可设为 150~200。使用已有的 `Set_image_twovalues(value)` 即可（它输出 0/1）。

### 阶段 2：去噪

调用已有的 `Bin_Image_Filter()`，它做两遍滤波：
- 第 1 遍：填充孤立黑洞（周围 ≥3 个白邻居 → 填白）
- 第 2 遍：去除孤立白点（周围 <2 个白邻居 → 变黑）

### 阶段 3：形态学闭运算（可选但推荐）

发光环可能因为 LED 间隔或遮挡出现边框断裂。闭运算（先膨胀后腐蚀）可以弥合小断口：

```c
// 闭运算 = 膨胀 + 腐蚀，弥合小裂缝
// value=1 因为二值图是0/1
PENGZHANG_operation(mt9v03x_image_BandW, 1, 3);   // 膨胀：3个以上邻居→变白
erosion_operation(mt9v03x_image_BandW, 1, 3);      // 腐蚀：恢复边框宽度
```

**注意**：`erosion_operation` 和 `PENGZHANG_operation` 有一个已知 bug——右下角的对角线邻居被重复取样（`[i-1][j+1]` 出现了两次替代了 `[i+1][j+1]`），但这在大多数情况下影响不大。如需精确修复，见下文"可选改进"。

### 阶段 4：连通域标记

调用已有的 `ConnectedComponentLabeling((unsigned char*)mt9v03x_image_BandW, labelBuffer)`。

调用后获得每个连通域的：
| 属性 | 全局变量 | 说明 |
|------|---------|------|
| 面积 | `area_map[label]` | 像素数 |
| 中心x | `Connected_x[label]` | 像素坐标 |
| 中心y | `Connected_y[label]` | 像素坐标 |
| 左边界 | `Connected_xmin[label]` | bounding box |
| 右边界 | `Connected_xmax[label]` | bounding box |
| 上边界 | `Connected_ymin[label]` | bounding box |
| 下边界 | `Connected_ymax[label]` | bounding box |
| 平均亮度 | `Connected_light[label]` | 原始灰度均值 |
| 标签总数 | `labelcount` | 有效标签 2 ~ labelcount+1 |

**注意**：`ConnectedComponentLabeling` 内部对 `mt9v03x_image` 做了亮度平均（`SearchNeighbor_4` 第 172 行：`Connected_light[labelIndex] += lightmap[searchIndex]`），所以 raw image 必须在 CCL 调用时仍然有效。

### 阶段 5：环形候选筛选（核心新逻辑）

对每个 label（2 到 labelcount+1），计算环特征评分：

#### 5.1 基本过滤
```c
if (area_map[label] < RING_AREA_MIN) continue;  // 太小，跳过
```

#### 5.2 正方形检测（宽高比）
```c
int bb_w = Connected_xmax[label] - Connected_xmin[label] + 1;
int bb_h = Connected_ymax[label] - Connected_ymin[label] + 1;
float aspect = (float)bb_w / (float)bb_h;
if (aspect < 0.55f || aspect > 1.8f) continue;  // 太扁或太高，不是正方形
```

允许一定范围的宽高比偏差（0.55~1.8），适应摄像头斜视带来的透视变形。

#### 5.3 中空度检测（环形 vs 实心的关键判别）

**这是识别"环"的核心逻辑**——区分实心方块和空心方框。

**方法：填充率（Fill Ratio）**

```c
int bbox_area = bb_w * bb_h;
float fill_ratio = (float)area_map[label] / (float)bbox_area;
```

分析：
- 实心正方形：`fill_ratio ≈ 1.0`（像素填满整个包围盒）
- 细框正方形环（边框厚度 2px）：外框 30px → 面积 ≈ 4×30×2 - 4×4 = 224，包围盒 900 → `fill_ratio ≈ 0.25`
- 粗框正方形环（边框厚度 5px）：外框 30px → 面积 ≈ 4×30×5 - 4×25 = 500，包围盒 900 → `fill_ratio ≈ 0.56`
- 一般环的填充率范围：**0.15 ~ 0.70**

```c
if (fill_ratio < RING_FILL_MIN || fill_ratio > RING_FILL_MAX) continue;
```

**补充验证：中心点空洞检查（更严格的判定）**

计算包围盒中心坐标，检查该点在二值图中是否为 0（黑色=空洞）：
```c
int mid_x = (Connected_xmin[label] + Connected_xmax[label]) / 2;
int mid_y = (Connected_ymin[label] + Connected_ymax[label]) / 2;
int is_hollow = (mt9v03x_image_BandW[mid_y][mid_x] == 0);

// 额外检查四条边的中点是否为白色（确认边框存在）
int top_mid    = mt9v03x_image_BandW[Connected_ymin[label]][mid_x];
int bottom_mid = mt9v03x_image_BandW[Connected_ymax[label]][mid_x];
int left_mid   = mt9v03x_image_BandW[mid_y][Connected_xmin[label]];
int right_mid  = mt9v03x_image_BandW[mid_y][Connected_xmax[label]];
int border_ok  = (top_mid + bottom_mid + left_mid + right_mid >= 3); // 至少3条边中点有白色

if (!is_hollow || !border_ok) continue;  // 必须中空且有完整边框
```

#### 5.4 综合评分

对通过所有筛选的候选环，计算一个综合分数用于排序：

```c
// 形状分：宽高比越接近 1 越好
float shape_score = 1.0f - fabsf(aspect - 1.0f);

// 中空分：填充率越接近理想环（~0.35）越好
float ideal_fill = 0.35f;
float hollow_score = 1.0f - fabsf(fill_ratio - ideal_fill) / 0.35f;
if (hollow_score < 0.0f) hollow_score = 0.0f;

// 尺寸分：面积在期望范围内得分高
float size_score;
int area = area_map[label];
if (area >= 100 && area <= 2000) {
    size_score = 1.0f;   // 期望范围
} else if (area < 100) {
    size_score = (float)area / 100.0f;  // 偏小扣分
} else {
    size_score = 2000.0f / (float)area; // 偏大扣分
}

// 亮度分：环的平均灰度应该较高（确实是发光的）
// Connected_light[label] 是原始灰度均值，发光环应该在 150+
float light_score;
if (Connected_light[label] >= 180) light_score = 1.0f;
else if (Connected_light[label] >= 120) light_score = (float)(Connected_light[label] - 120) / 60.0f;
else light_score = 0.0f;

float total_score = shape_score * 0.25f + hollow_score * 0.35f
                  + size_score  * 0.15f + light_score  * 0.25f;
```

### 阶段 6：最优候选锁定 + 时序平滑

在通过筛选的候选中选 `total_score` 最高的。同时沿用已有的空间锁定机制（`find_max` 中已实现），防止帧间跳动：

```c
// 空间锁定逻辑（从 find_max 移植）
static int lock_ring_x = -1;
static int lock_ring_y = -1;
static int lock_ring_label = -1;
static int lock_ring_active = 0;

// 如果上一帧锁定了某个环，且当前帧在锁定位置 <400 像素距离内有候选，
// 优先选择该候选（即使 total_score 略低），保证稳定性
```

### 阶段 7：输出

检测结果输出到全局变量：
```c
int ring_center_x;     // 环中心 x 坐标（像素）
int ring_center_y;     // 环中心 y 坐标（像素）
int ring_found;        // 1=检测到环, 0=未检测到
int ring_label;        // 连通域标签（调试用）
int ring_area;         // 环面积（调试用）
float ring_score;      // 综合评分（调试用）
int ring_screen_x;     // 归一化屏幕坐标 x (0-127)
int ring_screen_y;     // 归一化屏幕坐标 y (0-81)
```

---

## 代码实现清单

### 新建函数（写在 find.c 中，声明在 find.h 中）

#### 1. `void Threshold_OTSU_01(void)` — OTSU 二值化输出 0/1
替代 `Get_mt9v03x_image_BandW(0)` 用于环检测前的预处理。OTSU 阈值 → 像素 >= 阈值设为 1，否则 0。

#### 2. `int find_square_ring(void)` — 发光正方形环检测主函数
完整的检测流程，调用上述各阶段。返回 1 表示找到环，0 表示未找到。将结果写入 `ring_center_x/y` 等全局变量。

#### 3. `float compute_ring_fill_ratio(int label)` — 计算填充率（helper）
返回 `area_map[label] / (bb_w * bb_h)`。

#### 4. `int check_ring_hollow(int label)` — 中心空洞验证（helper）
检查包围盒中心的像素是否为 0（空洞），四条边中点是否有白色（边框存在）。返回 1 表示确认为环形。

### 修改现有代码

#### find.h 新增声明
```c
// ====== 发光正方形环检测 ======
#define RING_AREA_MIN      50       // 环最小面积（像素）
#define RING_AREA_MAX      3000     // 环最大面积（像素）
#define RING_FILL_MIN      0.15f    // 最小填充率（低于此值太细不是环）
#define RING_FILL_MAX      0.70f    // 最大填充率（高于此值太实不是环）
#define RING_ASPECT_MIN    0.55f    // 最小宽高比（允许透视变形）
#define RING_ASPECT_MAX    1.80f    // 最大宽高比
#define RING_LIGHT_MIN     120      // 环最低平均灰度（确认为发光体）

typedef struct {
    int label;           // 连通域标签
    int center_x;        // 环中心 x（像素坐标）
    int center_y;        // 环中心 y（像素坐标）
    int area;            // 环面积
    int bb_w;            // 包围盒宽度
    int bb_h;            // 包围盒高度
    float fill_ratio;    // 填充率 = area / (w*h)
    float aspect_ratio;  // 宽高比 = w/h
    float score;         // 综合评分 0~1
    int is_hollow;       // 中心是否空洞
    int border_intact;   // 四边是否完整
} RingCandidate;

extern int ring_center_x;
extern int ring_center_y;
extern int ring_found;
extern int ring_label;
extern int ring_area;
extern float ring_score;
extern int ring_screen_x;
extern int ring_screen_y;

void Threshold_OTSU_01(void);
int find_square_ring(void);
float compute_ring_fill_ratio(int label);
int check_ring_hollow(int label);
```

#### find.c 新增全局变量
```c
int ring_center_x = 0;
int ring_center_y = 0;
int ring_found = 0;
int ring_label = -1;
int ring_area = 0;
float ring_score = 0.0f;
int ring_screen_x = 0;
int ring_screen_y = 0;
```

---

## 环形检测核心算法伪代码

```c
int find_square_ring(void)
{
    ring_found = 0;

    // 1. 二值化：OTSU → 0/1
    Threshold_OTSU_01();

    // 2. 去噪
    Bin_Image_Filter();

    // 3. 形态学闭运算（弥合 LED 间隔造成的断口）
    PENGZHANG_operation(mt9v03x_image_BandW, 1, 3);
    erosion_operation(mt9v03x_image_BandW, 1, 3);

    // 4. 连通域标记
    ConnectedComponentLabeling((unsigned char*)mt9v03x_image_BandW, labelBuffer);

    // 5. 遍历所有标签，筛选环候选
    RingCandidate best;
    best.score = 0.0f;
    best.label = -1;

    for (int label = 2; label <= labelcount + 1; label++) {
        // 5.1 面积过滤
        if (area_map[label] < RING_AREA_MIN) continue;
        if (area_map[label] > RING_AREA_MAX) continue;

        // 5.2 包围盒和宽高比
        int bw = Connected_xmax[label] - Connected_xmin[label] + 1;
        int bh = Connected_ymax[label] - Connected_ymin[label] + 1;
        if (bw <= 0 || bh <= 0) continue;
        float aspect = (float)bw / (float)bh;
        if (aspect < RING_ASPECT_MIN || aspect > RING_ASPECT_MAX) continue;

        // 5.3 填充率（环形判定）
        float fill = (float)area_map[label] / (float)(bw * bh);
        if (fill < RING_FILL_MIN || fill > RING_FILL_MAX) continue;

        // 5.4 中心空洞 + 边框验证
        int hollow = check_ring_hollow(label);
        if (!hollow) continue;

        // 5.5 亮度验证（确实是发光体）
        if (Connected_light[label] < RING_LIGHT_MIN) continue;

        // 5.6 综合评分
        float shape  = 1.0f - fabsf(aspect - 1.0f);   // 越方越好
        float ideal  = 0.35f;
        float hollow_s = 1.0f - fabsf(fill - ideal) / 0.35f;
        if (hollow_s < 0.0f) hollow_s = 0.0f;
        float light_s = (Connected_light[label] >= 180) ? 1.0f
                      : (float)(Connected_light[label] - 120) / 60.0f;
        float score = shape * 0.25f + hollow_s * 0.35f + light_s * 0.40f;

        // 5.7 空间锁定：接近上一帧位置加分
        if (lock_ring_active) {
            int dx = Connected_x[label] - lock_ring_x;
            int dy = Connected_y[label] - lock_ring_y;
            int dist2 = dx*dx + dy*dy;
            if (dist2 < 400) score += 0.3f;  // 空间一致性加分
        }

        if (score > best.score) {
            best.label = label;
            best.score = score;
            best.center_x = Connected_x[label];
            best.center_y = Connected_y[label];
            best.area = area_map[label];
            best.bb_w = bw;
            best.bb_h = bh;
            best.fill_ratio = fill;
            best.aspect_ratio = aspect;
        }
    }

    // 6. 输出
    if (best.label > 0 && best.score > 0.3f) {  // 最低置信度阈值
        ring_found = 1;
        ring_label = best.label;
        ring_center_x = best.center_x;
        ring_center_y = best.center_y;
        ring_area = best.area;
        ring_score = best.score;
        ring_screen_x = (int)(best.center_x * 128.0f / MT9V03X_W);
        ring_screen_y = (int)(best.center_y * 81.0f / MT9V03X_H);

        // 更新空间锁定
        lock_ring_x = best.center_x;
        lock_ring_y = best.center_y;
        lock_ring_active = 1;
    } else {
        // 未找到环，保持上一帧锁定位置（持续短暂时间）
        ring_found = 0;
        // lock_ring_active 保持，下一帧继续尝试
    }

    return ring_found;
}

// 中心空洞 + 边框完整性验证
int check_ring_hollow(int label)
{
    int mid_x = (Connected_xmin[label] + Connected_xmax[label]) / 2;
    int mid_y = (Connected_ymin[label] + Connected_ymax[label]) / 2;

    // 边界保护
    if (mid_x < 0 || mid_x >= MT9V03X_W) return 0;
    if (mid_y < 0 || mid_y >= MT9V03X_H) return 0;

    // 中心必须是空洞（0=黑色）
    if (mt9v03x_image_BandW[mid_y][mid_x] != 0) return 0;

    // 四条边中点必须有白色（确认边框存在）
    int xmin = Connected_xmin[label];
    int xmax = Connected_xmax[label];
    int ymin = Connected_ymin[label];
    int ymax = Connected_ymax[label];

    int top    = (ymin >= 0) ? mt9v03x_image_BandW[ymin][mid_x] : 0;
    int bottom = (ymax < MT9V03X_H) ? mt9v03x_image_BandW[ymax][mid_x] : 0;
    int left   = (xmin >= 0) ? mt9v03x_image_BandW[mid_y][xmin] : 0;
    int right  = (xmax < MT9V03X_W) ? mt9v03x_image_BandW[mid_y][xmax] : 0;

    return (top + bottom + left + right >= 3);  // 至少3条边有白色
}
```

---

## 调用方式（集成到主循环）

在主循环的图像处理段中，替换或增补环检测逻辑：

```c
// 在主循环中（main_cm7_1.c 或图像处理任务）
if (mt9v03x_finish_flag == 1) {
    mt9v03x_finish_flag = 0;

    // === 发光正方形环检测 ===
    int found = find_square_ring();

    if (found) {
        // 使用 ring_center_x, ring_center_y 做导航控制
        // ring_screen_x (0-127), ring_screen_y (0-81) 可用于 PID 输入

        // 示例：计算环偏离画面中心的偏移量
        int offset_x = ring_center_x - MT9V03X_W / 2;  // 正=偏右
        int offset_y = ring_center_y - MT9V03X_H / 2;  // 正=偏下
        // ... PID 控制小车转向，使 ring 保持在画面中心 ...
    } else {
        // 环丢失，执行搜索策略（原地旋转等）
    }

    // ... 其他处理（道路跟踪 Seek_Road 等，如果需要同时使用）...
}
```

---

## 调试辅助

### 在 IPS200 显示屏上可视化检测结果

```c
// 在 find_square_ring() 末尾添加可视化输出
if (ring_found) {
    // 画包围盒
    int bbx = Connected_xmin[ring_label];
    int bby = Connected_ymin[ring_label];
    int bbw = Connected_xmax[ring_label] - Connected_xmin[ring_label];
    int bbh = Connected_ymax[ring_label] - Connected_ymin[ring_label];
    ips200_draw_rect(bbx, bby, bbw, bbh, RGB565_GREEN);  // 绿色包围盒

    // 画十字准星
    ips200_draw_line(0, ring_center_y, MT9V03X_W, ring_center_y, RGB565_RED);
    ips200_draw_line(ring_center_x, 0, ring_center_x, MT9V03X_H, RGB565_RED);

    // 显示评分（数值）
    ips200_show_float(0, 0, ring_score, 2);  // 左上角显示评分
    ips200_show_int(50, 0, ring_area, 5);     // 显示面积
}
```

### 串口输出调试信息

```c
// 通过 SeekFree 通信协议发送调试数据
// ring_found, ring_center_x, ring_center_y, ring_score, ring_area
// 可用上位机查看实时数据，辅助调参
```

---

## 可调参数说明

| 参数 | 默认值 | 调参建议 |
|------|--------|---------|
| `RING_AREA_MIN` | 50 | 环很远时面积小，需降低；很近时需升高避免误检小噪点 |
| `RING_AREA_MAX` | 3000 | 防止光照反射的大白斑被误检为环 |
| `RING_FILL_MIN` | 0.15 | 环边框极细时可能需要降低 |
| `RING_FILL_MAX` | 0.70 | 环边框很粗时可能需要升高 |
| `RING_ASPECT_MIN` | 0.55 | 摄像头倾斜角度大时放宽 |
| `RING_ASPECT_MAX` | 1.80 | 同上 |
| `RING_LIGHT_MIN` | 120 | 发光不够亮时降低，背景干扰大时升高 |
| 评分最低阈值 | 0.30 | 降低=更宽松=更多误检；升高=更严格=更多漏检 |
| 形态学参数 `yuzhi` | 3 | 闭运算的邻居阈值，降低=更激进弥合，升高=更保守 |
| 空间锁定距离阈值 | 400 | 单位为平方像素，环移动快时需增大 |

---

## 已知限制与后续优化方向

1. **多个发光环**：当前只选评分最高的一个。如果画面中需要同时跟踪多个环（如赛道多个信标），需要返回候选列表而非单一最佳。

2. **环快速移动**：空间锁定的距离阈值可能不够。如果小车快速转弯导致环大幅移动，锁可能失效。可考虑加入运动预测（卡尔曼滤波）。

3. **光照剧烈变化**：OTSU 在光照极不均匀时失效。可考虑局部自适应阈值（分块 OTSU）。

4. **环被部分遮挡**：如果环的一条边被遮挡，`check_ring_hollow` 的 border_intact 检查会失败。可降低门槛到 2/4 条边。

5. **形态学函数的对角线 Bug**：`erosion_operation` 和 `PENGZHANG_operation` 中右下角对角线 `[i+1][j+1]` 被误写为 `[i-1][j+1]`（与右上角重复）。如需精确形态学操作，应修复此 bug。

6. **非正方形环**：如果发光环是其他形状（菱形、圆形），当前正方形检测会失败。需要不同的形状判别逻辑。

---

## 修改文件清单

| 文件 | 修改内容 |
|------|---------|
| `project/code/find.h` | 新增结构体 `RingCandidate`、宏定义、函数声明、extern 变量声明 |
| `project/code/find.c` | 新增全局变量、`Threshold_OTSU_01()`、`find_square_ring()`、`check_ring_hollow()`、`compute_ring_fill_ratio()` 函数实现 |
| `project/code/image.h` | 可选：声明 `Threshold_OTSU_01()` |
| 主循环文件 | 调用 `find_square_ring()` 并基于结果做导航控制 |
