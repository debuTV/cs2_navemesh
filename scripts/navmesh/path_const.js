import { Instance } from "cs_script/point_script";
//==============================世界相关设置=====================================
export const origin = { x: -2300, y: -1150, z: -230 };
//==============================Recast设置======================================
//体素化参数
export const MESH_CELL_SIZE_XY = 6;                               // 体素大小
export const MESH_CELL_SIZE_Z = 1;                                // 体素高度
export const MESH_TRACE_SIZE_Z = 32;                              // 射线方块高度//太高，会把竖直方向上的间隙也忽略
export const MESH_WORLD_SIZE_XY = 4500;                           // 世界大小
export const MESH_WORLD_SIZE_Z = 800;                             // 世界高度
export const MESH_OPTIMIZATION_1 = true;                          // 优化1：是否修剪掉info_target{name:navmesh}不能去的体素
export const MESH_OPTIMIZATION_2 = true;                          // 优化2: 按行走高度忽略边界旁边1个体素，防止出现爬墙,这个选项需要CELL_SIZE足够小，(当出现耳割法错误时推荐开启)
export const MESH_ERODE_RADIUS = 3;                               // 按跳跃高度，腐蚀半径，如果一个区域应该不能被走到，但优化1未去除，可以增加此值,或者修改地图设计
//区域生成参数
export const REGION_MERGE_AREA = 1024;                            // 合并区域阈值（体素单位）
export const REGION_MIN_AREA = 8;                                // 最小区域面积（体素单位）
//轮廓生成参数
export const CONT_MAX_ERROR = MESH_CELL_SIZE_XY * 1.2;            // 原始点到简化后边的最大允许偏离距离（长度）
// 多边形网格配置
export const POLY_BIG_TRI = true;                                 // 耳割法每次割周长最短的
export const POLY_MAX_VERTS_PER_POLY = 6;                         // 每个多边形的最大顶点数
export const POLY_MERGE_LONGEST_EDGE_FIRST = true;                // 优先合并最长边
export const POLY_DETAIL_SAMPLE_DIST = 3;                         // 构建细节网格时，相邻采样点之间的“间距”,选3耗时比较合适
export const POLY_DETAIL_HEIGHT_ERROR = 5;                        // 构建细节网格时，采样点和计算点之差如果小于这个高度阈值就跳过
//其他参数
export const MAX_WALK_HEIGHT = 13 / MESH_CELL_SIZE_Z;             // 怪物最大可行走高度（体素单位）
export const MAX_JUMP_HEIGHT = 64 / MESH_CELL_SIZE_Z;             // 怪物最大可跳跃高度（体素单位）
export const AGENT_RADIUS = 32 / MESH_CELL_SIZE_XY;               // 人物通过所需宽度大小（长度）
export const AGENT_HEIGHT = 72 / MESH_CELL_SIZE_Z;                // 人物高度（体素单位）
//生成参数
export const PRINT_NAV_MESH = false;                              // 是否打印导航网格
export const LOAD_STATIC_MESH = true;                            // 载入静态导航网格，载入静态网格时，不能debug
//==============================Debug设置=======================================
export const MESH_DEBUG = false;                                  // 显示体素化后体素
export const REGION_DEBUG = false;                                // 显示区域
export const CONTOUR_DEBUG = false;                               // 显示区域简化后的轮廓
export const POLY_DEBUG = false;                                   // 显示最后的寻路多边形
export const POLY_DETAIL_DEBUG = false;                           // 显示最后的细节多边形
export const JUMP_LINK_DEBUG = false;                              // 显示跳点
export const LOAD_DEBUG = true;                                  // 载入静态数据时可开启，查看是否导入成功
//==============================Detour设置======================================
//A*寻路参数
export const ASTAR_OPTIMIZATION_1 = false;                        //是否预计算距离，推荐多边形在2000及以下可以打开
export const ASTAR_BLOCK_SIZE = 128;                              //多边形分块大小
export const ASTAR_HEURISTIC_SCALE = 1.2;                         //A*推荐数值
//Funnel参数
export const FUNNEL_DISTANCE = 25;                                //拉直的路径距离边缘多远(0-100，百分比，100%意味着只能走边的中点)
//高度修正参数
export const ADJUST_HEIGHT_DISTANCE = 50;                        //路径中每隔这个距离增加一个点，用于修正高度

/**
 * 返回一个随机的颜色
 * @returns {import("cs_script/point_script").Color}
 */
export function getRandomColor() {
    return {
        r: Math.floor(Math.random() * 255),
        g: Math.floor(Math.random() * 255),
        b: Math.floor(Math.random() * 255),
        a: 255
    };
}
/**
 * 根据体素(i,j,k)坐标返回世界(x,y,z)坐标
 * @param {number} i
 * @param {number} j
 * @param {number} k
 * @returns {import("cs_script/point_script").Vector}
 */
export function getpos(i, j, k) {
    return { x: origin.x + i * MESH_CELL_SIZE_XY, y: origin.y + j * MESH_CELL_SIZE_XY, z: origin.z + k * MESH_CELL_SIZE_Z }
}
/**
 * 得到体素左下角的世界坐标
 * @param {import("cs_script/point_script").Vector} pos
 * @returns {import("cs_script/point_script").Vector}
 */
export function getmins(pos) {
    return { x: pos.x - MESH_CELL_SIZE_XY / 2, y: pos.y - MESH_CELL_SIZE_XY / 2, z: pos.z - MESH_CELL_SIZE_Z / 2 };
}
/**
 * 得到体素右上角的世界坐标
 * @param {import("cs_script/point_script").Vector} pos
 * @returns {import("cs_script/point_script").Vector}
 */
export function getmaxs(pos) {
    return { x: pos.x + MESH_CELL_SIZE_XY / 2, y: pos.y + MESH_CELL_SIZE_XY / 2, z: pos.z + MESH_CELL_SIZE_Z / 2 };
}
/**
 * @param {import("cs_script/point_script").Vector} pos
 */
export function traceGroundAt(pos) {
    const start = { x: pos.x, y: pos.y, z: pos.z + MESH_CELL_SIZE_Z - 1 };
    const end = { x: pos.x, y: pos.y, z: pos.z - 1 };
    const S_E = Instance.TraceLine({ start, end, ignorePlayers: true });
    const E_S = Instance.TraceLine({ start: end, end: start, ignorePlayers: true });
    return { down: S_E, up: E_S }
}
/**
 * @param {import("cs_script/point_script").Vector} pos
 */
export function traceWallAt(pos) {
    const start = { x: pos.x + MESH_CELL_SIZE_XY / 2, y: pos.y, z: pos.z + MESH_CELL_SIZE_Z / 2 };
    const end = { x: pos.x - MESH_CELL_SIZE_XY / 2, y: pos.y, z: pos.z + MESH_CELL_SIZE_Z / 2 };
    let l = Instance.TraceLine({ start, end, ignorePlayers: true });
    if (l && l.didHit) return true;
    l = Instance.TraceLine({ start: end, end: start, ignorePlayers: true });
    if (l && l.didHit) return true;
    start.x = pos.x;
    start.y = pos.y + MESH_CELL_SIZE_XY / 2;
    end.x = pos.x;
    end.y = pos.y - MESH_CELL_SIZE_XY / 2;
    l = Instance.TraceLine({ start, end, ignorePlayers: true });
    if (l && l.didHit) return true;
    l = Instance.TraceLine({ start: end, end: start, ignorePlayers: true });
    if (l && l.didHit) return true;
    return false;
}
/**
 * @param {import("cs_script/point_script").Vector} pos
 */
export function traceGroundpd(pos) {
    const start = { x: pos.x, y: pos.y, z: origin.z + MESH_WORLD_SIZE_Z };
    const end = { x: pos.x, y: pos.y, z: pos.z - 1 };
    return Instance.TraceLine({ start, end, ignorePlayers: true });
}
/**
 * @param {import("cs_script/point_script").Vector} pos
 */
export function traceAirpd(pos) {
    const start = { x: pos.x, y: pos.y, z: pos.z - 1 };
    const end = { x: pos.x, y: pos.y, z: origin.z + MESH_WORLD_SIZE_Z };
    return Instance.TraceLine({ start, end, ignorePlayers: true });
}
/**
 * 返回三点是否共线
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @param {import("cs_script/point_script").Vector} c
 */
export function isCollinear(a, b, c) {
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const bcx = c.x - b.x;
    const bcy = c.y - b.y;
    return abx * bcy - aby * bcx === 0;
}
/**
 * 点p到线段ab距离的平方
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 */
export function distPtSegSq(p, a, b) {
    // 向量 ab 和 ap
    const abX = b.x - a.x;
    const abY = b.y - a.y;
    const apX = p.x - a.x;
    const apY = p.y - a.y;

    // 计算 ab 向量的平方长度
    const abSq = abX * abX + abY * abY;

    // 如果线段的起点和终点重合（abSq 为 0），直接计算点到起点的距离
    if (abSq === 0) {
        return apX * apX + apY * apY;
    }

    // 计算点p在ab上的投影 t
    const t = (apX * abX + apY * abY) / abSq;

    // 计算投影点的位置
    let nearestX, nearestY;

    if (t < 0) {
        // 投影点在a点左侧，最近点是a
        nearestX = a.x;
        nearestY = a.y;
    } else if (t > 1) {
        // 投影点在b点右侧，最近点是b
        nearestX = b.x;
        nearestY = b.y;
    } else {
        // 投影点在线段上，最近点是投影点
        nearestX = a.x + t * abX;
        nearestY = a.y + t * abY;
    }

    // 计算点p到最近点的距离的平方
    const dx = p.x - nearestX;
    const dy = p.y - nearestY;

    return dx * dx + dy * dy;
}
/**
 * xy平面上点abc构成的三角形面积的两倍，>0表示ABC逆时针，<0表示顺时针
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @param {import("cs_script/point_script").Vector} c
 */
export function area(a, b, c) {
    const ab = { x: b.x - a.x, y: b.y - a.y };
    const ac = { x: c.x - a.x, y: c.y - a.y };
    const s2 = (ab.x * ac.y - ac.x * ab.y);
    return s2;
}
/**
 * 返回cur在多边形中是否是锐角
 * @param {import("cs_script/point_script").Vector} prev
 * @param {import("cs_script/point_script").Vector} cur
 * @param {import("cs_script/point_script").Vector} next
 */
export function isConvex(prev, cur, next) {
    return area(prev, cur, next) > 0;
}
/**
 * xy平面上点p是否在abc构成的三角形内（不包括边上）
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @param {import("cs_script/point_script").Vector} c
 */
export function pointInTri(p, a, b, c) {
    const ab = area(a, b, p);
    const bc = area(b, c, p);
    const ca = area(c, a, p);
    //内轮廓与外轮廓那里会有顶点位置相同的时候
    return ab > 0 && bc > 0 && ca > 0;
}
/**
 * 点到线段最近点
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 */
export function closestPointOnSegment(p, a, b) {
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const abz = b.z - a.z;

    const apx = p.x - a.x;
    const apy = p.y - a.y;
    const apz = p.z - a.z;

    const d = abx * abx + aby * aby + abz * abz;
    let t = d > 0 ? (apx * abx + apy * aby + apz * abz) / d : 0;
    t = Math.max(0, Math.min(1, t));

    return {
        x: a.x + abx * t,
        y: a.y + aby * t,
        z: a.z + abz * t,
    };
}
/**
 * 点是否在凸多边形内(xy投影)
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector[]} verts
 * @param {number[]} poly
 */
export function pointInConvexPolyXY(p, verts, poly) {
    for (let i = 0; i < poly.length; i++) {
        const a = verts[poly[i]];
        const b = verts[poly[(i + 1) % poly.length]];
        if (area(a, b, p) < 0) return false;
    }
    return true;
}
/**
 * 点到 polygon 最近点(xy投影)
 * @param {import("cs_script/point_script").Vector} pos
 * @param {import("cs_script/point_script").Vector[]} verts
 * @param {number[]} poly
 */
export function closestPointOnPoly(pos, verts, poly) {
    // 1. 如果在多边形内部（XY），直接投影到平面
    if (pointInConvexPolyXY(pos, verts, poly)) {
        // 用平均高度（你也可以用平面方程）
        let maxz = -Infinity,minz=Infinity;
        for (const vi of poly)
        {
            maxz =Math.max(maxz,verts[vi].z);
            minz =Math.min(minz,verts[vi].z);
        }

        return { x: pos.x, y: pos.y, z:0,in:true};
    }

    // 2. 否则，找最近边
    let best = null;
    let bestDist = Infinity;

    for (let i = 0; i < poly.length; i++) {
        const a = verts[poly[i]];
        const b = verts[poly[(i + 1) % poly.length]];
        const c = closestPointOnSegment(pos, a, b);

        const dx = c.x - pos.x;
        const dy = c.y - pos.y;
        const dz = c.z - pos.z;
        const d = dx * dx + dy * dy + dz * dz;

        if (d < bestDist) {
            bestDist = d;
            best = {x: c.x, y: c.y, z:c.z,in:false};
        }
    }

    return best;
}
