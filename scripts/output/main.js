import { Instance } from 'cs_script/point_script';

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("cs_script/point_script").Color} Color */
const PathState = {
    WALK: 1,//下一个点直走
    JUMP: 2,//下一个点需要跳跃
    LADDER: 3//下一个点是梯子点，可能是起点也可能是终点，到这个点开启梯子状态，直到下一个点不是梯子
};
//==============================对外接口=========================================
/**
 * update(pos);         //更新pos所在tile，平均tile构建会最多卡1秒，减少tile大小或者增加体素大小均可改善
 * findPath(start,end); //寻路，返回路径点数组，路径点格式：{pos:Vector,mode:number}，mode是PathState类型,表示到下一个点的移动方式
 * debug(time);         //调试工具，持续time秒
 * init();              //初始化生成整个导航网格
 */
//==============================世界相关设置=====================================
const origin = { x: -3050, y: -2725, z: -780 };
const MESH_CELL_SIZE_XY = 8;                               // 体素大小
const MESH_CELL_SIZE_Z = 1;                                // 体素高度
const MESH_TRACE_SIZE_Z = 32;                              // 射线方块高度//太高，会把竖直方向上的间隙也忽略
const MESH_WORLD_SIZE_XY = 6600;                           // 世界大小
const MESH_WORLD_SIZE_Z = 980;                             // 世界高度
//==============================Recast设置======================================
//其他参数
const MAX_SLOPE = 65;                                      // 最大坡度（度数），超过这个角度的斜面将被视为不可行走
const MAX_WALK_HEIGHT = 13 / MESH_CELL_SIZE_Z;             // 怪物最大可行走高度（体素单位）
const MAX_JUMP_HEIGHT = 63 / MESH_CELL_SIZE_Z;             // 怪物最大可跳跃高度（体素单位）
const AGENT_RADIUS = 8 / MESH_CELL_SIZE_XY;               // 人物通过所需宽度半径大小（长度）
const AGENT_HEIGHT = 40 / MESH_CELL_SIZE_Z;                // 人物高度（体素单位）
//TILE参数
const TILE_SIZE = 512 / MESH_CELL_SIZE_XY;                 // 瓦片大小（体素单位），每个瓦片包含 tileSize*tileSize 个体素，过大可能导致性能问题，过小可能导致内存占用增加
const TILE_PADDING = AGENT_RADIUS + 1;                     // 瓦片边界添加的体素数量，防止寻路时穿模，值需要比MESH_ERODE_RADIUS大
//体素化参数
const MESH_ERODE_RADIUS = AGENT_RADIUS;                    // 按行走高度，腐蚀半径
//区域生成参数
const REGION_MERGE_AREA = 128;                             // 合并区域阈值（体素单位）
const REGION_MIN_AREA = 0;                                 // 最小区域面积（体素单位）
//轮廓生成参数
const CONT_MAX_ERROR = 1.5;                                // 原始点到简化后边的最大允许偏离距离（体素距离）
const POLY_MAX_VERTS_PER_POLY = 6;                         // 每个多边形的最大顶点数
const POLY_MERGE_LONGEST_EDGE_FIRST = true;                // 优先合并最长边
const POLY_DETAIL_SAMPLE_DIST = 3;                         // 构建细节网格时，相邻采样点之间的“间距”,选3耗时比较合适
const POLY_DETAIL_HEIGHT_ERROR = 5;                        // 构建细节网格时，采样点和计算点之差如果小于这个高度阈值就跳过
const ASTAR_HEURISTIC_SCALE = 1.2;                         //A*推荐数值
//Funnel参数
const FUNNEL_DISTANCE = 15;                                //拉直的路径距离边缘多远(0-100，百分比，100%意味着只能走边的中点)
//高度修正参数
const ADJUST_HEIGHT_DISTANCE = 50;                         //路径中每隔这个距离增加一个点，用于修正高度
/**
 * 点p到线段ab距离的平方
 * @param {Vector} p
 * @param {Vector} a
 * @param {Vector} b
 */
function distPtSegSq(p, a, b) {
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
 * @param {Vector} a
 * @param {Vector} b
 * @param {Vector} c
 */
function area(a, b, c) {
    const ab = { x: b.x - a.x, y: b.y - a.y };
    const ac = { x: c.x - a.x, y: c.y - a.y };
    const s2 = (ab.x * ac.y - ac.x * ab.y);
    return s2;
}
/**
 * 返回cur在多边形中是否是锐角
 * @param {Vector} prev
 * @param {Vector} cur
 * @param {Vector} next
 */
function isConvex(prev, cur, next) {
    return area(prev, cur, next) > 0;
}
/**
 * xy平面上点p是否在abc构成的三角形内（不包括边上）
 * @param {Vector} p
 * @param {Vector} a
 * @param {Vector} b
 * @param {Vector} c
 */
function pointInTri(p, a, b, c) {
    const ab = area(a, b, p);
    const bc = area(b, c, p);
    const ca = area(c, a, p);
    //内轮廓与外轮廓那里会有顶点位置相同的时候
    return ab > 0 && bc > 0 && ca > 0;
}
/**
 * 点到线段最近点
 * @param {Vector} p
 * @param {Vector} a
 * @param {Vector} b
 */
function closestPointOnSegment(p, a, b) {
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
 * @param {Vector} p
 * @param {Vector[]} verts
 * @param {number[]} poly
 */
function pointInConvexPolyXY(p, verts, poly) {
    for (let i = 0; i < poly.length; i++) {
        const a = verts[poly[i]];
        const b = verts[poly[(i + 1) % poly.length]];
        if (area(a, b, p) < 0) return false;
    }
    return true;
}
/**
 * 点到 polygon 最近点(xy投影)
 * @param {Vector} pos
 * @param {Vector[]} verts
 * @param {number[]} poly
 */
function closestPointOnPoly(pos, verts, poly) {
    // 1. 如果在多边形内部（XY），直接投影到平面
    if (pointInConvexPolyXY(pos, verts, poly)) {
        // 用平均高度（你也可以用平面方程）
        let maxz = -Infinity, minz = Infinity;
        for (const vi of poly) {
            maxz = Math.max(maxz, verts[vi].z);
            minz = Math.min(minz, verts[vi].z);
        }

        return { x: pos.x, y: pos.y, z: (maxz + minz) / 2, in: true };
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
            best = { x: c.x, y: c.y, z: c.z, in: false };
        }
    }

    return best;
}

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */

class FunnelHeightFixer {
    /**
    * @param {NavMeshMesh} navMesh
    * @param {NavMeshDetail} detailMesh
     * @param {number} stepSize
     */
    constructor(navMesh, detailMesh, stepSize = 0.5) {
        this.navMesh = navMesh;
        this.detailMesh = detailMesh;
        this.stepSize = stepSize;
        const polyCount = detailMesh.meshes.length;
        this.polyTriStart = new Uint16Array(polyCount);
        this.polyTriEnd   = new Uint16Array(polyCount);
        this.polyHasDetail = new Uint8Array(polyCount);
        for (let i = 0; i < polyCount; i++) {
            const mesh = detailMesh.meshes[i];
            const baseTri  = mesh[2];
            const triCount = mesh[3];
            this.polyHasDetail[i] = (triCount > 0)?1:0;
            this.polyTriStart[i] = baseTri;
            this.polyTriEnd[i]   = baseTri + triCount; // [start, end)
        }
        this.triAabbMinX = [];
        this.triAabbMinY = [];
        this.triAabbMaxX = [];
        this.triAabbMaxY = [];
        const { verts, tris } = detailMesh;

        for (let i = 0; i < tris.length; i++) {
            const [ia, ib, ic] = tris[i];
            const a = verts[ia];
            const b = verts[ib];
            const c = verts[ic];

            const minX = Math.min(a.x, b.x, c.x);
            const minY = Math.min(a.y, b.y, c.y);
            const maxX = Math.max(a.x, b.x, c.x);
            const maxY = Math.max(a.y, b.y, c.y);

            this.triAabbMinX[i] = minX;
            this.triAabbMinY[i] = minY;
            this.triAabbMaxX[i] = maxX;
            this.triAabbMaxY[i] = maxY;
        }

    }

    /* ===============================
       Public API
    =============================== */
    
    /**
     * @param {{ x: number; y: number; z: any; }} pos
     * @param {number} polyid
     * @param {{ id: number; mode: number; }[]} polyPath
     * @param {{ pos: { x: number; y: number; z: number; }; mode: number; }[]} out
     */
    addpoint(pos,polyid,polyPath,out)
    {
        polyid = this._advancePolyIndex(pos, polyid, polyPath);

        if (polyid >= polyPath.length) return;
        const h = this._getHeightOnDetail(polyPath[polyid].id, pos);
        out.push({
            pos: { x: pos.x, y: pos.y, z: h },
            mode: PathState.WALK
        });
        //Instance.DebugSphere({center:{ x: pos.x, y: pos.y, z: h },radius:1,duration:1/32,color:{r:0,g:255,b:0}});
                
    }
    /**
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} funnelPath
     * @param {{id:number,mode:number}[]} polyPath
     */
    fixHeight(funnelPath,polyPath) {
        if (funnelPath.length === 0) return [];
        /** @type {{pos:{x:number,y:number,z:number},mode:number}[]} */
        const result = [];
        let polyIndex = 0;
        
        for (let i = 0; i < funnelPath.length - 1; i++) {
            const curr = funnelPath[i];
            const next = funnelPath[i + 1];

            // 梯子点：始终原样保留，不参与地面采样。
            // 否则会把 LADDER 点重写成 WALK，出现“梯子被跳过”的现象。
            if (curr.mode == PathState.LADDER) {
                result.push(curr);
                continue;
            }
            // 跳跃落点(next=JUMP)：前一段不做插值，等待下一轮由 curr=JUMP 处理落地点。
            if (next.mode == PathState.JUMP) {
                result.push(curr);
                continue;
            }
            if (curr.mode == PathState.JUMP) result.push(curr);
            // 分段采样
            const samples = this._subdivide(curr.pos, next.pos);
            //Instance.Msg(samples.length);
            curr.pos.z;
            for (let j = (curr.mode == PathState.JUMP)?1:0; j < samples.length; j++) {
                const p = samples[j];
                // 推进 poly corridor
                polyIndex = this._advancePolyIndex(p, polyIndex, polyPath);

                if (polyIndex >= polyPath.length) break;
                const polyId = polyPath[polyIndex].id;
                const h = this._getHeightOnDetail(polyId, p);
                //如果这个样本点比前一个点高度发生足够变化，就在中间加入一个样本点
                //if(j>0&&Math.abs(preh-h)>5)
                //{
                //    const mid={x:(p.x+prep.pos.x)/2,y:(p.y+prep.pos.y)/2,z:p.z};
                //    this.addpoint(mid,preid,polyPath,result);
                //}
                result.push({
                    pos: { x: p.x, y: p.y, z: h },
                    mode: PathState.WALK
                });
                //Instance.DebugSphere({center:{ x: p.x, y: p.y, z: h },radius:1,duration:1/32,color:{r:255,g:0,b:0}});
                p.z;
                result[result.length - 1];
            }
        }

        const last = funnelPath[funnelPath.length - 1];
        if (result.length === 0 || result[result.length - 1].pos.x !== last.pos.x || result[result.length - 1].pos.y !== last.pos.y || result[result.length - 1].pos.z !== last.pos.z || result[result.length - 1].mode !== last.mode) {
            result.push(last);
        }

        return result;
    }

    /* ===============================
       Subdivide
    =============================== */

    /**
     * @param {{ x: any; y: any; z: any; }} a
     * @param {{ x: any; y: any; z?: number; }} b
     */
    _subdivide(a, b) {
        const out = [];
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const dist = Math.hypot(dx, dy);

        if (dist <= this.stepSize) {
            out.push(a);
            return out;
        }

        const n = Math.floor(dist / this.stepSize);
        for (let i = 0; i < n; i++) {
            const t = i / n;
            out.push({
                x: a.x + dx * t,
                y: a.y + dy * t,
                z: a.z
            });
        }
        return out;
    }

    /* ===============================
       Height Query
    =============================== */

    /**
     * @param {number} polyId
     * @param {{ z: number; y: number; x: number; }} p
     */
    _getHeightOnDetail(polyId, p) {
        const { verts, tris } = this.detailMesh;
        const start = this.polyTriStart[polyId];
        const end   = this.polyTriEnd[polyId];
        if(this.polyHasDetail[polyId]==0)return p.z;
        const px = p.x;
        const py = p.y;
        for (let i = start; i < end; i++) {
            if (
                px < this.triAabbMinX[i] || px > this.triAabbMaxX[i] ||
                py < this.triAabbMinY[i] || py > this.triAabbMaxY[i]
            ) {
                continue;
            }
            const [a, b, c] = tris[i];
            const va = verts[a];
            const vb = verts[b];
            const vc = verts[c];

            if (this._pointInTriXY(p, va, vb, vc)) {
                return this._baryHeight(p, va, vb, vc);
            }
        }

        // fallback（极少发生）
        return p.z;
    }
    /**
     * 三角形内插高度
     * @param {{ x: number; y: number; }} p
     * @param {{ x: any; y: any; z: any; }} a
     * @param {{ x: any; y: any; z: any; }} b
     * @param {{ x: any; y: any; z: any; }} c
     */
    _baryHeight(p, a, b, c) {
        const v0x = b.x - a.x, v0y = b.y - a.y;
        const v1x = c.x - a.x, v1y = c.y - a.y;
        const v2x = p.x - a.x, v2y = p.y - a.y;

        const d00 = v0x * v0x + v0y * v0y;
        const d01 = v0x * v1x + v0y * v1y;
        const d11 = v1x * v1x + v1y * v1y;
        const d20 = v2x * v0x + v2y * v0y;
        const d21 = v2x * v1x + v2y * v1y;

        const denom = d00 * d11 - d01 * d01;
        const v = (d11 * d20 - d01 * d21) / denom;
        const w = (d00 * d21 - d01 * d20) / denom;
        const u = 1.0 - v - w;

        return u * a.z + v * b.z + w * c.z;
    }

    /* ===============================
       Geometry helpers
    =============================== */

    /**
    * @param {{ y: number; x: number; z:number}} p
     * @param {number} polyId
     */
    _pointInPolyXY(p, polyId) {
        const poly = this.navMesh.polys[polyId];
        return pointInConvexPolyXY(p,this.navMesh.verts,poly);
    }
    /**
     * @param {{ y: number; x: number; }} p
     * @param {{ x: number; y: number;}} a
     * @param {{ x: number; y: number;}} b
     * @param {{ x: number; y: number;}} c
     */
    _pointInTriXY(p, a, b, c) {
        const s = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
        const t = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
        const u = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
        return (s >= 0 && t >= 0 && u >= 0) || (s <= 0 && t <= 0 && u <= 0);
    }

    /**
     * 优先用最近多边形推进 corridor，点一定在多边形投影内
     * @param {{x:number,y:number,z:number}} p
     * @param {number} startIndex
     * @param {{id:number,mode:number}[]} polyPath
     */
    _advancePolyIndex(p, startIndex, polyPath) {

        let bestIndex = startIndex;
        let bestDistSq = Infinity;

        for (let i = startIndex; i < startIndex+1; i++) {
            const polyId = polyPath[i].id;
            const poly = this.navMesh.polys[polyId];

            const cp = closestPointOnPoly(p, this.navMesh.verts, poly);
            if (!cp||!cp.in) continue;
            cp.z = cp.z;
            const dx = cp.x - p.x;
            const dy = cp.y - p.y;
            const dz = cp.z - p.z;
            const d = dx * dx + dy * dy + dz * dz;
            if (d < bestDistSq) {
                bestDistSq = d;
                bestIndex = i;
            }
        }
        return bestIndex;
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("../path_manager").NavMeshMesh} NavMeshMesh */
// 查询所在多边形优化
let spatialCellSize = 128;
let spatialGrid = new Map();
/**
 * Tool: NavMesh 与路径模块共享的纯工具函数集合（无状态静态方法）。
 * - 仅放“可复用且与业务对象解耦”的逻辑
 * - 不持有运行时状态，调用开销低
 */
class Tool {
    /**
        * 数值夹取。
     *
     * @param {number} value
     * @param {number} min
     * @param {number} max
     * @returns {number}
     */
    static clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
        * 三维向量线性插值。
        * t=0 返回 a，t=1 返回 b。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @param {number} t
     * @returns {Vector}
     */
    static lerpVector(a, b, t) {
        return {
            x: a.x + (b.x - a.x) * t,
            y: a.y + (b.y - a.y) * t,
            z: a.z + (b.z - a.z) * t
        };
    }

    /**
        * 生成“无序点对”稳定 key。
        * (a,b) 与 (b,a) 会得到相同 key。
     *
     * @param {number} a
     * @param {number} b
     * @param {string} [separator]
     * @returns {string}
     */
    static orderedPairKey(a, b, separator = "-") {
        const lo = Math.min(a, b);
        const hi = Math.max(a, b);
        return `${lo}${separator}${hi}`;
    }

    /**
        * 生成二维网格索引 key（x,y）。
     *
     * @param {number} x
     * @param {number} y
     * @param {string} [separator]
     * @returns {string}
     */
    static gridKey2(x, y, separator = "_") {
        return `${x}${separator}${y}`;
    }

    /**
        * Map<string, T[]> 的“取或建”辅助。
        * key 不存在时自动创建空数组并返回。
     *
     * @template T
     * @param {Map<string, T[]>} map
     * @param {string} key
     * @returns {T[]}
     */
    static getOrCreateArray(map, key) {
        let list = map.get(key);
        if (!list) {
            list = [];
            map.set(key, list);
        }
        return list;
    }

    /**
        * 点是否在线段上（XY 平面）。
        * - includeEndpoints=true: 端点算在线段上
        * - includeEndpoints=false: 端点不算在线段上（严格在线段内部）
     *
     * @param {number} px
     * @param {number} py
     * @param {number} x1
     * @param {number} y1
     * @param {number} x2
     * @param {number} y2
     * @param {{includeEndpoints?: boolean, epsilon?: number}} [options]
     * @returns {boolean}
     */
    static pointOnSegment2D(px, py, x1, y1, x2, y2, options) {
        const epsilon = options?.epsilon ?? 1e-6;
        const includeEndpoints = options?.includeEndpoints ?? true;

        const cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
        if (Math.abs(cross) > epsilon) return false;

        const dot = (px - x1) * (px - x2) + (py - y1) * (py - y2);
        return includeEndpoints ? dot <= epsilon : dot < -epsilon;
    }
    /**
     * @param {{verts: {x: number;y: number;z: number;}[];polys: number[][];neighbors: number[][][];}} mesh
     */
    static buildSpatialIndex(mesh) {
        spatialGrid.clear();
        for (let i = 0; i < mesh.polys.length; i++) {
            const poly = mesh.polys[i];

            let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
            for (const vi of poly) {
                const v = mesh.verts[vi];
                if (v.x < minX) minX = v.x;
                if (v.y < minY) minY = v.y;
                if (v.x > maxX) maxX = v.x;
                if (v.y > maxY) maxY = v.y;
            }

            const x0 = Math.floor(minX / spatialCellSize);
            const x1 = Math.ceil(maxX / spatialCellSize);
            const y0 = Math.floor(minY / spatialCellSize);
            const y1 = Math.ceil(maxY / spatialCellSize);

            for (let x = x0; x <= x1; x++) {
                for (let y = y0; y <= y1; y++) {
                    const key = `${x}_${y}`;
                    if (!spatialGrid.has(key)) spatialGrid.set(key, []);
                    spatialGrid.get(key).push(i);
                }
            }
        }
    }
    /**
     * 返回包含点的 poly index，找不到返回 -1
     * @param {Vector} p
     * @param {NavMeshMesh} mesh
     * @param {FunnelHeightFixer}[heightfixer]
     */
    static findNearestPoly(p,mesh,heightfixer) {
        let bestPoly = -1;
        let bestDist = Infinity;
        let bestPos = p;
        const x = Math.floor(p.x / spatialCellSize);
        const y = Math.floor(p.y / spatialCellSize);
        for (let i = -1; i <= 1; i++) {
            for (let j = -1; j <= 1; j++) {
                const key = `${x + i}_${y + j}`;
                //const key = `${x}_${y}`;
                const candidates = spatialGrid.get(key);
                if (!candidates) continue;
                //if (!candidates) return{pos:bestPos,poly:bestPoly};
                for (const polyIdx of candidates) {
                    const poly = mesh.polys[polyIdx];
                    const cp = closestPointOnPoly(p, mesh.verts, poly);
                    if (!cp) continue;

                    if (cp.in == true) {
                        const h = heightfixer?._getHeightOnDetail(polyIdx, p);
                        cp.z = h ?? cp.z;
                    }
                    //Instance.DebugSphere({center:{x:cp.x,y:cp.y,z:cp.z},radius:2,duration:30,color:{r:255,g:255,b:255}});
                    const dx = cp.x - p.x;
                    const dy = cp.y - p.y;
                    const dz = cp.z - p.z;
                    const d = dx * dx + dy * dy + dz * dz;

                    if (d < bestDist) {
                        bestDist = d;
                        bestPoly = polyIdx;
                        bestPos = cp;
                    }
                }
            }
        }
        //Instance.DebugSphere({center:{x:bestPos.x,y:bestPos.y,z:bestPos.z},radius:2,duration:30,color:{r:255,g:255,b:255}});
                    
        return { pos: bestPos, poly: bestPoly };
    }
}

/**
 * 并查集（Disjoint Set Union）。
 * 用于连通分量合并与查询（例如可达性裁剪）。
 */
class UnionFind {
    /**
     * 创建 size 个独立集合。
     *
     * @param {number} size
     */
    constructor(size) {
        this.parent = new Int32Array(size);
        this.rank = new Uint8Array(size);
        for (let i = 0; i < size; i++) this.parent[i] = i;
    }

    /**
        * 查找元素所属集合根节点（含路径压缩）。
     *
     * @param {number} x
     * @returns {number}
     */
    find(x) {
        let root = x;
        while (this.parent[root] !== root) root = this.parent[root];
        while (this.parent[x] !== x) {
            const p = this.parent[x];
            this.parent[x] = root;
            x = p;
        }
        return root;
    }

    /**
        * 合并 a 与 b 所在集合（按秩合并）。
     *
     * @param {number} a
     * @param {number} b
     */
    union(a, b) {
        let ra = this.find(a);
        let rb = this.find(b);
        if (ra === rb) return;
        if (this.rank[ra] < this.rank[rb]) {
            const t = ra;
            ra = rb;
            rb = t;
        }
        this.parent[rb] = ra;
        if (this.rank[ra] === this.rank[rb]) this.rank[ra]++;
    }
}

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("cs_script/point_script").Vector} Vector */

class PolyGraphAStar {
    /**
    * @param {NavMeshMesh} polys
    * @param {Map<number,NavMeshLink[]>} links
     * @param {FunnelHeightFixer} heightfixer
     */
    constructor(polys, links, heightfixer) {
        this.mesh = polys;
        this.polyCount = polys.polys.length;
        /**@type {Map<number,NavMeshLink[]>} */
        this.links = links;
        this.heightfixer = heightfixer;
        //预计算中心点
        this.centers = new Array(this.polyCount);
        for (let i = 0; i < this.polyCount; i++) {
            const poly = this.mesh.polys[i];
            let x = 0, y = 0, z = 0;
            for (const vi of poly) {
                const v = this.mesh.verts[vi];
                x += v.x; y += v.y; z += v.z;
            }
            const n = poly.length;
            this.centers[i] = {
                x: x / n, y: y / n, z: z / n
            };
        }

        this.heuristicScale = ASTAR_HEURISTIC_SCALE;
        Instance.Msg("多边形总数：" + this.polyCount + "跳点数：" + links.size);
        this.open = new MinHeap(this.polyCount);
    }

    /**
     * @param {import("cs_script/point_script").Vector} start
     * @param {import("cs_script/point_script").Vector} end
     */
    findPath(start, end) {
        const startPoly = Tool.findNearestPoly(start, this.mesh,this.heightfixer);
        const endPoly = Tool.findNearestPoly(end, this.mesh,this.heightfixer);
        //Instance.Msg(startPoly.poly+"   "+endPoly.poly);
        if (startPoly.poly < 0 || endPoly.poly < 0) {
            Instance.Msg(`跑那里去了?`);
            return { start: startPoly.pos, end: endPoly.pos, path: [] };
        }

        if (startPoly.poly == endPoly.poly) {
            return { start: startPoly.pos, end: endPoly.pos, path: [{ id: endPoly.poly, mode: 1 }] };
        }
        return { start: startPoly.pos, end: endPoly.pos, path: this.findPolyPath(startPoly.poly, endPoly.poly) };
    }
    /**
     * @param {number} start
     * @param {number} end
     */
    findPolyPath(start, end) {
        const open = this.open;
        const g = new Float32Array(this.polyCount);
        const parent = new Int32Array(this.polyCount);
        const walkMode = new Uint8Array(this.polyCount);// 0=none,1=walk,2=jump,//待更新3=climb
        const state = new Uint8Array(this.polyCount); // 0=none,1=open,2=closed
        g.fill(Infinity);
        parent.fill(-1);
        open.clear();
        g[start] = 0;
        // @ts-ignore
        open.push(start, this.distsqr(start, end) * this.heuristicScale);
        state[start] = 1;

        let closestNode = start;
        let minH = Infinity;

        while (!open.isEmpty()) {
            const current = open.pop();

            if (current === end) return this.reconstruct(parent, walkMode, end);
            state[current] = 2;

            const hToTarget = this.distsqr(current, end);
            if (hToTarget < minH) {
                minH = hToTarget;
                closestNode = current;
            }

            const neighbors = this.mesh.neighbors[current];
            for (let i = 0; i < neighbors.length; i++) {
                for (const n of neighbors[i]) {
                    if (n < 0 || state[n] == 2) continue;
                    // @ts-ignore
                    const tentative = g[current] + this.distsqr(current, n);
                    if (tentative < g[n]) {
                        parent[n] = current;
                        walkMode[n] = PathState.WALK;
                        g[n] = tentative;
                        // @ts-ignore
                        const f = tentative + this.distsqr(n, end) * this.heuristicScale;
                        if (state[n] != 1) {
                            open.push(n, f);
                            state[n] = 1;
                        }
                        else open.update(n, f);
                    }
                }
            }
            if (!this.links.has(current)) continue;
            // @ts-ignore
            for (const link of this.links.get(current)) {
                let v = -1;
                if (link.PolyA == current) v = link.PolyB;
                else if (link.PolyB == current) v = link.PolyA;
                if (v == -1 || state[v] == 2) continue;
                const moveCost = link.cost * link.cost;
                if (g[current] + moveCost < g[v]) {
                    g[v] = g[current] + moveCost;
                    // @ts-ignore
                    const f = g[v] + this.distsqr(v, end) * this.heuristicScale;
                    parent[v] = current;
                    walkMode[v] = link.type;
                    if (state[v] != 1) {
                        open.push(v, f);
                        state[v] = 1;
                    }
                    else open.update(v, f);
                }
            }
        }
        return this.reconstruct(parent, walkMode, closestNode);
    }
    /**
     * @param {Int32Array} parent
     * @param {Uint8Array} walkMode
     * @param {number} cur
     */
    reconstruct(parent, walkMode, cur) {
        const path = [];
        while (cur !== -1) {
            path.push({ id: cur, mode: walkMode[cur] });
            cur = parent[cur];
        }
        return path.reverse();
    }

    /**
     * @param {number} a
     * @param {number} b
     */
    distsqr(a, b) {
        const pa = this.centers[a];
        const pb = this.centers[b];
        const dx = pa.x - pb.x;
        const dy = pa.y - pb.y;
        const dz = pa.z - pb.z;
        return dx * dx + dy * dy + dz * dz;
    }
}
class MinHeap {
    /**
     * @param {number} polyCount
     */
    constructor(polyCount) {
        this.nodes = new Int32Array(polyCount);
        this.costs = new Float32Array(polyCount);
        this.index = new Int32Array(polyCount).fill(-1);
        this.size = 0;
    }
    clear() {
        this.index.fill(-1);
        this.size = 0;
    }
    isEmpty() {
        return this.size === 0;
    }

    /**
     * @param {number} node
     * @param {number} cost
     */
    push(node, cost) {
        let i = this.size++;
        this.nodes[i] = node;
        this.costs[i] = cost;
        this.index[node] = i;
        this._up(i);
    }

    pop() {
        if (this.size === 0) return -1;
        const topNode = this.nodes[0];
        this.index[topNode] = -1;
        this.size--;
        if (this.size > 0) {
            this.nodes[0] = this.nodes[this.size];
            this.costs[0] = this.costs[this.size];
            this.index[this.nodes[0]] = 0;
            this._down(0);
        }
        return topNode;
    }

    /**
     * @param {number} node
     * @param {number} cost
     */
    update(node, cost) {
        const i = this.index[node];
        if (i < 0) return;
        this.costs[i] = cost;
        this._up(i);
    }

    /**
     * @param {number} i
     */
    _up(i) {
        while (i > 0) {
            const p = (i - 1) >> 1;
            if (this.costs[p] <= this.costs[i]) break;
            this._swap(i, p);
            i = p;
        }
    }

    /**
     * @param {number} i
     */
    _down(i) {
        const n = this.size;
        while (true) {
            let l = i * 2 + 1;
            let r = l + 1;
            let m = i;

            if (l < n && this.costs[l] < this.costs[m]) m = l;
            if (r < n && this.costs[r] < this.costs[m]) m = r;
            if (m === i) break;

            this._swap(i, m);
            i = m;
        }
    }

    /**
     * @param {number} a
     * @param {number} b
     */
    _swap(a, b) {
        const ca = this.costs[a];
        const cb = this.costs[b];
        const na = this.nodes[a];
        const nb = this.nodes[b];
        this.costs[a] = cb;
        this.costs[b] = ca;
        this.nodes[a] = nb;
        this.nodes[b] = na;
        this.index[na] = b;
        this.index[nb] = a;
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
class FunnelPath {
    /**
    * @param {NavMeshMesh} mesh
     * @param {Vector[]} centers
     * @param {Map<number,NavMeshLink[]>} links
     */
    constructor(mesh, centers, links) {
        this.mesh = mesh;
        this.centers = centers;
        /**@type {Map<number,NavMeshLink[]>} */
        this.links = links;
        //Instance.Msg(this.links.size);
    }
    // 返回 pA 到 pB 的跳点
    /**
     * @param {number} polyA
     * @param {number} polyB
     */
    getlink(polyA, polyB) {
        // @ts-ignore
        for (const link of this.links.get(polyA)) {
            if (link.PolyB == polyB) return { start: link.PosB, end: link.PosA };
            if(link.PolyA == polyB)return { start: link.PosA, end: link.PosB };
        }
    }
    /**
     * @param {{id:number,mode:number}[]} polyPath
     * @param {Vector} startPos
     * @param {Vector} endPos
     */
    build(polyPath, startPos, endPos) {
        if (!polyPath || polyPath.length === 0) return [];
        if (polyPath.length === 1) return [{pos:startPos,mode:PathState.WALK}, {pos:endPos,mode:PathState.WALK}];
        const ans = [];
        // 当前这一段行走路径的起点坐标
        let currentSegmentStartPos = startPos;
        // 当前这一段行走路径在 polyPath 中的起始索引
        let segmentStartIndex = 0;
        for (let i = 1; i < polyPath.length; i++) {
            const prevPoly = polyPath[i - 1];
            const currPoly = polyPath[i];
            if (currPoly.mode === PathState.JUMP || currPoly.mode === PathState.LADDER)// 到第 i 个多边形需要特殊过渡（跳跃/梯子）
            {
                // 1. 获取跳点坐标信息
                const linkInfo = this.getlink(currPoly.id,prevPoly.id);
                if (!linkInfo)continue;
                const walkPathSegment = polyPath.slice(segmentStartIndex, i);
                const portals = this.buildPortals(walkPathSegment, currentSegmentStartPos, linkInfo.start, FUNNEL_DISTANCE);
                const smoothedWalk = this.stringPull(portals);
                for (const p of smoothedWalk) ans.push({pos:p,mode:PathState.WALK});
                ans.push({pos:linkInfo.end,mode:currPoly.mode});
                currentSegmentStartPos = linkInfo.end; // 下一段从落地点开始
                segmentStartIndex = i; // 下一段多边形从 currPoly 开始
            }
        }
        const lastWalkSegment = polyPath.slice(segmentStartIndex, polyPath.length);
        const lastPortals = this.buildPortals(lastWalkSegment, currentSegmentStartPos, endPos, FUNNEL_DISTANCE);
        const lastSmoothed = this.stringPull(lastPortals);

        for (const p of lastSmoothed) ans.push({pos:p,mode:PathState.WALK});
        return this.removeDuplicates(ans);
    }
    /**
    * 简单去重，防止相邻点坐标完全一致
     * @param {{pos:Vector,mode:number}[]} path
     */
    removeDuplicates(path) {
        if (path.length < 2) return path;
        const res = [path[0]];
        for (let i = 1; i < path.length; i++) {
            const last = res[res.length - 1];
            const curr = path[i];
            const d = (last.pos.x - curr.pos.x) ** 2 + (last.pos.y - curr.pos.y) ** 2 + (last.pos.z - curr.pos.z) ** 2;
            // 容差阈值
            if (d > 0.001) {
                res.push(curr);
            }
        }
        return res;
    }
    /* ===============================
       Portal Construction
    =============================== */

    /**
     * @param {{id:number,mode:number}[]} polyPath
     * @param {Vector} startPos
     * @param {Vector} endPos
     * @param {number} funnelDistance
     */
    buildPortals(polyPath, startPos, endPos, funnelDistance) {
        const portals = [];

        // 起点
        portals.push({ left: startPos, right: startPos });
        for (let i = 0; i < polyPath.length - 1; i++) {
            const a = polyPath[i].id;
            const b = polyPath[i + 1].id;
            const por = this.findPortal(a, b, funnelDistance);
            if (!por) continue;
            portals.push(por);
        }
        // 终点
        portals.push({ left: endPos, right: endPos });
        return portals;
    }

    /**
    * 寻找两个多边形的公共边
     * @param {number} pa
     * @param {number} pb
     * @param {number} funnelDistance
     */
    findPortal(pa, pb, funnelDistance) {
        const poly = this.mesh.polys[pa];
        const neigh = this.mesh.neighbors[pa];

        for (let ei = 0; ei < neigh.length; ei++) {
            if (!neigh[ei].includes(pb)) continue;

            const v0 = this.mesh.verts[poly[ei]];
            const v1 = this.mesh.verts[poly[(ei + 1) % poly.length]];

            // 统一左右（从 pa 看向 pb）
            const ca = this.centers[pa];
            const cb = this.centers[pb];

            if (area(ca, cb, v0) < 0) {
                return this._applyFunnelDistance(v0, v1, funnelDistance);
            } else {
                return this._applyFunnelDistance(v1, v0, funnelDistance);
            }
        }
    }
    /**
    * 根据参数收缩门户宽度
     * @param {Vector} left 
     * @param {Vector} right 
     * @param {number} distance 0-100
     */
    _applyFunnelDistance(left, right, distance) {
        // 限制在 0-100
        const t = Tool.clamp(distance, 0, 100) / 100.0;

        // 若 t 为 0，保持原样
        if (t === 0) return { left, right };

        // 计算中点
        const midX = (left.x + right.x) * 0.5;
        const midY = (left.y + right.y) * 0.5;
        const midZ = (left.z + right.z) * 0.5;
        const mid = { x: midX, y: midY, z: midZ };

        // 使用线性插值将端点向中点移动
        // t=0 保持端点, t=1 变成中点
        const newLeft = Tool.lerpVector(left, mid, t);
        const newRight = Tool.lerpVector(right, mid, t);

        return { left: newLeft, right: newRight };
    }
    /* ===============================
       Funnel (String Pull)
    =============================== */

    /**
       * @param {{left:Vector,right:Vector}[]} portals
       */
    stringPull(portals) {
        const path = [];

        let apex = portals[0].left;
        let left = portals[0].left;
        let right = portals[0].right;

        let apexIndex = 0;
        let leftIndex = 0;
        let rightIndex = 0;

        path.push(apex);

        for (let i = 1; i < portals.length; i++) {
            const pLeft = portals[i].left;
            const pRight = portals[i].right;

            // 更新右边
            if (area(apex, right, pRight) <= 0) {
                if (apex === right || area(apex, left, pRight) > 0) {
                    right = pRight;
                    rightIndex = i;
                } else {
                    path.push(left);
                    apex = left;
                    apexIndex = leftIndex;
                    left = apex;
                    right = apex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    i = apexIndex;
                    continue;
                }
            }

            // 更新左边
            if (area(apex, left, pLeft) >= 0) {
                if (apex === left || area(apex, right, pLeft) < 0) {
                    left = pLeft;
                    leftIndex = i;
                } else {
                    path.push(right);
                    apex = right;
                    apexIndex = rightIndex;
                    left = apex;
                    right = apex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    i = apexIndex;
                    continue;
                }
            }
        }

        path.push(portals[portals.length - 1].left);
        return path;
    }
}

// 计算最大可能的 span 数量
const totalspan = ((MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1) * ((MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1) * ((MESH_WORLD_SIZE_Z / MESH_TRACE_SIZE_Z) + 1);

// SOA 结构 (Structure of Arrays) 内存布局优化
// 按属性分离存储，提高缓存局部性，减少内存碎片
const floor = new Int16Array(totalspan);           // ±32k，足够存储高度差
const ceiling = new Int16Array(totalspan);         // ±32k
const next = new Uint32Array(totalspan);            // 指向下一个 span，最多 4G 个
const regionId = new Uint16Array(totalspan);       // ±65k 个区域，足够
const distance = new Uint16Array(totalspan);       // 距离场，最多 65k（腐蚀半径通常 <256）
const denoisedistance = new Uint16Array(totalspan);       // 降噪距离场，最多 65k（腐蚀半径通常 <256）
const neighbor = new Uint32Array(totalspan * 4);   // 每个 span 4 邻居（W,N,E,S），0 表示无邻居
const use = new Uint8Array(Math.ceil(totalspan / 8)); // 位图：1 bit = 1 span，1 byte = 8 spans
const DISTANCE_INF = 0xFFFF;

// 内存占用计算：
// Int16Array: 2 bytes * totalspan
// Int16Array: 2 bytes * totalspan
// Uint32Array: 4 bytes * totalspan
// Uint16Array: 2 bytes * totalspan
// Uint16Array: 2 bytes * totalspan
// Uint8Array: 1 byte * (totalspan/8)
// ≈ 13 bytes per span (vs 40+ bytes with object fields)

// ============ 纯函数式 API ============
// 所有操作都直接基于 ID，无需创建对象，内存占用为 0
class OpenSpan{
    /**
     * 初始化一个 span
     * @param {number} id 
     * @param {number} m_floor 
     * @param {number} m_ceiling 
     */
    static initSpan(id, m_floor, m_ceiling) {
        floor[id] = m_floor;
        ceiling[id] = m_ceiling;
        next[id] = 0;
        regionId[id] = 0;
        distance[id] = 0;
        denoisedistance[id]=0;
        const base = id << 2;
        neighbor[base] = 0;
        neighbor[base + 1] = 0;
        neighbor[base + 2] = 0;
        neighbor[base + 3] = 0;
        use[id >> 3] |= (1 << (id & 7));  // 设置 use 位
    }

    /**
     * 获取 floor 值
     * @param {number} id 
     * @returns {number}
     */
    static getFloor(id) {
        return floor[id];
    }

    /**
     * 设置 floor 值
     * @param {number} id 
     * @param {number} value 
     */
    static setFloor(id, value) {
        floor[id] = value;
    }

    /**
     * 获取 ceiling 值
     * @param {number} id 
     * @returns {number}
     */
    static getCeiling(id) {
        return ceiling[id];
    }

    /**
     * 设置 ceiling 值
     * @param {number} id 
     * @param {number} value 
     */
    static setCeiling(id, value) {
        ceiling[id] = value;
    }

    /**
     * 获取下一个 span 的 ID
     * @param {number} id 
     * @returns {number} 0 表示没有下一个
     */
    static getNext(id) {
        return next[id];
    }

    /**
     * 设置下一个 span 的 ID
     * @param {number} id 
     * @param {number} nextId 
     */
    static setNext(id, nextId) {
        next[id] = nextId;
    }

    /**
     * 获取 use 状态
     * @param {number} id 
     * @returns {boolean}
     */
    static getUse(id) {
        return (use[id >> 3] & (1 << (id & 7))) !== 0;
    }

    /**
     * 设置 use 状态
     * @param {number} id 
     * @param {boolean} flag 
     */
    static setUse(id, flag) {
        if (flag) {
            use[id >> 3] |= (1 << (id & 7));
        } else {
            use[id >> 3] &= ~(1 << (id & 7));
        }
    }

    /**
     * 获取 region ID
     * @param {number} id 
     * @returns {number}
     */
    static getRegionId(id) {
        return regionId[id];
    }

    /**
     * 设置 region ID
     * @param {number} id 
     * @param {number} rid 
     */
    static setRegionId(id, rid) {
        regionId[id] = rid;
    }

    /**
     * 获取距离值
     * @param {number} id 
     * @returns {number}
     */
    static getDistance(id) {
        const d = distance[id];
        return d === DISTANCE_INF ? Infinity : d;
    }

    /**
     * 设置距离值
     * @param {number} id 
     * @param {number} dist 
     */
    static setDistance(id, dist) {
        if (!Number.isFinite(dist)) {
            distance[id] = DISTANCE_INF;
            return;
        }

        if (dist <= 0) {
            distance[id] = 0;
            return;
        }

        const clamped = Math.min(DISTANCE_INF - 1, Math.floor(dist));
        distance[id] = clamped;
    }

    /**
     * 获取距离值
     * @param {number} id 
     * @returns {number}
     */
    static getDenoiseDistance(id) {
        const d = denoisedistance[id];
        return d === DISTANCE_INF ? Infinity : d;
    }

    /**
     * 设置距离值
     * @param {number} id 
     * @param {number} dist 
     */
    static setDenoiseDistance(id, dist) {
        if (!Number.isFinite(dist)) {
            denoisedistance[id] = DISTANCE_INF;
            return;
        }

        if (dist <= 0) {
            denoisedistance[id] = 0;
            return;
        }

        const clamped = Math.min(DISTANCE_INF - 1, Math.floor(dist));
        denoisedistance[id] = clamped;
    }
    /**
     * 获取指定方向邻居 spanId
     * @param {number} id
     * @param {number} dir 0:W, 1:N, 2:E, 3:S
     * @returns {number}
     */
    static getNeighbor(id, dir) {
        return neighbor[(id << 2) + dir];
    }

    /**
     * 设置指定方向邻居 spanId
     * @param {number} id
     * @param {number} dir 0:W, 1:N, 2:E, 3:S
     * @param {number} neighborId
     */
    static setNeighbor(id, dir, neighborId) {
        neighbor[(id << 2) + dir] = neighborId;
    }

    /**
     * 清空 [startId, endId] 范围内的 span 数据
     * @param {number} startId
     * @param {number} endId
     */
    static clearRange(startId, endId) {
        const s = Math.max(1, startId | 0);
        const e = Math.max(s, endId | 0);
        for (let id = s; id <= e; id++) {
            floor[id] = 0;
            ceiling[id] = 0;
            next[id] = 0;
            regionId[id] = 0;
            distance[id] = 0;
            denoisedistance[id]=0;
            const base = id << 2;
            neighbor[base] = 0;
            neighbor[base + 1] = 0;
            neighbor[base + 2] = 0;
            neighbor[base + 3] = 0;
            use[id >> 3] &= ~(1 << (id & 7));
        }
    }

    /**
     * 双向通行检查（id1 和 id2 之间能否通行）
     * @param {number} id1 
     * @param {number} id2 
     * @param {number} maxStep 
     * @param {number} agentHeight 
     * @returns {boolean}
     */
    static canTraverseTo(id1, id2, maxStep = MAX_WALK_HEIGHT, agentHeight = AGENT_HEIGHT) {
        // 检查 id2 是否在使用
        if (!this.getUse(id2)) return false;
        
        // 高度差检查
        if (Math.abs(floor[id2] - floor[id1]) > maxStep) {
            return false;
        }

        // 检查两个 span 之间能否通行
        const floorLevel = Math.max(floor[id1], floor[id2]);
        const ceilLevel = Math.min(ceiling[id1], ceiling[id2]);

        if (ceilLevel - floorLevel < agentHeight) {
            return false;
        }

        return true;
    }

    /**
     * 单向通行检查（从 id1 只能往上到 id2）
     * @param {number} id1 
     * @param {number} id2 
     * @param {number} maxStep 
     * @param {number} agentHeight 
     * @returns {boolean}
     */
    static canTo(id1, id2, maxStep = MAX_WALK_HEIGHT, agentHeight = AGENT_HEIGHT) {
        // 检查 id2 是否在使用
        //if (!this.getUse(id2)) return false;
        
        // 只允许上升 maxStep 高度
        if (floor[id2] - floor[id1] > maxStep) {
            return false;
        }

        // 检查高度空间
        const floorLevel = floor[id1];
        const ceilLevel = ceiling[id2];

        if (ceilLevel - floorLevel < agentHeight) {
            return false;
        }

        return true;
    }
}

/**@typedef {import("cs_script/point_script").Vector} Vector */

class OpenHeightfield {
    /**
     * @param {number} tx
     * @param {number} ty
     * @param {number} tileSize
     * @param {number} fullGrid
     * @param {number} tilePadding
     */
    constructor(tx, ty, tileSize, fullGrid, tilePadding) {
        this.SPAN_ID = 1;

        this.tx = tx;
        this.ty = ty;
        this.tileSize = tileSize;
        this.fullGrid = fullGrid;
        this.tilePadding = tilePadding;

        this.coreMinX = tx * tileSize;
        this.coreMinY = ty * tileSize;
        this.coreMaxX = Math.min(fullGrid - 1, this.coreMinX + tileSize - 1);
        this.coreMaxY = Math.min(fullGrid - 1, this.coreMinY + tileSize - 1);

        this.buildMinX = Math.max(0, this.coreMinX - tilePadding);
        this.buildMinY = Math.max(0, this.coreMinY - tilePadding);
        this.buildMaxX = Math.min(fullGrid - 1, this.coreMaxX + tilePadding);
        this.buildMaxY = Math.min(fullGrid - 1, this.coreMaxY + tilePadding);

        this.localCoreMinX = this.coreMinX - this.buildMinX;
        this.localCoreMinY = this.coreMinY - this.buildMinY;
        this.localCoreMaxX = this.coreMaxX - this.buildMinX;
        this.localCoreMaxY = this.coreMaxY - this.buildMinY;

        this.baseX = this.buildMinX;
        this.baseY = this.buildMinY;
        this.gridX = this.buildMaxX - this.buildMinX + 1;
        this.gridY = this.buildMaxY - this.buildMinY + 1;
        this.tileCoreMinX = this.coreMinX;
        this.tileCoreMaxX = this.coreMaxX + 1;
        this.tileCoreMinY = this.coreMinY;
        this.tileCoreMaxY = this.coreMaxY + 1;

        this.cells = new Array(this.gridX);
        for (let i = 0; i < this.gridX; i++) {
            this.cells[i] = new Uint32Array(this.gridY);
        }

        this.mins = { x: -MESH_CELL_SIZE_XY / 2, y: -MESH_CELL_SIZE_XY / 2, z: -MESH_TRACE_SIZE_Z / 2 };
        this.maxs = { x: MESH_CELL_SIZE_XY / 2, y: MESH_CELL_SIZE_XY / 2, z: MESH_TRACE_SIZE_Z / 2 };
    }
    init() {
        const minZ = origin.z;
        const maxZ = origin.z + MESH_WORLD_SIZE_Z;
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                const worldX = origin.x + (this.baseX + x) * MESH_CELL_SIZE_XY;
                const worldY = origin.y + (this.baseY + y) * MESH_CELL_SIZE_XY;
                this.cells[x][y] = this.voxelizeColumn(worldX, worldY, minZ, maxZ);
            }

        }
        this.erode(MESH_ERODE_RADIUS);
        this.markPaddingAsUnwalkable();
    }

    /**
     * @param {number} wx
     * @param {number} wy
     * @param {number} minZ
     * @param {number} maxZ
     */
    voxelizeColumn(wx, wy, minZ, maxZ) {
        let head = 0;  // 0 表示链表为空
        let currentZ = maxZ;
        const radius = MESH_TRACE_SIZE_Z / 2;

        while (currentZ >= minZ + radius) {
            //寻找地板 (floor)
            const downStart = { x: wx, y: wy, z: currentZ };
            const downEnd = { x: wx, y: wy, z: minZ };
            const downTr = Instance.TraceBox({ mins: this.mins, maxs: this.maxs, start: downStart, end: downEnd, ignorePlayers: true });
            if (!downTr || !downTr.didHit) break; // 下面没东西了，结束

            const floorZ = downTr.end.z - radius;

            //从地板向上寻找天花板 (ceiling)
            const upStart = { x: wx, y: wy, z: downTr.end.z + 1 };
            const upEnd = { x: wx, y: wy, z: maxZ };
            const upTr = Instance.TraceBox({ mins: this.mins, maxs: this.maxs, start: upStart, end: upEnd, ignorePlayers: true });

            let ceilingZ = maxZ;
            if (upTr.didHit) ceilingZ = upTr.end.z + radius;

            const floor = Math.round(floorZ - origin.z);
            const ceiling = Math.round(ceilingZ - origin.z);

            const slopeWalkable = this.isSlopeWalkableByNormal(downTr.normal);
            if ((ceiling - floor) >= AGENT_HEIGHT && slopeWalkable) {
                const newId = this.SPAN_ID++;
                OpenSpan.initSpan(newId, floor, ceiling);

                if (head === 0 || floor < OpenSpan.getFloor(head)) {
                    OpenSpan.setNext(newId, head);
                    head = newId;
                } else {
                    let curr = head;
                    while (OpenSpan.getNext(curr) !== 0 && OpenSpan.getFloor(OpenSpan.getNext(curr)) < floor) {
                        curr = OpenSpan.getNext(curr);
                    }
                    OpenSpan.setNext(newId, OpenSpan.getNext(curr));
                    OpenSpan.setNext(curr, newId);
                }
            }

            currentZ = floorZ - radius - 1;
        }

        return head;
    }

    /**
     * 根据命中法线判断坡度是否可行走。
     * @param {Vector} normal
     * @returns {boolean}
     */
    isSlopeWalkableByNormal(normal) {
        if (!normal) return false;

        const len = Math.hypot(normal.x, normal.y, normal.z);
        if (len <= 1e-6) return false;

        const upDot = Math.max(-1, Math.min(1, normal.z / len));
        const slopeDeg = Math.acos(upDot) * 180 / Math.PI;
        return slopeDeg <= MAX_SLOPE;
    }
    /**
     * 根据半径腐蚀可行走区域
     * @param {number} radius
     */
    erode(radius) {
        if (radius <= 0) return;

        // 1. 初始化距离场，默认给一个很大的值
        // 使用 Uint16Array 节省内存，索引为 span id
        const distances = new Uint16Array(this.SPAN_ID + 1).fill(65535);
        const dirs = [{ dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 1, dy: 0 }, { dx: 0, dy: -1 }];

        // 2. 标记边界点（距离为 0）
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let spanId = this.cells[i][j];
                while (spanId !== 0) {
                    if (OpenSpan.getUse(spanId)) {
                        let isBoundary = false;
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;

                            // 触碰地图边界或没有邻居，即为边界
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                                isBoundary = true;
                                break;
                            }

                            let hasNeighborInDir = false;
                            let nspanId = this.cells[nx]?.[ny] || 0;
                            while (nspanId !== 0) {
                                if (OpenSpan.getUse(nspanId)) {
                                    if (OpenSpan.canTraverseTo(spanId, nspanId)) {
                                        hasNeighborInDir = true;
                                        break;
                                    }
                                }
                                nspanId = OpenSpan.getNext(nspanId);
                            }

                            // 任一方向缺失可达邻居，就视为边界
                            if (!hasNeighborInDir) {
                                isBoundary = true;
                                break;
                            }
                        }
                        if (isBoundary) distances[spanId] = 0;
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 3. 两次遍历计算精确距离场 (Pass 1: Top-Left to Bottom-Right)
        this._passDist(distances, true);
        // (Pass 2: Bottom-Right to Top-Left)
        this._passDist(distances, false);

        // 4. 根据 AGENT_RADIUS 删除不合格的 Span
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let spanId = this.cells[i][j];
                while (spanId !== 0) {
                    if (OpenSpan.getUse(spanId)) {
                        // 如果距离边界太近，则剔除
                        if (distances[spanId] < radius) {
                            OpenSpan.setUse(spanId, false);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 内部辅助：距离场传递
     * @param {Uint16Array} distances
     * @param {boolean} forward
     */
    _passDist(distances, forward) {
        const dirs = [{ dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 1, dy: 0 }, { dx: 0, dy: -1 }];
        const startX = forward ? 0 : this.gridX - 1;
        const endX = forward ? this.gridX : -1;
        const step = forward ? 1 : -1;

        for (let i = startX; i !== endX; i += step) {
            for (let j = forward ? 0 : this.gridY - 1; j !== (forward ? this.gridY : -1); j += step) {
                let spanId = this.cells[i][j];
                while (spanId !== 0) {
                    if (OpenSpan.getUse(spanId)) {
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;

                            let nspanId = this.cells[nx]?.[ny] || 0;
                            while (nspanId !== 0) {
                                if (OpenSpan.getUse(nspanId)) {
                                    if (OpenSpan.canTraverseTo(spanId, nspanId)) {
                                        // 核心公式：当前点距离 = min(当前距离, 邻居距离 + 1)
                                        distances[spanId] = Math.min(distances[spanId], distances[nspanId] + 1);
                                    }
                                }
                                nspanId = OpenSpan.getNext(nspanId);
                            }
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 仅让 tile core 参与区域和轮廓生成，padding 只提供体素上下文
     */
    markPaddingAsUnwalkable() {
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                if (i >= this.localCoreMinX && i <= this.localCoreMaxX && j >= this.localCoreMinY && j <= this.localCoreMaxY) continue;

                let spanId = this.cells[i][j];
                while (spanId !== 0) {
                    OpenSpan.setUse(spanId, false);
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    debug(duration = 30) {
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let spanId = this.cells[i][j];
                while (spanId !== 0) {
                    if (OpenSpan.getUse(spanId)) {
                        const c = {
                            r: 255,
                            g: 255,
                            b: 0
                        };
                        Instance.DebugSphere({
                            center: {
                                x: origin.x + (this.baseX + i) * MESH_CELL_SIZE_XY,
                                y: origin.y + (this.baseY + j) * MESH_CELL_SIZE_XY,
                                z: origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z
                            },
                            radius: 3,
                            duration,
                            color: c
                        });
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }
}

class RegionGenerator {
    /**
     * @param {OpenHeightfield} openHeightfield
     */
    constructor(openHeightfield) {
        this.hf = openHeightfield.cells;
        this.baseX = openHeightfield.baseX;
        this.baseY = openHeightfield.baseY;

        this.gridX = openHeightfield.gridX;
        this.gridY = openHeightfield.gridY;

        this.nextRegionId = 1;
    }

    init() {
        this.buildCompactNeighbors();
        this.buildDistanceField();
        this.buildRegionsWatershed();
        this.mergeAndFilterRegions();
    }
    //为span建立邻居关系
    buildCompactNeighbors() {
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        for (let d = 0; d < 4; d++) {
                            const nx = x + dirs[d].dx;
                            const ny = y + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                                OpenSpan.setNeighbor(spanId, d, 0);
                                continue;
                            }

                            let best = 0;
                            let bestDiff = Infinity;
                            let nspanId = this.hf[nx][ny];

                            while (nspanId !== 0) {
                                if(OpenSpan.getUse(nspanId))
                                {
                                    if (OpenSpan.canTraverseTo(spanId, nspanId)) {
                                        const diff = Math.abs(OpenSpan.getFloor(spanId) - OpenSpan.getFloor(nspanId));
                                        if (diff < bestDiff) {
                                            best = nspanId;
                                            bestDiff = diff;
                                        }
                                    }
                                }
                                nspanId = OpenSpan.getNext(nspanId);
                            }

                            OpenSpan.setNeighbor(spanId, d, best);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 获取对角线邻居。
     * 例如：西北 (NW) = 先向西(0)再向北(1)
     * @param {number} spanId 
     * @param {number} dir1 
     * @param {number} dir2 
     * @returns {number} 邻居spanId，0表示无邻居
     */
    getDiagonalNeighbor(spanId, dir1, dir2) {
        const first = OpenSpan.getNeighbor(spanId, dir1);
        if (first !== 0) {
            const diagonal = OpenSpan.getNeighbor(first, dir2);
            if (diagonal !== 0) return diagonal;
        }

        const second = OpenSpan.getNeighbor(spanId, dir2);
        if (second !== 0) {
            return OpenSpan.getNeighbor(second, dir1);
        }

        return 0;
    }
    //构建距离场
    buildDistanceField() {
        // 1. 初始化：边界设为0，内部设为无穷大
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        // 如果任意一个邻居缺失，说明是边界
                        OpenSpan.setDistance(spanId, this.isBorderSpan(spanId) ? 0 : Infinity);
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 第一遍扫描：从左下到右上
        // 西(0)、西南(0+3)、南(3)、东南(3+2)
        for (let y = 0; y < this.gridY; y++) {
            for (let x = 0; x < this.gridX; x++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getDistance(spanId) > 0) {
                            // 西
                            let n = OpenSpan.getNeighbor(spanId, 0);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 西南
                            let nd = this.getDiagonalNeighbor(spanId, 0, 3);
                            if (nd !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd) + 3));
                            // 南
                            n = OpenSpan.getNeighbor(spanId, 3);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 东南
                            nd = this.getDiagonalNeighbor(spanId, 3, 2);
                            if (nd !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd) + 3));
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 第二遍扫描：从右上到左下
        // 东(2)、东北(2+1)、北(1)、西北(1+0)
        for (let y = this.gridY - 1; y >= 0; y--) {
            for (let x = this.gridX - 1; x >= 0; x--) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getDistance(spanId) > 0) {
                            // 东
                            let n = OpenSpan.getNeighbor(spanId, 2);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 东北
                            let nd = this.getDiagonalNeighbor(spanId, 2, 1);
                            if (nd !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd) + 3));
                            // 北
                            n = OpenSpan.getNeighbor(spanId, 1);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 西北
                            let nd2 = this.getDiagonalNeighbor(spanId, 1, 0);
                            if (nd2 !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd2) + 3));
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
        // 第二遍扫描后，distance 场已经稳定了，可以用来做降噪了
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        let all=OpenSpan.getDistance(spanId);
                        let n = OpenSpan.getNeighbor(spanId, 0);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = OpenSpan.getNeighbor(spanId, 1);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = OpenSpan.getNeighbor(spanId, 2);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = OpenSpan.getNeighbor(spanId, 3);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);

                        n = this.getDiagonalNeighbor(spanId, 0,3);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = this.getDiagonalNeighbor(spanId, 0,1);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);

                        n = this.getDiagonalNeighbor(spanId, 2,3);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = this.getDiagonalNeighbor(spanId, 2,1);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);

                        // 如果任意一个邻居缺失，说明是边界
                        OpenSpan.setDenoiseDistance(spanId, all/9);
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 是否是边界span
     * @param {number} spanId
     */
    isBorderSpan(spanId) {
        for (let d = 0; d < 4; d++) {
            if (OpenSpan.getNeighbor(spanId, d) === 0) return true;
        }
        return false;
    }

    //洪水扩张
    buildRegionsWatershed() {
        // 1) 按 denoiseDistance 收集所有可用 span，并重置 regionId
        //    distBuckets: 下标=距离值，value=该距离上的 span 列表
        /** @type {number[][]} */
        const distBuckets = [];
        let maxDist = 0;

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        OpenSpan.setRegionId(spanId, 0);
                        const dist = OpenSpan.getDenoiseDistance(spanId);
                        if (Number.isFinite(dist) && dist >= 0) {
                            const d = Math.floor(dist);
                            if (!distBuckets[d]) distBuckets[d] = [];
                            distBuckets[d].push(spanId);
                            if (d > maxDist) maxDist = d;
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 2) 生成“每隔2个距离一个批次”的批次列表（从大到小）
        //    这里的阈值计算会自然形成：当 maxDist 为偶数时，首批包含 d-2/d-1/d
        /** @type {number[][]} */
        const batches = [];
        let coveredMin = maxDist + 1;
        let level = (maxDist + 1) & -2;

        while (coveredMin > 0) {
            const threshold = Math.max(level - 2, 0);
            const batch = [];

            for (let dist = coveredMin - 1; dist >= threshold; dist--) {
                const list = distBuckets[dist];
                if (list && list.length > 0) batch.push(...list);
            }

            if (batch.length > 0) batches.push(batch);

            coveredMin = threshold;
            level = Math.max(level - 2, 0);
        }

        // 3) 逐批处理（从高距离到低距离）
        for (const batch of batches) {
            // batchSet 用于 O(1) 判断邻居是否仍在当前批次内
            const batchSet = new Set(batch);

            // queue 是“旧水位”的广度扩张队列（BFS）
            // 只装入已经被赋予 region 的节点，向同批次未赋值节点扩散
            const queue = [];

            // 3.1 先尝试让本批次节点接入已有 region（来自历史批次或已处理节点）
            for (const spanId of batch) {
                if (OpenSpan.getRegionId(spanId) !== 0) {
                    queue.push(spanId);
                    continue;
                }

                let bestRegion = 0;
                let maxNeighborDist = -1;

                // 从4邻域中挑一个“最靠内”（距离更大）的已有 region 作为接入目标
                for (let d = 0; d < 4; d++) {
                    const n = OpenSpan.getNeighbor(spanId, d);
                    if (n === 0) continue;

                    const neighborRegion = OpenSpan.getRegionId(n);
                    if (neighborRegion === 0) continue;

                    const neighborDist = OpenSpan.getDenoiseDistance(n);
                    if (neighborDist > maxNeighborDist) {
                        maxNeighborDist = neighborDist;
                        bestRegion = neighborRegion;
                    }
                }

                if (bestRegion !== 0) {
                    OpenSpan.setRegionId(spanId, bestRegion);
                    queue.push(spanId);
                }
            }

            // 3.2 旧水位 BFS：在当前批次内，把已接入的 region 尽量向外扩散
            for (let q = 0; q < queue.length; q++) {
                const current = queue[q];
                const rid = OpenSpan.getRegionId(current);

                for (let d = 0; d < 4; d++) {
                    const n = OpenSpan.getNeighbor(current, d);
                    if (n === 0) continue;
                    if (!batchSet.has(n)) continue;
                    if (OpenSpan.getRegionId(n) !== 0) continue;

                    OpenSpan.setRegionId(n, rid);
                    queue.push(n);
                }
            }

            // 3.3 对仍未覆盖的节点创建新水位（新 region），并立即 DFS 泛洪
            for (const spanId of batch) {
                if (OpenSpan.getRegionId(spanId) !== 0) continue;

                const rid = this.nextRegionId++;
                OpenSpan.setRegionId(spanId, rid);

                // stack 是“新水位”深度扩张栈（DFS）
                const stack = [spanId];
                while (stack.length > 0) {
                    const current = stack.pop();
                    if (current === undefined) break;

                    for (let d = 0; d < 4; d++) {
                        const n = OpenSpan.getNeighbor(current, d);
                        if (n === 0) continue;
                        if (!batchSet.has(n)) continue;
                        if (OpenSpan.getRegionId(n) !== 0) continue;

                        OpenSpan.setRegionId(n, rid);
                        stack.push(n);
                    }
                }
            }
        }
    }
    //合并过滤小region
    mergeAndFilterRegions() {
        /**@type {Map<number,number[]>} */
        const regionSpans = new Map();

        //统计每个region包含的span
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            if (!regionSpans.has(OpenSpan.getRegionId(spanId))) regionSpans.set(OpenSpan.getRegionId(spanId), []);
                            regionSpans.get(OpenSpan.getRegionId(spanId))?.push(spanId);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
        //合并过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MERGE_AREA) continue;
            const neighbors = new Map();
            for (const spanId of spans) {
                for (let d = 0; d < 4; d++) {
                    const n = OpenSpan.getNeighbor(spanId, d);
                    if (n !== 0 && OpenSpan.getRegionId(n) !== id) {
                        neighbors.set(
                            OpenSpan.getRegionId(n),
                            (neighbors.get(OpenSpan.getRegionId(n)) ?? 0) + 1
                        );
                    }
                }
            }

            let best = 0;
            let bestCount = 0;
            for (const [nid, count] of neighbors) {
                if (count > bestCount) {
                    best = nid;
                    bestCount = count;
                }
            }

            if (best > 0) {
                for (const spanId of spans) {
                    OpenSpan.setRegionId(spanId, best);
                    regionSpans.get(OpenSpan.getRegionId(spanId))?.push(spanId);
                }
                regionSpans.set(id, []);
            }
        }
        //统计每个region包含的span
        regionSpans.clear();
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            if (!regionSpans.has(OpenSpan.getRegionId(spanId))) regionSpans.set(OpenSpan.getRegionId(spanId), []);
                            regionSpans.get(OpenSpan.getRegionId(spanId))?.push(spanId);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
        //忽略过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MIN_AREA) continue;
            for (const spanId of spans) {
                if (OpenSpan.getRegionId(spanId) == id) OpenSpan.setRegionId(spanId, 0);
            }
        }
    }
    /**
     * Debug: 绘制 Region（按 regionId 上色）
     * @param {number} duration
     */
    debugDrawRegions(duration = 5) {
        const colorCache = new Map();

        const randomColor = (/** @type {number} */ id) => {
            if (!colorCache.has(id)) {
                colorCache.set(id, {
                    r: (id * 97) % 255,
                    g: (id * 57) % 255,
                    b: (id * 17) % 255
                });
            }
            return colorCache.get(id);
        };

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            const c = randomColor(OpenSpan.getRegionId(spanId));

                            const center = {
                                x: origin.x + (this.baseX + x + 0.5) * MESH_CELL_SIZE_XY,
                                y: origin.y + (this.baseY + y + 0.5) * MESH_CELL_SIZE_XY,
                                z: origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z
                            };

                            Instance.DebugSphere({
                                center,
                                radius: 3,
                                color: c,
                                duration
                            });
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }
    /**
     * Debug: 绘制 Distance Field（亮度 = 距离）
     */
    debugDrawDistance(duration = 5) {
        let maxDist = 0;

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        maxDist = Math.max(maxDist, OpenSpan.getDistance(spanId));
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getDistance(spanId) < Infinity) {
                            const t = OpenSpan.getDistance(spanId) / maxDist;
                            const c = {
                                r: Math.floor(255 * t),
                                g: Math.floor(255 * (1 - t)),
                                b: 0
                            };

                            Instance.DebugSphere({
                                center: {
                                    x: origin.x + (this.baseX + x) * MESH_CELL_SIZE_XY,
                                    y: origin.y + (this.baseY + y) * MESH_CELL_SIZE_XY,
                                    z: origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z
                                },
                                radius: 3,
                                color: c,
                                duration
                            });
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

}

class ContourBuilder {
    /**
     * @param {OpenHeightfield} hf
     */
    constructor(hf) {
        /** @type {boolean} */
        this.error = false;
        /** @type {number[][]} */
        this.hf = hf.cells;
        this.gridX = hf.gridX;
        this.gridY = hf.gridY;
        this.baseX = hf.baseX;
        this.baseY = hf.baseY;
        this.tileCoreMinX = hf.tileCoreMinX;
        this.tileCoreMaxX = hf.tileCoreMaxX;
        this.tileCoreMinY = hf.tileCoreMinY;
        this.tileCoreMaxY = hf.tileCoreMaxY;

        /** @type {Contour[][]} */
        this.contours = [];
    }

    buildCompactNeighbors() {
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if (OpenSpan.getUse(spanId)) {
                        for (let d = 0; d < 4; d++) {
                            const nx = x + dirs[d].dx;
                            const ny = y + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                                OpenSpan.setNeighbor(spanId, d, 0);
                                continue;
                            }

                            let best = 0;
                            let bestDiff = Infinity;
                            let nspanId = this.hf[nx][ny];
                            while (nspanId !== 0) {
                                if (OpenSpan.getUse(nspanId) && OpenSpan.canTraverseTo(spanId, nspanId)) {
                                    const diff = Math.abs(OpenSpan.getFloor(spanId) - OpenSpan.getFloor(nspanId));
                                    if (diff < bestDiff) {
                                        best = nspanId;
                                        bestDiff = diff;
                                    }
                                }
                                nspanId = OpenSpan.getNext(nspanId);
                            }
                            OpenSpan.setNeighbor(spanId, d, best);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 边界边：没有邻居，或邻居 region 不同
     * @param {number} spanId
     * @param {number} dir
     */
    isBoundaryEdge(spanId, dir) {
        const n = OpenSpan.getNeighbor(spanId, dir);
        if (n === 0) return true;
        return OpenSpan.getRegionId(n) !== OpenSpan.getRegionId(spanId);
    }
    /**
     * @param {number} spanId
     * @param {number} dir
     */
    getNeighborregionid(spanId, dir) {
        const n = OpenSpan.getNeighbor(spanId, dir);
        if (n !== 0) return OpenSpan.getRegionId(n);
        else return 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} spanId
     * @param {number} dir
     */
    edgeKey(x, y, spanId, dir) {
        return `${x},${y},${spanId},${dir}`;
    }

    /**
     * @param {number} x
     * @param {number} y
     * @param {number} dir
     */
    move(x, y, dir) {
        switch (dir) {
            case 0: return { x: x - 1, y };
            case 1: return { x, y: y + 1 };
            case 2: return { x: x + 1, y };
            case 3: return { x, y: y - 1 };
        }
        return { x, y };
    }

    /**
     * dir对应的cell角点
     * @param {number} x
     * @param {number} y
     * @param {number} dir
     */
    corner(x, y, dir) {
        switch (dir) {
            case 0: return { x, y };
            case 1: return { x, y: y + 1 };
            case 2: return { x: x + 1, y: y + 1 };
            case 3: return { x: x + 1, y };
        }
        return { x, y };
    }

    init() {
        /** @type {Set<string>} */
        const visited = new Set();
        this.buildCompactNeighbors();

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            for (let dir = 0; dir < 4; dir++) {
                                if (this.isBoundaryEdge(spanId, dir)) {

                                    const key = this.edgeKey(x, y, spanId, dir);
                                    if (visited.has(key)) continue;

                                    let contour = this.traceContour(x, y, spanId, dir, visited);
                                    if (contour && contour.length >= 3) {
                                        //外轮廓：逆时针（CCW）
                                        //洞轮廓：顺时针（CW）
                                        contour = this.simplifyContour(contour);
                                        if (!contour || contour.length < 2) continue;

                                        if (!this.isDegenerateContour(contour) && contour.length >= 3) {
                                            this.contours.push(contour);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }
    /**
     * @param {Contour[]} contour
     */
    simplifyContour(contour) {
        const n = contour.length;
        if (n < 4) return contour.slice();
        const pts = contour.slice();

        const locked = new Array(n).fill(0);
        let lockCount = 0;
        for (let i = 0; i < n; i++) {
            const cur = pts[i];
            const next = pts[(i + 1) % n];
            const prev = pts[(i - 1 + n) % n];
            const isPortalChange = next.neighborRegionId !== cur.neighborRegionId;
            const keepBorderPoint = this.isPointOnTileBorder(cur) && !this.isBorderCollinearPoint(prev, cur, next);

            if (isPortalChange || keepBorderPoint) {
                locked[i] = 1;
                //Instance.DebugSphere({center: vec.Zfly(this.contourPointToWorld(cur),20*Math.random()), radius: 2, color:{r: 255, g: next.neighborRegionId!=0?255:0, b: 0},duration: 30});
                lockCount++;
            }
        }

        if (lockCount === 0) {
            let minId = 0;
            let maxId = 0;
            for (let i = 1; i < n; i++) {
                const p = pts[i];
                if (p.x < pts[minId].x || (p.x === pts[minId].x && p.y < pts[minId].y)) minId = i;
                if (p.x > pts[maxId].x || (p.x === pts[maxId].x && p.y > pts[maxId].y)) maxId = i;
            }
            locked[minId] = 1;
            locked[maxId] = 1;
        }

        /** @type {Contour[]} */
        const out = [];

        let i = 0;
        let firstLocked = -1;
        let lastLocked = -1;
        while (i < n - 1) {
            if (locked[i] === 0) {
                i++;
                continue;
            }

            if (firstLocked === -1) firstLocked = i;
            let j = i + 1;
            while (j < n - 1 && locked[j] === 0) j++;
            if (locked[j]) lastLocked = j;

            if (locked[i] && locked[j]) {
                // 锁点就是切换点：只看锁点后的第一条边类型
                const portalRegionId = pts[(i + 1) % n]?.neighborRegionId ?? 0;
                if (portalRegionId > 0) {
                    out.push(pts[i]);
                } else {
                    this.simplifySegmentByMaxError(pts, i, j, out);
                }
            }
            i = j;
        }

        // wrap 段同样只看锁点后的第一条边类型
        const wrapPortalRegionId = pts[(lastLocked + 1) % n]?.neighborRegionId ?? 0;
        if (wrapPortalRegionId > 0) {
            out.push(pts[lastLocked]);
        } else {
            this.simplifySegmentByMaxErrorWrap(pts, lastLocked, firstLocked, out);
        }

        if (out.length >= 3) {
            const indexByPoint = new Map();
            for (let k = 0; k < n; k++) {
                indexByPoint.set(pts[k], k);
            }

            /** @type {number[]} */
            const outIndices = [];
            for (const p of out) {
                const idx = indexByPoint.get(p);
                if (idx !== undefined) outIndices.push(idx);
            }
            this.splitLongEdges(pts, outIndices);
            return outIndices.map((idx) => pts[idx]);
        }

        return out;
    }
    /**
     * @param {Contour[]} pts
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     */
    simplifySegmentByMaxError(pts, i0, i1, out) {
        const a = pts[i0];
        const b = pts[i1];
        let maxDistSq = 0;
        let index = -1;

        for (let i = i0 + 1; i < i1; i++) {
            const d = distPtSegSq(pts[i], a, b);
            if (d > maxDistSq) {
                maxDistSq = d;
                index = i;
            }
        }

        const maxErrorSq = this.getContourMaxErrorSq();
        if (index !== -1 && maxDistSq > maxErrorSq) {
            this.simplifySegmentByMaxError(pts, i0, index, out);
            this.simplifySegmentByMaxError(pts, index, i1, out);
        } else {
            out.push(a);
        }
    }

    /**
     * @param {Contour[]} pts
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     */
    simplifySegmentByMaxErrorWrap(pts, i0, i1, out) {
        if (i0 < 0 || i1 < 0) return;

        const n = pts.length;
        const a = pts[i0];
        const b = pts[i1];
        let maxDistSq = 0;
        let index = -1;

        for (let i = i0 + 1; i < n; i++) {
            const d = distPtSegSq(pts[i], a, b);
            if (d > maxDistSq) {
                maxDistSq = d;
                index = i;
            }
        }
        for (let i = 0; i < i1; i++) {
            const d = distPtSegSq(pts[i], a, b);
            if (d > maxDistSq) {
                maxDistSq = d;
                index = i;
            }
        }

        const maxErrorSq = this.getContourMaxErrorSq();
        if (index !== -1 && maxDistSq > maxErrorSq) {
            if (index < i0) this.simplifySegmentByMaxErrorWrap(pts, i0, index, out);
            else this.simplifySegmentByMaxError(pts, i0, index, out);

            if (index < i1) this.simplifySegmentByMaxError(pts, index, i1, out);
            else this.simplifySegmentByMaxErrorWrap(pts, index, i1, out);
        } else {
            out.push(a);
        }
    }

    /**
     * 线段是否位于当前 tile 的边界上。
     * @param {Contour} a
     * @param {Contour} b
     */
    isSegmentOnTileBorder(a, b) {
        if (this.isPointOnTileBorder(a) || this.isPointOnTileBorder(b)) return true;

        const minX = this.tileCoreMinX;
        const maxX = this.tileCoreMaxX;
        const minY = this.tileCoreMinY;
        const maxY = this.tileCoreMaxY;

        if (a.x === minX && b.x === minX) return true;
        if (a.x === maxX && b.x === maxX) return true;
        if (a.y === minY && b.y === minY) return true;
        if (a.y === maxY && b.y === maxY) return true;

        return false;
    }

    /**
     * 点是否落在当前 tile 的外边界上。
     * @param {Contour} p
     */
    isPointOnTileBorder(p) {
        const minX = this.tileCoreMinX;
        const maxX = this.tileCoreMaxX;
        const minY = this.tileCoreMinY;
        const maxY = this.tileCoreMaxY;

        if (p.x === minX || p.x === maxX) return true;
        if (p.y === minY || p.y === maxY) return true;

        return false;
    }

    /**
     * tile 边界上的“纯共线中间点”判定。
     * 仅当 prev-cur-next 同在同一条 tile 外边界线上时返回 true。
     * @param {Contour} prev
     * @param {Contour} cur
     * @param {Contour} next
     */
    isBorderCollinearPoint(prev, cur, next) {
        const minX = this.tileCoreMinX;
        const maxX = this.tileCoreMaxX;
        const minY = this.tileCoreMinY;
        const maxY = this.tileCoreMaxY;

        if (prev.x === minX && cur.x === minX && next.x === minX) return true;
        if (prev.x === maxX && cur.x === maxX && next.x === maxX) return true;
        if (prev.y === minY && cur.y === minY && next.y === minY) return true;
        if (prev.y === maxY && cur.y === maxY && next.y === maxY) return true;

        return false;
    }

    /**
     * @param {Contour[]} pts
     * @param {number[]} simplified
     */
    splitLongEdges(pts, simplified) {
        const maxEdgeLen = this.getContourMaxEdgeLen();
        if (maxEdgeLen <= 0) return;

        let guard = 0;
        while (guard++ < pts.length * 8) {
            let inserted = false;
            for (let i = 0; i < simplified.length; i++) {
                const i0 = simplified[i];
                const i1 = simplified[(i + 1) % simplified.length];
                const dx = Math.abs(pts[i1].x - pts[i0].x);
                const dy = Math.abs(pts[i1].y - pts[i0].y);
                if (Math.max(dx, dy) <= maxEdgeLen) continue;

                const mid = this.pickMidIndexOnArc(pts.length, i0, i1);
                if (mid === -1) continue;
                simplified.splice(i + 1, 0, mid);
                inserted = true;
                break;
            }
            if (!inserted) break;
        }
    }

    /**
     * @param {number} n
     * @param {number} i0
     * @param {number} i1
     */
    pickMidIndexOnArc(n, i0, i1) {
        let steps = 0;
        let i = (i0 + 1) % n;
        while (i !== i1) {
            steps++;
            i = (i + 1) % n;
        }
        if (steps <= 1) return -1;
        const half = Math.floor(steps / 2);
        i = (i0 + 1) % n;
        for (let s = 0; s < half; s++) {
            i = (i + 1) % n;
        }
        return i;
    }

    /**
     * @param {Contour[]} contour
     */
    countUniqueXY(contour) {
        const set = new Set();
        for (const p of contour) set.add(`${p.x}|${p.y}`);
        return set.size;
    }

    /**
     * @param {Contour[]} contour
     */
    isDegenerateContour(contour) {
        if (!contour || contour.length < 3) return true;
        if (this.countUniqueXY(contour) < 3) return true;
        return Math.abs(this.computeSignedArea2D(contour)) <= 1e-6;
    }

    /**
     * @param {Contour[]} contour
     */
    computeSignedArea2D(contour) {
        let area = 0;
        const n = contour.length;
        for (let i = 0; i < n; i++) {
            const a = contour[i];
            const b = contour[(i + 1) % n];
            area += a.x * b.y - b.x * a.y;
        }
        return area * 0.5;
    }

    /**
     * @param {number} sx 起始 cell x
     * @param {number} sy 起始 cell y
     * @param {number} startSpanId
     * @param {number} startDir 起始边方向
     * @returns {Contour[] | null}
     * @param {Set<string>} visited
     */
    traceContour(sx, sy, startSpanId, startDir, visited) {
        let x = sx;
        let y = sy;
        let spanId = startSpanId;
        let dir = startDir;

        const verts = [];

        let iter = 0;
        const MAX_ITER = this.gridX * this.gridY * 4;
        if (!this.isBoundaryEdge(startSpanId, startDir)) return null;
        const startKey = this.edgeKey(x, y, spanId, dir);
        while (iter++ < MAX_ITER) {
            const key = this.edgeKey(x, y, spanId, dir);
            //回到起点
            if (key === startKey && verts.length > 0) break;

            if (visited.has(key)) {
                Instance.Msg("奇怪的轮廓边,找了一遍现在又找一遍");
                this.error=true;
                return null;
            }
            visited.add(key);

            // 只有在边界边才输出顶点
            if (this.isBoundaryEdge(spanId, dir)) {
                const c = this.corner(x, y, dir);

                const h = this.getCornerHeightFromEdge(x, y, spanId, dir);
                const nid = this.getNeighborregionid(spanId, dir);
                //Instance.Msg(nid);
                if (h !== null) {
                    verts.push({
                        x: this.baseX + c.x,
                        y: this.baseY + c.y,
                        z: h,
                        regionId: OpenSpan.getRegionId(spanId),      //当前span的region
                        neighborRegionId: nid   //对面span的region（或 0）
                    });
                }

            }

            // 顺序：右转 → 直行 → 左转 → 后转
            let advanced = false;
            for (let i = 0; i < 4; i++) {
                const ndir = (dir + 3 - i + 4) % 4;
                const nspanId = OpenSpan.getNeighbor(spanId, ndir);

                // 这条边是boundary，就沿边走
                if (nspanId === 0 || OpenSpan.getRegionId(nspanId) !== OpenSpan.getRegionId(spanId)) {
                    dir = ndir;
                    advanced = true;
                    break;
                }

                // 否则穿过这条边
                const p = this.move(x, y, ndir);
                x = p.x;
                y = p.y;
                spanId = nspanId;
                dir = (ndir + 2) % 4;
                advanced = true;
                break;
            }

            if (!advanced) {
                Instance.Msg("轮廓断啦");
                this.error=true;
                return null;
            }
        }
        if (verts.length < 3) {
            this.error=true;
            return null;
        }
        return verts;
    }

    /**
     * @param {number} x
     * @param {number} y
     * @param {number} spanId
     * @param {number} dir
     */
    getCornerHeightFromEdge(x, y, spanId, dir) {
        let maxFloor = OpenSpan.getFloor(spanId);
        const leftDir = (dir + 3) & 3;
        // 只使用 buildCompactNeighbors 建好的 walkable 邻接，
        // 避免在相邻 cell 的整列 span 中误取到“非当前可走链路”的高度层。
        const left = OpenSpan.getNeighbor(spanId, leftDir);
        if (left !== 0) {
            const h = OpenSpan.getFloor(left);
            if (h > maxFloor) maxFloor = h;
        }

        const front = OpenSpan.getNeighbor(spanId, dir);
        if (front !== 0) {
            const h = OpenSpan.getFloor(front);
            if (h > maxFloor) maxFloor = h;
        }

        // 对角采用“先左再前”与“先前再左”两条可走链路择优。
        let diag = 0;
        if (left !== 0) diag = OpenSpan.getNeighbor(left, dir);
        if (diag === 0 && front !== 0) diag = OpenSpan.getNeighbor(front, leftDir);
        if (diag !== 0) {
            const h = OpenSpan.getFloor(diag);
            if (h > maxFloor) maxFloor = h;
        }

        return maxFloor;
    }
    /**
     * @param {number} x
     * @param {number} y
     */
    inBounds(x, y) {
        return x >= 0 && y >= 0 && x < this.gridX && y < this.gridY;
    }

    getContourMaxErrorSq() {
        const e = CONT_MAX_ERROR;
        return e * e;
    }

    getContourMaxEdgeLen() {
        return 0;
    }

    /**
     * @param {Contour} v
     */
    contourPointToWorld(v) {
        return {
            x: origin.x + v.x * MESH_CELL_SIZE_XY ,//- MESH_CELL_SIZE_XY / 2,
            y: origin.y + v.y * MESH_CELL_SIZE_XY ,//- MESH_CELL_SIZE_XY / 2,
            z: origin.z + v.z * MESH_CELL_SIZE_Z,
        };
    }

    debugDrawContours(duration = 5) {
        Instance.Msg(`一共${this.contours.length}个轮廓`);
        for (const contour of this.contours) {
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            const z = Math.random() * 20;
            for (let i = 0; i < contour.length; i++) {
                const a = this.contourPointToWorld(contour[i]);
                const b = this.contourPointToWorld(contour[(i + 1) % contour.length]);
                const start = {
                    x: a.x,
                    y: a.y,
                    z: a.z + z
                };
                const end = {
                    x: b.x,
                    y: b.y,
                    z: b.z + z
                };
                Instance.DebugLine({
                    start,
                    end,
                    color,
                    duration
                });
            }
        }
    }
}
/**
 * @typedef {Object} Contour
 * @property {number} x
 * @property {number} y
 * @property {number} z
 * x,y 为离散格点坐标；z 为离散高度层
 * @property {number} regionId
 * @property {number} neighborRegionId
 */

/** @typedef {import("cs_script/point_script").Vector} Vector */

/**
 * vec: 轻量向量工具类（无状态静态方法）。
 *
 * 约定：
 * - 不修改传入参数，所有方法返回新对象或标量
 * - `2D` 后缀表示仅计算 XY 分量（通常保留原 z 或返回 z=0）
 */
class vec {
    /**
     * 三维向量加法。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static add(a, b) {
        return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
    }

    /**
     * 二维向量加法（仅累加 XY，z 保留 a.z）。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static add2D(a, b) {
        return { x: a.x + b.x, y: a.y + b.y, z: a.z };
    }

    /**
     * 三维向量减法。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static sub(a, b) {
        return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
    }

    /**
     * 三维向量按标量缩放。
     *
     * @param {Vector} a
     * @param {number} s
     * @returns {Vector}
     */
    static scale(a, s) {
        return { x: a.x * s, y: a.y * s, z: a.z * s };
    }

    /**
     * 二维向量按标量缩放（仅缩放 XY，z 保留 a.z）。
     *
     * @param {Vector} a
     * @param {number} s
     * @returns {Vector}
     */
    static scale2D(a, s) {
        return {
            x: a.x * s,
            y: a.y * s,
            z: a.z
        };
    }

    /**
     * 构造一个向量对象。
     *
     * @param {number} [x]
     * @param {number} [y]
     * @param {number} [z]
     * @returns {Vector}
     */
    static get(x = 0, y = 0, z = 0) {
        return { x, y, z };
    }

    /**
     * 克隆向量。
     *
     * @param {Vector} a
     * @returns {Vector}
     */
    static clone(a) {
        return { x: a.x, y: a.y, z: a.z };
    }

    /**
     * 计算三维欧氏距离。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static length(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dz = a.z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * 计算二维欧氏距离（仅 XY）。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static length2D(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * 返回点在 Z 轴上偏移后的新坐标。
     *
     * @param {Vector} pos
     * @param {number} height
     * @returns {Vector}
     */
    static Zfly(pos, height) {
        return { x: pos.x, y: pos.y, z: pos.z + height };
    }

    /**
     * 输出向量坐标到游戏消息。
     *
     * @param {Vector} pos
     */
    static msg(pos) {
        Instance.Msg(`{${pos.x} ${pos.y} ${pos.z}}`);
    }

    /**
     * 三维点积。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {number}
     */
    static dot(a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * 二维点积（仅 XY）。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {number}
     */
    static dot2D(a, b) {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * 三维叉积。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static cross(a, b) {
        return {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x
        };
    }

    /**
     * 三维单位化。
     * 当长度过小（<1e-6）时返回零向量，避免除零。
     *
     * @param {Vector} a
     * @returns {Vector}
     */
    static normalize(a) {
        const len = this.length(a);
        if (len < 1e-6) {
            return { x: 0, y: 0, z: 0 };
        }
        return this.scale(a, 1 / len);
    }

    /**
     * 二维单位化（仅 XY，返回 z=0）。
     * 当长度过小（<1e-6）时返回零向量。
     *
     * @param {Vector} a
     * @returns {Vector}
     */
    static normalize2D(a) {
        const len = this.length2D(a);
        if (len < 1e-6) {
            return { x: 0, y: 0, z: 0 };
        }
        return {
            x: a.x / len,
            y: a.y / len,
            z: 0
        };
    }

    /**
     * 判断是否为近似零向量。
     *
     * @param {Vector} a
     * @returns {boolean}
     */
    static isZero(a) {
        return (
            Math.abs(a.x) < 1e-6 &&
            Math.abs(a.y) < 1e-6 &&
            Math.abs(a.z) < 1e-6
        );
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_contourbuilder").Contour} Contour */
class PolyMeshBuilder {
    /**
     * @param {Contour[][]} contours
     */
    constructor(contours) {
        /** @type {boolean} */
        this.error = false;
        /** @type {Contour[][]} */
        this.contours = contours;

        /** @type {Vector[]} */
        this.verts = [];
        /** @type {Map<string, number>} */
        this.vertIndexMap = new Map();
        /** @type {number[][]} */
        this.polys = [];
        /** @type {number[]} */
        this.regions = [];
        /** @type {number[][][]} */
        this.neighbors = [];
        this.worldConverted = false;
    }

    init() {
        this.error = false;
        /** @type {{x:number,y:number,z:number,regionId:number}[][]} */
        const allPolys = [];

        const grouped = this.groupContoursByRegion(this.contours);
        for (const regionContours of grouped.values()) {
            const simpleContours = this.buildSimpleRegionContours(regionContours);
            for (const contour of simpleContours) {
                const tris = this.triangulate(contour);
                if (tris.length === 0) continue;

                const merged = this.mergeTriangles(tris, POLY_MERGE_LONGEST_EDGE_FIRST);
                for (const poly of merged) allPolys.push(poly);
            }
        }

        for (const p of allPolys) this.addPolygon(p);
        this.buildAdjacency();
        this.convertVertsToWorldAfterAdjacency();
    }

    return() {
        return {
            verts: this.verts,
            polys: this.polys,
            regions: this.regions,
            neighbors: this.neighbors
        };
    }

    /**
     * @param {Contour[][]} contours
     */
    groupContoursByRegion(contours) {
        /** @type {Map<number, Contour[][]>} */
        const byRegion = new Map();
        for (const contour of contours) {
            if (!contour || contour.length < 3 || this.isDegenerateContour(contour)) continue;
            const rid = contour[0].regionId;
            if (!byRegion.has(rid)) byRegion.set(rid, []);
            byRegion.get(rid)?.push(contour);
        }
        return byRegion;
    }

    /**
     * Recast风格：每个 region 内先识别 outer/holes，再将 holes 桥接到 outer，得到简单多边形。
     * @param {Contour[][]} regionContours
     */
    buildSimpleRegionContours(regionContours) {
        /** @type {Contour[][]} */
        const candidates = [];
        for (const contour of regionContours) {
            if (this.isDegenerateContour(contour)) continue;
            const sanitized = this.sanitizeContour(contour);
            if (sanitized.length >= 3 && !this.isDegenerateContour(sanitized)) {
                candidates.push(sanitized);
            }
        }
        if (candidates.length === 0) return [];

        candidates.sort((a, b) => Math.abs(this.computeSignedArea(b)) - Math.abs(this.computeSignedArea(a)));

        /** @type {Contour[][]} */
        const outers = [];
        /** @type {Contour[][][]} */
        const holeGroups = [];

        for (let i = 0; i < candidates.length; i++) {
            const contour = candidates[i].slice();
            const point = contour[0];

            let depth = 0;
            for (let j = 0; j < i; j++) {
                if (this.pointInPolygon2D(point, candidates[j])) depth++;
            }

            const isHole = (depth & 1) === 1;
            if (!isHole) {
                this.ensureWinding(contour, true);
                outers.push(contour);
                holeGroups.push([]);
                continue;
            }

            let bestOuter = -1;
            let bestArea = Infinity;
            for (let k = 0; k < outers.length; k++) {
                if (!this.pointInPolygon2D(point, outers[k])) continue;
                const a = Math.abs(this.computeSignedArea(outers[k]));
                if (a < bestArea) {
                    bestArea = a;
                    bestOuter = k;
                }
            }

            if (bestOuter >= 0) {
                this.ensureWinding(contour, false);
                holeGroups[bestOuter].push(contour);
            }
        }

        /** @type {Contour[][]} */
        const result = [];
        for (let i = 0; i < outers.length; i++) {
            let merged = outers[i].slice();
            const holes = holeGroups[i].slice();
            holes.sort((a, b) => this.getLeftMostPoint(a).x - this.getLeftMostPoint(b).x);

            for (let h = 0; h < holes.length; h++) {
                merged = this.mergeHoleIntoOuter(merged, holes, h);
                merged = this.sanitizeContour(merged);
                if (merged.length < 3) break;
            }

            if (merged.length >= 3 && !this.isDegenerateContour(merged)) {
                this.ensureWinding(merged, true);
                result.push(merged);
            }
        }

        return result;
    }

    /**
     * @param {Contour[]} contour
     */
    sanitizeContour(contour) {
        /** @type {Contour[]} */
        const out = [];
        for (let i = 0; i < contour.length; i++) {
            const cur = contour[i];
            const prev = out[out.length - 1];
            if (prev && prev.x === cur.x && prev.y === cur.y) continue;
            out.push(cur);
        }

        if (out.length >= 2) {
            const a = out[0];
            const b = out[out.length - 1];
            if (a.x === b.x && a.y === b.y) out.pop();
        }

        let i = 0;
        while (out.length >= 3 && i < out.length) {
            const n = out.length;
            const a = out[(i + n - 1) % n];
            const b = out[i];
            const c = out[(i + 1) % n];
            if (Math.abs(area(a, b, c)) <= 1e-9) {
                out.splice(i, 1);
                continue;
            }
            i++;
        }

        return out;
    }

    /**
     * @param {Contour[]} contour
        * @param {boolean} ccw
     */
    ensureWinding(contour, ccw) {
        const area2 = this.computeSignedArea(contour);
        if (ccw && area2 < 0) contour.reverse();
        if (!ccw && area2 > 0) contour.reverse();
    }

    /**
     * @param {Contour[]} contour
     */
    isDegenerateContour(contour) {
        if (!contour || contour.length < 3) return true;
        const unique = new Set();
        for (const p of contour) unique.add(`${p.x}|${p.y}`);
        if (unique.size < 3) return true;
        return Math.abs(this.computeSignedArea(contour)) <= 1e-6;
    }

    /**
     * @param {Contour[]} contour
     */
    computeSignedArea(contour) {
        let sum = 0;
        for (let i = 0; i < contour.length; i++) {
            const a = contour[i];
            const b = contour[(i + 1) % contour.length];
            sum += a.x * b.y - b.x * a.y;
        }
        return sum * 0.5;
    }

    /**
     * @param {{x:number,y:number}[]} contour
     */
    computeSignedAreaXY(contour) {
        let sum = 0;
        for (let i = 0; i < contour.length; i++) {
            const a = contour[i];
            const b = contour[(i + 1) % contour.length];
            sum += a.x * b.y - b.x * a.y;
        }
        return sum * 0.5;
    }

    /**
     * @param {Contour} pt
     * @param {Contour[]} polygon
     */
    pointInPolygon2D(pt, polygon) {
        let inside = false;
        for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            const pi = polygon[i];
            const pj = polygon[j];
            const intersects = ((pi.y > pt.y) !== (pj.y > pt.y))
                && (pt.x < (pj.x - pi.x) * (pt.y - pi.y) / ((pj.y - pi.y) || 1e-9) + pi.x);
            if (intersects) inside = !inside;
        }
        return inside;
    }

    /**
     * @param {Contour[]} contour
     */
    getLeftMostPoint(contour) {
        let p = contour[0];
        for (let i = 1; i < contour.length; i++) {
            const v = contour[i];
            if (v.x < p.x || (v.x === p.x && v.y < p.y)) p = v;
        }
        return p;
    }

    /**
     * @param {Contour} p1
     * @param {Contour} p2
     * @param {Contour} p3
     * @param {Contour} p4
     * @param {boolean} includeEnd
     */
    segmentsIntersect(p1, p2, p3, p4, includeEnd) {
        const cross = (
            /** @type {{x:number,y:number}} */ a,
            /** @type {{x:number,y:number}} */ b,
            /** @type {{x:number,y:number}} */ c
        ) => (c.y - a.y) * (b.x - a.x) - (b.y - a.y) * (c.x - a.x);

        const d1 = cross(p1, p2, p3);
        const d2 = cross(p1, p2, p4);
        const d3 = cross(p3, p4, p1);
        const d4 = cross(p3, p4, p2);
        if (includeEnd) return (d1 * d2 <= 0 && d3 * d4 <= 0);
        return (d1 * d2 < 0 && d3 * d4 < 0);
    }

    /**
     * @param {Contour} holePt
     * @param {Contour[]} outer
     * @param {Contour[][]} holes
     * @param {number} holeId
     */
    findBridgeOuterIndex(holePt, outer, holes, holeId) {
        const hole = holes[holeId];
        let bestDistSq = Infinity;
        let bestIdx = -1;

        for (let i = 0; i < outer.length; i++) {
            const a = outer[i];
            const dx = holePt.x - a.x;
            const dy = holePt.y - a.y;
            const distSq = dx * dx + dy * dy;
            if (distSq >= bestDistSq) continue;

            let intersects = false;

            for (let j = 0; j < outer.length; j++) {
                const p1 = outer[j];
                const p2 = outer[(j + 1) % outer.length];
                if (j === i || (j + 1) % outer.length === i) continue;
                if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                    intersects = true;
                    break;
                }
            }
            if (intersects) continue;

            for (let j = 0; j < hole.length; j++) {
                const p1 = hole[j];
                const p2 = hole[(j + 1) % hole.length];
                if (p1 === holePt || p2 === holePt) continue;
                if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                    intersects = true;
                    break;
                }
            }
            if (intersects) continue;

            for (let k = holeId + 1; k < holes.length; k++) {
                const other = holes[k];
                for (let j = 0; j < other.length; j++) {
                    const p1 = other[j];
                    const p2 = other[(j + 1) % other.length];
                    if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                        intersects = true;
                        break;
                    }
                }
                if (intersects) break;
            }
            if (intersects) continue;

            bestDistSq = distSq;
            bestIdx = i;
        }

        return bestIdx;
    }

    /**
     * @param {Contour[]} outer
     * @param {Contour[][]} holes
     * @param {number} holeId
     */
    mergeHoleIntoOuter(outer, holes, holeId) {
        const hole = holes[holeId];
        let oi = -1;
        let holePt = hole[0];
        let hi = 0;

        for (hi = 0; hi < hole.length; hi++) {
            holePt = hole[hi];
            oi = this.findBridgeOuterIndex(holePt, outer, holes, holeId);
            if (oi >= 0) break;
        }

        if (oi < 0) {
            Instance.Msg("未找到洞桥接点，跳过该洞");
            this.error=true;
            return outer;
        }

        /** @type {Contour[]} */
        const merged = [];

        for (let i = 0; i <= oi; i++) merged.push(outer[i]);
        merged.push(holePt);
        for (let i = 1; i <= hole.length; i++) merged.push(hole[(hi + i) % hole.length]);
        merged.push(outer[oi]);
        for (let i = oi + 1; i < outer.length; i++) merged.push(outer[i]);

        return merged;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    triangulate(poly) {
        let verts = this.sanitizeTriangulationInput(poly);
        if (verts.length < 3) return [];
        if (this.computeSignedAreaXY(verts) < 0) verts = verts.reverse();

        /** @type {{x:number,y:number,z:number,regionId:number}[][]} */
        const result = [];

        let guard = 0;
        while (verts.length > 3 && guard++ < 5000) {
            let bestIndex = -1;
            let bestPerimeter = Infinity;

            for (let i = 0; i < verts.length; i++) {
                const prev = verts[(i - 1 + verts.length) % verts.length];
                const cur = verts[i];
                const next = verts[(i + 1) % verts.length];

                if (!isConvex(prev, cur, next)) continue;

                let blocked = false;
                for (let j = 0; j < verts.length; j++) {
                    if (j === i || j === (i - 1 + verts.length) % verts.length || j === (i + 1) % verts.length) continue;
                    if (pointInTri(verts[j], prev, cur, next)) {
                        blocked = true;
                        break;
                    }
                }
                if (blocked) continue;

                for (let j = 0; j < verts.length; j++) {
                    if (j === i || j === (i - 1 + verts.length) % verts.length || j === (i + 1) % verts.length) continue;
                    if (distPtSegSq(verts[j], prev, next) <= 1e-9) {
                        if (vec.length2D(prev, verts[j]) === 0 || vec.length2D(next, verts[j]) === 0) continue;
                        blocked = true;
                        break;
                    }
                }
                if (blocked) continue;

                const perimeter = vec.length2D(prev, cur) + vec.length2D(cur, next) + vec.length2D(next, prev);
                {
                    if (perimeter < bestPerimeter) {
                        bestPerimeter = perimeter;
                        bestIndex = i;
                    }
                }
            }

            if (bestIndex < 0) break;

            const prev = verts[(bestIndex - 1 + verts.length) % verts.length];
            const cur = verts[bestIndex];
            const next = verts[(bestIndex + 1) % verts.length];
            result.push([prev, cur, next]);
            verts.splice(bestIndex, 1);
        }

        if (verts.length === 3) {
            result.push([verts[0], verts[1], verts[2]]);
            return result;
        }

        if (verts.length !== 0) {
            this.error = true;
            Instance.Msg(`区域(${poly[0].regionId})：耳切失败，跳过该轮廓`);
            return [];
        }

        return result;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    sanitizeTriangulationInput(poly) {
        /** @type {{x:number,y:number,z:number,regionId:number}[]} */
        const out = [];
        for (let i = 0; i < poly.length; i++) {
            const cur = poly[i];
            const prev = out[out.length - 1];
            if (prev && prev.x === cur.x && prev.y === cur.y) continue;
            out.push(cur);
        }

        if (out.length >= 2) {
            const a = out[0];
            const b = out[out.length - 1];
            if (a.x === b.x && a.y === b.y) out.pop();
        }

        let i = 0;
        while (out.length >= 3 && i < out.length) {
            const n = out.length;
            const a = out[(i + n - 1) % n];
            const b = out[i];
            const c = out[(i + 1) % n];
            if (Math.abs(area(a, b, c)) <= 1e-9) {
                out.splice(i, 1);
                continue;
            }
            i++;
        }

        return out;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[][]} tris
     * @param {boolean} longestEdgeFirst
     */
    mergeTriangles(tris, longestEdgeFirst) {
        const polys = tris.map((t) => t.slice());
        let merged = true;

        while (merged) {
            merged = false;

            let bestI = -1;
            let bestJ = -1;
            let bestPoly = null;
            let bestDist = -Infinity;

            for (let i = 0; i < polys.length; i++) {
                for (let j = i + 1; j < polys.length; j++) {
                    const info = this.getMergeInfo(polys[i], polys[j]);
                    if (!info) continue;

                    if (!longestEdgeFirst) {
                        bestI = i;
                        bestJ = j;
                        bestPoly = info.info;
                        break;
                    }

                    if (info.dist > bestDist) {
                        bestDist = info.dist;
                        bestI = i;
                        bestJ = j;
                        bestPoly = info.info;
                    }
                }
                if (!longestEdgeFirst && bestPoly) break;
            }

            if (!bestPoly) break;

            polys[bestI] = bestPoly;
            polys.splice(bestJ, 1);
            merged = true;
        }

        return polys;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} a
     * @param {{x:number,y:number,z:number,regionId:number}[]} b
     */
    getMergeInfo(a, b) {
        let ai = -1;
        let bi = -1;
        const eps = 1e-6;

        for (let i = 0; i < a.length; i++) {
            const an = (i + 1) % a.length;
            for (let j = 0; j < b.length; j++) {
                const bn = (j + 1) % b.length;
                if (vec.length(a[i], b[bn]) <= eps && vec.length(a[an], b[j]) <= eps) {
                    ai = i;
                    bi = j;
                    break;
                }
            }
            if (ai >= 0) break;
        }

        if (ai < 0) return null;

        /** @type {{x:number,y:number,z:number,regionId:number}[]} */
        const merged = [];
        const nA = a.length;
        const nB = b.length;
        for (let i = 0; i < nA - 1; i++) merged.push(a[(ai + 1 + i) % nA]);
        for (let i = 0; i < nB - 1; i++) merged.push(b[(bi + 1 + i) % nB]);

        if (merged.length > POLY_MAX_VERTS_PER_POLY) return null;
        if (!this.isPolyConvex(merged)) return null;

        const v1 = a[ai];
        const v2 = a[(ai + 1) % nA];
        const distSq = (v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2;

        return { info: merged, dist: distSq };
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    isPolyConvex(poly) {
        const n = poly.length;
        for (let i = 0; i < n; i++) {
            if (area(poly[i], poly[(i + 1) % n], poly[(i + 2) % n]) < -1e-6) return false;
        }
        return true;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    addPolygon(poly) {
        const idx = [];
        for (const v of poly) {
            const key = `${v.x}|${v.y}|${v.z}`;
            let vi = this.vertIndexMap.get(key);
            if (vi === undefined) {
                vi = this.verts.length;
                this.verts.push({ x: v.x, y: v.y, z: v.z });
                this.vertIndexMap.set(key, vi);
            }
            idx.push(vi);
        }

        this.polys.push(idx);
        this.regions.push(poly[0].regionId);
        this.neighbors.push(new Array(idx.length).fill(0).map(() => []));
    }

    convertVertsToWorldAfterAdjacency() {
        if (this.worldConverted) return;
        for (let i = 0; i < this.verts.length; i++) {
            this.verts[i] = this.toWorldVertex(this.verts[i]);
        }
        this.worldConverted = true;
    }

    /**
     * @param {{x:number,y:number,z:number}} v
     */
    toWorldVertex(v) {
        return {
            x: origin.x + v.x * MESH_CELL_SIZE_XY,// - MESH_CELL_SIZE_XY / 2,
            y: origin.y + v.y * MESH_CELL_SIZE_XY,// - MESH_CELL_SIZE_XY / 2,
            z: origin.z + v.z * MESH_CELL_SIZE_Z
        };
    }

    buildAdjacency() {
        /**@type {Map<string, {poly: number, edge: number}>} */
        const edgeMap = new Map();

        for (let pi = 0; pi < this.polys.length; pi++) {
            const poly = this.polys[pi];
            for (let ei = 0; ei < poly.length; ei++) {
                const a = poly[ei];
                const b = poly[(ei + 1) % poly.length];
                const k = a < b ? `${a},${b}` : `${b},${a}`;

                if (!edgeMap.has(k)) {
                    edgeMap.set(k, { poly: pi, edge: ei });
                } else {
                    const other = edgeMap.get(k);
                    if(!other)continue;
                    this.neighbors[pi][ei].push(other.poly);
                    this.neighbors[other.poly][other.edge].push(pi);
                }
            }
        }
    }

    debugDrawPolys(duration = 5) {
        for (let pi = 0; pi < this.polys.length; pi++) {
            const poly = this.polys[pi];
            const color = { r: 255, g: 255, b: 0 };
            for (let i = 0; i < poly.length; i++) {
                const start = vec.Zfly(this.verts[poly[i]], 0);
                const end = vec.Zfly(this.verts[poly[(i + 1) % poly.length]], 0);
                Instance.DebugLine({ start, end, color, duration });
            }
        }
    }

    debugDrawAdjacency(duration = 15) {
        for (let i = 0; i < this.polys.length; i++) {
            const start = this.polyCenter(i);
            for (let e = 0; e < this.neighbors[i].length; e++) {
                for(let ni = 0; ni < this.neighbors[i][e].length; ni++){
                    const neighborIndex = this.neighbors[i][e][ni];
                    if(neighborIndex < 0 || neighborIndex < i) continue;
                    const end = this.polyCenter(neighborIndex);
                    Instance.DebugLine({ start, end, color: { r: 255, g: 0, b: 255 }, duration });
                }
            }
        }
    }

    /**
     * @param {number} pi
     */
    polyCenter(pi) {
        const poly = this.polys[pi];
        let x = 0;
        let y = 0;
        let z = 0;

        for (const vi of poly) {
            const v = this.verts[vi];
            x += v.x;
            y += v.y;
            z += v.z;
        }

        const n = poly.length;
        return { x: x / n, y: y / n, z: z / n };
    }

    debugDrawSharedEdges(duration = 15) {
        for (let i = 0; i < this.polys.length; i++) {
            const polyA = this.polys[i];
            for (let ei = 0; ei < polyA.length; ei++) {
                const ni = this.neighbors[i][ei];
                if(ni.length<=0) continue;
                const start = vec.Zfly(this.verts[polyA[ei]], 20);
                const end = vec.Zfly(this.verts[polyA[(ei + 1) % polyA.length]], 20);
                Instance.DebugLine({ start, end, color: { r: 0, g: 255, b: 0 }, duration });
            }
        }
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */

class PolyMeshDetailBuilder {
    /**
     * @param {NavMeshMesh} mesh
     * @param {OpenHeightfield} hf
     */
    constructor(mesh, hf) {
        /** @type {boolean} */
        this.error = false;
        this.mesh = mesh;
        /**@type {OpenHeightfield} */
        this.hf = hf;
        /**@type {Vector[]}*/
        this.verts = [];
        /**@type {number[][]}*/
        this.tris = [];
        /**@type {number[][]}*/
        this.meshes = [];
        /**@type {number[]} */
        this.triTopoly=[];
    }

    init() {
        this.error = false;
        for (let pi = 0; pi < this.mesh.polys.length; pi++) {
            this.buildPoly(pi);
        }

        return {
            verts: this.verts,
            tris: this.tris,
            meshes: this.meshes,
            triTopoly:this.triTopoly
        };
    }
    debugDrawPolys(duration = 5) {
        for (let pi = 0; pi < this.tris.length; pi++) {
            const tri = this.tris[pi];
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            Instance.DebugLine({ start:this.verts[tri[0]], end:this.verts[tri[1]], color, duration });
            Instance.DebugLine({ start:this.verts[tri[1]], end:this.verts[tri[2]], color, duration });
            Instance.DebugLine({ start:this.verts[tri[2]], end:this.verts[tri[0]], color, duration });
        }
    }
    /**
     * @param {number} pi
     */
    buildPoly(pi) {
        const poly = this.mesh.polys[pi];
        const regionid=this.mesh.regions[pi];
        const polyVerts = this.getPolyVerts(this.mesh, poly);
        // 待优化：内部采样点高度可改为基于细分后三角形插值

        // 1. 为多边形边界顶点采样高度
        const borderVerts = this.applyHeights(polyVerts, this.hf,regionid);
        // 2. 计算边界平均高度和高度范围
        const borderHeightInfo = this.calculateBorderHeightInfo(borderVerts);
        // 3. 获取初始三角划分（用于高度误差检查）
        const initialVertices = [...borderVerts];
        const initialConstraints = [];
        for (let i = 0; i < borderVerts.length; i++) {
            const j = (i + 1) % borderVerts.length;
            initialConstraints.push([i, j]);
        }
        // 4. 执行初始划分（基于边界点）
        const trianglesCDT = new SimplifiedCDT(initialVertices, initialConstraints, () => {
            this.error = true;
        });
        let triangles = trianglesCDT.getTri();
        // 5. 生成内部采样点
        let rawSamples = this.buildDetailSamples(polyVerts, borderHeightInfo, this.hf,triangles,trianglesCDT.vertices,regionid);
        // 6. 过滤内部采样点：仅保留高度误差较大的点
        while(rawSamples.length>0)
        {
            let insert=false;
            let heightDiff = 0;
            let heightid = -1;
            triangles = trianglesCDT.getTri();
            let toRemoveIndices = [];
            for (let i=0;i<rawSamples.length;i++) {
                const sample=rawSamples[i];
                let diff=0;
                // 找到包含采样点的三角形
                for (const tri of triangles) {
                    if (tri.containsPoint(sample, trianglesCDT.vertices)) {
                        const interpolatedHeight = tri.interpolateHeight(sample.x, sample.y, trianglesCDT.vertices);
                        diff = Math.abs(sample.z - interpolatedHeight);
                        if(this.isNearTriangleEdge(sample,tri,trianglesCDT.vertices)) diff = 0;
                        break;
                    }
                }
                // 仅当高度误差超过阈值时保留
                if(diff<=POLY_DETAIL_HEIGHT_ERROR)toRemoveIndices.push(i);
                else if (diff > heightDiff) {
                    heightDiff=diff;
                    heightid=i;
                    insert=true;
                }
            }
            if(insert)trianglesCDT.insertPointSimplified(rawSamples[heightid]);
            else break;
            for (let i = toRemoveIndices.length - 1; i >= 0; i--) {
                rawSamples.splice(toRemoveIndices[i], 1);
            }
        }
        // 7. 添加到全局列表
        const baseVert = this.verts.length;
        const baseTri = this.tris.length;
        const allVerts=trianglesCDT.vertices;
        for (const v of allVerts) {
            this.verts.push(v);
        }
        triangles = trianglesCDT.getTri();
        if (trianglesCDT.error) this.error = true;

        for (const tri of triangles) {
            this.tris.push([
                baseVert + tri.a,
                baseVert + tri.b,
                baseVert + tri.c
            ]);
            this.triTopoly.push(pi);
        }

        this.meshes.push([
            baseVert,
            allVerts.length,
            baseTri,
            triangles.length
        ]);
    }
    /**
    * 计算边界顶点高度信息
     * @param {Vector[]} borderVerts
     * @returns {{avgHeight: number, minHeight: number, maxHeight: number, heightRange: number}}
     */
    calculateBorderHeightInfo(borderVerts) {
        let sumHeight = 0;
        let minHeight = Infinity;
        let maxHeight = -Infinity;

        for (const v of borderVerts) {
            sumHeight += v.z;
            minHeight = Math.min(minHeight, v.z);
            maxHeight = Math.max(maxHeight, v.z);
        }

        const avgHeight = sumHeight / borderVerts.length;
        const heightRange = maxHeight - minHeight;

        return {
            avgHeight,
            minHeight,
            maxHeight,
            heightRange
        };
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {number[]} poly
     */
    getPolyVerts(mesh, poly) {
        return poly.map(vi => mesh.verts[vi]);
    }
    /**
    * 生成内部采样点（带高度误差检查）
     * @param {Vector[]} polyVerts
     * @param {{avgHeight: number;minHeight: number;maxHeight: number;heightRange: number;}} heightInfo
     * @param {OpenHeightfield} hf
     * @returns {Vector[]}
     * @param {Triangle[]} initialTriangles
     * @param {Vector[]} initialVertices
     * @param {number} regionid
     */
    buildDetailSamples(polyVerts, heightInfo, hf,initialTriangles,initialVertices,regionid) {
        const samples = [];
        // 2. AABB
        let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
        for (const v of polyVerts) {
            minx = Math.min(minx, v.x);
            miny = Math.min(miny, v.y);
            maxx = Math.max(maxx, v.x);
            maxy = Math.max(maxy, v.y);
        }

        const step = POLY_DETAIL_SAMPLE_DIST * MESH_CELL_SIZE_XY;
        for (let x = minx + step / 2; x <= maxx; x += step) {
            for (let y = miny + step / 2; y <= maxy; y += step) {
                if (this.pointInPoly2D(x, y, polyVerts)) {
                    // 采样高度
                    let triheight=heightInfo.avgHeight;

                    // 计算与边界平均高度的差值
                    //const heightDiff = Math.abs(height - heightInfo.avgHeight);
                    for (const tri of initialTriangles) {
                        if (tri.containsPoint({x, y,z:heightInfo.avgHeight},initialVertices)) {
                            // 使用三角形插值计算高度
                            triheight = tri.interpolateHeight(x, y, initialVertices);
                            break;
                        }
                    }
                    const height=this.sampleHeight(hf, x, y, triheight??heightInfo.avgHeight,regionid);
                    // 检查是否超过阈值
                    if(Math.abs(height - triheight)>POLY_DETAIL_HEIGHT_ERROR) {
                        samples.push({ x: x, y: y, z: height });
                    }
                }
            }
        }
        return samples;
    }
    /**
     * @param {Vector} sample
     * @param {Triangle} tri
     * @param {Vector[]} verts
     */
    isNearTriangleEdge(sample, tri, verts) {

        const dis = Math.min(distPtSegSq(sample,verts[tri.a],verts[tri.b]),distPtSegSq(sample,verts[tri.b],verts[tri.c]),distPtSegSq(sample,verts[tri.c],verts[tri.a]));
        if (dis < POLY_DETAIL_SAMPLE_DIST * 0.5) return true;
        return false;
    }
    /**
     * @param {Vector[]} polyVerts
     * @param {OpenHeightfield} hf
     * @param {number} regionid
     */
    applyHeights(polyVerts, hf,regionid) {
        const resultVerts = [];
        const n = polyVerts.length;
        const step = POLY_DETAIL_SAMPLE_DIST * MESH_CELL_SIZE_XY;
        for (let i = 0; i < n; i++) {
            const a = polyVerts[i];
            const b = polyVerts[(i + 1) % n];
            // 对当前顶点采样高度
            const az = this.sampleHeight(hf, a.x, a.y, a.z,regionid);
            const bz = this.sampleHeight(hf, b.x, b.y, b.z, regionid);
            const A = { x: a.x, y: a.y, z: az };
            const B = { x: b.x, y: b.y, z: bz };
            // 添加当前顶点（起始点）
            resultVerts.push(A);

            // 细分当前边
            const samples = this.sampleEdgeWithHeightCheck(
                A, 
                B, 
                hf,
                step
            );
            // 递归插点
            this.subdivideEdgeByHeight(
                A,
                B,
                samples,
                hf,
                regionid,
                resultVerts
            );
        }
        
        return resultVerts;
    }
    /**
    * 在 [start, end] 之间递归插入高度误差最大的点
     * @param {Vector} start
     * @param {Vector} end
    * @param {Vector[]} samples // 该边上的细分点（不含 start/end）
     * @param {OpenHeightfield} hf
     * @param {number} regionid
     * @param {Vector[]} outVerts
     */
    subdivideEdgeByHeight(start, end,samples,hf,regionid,outVerts) {
        let maxError = 0;
        let maxIndex = -1;
        let maxVert = null;

        const total = samples.length;

        for (let i = 0; i < total; i++) {
            const s = samples[i];
            const t = (i + 1) / (total + 1);

            // 不加入该点时的插值高度
            const interpZ = start.z * (1 - t) + end.z * t;

            const h = this.sampleHeight(hf, s.x, s.y, interpZ, regionid);
            const err = Math.abs(h - interpZ);

            if (err > maxError) {
                maxError = err;
                maxIndex = i;
                maxVert = { x: s.x, y: s.y, z: h };
            }
        }

        // 没有需要加入的点
        if (maxError <= POLY_DETAIL_HEIGHT_ERROR || maxIndex === -1||!maxVert) {
            return;
        }

        // 递归左半段
        this.subdivideEdgeByHeight(
            start,
            maxVert,
            samples.slice(0, maxIndex),
            hf,
            regionid,
            outVerts
        );

        // 插入当前最大误差点（保持顺序）
        outVerts.push(maxVert);

        // 递归右半段
        this.subdivideEdgeByHeight(
            maxVert,
            end,
            samples.slice(maxIndex + 1),
            hf,
            regionid,
            outVerts
        );
    }
    /**
    * 在边上采样并检查高度误差
     * @param {Vector} start
     * @param {Vector} end
     * @param {OpenHeightfield} hf
     * @param {number} sampleDist
     * @returns {Vector[]}
     */
    sampleEdgeWithHeightCheck(start, end, hf, sampleDist) {
        const samples = [];
        
        // 计算边向量和长度
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const length = Math.sqrt(dx * dx + dy * dy);
        
        if (length <= 1e-6) {
            return []; // 边长度为 0，不采样
        }
        
        // 计算方向向量
        const dirX = dx / length;
        const dirY = dy / length;
        // 计算采样点数（不包含起点和终点）
        const numSamples = Math.floor(length / sampleDist);
        
        // 记录采样点高度

        for (let i = 1; i <= numSamples; i++) {
            const t = i / (numSamples + 1); // 确保不会采样到端点
            const x = start.x + dirX * length * t;
            const y = start.y + dirY * length * t;
            const z = start.z * (1 - t) + end.z * t;
            samples.push({ x, y, z });
        }
        
        return samples;
    }
    /**
     * @param {OpenHeightfield} hf
     * @param {number} wx
     * @param {number} wy
     * @param {number} fallbackZ
     * @param {number} regionid
     */
    sampleHeight(hf, wx, wy, fallbackZ,regionid) {
        const globalIx = Math.round((wx - origin.x+ MESH_CELL_SIZE_XY / 2) / MESH_CELL_SIZE_XY);
        const globalIy = Math.round((wy - origin.y+ MESH_CELL_SIZE_XY / 2) / MESH_CELL_SIZE_XY);
        const ix = globalIx - (hf.baseX);
        const iy = globalIy - (hf.baseY);

        if (ix < 0 || iy < 0 || ix >= hf.gridX || iy >= hf.gridY) return fallbackZ;

        let best = null;
        let bestDiff = Infinity;
        let spanId = hf.cells[ix][iy];
        while (spanId !== 0) {
            if(OpenSpan.getRegionId(spanId)===regionid)
            {
                const z = origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z;
                const d = Math.abs(z - fallbackZ);
                if (d < bestDiff) {
                    bestDiff = d;
                    best = z;
                }
            }
            spanId = OpenSpan.getNext(spanId);
        }
        // 如果没有找到合适的 span，开始螺旋式搜索
        if (best === null) {
            const maxRadius = Math.max(hf.gridX, hf.gridY); // 搜索最大半径
            let radius = 1; // 初始半径
            out:
            while (radius <= maxRadius) {
                // 螺旋式外扩，检查四个方向
                for (let offset = 0; offset <= radius; offset++) {
                    // 检查 (ix + offset, iy + radius) 等候选位置
                    let candidates = [
                        [ix + offset, iy + radius], // 上
                        [ix + radius, iy + offset], // 右
                        [ix - offset, iy - radius], // 下
                        [ix - radius, iy - offset]  // 左
                    ];

                    for (const [nx, ny] of candidates) {
                        if (nx >= 0 && ny >= 0 && nx < hf.gridX && ny < hf.gridY) {
                            // 在有效范围内查找对应 span
                            spanId = hf.cells[nx][ny];
                            while (spanId !== 0) {
                                if(OpenSpan.getRegionId(spanId)===regionid)
                                {
                                    const z = origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z;
                                    const d = Math.abs(z - fallbackZ);
                                    if (d < bestDiff) {
                                        bestDiff = d;
                                        best = z;
                                        break out;
                                    }
                                }
                                spanId = OpenSpan.getNext(spanId);
                            }
                        }
                    }
                }
                // 增大半径，继续搜索
                radius++;
            }
        }

        // 如果最终未找到合适 span，返回 fallbackZ
        return best ?? fallbackZ;
    }
    /**
    * 判断点是否在多边形内（不含边界）
    * 使用 odd-even rule（射线法）
     *
     * @param {number} px
     * @param {number} py
     * @param {{x:number,y:number}[]} poly
     * @returns {boolean}
     */
    pointInPoly2D(px, py, poly) {
        let inside = false;
        const n = poly.length;

        for (let i = 0, j = n - 1; i < n; j = i++) {
            const xi = poly[i].x, yi = poly[i].y;
            const xj = poly[j].x, yj = poly[j].y;

            // ===== 点在边上，按 outside 处理 =====
            if (Tool.pointOnSegment2D(px, py, xi, yi, xj, yj, { includeEndpoints: true })) {
                return false;
            }

            // ===== 射线法 =====
            const intersect =
                ((yi > py) !== (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi);

            if (intersect) inside = !inside;
        }

        return inside;
    }

}

/**
 * 简化的约束 Delaunay 三角剖分器
 */
class SimplifiedCDT {
    /**
    * @param {Vector[]} vertices 顶点列表
    * @param {number[][]} constraints 约束边列表
     * @param {(() => void)} onError
     */
    constructor(vertices, constraints, onError) {
        /** @type {boolean} */
        this.error = false;
        /** @type {(() => void) | undefined} */
        this.onError = onError;
        this.vertices = vertices;
        this.constraints = constraints;
        /** @type {Triangle[]} */
        this.triangles = [];
        
        // 构建约束边查找集合
        this.constraintEdges = new Set();
        for (const [a, b] of constraints) {
            // 规范化边键（小索引在前）
            const key = Tool.orderedPairKey(a, b);
            this.constraintEdges.add(key);
        }
        // 初始剖分：耳切法
        this.earClipping(vertices);
    }

    /**
    * @returns {Triangle[]} 三角形顶点索引列表
     */
    getTri() {
        return this.triangles;
    }
    /**
     * @param {Vector[]} poly
     */
    earClipping(poly) {
        const verts = Array.from({ length: poly.length }, (_, i) => i);
        let guard = 0;
        while (verts.length > 3 && guard++ < 5000) {
            let bestEar=null;
            let minPerimeter=Infinity;
            let bestIndex=-1;

            for (let i = 0; i < verts.length; i++) {
                const prev = poly[verts[(i - 1 + verts.length) % verts.length]];
                const cur = poly[verts[i]];
                const next = poly[verts[(i + 1) % verts.length]];
                // cur 对应角度是否小于 180 度
                if (!isConvex(prev, cur, next)) continue;
                // 检查三角形是否包含其他点
                let contains = false;
                for (let j = 0; j < verts.length; j++) {
                    if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                    if (pointInTri(poly[verts[j]], prev, cur, next)) {
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                // 其他点不能在线段 prev-next 上
                for (let j = 0; j < verts.length; j++) {
                    if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                    if (distPtSegSq(poly[verts[j]], prev, next) == 0) // 判断点是否在线段上
                    {
                        if (vec.length2D(prev, poly[verts[j]]) == 0 || vec.length2D(next, poly[verts[j]]) == 0) continue;
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                const perimeter = 
                vec.length2D(prev, cur) +
                vec.length2D(cur, next) +
                vec.length2D(next, prev);
            
                // 找到周长最小的耳朵
                if (perimeter < minPerimeter) {
                    minPerimeter = perimeter;
                    bestEar = {p:verts[(i - 1 + verts.length) % verts.length], c:verts[i], n:verts[(i + 1) % verts.length]};
                    bestIndex = i;
                }
            }
            // 找到最佳耳朵则切除
            if (bestEar && bestIndex !== -1) {
                this.triangles.push(new Triangle(bestEar.p, bestEar.c, bestEar.n));
                verts.splice(bestIndex, 1);
            } else {
                // 找不到耳朵，退出循环
                break;
            }
        }
        if (verts.length == 3) {
            this.triangles.push(new Triangle(verts[0], verts[1], verts[2]));
        }else {
            this.error = true;
            if (this.onError) this.onError();
            Instance.Msg("细节多边形耳切失败");
        }
    }
    /**
     * 简化的点插入方法
     * @param {Vector} point
     */
    insertPointSimplified(point) {

        const pointIndex = this.vertices.length;
        this.vertices.push(point);
        const p=this.vertices[pointIndex];
        let targetIdx = -1;

        // 找到包含点的三角形
        for (let i = 0; i < this.triangles.length; i++) {
            if (this.triangles[i].containsPoint(p, this.vertices)) {
                targetIdx = i;
                break;
            }
        }
        
        if (targetIdx === -1) {
            // 点不在任何三角形内（可能在边上），尝试处理边上点
            this.handlePointOnEdge(pointIndex);
            //Instance.Msg("点在边上");
            return;
        }

        const t = this.triangles[targetIdx];

        this.triangles.splice(targetIdx, 1);

        // 分裂为三个新三角形
        const t1 = new Triangle(t.a, t.b, pointIndex);
        const t2 = new Triangle(t.b, t.c, pointIndex);
        const t3 = new Triangle(t.c, t.a, pointIndex);
        
        this.triangles.push(t1, t2, t3);

        // 只对这三条边进行局部优化
        this.legalizeEdge(pointIndex, t.a, t.b);
        this.legalizeEdge(pointIndex, t.b, t.c);
        this.legalizeEdge(pointIndex, t.c, t.a);
    }
    /**
     * 处理点在边上的情况
     * @param {number} pointIndex 
     */
    handlePointOnEdge(pointIndex) {
        const p = this.vertices[pointIndex];
        // 先检查是否在约束边上
        for (const [a, b] of this.constraints) {
            if (Tool.pointOnSegment2D(p.x, p.y, this.vertices[a].x, this.vertices[a].y, this.vertices[b].x, this.vertices[b].y, { includeEndpoints: true })) {
                return;
            }
        }
        // 查找包含该点的边
        for (let i = 0; i < this.triangles.length; i++) {
            const tri = this.triangles[i];
            const edges = tri.edges();
            
            for (const [a, b] of edges) {
                if (this.isConstraintEdge(a, b)) continue;
                if (Tool.pointOnSegment2D(p.x, p.y, this.vertices[a].x, this.vertices[a].y, this.vertices[b].x, this.vertices[b].y, { includeEndpoints: true })) {
                    // 找到共享该边的另一个三角形
                    const otherTri = this.findAdjacentTriangleByEdge([a, b], tri);
                    
                    if (otherTri) {

                        // 移除两个共享该边的三角形
                        this.triangles.splice(this.triangles.indexOf(tri), 1);
                        this.triangles.splice(this.triangles.indexOf(otherTri), 1);
                        
                        // 获取两个三角形中不在该边上的顶点
                        const c = tri.oppositeVertex(a, b);
                        const d = otherTri.oppositeVertex(a, b);
                        
                        // 创建四个新三角形
                        const t1=new Triangle(a, pointIndex, c);
                        const t2=new Triangle(pointIndex, b, c);
                        const t3=new Triangle(a, d, pointIndex);
                        const t4=new Triangle(pointIndex, d, b);

                        this.triangles.push(t1,t2,t3,t4);

                        // 优化新产生的边
                        this.legalizeEdge(pointIndex, a, c);
                        this.legalizeEdge(pointIndex, b, c);
                        this.legalizeEdge(pointIndex, a, d);
                        this.legalizeEdge(pointIndex, b, d);
                        
                        return;
                    }
                }
            }
        }
    }
    /**
    * 局部递归优化 (Standard Delaunay Legalization)
    * @param {number} pIdx 新插入点
    * @param {number} v1 边的一端
    * @param {number} v2 边的另一端
     */
    legalizeEdge(pIdx, v1, v2) {
        // 约束边不可翻转
        if (this.isConstraintEdge(v1, v2)) {
            return;
        }
        
        const edge = [v1, v2];
        const triangleWithP = this.findTriangleByVerts(v1, v2, pIdx);
        if (!triangleWithP) return;
        
        const t2 = this.findAdjacentTriangleByEdge(edge, triangleWithP);
        if (!t2) return;

        const otherVert = t2.oppositeVertex(v1, v2);
        
        // 检查 Delaunay 条件
        if (this.inCircumcircle(
            this.vertices[v1], 
            this.vertices[v2], 
            this.vertices[pIdx], 
            this.vertices[otherVert]
        )) {
            // 翻转边
            this.removeTriangle(t2);
            this.removeTriangle(triangleWithP);

            // 创建两个新三角形
            const tt1=new Triangle(v1, otherVert, pIdx);
            const tt2=new Triangle(v2, otherVert, pIdx);

            this.triangles.push(tt1,tt2);

            // 递归优化新产生的两条外边
            this.legalizeEdge(pIdx, v1, otherVert);
            this.legalizeEdge(pIdx, v2, otherVert);
        }
    }
    
    /**
    * 检查是否为约束边
     * @param {number} a
     * @param {number} b
     */
    isConstraintEdge(a, b) {
        const key = Tool.orderedPairKey(a, b);
        return this.constraintEdges.has(key);
    }

    /**
    * 通过三个顶点找到三角形
     * @param {number} a
     * @param {number} b
     * @param {number} c
     */
    findTriangleByVerts(a, b, c) {
        for (const tri of this.triangles) {
            if ((tri.a === a && tri.b === b && tri.c === c) ||
                (tri.a === a && tri.b === c && tri.c === b) ||
                (tri.a === b && tri.b === a && tri.c === c) ||
                (tri.a === b && tri.b === c && tri.c === a) ||
                (tri.a === c && tri.b === a && tri.c === b) ||
                (tri.a === c && tri.b === b && tri.c === a)) {
                return tri;
            }
        }
        return null;
    }
    
    /**
    * 通过共享边找到相邻三角形
     * @param {number[]} edge
     * @param {Triangle} excludeTriangle
     */
    findAdjacentTriangleByEdge(edge, excludeTriangle) {
        const [a, b] = edge;
        
        for (const tri of this.triangles) {
            if (tri === excludeTriangle) continue;
            
            if ((tri.a === a && tri.b === b) ||
                (tri.a === b && tri.b === a) ||
                (tri.a === a && tri.c === b) ||
                (tri.a === b && tri.c === a) ||
                (tri.b === a && tri.c === b) ||
                (tri.b === b && tri.c === a)) {
                return tri;
            }
        }
        
        return null;
    }
    
    /**
    * 移除三角形
     * @param {Triangle} triangle
     */
    removeTriangle(triangle) {
        const index = this.triangles.indexOf(triangle);
        if (index !== -1) {
            this.triangles.splice(index, 1);
        }
    }

    /**
    * 检查点是否在三角形外接圆内
     * @param {{ x: any; y: any;}} a
     * @param {{ x: any; y: any;}} b
     * @param {{ x: any; y: any;}} c
     * @param {{ x: any; y: any;}} d
     */
    inCircumcircle(a, b, c, d) {
        const orient =
        (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
        const ax = a.x, ay = a.y;
        const bx = b.x, by = b.y;
        const cx = c.x, cy = c.y;
        const dx = d.x, dy = d.y;
        
        const adx = ax - dx;
        const ady = ay - dy;
        const bdx = bx - dx;
        const bdy = by - dy;
        const cdx = cx - dx;
        const cdy = cy - dy;
        
        const abdet = adx * bdy - bdx * ady;
        const bcdet = bdx * cdy - cdx * bdy;
        const cadet = cdx * ady - adx * cdy;
        const alift = adx * adx + ady * ady;
        const blift = bdx * bdx + bdy * bdy;
        const clift = cdx * cdx + cdy * cdy;
        
        const det = alift * bcdet + blift * cadet + clift * abdet;
        
        return orient > 0 ? det > 0 : det < 0;
    }
}
/**
 * 三角形类
 */
class Triangle {
    /**
     * @param {number} a
     * @param {number} b
     * @param {number} c
     */
    constructor(a, b, c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /**
    * 返回三角形的三条边
     * @returns {number[][]}
     */
    edges() {
        return [
            [this.a, this.b],
            [this.b, this.c],
            [this.c, this.a]
        ];
    }

    /**
    * 检查是否包含某条边
     * @param {number[]} edge
     * @returns {boolean}
     */
    hasEdge(edge) {
        const [e1, e2] = edge;
        return (this.a === e1 && this.b === e2) ||
            (this.b === e1 && this.c === e2) ||
            (this.c === e1 && this.a === e2) ||
            (this.a === e2 && this.b === e1) ||
            (this.b === e2 && this.c === e1) ||
            (this.c === e2 && this.a === e1);
    }

    /**
    * 检查点是否在三角形内
     * @param {Vector} point
     * @param {Vector[]} vertices
     * @returns {boolean}
     */
    containsPoint(point, vertices) {
        const va = vertices[this.a];
        const vb = vertices[this.b];
        const vc = vertices[this.c];

        return pointInTri(point, va, vb, vc);
    }

    /**
    * 找到边对面的顶点
     * @param {number} v1
     * @param {number} v2
     * @returns {number}
     */
    oppositeVertex(v1, v2) {
        if (this.a !== v1 && this.a !== v2) return this.a;
        if (this.b !== v1 && this.b !== v2) return this.b;
        if (this.c !== v1 && this.c !== v2) return this.c;
        return -1;
    }
    /**
    * 计算点在三角形平面上的插值高度
    * @param {number} x 点的 x 坐标
    * @param {number} y 点的 y 坐标
     * @param {Vector[]} vertices
    * @returns {number} 插值高度
     */
    interpolateHeight(x, y, vertices) {
        const va = vertices[this.a];
        const vb = vertices[this.b];
        const vc = vertices[this.c];
        
        // 使用重心坐标插值
        const denom = (vb.y - vc.y) * (va.x - vc.x) + (vc.x - vb.x) * (va.y - vc.y);
        
        if (Math.abs(denom) < 1e-6) {
            // 三角形退化时，返回三个顶点高度平均值
            return (va.z + vb.z + vc.z) / 3;
        }
        
        const u = ((vb.y - vc.y) * (x - vc.x) + (vc.x - vb.x) * (y - vc.y)) / denom;
        const v = ((vc.y - va.y) * (x - vc.x) + (va.x - vc.x) * (y - vc.y)) / denom;
        const w = 1 - u - v;
        
        // 插值高度
        return u * va.z + v * vb.z + w * vc.z;
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
class JumpLinkBuilder
{
    /**
    * @param {NavMeshMesh} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        // 待更新：Max Jump Down Dist: 157
        this.jumpDist = 64;
        this.jumpHeight = MAX_JUMP_HEIGHT*MESH_CELL_SIZE_Z;
        this.walkHeight = MAX_WALK_HEIGHT*MESH_CELL_SIZE_Z;
        this.agentHeight = AGENT_HEIGHT * MESH_CELL_SIZE_Z;
        this.linkdist=250;// 可行走区域 A 与可行走区域 B 之间跳点最小间距
        /**@type {NavMeshLink[]}*/
        this.links = [];
        // 存储每个多边形所属的连通区域 ID
        /**@type {number[] | Int32Array<ArrayBuffer>}*/
        this.islandIds=[];
    }
    /**
     * 收集所有边界边
     */
    collectBoundaryEdges() {
        const edges = [];
        const { polys, verts, neighbors } = this.mesh;

        for (let i = 0; i < polys.length; i++) {
            const poly = polys[i];
            for (let j = 0; j < poly.length; j++) {
                // 如果没有邻居，就是边界边
                const neighList = neighbors[i][j];
                if (neighList.length === 0) {
                    const v1 = verts[poly[j]];
                    const v2 = verts[poly[(j + 1) % poly.length]];
                    edges.push({
                        polyIndex: i,
                        p1: v1,
                        p2: v2
                    });
                    //Instance.DebugLine({start:v1,end:v2,duration:60,color:{r:255,g:0,b:0}});
                }
            }
        }
        return edges;
    }
    /**
     * 判断两个多边形是否已经是物理邻居
     * @param {number} idxA
     * @param {number} idxB
     */
    areNeighbors(idxA, idxB) {
        const edgeList = this.mesh.neighbors[idxA] || [];
        for (const entry of edgeList) {
            if (entry.includes(idxB)) return true;
        }
        return false;
    }
    /**
     * @param {Vector} p1
     * @param {Vector} p2
     * @param {Vector} p3
     * @param {Vector} p4
     */
    closestPtSegmentSegment(p1, p2, p3, p4) {
        // 算法来源：Real-Time Collision Detection (Graham Walsh)
        // 计算线段 S1(p1,p2) 与 S2(p3,p4) 之间最近点
        
        const d1 = { x: p2.x - p1.x, y: p2.y - p1.y}; // 忽略 Z 参与平面距离计算
        const d2 = { x: p4.x - p3.x, y: p4.y - p3.y};
        const r = { x: p1.x - p3.x, y: p1.y - p3.y};

        const a = d1.x * d1.x + d1.y * d1.y; // Squared length of segment S1
        const e = d2.x * d2.x + d2.y * d2.y; // Squared length of segment S2
        const f = d2.x * r.x + d2.y * r.y;

        const EPSILON = 1e-6;

        // 检查线段是否退化成点
        if (a <= EPSILON && e <= EPSILON) {
            // 两个都是点
            return { dist: vec.length(p1, p3), ptA: p1, ptB: p3 };
        }
        
        let s, t;
        if (a <= EPSILON) {
            // S1 是点
            s = 0.0;
            t = f / e;
            t = Math.max(0.0, Math.min(1.0, t));
        } else {
            const c = d1.x * r.x + d1.y * r.y;
            if (e <= EPSILON) {
                // S2 是点
                t = 0.0;
                s = Math.max(0.0, Math.min(1.0, -c / a));
            } else {
                // 常规情况：两条线段
                const b = d1.x * d2.x + d1.y * d2.y;
                const denom = a * e - b * b;

                if (denom !== 0.0) {
                    s = Math.max(0.0, Math.min(1.0, (b * f - c * e) / denom));
                } else {
                    // 平行
                    s = 0.0;
                }

                t = (b * s + f) / e;

                if (t < 0.0) {
                    t = 0.0;
                    s = Math.max(0.0, Math.min(1.0, -c / a));
                } else if (t > 1.0) {
                    t = 1.0;
                    s = Math.max(0.0, Math.min(1.0, (b - c) / a));
                }
            }
        }

        // 计算最近点坐标（包含 Z）
        // 注意：t 和 s 在 XY 平面求得，再应用到 3D 坐标
        const ptA = {
            x: p1.x + (p2.x - p1.x) * s,
            y: p1.y + (p2.y - p1.y) * s,
            z: p1.z + (p2.z - p1.z) * s
        };

        const ptB = {
            x: p3.x + (p4.x - p3.x) * t,
            y: p3.y + (p4.y - p3.y) * t,
            z: p3.z + (p4.z - p3.z) * t
        };

        return {
            dist: vec.length(ptA, ptB),
            ptA,
            ptB
        };
    }
    //mesh内所有连接，根据constructor给的mesh
    init() {
        // 1) 设置日志标签（用于区分构建类型）。
        const logTag = "JumpLink";
        // 2) 构建“已存在 poly 对”集合；普通模式下传空数组，表示不做外部去重。
        const existingPairSet = this._buildExistingPairSet([]);

        // 3) 计算 mesh 连通分量（islandIds），后续用于“同岛且高度可走”过滤。
        this.buildConnectivity();
        // 4) 收集边界边（只在边界边之间寻找 jump 候选）。
        const boundaryEdges = this.collectBoundaryEdges();
        // 5) 为边界边建立空间网格索引，加速近邻边查询。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges);
        // 6) 收集候选并执行首轮筛选，得到每个 poly 对的最优候选。
        const { bestJumpPerPoly, pairChecks, nearChecks } = this._collectBestJumpCandidates(boundaryEdges, edgeGrid);
        // 7) 对候选做收尾去重（pair 去重 + 岛对近距去重），并生成最终 links。
        const finalLinks = this._finalizeJumpLinks(bestJumpPerPoly, existingPairSet);

        // 8) 回写结果并输出统计日志。
        this.links = finalLinks;
        Instance.Msg(`${logTag}统计: 边=${boundaryEdges.length} pair=${pairChecks} near=${nearChecks} link=${this.links.length}`);
        // 9) 返回构建完成的 links。
        return this.links;
    }

    /**
     * 仅构建跨 tile 的 jump link。
     * @param {(string)[]} polyTileKeys
     * @param {NavMeshLink[]} existingLinks
     */
    initInterTile(polyTileKeys, existingLinks) {
        // 1) 读取/兜底 tile 键映射（polyIndex -> tileId），用于“仅跨 tile”筛选。不存在则直接跳过
        const tileKeys = polyTileKeys;
        // 2) 设置日志标签。
        const logTag = "InterTileJumpLink";

        // 3) 构建“已存在 poly 对”集合，避免与外部已有 links 重复。
        const existingPairSet = this._buildExistingPairSet(existingLinks);
        // 4) 计算 mesh 连通分量。
        this.buildConnectivity();
        // 5) 收集边界边。
        const boundaryEdges = this.collectBoundaryEdges();
        // 6) 建立边界边空间索引。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges);
        // 7) 收集候选并筛选：额外过滤“同 tile”pair，只保留跨 tile 候选。
        const { bestJumpPerPoly, pairChecks, nearChecks } = this._collectBestJumpCandidates(
            boundaryEdges,
            edgeGrid,
            (edgeA, edgeB) => {
                const tileA = tileKeys[edgeA.polyIndex];
                const tileB = tileKeys[edgeB.polyIndex];
                return !tileA || !tileB || tileA === tileB;
            }
        );
        // 8) 对候选做收尾去重并生成最终 links。
        const finalLinks = this._finalizeJumpLinks(bestJumpPerPoly, existingPairSet);

        // 9) 回写结果并输出统计日志。
        this.links = finalLinks;
        Instance.Msg(`${logTag}统计: 边=${boundaryEdges.length} pair=${pairChecks} near=${nearChecks} link=${this.links.length}`);
        // 10) 返回构建完成的 links。
        return this.links;
    }

    /**
     * @param {NavMeshLink[]} existingLinks
     */
    _buildExistingPairSet(existingLinks) {
        const pairSet = new Set();
        for (const link of existingLinks || []) {
            if (link.PolyA < 0 || link.PolyB < 0) continue;
            pairSet.add(Tool.orderedPairKey(link.PolyA, link.PolyB, "_"));
        }
        return pairSet;
    }

    /**
     * @param {{polyIndex:number,p1:Vector,p2:Vector}[]} boundaryEdges
     * @param {{grid:Map<string, number[]>, metas:{minX:number,maxX:number,minY:number,maxY:number}[], cellSize:number}} edgeGrid
     * @param {(edgeA:{polyIndex:number,p1:Vector,p2:Vector},edgeB:{polyIndex:number,p1:Vector,p2:Vector}) => boolean} [shouldSkipPair]
     */
    _collectBestJumpCandidates(boundaryEdges, edgeGrid, shouldSkipPair) {
        // Key: "polyA_polyB", Value: { targetPoly, dist, startPos, endPos }
        const bestJumpPerPoly = new Map();
        let pairChecks = 0;
        let nearChecks = 0;

        for (let i = 0; i < boundaryEdges.length; i++) {
            const candidateIndices = this.queryNearbyEdges(edgeGrid, i, this.jumpDist);
            for (const j of candidateIndices) {
                if (j <= i) continue;
                const edgeA = boundaryEdges[i];
                const edgeB = boundaryEdges[j];
                pairChecks++;

                if (edgeA.polyIndex === edgeB.polyIndex) continue;
                if (shouldSkipPair && shouldSkipPair(edgeA, edgeB)) continue;

                const minBoxDist = this.bboxMinDist2D(edgeGrid.metas[i], edgeGrid.metas[j]);
                if (minBoxDist > this.jumpDist) continue;
                nearChecks++;

                const closestResult = this.closestPtSegmentSegment(edgeA.p1, edgeA.p2, edgeB.p1, edgeB.p2);
                if (!closestResult) continue;

                const { dist, ptA, ptB } = closestResult;
                if (!ptA || !ptB) continue;
                if (vec.length2D(ptA, ptB) > this.jumpDist) continue;

                if (this.islandIds[edgeA.polyIndex] === this.islandIds[edgeB.polyIndex] && Math.abs(ptA.z - ptB.z) <= this.walkHeight) continue;
                const heightDiff = Math.abs(ptA.z - ptB.z);
                if (heightDiff > this.jumpHeight) continue;
                if (heightDiff < 1 && dist < 1) continue;
                if (!this.validateJumpPath(ptA, ptB)) continue;

                this.updateBestCandidate(bestJumpPerPoly, edgeA.polyIndex, edgeB.polyIndex, dist, ptA, ptB);
            }
        }

        return { bestJumpPerPoly, pairChecks, nearChecks };
    }

    /**
     * @param {Map<string,any>} bestJumpPerPoly
     * @param {Set<string>} existingPairSet
     */
    _finalizeJumpLinks(bestJumpPerPoly, existingPairSet) {
        const sortedCandidates = Array.from(bestJumpPerPoly.values()).sort((a, b) => a.dist - b.dist);

        const finalLinks = [];
        for (const cand of sortedCandidates) {
            const pairKey = Tool.orderedPairKey(cand.startPoly, cand.endPoly, "_");
            if (existingPairSet.has(pairKey)) continue;

            const islandA = this.islandIds[cand.startPoly];
            const islandB = this.islandIds[cand.endPoly];

            let tooClose = false;
            for (const existing of finalLinks) {
                const exIslandA = this.islandIds[existing.PolyA];
                const exIslandB = this.islandIds[existing.PolyB];

                if ((islandA === exIslandA && islandB === exIslandB)
                    || (islandA === exIslandB && islandB === exIslandA)) {
                    const dSqStart = vec.length(cand.startPos, existing.PosA);
                    const dSqEnd = vec.length(cand.endPos, existing.PosB);

                    if (dSqStart < this.linkdist || dSqEnd < this.linkdist) {
                        tooClose = true;
                        break;
                    }
                }
            }

            if (!tooClose) {
                finalLinks.push({
                    PolyA: cand.startPoly,
                    PolyB: cand.endPoly,
                    PosA: cand.startPos,
                    PosB: cand.endPos,
                    cost: cand.dist * 1.5,
                    type: (Math.abs(cand.startPos.z - cand.endPos.z) <= this.walkHeight ? PathState.WALK : PathState.JUMP)
                });
                existingPairSet.add(pairKey);
            }
        }

        return finalLinks;
    }
    /**
     * 计算多边形网格的连通分量
     * 给互相连通的多边形打上相同标识
     */
    buildConnectivity() {
        const numPolys = this.mesh.polys.length;
        this.islandIds = new Int32Array(numPolys).fill(-1);
        let currentId = 0;

        for (let i = 0; i < numPolys; i++) {
            if (this.islandIds[i] !== -1) continue;

            currentId++;
            const stack = [i];
            this.islandIds[i] = currentId;

            while (stack.length > 0) {
                const u = stack.pop();
                // 遍历该多边形所有邻居
                if (u === undefined) break;
                const neighbors = this.mesh.neighbors[u] || [];
                for (const entry of neighbors) {
                    for (const v of entry) {
                        if (v >= 0 && this.islandIds[v] === -1) {
                            this.islandIds[v] = currentId;
                            stack.push(v);
                        }
                    }
                }
            }
        }
        Instance.Msg(`共有${currentId}个独立行走区域`);
    }

    /**
     * @param {{polyIndex:number,p1:Vector,p2:Vector}[]} edges
     */
    buildEdgeGrid(edges) {
        const cellSize = Math.max(this.jumpDist, MESH_CELL_SIZE_XY * 4);
        /** @type {Map<string, number[]>} */
        const grid = new Map();
        /** @type {{minX:number,maxX:number,minY:number,maxY:number}[]} */
        const metas = new Array(edges.length);

        for (let i = 0; i < edges.length; i++) {
            const e = edges[i];
            const minX = Math.min(e.p1.x, e.p2.x);
            const maxX = Math.max(e.p1.x, e.p2.x);
            const minY = Math.min(e.p1.y, e.p2.y);
            const maxY = Math.max(e.p1.y, e.p2.y);
            metas[i] = { minX, maxX, minY, maxY };

            const x0 = Math.floor(minX / cellSize);
            const x1 = Math.floor(maxX / cellSize);
            const y0 = Math.floor(minY / cellSize);
            const y1 = Math.floor(maxY / cellSize);

            for (let x = x0; x <= x1; x++) {
                for (let y = y0; y <= y1; y++) {
                    const k = Tool.gridKey2(x, y);
                    Tool.getOrCreateArray(grid, k).push(i);
                }
            }
        }

        return { grid, metas, cellSize };
    }

    /**
     * @param {{grid:Map<string, number[]>, metas:{minX:number,maxX:number,minY:number,maxY:number}[], cellSize:number}} edgeGrid
     * @param {number} edgeIndex
     * @param {number} expand
     */
    queryNearbyEdges(edgeGrid, edgeIndex, expand) {
        const m = edgeGrid.metas[edgeIndex];
        const x0 = Math.floor((m.minX - expand) / edgeGrid.cellSize);
        const x1 = Math.floor((m.maxX + expand) / edgeGrid.cellSize);
        const y0 = Math.floor((m.minY - expand) / edgeGrid.cellSize);
        const y1 = Math.floor((m.maxY + expand) / edgeGrid.cellSize);

        const ans = [];
        const seen = new Set();
        for (let x = x0; x <= x1; x++) {
            for (let y = y0; y <= y1; y++) {
                const k = Tool.gridKey2(x, y);
                const list = edgeGrid.grid.get(k);
                if (!list) continue;
                for (const idx of list) {
                    if (seen.has(idx)) continue;
                    seen.add(idx);
                    ans.push(idx);
                }
            }
        }
        return ans;
    }

    /**
     * @param {{minX:number,maxX:number,minY:number,maxY:number}} a
     * @param {{minX:number,maxX:number,minY:number,maxY:number}} b
     */
    bboxMinDist2D(a, b) {
        const dx = Math.max(0, Math.max(a.minX, b.minX) - Math.min(a.maxX, b.maxX));
        const dy = Math.max(0, Math.max(a.minY, b.minY) - Math.min(a.maxY, b.maxY));
        return Math.hypot(dx, dy);
    }

    /**
     * @param {Vector} a
     * @param {Vector} b
     */
    validateJumpPath(a, b) {
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        if (Math.hypot(dx, dy) < 1e-4) return false;
        const z=Math.max(a.z, b.z)+8;

        const start = { x: a.x, y: a.y, z: z };
        const end = { x: b.x, y: b.y, z: z };

        const boxMins = { x: -1, y: -1, z: 0 };
        const boxMaxs = { x: 1, y: 1, z: 52 };

        const hit = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start,
            end,
            ignorePlayers: true
        });
        if (hit && hit.didHit) return false;

        const hitReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start: end,
            end: start,
            ignorePlayers: true
        });
        if (hitReverse && hitReverse.didHit) return false;

        return true;
    }

    /**
     * @param {Vector} p
     */
    hasStandClearance(p) {
        const floorCheck = Instance.TraceLine({
            start: { x: p.x, y: p.y, z: p.z + 8 },
            end: { x: p.x, y: p.y, z: p.z - 32 },
            ignorePlayers: true,
        });
        if (!floorCheck || !floorCheck.didHit) return false;

        const upCheck = Instance.TraceLine({
            start: { x: p.x, y: p.y, z: p.z + 4 },
            end: { x: p.x, y: p.y, z: p.z + Math.max(8, this.agentHeight - 4) },
            ignorePlayers: true,
        });
        if (upCheck && upCheck.didHit) return false;

        return true;
    }
    /**
     * @param {Map<string,any>} map
     * @param {number} idxA
     * @param {number} idxB
    * @param {number} dist 两个多边形边界边之间的最短距离
     * @param {Vector} ptA
     * @param {Vector} ptB
     */
    updateBestCandidate(map, idxA, idxB, dist, ptA, ptB) {
        // 检查是否已记录过该多边形对的跳跃目标
        const key = Tool.orderedPairKey(idxA, idxB, "_");

        const existing = map.get(key);
        // 若未记录或发现更近目标，则更新
        if (!existing || dist < existing.dist) {
            map.set(key, {
                startPoly: idxA,
                endPoly: idxB,
                dist: dist,
                startPos: { ...ptA },
                endPos: { ...ptB }
            });
        }
    }
    debugDraw(duration = 10) {
        for (const link of this.links) {
            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: { r: 0, g: (link.type==1?255:0), b: 255 },
                duration
            });
            //Instance.DebugSphere({ center: link.PosA, radius: 4, color: { r: 0, g: 255, b: 0 }, duration });
            //Instance.DebugSphere({ center: link.endPos, radius: 4, color: { r: 255, g: 0, b: 0 }, duration });
            let poly = this.mesh.polys[link.PolyB];
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({start,end,color:{ r: 255, g: 0, b: 255 },duration});
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
            poly = this.mesh.polys[link.PolyA];
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({start,end,color:{ r: 255, g: 0, b: 255 },duration});
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
        }
    }
}

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */

/**
 * 单 tile 构建器：
 * - 只负责当前 tile 内构建（体素化/区域/轮廓/多边形/细节/link）
 * - 返回 tile 数据，由 TileManager 负责 tile 间聚合与裁剪
 */
class tile {
    constructor() {
        /** @type {OpenHeightfield | undefined} */
        this.hf = undefined;
        /** @type {RegionGenerator | undefined} */
        this.regionGen = undefined;
        /** @type {ContourBuilder | undefined} */
        this.contourBuilder = undefined;
        /** @type {PolyMeshBuilder | undefined} */
        this.polyMeshGenerator = undefined;
        /** @type {PolyMeshDetailBuilder | undefined} */
        this.polidetail = undefined;
        /** @type {JumpLinkBuilder | undefined} */
        this.jumplinkbuilder = undefined;
        //边界体素
        this.tilePadding = Math.max(0, TILE_PADDING | 0);
        //tile大小,不含padding
        this.tileSize = Math.max(1, TILE_SIZE | 0);
        //一边有几个体素
        this.fullGrid = Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        this.tilesX = Math.ceil(this.fullGrid / this.tileSize);
        this.tilesY = Math.ceil(this.fullGrid / this.tileSize);
    }

    /**
     * 仅构建给定世界坐标所在的 tile。
     * @param {{x:number,y:number,z:number}} pos
     */
    buildTileNavMeshAtPos(pos) {
        const gx = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.x - origin.x) / MESH_CELL_SIZE_XY)));
        const gy = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.y - origin.y) / MESH_CELL_SIZE_XY)));
        const tx = Math.max(0, Math.min(this.tilesX - 1, Math.floor(gx / this.tileSize)));
        const ty = Math.max(0, Math.min(this.tilesY - 1, Math.floor(gy / this.tileSize)));
        return this.buildTile(tx, ty);
    }

    /**
     * @param {number} tx
     * @param {number} ty
     */
    buildTile(tx, ty) {
        const nowMs = () => new Date().getTime();
        const timing = {hfInit: 0,region: 0,contour: 0,poly: 0,detail: 0,merge: 0,jumpLinks: 0,};

        let tileHasError = false;
        const tileStartMs = nowMs();
        Instance.Msg(`开始构建 Tile (${tx+1}/${this.tilesX},${ty+1}/${this.tilesY})`);
        let phaseStartMs = nowMs();

        this.hf = new OpenHeightfield(tx, ty, this.tileSize, this.fullGrid, this.tilePadding);
        this.hf.init();
        timing.hfInit += nowMs() - phaseStartMs;
        phaseStartMs = nowMs();

        this.regionGen = new RegionGenerator(this.hf);
        this.regionGen.init();
        timing.region += nowMs() - phaseStartMs;
        phaseStartMs = nowMs();

        this.contourBuilder = new ContourBuilder(this.hf);
        this.contourBuilder.init();

        if (this.contourBuilder.error) tileHasError = true;
        timing.contour += nowMs() - phaseStartMs;
        phaseStartMs = nowMs();

        this.polyMeshGenerator = new PolyMeshBuilder(this.contourBuilder.contours);
        this.polyMeshGenerator.init();

        const tileMesh = this.polyMeshGenerator.return();
        if (this.polyMeshGenerator.error) tileHasError = true;
        timing.poly += nowMs() - phaseStartMs;

        /** @type {NavMeshDetail} */
        let tileDetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        if (tileMesh.polys.length > 0) {
            phaseStartMs = nowMs();

            this.polidetail = new PolyMeshDetailBuilder(tileMesh, this.hf);
            tileDetail = this.polidetail.init();

            if (this.polidetail.error) tileHasError = true;
            timing.detail += nowMs() - phaseStartMs;
        }

        /**
         * @type {NavMeshLink[]}
         */
        let tileLinks = [];
        if (tileMesh.polys.length > 0) {
            phaseStartMs = nowMs();
            
            this.jumplinkbuilder = new JumpLinkBuilder(tileMesh);
            tileLinks = this.jumplinkbuilder.init();

            timing.jumpLinks += nowMs() - phaseStartMs;
        }

        OpenSpan.clearRange(1, this.hf.SPAN_ID + 2);
        const tileCostMs = nowMs() - tileStartMs;
        Instance.Msg(`完成 Tile (${tx+1}/${this.tilesX},${ty+1}/${this.tilesY}),耗时${tileCostMs}ms`);

        return {tileId: `${tx}_${ty}`,tx,ty,mesh: tileMesh,detail: tileDetail,links: tileLinks,hasError: tileHasError,timing};
    }

    /**
     * @param {{tx:number,ty:number}[]} tiles
     * @param {number} duration
     */
    debugDrawErrorTiles(tiles, duration = 120) {
        if (!tiles || tiles.length === 0) return;
        const color = { r: 255, g: 255, b: 255 };

        for (const tile of tiles) {
            const coreMinX = tile.tx * this.tileSize;
            const coreMinY = tile.ty * this.tileSize;
            const coreMaxX = Math.min(this.fullGrid - 1, coreMinX + this.tileSize - 1);
            const coreMaxY = Math.min(this.fullGrid - 1, coreMinY + this.tileSize - 1);

            const minX = origin.x + coreMinX * MESH_CELL_SIZE_XY;
            const minY = origin.y + coreMinY * MESH_CELL_SIZE_XY;
            const maxX = origin.x + (coreMaxX + 1) * MESH_CELL_SIZE_XY;
            const maxY = origin.y + (coreMaxY + 1) * MESH_CELL_SIZE_XY;

            const z0 = origin.z + 8;
            const z1 = origin.z + 500;

            const a0 = { x: minX, y: minY, z: z0 };
            const b0 = { x: maxX, y: minY, z: z0 };
            const c0 = { x: maxX, y: maxY, z: z0 };
            const d0 = { x: minX, y: maxY, z: z0 };
            const a1 = { x: minX, y: minY, z: z1 };
            const b1 = { x: maxX, y: minY, z: z1 };
            const c1 = { x: maxX, y: maxY, z: z1 };
            const d1 = { x: minX, y: maxY, z: z1 };

            Instance.DebugLine({ start: a0, end: b0, color, duration });
            Instance.DebugLine({ start: b0, end: c0, color, duration });
            Instance.DebugLine({ start: c0, end: d0, color, duration });
            Instance.DebugLine({ start: d0, end: a0, color, duration });

            Instance.DebugLine({ start: a1, end: b1, color, duration });
            Instance.DebugLine({ start: b1, end: c1, color, duration });
            Instance.DebugLine({ start: c1, end: d1, color, duration });
            Instance.DebugLine({ start: d1, end: a1, color, duration });

            Instance.DebugLine({ start: a0, end: a1, color, duration });
            Instance.DebugLine({ start: b0, end: b1, color, duration });
            Instance.DebugLine({ start: c0, end: c1, color, duration });
            Instance.DebugLine({ start: d0, end: d1, color, duration });

            Instance.DebugLine({ start: a1, end: c1, color, duration });
            Instance.DebugLine({ start: b1, end: d1, color, duration });
        }
    }

}

/**
 * NavMesh 调试工具集合。
 *
 * 分层说明：
 * - MESH：体素/高度场层
 * - REGION：区域分割层
 * - CONTOUR：轮廓层
 * - POLY：主多边形层
 * - DETAIL：细节三角网层
 * - LINK：跳点连接层
 * - PATH：寻路结果层（poly path / funnel path / final path）
 */
class NavMeshDebugTools {
    /**
     * @param {import("./path_manager").NavMesh} nav
     */
    constructor(nav) {
        /** @type {import("./path_manager").NavMesh} */
        this.nav = nav;
        /** @type {number[]} */
        this._polyAreas = [];
        /** @type {number[]} */
        this._polyPrefix = [];
        /** @type {number} */
        this._totalPolyArea = 0;
    }
    /**
        * 绘制最终细节网格（三角形）。
        * 对应调试部分：DETAIL。
        * 优先使用最终 meshdetail（支持 tile merge 后），无数据时回退到 polidetail 临时数据。
        *
     * 基于最终 this.nav.meshdetail 画细节三角形（兼容 tile merge 后结果）
     * @param {number} [duration]
     */
    debugDrawMeshDetail(duration = 10) {
        const detail = this.nav.meshdetail;
        if (detail?.verts && detail?.tris && detail.tris.length > 0) {
            for (let i = 0; i < detail.tris.length; i++) {
                const tri = detail.tris[i];
                if (!tri || tri.length < 3) continue;
                const a = detail.verts[tri[0]];
                const b = detail.verts[tri[1]];
                const c = detail.verts[tri[2]];
                if (!a || !b || !c) continue;
                const color = { r: 0, g: 180, b: 255 };
                Instance.DebugLine({ start: a, end: b, color, duration });
                Instance.DebugLine({ start: b, end: c, color, duration });
                Instance.DebugLine({ start: c, end: a, color, duration });
            }
            return;
        }
    }
    debugLinks(duration = 30) {
        for (const link of this.nav.links) {
            const isJump = link.type === PathState.JUMP;
            const isLadder = link.type === PathState.LADDER;
            const lineColor = isLadder
                ? { r: 255, g: 165, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 0, b: 255 });
            const startColor = isLadder
                ? { r: 255, g: 215, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 255, b: 0 });

            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: lineColor,
                duration
            });
            Instance.DebugSphere({ center: link.PosA, radius: 4, color: startColor, duration });
            const poly = this.nav.mesh.polys[link.PolyB];
            for (let i = 0; i < poly.length; i++) {
                const start = this.nav.mesh.verts[poly[i]];
                const end = this.nav.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({ start, end, color: isLadder ? { r: 255, g: 140, b: 0 } : { r: 255, g: 0, b: 255 }, duration });
            }
        }
    }
    /**
     * 绘制最终主多边形网格边线（不含 links）。
     * @param {number} duration
     */
    debugDrawMeshPolys(duration = 10) {
        if (!this.nav.mesh) return;
        for (let pi = 0; pi < this.nav.mesh.polys.length; pi++) {
            const poly = this.nav.mesh.polys[pi];
            const color = { r: 255, g: 0, b: 0 };
            for (let i = 0; i < poly.length; i++) {
                const start = this.nav.mesh.verts[poly[i]];
                const end = this.nav.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({ start, end, color, duration });
            }
        }
    }

    /**
        * 绘制最终多边形邻接关系（中心点连线）。
        * 对应调试部分：POLY 邻接/跨 tile 连接检查。
        *
     * 直接基于最终 this.nav.mesh.neighbors 画多边形连接关系
     * @param {number} [duration]
     */
    debugDrawMeshConnectivity(duration = 15) {
        if (!this.nav.mesh) return;
        const mesh = this.nav.mesh;
        const drawn = new Set();
        for (let i = 0; i < mesh.polys.length; i++) {
            const start = this._meshPolyCenter(i);
            const neighbors = mesh.neighbors?.[i] || [];
            for (let e = 0; e < neighbors.length; e++) {
                const edgeNei = neighbors[e];
                const neiList = Array.isArray(edgeNei)
                    ? edgeNei
                    : (typeof edgeNei === "number" && edgeNei >= 0 ? [edgeNei] : []);

                for (const ni of neiList) {
                    if (ni < 0) continue;
                    const a = Math.min(i, ni);
                    const b = Math.max(i, ni);
                    const k = `${a}|${b}`;
                    if (drawn.has(k)) continue;
                    drawn.add(k);

                    const end = this._meshPolyCenter(ni);
                    Instance.DebugLine({
                        start,
                        end,
                        color: { r: 255, g: 0, b: 255 },
                        duration
                    });
                }
            }
        }
    }

    /**
        * 计算指定多边形中心点。
        * 对应调试部分：内部工具（给邻接线、路径可视化复用）。
        *
     * @param {number} polyIndex
     */
    _meshPolyCenter(polyIndex) {
        const poly = this.nav.mesh.polys[polyIndex];
        let x = 0;
        let y = 0;
        let z = 0;
        for (const vi of poly) {
            const v = this.nav.mesh.verts[vi];
            x += v.x;
            y += v.y;
            z += v.z;
        }
        const n = poly.length || 1;
        return { x: x / n, y: y / n, z: z / n };
    }

    /**
        * 绘制 Funnel 拉直后的路径点序列。
        * 对应调试部分：PATH（funnel 阶段）。
        *
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     * @param {number} [duration]
     */
    debugDrawfunnelPath(path, duration = 10) {
        if (!path || path.length < 2) {
            Instance.Msg("No path to draw");
            return;
        }
        const color = { r: 0, g: 255, b: 0 };
        const colorJ = { r: 0, g: 255, b: 255 };

        const last = path[0].pos;
        Instance.DebugSphere({ center: { x: last.x, y: last.y, z: last.z }, radius: 3, color: { r: 255, g: 0, b: 0 }, duration });
        for (let i = 1; i < path.length; i++) {
            const a = path[i - 1].pos;
            const b = path[i].pos;
            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color: path[i].mode == PathState.WALK ? color:colorJ,
                duration
            });
            Instance.DebugSphere({ center: { x: b.x, y: b.y, z: b.z }, radius: 3, color: path[i].mode == 2 ? colorJ : color, duration });
        }
    }

    /**
        * 绘制最终输出路径（含跳跃段颜色区分）。
        * 对应调试部分：PATH（最终路径）。
        *
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     * @param {number} [duration]
     */
    debugDrawPath(path, duration = 10) {
        const color = { r: 0, g: 0, b: 255 };
        const colorJ = { r: 255, g: 255, b: 0 };
        if (!path || path.length == 2) {
            if (path && path.length == 2) {
                Instance.DebugSphere({ center: { x: path[0].pos.x, y: path[0].pos.y, z: path[0].pos.z }, radius: 3, color: { r: 0, g: 0, b: 255 }, duration });
                Instance.DebugLine({
                    start: { x: path[0].pos.x, y: path[0].pos.y, z: path[0].pos.z },
                    end: { x: path[1].pos.x, y: path[1].pos.y, z: path[1].pos.z },
                    color: path[1].mode == PathState.WALK ? color:colorJ,
                    duration
                });
                Instance.DebugSphere({ center: { x: path[1].pos.x, y: path[1].pos.y, z: path[1].pos.z }, radius: 3, color: path[1].mode == PathState.WALK ? color:colorJ, duration });
            } else Instance.Msg("No path to draw");
            return;
        }

        const last = path[0].pos;
        Instance.DebugSphere({ center: { x: last.x, y: last.y, z: last.z }, radius: 3, color: { r: 0, g: 0, b: 255 }, duration });
        for (let i = 1; i < path.length; i++) {
            const a = path[i - 1].pos;
            const b = path[i].pos;
            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color: path[i].mode == PathState.WALK ? color:colorJ,
                duration
            });
            Instance.DebugSphere({ center: { x: b.x, y: b.y, z: b.z }, radius: 3, color: path[i].mode == PathState.WALK ? color:colorJ, duration });
        }
    }

    /**
        * 绘制 A* 返回的多边形路径（按 poly 中心点串线）。
        * 对应调试部分：PATH（poly path 阶段）。
        *
     * @param {{id:number,mode:number}[]} polyPath
     * @param {number} [duration]
     */
    debugDrawPolyPath(polyPath, duration = 10) {
        if (!polyPath || polyPath.length === 0 || !this.nav.mesh) return;

        let prev = null;
        const color = {
            r: Math.floor(100 + Math.random() * 155),
            g: Math.floor(100 + Math.random() * 155),
            b: Math.floor(100 + Math.random() * 155),
        };
        const colorJ = {
            r: Math.floor(100 + Math.random() * 155),
            g: Math.floor(100 + Math.random() * 155),
            b: Math.floor(100 + Math.random() * 155),
        };
        for (const pi of polyPath) {
            Instance.Msg(pi.id);
            const poly = this.nav.mesh.polys[pi.id];
            let cx = 0, cy = 0, cz = 0;
            for (const vi of poly) {
                const v = this.nav.mesh.verts[vi];
                cx += v.x; cy += v.y; cz += v.z;
            }
            cx /= poly.length;
            cy /= poly.length;
            cz /= poly.length;

            const center = { x: cx, y: cy, z: cz };
            if (pi.mode == 2) {
                Instance.DebugSphere({ center, radius: 10, color: colorJ, duration });
                if (prev) Instance.DebugLine({ start: prev, end: center, color: colorJ, duration });
            } else {
                Instance.DebugSphere({ center, radius: 10, color, duration });
                if (prev) Instance.DebugLine({ start: prev, end: center, color, duration });
            }
            prev = center;
        }
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
//不多，可以每次都重新构建
class LadderLinkBuilder {
    /**
     * @param {NavMeshMesh} polyMesh
     * @param {NavMeshDetail} detailMesh
     */
    constructor(polyMesh,detailMesh) {
        this.mesh = polyMesh;
        /** @type {boolean} */
        this.error = false;
        /** @type {NavMeshLink[]} */
        this.links = [];
        this.heightfixer=new FunnelHeightFixer(this.mesh,detailMesh,ADJUST_HEIGHT_DISTANCE);
    }

    init() {
        this.error = false;
        this.links = [];
        Tool.buildSpatialIndex(this.mesh);
        if (!this.mesh || !this.mesh.polys || this.mesh.polys.length === 0) return this.links;

        /** @type {Map<string, Vector[]>} */
        const groups = new Map();
        const ents = Instance.FindEntitiesByClass("info_target");

        for (const ent of ents) {
            const name = ent.GetEntityName();
            if (!name.startsWith("LADDER_")) continue;

            const tag = name.slice("LADDER_".length);
            if (!tag) continue;

            const p = ent.GetAbsOrigin();
            if (!p) continue;

            if (!groups.has(tag)) groups.set(tag, []);
            groups.get(tag)?.push({ x: p.x, y: p.y, z: p.z });
        }

        let rawPairs = 0;
        let validPairs = 0;

        for (const [tag, points] of groups) {
            if (points.length < 2) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 点位不足(=${points.length})，已跳过`);
                continue;
            }
            if (points.length !== 2) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 点位数量过多(${points.length})，已跳过`);
                continue;
            }

            points.sort((a, b) => a.z - b.z);
            const aPos = points[0];
            const bPos = points[points.length - 1];
            rawPairs++;

            const aNearest = Tool.findNearestPoly(aPos, this.mesh,this.heightfixer);
            const bNearest = Tool.findNearestPoly(bPos, this.mesh,this.heightfixer);
            const aPoly = aNearest.poly;
            const bPoly = bNearest.poly;
            if (aPoly < 0 || bPoly < 0) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 找不到最近多边形，已跳过`);
                continue;
            }
            if (aPoly === bPoly) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 两端落在同一 poly(${aPoly})，已跳过`);
                continue;
            }
            const cost = Math.max(1, vec.length(aNearest.pos, bNearest.pos));
            this.links.push({
                PolyA: aPoly,
                PolyB: bPoly,
                PosA: aPos,
                PosB: bPos,
                cost,
                type: PathState.LADDER
            });
            validPairs++;
        }
        Instance.Msg(`LadderLink统计: group=${groups.size} pair=${rawPairs} link=${this.links.length} valid=${validPairs}`);
        return this.links;
    }

    /**
     * @param {number} duration
     */
    debugDraw(duration = 30) {
        for (const link of this.links) {
            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: { r: 255, g: 165, b: 0 },
                duration
            });
            Instance.DebugSphere({ center: link.PosA, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
            Instance.DebugSphere({ center: link.PosB, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
        }
    }
}

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("./path_tile").tile} tile */

/**
 * @typedef {{
 *  tileId:string,
 *  tx:number,
 *  ty:number,
 *  mesh:NavMeshMesh,
 *  detail:NavMeshDetail,
 *  links:NavMeshLink[]
 * }} TileData
 */

class TileManager {
    /**
     * 负责 tile 间聚合：
     * - 维护 tile 数据集合（add/remove/update）
     * - 重建跨 tile 邻接与最终 mesh/detail/links
     */
    constructor() {
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();

        /** @type {NavMeshMesh} */
        this.mesh = { verts: [], polys: [],regions:[], neighbors: [] };

        /** @type {NavMeshDetail} */
        this.meshdetail = { verts: [], tris: [], triTopoly: [], meshes: [] };

        /** @type {NavMeshLink[]} */
        this.links = [];
        /** @type {NavMeshLink[]} */
        this.supprlink=[];
        /** @type {NavMeshLink[]} */
        this.baseLinks=[];
        /** @type {Map<string, {vertBase:number,vertCount:number,polyBase:number,polyCount:number,detailVertBase:number,detailVertCount:number,triBase:number,triCount:number,meshRecBase:number,meshRecCount:number}>} */
        this.tileRanges = new Map();
    }

    /**
     * @param {string|number} tileId
     * @param {number} tx
     * @param {number} ty
    * @param {NavMeshMesh} tileMesh
    * @param {NavMeshDetail} tileDetail
    * @param {NavMeshLink[]} [tileLinks]
     */
    addtile(tileId, tx, ty, tileMesh, tileDetail, tileLinks = []) {
        const key = String(tileId);
        if (this.tiles.has(key)) {
            this.removetile(key);
        }
        this.tiles.set(key, {
            tileId: key,
            tx,
            ty,
            mesh: tileMesh,
            detail: tileDetail,
            links: tileLinks
        });
        this._appendTileData(key, tileMesh, tileDetail, tileLinks);
        this._rebuildDeferredLinks(key);
    }

    /**
     * @param {string|number} tileId
     */
    removetile(tileId) {
        const key = String(tileId);
        if (!this.tiles.has(key)) return;
        this.tiles.delete(key);
        this._removeTileData(key);
    }

    /**
     * @param {string} tileId
     * @param {number} tx
     * @param {number} ty
    * @param {NavMeshMesh} tileMesh
    * @param {NavMeshDetail} tileDetail
    * @param {NavMeshLink[]} tileLinks
     */
    updatetile(tileId, tx, ty, tileMesh, tileDetail, tileLinks) {
        const key = String(tileId);
        if (this.tiles.has(key)) {
            this.tiles.delete(key);
            this._removeTileData(key);
        }
        this.tiles.set(key, {
            tileId: key,
            tx,
            ty,
            mesh: tileMesh,
            detail: tileDetail,
            links: tileLinks
        });
        this._appendTileData(key, tileMesh, tileDetail, tileLinks);
        this._rebuildDeferredLinks(key);
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {NavMeshDetail} meshdetail
     */
    buildLadderLinksForMesh(mesh,meshdetail) {
        this.ladderlinkbuilder = new LadderLinkBuilder(mesh, meshdetail);
        return this.ladderlinkbuilder.init();
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {(string)[]} polyTileKeys
     * @param {NavMeshLink[]} existingLinks
     */
    buildInterTileJumpLinksForMesh(mesh, polyTileKeys, existingLinks) {
        const builder = new JumpLinkBuilder(mesh);
        return builder.initInterTile(polyTileKeys, existingLinks);
    }
    return() {
        return {
            mesh: this.mesh,
            meshdetail: this.meshdetail,
            links: this.links
        };
    }

    /**
     * 初始化，什么都没有
     * @param {tile} tileBuilder
     */
    rebuildAll(tileBuilder) {
        this.tiles.clear();
        this.tileRanges.clear();
        this.mesh = { verts: [], polys: [], regions: [], neighbors: [] };
        this.meshdetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        this.baseLinks = [];
        this.links = [];
        this.supprlink = [];

        const timing = {
            hfInit: 0,
            region: 0,
            contour: 0,
            poly: 0,
            detail: 0,
            merge: 0,
            jumpLinks: 0,
        };

        /** @type {{tx:number,ty:number}[]} */
        const errorTiles = [];

        for (let ty = 0; ty < tileBuilder.tilesY; ty++) {
            for (let tx = 0; tx < tileBuilder.tilesX; tx++) {
                const tileData = tileBuilder.buildTile(tx, ty);
                timing.hfInit += tileData.timing.hfInit;
                timing.region += tileData.timing.region;
                timing.contour += tileData.timing.contour;
                timing.poly += tileData.timing.poly;
                timing.detail += tileData.timing.detail;
                timing.merge += tileData.timing.merge;
                timing.jumpLinks += tileData.timing.jumpLinks;
                if (tileData.hasError) errorTiles.push({ tx, ty });
                const key = String(tileData.tileId);
                this.tiles.set(key, {
                    tileId: key,
                    tx: tileData.tx,
                    ty: tileData.ty,
                    mesh: tileData.mesh,
                    detail: tileData.detail,
                    links: tileData.links
                });
                this._appendTileData(key, tileData.mesh, tileData.detail, tileData.links);
            }
        }
        this._rebuildDeferredLinks();
        if (errorTiles.length > 0) {
            const dedup = new Map();
            for (const tile of errorTiles) dedup.set(`${tile.tx}|${tile.ty}`, tile);
            const drawTiles = Array.from(dedup.values());
            tileBuilder.debugDrawErrorTiles(drawTiles, 60);
            Instance.Msg(`Tile报错统计: ${drawTiles.length} 个tile存在步骤报错，已在地图高亮`);
        }
        this.pruneUnreachablePolys();
        Instance.Msg(`Tile阶段耗时统计: 体素化=${timing.hfInit}ms, 区域=${timing.region}ms, 轮廓=${timing.contour}ms, 多边形=${timing.poly}ms, 细节=${timing.detail}ms, 合并=${timing.merge}ms`);
        return { timing, errorTiles };
    }

    /**
     * @param {string} tileId
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink[]} tileLinks
     */
    _appendTileData(tileId, tileMesh, tileDetail, tileLinks) {
        const vertBase = this.mesh.verts.length;
        const polyBase = this.mesh.polys.length;

        const detailVertBase = this.meshdetail.verts.length;
        const triBase = this.meshdetail.tris.length;
        const meshRecBase = this.meshdetail.meshes.length;
        //放入点
        for (const v of tileMesh.verts) this.mesh.verts.push({ x: v.x, y: v.y, z: v.z });
        //放入邻居关系
        for (let i = 0; i < tileMesh.polys.length; i++) {
            const poly = tileMesh.polys[i];
            this.mesh.polys.push(poly.map((vi) => vertBase + vi));
            const oldNei = tileMesh.neighbors[i];
            this.mesh.neighbors.push(oldNei.map((entry) => {
                /**
                 * @type {number[]}
                 */
                const mapped = [];
                for (const n of entry) if (n >= 0) mapped.push(polyBase + n);
                return mapped;
            }));
        }
        //放入细节点
        for (const v of tileDetail.verts) this.meshdetail.verts.push({ x: v.x, y: v.y, z: v.z });
        //放入细节三角形
        for (let i = 0; i < tileDetail.tris.length; i++) {
            const tri = tileDetail.tris[i];
            this.meshdetail.tris.push([
                detailVertBase + tri[0],
                detailVertBase + tri[1],
                detailVertBase + tri[2]
            ]);
            this.meshdetail.triTopoly.push(polyBase + tileDetail.triTopoly[i]);
        }
        //mesh相关数据，长度之类的
        for (const m of tileDetail.meshes) {
            this.meshdetail.meshes.push([
                detailVertBase + m[0],
                m[1],
                triBase + m[2],
                m[3]
            ]);
        }
        //放入链接
        for (const link of tileLinks) {
            if (link.PolyA < 0 || link.PolyB < 0) continue;
            this.baseLinks.push({
                ...link,
                PolyA: polyBase + link.PolyA,
                PolyB: polyBase + link.PolyB,
                PosA: { x: link.PosA.x, y: link.PosA.y, z: link.PosA.z },
                PosB: { x: link.PosB.x, y: link.PosB.y, z: link.PosB.z }
            });
        }
        //记录 tile 在全局 mesh/detail 中的范围
        this.tileRanges.set(tileId, {
            vertBase,
            vertCount: tileMesh.verts.length,
            polyBase,
            polyCount: tileMesh.polys.length,
            detailVertBase,
            detailVertCount: tileDetail.verts.length,
            triBase,
            triCount: tileDetail.tris.length,
            meshRecBase,
            meshRecCount: tileDetail.meshes.length
        });

        this._linkTileWithNeighborTiles(tileId);
    }

    /**
     * 新 tile append 后，增量补齐其与周围 4 个 tile 的跨 tile neighbors。
     * @param {string} tileId
     */
    _linkTileWithNeighborTiles(tileId) {
        const tileData = this.tiles.get(tileId);
        const curRange = this.tileRanges.get(tileId);
        if (!tileData || !curRange || curRange.polyCount <= 0) return;

        const neighborTiles = this._collectNeighborTiles(tileData.tx, tileData.ty, tileId);
        if (neighborTiles.length === 0) return;
        //邻居 tile 的“开放边”
        const openEdgeStore = {
            exact: new Map(),
            buckets: new Map()
        };
        //收集所有邻居中的多边形的开放边(无邻居边)
        for (const nei of neighborTiles) {
            const neiRange = this.tileRanges.get(nei);
            if (!neiRange || neiRange.polyCount <= 0) continue;

            const end = neiRange.polyBase + neiRange.polyCount;
            for (let poly = neiRange.polyBase; poly < end; poly++) {
                const gpoly = this.mesh.polys[poly];
                if (gpoly.length === 0) continue;
                const edgeList = this.mesh.neighbors[poly];

                for (let edge = 0; edge < gpoly.length; edge++) {
                    if (edgeList[edge].length > 0) continue;
                    const va = gpoly[edge];
                    const vb = gpoly[(edge + 1) % gpoly.length];
                    const edgeRec = this.buildOpenEdgeRecord(this.mesh, poly, edge, va, vb);
                    this.addOpenEdge(openEdgeStore, edgeRec);
                }
            }
        }
        const curEnd = curRange.polyBase + curRange.polyCount;
        for (let poly = curRange.polyBase; poly < curEnd; poly++) {
            const gpoly = this.mesh.polys[poly];
            if (gpoly.length === 0) continue;
            const edgeList = this.mesh.neighbors[poly];

            for (let edge = 0; edge < gpoly.length; edge++) {
                if (edgeList[edge].length > 0) continue;

                const va = gpoly[edge];
                const vb = gpoly[(edge + 1) % gpoly.length];

                const reverseKey = `${vb}|${va}`;
                const exactMatched = this.getOpenEdgesByExactKey(openEdgeStore, reverseKey);
                for (const cand of exactMatched) {
                    this.addNeighborLink(this.mesh, poly, edge, cand.poly, cand.edge);
                }

                const fuzzyMatched = this.findOpenEdgesByOverlap(this.mesh, openEdgeStore, poly, edge, curRange.polyBase);
                for (const cand of fuzzyMatched) {
                    this.addNeighborLink(this.mesh, poly, edge, cand.poly, cand.edge);
                }
                //可以维护一个所有tile的边界边
            }
        }
    }

    /**
     * @param {number} tx
     * @param {number} ty
     * @param {string} selfTileId
     * @param {boolean} [includeDiagonal] 是否包含对角线邻居
     * @returns {string[]}
     */
    _collectNeighborTiles(tx, ty, selfTileId, includeDiagonal = false) {
        /** @type {string[]} */
        const out = [];
        for (const [tileId, tileData] of this.tiles.entries()) {
            if (tileId === selfTileId) continue;
            const rawDx = tileData.tx - tx;
            const rawDy = tileData.ty - ty;
            const dx = Math.abs(rawDx);
            const dy = Math.abs(rawDy);
            if(!includeDiagonal)
            {    
                if (dx+dy==1) {
                    out.push(tileId);
                }
            }
            else
            {
                if (dx <= 1 && dy <= 1) {
                    out.push(tileId);
                }
            }
        }
        return out;
    }

    /**
     * @param {string} tileId
     */
    _removeTileData(tileId) {
        // 1) 读取该 tile 在全局数组中的范围；没有范围说明未被 append，直接返回。
        const range = this.tileRanges.get(tileId);
        if (!range) return;

        // 2) 预先计算被删除区间的右边界，用于后续索引重映射判断。
        const vertEnd = range.vertBase + range.vertCount;
        const polyEnd = range.polyBase + range.polyCount;
        const dVertEnd = range.detailVertBase + range.detailVertCount;
        const triEnd = range.triBase + range.triCount;

        // 3) 从主 mesh 中删除该 tile 占用的顶点/多边形/邻接记录。
        this.mesh.verts.splice(range.vertBase, range.vertCount);
        this.mesh.polys.splice(range.polyBase, range.polyCount);
        this.mesh.neighbors.splice(range.polyBase, range.polyCount);

        // 4) 重映射剩余多边形中的顶点索引（位于删除区间之后的索引整体左移）。
        for (const poly of this.mesh.polys) {
            for (let i = 0; i < poly.length; i++) {
                if (poly[i] >= vertEnd) poly[i] -= range.vertCount;
            }
        }

        // 5) 重映射剩余多边形邻接：
        //    - 指向被删多边形的邻接要移除；
        //    - 位于被删区间之后的多边形索引要左移。
        for (let p = 0; p < this.mesh.neighbors.length; p++) {
            const edgeList = this.mesh.neighbors[p] || [];
            for (let e = 0; e < edgeList.length; e++) {
                const mapped = [];
                for (const n of edgeList[e]) {
                    if (n >= range.polyBase && n < polyEnd) continue;
                    mapped.push(n >= polyEnd ? n - range.polyCount : n);
                }
                edgeList[e] = mapped;
            }
        }

        // 6) 从 detail mesh 中删除该 tile 占用的 detail 顶点/三角形/triTopoly/mesh 记录。
        this.meshdetail.verts.splice(range.detailVertBase, range.detailVertCount);
        this.meshdetail.tris.splice(range.triBase, range.triCount);
        this.meshdetail.triTopoly.splice(range.triBase, range.triCount);
        this.meshdetail.meshes.splice(range.meshRecBase, range.meshRecCount);

        // 7) 重映射 detail 三角形顶点索引（位于删除区间之后的索引左移）。
        for (const tri of this.meshdetail.tris) {
            for (let i = 0; i < 3; i++) {
                if (tri[i] >= dVertEnd) tri[i] -= range.detailVertCount;
            }
        }

        // 8) 重映射 triTopoly（位于删除区间之后的 poly 索引左移）。
        for (let i = 0; i < this.meshdetail.triTopoly.length; i++) {
            const p = this.meshdetail.triTopoly[i];
            this.meshdetail.triTopoly[i] = p >= polyEnd ? p - range.polyCount : p;
        }

        // 9) 重映射 detail.meshes 里的 detail 顶点基址/三角形基址。
        for (const m of this.meshdetail.meshes) {
            if (m[0] >= dVertEnd) m[0] -= range.detailVertCount;
            if (m[2] >= triEnd) m[2] -= range.triCount;
        }

        // 10) 重映射 Links：
        //     - 端点落在被删 poly 区间内的 link 直接丢弃；
        //     - 其余 link 的 poly 索引按删除量左移。
        /** @param {NavMeshLink} link */
        const remapLink = (link) => {
            if (link.PolyA >= range.polyBase && link.PolyA < polyEnd) return null;
            if (link.PolyB >= range.polyBase && link.PolyB < polyEnd) return null;
            return {
                ...link,
                PolyA: link.PolyA >= polyEnd ? link.PolyA - range.polyCount : link.PolyA,
                PolyB: link.PolyB >= polyEnd ? link.PolyB - range.polyCount : link.PolyB,
            };
        };

        this.links = this.links.map(remapLink).filter((l) => !!l);

        // 11) 删除该 tile 的范围记录，并把其后 tile 的各类 base 统一左移。
        this.tileRanges.delete(tileId);
        for (const [k, r] of this.tileRanges.entries()) {
            if (r.vertBase > range.vertBase) r.vertBase -= range.vertCount;
            if (r.polyBase > range.polyBase) r.polyBase -= range.polyCount;
            if (r.detailVertBase > range.detailVertBase) r.detailVertBase -= range.detailVertCount;
            if (r.triBase > range.triBase) r.triBase -= range.triCount;
            if (r.meshRecBase > range.meshRecBase) r.meshRecBase -= range.meshRecCount;
            this.tileRanges.set(k, r);
        }
    }
    /**
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     */
    _rebuildDeferredLinks(targettileId) {
        const polyTileKeys = this._buildPolyTileKeys(targettileId);
        const interTileJumpLinks = this.buildInterTileJumpLinksForMesh(this.mesh, polyTileKeys, this.baseLinks);
        const ladderLinks = this.buildLadderLinksForMesh(this.mesh, this.meshdetail);
        this.supprlink = [...interTileJumpLinks,...ladderLinks];

        this.links = [...this.baseLinks, ...this.supprlink];
    }

    /**
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     * @returns {(string)[]}
     */
    _buildPolyTileKeys(targettileId) {
        const neitileid=[];
        const polyTileKeys = new Array(this.mesh.polys.length);
        if(targettileId) {
            const tileData = this.tiles.get(targettileId);
            if(tileData)neitileid.push(...this._collectNeighborTiles(tileData.tx,tileData.ty,targettileId,true));
            for (const [tileId, range] of this.tileRanges.entries()) {
                if(tileId!=targettileId||!neitileid.includes(tileId)) continue;
                const end = range.polyBase + range.polyCount;
                for (let p = range.polyBase; p < end; p++) {
                    polyTileKeys[p] = tileId;
                }
            }
        }
        else
        {
            for (const [tileId, range] of this.tileRanges.entries()) {
                const end = range.polyBase + range.polyCount;
                for (let p = range.polyBase; p < end; p++) {
                    polyTileKeys[p] = tileId;
                }
            }
        }
        return polyTileKeys;
    }

    /**
     * @param {tile} tileBuilder
     * @param {{x:number,y:number,z:number}} pos
     */
    rebuildAtPos(tileBuilder, pos) {
        const tileData = tileBuilder.buildTileNavMeshAtPos(pos);
        if (!tileData) return null;
        this.updatetile(tileData.tileId, tileData.tx, tileData.ty, tileData.mesh, tileData.detail, tileData.links);
        this.pruneUnreachablePolys();
        return tileData;
    }

    /**
     * 切换 pos 所在 tile：
     * - 若已存在则删除
     * - 若不存在则构建并添加
     * @param {tile} tileBuilder
     * @param {{x:number,y:number,z:number}} pos
     */
    reversetile(tileBuilder, pos) {
        const tileData = tileBuilder.buildTileNavMeshAtPos(pos);
        if (!tileData) return null;

        const tileId = String(tileData.tileId);
        if (this.tiles.has(tileId)) {
            this.removetile(tileId);
            this.pruneUnreachablePolys();
            return { action: "remove", tileId, tx: tileData.tx, ty: tileData.ty };
        }

        this.addtile(tileId, tileData.tx, tileData.ty, tileData.mesh, tileData.detail, tileData.links || []);
        this.pruneUnreachablePolys();
        return { action: "add", tileId, tx: tileData.tx, ty: tileData.ty };
    }

    pruneUnreachablePolys() {
        if (!this.mesh || !this.meshdetail || this.mesh.polys.length === 0) return;

        const polyCount = this.mesh.polys.length;
        const reachable = new Uint8Array(polyCount);
        const uf = new UnionFind(polyCount);

        for (let p = 0; p < polyCount; p++) {
            const nei = this.mesh.neighbors[p];
            for (const edgeNei of nei) {
                for (const n of edgeNei) {
                    if (n >= 0 && n < polyCount) uf.union(p, n);
                }
            }
        }

        /** @type {NavMeshLink[]} */
        const links = this.links;
        for (const link of links) {
            if (link.PolyA >= 0 && link.PolyA < polyCount && link.PolyB >= 0 && link.PolyB < polyCount) {
                uf.union(link.PolyA, link.PolyB);
            }
        }
        Tool.buildSpatialIndex(this.mesh);
        /** @type {number[]} */
        const seedPolys = [];
        const slist = Instance.FindEntitiesByClass("info_target");
        for (const ent of slist) {
            if (ent.GetEntityName() === "navmesh") {
                const seed = Tool.findNearestPoly(ent.GetAbsOrigin(), this.mesh).poly;
                if (seed >= 0 && seed < polyCount) seedPolys.push(seed);
            }
        }

        if (seedPolys.length === 0) {
            Instance.Msg("可达性筛选跳过: 未找到 info_target{name=navmesh} 种子");
            return;
        }

        const keepRoots = new Set();
        for (const seed of seedPolys) keepRoots.add(uf.find(seed));
        for (let i = 0; i < polyCount; i++) {
            if (keepRoots.has(uf.find(i))) reachable[i] = 1;
        }

        let keepCount = 0;
        for (let i = 0; i < polyCount; i++) if (reachable[i]) keepCount++;
        if (keepCount === polyCount) return;

        const oldToNewPoly = new Int32Array(polyCount).fill(-1);
        let newPolyCount = 0;
        for (let i = 0; i < polyCount; i++) {
            if (reachable[i]) oldToNewPoly[i] = newPolyCount++;
        }

        /** @type {NavMeshMesh} */
        const newMesh = { verts: [], polys: [], regions: [], neighbors: [] };
        const oldToNewMeshVert = new Map();

        for (let oldPi = 0; oldPi < polyCount; oldPi++) {
            if (!reachable[oldPi]) continue;

            const oldPoly = this.mesh.polys[oldPi];
            const newPoly = [];
            for (const oldVi of oldPoly) {
                if (!oldToNewMeshVert.has(oldVi)) {
                    oldToNewMeshVert.set(oldVi, newMesh.verts.length);
                    newMesh.verts.push(this.mesh.verts[oldVi]);
                }
                newPoly.push(oldToNewMeshVert.get(oldVi));
            }
            newMesh.polys.push(newPoly);

            const oldNei = this.mesh.neighbors[oldPi];
            newMesh.neighbors.push(oldNei.map((entry) => {
                const mapped = [];
                for (const n of entry) {
                    if (n >= 0 && reachable[n]) mapped.push(oldToNewPoly[n]);
                }
                return mapped;
            }));
        }

        /** @type {NavMeshDetail} */
        const newDetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        const oldToNewDetailVert = new Map();

        for (let oldPi = 0; oldPi < polyCount; oldPi++) {
            if (!reachable[oldPi]) continue;
            const newPi = oldToNewPoly[oldPi];

            const meshRec = this.meshdetail.meshes[oldPi];
            if (!meshRec) {
                newDetail.meshes.push([0, 0, newDetail.tris.length, 0]);
                continue;
            }

            const baseTri = meshRec[2];
            const triCount = meshRec[3];
            const triStartNew = newDetail.tris.length;

            for (let ti = baseTri; ti < baseTri + triCount; ti++) {
                const oldTri = this.meshdetail.tris[ti];
                if (!oldTri) continue;
                /** @type {number[]} */
                const remappedTri = [];
                for (const oldDvi of oldTri) {
                    if (!oldToNewDetailVert.has(oldDvi)) {
                        oldToNewDetailVert.set(oldDvi, newDetail.verts.length);
                        newDetail.verts.push(this.meshdetail.verts[oldDvi]);
                    }
                    remappedTri.push(oldToNewDetailVert.get(oldDvi));
                }
                newDetail.tris.push(remappedTri);
                newDetail.triTopoly.push(newPi);
            }

            newDetail.meshes.push([0, 0, triStartNew, newDetail.tris.length - triStartNew]);
        }

        this.mesh = newMesh;
        this.meshdetail = newDetail;

        if (this.links && this.links.length > 0) {
            const remappedLinks = [];
            for (const link of this.links) {
                const na = oldToNewPoly[link.PolyA];
                const nb = oldToNewPoly[link.PolyB];
                if (na < 0 || nb < 0) continue;
                remappedLinks.push({
                    ...link,
                    PolyA: na,
                    PolyB: nb,
                });
            }
            this.links = remappedLinks;
        }

        Instance.Msg(`可达性筛选完成: ${polyCount} -> ${keepCount}`);
    }

    _rebuild() {
        /** @type {NavMeshMesh} */
        const mergedMesh = { verts: [], polys: [], regions: [], neighbors: [] };

        /** @type {NavMeshDetail} */
        const mergedDetail = { verts: [], tris: [], triTopoly: [], meshes: [] };

        /** @type {NavMeshLink[]} */
        const mergedLinks = [];

        /** @type {string[]} */
        const polyTileKeys = [];

        const globalVertMap = new Map();
        const globalOpenEdgeStore = {
            exact: new Map(),
            buckets: new Map()
        };

        for (const { tileId, mesh, detail, links } of this.tiles.values()) {
            const { polyMap } = this.mergeTileMesh(mergedMesh, mesh, globalVertMap, globalOpenEdgeStore);
            for (const globalPi of polyMap) polyTileKeys[globalPi] = tileId;

            this.mergeTileDetail(mergedDetail, detail, polyMap);

            for (const link of links || []) {
                if (link.PolyA < 0 || link.PolyB < 0) continue;
                const mappedA = polyMap[link.PolyA];
                const mappedB = polyMap[link.PolyB];
                if (mappedA == null || mappedB == null) continue;
                mergedLinks.push({
                    ...link,
                    PolyA: mappedA,
                    PolyB: mappedB,
                    PosA: { x: link.PosA.x, y: link.PosA.y, z: link.PosA.z },
                    PosB: { x: link.PosB.x, y: link.PosB.y, z: link.PosB.z }
                });
            }
        }

        this._appendInterTileWalkLinks(mergedMesh, mergedLinks, polyTileKeys);

        // 所有 tile 合并完成后，再统一构建“延后连接”（如梯子、传送点等）
        this.supprlink = [];
        const deferredLinks = this.buildLadderLinksForMesh(mergedMesh, mergedDetail);
        if (deferredLinks && deferredLinks.length > 0) {
            this.supprlink.push(...deferredLinks);
            mergedLinks.push(...deferredLinks);
        }

        this.mesh = mergedMesh;
        this.meshdetail = mergedDetail;

        this.links = mergedLinks;
    }

    /**
    * @param {NavMeshMesh} globalMesh
    * @param {NavMeshMesh} tileMesh
     * @param {Map<string,number>} globalVertMap
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} globalOpenEdgeStore
     */
    mergeTileMesh(globalMesh, tileMesh, globalVertMap, globalOpenEdgeStore) {
        const tilePolyStart = globalMesh.polys.length;

        const localVertToGlobal = new Array(tileMesh.verts.length).fill(-1);
        for (let i = 0; i < tileMesh.verts.length; i++) {
            const v = tileMesh.verts[i];
            const k = `${v.x}|${v.y}|${v.z}`;
            let gi = globalVertMap.get(k);
            if (gi === undefined) {
                gi = globalMesh.verts.length;
                globalMesh.verts.push(v);
                globalVertMap.set(k, gi);
            }
            localVertToGlobal[i] = gi;
        }

        const boundaryFlagsByLocalPoly = [];
        const polyMap = new Array(tileMesh.polys.length).fill(-1);
        for (let i = 0; i < tileMesh.polys.length; i++) {
            const gpoly = tileMesh.polys[i].map((vi) => localVertToGlobal[vi]);

            const globalPi = globalMesh.polys.length;
            polyMap[i] = globalPi;
            globalMesh.polys.push(gpoly);
            globalMesh.neighbors.push(new Array(gpoly.length).fill(0).map(() => []));
        }

        for (let i = 0; i < tileMesh.polys.length; i++) {
            const globalPi = polyMap[i];
            const oldNei = tileMesh.neighbors[i];
            globalMesh.neighbors[globalPi] = oldNei.map((entry) => {
                const ans = [];
                for (const n of entry) {
                    if (n >= 0) ans.push(polyMap[n]);
                }
                return ans;
            });
            boundaryFlagsByLocalPoly[i] = oldNei.map((entry) => entry.length === 0);
        }

        for (let i = 0; i < tileMesh.polys.length; i++) {
            const globalPi = polyMap[i];
            const gpoly = globalMesh.polys[globalPi];
            const boundaryFlags = boundaryFlagsByLocalPoly[i] || [];
            for (let ei = 0; ei < gpoly.length; ei++) {
                if (!boundaryFlags[ei]) continue;

                const va = gpoly[ei];
                const vb = gpoly[(ei + 1) % gpoly.length];

                const reverseKey = `${vb}|${va}`;
                const exactMatched = this.getOpenEdgesByExactKey(globalOpenEdgeStore, reverseKey);
                for (const cand of exactMatched) {
                    if (cand.poly >= tilePolyStart) continue;
                    this.addNeighborLink(globalMesh, globalPi, ei, cand.poly, cand.edge);
                }

                const fuzzyMatched = this.findOpenEdgesByOverlap(globalMesh, globalOpenEdgeStore, globalPi, ei, tilePolyStart);
                for (const cand of fuzzyMatched) {
                    this.addNeighborLink(globalMesh, globalPi, ei, cand.poly, cand.edge);
                }

                const openEdge = this.buildOpenEdgeRecord(globalMesh, globalPi, ei, va, vb);
                this.addOpenEdge(globalOpenEdgeStore, openEdge);
            }
        }

        return {
            polyMap
        };
    }

    /**
        * @param {NavMeshMesh} mesh
     * @param {number} poly
     * @param {number} edge
     * @param {number} va
     * @param {number} vb
     */
    buildOpenEdgeRecord(mesh, poly, edge, va, vb) {
        const a = mesh.verts[va];
        const b = mesh.verts[vb];
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const len = Math.hypot(dx, dy);
        const major = Math.abs(dx) >= Math.abs(dy) ? 0 : 1;
        const lineCoord = major === 0 ? (a.y + b.y) * 0.5 : (a.x + b.x) * 0.5;
        const pa = major === 0 ? a.x : a.y;
        const pb = major === 0 ? b.x : b.y;
        const projMin = Math.min(pa, pb);
        const projMax = Math.max(pa, pb);
        const dirX = len > 1e-6 ? dx / len : 0;
        const dirY = len > 1e-6 ? dy / len : 0;
        const centerZ = (a.z + b.z) * 0.5;
        const bucketScale = Math.max(1e-4, MESH_CELL_SIZE_XY * 0.6);
        const bucketId = Math.round(lineCoord / bucketScale);
        return {poly,edge,va,vb,exactKey: `${va}|${vb}`,major,lineCoord,projMin,projMax,dirX,dirY,centerZ,bucketId,};
    }

    /**
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} store
     * @param {any} edgeRec
     */
    addOpenEdge(store, edgeRec) {
        const list = Tool.getOrCreateArray(store.exact, edgeRec.exactKey);
        if (!list.includes(edgeRec)) list.push(edgeRec);

        const bucketKey = `${edgeRec.major}|${edgeRec.bucketId}`;
        const bucket = Tool.getOrCreateArray(store.buckets, bucketKey);
        if (!bucket.includes(edgeRec)) bucket.push(edgeRec);
    }

    /**
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} store
     * @param {string} key
     */
    getOpenEdgesByExactKey(store, key) {
        const list = store.exact.get(key);
        if (!list || list.length === 0) return [];
        return list;
    }

    /**
    * @param {NavMeshMesh} mesh
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} store
     * @param {number} poly
    * @param {number} edge
    * @param {number} tilePolyStart
     */
    findOpenEdgesByOverlap(mesh, store, poly, edge, tilePolyStart) {
        const gpoly = mesh.polys[poly];
        const va = gpoly[edge];
        const vb = gpoly[(edge + 1) % gpoly.length];
        const cur = this.buildOpenEdgeRecord(mesh, poly, edge, va, vb);

        const lineTol = MESH_CELL_SIZE_XY * 0.6;
        const maxProjGapXY = MESH_CELL_SIZE_XY;
        const minXYOverlap = 0.1;
        const maxZDiff = MAX_WALK_HEIGHT * MESH_CELL_SIZE_Z;

        /** @type {any[]} */
        const candidates = [];
        const dedup = new Set();
        for (let b = cur.bucketId - 1; b <= cur.bucketId + 1; b++) {
            const bucketKey = `${cur.major}|${b}`;
            const bucket = store.buckets.get(bucketKey);
            if (!bucket || bucket.length === 0) continue;

            for (const candidate of bucket) {
                if (candidate.poly === poly) continue;
                if (candidate.poly >= tilePolyStart) continue;
                if (Math.abs(candidate.lineCoord - cur.lineCoord) > lineTol) continue;

                const dot = cur.dirX * candidate.dirX + cur.dirY * candidate.dirY;
                if (dot > -0.8) continue;

                const curSeg = this.getEdgeProjectionSegments(mesh, cur);
                const candSeg = this.getEdgeProjectionSegments(mesh, candidate);
                const projGapX = this.getProjectionGap(curSeg.xMin, curSeg.xMax, candSeg.xMin, candSeg.xMax);
                const projGapY = this.getProjectionGap(curSeg.yMin, curSeg.yMax, candSeg.yMin, candSeg.yMax);
                const projGapXY = Math.hypot(projGapX, projGapY);
                if (projGapXY >= maxProjGapXY) continue;

                const overlapMin = Math.max(cur.projMin, candidate.projMin);
                const overlapMax = Math.min(cur.projMax, candidate.projMax);
                if (overlapMax < overlapMin) continue;
                if ((overlapMax - overlapMin) < minXYOverlap) continue;

                const curOverlapZ = this.getEdgeZRangeOnMajorOverlap(mesh, cur, cur.major, overlapMin, overlapMax);
                const candOverlapZ = this.getEdgeZRangeOnMajorOverlap(mesh, candidate, cur.major, overlapMin, overlapMax);
                const projGapZ = this.getProjectionGap(curOverlapZ.min, curOverlapZ.max, candOverlapZ.min, candOverlapZ.max);
                if (projGapZ >= maxZDiff) continue;

                const ck = `${candidate.poly}|${candidate.edge}`;
                if (dedup.has(ck)) continue;
                dedup.add(ck);
                candidates.push(candidate);
            }
        }

        return candidates;
    }

    /**
     * 返回边在 x/y/z 轴上的投影区间。
    * @param {NavMeshMesh} mesh
     * @param {{va:number,vb:number}} edgeRec
     */
    getEdgeProjectionSegments(mesh, edgeRec) {
        const a = mesh.verts[edgeRec.va];
        const b = mesh.verts[edgeRec.vb];
        return {
            xMin: Math.min(a.x, b.x),
            xMax: Math.max(a.x, b.x),
            yMin: Math.min(a.y, b.y),
            yMax: Math.max(a.y, b.y),
            zMin: Math.min(a.z, b.z),
            zMax: Math.max(a.z, b.z),
        };
    }

    /**
     * 两个一维投影区间的间距（有重叠时为0）。
     * @param {number} minA
     * @param {number} maxA
     * @param {number} minB
     * @param {number} maxB
     */
    getProjectionGap(minA, maxA, minB, maxB) {
        const overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
        if (overlap >= 0) return 0;
        return -overlap;
    }

    /**
     * 取边在“主轴重叠区间”内的 Z 投影区间。
    * @param {NavMeshMesh} mesh
     * @param {{va:number,vb:number}} edgeRec
    * @param {number} major 0:x 轴, 1:y 轴
     * @param {number} overlapMin
     * @param {number} overlapMax
     */
    getEdgeZRangeOnMajorOverlap(mesh, edgeRec, major, overlapMin, overlapMax) {
        const a = mesh.verts[edgeRec.va];
        const b = mesh.verts[edgeRec.vb];

        const ca = major === 0 ? a.x : a.y;
        const cb = major === 0 ? b.x : b.y;
        const dc = cb - ca;

        if (Math.abs(dc) <= 1e-6) {
            const zMin = Math.min(a.z, b.z);
            const zMax = Math.max(a.z, b.z);
            return { min: zMin, max: zMax };
        }

        const t0 = Math.max(0, Math.min(1, (overlapMin - ca) / dc));
        const t1 = Math.max(0, Math.min(1, (overlapMax - ca) / dc));

        const z0 = a.z + (b.z - a.z) * t0;
        const z1 = a.z + (b.z - a.z) * t1;

        return {
            min: Math.min(z0, z1),
            max: Math.max(z0, z1),
        };
    }

    /**
     * @param {NavMeshMesh} mesh
     * @param {NavMeshLink[]} links
     * @param {string[]} polyTileKeys
     */
    _appendInterTileWalkLinks(mesh, links, polyTileKeys) {
        const pairKeySet = new Set();
        for (const link of links) {
            pairKeySet.add(this._polyPairKey(link.PolyA, link.PolyB));
        }

        const maxZDiff = MAX_WALK_HEIGHT * MESH_CELL_SIZE_Z;

        for (let poly = 0; poly < mesh.polys.length; poly++) {
            const edgeList = mesh.neighbors[poly] || [];
            for (let edge = 0; edge < edgeList.length; edge++) {
                const nList = this._normalizeNeighbor(edgeList[edge]);
                for (const n of nList) {
                    if (n < 0 || n <= poly) continue;
                    if (polyTileKeys[poly] === polyTileKeys[n]) continue;

                    const pairKey = this._polyPairKey(poly, n);
                    if (pairKeySet.has(pairKey)) continue;

                    const edgeB = this._findNeighborEdgeIndex(mesh, n, poly);
                    if (edgeB < 0) continue;

                    const posA = this._edgeMidpoint(mesh, poly, edge);
                    const posB = this._edgeMidpoint(mesh, n, edgeB);
                    if (Math.abs(posA.z - posB.z) > maxZDiff) continue;

                    const dx = posA.x - posB.x;
                    const dy = posA.y - posB.y;
                    const dz = posA.z - posB.z;
                    const cost = Math.max(1, Math.hypot(dx, dy, dz));

                    links.push({
                        PolyA: poly,
                        PolyB: n,
                        PosA: posA,
                        PosB: posB,
                        cost,
                        type: PathState.WALK
                    });
                    pairKeySet.add(pairKey);
                }
            }
        }
    }

    /**
     * @param {number} a
     * @param {number} b
     */
    _polyPairKey(a, b) {
        return a < b ? `${a}|${b}` : `${b}|${a}`;
    }

    /**
     * @param {NavMeshMesh} mesh
     * @param {number} poly
     * @param {number} target
     */
    _findNeighborEdgeIndex(mesh, poly, target) {
        const edgeList = mesh.neighbors[poly] || [];
        for (let edge = 0; edge < edgeList.length; edge++) {
            const nList = this._normalizeNeighbor(edgeList[edge]);
            if (nList.includes(target)) return edge;
        }
        return -1;
    }

    /**
     * @param {NavMeshMesh} mesh
     * @param {number} poly
     * @param {number} edge
     */
    _edgeMidpoint(mesh, poly, edge) {
        const p = mesh.polys[poly];
        const va = mesh.verts[p[edge]];
        const vb = mesh.verts[p[(edge + 1) % p.length]];
        return {
            x: (va.x + vb.x) * 0.5,
            y: (va.y + vb.y) * 0.5,
            z: (va.z + vb.z) * 0.5
        };
    }

    /**
    * @param {NavMeshMesh} mesh
     * @param {number} polyA
     * @param {number} edgeA
     * @param {number} polyB
     * @param {number} edgeB
     */
    addNeighborLink(mesh, polyA, edgeA, polyB, edgeB) {
        const listA = mesh.neighbors[polyA][edgeA];
        const listB = mesh.neighbors[polyB][edgeB];
        if (!listA.includes(polyB)) listA.push(polyB);
        if (!listB.includes(polyA)) listB.push(polyA);
        mesh.neighbors[polyA][edgeA] = listA;
        mesh.neighbors[polyB][edgeB] = listB;
    }

    /**
    * @param {NavMeshDetail} globalDetail
    * @param {NavMeshDetail} tileDetail
     * @param {number[]} polyMap
     */
    mergeTileDetail(globalDetail, tileDetail, polyMap) {
        const vertBase = globalDetail.verts.length;
        const triBase = globalDetail.tris.length;

        for (const v of tileDetail.verts) {
            globalDetail.verts.push(v);
        }

        for (let i = 0; i < tileDetail.tris.length; i++) {
            const tri = tileDetail.tris[i];
            globalDetail.tris.push([
                vertBase + tri[0],
                vertBase + tri[1],
                vertBase + tri[2]
            ]);
            globalDetail.triTopoly.push(polyMap[tileDetail.triTopoly[i]]);
        }

        for (const m of tileDetail.meshes) {
            globalDetail.meshes.push([
                vertBase + m[0],
                m[1],
                triBase + m[2],
                m[3]
            ]);
        }
    }

    /**
     * @param {number|number[]|undefined|null} entry
     * @returns {number[]}
     */
    _normalizeNeighbor(entry) {
        if (Array.isArray(entry)) return entry;
        if (typeof entry === "number") return entry >= 0 ? [entry] : [];
        return [];
    }

    /**
    * @param {NavMeshMesh} mesh
     */
    _cloneMesh(mesh) {
        return {
            verts: (mesh.verts).map((v) => ({ x: v.x, y: v.y, z: v.z })),
            polys: (mesh.polys).map((poly) => poly.slice()),
            regions: [],
            neighbors: (mesh.neighbors).map((edgeList) =>edgeList.slice())
        };
    }

    /**
    * @param {NavMeshDetail} detail
     */
    _cloneDetail(detail) {
        return {
            verts: (detail.verts).map((v) => ({ x: v.x, y: v.y, z: v.z })),
            tris: (detail.tris).map((tri) => tri.slice()),
            triTopoly: (detail.triTopoly).slice(),
            meshes: (detail.meshes).map((m) => m.slice())
        };
    }

    /**
    * @param {NavMeshLink[]} links
     */
    _cloneLinks(links) {
        if (links.length === 0) return [];
        return links.map((link) => ({
            PolyA: link.PolyA,
            PolyB: link.PolyB,
            PosA: { x: link.PosA.x, y: link.PosA.y, z: link.PosA.z },
            PosB: { x: link.PosB.x, y: link.PosB.y, z: link.PosB.z },
            cost: link.cost,
            type: link.type
        }));
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/**
 * @typedef {{
 *  verts:Vector[],
 *  polys:number[][],
 *  regions:number[],
 *  neighbors:number[][][]
 * }} NavMeshMesh
 */

/**
 * @typedef {{
 *  verts:Vector[],
 *  tris:number[][],
 *  triTopoly:number[],
 *  meshes:number[][]
 * }} NavMeshDetail
 */

/**
 * @typedef {{
 *  PolyA:number,
 *  PolyB:number,
 *  PosA:Vector,
 *  PosB:Vector,
 *  cost:number,
 *  type:number
 * }} NavMeshLink
 */

class NavMesh {
    constructor() {
        /**@type {PolyGraphAStar} */
        this.astar;
        /**@type {NavMeshMesh} */
        this.mesh;
        /**@type {NavMeshDetail} */
        this.meshdetail;
        /**@type {FunnelPath} */
        this.funnel;
        /**@type {FunnelHeightFixer} */
        this.heightfixer;
        /**@type {NavMeshLink[]} */
        this.links;
        /** @type {TileManager} */
        this.tileManager = new TileManager();
        /** @type {tile} */
        this.tile = new tile();
        this.debugTools = new NavMeshDebugTools(this);
        //删除prop_door_rotating实体？也许应该弄一个目录，让作者把门一类的实体名字放里面
    }
    /**
     * 导出导航网格数据为文本字符串
     */
    exportNavData() {
        const charsPerLine = 500;
        if (!this.mesh) {
            Instance.Msg("错误：没有可导出的数据！请先生成网格。");
        }
        this.mesh.verts = this.mesh.verts.map(v => ({
            x: Math.round(v.x * 100) / 100,
            y: Math.round(v.y * 100) / 100,
            z: Math.round(v.z * 100) / 100
        }));
        this.meshdetail.verts=this.meshdetail.verts.map(v => ({
            x: Math.round(v.x * 100) / 100,
            y: Math.round(v.y * 100) / 100,
            z: Math.round(v.z * 100) / 100
        }));
        const data = {
            mesh: this.mesh,           // 包含 verts, polys, regions, neighbors
            meshdetail: this.meshdetail,
            links: this.links || [],
        };

        // 使用 JSON 序列化
        const jsonStr = JSON.stringify(data);
        // 2. 将字符串切割成指定长度的块
        Instance.Msg("--- NAV DATA START ---");
        for (let i = 0; i < jsonStr.length; i += charsPerLine) {
            Instance.Msg("+`"+jsonStr.substring(i, i + charsPerLine)+"`");
        }
        Instance.Msg("--- NAV DATA END ---");
    }
    /**
     * 从文本字符串恢复导航网格
     * @param {string} jsonStr 
     */
    importNavData(jsonStr) {
        try {
            const cleanJson = jsonStr.replace(/\s/g, "");
            const data = JSON.parse(cleanJson);

            // 1. 恢复核心网格数据
            this.mesh = data.mesh;
            this.links=data.links;
            this.meshdetail=data.meshdetail;
            Instance.Msg(`导航数据加载成功！多边形数量: ${this.mesh.polys.length}`);
            return true;
        } catch (e) {
            Instance.Msg(`加载导航数据失败: ${e}`);
            return false;
        }
    }
    init() {
        {
            this.tileManager = new TileManager();
            this.tileManager.rebuildAll(this.tile);
            let merged = this.tileManager.return();
            this.mesh = merged.mesh;
            this.meshdetail = merged.meshdetail;
            this.links = merged.links;
        }
        this._refreshRuntime();
    }

    /**
     * 仅更新 pos 所在 tile 的导航网格。
     * @param {Vector} pos
     */
    initpos(pos) {
        //加入scriptin？让其可以根据act实体位置进行tile刷新？
        this.tileManager.rebuildAtPos(this.tile, pos);
        const merged = this.tileManager.return();
        this.mesh = merged.mesh;
        this.meshdetail = merged.meshdetail;
        this.links = merged.links;
        this._refreshRuntime();
    }
    /**
     * @param {Vector} pos
     */
    update(pos)
    {
        this.tileManager.reversetile(this.tile, pos);
        const merged = this.tileManager.return();
        this.mesh = merged.mesh;
        this.meshdetail = merged.meshdetail;
        this.links = merged.links;
        this._refreshRuntime();
    }
    _refreshRuntime() {
        Tool.buildSpatialIndex(this.mesh);
        /**@type {Map<number,NavMeshLink[]>} */
        const links = new Map();
        for (const link of this.links) {
            const polyA = link.PolyA;
            const polyB = link.PolyB;
            if (!links.has(polyA)) links.set(polyA, []);
            if (!links.has(polyB)) links.set(polyB, []);
            links.get(polyA)?.push(link);
            links.get(polyB)?.push(link);
        }
        this.heightfixer = new FunnelHeightFixer(this.mesh, this.meshdetail, ADJUST_HEIGHT_DISTANCE);
        this.astar = new PolyGraphAStar(this.mesh, links, this.heightfixer);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers, links);
    }
    /**
     * 只调试“可持久保存”的数据（mesh/detail/links）。
     * @param {number} duration
     */
    debug(duration = 60) {
        {
            Instance.Msg(`多边形总数: ${this.mesh.polys.length}`);
            this.debugTools.debugDrawMeshPolys(duration);
            this.debugTools.debugDrawMeshConnectivity(duration);
            this.debugTools.debugLinks(duration);
            return;
        }
    }
    /**
     * 输入起点终点，返回世界坐标路径点
     * @param {Vector} start
     * @param {Vector} end
     * @returns {{pos:Vector,mode:number}[]}
     */
    findPath(start, end) {
        //Instance.DebugLine({start,end,duration:1,color:{r:0,g:255,b:0}});
        const polyPath=this.astar.findPath(start,end);
        //this.debugTools.debugDrawPolyPath(polyPath.path,1/2);
        //if (!polyPath || polyPath.path.length === 0) return [];
        const funnelPath = this.funnel.build(polyPath.path, polyPath.start, polyPath.end);
        //this.debugTools.debugDrawfunnelPath(funnelPath,1/2);
        const ans=this.heightfixer.fixHeight(funnelPath,polyPath.path);
        //this.debugTools.debugDrawPath(ans,1/2);
        //if (!ans || ans.length === 0) return [];
        //多边形总数：649跳点数：82
        //100次A*           42ms
        //100次funnelPath   55ms-42=13ms
        //100次50fixHeight    106ms-55=51ms
        //100次200fixHeight    70ms-55=15ms
        return ans;
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
Instance.ServerCommand("bot_kick");
Instance.ServerCommand("mp_warmup_offline_enabled 1");
Instance.ServerCommand("mp_warmup_pausetimer 1");
Instance.ServerCommand("mp_roundtime 60");
Instance.ServerCommand("mp_freezetime 1");
Instance.ServerCommand("mp_ignore_round_win_conditions 1");
Instance.ServerCommand("weapon_accuracy_nospread 1");
Instance.ServerCommand("mat_fullbright 1");
//for(let l=0;l<30;l++)
//{
//    Instance.ServerCommand("bot_add");
//}
let pathfinder = new NavMesh();
//Instance.OnScriptReload({
//    before: () => {
//    },
//    after: () => {
//        //let start = new Date();
//        //Instance.Msg("导航初始化中");
//        //pathfinder.init();
//        //let end = new Date();
//        //Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
//    }
//});
let path_ini=false;

function init()
{
    if(path_ini)return;
    let start = new Date();
    Instance.Msg("导航初始化中");
    pathfinder.init();
    let end = new Date();
    Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
    path_ini=true;
}
//let start={x:3457,y:-984,z:-352};
//let end={x:-2960,y:-625,z:-416};
let pd=false;
//let sss=false;
Instance.SetThink(() => {
    if(pd==true)
    {
        //pathfinder.randomTest(10);
        //var players=Instance.FindEntitiesByClass("player");
        //players.forEach((e)=>{
        //    if(e&&e instanceof CSPlayerPawn)
        //    {
        //        var p=e.GetPlayerController()?.GetPlayerPawn();
        //        if(p)
        //        {
        //            const pos=p.GetAbsOrigin();
        //            end={x:pos.x,y:pos.y,z:pos.z};
        //            return;
        //        }
        //    }
        //})
        pathfinder.debug(1);
        //for(let i=0;i<1;i++)pathfinder.findPath(start,end);
    }
    Instance.SetNextThink(Instance.GetGameTime()+1);
});
Instance.SetNextThink(Instance.GetGameTime()+1);
Instance.OnBulletImpact((event)=>{
    pathfinder.update(event.position);
    //pathfinder.debug(30);
    //if(sss)end=event.position;
    //else start=event.position;
    //sss=!sss;
    //pathfinder.findPath(start,end);
    //pathfinder.findPath(start,end);
});
Instance.OnPlayerChat((event) => {
    const text = (event.text || "").trim().toLowerCase().split(' ')[0];
    if (text === "debug" || text === "!debug")
    {
        //每秒200000次tracebox,每tick 3125次，每个怪物平均10次，总共312只怪物
        //10v10 1000次tracebox
        init();
        Instance.Msg("开始调试");
        //pathfinder.debug(60);
        //pathfinder.debugTools.debug(60);
        //pathfinder.debugTools.testinit();
        pd=true;
        //pathfinder.debugTools.testinit();
        //pathfinder.debugTools.randomTest(num);
    }
    //if (text === "c" || text === "!c")
    //{
    //    const p=event.player?.GetPlayerPawn();
    //    if(p)
    //    {
    //        const pos=p.GetAbsOrigin();
    //        start={x:pos.x,y:pos.y,z:pos.z};
    //        //Instance.Msg(`${Math.floor(pos.x)}  ${Math.floor(pos.y)}  ${Math.floor(pos.z)}`);
    //    }
    //}
    //if (text === "v" || text === "!v")
    //{
    //    const p=event.player?.GetPlayerPawn();
    //    if(p)
    //    {
    //        const pos=p.GetAbsOrigin();
    //        end={x:pos.x,y:pos.y,z:pos.z};
    //        //const path = pathfinder.findPath(start,end);
    //        //Instance.Msg(`${Math.floor(end.x)}  ${Math.floor(end.y)}  ${Math.floor(end.z)}`);
    //        //pathfinder.debugDrawPath(path,30);
    //    }
    //}
});
