import { Instance } from 'cs_script/point_script';

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("cs_script/point_script").Color} Color */
//navmesh最多65535个顶点
const PathState = {
    WALK: 1,//下一个点直走
    JUMP: 2,//下一个点需要跳跃
    LADDER: 3,//下一个点是梯子点，开启梯子状态，直到下一个点不是梯子
    PORTAL: 4//下一个点是传送点，开启瞬移模式
};
//==============================世界相关设置=====================================
const origin = { x: -3050, y: -2725, z: -780 };
const MESH_CELL_SIZE_XY = 8;                               // 体素大小
const MESH_CELL_SIZE_Z = 1;                                // 体素高度
const MESH_TRACE_SIZE_Z = 32;                              // 射线方块高度//太高，会把竖直方向上的间隙也忽略
const MESH_WORLD_SIZE_XY = 6600;                           // 世界大小
const MESH_WORLD_SIZE_Z = 980;                             // 世界高度
//==============================数据结构设置=====================================
const MAX_POLYS = 65535;                                   // 多边形最大数量，受限于16位索引
const MAX_VERTS = 65535;                                   // 顶点最大数量
const MAX_TRIS = 65535;                                    // 三角形最大数量
const MAX_LINKS = 4096;                                    // 跳点最大数量
//==============================Recast设置======================================
//其他参数
const MAX_SLOPE = 65;                                      // 最大坡度（度数），超过这个角度的斜面将被视为不可行走
const MAX_WALK_HEIGHT = 13 / MESH_CELL_SIZE_Z;             // 怪物最大可行走高度（体素单位）
const MAX_JUMP_HEIGHT = 63 / MESH_CELL_SIZE_Z;             // 怪物最大可跳跃高度（体素单位）
const AGENT_RADIUS = 8 / MESH_CELL_SIZE_XY;                // 人物通过所需宽度半径大小（长度）
const AGENT_HEIGHT = 40 / MESH_CELL_SIZE_Z;                // 人物高度（体素单位）
//TILE参数
const TILE_SIZE = 512 / MESH_CELL_SIZE_XY;                 // 瓦片大小（体素单位），每个瓦片包含 tileSize*tileSize 个体素，过大可能导致性能问题，过小可能导致内存占用增加
const TILE_PADDING = AGENT_RADIUS + 1;                     // 瓦片边界添加的体素数量，防止寻路时穿模，值需要比MESH_ERODE_RADIUS大
const TILE_OPTIMIZATION_1 = true;                          // 优化1：是否修剪掉info_target{name:navmesh}不能去的平台
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
//==============================Detour设置======================================
//A*寻路参数
const OFF_MESH_LINK_COST_SCALE=1;                         // 特殊点的权重，越大越不倾向于特殊点
const ASTAR_HEURISTIC_SCALE = 1.2;                         // A*推荐数值
//Funnel参数
const FUNNEL_DISTANCE = 15;                                // 拉直的路径距离边缘多远(0-100，百分比，100%意味着只能走边的中点)
const ADJUST_HEIGHT_DISTANCE = 200;                         // 路径中每隔这个距离增加一个点，用于修正高度
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
 * @param {Float32Array} verts
 * @param {number} start
 * @param {number} end
 */
function pointInConvexPolyXY(p, verts, start, end) {
    for (let i = start; i <= end; i++) {
        const a = { x: verts[i * 3], y: verts[i * 3 + 1]};
        const b = { x: verts[((i < end) ? (i + 1) : start) * 3], y: verts[((i < end) ? (i + 1) : start) * 3 + 1]};
        if (area(a, b, p) < 0) return false;
    }
    return true;
}
/**
 * 点到 polygon 最近点(xy投影)
 * @param {Vector} pos
 * @param {Float32Array} verts
 * @param {number} start
 * @param {number} end
 */
function closestPointOnPoly(pos, verts, start, end) {
    // 1. 如果在多边形内部（XY），直接投影到平面
    if (pointInConvexPolyXY(pos, verts, start, end)) {
        // 用平均高度（你也可以用平面方程）
        let maxz = -Infinity, minz = Infinity;
        start*=3;
        end*=3;
        for (let i = start; i <= end; i+=3) {
            const z = verts[i + 2];
            if (z > maxz) maxz = z;
            if (z < minz) minz = z;
        }
        return { x: pos.x, y: pos.y, z: (maxz + minz) >>1, in: true };
    }
    // 2. 否则，找最近边
    let best = null;
    let bestDist = Infinity;
    for (let i = start; i <= end; i++) {
        const ia = i;
        const ib = (i < end) ? (i + 1) : start;
        const a = { x: verts[ia * 3], y: verts[ia * 3 + 1], z: verts[ia * 3 + 2] };
        const b = { x: verts[ib * 3], y: verts[ib * 3 + 1], z: verts[ib * 3 + 2] };
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
        const polyCount = this.navMesh.polyslength;
        this.polyTriStart = new Uint16Array(polyCount);
        this.polyTriEnd   = new Uint16Array(polyCount);
        this.polyHasDetail = new Uint8Array(polyCount);
        for (let i = 0; i < polyCount; i++) {
            const baseTri  = detailMesh.baseTri[i];
            const triCount = detailMesh.triCount[i];
            this.polyHasDetail[i] = (triCount > 0) ? 1 : 0;
            this.polyTriStart[i] = baseTri;
            this.polyTriEnd[i]   = baseTri + triCount; // [start, end)
        }
        this.triAabbMinX = new Float32Array(detailMesh.trislength);
        this.triAabbMinY = new Float32Array(detailMesh.trislength);
        this.triAabbMaxX = new Float32Array(detailMesh.trislength);
        this.triAabbMaxY = new Float32Array(detailMesh.trislength);
        this.vert=new Array();
        this.tris=new Array();
        const { verts, tris } = detailMesh;
        for(let i=0;i<detailMesh.vertslength;i++)
        {
            this.vert[i]={x: verts[i * 3], y: verts[i * 3 + 1], z: verts[i * 3 + 2]};
        }
        for (let i = 0; i < detailMesh.trislength; i++) {
            this.tris[i] = { a: tris[i * 3], b: tris[i * 3 + 1], c: tris[i * 3 + 2] };

            const { a, b, c } = this.tris[i];
            const minX = Math.min(this.vert[a].x, this.vert[b].x, this.vert[c].x);
            const minY = Math.min(this.vert[a].y, this.vert[b].y, this.vert[c].y);
            const maxX = Math.max(this.vert[a].x, this.vert[b].x, this.vert[c].x);
            const maxY = Math.max(this.vert[a].y, this.vert[b].y, this.vert[c].y);

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
            //let preh=curr.pos.z;
            //let prep=curr;
            for (let j = (curr.mode == PathState.JUMP)?1:0; j < samples.length; j++) {
                const p = samples[j];
                // 跳过重复首点
                //if (result.length > 0) {
                //    const last = result[result.length - 1].pos;
                //    if (posDistance2Dsqr(last, p) < 1e-4) continue;
                //}
                //const preid=polyIndex;
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
                //preh=p.z;
                //prep=result[result.length - 1];
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
        const vert=this.vert;
        const tri=this.tris;
        const start = this.polyTriStart[polyId];
        const end   = this.polyTriEnd[polyId];
        if (this.polyHasDetail[polyId] === 0) return p.z;
        for (let i = start; i < end; i++) {
            if (
                p.x < this.triAabbMinX[i] || p.x > this.triAabbMaxX[i] ||
                p.y < this.triAabbMinY[i] || p.y > this.triAabbMaxY[i]
            ) {
                continue;
            }
            if (this._pointInTriXY(p, vert[tri[i].a], vert[tri[i].b], vert[tri[i].c])) {
                return this._baryHeight(p, vert[tri[i].a], vert[tri[i].b], vert[tri[i].c]);
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
        const start = this.navMesh.polys[polyId * 2];
        const end = this.navMesh.polys[polyId * 2 + 1];
        return pointInConvexPolyXY(p, this.navMesh.verts, start, end);
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

        for (let i = startIndex; i <= polyPath.length-1; i++) {
            const polyId = polyPath[i].id;
            const start = this.navMesh.polys[polyId * 2];
            const end = this.navMesh.polys[polyId * 2 + 1];
            const cp = closestPointOnPoly(p, this.navMesh.verts, start, end);
            if (!cp||!cp.in) continue;
            return i;
        }
        return bestIndex;
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("../path_manager").NavMeshMesh} NavMeshMesh */
// 查询所在多边形优化
let spatialCellSize = 128;

// 压缩网格（CSR）
let gridMinX = 0;
let gridMinY = 0;
let gridW = 0;
let gridH = 0;

// 长度 = gridW * gridH
let cellStart = new Uint32Array(0); // 建议长度 N+1，便于取区间
let cellItems = new Int32Array(0);  // 扁平候选 poly 列表
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
     * @param {NavMeshMesh} mesh
     */
    static buildSpatialIndex(mesh) {
        const polyCount = mesh.polyslength;
        if (polyCount <= 0) {
            gridW = gridH = 0;
            cellStart = new Uint32Array(0);
            cellItems = new Int32Array(0);
            return;
        }
        // 假设mesh.polys为TypedArray，每个poly用起止索引
        // mesh.polys: [start0, end0, start1, end1, ...]，verts为flat xyz数组
        const c0x = new Int32Array(polyCount);
        const c1x = new Int32Array(polyCount);
        const c0y = new Int32Array(polyCount);
        const c1y = new Int32Array(polyCount);

        let minCellX = Infinity;
        let minCellY = Infinity;
        let maxCellX = -Infinity;
        let maxCellY = -Infinity;
        // pass1: 每个 poly 的 cell AABB + 全局边界
        for (let i = 0; i < polyCount; i++) {
            const start = mesh.polys[i << 1];
            const end = mesh.polys[(i << 1) + 1];

            let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
            for (let vi = start; vi <= end; vi++) {
                const v3 = vi * 3;
                const x = mesh.verts[v3];
                const y = mesh.verts[v3 + 1];
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x > maxX) maxX = x;
                if (y > maxY) maxY = y;
            }

            const x0 = Math.floor(minX / spatialCellSize);
            const x1 = Math.floor(maxX / spatialCellSize);
            const y0 = Math.floor(minY / spatialCellSize);
            const y1 = Math.floor(maxY / spatialCellSize);

            c0x[i] = x0; c1x[i] = x1;
            c0y[i] = y0; c1y[i] = y1;

            if (x0 < minCellX) minCellX = x0;
            if (y0 < minCellY) minCellY = y0;
            if (x1 > maxCellX) maxCellX = x1;
            if (y1 > maxCellY) maxCellY = y1;
        }

        gridMinX = minCellX;
        gridMinY = minCellY;
        gridW = (maxCellX - minCellX + 1) | 0;
        gridH = (maxCellY - minCellY + 1) | 0;

        const N = gridW * gridH;
        const cellCount = new Uint32Array(N);

        // pass2: 统计每个 cell 的候选数量
        for (let i = 0; i < polyCount; i++) {
            for (let y = c0y[i]; y <= c1y[i]; y++) {
                const row = (y - gridMinY) * gridW;
                for (let x = c0x[i]; x <= c1x[i]; x++) {
                    const idx = row + (x - gridMinX);
                    cellCount[idx]++;
                }
            }
        }

        // prefix sum -> cellStart (N+1)
        cellStart = new Uint32Array(N + 1);
        for (let i = 0; i < N; i++) {
            cellStart[i + 1] = cellStart[i] + cellCount[i];
        }

        cellItems = new Int32Array(cellStart[N]);
        const writePtr = new Uint32Array(cellStart.subarray(0, N));

        // pass3: 写入 poly 索引
        for (let i = 0; i < polyCount; i++) {
            for (let y = c0y[i]; y <= c1y[i]; y++) {
                const row = (y - gridMinY) * gridW;
                for (let x = c0x[i]; x <= c1x[i]; x++) {
                    const idx = row + (x - gridMinX);
                    const w = writePtr[idx]++;
                    cellItems[w] = i;
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
    static findNearestPoly(p, mesh, heightfixer) {
        //Instance.DebugSphere({center:{x:p.x,y:p.y,z:p.z},radius:2,duration:30,color:{r:255,g:255,b:255}});
        if (gridW <= 0 || gridH <= 0 || cellStart.length === 0) {
            return { pos: p, poly: -1 };
        }
        let bestPoly = -1;
        let bestDist = Infinity;
        let bestPos = p;
        const cx = Math.floor(p.x / spatialCellSize);
        const cy = Math.floor(p.y / spatialCellSize);
        for(let ring=0;ring<=1;ring++)
        {
            let inpoly=false;
            for (let i = -ring; i <= ring; i++)
            {
                const x = cx + i;
                if (x < gridMinX || x >= gridMinX + gridW) continue;
                for (let j = -ring; j <= ring; j++) {
                    if(i+j<ring)continue;
                    const y = cy + j;
                    if (y < gridMinY || y >= gridMinY + gridH) continue;
                    const idx = (y - gridMinY) * gridW + (x - gridMinX);
                    const begin = cellStart[idx];
                    const end = cellStart[idx + 1];
                    for (let it = begin; it < end; it++) {
                        const polyIdx = cellItems[it];
                        // TypedArray结构：每个poly用起止索引
                        const start = mesh.polys[polyIdx * 2];
                        const end = mesh.polys[polyIdx * 2 + 1];
                        // 传递顶点索引区间给closestPointOnPoly
                        const cp = closestPointOnPoly(p, mesh.verts, start, end);
                        if (!cp) continue;
                        if (cp.in === true) {
                            const h = heightfixer?._getHeightOnDetail(polyIdx, p);
                            cp.z = h ?? cp.z;
                            inpoly=true;
                        }
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
            if(inpoly)break;
        }
        return { pos: bestPos, poly: bestPoly };
    }
    /**
     * 导出时只保留已使用长度，避免打印大量尾部 0。
     * @param {import("../path_tilemanager").TileData} td
     */
    static _compactTileData(td) {
        return {
            tileId: td.tileId,
            tx: td.tx,
            ty: td.ty,
            mesh: this._compactMesh(td.mesh),
            detail: this._compactDetail(td.detail, td.mesh?.polyslength ?? 0),
            links: this._compactLinks(td.links)
        };
    }

    /**
     * @param {import("../path_manager").NavMeshMesh} mesh
     */
    static _compactMesh(mesh) {
        const polyslength = mesh.polyslength;
        const vertslength = mesh.vertslength;
        const polys = this._typedSlice(mesh.polys, polyslength * 2);
        const verts = this._typedSlice(mesh.verts, vertslength * 3);
        const regions = this._typedSlice(mesh.regions, polyslength);
        /** @type {number[][][]} */
        const neighbors = new Array(polyslength);
        for (let p = 0; p < polyslength; p++) {
            const start = polys[p * 2];
            const end = polys[p * 2 + 1];
            const edgeCount = Math.max(0, end - start + 1);
            const edgeLists = new Array(edgeCount);
            const srcEdges = mesh.neighbors[p];
            for (let e = 0; e < edgeCount; e++) {
                const list = srcEdges[e];
                const count = list[0];
                const used = Math.max(1, count + 1);
                edgeLists[e] = this._typedSlice(list, used);
            }
            neighbors[p] = edgeLists;
        }
        return { verts, vertslength, polys, polyslength, regions, neighbors };
    }

    /**
     * @param {import("../path_manager").NavMeshDetail} detail
     * @param {number} polyCount
     */
    static _compactDetail(detail, polyCount) {
        const vertslength = detail.vertslength;
        const trislength = detail.trislength;
        return {
            verts: this._typedSlice(detail.verts, vertslength * 3),
            vertslength,
            tris: this._typedSlice(detail.tris, trislength * 3),
            trislength,
            triTopoly: this._typedSlice(detail.triTopoly, trislength),
            baseVert: this._typedSlice(detail.baseVert, polyCount),
            vertsCount: this._typedSlice(detail.vertsCount, polyCount),
            baseTri: this._typedSlice(detail.baseTri, polyCount),
            triCount: this._typedSlice(detail.triCount, polyCount)
        };
    }

    /**
     * @param {import("../path_manager").NavMeshLink} links
     */
    static _compactLinks(links) {
        const len = links.length;
        return {
            poly: this._typedSlice(links.poly, len * 2),
            cost: this._typedSlice(links.cost, len),
            type: this._typedSlice(links.type, len),
            pos: this._typedSlice(links.pos, len * 6),
            length: len
        };
    }

    /**
     * TypedArray / Array 按有效长度切片并转普通数组，便于 JSON 紧凑输出。
     * @param {number} usedLen
     * @param {Uint16Array<ArrayBufferLike> | Float32Array<ArrayBufferLike> | Int32Array<ArrayBufferLike> | Int16Array<ArrayBufferLike> | Uint8Array<ArrayBufferLike>} arr
     */
    static _typedSlice(arr, usedLen) {
        const n = Math.max(0, usedLen | 0);
        if (!arr) return [];
        return Array.from(arr.subarray(0, n));
    }

    /**
     * 把导出的普通对象 mesh 恢复为 TypedArray 结构。
     * @param {{
     *  verts: number[],
     *  vertslength: number,
     *  polys: number[],
     *  polyslength: number,
     *  regions?: number[],
     *  neighbors?: number[][][]
     * }} mesh
     * @returns {import("../path_manager").NavMeshMesh}
     */
    static toTypedMesh(mesh) {
        const polyslength = mesh?.polyslength ?? ((mesh?.polys?.length ?? 0) >> 1);
        const vertslength = mesh?.vertslength ?? Math.floor((mesh?.verts?.length ?? 0) / 3);

        const typedPolys = new Int32Array(mesh?.polys ?? []);
        const typedVerts = new Float32Array(mesh?.verts ?? []);
        const typedRegions = new Int16Array(
            (mesh?.regions && mesh.regions.length > 0) ? mesh.regions : new Array(polyslength).fill(0)
        );

        /** @type {Int16Array[][]} */
        const typedNeighbors = new Array(polyslength);
        for (let p = 0; p < polyslength; p++) {
            const start = typedPolys[p << 1];
            const end = typedPolys[(p << 1) + 1];
            const edgeCount = Math.max(0, end - start + 1);
            const srcEdges = mesh?.neighbors?.[p] ?? [];
            const edgeLists = new Array(edgeCount);

            for (let e = 0; e < edgeCount; e++) {
                const srcList = srcEdges[e] ?? [0];
                const count = Math.max(0, srcList[0] | 0);
                const len = Math.max(1, count + 1);
                const out = new Int16Array(len);
                out[0] = count;
                for (let i = 1; i < len && i < srcList.length; i++) {
                    out[i] = srcList[i] | 0;
                }
                edgeLists[e] = out;
            }

            typedNeighbors[p] = edgeLists;
        }

        return {
            verts: typedVerts,
            vertslength,
            polys: typedPolys,
            polyslength,
            regions: typedRegions,
            neighbors: typedNeighbors
        };
    }

    /**
     * 把导出的普通对象 detail 恢复为 TypedArray 结构。
     * @param {{
     *  verts: number[],
     *  vertslength: number,
     *  tris: number[],
     *  trislength: number,
     *  triTopoly: number[],
     *  baseVert: number[],
     *  vertsCount: number[],
     *  baseTri: number[],
     *  triCount: number[]
     * }} detail
     * @returns {import("../path_manager").NavMeshDetail}
     */
    static toTypedDetail(detail) {
        const vertslength = detail?.vertslength ?? Math.floor((detail?.verts?.length ?? 0) / 3);
        const trislength = detail?.trislength ?? Math.floor((detail?.tris?.length ?? 0) / 3);
        return {
            verts: new Float32Array(detail?.verts ?? []),
            vertslength,
            tris: new Uint16Array(detail?.tris ?? []),
            trislength,
            triTopoly: new Uint16Array(detail?.triTopoly ?? []),
            baseVert: new Uint16Array(detail?.baseVert ?? []),
            vertsCount: new Uint16Array(detail?.vertsCount ?? []),
            baseTri: new Uint16Array(detail?.baseTri ?? []),
            triCount: new Uint16Array(detail?.triCount ?? [])
        };
    }

    /**
     * 把导出的普通对象 links 恢复为 TypedArray 结构。
     * @param {{
     *  poly: number[],
     *  cost: number[],
     *  type: number[],
     *  pos: number[],
     *  length: number
     * }} links
     * @returns {import("../path_manager").NavMeshLink}
     */
    static toTypedLinks(links) {
        const length = links?.length ?? Math.min(
            Math.floor((links?.poly?.length ?? 0) / 2),
            links?.cost?.length ?? 0,
            links?.type?.length ?? 0,
            Math.floor((links?.pos?.length ?? 0) / 6)
        );
        return {
            poly: new Uint16Array(links?.poly ?? []),
            cost: new Float32Array(links?.cost ?? []),
            type: new Uint8Array(links?.type ?? []),
            pos: new Float32Array(links?.pos ?? []),
            length
        };
    }
}

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("cs_script/point_script").Vector} Vector */

class PolyGraphAStar {
    /**
    * @param {NavMeshMesh} polys
    * @param {Map<number,import("./path_manager").NavMeshLinkARRAY[]>} links
     * @param {FunnelHeightFixer} heightfixer
     */
    constructor(polys, links, heightfixer) {
        this.mesh = polys;
        this.polyCount = polys.polyslength;
        /**@type {Map<number,import("./path_manager").NavMeshLinkARRAY[]>} */
        this.links = links;
        this.heightfixer = heightfixer;
        //预计算中心点
        this.centers = new Array(this.polyCount);
        for (let i = 0; i < this.polyCount; i++) {
            const startVert = this.mesh.polys[i * 2];
            const endVert = this.mesh.polys[i * 2 + 1];
            let x = 0, y = 0, z = 0;
            for (let vi = startVert; vi <= endVert; vi++) {
                const base = vi * 3;
                x += this.mesh.verts[base];
                y += this.mesh.verts[base + 1];
                z += this.mesh.verts[base + 2];
            }
            const n = endVert - startVert + 1;
            this.centers[i] = {
                x: x / n,
                y: y / n,
                z: z / n
            };
        }
        this.heuristicScale = ASTAR_HEURISTIC_SCALE;
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
            return { start: startPoly.pos, end: endPoly.pos, path: [{ id: endPoly.poly, mode: PathState.WALK }] };
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
            if (neighbors)
            {
                for (let i = 0; i < neighbors.length; i++) {
                    const entry = neighbors[i];
                    if (!entry) continue;
                    const count = entry[0];
                    if (count <= 0) continue;
                    for (let k = 1; k <= count; k++) {
                        const n = entry[k];
                        if (state[n] == 2) continue;
                        const tentative = g[current] + this.distsqr(current, n);
                        if (tentative < g[n]) {
                            parent[n] = current;
                            walkMode[n] = PathState.WALK;
                            g[n] = tentative;
                            const f = tentative + this.distsqr(n, end) * this.heuristicScale;
                            if (state[n] != 1) {
                                open.push(n, f);
                                state[n] = 1;
                            } else open.update(n, f);
                        }
                    }
                }
            }
            const linkSet = this.links.get(current);
            if (!linkSet) continue;
            for (const link of linkSet) {
                let v = -1;
                if (link.PolyA == current) v = link.PolyB;
                else if (link.PolyB == current) v = link.PolyA;
                if (v == -1 || state[v] == 2) continue;
                const moveCost = link.cost;
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
            //for (let li = 0; li < linkSet.length; li++) {
            //    let v = -1;
            //    const a = linkSet.poly[li * 2];
            //    const b = linkSet.poly[li * 2 + 1];
            //    if (a === current) v = b;
            //    else if (b === current) v = a;
            //    if (state[v] == 2) continue;
            //    const moveCost = linkSet.cost[li];
            //    if (g[current] + moveCost < g[v]) {
            //        g[v] = g[current] + moveCost;
            //        const f = g[v] + this.distsqr(v, end) * this.heuristicScale;
            //        parent[v] = current;
            //        walkMode[v] = linkSet.type[li];
            //        if (state[v] != 1) {
            //            open.push(v, f);
            //            state[v] = 1;
            //        }
            //        else open.update(v, f);
            //    }
            //}
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
        this.nodes = new Uint16Array(polyCount);
        this.costs = new Float32Array(polyCount);
        this.index = new Int16Array(polyCount).fill(-1);
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
     * @param {Map<number,import("./path_manager").NavMeshLinkARRAY[]>} links 每个poly映射到typed link容器
     */
    constructor(mesh, centers, links) {
        this.mesh = mesh;
        this.centers = centers;
        /**@type {Map<number,import("./path_manager").NavMeshLinkARRAY[]>} */
        this.links = links;
        //Instance.Msg(this.links.size);
    }
    // 返回 pA 到 pB 的跳点
    /**
    * @param {number} polyA
    * @param {number} polyB
    */
    getlink(polyA, polyB) {
        const linkSet = this.links.get(polyA);
        if (!linkSet) return;
        for (const link of linkSet) {
            if (link.PolyB == polyB) return { start: link.PosB, end: link.PosA };
            if(link.PolyA == polyB)return { start: link.PosA, end: link.PosB };
        }
        //for (let i = 0; i < linkSet.length; i++) {
        //    const a = linkSet.poly[i<<1];
        //    const b = linkSet.poly[(i<<1) + 1];
        //    const posBase = i * 6;
        //    if (a === polyA && b === polyB) {
        //        return {
        //            start: {
        //                x: linkSet.pos[posBase + 3],
        //                y: linkSet.pos[posBase + 4],
        //                z: linkSet.pos[posBase + 5]
        //            },
        //            end: {
        //                x: linkSet.pos[posBase],
        //                y: linkSet.pos[posBase + 1],
        //                z: linkSet.pos[posBase + 2]
        //            }
        //        };
        //    }
        //    if (a === polyB && b === polyA) {
        //        return {
        //            start: {
        //                x: linkSet.pos[posBase],
        //                y: linkSet.pos[posBase + 1],
        //                z: linkSet.pos[posBase + 2]
        //            },
        //            end: {
        //                x: linkSet.pos[posBase + 3],
        //                y: linkSet.pos[posBase + 4],
        //                z: linkSet.pos[posBase + 5]
        //            }
        //        };
        //    }
        //}
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
            if (currPoly.mode !=PathState.WALK)// 到第 i 个多边形需要特殊过渡（跳跃/梯子/传送）
            {
                // 1. 获取跳点坐标信息
                const linkInfo = this.getlink(currPoly.id,prevPoly.id);
                if (!linkInfo)continue;
                const portals = this.buildPortals(polyPath,segmentStartIndex,i-1, currentSegmentStartPos, linkInfo.start, FUNNEL_DISTANCE);
                const smoothedWalk = this.stringPull(portals);
                for (const p of smoothedWalk) ans.push({pos:p,mode:PathState.WALK});
                ans.push({pos:linkInfo.end,mode:currPoly.mode});
                currentSegmentStartPos = linkInfo.end; // 下一段从落地点开始
                segmentStartIndex = i; // 下一段多边形从 currPoly 开始
            }
        }
        const lastPortals = this.buildPortals(polyPath, segmentStartIndex, polyPath.length-1, currentSegmentStartPos, endPos, FUNNEL_DISTANCE);
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
            if (d > 1) {
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
     * @param {number}start
     * @param {number}end
     * @param {Vector} startPos
     * @param {Vector} endPos
     * @param {number} funnelDistance
     */
    buildPortals(polyPath, start, end, startPos, endPos, funnelDistance) {
        const portals = [];

        // 起点
        portals.push({ left: startPos, right: startPos });
        for (let i = start; i < end; i++) {
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
        const startVert = this.mesh.polys[pa * 2];
        const endVert = this.mesh.polys[pa * 2 + 1];
        const vertCount = endVert - startVert + 1;
        if (vertCount <= 0) return;
        const neigh = this.mesh.neighbors[pa];
        if (!neigh) return;

        for (let ei = 0; ei < vertCount; ei++) {
            const entry = neigh[ei];
            if (!entry) continue;
            const count = entry[0] | 0;
            if (count <= 0) continue;
            let connected = false;
            for (let k = 1; k <= count; k++) {
                if (entry[k] === pb) {
                    connected = true;
                    break;
                }
            }
            if (!connected) continue;

            const vi0 = startVert + ei;
            const vi1 = startVert + ((ei + 1) % vertCount);
            const v0 = {
                x: this.mesh.verts[vi0 * 3],
                y: this.mesh.verts[vi0 * 3 + 1],
                z: this.mesh.verts[vi0 * 3 + 2]
            };
            const v1 = {
                x: this.mesh.verts[vi1 * 3],
                y: this.mesh.verts[vi1 * 3 + 1],
                z: this.mesh.verts[vi1 * 3 + 2]
            };

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

class StaticData
{
    constructor()
    {
        this.Data = ``+`{"tiles":[["0_0",{"tileId":"0_0","tx":0,"ty":0,"mesh":{"verts":[-3042,-2717,-416,-2538,-2717,-416,-2538,-2693,-416,-3042,-2693,-416],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[-3042,-2717,-416,-2538,-2717,-416,-2538,-2693,-416,-3042,-2693,-416],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"lengt`
+`h":0}}],["1_0",{"tileId":"1_0","tx":1,"ty":0,"mesh":{"verts":[-2282,-2693,-416,-2538,-2693,-416,-2538,-2717,-416,-2026,-2717,-416,-2026,-2437,-408,-2274,-2437,-408,-2266,-2629,-416,-2026,-2437,-408,-2266,-2629,-416,-2282,-2693,-416,-2026,-2717,-416,-2274,-2421,-24,-2026,-2421,-24,-2026,-2309,-24,-2274,-2309,-24,-2026,-2293,104,-2026,-2285,104,-2274,-2285,104,-2274,-2269,88,-2026,-2269,88,-2026,-2213,88,-2274,-2213,88],"vertslength":22,"polys":[0,3,4,6,7,10,11,14,15,17,18,21],"polyslength":6,"reg`
+`ions":[1,1,1,2,4,3],"neighbors":[[[0],[0],[0],[1,2]],[[0],[0],[1,2]],[[1,1],[0],[1,0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2282,-2693,-416,-2538,-2693,-416,-2538,-2717,-416,-2026,-2717,-416,-2026,-2437,-408,-2274,-2437,-408,-2266,-2629,-416,-2026,-2437,-408,-2266,-2629,-416,-2282,-2693,-416,-2026,-2717,-416,-2274,-2421,-24,-2026,-2421,-24,-2026,-2309,-24,-2274,-2309,-24,-2026,-2293,104,-2026,-2285,104,-2274,-2285,104,-2274,-2269,88,-2026,-2269,88,-2026,-22`
+`13,88,-2274,-2213,88],"vertslength":22,"tris":[0,1,2,0,2,3,4,5,6,8,9,10,7,8,10,14,11,12,12,13,14,15,16,17,21,18,19,19,20,21],"trislength":10,"triTopoly":[0,0,1,2,2,3,3,4,5,5],"baseVert":[0,4,7,11,15,18],"vertsCount":[4,3,4,4,3,4],"baseTri":[0,2,3,5,7,8],"triCount":[2,1,2,2,1,2]},"links":{"poly":[4,5],"cost":[768],"type":[2],"pos":[-2026,-2285,104,-2026,-2269,88],"length":1}}],["2_0",{"tileId":"2_0","tx":2,"ty":0,"mesh":{"verts":[-1906,-2541,-408,-1922,-2517,-408,-2026,-2501,-408,-2026,-2717,-416`
+`,-1898,-2717,-416,-1914,-2333,-408,-1922,-2309,-408,-1970,-2309,-408,-1970,-2429,-408,-1970,-2429,-408,-2026,-2437,-408,-2026,-2501,-408,-1922,-2517,-408,-1890,-2517,-408,-1890,-2325,-408,-1914,-2333,-408,-1970,-2429,-408,-1890,-2517,-408,-1890,-2517,-408,-1906,-2541,-408,-1898,-2717,-416,-1874,-2301,-416,-1890,-2325,-408,-1890,-2517,-408,-1514,-2213,-408,-1874,-2213,-416,-1874,-2301,-416,-1874,-2301,-416,-1890,-2517,-408,-1898,-2717,-416,-1514,-2717,-416,-1514,-2213,-408,-2026,-2309,-24,-2026,-`
+`2421,-24,-1986,-2421,-24,-1986,-2309,-24,-1890,-2213,104,-1898,-2213,104,-1906,-2285,104,-1890,-2293,104,-1906,-2285,104,-2026,-2285,104,-2026,-2293,104,-1890,-2293,104,-2026,-2213,88,-2026,-2269,88,-1914,-2269,88,-1914,-2213,88,-1530,-2717,-504,-1514,-2717,-504,-1514,-2685,-504,-1530,-2685,-504],"vertslength":52,"polys":[0,4,5,8,9,13,14,17,18,20,21,23,24,26,27,31,32,35,36,39,40,43,44,47,48,51],"polyslength":13,"regions":[2,1,1,1,1,1,1,1,5,6,6,4,7],"neighbors":[[[0],[1,2],[0],[0],[1,4]],[[0],[0]`
+`,[0],[1,3]],[[0],[0],[1,0],[0],[1,3]],[[0],[1,1],[1,2],[1,5]],[[0],[1,0],[1,7]],[[0],[1,3],[1,7]],[[0],[0],[1,7]],[[1,5],[1,4],[0],[0],[1,6]],[[0],[0],[0],[0]],[[0],[0],[1,10],[0]],[[0],[0],[0],[1,9]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-1906,-2541,-408,-1922,-2517,-408,-2026,-2501,-408,-2026,-2544.199951171875,-408,-2026,-2565.800048828125,-416,-2026,-2717,-416,-1898,-2717,-416,-1905,-2563,-416,-1914,-2333,-408,-1922,-2309,-408,-1970,-2309,-408,-1970,-2429,-408,-1970,-2429,`
+`-408,-2026,-2437,-408,-2026,-2501,-408,-1922,-2517,-408,-1890,-2517,-416,-1906,-2499.39990234375,-408,-1890,-2325,-416,-1914,-2333,-408,-1970,-2429,-408,-1906,-2499.39990234375,-408,-1890,-2517,-416,-1910,-2481,-408,-1890,-2517,-416,-1906,-2541,-408,-1905,-2563,-416,-1898,-2717,-416,-1891.77783203125,-2561.4443359375,-416,-1890.888916015625,-2539.22216796875,-408,-1874,-2301,-416,-1890,-2325,-416,-1890,-2517,-416,-1514,-2213,-408,-1874,-2213,-416,-1874,-2301,-416,-1874,-2301,-416,-1890,-2517,-41`
+`6,-1890.888916015625,-2539.22216796875,-408,-1891.77783203125,-2561.4443359375,-416,-1898,-2717,-416,-1514,-2717,-416,-1514,-2258.818115234375,-416,-1514,-2213,-408,-1526,-2249,-408,-2026,-2309,-24,-2026,-2421,-24,-1986,-2421,-24,-1986,-2309,-24,-1890,-2213,104,-1898,-2213,104,-1906,-2285,104,-1890,-2293,104,-1906,-2285,104,-2026,-2285,104,-2026,-2293,104,-1890,-2293,104,-2026,-2213,88,-2026,-2269,88,-1914,-2269,88,-1914,-2213,88,-1530,-2717,-504,-1514,-2717,-504,-1514,-2685,-504,-1530,-2685,-50`
+`4],"vertslength":65,"tris":[7,0,1,1,2,3,1,3,4,7,1,4,7,4,5,5,6,7,8,9,10,8,10,11,15,16,17,12,13,14,15,17,12,12,14,15,22,18,23,18,19,23,20,19,23,20,21,23,22,21,23,29,24,25,28,29,25,28,25,26,26,27,28,30,31,32,33,34,35,38,39,40,37,38,40,36,37,40,36,40,41,36,41,42,42,43,44,43,36,44,36,42,44,48,45,46,46,47,48,49,50,51,49,51,52,53,54,55,53,55,56,60,57,58,58,59,60,64,61,62,62,63,64],"trislength":41,"triTopoly":[0,0,0,0,0,0,1,1,2,2,2,2,3,3,3,3,3,4,4,4,4,5,6,7,7,7,7,7,7,7,7,8,8,9,9,10,10,11,11,12,12],"base`
+`Vert":[0,8,12,18,24,30,33,36,45,49,53,57,61],"vertsCount":[8,4,6,6,6,3,3,9,4,4,4,4,4],"baseTri":[0,6,8,12,17,21,22,23,31,33,35,37,39],"triCount":[6,2,4,5,4,1,1,8,2,2,2,2,2]},"links":{"poly":[9,11],"cost":[525.6585083007812],"type":[2],"pos":[-1904.3414306640625,-2270.0732421875,104,-1914,-2269,88],"length":1}}],["3_0",{"tileId":"3_0","tx":3,"ty":0,"mesh":{"verts":[-1090,-2661,-504,-1114,-2589,-504,-1258,-2589,-504,-1266,-2685,-504,-1242,-2717,-504,-1266,-2685,-504,-1514,-2685,-504,-1514,-2717,-5`
+`04,-1242,-2717,-504,-1002,-2213,-416,-1066,-2213,-408,-1074,-2237,-408,-1002,-2533,-416,-1002,-2533,-416,-1074,-2237,-408,-1098,-2213,-408,-1514,-2213,-408,-1514,-2717,-416,-1002,-2717,-202,-1002,-2549,-190,-1458,-2709,-190,-1002,-2717,-459,-1002,-2581,-419,-1106,-2613,-418,-1002,-2717,-459,-1106,-2613,-418,-1138,-2605,-416,-1434,-2709,-416,-1426,-2685,-209,-1402,-2669,-210,-1426,-2669,-214,-1098,-2581,-504,-1114,-2589,-504,-1090,-2661,-504,-1002,-2637,-504,-1002,-2525,-504,-1106,-2517,-504,-109`
+`8,-2581,-504,-1002,-2637,-504],"vertslength":39,"polys":[0,4,5,8,9,12,13,17,18,20,21,23,24,27,28,30,31,34,35,38],"polyslength":10,"regions":[4,4,1,1,2,3,3,6,5,5],"neighbors":[[[1,8],[0],[0],[1,1],[0]],[[0],[0],[0],[1,0]],[[0],[0],[1,3],[0]],[[1,2],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,6]],[[1,5],[0],[0],[0]],[[0],[0],[0]],[[0],[1,0],[0],[1,9]],[[0],[0],[1,8],[0]]]},"detail":{"verts":[-1090,-2661,-504,-1114,-2589,-504,-1258,-2589,-504,-1266,-2685,-504,-1242,-2717,-504,-1266,-2685,-504,-1514,`
+`-2685,-504,-1514,-2717,-504,-1242,-2717,-504,-1002,-2213,-416,-1044.6666259765625,-2213,-416,-1066,-2213,-408,-1074,-2237,-408,-1068.4615478515625,-2259.769287109375,-416,-1002,-2533,-416,-1002,-2533,-416,-1068.4615478515625,-2259.769287109375,-416,-1074,-2237,-408,-1098,-2213,-408,-1514,-2213,-408,-1514,-2258.818115234375,-416,-1514,-2717,-416,-1502,-2249,-408,-1002,-2717,-202,-1002,-2570,-202,-1002,-2549,-190,-1458,-2709,-190,-1435.199951171875,-2709.39990234375,-190,-1412.4000244140625,-2709.`
+`800048828125,-202,-1398,-2705,-202,-1002,-2717,-457,-1002,-2581,-419,-1106,-2613,-419,-1002,-2717,-457,-1106,-2613,-419,-1138,-2605,-416,-1434,-2709,-416,-1278,-2681,-416,-1426,-2685,-212,-1402,-2669,-210,-1426,-2669,-212,-1098,-2581,-504,-1114,-2589,-504,-1090,-2661,-504,-1002,-2637,-504,-1002,-2525,-504,-1106,-2517,-504,-1098,-2581,-504,-1002,-2637,-504],"vertslength":49,"tris":[2,3,4,0,1,2,0,2,4,5,6,7,5,7,8,10,11,12,10,12,13,9,10,13,9,13,14,16,17,18,18,19,22,19,20,22,20,21,22,15,21,22,18,16,2`
+`2,15,16,22,23,24,29,24,25,29,26,25,29,26,27,29,23,28,29,27,28,29,30,31,32,35,36,37,36,33,37,33,34,37,35,34,37,38,39,40,41,42,43,41,43,44,45,46,47,45,47,48],"trislength":32,"triTopoly":[0,0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,4,4,4,4,4,4,5,6,6,6,6,7,8,8,9,9],"baseVert":[0,5,9,15,23,30,33,38,41,45],"vertsCount":[5,4,6,8,7,3,5,3,4,4],"baseTri":[0,3,5,9,16,22,23,27,28,30],"triCount":[3,2,4,7,6,1,4,1,2,2]},"links":{"poly":[4,7],"cost":[759.361328125],"type":[2],"pos":[-1422.0098876953125,-2696.371826171875,-`
+`190,-1426,-2685,-209],"length":1}}],["4_0",{"tileId":"4_0","tx":4,"ty":0,"mesh":{"verts":[-858,-2717,-470,-874,-2677,-459,-938,-2685,-455,-1002,-2717,-460,-938,-2685,-455,-970,-2581,-422,-1002,-2581,-420,-1002,-2717,-460,-970,-2549,-190,-1002,-2549,-190,-1002,-2717,-202,-914,-2717,-190,-946,-2629,-504,-906,-2645,-504,-890,-2637,-504,-890,-2589,-504,-1002,-2637,-504,-946,-2629,-504,-890,-2589,-504,-946,-2517,-504,-1002,-2525,-504,-890,-2589,-504,-834,-2605,-504,-754,-2581,-504,-490,-2341,-504,-54`
+`6,-2333,-504,-546,-2453,-504,-490,-2557,-504,-954,-2445,-504,-946,-2517,-504,-890,-2589,-504,-754,-2581,-504,-490,-2557,-504,-546,-2453,-504,-1002,-2533,-416,-930,-2541,-416,-770,-2501,-413,-1002,-2213,-416,-770,-2501,-413,-490,-2477,-413,-490,-2213,-416,-1002,-2213,-416,-906,-2709,-416,-858,-2717,-416,-858,-2701,-416,-914,-2557,-416,-954,-2573,-416,-762,-2525,-419,-890,-2557,-421,-834,-2717,-472,-490,-2717,-480,-490,-2501,-420],"vertslength":52,"polys":[0,3,4,7,8,11,12,15,16,20,21,23,24,27,28,3`
+`3,34,37,38,41,42,46,47,51],"polyslength":12,"regions":[5,5,4,3,3,3,3,3,1,1,7,2],"neighbors":[[[0],[0],[1,1],[0]],[[0],[0],[0],[1,0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,4]],[[0],[1,3],[1,7],[0],[0]],[[0],[0],[1,7]],[[0],[0],[1,7],[0]],[[0],[1,4],[1,5],[0],[1,6],[0]],[[0],[0],[1,9],[0]],[[0],[0],[0],[1,8]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]]]},"detail":{"verts":[-858,-2717,-467,-874,-2677,-459,-938,-2685,-458,-1002,-2717,-458,-938,-2685,-458,-950.7999877929688,-2643.39990234375,-438,-970,-2`
+`581,-422,-1002,-2581,-421,-1002,-2717,-458,-970,-2549,-190,-1002,-2549,-190,-1002,-2570,-202,-1002,-2717,-202,-958,-2717,-202,-936,-2717,-190,-914,-2717,-190,-966,-2633,-190,-946,-2629,-504,-906,-2645,-504,-890,-2637,-504,-890,-2589,-504,-1002,-2637,-504,-946,-2629,-504,-890,-2589,-504,-946,-2517,-504,-1002,-2525,-504,-890,-2589,-504,-834,-2605,-504,-754,-2581,-504,-490,-2341,-504,-546,-2333,-504,-546,-2453,-504,-490,-2557,-504,-954,-2445,-504,-946,-2517,-504,-890,-2589,-504,-754,-2581,-504,-490`
+`,-2557,-504,-546,-2453,-504,-1002,-2533,-416,-930,-2541,-416,-770,-2501,-416,-1002,-2213,-416,-770,-2501,-416,-490,-2477,-416,-490,-2389,-409,-490,-2213,-416,-1002,-2213,-416,-906,-2709,-416,-858,-2717,-416,-858,-2701,-416,-914,-2557,-416,-954,-2573,-416,-762,-2525,-419,-890,-2557,-419,-834,-2717,-470,-490,-2717,-480,-490,-2501,-420,-590,-2705,-479],"vertslength":59,"tris":[0,1,2,0,2,3,5,6,7,8,4,5,5,7,8,9,10,11,14,15,16,14,13,16,11,12,16,13,12,16,15,9,16,11,9,16,17,18,19,17,19,20,21,22,23,23,24,`
+`25,21,23,25,26,27,28,29,30,31,29,31,32,33,34,35,33,35,36,36,37,38,33,36,38,39,40,41,39,41,42,43,44,45,43,45,46,43,46,47,48,49,50,51,52,48,48,50,51,53,54,55,53,55,58,55,56,58,56,57,58,53,57,58],"trislength":37,"triTopoly":[0,0,1,1,1,2,2,2,2,2,2,2,3,3,4,4,4,5,6,6,7,7,7,7,8,8,9,9,9,10,10,10,11,11,11,11,11],"baseVert":[0,4,9,17,21,26,29,33,39,43,48,53],"vertsCount":[4,5,8,4,5,3,4,6,4,5,5,6],"baseTri":[0,2,5,12,14,17,18,20,24,26,29,32],"triCount":[2,3,7,2,3,1,2,4,2,3,3,5]},"links":{"poly":[],"cost":[`
+`],"type":[],"pos":[],"length":0}}],["5_0",{"tileId":"5_0","tx":5,"ty":0,"mesh":{"verts":[-282,-2485,-418,-490,-2501,-420,-490,-2717,-480,22,-2717,-480,22,-2493,-420,22,-2557,-504,22,-2325,-504,-34,-2333,-504,-34,-2333,-504,-42,-2269,-504,-306,-2269,-504,-306,-2341,-504,-306,-2341,-504,-490,-2341,-504,-490,-2557,-504,22,-2557,-504,-34,-2333,-504,-306,-2341,-504,-490,-2557,-504,-426,-2253,-416,-410,-2213,-416,-490,-2213,-416,-138,-2461,-416,-146,-2269,-410,-178,-2245,-414,-386,-2277,-414,-490,-247`
+`7,-413,-154,-2213,-414,-386,-2213,-416,-370,-2229,-416,-178,-2245,-414,-178,-2245,-414,-370,-2229,-416,-386,-2277,-414,-426,-2253,-416,-490,-2213,-416,-490,-2477,-413,-386,-2277,-414,-426,-2253,-416,-490,-2477,-413,-386,-2229,-366,-410,-2229,-366,-410,-2253,-366,-386,-2261,-366,-106,-2213,-414,-146,-2213,-414,-122,-2229,-414,-106,-2213,-414,-122,-2229,-414,-122,-2253,-413,22,-2253,-414,22,-2253,-414,-122,-2253,-413,-146,-2269,-410,-138,-2461,-416,22,-2469,-413,22,-2221,-416,22,-2213,-416,6,-2213`
+`,-416,22,-2389,-768,22,-2213,-768,14,-2213,-768],"vertslength":62,"polys":[0,4,5,7,8,11,12,14,15,18,19,21,22,26,27,30,31,33,34,36,37,39,40,43,44,46,47,50,51,55,56,58,59,61],"polyslength":17,"regions":[3,1,1,1,1,2,2,2,2,2,2,5,4,4,4,6,7],"neighbors":[[[0],[0],[0],[0],[0]],[[0],[0],[1,4]],[[0],[0],[0],[1,4]],[[0],[0],[1,4]],[[1,1],[1,2],[1,3],[0]],[[0],[0],[1,9]],[[1,14],[0],[1,8],[1,10],[0]],[[0],[0],[1,8],[0]],[[1,7],[0],[1,6]],[[1,5],[0],[1,10]],[[0],[1,9],[1,6]],[[0],[0],[0],[0]],[[0],[0],[1,13`
+`]],[[1,12],[0],[1,14],[0]],[[1,13],[0],[1,6],[0],[0]],[[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[-282,-2485,-418,-490,-2501,-420,-490,-2717,-480,22,-2717,-480,22,-2493,-420,22,-2557,-504,22,-2325,-504,-34,-2333,-504,-34,-2333,-504,-42,-2269,-504,-306,-2269,-504,-306,-2341,-504,-306,-2341,-504,-490,-2341,-504,-490,-2557,-504,22,-2557,-504,-34,-2333,-504,-306,-2341,-504,-490,-2557,-504,-426,-2253,-416,-410,-2213,-416,-490,-2213,-416,-138,-2461,-416,-144.22222900390625,-2311.666748046875,-404`
+`,-146,-2269,-410,-178,-2245,-414,-386,-2277,-414,-438,-2377,-408,-490,-2477,-416,-166,-2321,-404,-154,-2213,-414,-386,-2213,-416,-370,-2229,-416,-178,-2245,-414,-178,-2245,-414,-370,-2229,-416,-386,-2277,-414,-426,-2253,-416,-490,-2213,-416,-490,-2367,-409,-490,-2477,-416,-458,-2365,-408,-386,-2277,-414,-426,-2253,-416,-458,-2365,-408,-490,-2477,-416,-438,-2377,-408,-386,-2229,-366,-410,-2229,-366,-410,-2253,-366,-386,-2261,-366,-106,-2213,-414,-146,-2213,-414,-122,-2229,-414,-106,-2213,-414,-12`
+`2,-2229,-414,-122,-2253,-415,22,-2253,-414,22,-2253,-414,-122,-2253,-415,-146,-2269,-413,-144.22222900390625,-2311.666748046875,-404,-138,-2461,-416,22,-2469,-416,22,-2339.39990234375,-405,22,-2221,-416,22,-2213,-416,6,-2213,-416,22,-2389,-768,22,-2213,-768,14,-2213,-768],"vertslength":71,"tris":[0,1,2,3,4,0,0,2,3,5,6,7,8,9,10,8,10,11,12,13,14,15,16,17,15,17,18,19,20,21,27,28,22,27,22,29,22,23,29,25,24,29,23,24,29,25,26,29,27,26,29,31,32,33,30,31,33,34,35,36,39,40,41,39,41,37,37,38,39,44,45,46,4`
+`4,46,42,42,43,44,47,48,49,47,49,50,51,52,53,54,55,56,54,56,57,59,60,61,58,59,61,64,58,61,62,63,64,61,62,64,65,66,67,68,69,70],"trislength":38,"triTopoly":[0,0,0,1,2,2,3,4,4,5,6,6,6,6,6,6,6,7,7,8,9,9,9,10,10,10,11,11,12,13,13,14,14,14,14,14,15,16],"baseVert":[0,5,8,12,15,19,22,30,34,37,42,47,51,54,58,65,68],"vertsCount":[5,3,4,3,4,3,8,4,3,5,5,4,3,4,7,3,3],"baseTri":[0,3,4,6,7,9,10,17,19,20,23,26,28,29,31,36,37],"triCount":[3,1,2,1,2,1,7,2,1,3,3,2,1,2,5,1,1]},"links":{"poly":[5,11],"cost":[3802.96`
+`5576171875],"type":[2],"pos":[-415.5172424316406,-2226.793212890625,-416,-410,-2229,-366],"length":1}}],["6_0",{"tileId":"6_0","tx":6,"ty":0,"mesh":{"verts":[534,-2717,-466,534,-2549,-416,286,-2509,-418,22,-2493,-420,22,-2717,-480,166,-2373,-504,166,-2325,-504,22,-2325,-504,214,-2565,-504,198,-2373,-504,166,-2373,-504,166,-2373,-504,22,-2325,-504,22,-2557,-504,214,-2565,-504,22,-2469,-413,414,-2501,-413,422,-2389,-414,22,-2261,-414,38,-2213,-768,22,-2213,-768,22,-2389,-768,46,-2389,-768,22,-2389`
+`,-768,246,-2397,-768,46,-2389,-768,534,-2389,-416,534,-2293,-416,494,-2285,-416,22,-2221,-416,494,-2213,-416,22,-2213,-416,22,-2221,-416,494,-2285,-416,62,-2365,-752,534,-2365,-752,534,-2213,-752,62,-2213,-752,446,-2373,-544,454,-2389,-544,534,-2373,-544,318,-2333,-544,326,-2349,-544,446,-2373,-544,446,-2373,-544,534,-2373,-544,534,-2269,-544,118,-2269,-544,318,-2333,-544,326,-2421,-504,326,-2373,-504,198,-2373,-504,342,-2429,-504,326,-2421,-504,198,-2373,-504,214,-2565,-504,350,-2581,-504,534,-`
+`2613,-504,534,-2565,-504,470,-2557,-504,350,-2581,-504,470,-2557,-504,478,-2421,-504,342,-2429,-504,350,-2581,-504,414,-2501,-413,534,-2525,-413,534,-2421,-414,422,-2389,-414,510,-2277,-288,534,-2277,-288,534,-2213,-288,510,-2213,-288,534,-2405,-544,534,-2381,-544,518,-2389,-544,534,-2261,-415,534,-2213,-415,526,-2213,-415],"vertslength":79,"polys":[0,4,5,7,8,10,11,14,15,18,19,22,23,25,26,29,30,33,34,37,38,40,41,43,44,48,49,51,52,56,57,60,61,64,65,68,69,72,73,75,76,78],"polyslength":21,"regions"`
+`:[1,2,2,2,4,10,10,7,7,6,9,9,9,3,3,5,5,8,13,14,15],"neighbors":[[[0],[0],[0],[0],[0]],[[0],[0],[1,3]],[[1,14],[0],[1,3]],[[1,1],[0],[0],[1,2]],[[0],[1,17],[0],[0]],[[0],[0],[1,6],[0]],[[0],[0],[1,5]],[[0],[0],[1,8],[0]],[[0],[0],[1,7],[0]],[[0],[0],[0],[0]],[[0],[0],[1,12]],[[0],[0],[1,12]],[[1,10],[0],[0],[0],[1,11]],[[0],[0],[1,14]],[[0],[1,13],[1,2],[0],[1,16]],[[0],[0],[1,16],[0]],[[0],[0],[1,14],[1,15]],[[0],[0],[0],[1,4]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[53`
+`4,-2717,-464,534,-2549,-416,286,-2509,-418,22,-2493,-420,22,-2717,-480,490,-2561,-416,166,-2373,-504,166,-2325,-504,22,-2325,-504,214,-2565,-504,198,-2373,-504,166,-2373,-504,166,-2373,-504,22,-2325,-504,22,-2557,-504,214,-2565,-504,22,-2469,-416,414,-2501,-416,422,-2389,-414,22,-2261,-414,22,-2353.4443359375,-406,38,-2213,-768,22,-2213,-768,22,-2389,-768,46,-2389,-768,22,-2389,-768,246,-2397,-768,46,-2389,-768,534,-2389,-416,534,-2293,-416,494,-2285,-416,22,-2221,-416,494,-2213,-416,22,-2213,-4`
+`16,22,-2221,-416,494,-2285,-416,62,-2365,-752,534,-2365,-752,534,-2213,-752,62,-2213,-752,446,-2373,-544,454,-2389,-544,534,-2373,-544,318,-2333,-544,326,-2349,-544,446,-2373,-544,446,-2373,-544,534,-2373,-544,534,-2269,-544,118,-2269,-544,318,-2333,-544,326,-2421,-504,326,-2373,-504,198,-2373,-504,342,-2429,-504,326,-2421,-504,198,-2373,-504,214,-2565,-504,350,-2581,-504,534,-2613,-504,534,-2565,-504,470,-2557,-504,350,-2581,-504,470,-2557,-504,478,-2421,-504,342,-2429,-504,350,-2581,-504,414,-`
+`2501,-416,534,-2525,-416,534,-2421,-414,422,-2389,-414,510,-2277,-288,534,-2277,-288,534,-2213,-288,510,-2213,-288,534,-2405,-544,534,-2381,-544,518,-2389,-544,534,-2261,-415,534,-2213,-415,526,-2213,-415],"vertslength":81,"tris":[2,3,4,0,2,4,0,1,5,1,2,5,2,0,5,6,7,8,9,10,11,12,13,14,12,14,15,16,17,18,18,19,20,16,18,20,21,22,23,21,23,24,25,26,27,28,29,30,28,30,31,32,33,34,32,34,35,39,36,37,37,38,39,40,41,42,43,44,45,46,47,48,50,46,48,48,49,50,51,52,53,54,55,56,57,58,54,54,56,57,59,60,61,59,61,62,`
+`63,64,65,63,65,66,67,68,69,67,69,70,74,71,72,72,73,74,75,76,77,78,79,80],"trislength":40,"triTopoly":[0,0,0,0,0,1,2,3,3,4,4,4,5,5,6,7,7,8,8,9,9,10,11,12,12,12,13,14,14,14,15,15,16,16,17,17,18,18,19,20],"baseVert":[0,6,9,12,16,21,25,28,32,36,40,43,46,51,54,59,63,67,71,75,78],"vertsCount":[6,3,3,4,5,4,3,4,4,4,3,3,5,3,5,4,4,4,4,3,3],"baseTri":[0,5,6,7,9,12,14,15,17,19,21,22,23,26,27,30,32,34,36,38,39],"triCount":[5,1,1,2,3,2,1,2,2,2,1,1,3,1,3,2,2,2,2,1,1]},"links":{"poly":[10,19,11,13],"cost":[92.3`
+`0769348144531,3230.769287109375],"type":[1,2],"pos":[532.4615478515625,-2373.3076171875,-544,534,-2381,-544,330.6153869628906,-2349.923095703125,-544,326,-2373,-504],"length":2}}],["7_0",{"tileId":"7_0","tx":7,"ty":0,"mesh":{"verts":[534,-2717,-466,790,-2717,-451,782,-2597,-419,534,-2549,-416,790,-2661,-504,870,-2717,-504,1046,-2717,-504,534,-2565,-504,534,-2613,-504,790,-2661,-504,790,-2661,-504,1046,-2717,-504,1046,-2557,-504,534,-2565,-504,814,-2573,-416,814,-2541,-416,790,-2501,-413,534,-242`
+`1,-414,534,-2525,-416,534,-2269,-544,534,-2405,-544,1030,-2405,-544,1030,-2269,-544,534,-2397,-768,750,-2397,-768,534,-2389,-768,782,-2301,-416,534,-2293,-416,534,-2389,-416,798,-2477,-416,1046,-2213,-416,790,-2213,-416,782,-2301,-416,798,-2477,-416,1046,-2477,-416,534,-2213,-752,534,-2365,-752,966,-2365,-752,966,-2213,-752,774,-2213,-288,534,-2213,-288,534,-2277,-288,774,-2285,-288,758,-2213,-415,534,-2213,-415,534,-2261,-415,758,-2269,-415,1022,-2389,-768,1022,-2213,-768,998,-2213,-768,990,-23`
+`89,-768,990,-2389,-768,790,-2397,-768,1022,-2389,-768,814,-2541,-416,814,-2573,-416,830,-2581,-416,838,-2525,-416,838,-2525,-416,830,-2581,-416,830,-2717,-416,1046,-2717,-416,1046,-2509,-416,1046,-2509,-416,830,-2509,-416,838,-2525,-416],"vertslength":66,"polys":[0,3,4,6,7,9,10,13,14,18,19,22,23,25,26,29,30,34,35,38,39,42,43,46,47,50,51,53,54,57,58,62,63,65],"polyslength":17,"regions":[4,3,3,3,7,6,10,1,1,5,8,9,11,11,2,2,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,3]],[[0],[0],[1,3]],[[1,1],[0]`
+`,[0],[1,2]],[[1,14],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,8]],[[0],[0],[1,7],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,13]],[[0],[0],[1,12]],[[1,4],[0],[1,15],[0]],[[1,14],[0],[0],[0],[1,16]],[[0],[0],[1,15]]]},"detail":{"verts":[534,-2717,-463,790,-2717,-449,782,-2597,-419,534,-2549,-416,790,-2661,-504,870,-2717,-504,1046,-2717,-504,534,-2565,-504,534,-2613,-504,790,-2661,-504,790,-2661,-504,1046,-2717,-504,1046,-2557,-504,534,-256`
+`5,-504,814,-2573,-416,814,-2541,-416,790,-2501,-413,534,-2421,-414,534,-2525,-416,534,-2269,-544,534,-2405,-544,1030,-2405,-544,1030,-2269,-544,534,-2397,-768,750,-2397,-768,534,-2389,-768,782,-2301,-416,534,-2293,-416,534,-2389,-416,798,-2477,-416,1046,-2213,-416,790,-2213,-416,782,-2301,-416,798,-2477,-416,1046,-2477,-416,534,-2213,-752,534,-2365,-752,966,-2365,-752,966,-2213,-752,774,-2213,-288,534,-2213,-288,534,-2277,-288,774,-2285,-288,758,-2213,-415,534,-2213,-415,534,-2261,-415,758,-2269`
+`,-415,1022,-2389,-768,1022,-2213,-768,998,-2213,-768,990,-2389,-768,990,-2389,-768,790,-2397,-768,1022,-2389,-768,814,-2541,-416,814,-2573,-416,830,-2581,-416,838,-2525,-416,838,-2525,-416,830,-2581,-416,830,-2717,-416,1046,-2717,-416,1046,-2509,-416,1046,-2509,-416,830,-2509,-416,838,-2525,-416],"vertslength":66,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,10,11,12,10,12,13,14,15,16,18,14,16,16,17,18,22,19,20,20,21,22,23,24,25,26,27,28,26,28,29,31,32,33,33,34,30,30,31,33,38,35,36,36,37,38,39,40,41,39,41,42,`
+`43,44,45,43,45,46,47,48,49,47,49,50,51,52,53,54,55,56,54,56,57,58,59,60,58,60,61,58,61,62,63,64,65],"trislength":32,"triTopoly":[0,0,1,2,3,3,4,4,4,5,5,6,7,7,8,8,8,9,9,10,10,11,11,12,12,13,14,14,15,15,15,16],"baseVert":[0,4,7,10,14,19,23,26,30,35,39,43,47,51,54,58,63],"vertsCount":[4,3,3,4,5,4,3,4,5,4,4,4,4,3,4,5,3],"baseTri":[0,2,3,4,6,9,11,12,14,17,19,21,23,25,26,28,31],"triCount":[2,1,1,2,3,2,1,2,3,2,2,2,2,1,2,3,1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["8_0",{"tileId"`
+`:"8_0","tx":8,"ty":0,"mesh":{"verts":[1046,-2557,-504,1046,-2717,-504,1558,-2717,-504,1558,-2557,-504,1046,-2717,-416,1062,-2717,-416,1070,-2637,-416,1046,-2509,-416,1238,-2693,-412,1246,-2565,-408,1230,-2557,-415,1070,-2637,-416,1230,-2557,-415,1230,-2509,-414,1046,-2509,-416,1070,-2637,-416,1286,-2357,-416,1302,-2349,-416,1254,-2285,-415,1230,-2277,-416,1558,-2213,-640,1470,-2213,-640,1518,-2229,-637,1294,-2373,-416,1286,-2357,-416,1230,-2277,-416,1046,-2213,-416,1046,-2477,-416,1238,-2445,-41`
+`5,1558,-2213,-640,1518,-2229,-637,1510,-2373,-548,1558,-2445,-560,1046,-2477,-416,1230,-2477,-416,1238,-2445,-415,1230,-2277,-416,1230,-2213,-408,1046,-2213,-416,1558,-2445,-560,1510,-2373,-548,1294,-2373,-416,1238,-2445,-415,1102,-2717,-427,1206,-2709,-419,1102,-2685,-420,1246,-2565,-408,1238,-2693,-412,1294,-2717,-412,1558,-2717,-414,1558,-2565,-415,1270,-2277,-416,1254,-2285,-415,1302,-2349,-416,1262,-2213,-416,1270,-2277,-416,1302,-2349,-416,1478,-2325,-416,1494,-2213,-416,1302,-2349,-416,14`
+`94,-2349,-416,1478,-2325,-416,1406,-2533,-164,1390,-2477,-164,1342,-2461,-164,1342,-2461,-164,1342,-2405,-164,1262,-2397,-164,1342,-2461,-164,1262,-2397,-164,1262,-2533,-164,1406,-2533,-164,1262,-2397,-164,1342,-2405,-164,1350,-2389,-164,1262,-2213,-164,1414,-2389,-164,1430,-2437,-164,1494,-2437,-164,1494,-2213,-164,1262,-2213,-164,1350,-2389,-164,1414,-2389,-164,1494,-2213,-164,1318,-2373,-560,1318,-2445,-560,1422,-2445,-560,1422,-2373,-560,1366,-2413,-118,1366,-2445,-118,1398,-2445,-118,1398,-`
+`2413,-118,1430,-2437,-164,1390,-2477,-164,1406,-2533,-164,1494,-2533,-164,1494,-2437,-164,1518,-2469,-176,1558,-2469,-176,1558,-2365,-176,1518,-2365,-176,1558,-2341,-640,1558,-2333,-640,1518,-2333,-640,1526,-2541,-88,1558,-2541,-88,1558,-2213,-88,1518,-2213,-88,1542,-2333,-168,1558,-2333,-168,1558,-2221,-168,1542,-2221,-168,1558,-2317,-288,1558,-2213,-288,1542,-2213,-288,1542,-2325,-288,1558,-2309,-415,1558,-2213,-415,1550,-2213,-415,1542,-2309,-415],"vertslength":120,"polys":[0,3,4,7,8,11,12,15`
+`,16,19,20,22,23,28,29,32,33,35,36,38,39,42,43,45,46,50,51,53,54,58,59,61,62,64,65,67,68,71,72,75,76,79,80,83,84,87,88,91,92,96,97,100,101,103,104,107,108,111,112,115,116,119],"polyslength":31,"regions":[3,4,4,4,1,1,1,1,1,1,1,11,5,6,6,6,7,7,7,2,2,2,9,13,8,10,14,12,15,16,17],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,3],[0]],[[1,12],[0],[1,3],[0]],[[0],[0],[1,1],[1,2]],[[0],[1,13],[0],[1,6]],[[0],[0],[1,7]],[[0],[1,4],[1,9],[0],[1,8],[1,10]],[[1,5],[0],[1,10],[0]],[[0],[0],[1,6]],[[0],[0],[1,6]],[`
+`[1,7],[0],[1,6],[0]],[[0],[0],[0]],[[1,2],[0],[0],[0],[0]],[[0],[1,4],[1,14]],[[0],[1,13],[1,15],[0],[0]],[[0],[0],[1,14]],[[1,24],[0],[1,18]],[[0],[1,19],[1,18]],[[1,17],[0],[0],[1,16]],[[1,17],[0],[1,21],[0]],[[0],[1,24],[0],[1,21]],[[1,19],[0],[1,20],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,16],[0],[0],[1,20]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1046,-2557,-504,1046,-2717,-504,1558,-2717,-504,1558`
+`,-2557,-504,1046,-2717,-416,1062,-2717,-416,1070,-2637,-416,1046,-2509,-416,1238,-2693,-416,1246,-2565,-414,1230,-2557,-415,1070,-2637,-416,1230,-2557,-415,1230,-2509,-414,1046,-2509,-416,1070,-2637,-416,1286,-2357,-416,1302,-2349,-416,1254,-2285,-415,1230,-2277,-408,1241.199951171875,-2293,-415,1558,-2213,-640,1470,-2213,-640,1518,-2229,-640,1294,-2373,-416,1286,-2357,-416,1241.199951171875,-2293,-415,1230,-2277,-408,1046,-2213,-416,1046,-2477,-416,1238,-2445,-415,1226,-2297,-416,1558,-2213,-64`
+`0,1518,-2229,-640,1511.142822265625,-2352.428466796875,-562,1510,-2373,-560,1558,-2445,-560,1558,-2352.199951171875,-562,1558,-2236.199951171875,-640,1046,-2477,-416,1230,-2477,-416,1238,-2445,-415,1230,-2277,-408,1230,-2213,-408,1046,-2213,-416,1558,-2445,-560,1510,-2373,-560,1445.199951171875,-2373,-511,1294,-2373,-416,1238,-2445,-415,1306.5714111328125,-2445,-420,1512.2857666015625,-2445,-560,1102,-2717,-424,1206,-2709,-419,1102,-2685,-419,1246,-2565,-414,1238,-2693,-416,1294,-2717,-416,1558,`
+`-2717,-415,1558,-2565,-415,1270,-2277,-416,1254,-2285,-416,1302,-2349,-416,1262,-2213,-416,1270,-2277,-416,1302,-2349,-416,1478,-2325,-416,1494,-2213,-416,1302,-2349,-416,1494,-2349,-416,1478,-2325,-416,1406,-2533,-164,1390,-2477,-164,1342,-2461,-164,1342,-2461,-164,1342,-2405,-164,1262,-2397,-164,1342,-2461,-164,1262,-2397,-164,1262,-2533,-164,1406,-2533,-164,1262,-2397,-164,1342,-2405,-164,1350,-2389,-164,1262,-2213,-164,1414,-2389,-164,1430,-2437,-164,1494,-2437,-164,1494,-2213,-164,1262,-221`
+`3,-164,1350,-2389,-164,1414,-2389,-164,1494,-2213,-164,1318,-2373,-560,1318,-2445,-560,1422,-2445,-560,1422,-2373,-560,1366,-2413,-118,1366,-2445,-118,1398,-2445,-118,1398,-2413,-118,1430,-2437,-164,1390,-2477,-164,1406,-2533,-164,1494,-2533,-164,1494,-2437,-164,1518,-2469,-176,1558,-2469,-176,1558,-2365,-176,1518,-2365,-176,1558,-2341,-640,1558,-2333,-640,1518,-2333,-640,1526,-2541,-88,1558,-2541,-88,1558,-2213,-88,1518,-2213,-88,1542,-2333,-168,1558,-2333,-168,1558,-2221,-168,1542,-2221,-168,1`
+`558,-2317,-288,1558,-2213,-288,1542,-2213,-288,1542,-2325,-288,1558,-2309,-415,1558,-2213,-415,1550,-2213,-415,1542,-2309,-415],"vertslength":129,"tris":[3,0,1,1,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,18,19,20,18,20,16,16,17,18,21,22,23,29,30,31,30,24,31,27,26,31,24,25,31,26,25,31,27,28,31,29,28,31,38,32,33,34,35,36,34,36,37,37,38,33,33,34,37,39,40,41,42,43,44,51,45,46,48,49,50,51,46,47,47,48,50,47,50,51,52,53,54,55,56,57,57,58,59,55,57,59,60,61,62,63,64,65,65,66,67,63,65,67,68,69,70,7`
+`1,72,73,74,75,76,77,78,79,77,79,80,81,82,83,81,83,84,85,86,87,85,87,88,90,91,92,89,90,92,96,93,94,94,95,96,100,97,98,98,99,100,101,102,103,104,105,101,101,103,104,109,106,107,107,108,109,110,111,112,113,114,115,113,115,116,120,117,118,118,119,120,121,122,123,121,123,124,125,126,127,125,127,128],"trislength":68,"triTopoly":[0,0,1,1,2,2,3,3,4,4,4,5,6,6,6,6,6,6,6,7,7,7,7,7,8,9,10,10,10,10,10,11,12,12,12,13,14,14,14,15,16,17,18,18,19,19,20,20,21,21,22,22,23,23,24,24,24,25,25,26,27,27,28,28,29,29,30,`
+`30],"baseVert":[0,4,8,12,16,21,24,32,39,42,45,52,55,60,63,68,71,74,77,81,85,89,93,97,101,106,110,113,117,121,125],"vertsCount":[4,4,4,4,5,3,8,7,3,3,7,3,5,3,5,3,3,3,4,4,4,4,4,4,5,4,3,4,4,4,4],"baseTri":[0,2,4,6,8,11,12,19,24,25,26,31,32,35,36,39,40,41,42,44,46,48,50,52,54,57,59,60,62,64,66],"triCount":[2,2,2,2,3,1,7,5,1,1,5,1,3,1,3,1,1,1,2,2,2,2,2,2,3,2,1,2,2,2,2]},"links":{"poly":[7,26,16,23],"cost":[2922.2353515625,3951.60009765625],"type":[2,2],"pos":[1558,-2341,-595.862060546875,1558,-2341,-6`
+`40,1358.800048828125,-2466.60009765625,-164,1366,-2445,-118],"length":2}}],["9_0",{"tileId":"9_0","tx":9,"ty":0,"mesh":{"verts":[1558,-2557,-504,1558,-2717,-504,2070,-2717,-504,2070,-2557,-504,1558,-2565,-415,1558,-2717,-415,2070,-2717,-412,2070,-2565,-416,1558,-2565,-113,1558,-2581,-117,1582,-2581,-117,1582,-2565,-113,1798,-2213,-88,1654,-2213,-88,1670,-2229,-88,2070,-2213,-88,1966,-2213,-88,1958,-2277,-88,2070,-2541,-88,1806,-2277,-88,1798,-2213,-88,1670,-2229,-88,1670,-2373,-88,1566,-2381,-88`
+`,1574,-2213,-88,1558,-2213,-88,1558,-2541,-88,1806,-2277,-88,1670,-2373,-88,1654,-2389,-88,1558,-2541,-88,2070,-2541,-88,1958,-2277,-88,1654,-2389,-88,1566,-2381,-88,1558,-2541,-88,1558,-2365,-176,1558,-2469,-176,1598,-2469,-176,1598,-2365,-176,1590,-2229,-637,1614,-2213,-638,1558,-2213,-640,1558,-2445,-560,1590,-2445,-560,1590,-2229,-637,1558,-2213,-640,1558,-2341,-640,1590,-2341,-640,1558,-2333,-640,1558,-2213,-288,1558,-2325,-288,1630,-2325,-288,1638,-2213,-288,1622,-2213,-415,1558,-2213,-415`
+`,1558,-2309,-415,1622,-2317,-415,1582,-2229,-20,1582,-2309,-20,1598,-2293,-20,1646,-2229,-20,1582,-2229,-20,1598,-2293,-20,1646,-2309,-20,1622,-2317,-88,1598,-2309,-88,1598,-2357,-88,1638,-2357,-88,1630,-2301,-88,1622,-2317,-88,1638,-2357,-88,1598,-2237,-88,1598,-2301,-88,1630,-2301,-88,1638,-2237,-88,1638,-2357,-88,1638,-2237,-88,1630,-2301,-88,1598,-2357,0,1630,-2357,0,1638,-2325,0,1598,-2317,0,1614,-2565,-113,1614,-2581,-117,1638,-2581,-117,1638,-2565,-113,1782,-2229,-416,1878,-2237,-416,1886`
+`,-2213,-416,1662,-2213,-416,1654,-2333,-416,1766,-2333,-416,1782,-2229,-416,1662,-2213,-416,1662,-2277,-168,1662,-2333,-168,1702,-2333,-168,1702,-2277,-168,1774,-2277,-168,1734,-2277,-168,1734,-2325,-168,1774,-2333,-168,1798,-2261,-224,1790,-2333,-224,1862,-2333,-224,1870,-2261,-225,1862,-2325,-414,1862,-2269,-414,1806,-2261,-414,1798,-2325,-414,1862,-2325,-318,1862,-2269,-318,1806,-2261,-318,1798,-2325,-318,1806,-2277,-168,1806,-2333,-168,1838,-2333,-168,1838,-2277,-168,1950,-2261,72,1950,-2213`
+`,72,1814,-2213,72,1814,-2261,72,1942,-2253,-88,1942,-2213,-88,1822,-2213,-88,1822,-2253,-88,1870,-2277,-162,1870,-2333,-162,1910,-2333,-162,1910,-2277,-168,1886,-2213,-416,1878,-2237,-416,1894,-2245,-416,2022,-2333,-416,2030,-2349,-416,2070,-2333,-416,1958,-2325,-416,2022,-2333,-416,2070,-2333,-416,2070,-2213,-416,1886,-2213,-416,1894,-2245,-416,1894,-2245,-416,1894,-2333,-416,1958,-2325,-416,1974,-2221,-168,1958,-2221,-168,1958,-2261,-168,1958,-2261,-168,1942,-2269,-168,1942,-2325,-162,1958,-22`
+`61,-168,1942,-2325,-162,1974,-2333,-162,1974,-2221,-168,2006,-2333,-162,2046,-2333,-162,2046,-2277,-168,2006,-2269,-168],"vertslength":160,"polys":[0,3,4,7,8,11,12,14,15,18,19,22,23,26,27,32,33,35,36,39,40,42,43,46,47,49,50,53,54,57,58,60,61,64,65,68,69,71,72,75,76,78,79,82,83,86,87,90,91,94,95,98,99,102,103,106,107,110,111,114,115,118,119,122,123,126,127,130,131,133,134,136,137,142,143,145,146,148,149,151,152,155,156,159],"polyslength":42,"regions":[2,3,25,1,1,1,1,1,1,13,22,22,26,6,7,9,9,14,14,`
+`14,14,16,29,4,4,17,18,8,10,11,23,12,19,20,5,5,5,5,24,24,24,21],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,5]],[[0],[0],[1,7],[0]],[[0],[1,3],[0],[1,7]],[[0],[0],[0],[1,8]],[[1,5],[0],[1,8],[0],[1,4],[0]],[[0],[1,6],[1,7]],[[0],[0],[0],[0]],[[0],[0],[1,11]],[[0],[0],[1,10],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,16]],[[0],[1,15],[0],[0]],[[0],[0],[0],[1,18]],[[0],[1,17],[1,20]],[[0],[0],[1,20],[0]],[[0],[1,19],[1,18]],[[0],[0],[0],[0]`
+`],[[0],[0],[0],[0]],[[0],[1,34],[0],[1,24]],[[0],[0],[1,23],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,23],[0],[1,36]],[[0],[0],[1,36]],[[0],[1,35],[0],[0],[1,34],[1,37]],[[0],[0],[1,36]],[[0],[0],[1,40]],[[0],[0],[1,40]],[[1,39],[0],[0],[1,38]],[[0],[0],[0],[0]]]},"detail":{"verts":[1558,-2557,-504,1558,-2717,-504,2070,-2717,-504,2070,-2557,-504,1558,-2565,-415,1558,-2`
+`717,-415,2070,-2717,-412,2070,-2565,-416,1558,-2565,-113,1558,-2581,-113,1582,-2581,-113,1582,-2565,-113,1798,-2213,-88,1654,-2213,-88,1670,-2229,-88,2070,-2213,-88,1966,-2213,-88,1958,-2277,-88,2070,-2541,-88,1806,-2277,-88,1798,-2213,-88,1670,-2229,-88,1670,-2373,-88,1566,-2381,-88,1574,-2213,-88,1558,-2213,-88,1558,-2541,-88,1806,-2277,-88,1670,-2373,-88,1654,-2389,-88,1558,-2541,-88,2070,-2541,-88,1958,-2277,-88,1654,-2389,-88,1566,-2381,-88,1558,-2541,-88,1558,-2365,-176,1558,-2469,-176,159`
+`8,-2469,-176,1598,-2365,-176,1590,-2229,-638,1614,-2213,-638,1558,-2213,-640,1558,-2445,-560,1590,-2445,-560,1590,-2358.60009765625,-560,1590,-2250.60009765625,-631,1590,-2229,-638,1558,-2213,-640,1558,-2236.199951171875,-640,1558,-2352.199951171875,-562,1558,-2341,-640,1590,-2341,-640,1558,-2333,-640,1558,-2213,-288,1558,-2325,-288,1630,-2325,-288,1638,-2213,-288,1622,-2213,-415,1558,-2213,-415,1558,-2309,-415,1622,-2317,-415,1582,-2229,-20,1582,-2309,-20,1598,-2293,-20,1646,-2229,-20,1582,-222`
+`9,-20,1598,-2293,-20,1646,-2309,-20,1622,-2317,-88,1598,-2309,-88,1598,-2357,-88,1638,-2357,-88,1630,-2301,-88,1622,-2317,-88,1638,-2357,-88,1598,-2237,-88,1598,-2301,-88,1630,-2301,-88,1638,-2237,-88,1638,-2357,-88,1638,-2237,-88,1630,-2301,-88,1598,-2357,0,1630,-2357,0,1638,-2325,0,1598,-2317,0,1598,-2337,8,1614,-2565,-113,1614,-2581,-113,1638,-2581,-113,1638,-2565,-113,1782,-2229,-416,1878,-2237,-416,1886,-2213,-416,1662,-2213,-416,1654,-2333,-416,1766,-2333,-416,1782,-2229,-416,1662,-2213,-4`
+`16,1662,-2277,-168,1662,-2333,-168,1702,-2333,-168,1702,-2277,-168,1774,-2277,-168,1734,-2277,-168,1734,-2325,-168,1774,-2333,-168,1798,-2261,-225,1790,-2333,-224,1862,-2333,-224,1870,-2261,-225,1862,-2325,-414,1862,-2269,-414,1806,-2261,-414,1798,-2325,-414,1862,-2325,-318,1862,-2269,-318,1806,-2261,-318,1798,-2325,-318,1806,-2277,-168,1806,-2333,-168,1838,-2333,-168,1838,-2277,-168,1950,-2261,72,1950,-2213,72,1814,-2213,72,1814,-2261,72,1942,-2253,-88,1942,-2213,-88,1822,-2213,-88,1822,-2253,-`
+`88,1870,-2277,-163,1870,-2333,-163,1910,-2333,-162,1910,-2277,-168,1886,-2213,-416,1878,-2237,-416,1894,-2245,-416,2022,-2333,-416,2030,-2349,-416,2070,-2333,-416,1958,-2325,-416,2022,-2333,-416,2070,-2333,-416,2070,-2213,-416,1886,-2213,-416,1894,-2245,-416,1894,-2245,-416,1894,-2333,-416,1958,-2325,-416,1974,-2221,-168,1958,-2221,-168,1958,-2261,-168,1958,-2261,-168,1942,-2269,-168,1942,-2325,-168,1958,-2261,-168,1942,-2325,-168,1974,-2333,-162,1974,-2221,-168,2006,-2333,-162,2046,-2333,-162,2`
+`046,-2277,-168,2006,-2269,-168],"vertslength":165,"tris":[3,0,1,1,2,3,7,4,5,5,6,7,11,8,9,9,10,11,12,13,14,15,16,17,15,17,18,19,20,21,19,21,22,23,24,25,23,25,26,27,28,29,32,27,29,32,29,30,30,31,32,33,34,35,39,36,37,37,38,39,40,41,42,47,48,49,46,47,49,43,44,45,50,43,45,50,45,46,46,49,50,51,52,53,54,55,56,54,56,57,58,59,60,58,60,61,62,63,64,65,66,67,65,67,68,69,70,71,69,71,72,73,74,75,76,77,78,76,78,79,80,81,82,87,83,84,85,86,87,84,85,87,91,88,89,89,90,91,92,93,94,92,94,95,96,97,98,96,98,99,103,100`
+`,101,101,102,103,104,105,106,104,106,107,108,109,110,108,110,111,112,113,114,112,114,115,116,117,118,116,118,119,123,120,121,121,122,123,127,124,125,125,126,127,131,128,129,129,130,131,135,132,133,133,134,135,136,137,138,139,140,141,142,143,144,146,147,142,142,144,145,142,145,146,148,149,150,151,152,153,154,155,156,157,158,159,157,159,160,161,162,163,161,163,164],"trislength":81,"triTopoly":[0,0,1,1,2,2,3,4,4,5,5,6,6,7,7,7,7,8,9,9,10,11,11,11,11,11,11,12,13,13,14,14,15,16,16,17,17,18,19,19,20,21`
+`,21,21,22,22,23,23,24,24,25,25,26,26,27,27,28,28,29,29,30,30,31,31,32,32,33,33,34,35,36,36,36,36,37,38,39,40,40,41,41],"baseVert":[0,4,8,12,15,19,23,27,33,36,40,43,51,54,58,62,65,69,73,76,80,83,88,92,96,100,104,108,112,116,120,124,128,132,136,139,142,148,151,154,157,161],"vertsCount":[4,4,4,3,4,4,4,6,3,4,3,8,3,4,4,3,4,4,3,4,3,5,4,4,4,4,4,4,4,4,4,4,4,4,3,3,6,3,3,3,4,4],"baseTri":[0,2,4,6,7,9,11,13,17,18,20,21,27,28,30,32,33,35,37,38,40,41,44,46,48,50,52,54,56,58,60,62,64,66,68,69,70,74,75,76,77,7`
+`9],"triCount":[2,2,2,1,2,2,2,4,1,2,1,6,1,2,2,1,2,2,1,2,1,3,2,2,2,2,2,2,2,2,2,2,2,2,1,1,4,1,1,1,2,2]},"links":{"poly":[2,22,11,12,15,21,25,26,26,30,30,33,33,39,40,41],"cost":[1536,2763.95263671875,1032,1536,1536,1590,1537.10205078125,1536],"type":[1,2,2,1,1,1,1,1],"pos":[1582,-2581,-117,1614,-2581,-117,1590,-2341,-597.0740966796875,1590,-2341,-640,1586,-2305,-20,1598,-2317,0,1702,-2325,-168,1734,-2325,-168,1774,-2277,-168,1806,-2277,-168,1838,-2333,-168,1870,-2333,-162,1910,-2325,-162.85714721679`
+`688,1942,-2325,-162,1974,-2333,-162,2006,-2333,-162],"length":8}}],["10_0",{"tileId":"10_0","tx":10,"ty":0,"mesh":{"verts":[2582,-2717,-504,2582,-2685,-504,2310,-2685,-504,2070,-2717,-504,2310,-2685,-504,2302,-2557,-504,2070,-2557,-504,2070,-2717,-504,2582,-2717,-415,2582,-2693,-415,2302,-2693,-415,2070,-2717,-412,2302,-2693,-415,2294,-2565,-416,2070,-2565,-416,2070,-2717,-412,2070,-2213,-88,2070,-2541,-88,2134,-2541,-88,2134,-2213,-88,2110,-2213,-416,2070,-2213,-416,2070,-2333,-416,2118,-2333,-`
+`416,2078,-2221,-168,2078,-2333,-162,2118,-2333,-162,2118,-2221,-168,2158,-2533,-100,2214,-2533,-100,2222,-2413,-100,2158,-2213,-100,2222,-2413,-100,2294,-2413,-100,2294,-2213,-100,2158,-2213,-100],"vertslength":36,"polys":[0,3,4,7,8,11,12,15,16,19,20,23,24,27,28,31,32,35],"polyslength":9,"regions":[1,1,2,2,4,5,6,3,3],"neighbors":[[[0],[0],[1,1],[0]],[[0],[0],[0],[1,0]],[[0],[0],[1,3],[0]],[[0],[0],[0],[1,2]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,8],[0]],[[0],[0],[0],[`
+`1,7]]]},"detail":{"verts":[2582,-2717,-504,2582,-2685,-504,2310,-2685,-504,2070,-2717,-504,2310,-2685,-504,2302,-2557,-504,2070,-2557,-504,2070,-2717,-504,2582,-2717,-415,2582,-2693,-415,2302,-2693,-415,2070,-2717,-412,2302,-2693,-415,2294,-2565,-416,2070,-2565,-416,2070,-2717,-412,2070,-2213,-88,2070,-2541,-88,2134,-2541,-88,2134,-2213,-88,2110,-2213,-416,2070,-2213,-416,2070,-2333,-416,2118,-2333,-416,2078,-2221,-168,2078,-2333,-162,2118,-2333,-162,2118,-2221,-168,2090,-2321,-168,2158,-2533,-1`
+`00,2214,-2533,-100,2222,-2413,-100,2158,-2213,-100,2222,-2413,-100,2294,-2413,-100,2294,-2213,-100,2158,-2213,-100],"vertslength":37,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,19,16,17,17,18,19,20,21,22,20,22,23,25,26,28,26,27,28,27,24,28,25,24,28,29,30,31,29,31,32,33,34,35,33,35,36],"trislength":20,"triTopoly":[0,0,1,1,2,2,3,3,4,4,5,5,6,6,6,6,7,7,8,8],"baseVert":[0,4,8,12,16,20,24,29,33],"vertsCount":[4,4,4,4,4,4,5,4,4],"baseTri":[0,2,4,6,8,10,12,16,18],"triCount":[2,2,2,2`
+`,2,2,4,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["11_0",{"tileId":"11_0","tx":11,"ty":0,"mesh":{"verts":[2582,-2685,-504,2582,-2717,-504,3094,-2717,-504,3094,-2685,-504,2582,-2693,-415,2582,-2717,-415,3094,-2717,-415,3094,-2693,-415],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2582,-2685,-504,2582,-2717,-504,3094,-2717,-504,3094,-2685,-504,2582,-2693,-415,2582,-2717,-415,3094,-2`
+`717,-415,3094,-2693,-415],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_0",{"tileId":"12_0","tx":12,"ty":0,"mesh":{"verts":[3094,-2685,-504,3094,-2717,-504,3550,-2717,-504,3550,-2685,-504,3094,-2693,-415,3094,-2717,-415,3550,-2717,-415,3550,-2693,-415],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"nei`
+`ghbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[3094,-2685,-504,3094,-2717,-504,3550,-2717,-504,3550,-2685,-504,3094,-2693,-415,3094,-2717,-415,3550,-2717,-415,3550,-2693,-415],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["0_1",{"tileId":"0_1","tx":0,"ty":1,"mesh":{"verts":[-3034,-1701,-464,-3042,-1701,-464`
+`,-3042,-2029,-464],"vertslength":3,"polys":[0,2],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0]]]},"detail":{"verts":[-3034,-1701,-464,-3042,-1701,-464,-3042,-2029,-464],"vertslength":3,"tris":[0,1,2],"trislength":1,"triTopoly":[0],"baseVert":[0],"vertsCount":[3],"baseTri":[0],"triCount":[1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_1",{"tileId":"1_1","tx":1,"ty":1,"mesh":{"verts":[-2026,-1813,88,-2050,-1813,88,-2058,-1973,88,-2026,-2213,88,-2154,-1957,88,-2154,`
+`-1813,88,-2274,-1797,88,-2274,-2213,88,-2026,-2213,88,-2058,-1973,88,-2138,-1973,88,-2026,-2213,88,-2138,-1973,88,-2154,-1957,88,-2274,-2213,88,-2058,-1797,88,-2050,-1813,88,-2026,-1813,88,-2026,-1701,88,-2274,-1797,88,-2154,-1813,88,-2138,-1797,88,-2138,-1797,88,-2058,-1797,88,-2026,-1701,88,-2274,-1701,88,-2274,-1797,88,-2122,-1821,88,-2122,-1941,88,-2082,-1941,88,-2082,-1821,88],"vertslength":31,"polys":[0,3,4,7,8,10,11,14,15,18,19,21,22,26,27,30],"polyslength":8,"regions":[1,1,1,1,2,2,2,3],"`
+`neighbors":[[[1,4],[0],[1,2],[0]],[[0],[1,5],[0],[1,3]],[[1,0],[0],[1,3]],[[1,2],[0],[1,1],[0]],[[0],[1,0],[0],[1,6]],[[1,1],[0],[1,6]],[[0],[1,4],[0],[0],[1,5]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2026,-1813,88,-2050,-1813,88,-2058,-1973,88,-2026,-2213,88,-2154,-1957,88,-2154,-1813,88,-2274,-1797,88,-2274,-2213,88,-2026,-2213,88,-2058,-1973,88,-2138,-1973,88,-2026,-2213,88,-2138,-1973,88,-2154,-1957,88,-2274,-2213,88,-2058,-1797,88,-2050,-1813,88,-2026,-1813,88,-2026,-1701,88,-2274,-1797,88`
+`,-2154,-1813,88,-2138,-1797,88,-2138,-1797,88,-2058,-1797,88,-2026,-1701,88,-2274,-1701,88,-2274,-1797,88,-2122,-1821,88,-2122,-1941,88,-2082,-1941,88,-2082,-1821,88],"vertslength":31,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,11,12,13,11,13,14,15,16,17,15,17,18,19,20,21,22,23,24,25,26,22,22,24,25,30,27,28,28,29,30],"trislength":15,"triTopoly":[0,0,1,1,2,3,3,4,4,5,6,6,6,7,7],"baseVert":[0,4,8,11,15,19,22,27],"vertsCount":[4,4,3,4,4,3,5,4],"baseTri":[0,2,4,5,7,9,10,13],"triCount":[2,2,1,2,2,1,3,2]},"`
+`links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_1",{"tileId":"2_1","tx":2,"ty":1,"mesh":{"verts":[-1938,-2093,88,-1914,-2101,88,-1914,-2077,88,-1938,-2085,88,-1938,-2093,88,-1938,-2085,88,-1946,-2069,88,-2026,-2061,88,-1946,-2117,88,-1914,-2213,88,-1914,-2117,88,-1946,-2117,88,-2026,-2213,88,-1914,-2213,88,-1946,-2117,88,-2026,-2061,88,-2026,-2061,88,-1946,-2069,88,-1938,-2053,88,-1914,-1701,88,-2026,-1701,88,-1938,-2053,88,-1914,-2061,88,-1914,-1701,88,-1898,-1701,104,-1898,-22`
+`13,104,-1890,-2213,104,-1890,-1701,104,-1810,-2053,-408,-1802,-2069,-408,-1762,-2069,-408,-1730,-2061,-408,-1730,-1885,-408,-1874,-1877,-408,-1874,-2061,-408,-1810,-2053,-408,-1730,-1885,-408,-1754,-2085,-416,-1762,-2069,-408,-1802,-2069,-408,-1810,-2085,-416,-1810,-2085,-416,-1874,-2085,-416,-1874,-2213,-416,-1514,-2213,-408,-1514,-2085,-408,-1754,-2085,-416,-1730,-1701,-408,-1874,-1701,-408,-1874,-1877,-408,-1730,-1885,-408,-1714,-2069,-140,-1514,-2069,-140,-1514,-1701,-140,-1714,-1701,-140],"`
+`vertslength":55,"polys":[0,3,4,8,9,11,12,15,16,20,21,23,24,27,28,32,33,36,37,40,41,46,47,50,51,54],"polyslength":13,"regions":[6,6,6,6,5,5,8,2,2,4,4,3,1],"neighbors":[[[0],[0],[0],[1,1]],[[1,0],[0],[1,4],[1,3],[0]],[[0],[0],[1,3]],[[0],[1,2],[1,1],[0]],[[1,1],[0],[1,5],[0],[0]],[[0],[0],[1,4]],[[0],[0],[0],[0]],[[0],[1,9],[0],[0],[1,8]],[[0],[0],[1,7],[1,11]],[[0],[1,7],[0],[1,10]],[[0],[0],[0],[0],[0],[1,9]],[[0],[0],[1,8],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-1938,-2093,88,-1914,-2101,8`
+`8,-1914,-2077,88,-1938,-2085,88,-1938,-2093,88,-1938,-2085,88,-1946,-2069,88,-2026,-2061,88,-1946,-2117,88,-1914,-2213,88,-1914,-2117,88,-1946,-2117,88,-2026,-2213,88,-1914,-2213,88,-1946,-2117,88,-2026,-2061,88,-2026,-2061,88,-1946,-2069,88,-1938,-2053,88,-1914,-1701,88,-2026,-1701,88,-1938,-2053,88,-1914,-2061,88,-1914,-1701,88,-1898,-1701,104,-1898,-2213,104,-1890,-2213,104,-1890,-1701,104,-1810,-2053,-408,-1802,-2069,-408,-1762,-2069,-408,-1730,-2061,-408,-1730,-1885,-408,-1874,-1877,-408,-1`
+`874,-2061,-408,-1810,-2053,-408,-1730,-1885,-408,-1754,-2085,-416,-1762,-2069,-408,-1802,-2069,-408,-1810,-2085,-408,-1772.6666259765625,-2085,-408,-1810,-2085,-408,-1831.3333740234375,-2085,-416,-1874,-2085,-416,-1874,-2213,-416,-1514,-2213,-408,-1514,-2085,-408,-1710.3636474609375,-2085,-408,-1754,-2085,-416,-1772.6666259765625,-2085,-408,-1814,-2105,-416,-1742,-2129,-416,-1730,-1701,-408,-1874,-1701,-408,-1874,-1877,-408,-1730,-1885,-408,-1714,-2069,-140,-1514,-2069,-140,-1514,-1701,-140,-171`
+`4,-1701,-140],"vertslength":61,"tris":[3,0,1,1,2,3,4,5,6,8,4,6,6,7,8,9,10,11,12,13,14,12,14,15,16,17,18,20,16,18,18,19,20,21,22,23,27,24,25,25,26,27,28,29,30,28,30,31,28,31,32,33,34,35,33,35,36,41,37,38,39,40,41,38,39,41,46,47,48,50,42,51,42,43,51,45,44,51,43,44,51,45,46,52,46,48,52,48,49,52,45,51,52,49,50,52,51,50,52,53,54,55,53,55,56,60,57,58,58,59,60],"trislength":37,"triTopoly":[0,0,1,1,1,2,3,3,4,4,4,5,6,6,7,7,7,8,8,9,9,9,10,10,10,10,10,10,10,10,10,10,10,11,11,12,12],"baseVert":[0,4,9,12,16,`
+`21,24,28,33,37,42,53,57],"vertsCount":[4,5,3,4,5,3,4,5,4,5,11,4,4],"baseTri":[0,2,5,6,8,11,12,14,17,19,22,33,35],"triCount":[2,3,1,2,3,1,2,3,2,3,11,2,2]},"links":{"poly":[0,6,4,6],"cost":[768,768],"type":[2,2],"pos":[-1914,-2101,88,-1898,-2101,104,-1914,-1701,88,-1898,-1701,104],"length":2}}],["3_1",{"tileId":"3_1","tx":3,"ty":1,"mesh":{"verts":[-1514,-2085,-408,-1514,-2213,-408,-1002,-2213,-416,-1002,-2085,-408,-1178,-1701,-140,-1514,-1701,-140,-1514,-2069,-140,-1170,-2069,-140,-1354,-2085,-189`
+`,-1354,-2101,-193,-1330,-2101,-193,-1330,-2085,-189,-1002,-2053,-168,-1002,-1989,-168,-1042,-1981,-168,-1162,-2053,-168,-1042,-1781,-168,-1002,-1773,-168,-1002,-1701,-168,-1162,-1701,-168,-1162,-2053,-168,-1042,-1981,-168,-1042,-1781,-168,-1162,-1701,-168,-1002,-1973,-88,-1002,-1957,-88,-1018,-1949,-88,-1026,-1973,-88,-1018,-1813,-88,-1002,-1805,-88,-1002,-1789,-88,-1026,-1789,-88,-1018,-1901,-88,-1002,-1893,-88,-1002,-1861,-88,-1018,-1853,-88,-1018,-1853,-88,-1018,-1813,-88,-1026,-1789,-88,-102`
+`6,-1973,-88,-1018,-1949,-88,-1018,-1901,-88,-1018,-1965,-168,-1002,-1965,-168,-1002,-1797,-168,-1018,-1797,-168],"vertslength":46,"polys":[0,3,4,7,8,11,12,15,16,19,20,23,24,27,28,31,32,35,36,41,42,45],"polyslength":11,"regions":[2,1,4,3,3,3,5,5,5,5,6],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,5],[0]],[[0],[0],[0],[1,5]],[[1,3],[0],[1,4],[0]],[[0],[0],[1,9],[0]],[[0],[0],[0],[1,9]],[[0],[0],[0],[1,9]],[[0],[1,7],[0],[1,6],[0],[1,8]],[[0],[0],[0],[0]]]},"detail`
+`":{"verts":[-1514,-2085,-408,-1514,-2213,-408,-1071.8182373046875,-2213,-408,-1048.54541015625,-2213,-416,-1002,-2213,-416,-1002,-2085,-408,-1046,-2153,-416,-1178,-1701,-140,-1514,-1701,-140,-1514,-2069,-140,-1170,-2069,-140,-1354,-2085,-189,-1354,-2101,-189,-1330,-2101,-189,-1330,-2085,-189,-1002,-2053,-168,-1002,-1989,-168,-1042,-1981,-168,-1162,-2053,-168,-1042,-1781,-168,-1002,-1773,-168,-1002,-1701,-168,-1162,-1701,-168,-1162,-2053,-168,-1042,-1981,-168,-1042,-1781,-168,-1162,-1701,-168,-10`
+`02,-1973,-88,-1002,-1957,-88,-1018,-1949,-88,-1026,-1973,-88,-1018,-1813,-88,-1002,-1805,-88,-1002,-1789,-88,-1026,-1789,-88,-1018,-1901,-88,-1002,-1893,-88,-1002,-1861,-88,-1018,-1853,-88,-1018,-1853,-88,-1018,-1813,-88,-1026,-1789,-88,-1026,-1973,-88,-1018,-1949,-88,-1018,-1901,-88,-1018,-1965,-168,-1002,-1965,-168,-1002,-1797,-168,-1018,-1797,-168],"vertslength":49,"tris":[0,1,2,0,2,6,5,0,6,2,3,6,5,4,6,3,4,6,7,8,9,7,9,10,14,11,12,12,13,14,15,16,17,15,17,18,19,20,21,19,21,22,23,24,25,23,25,26,`
+`27,28,29,27,29,30,31,32,33,31,33,34,35,36,37,35,37,38,39,40,41,42,43,44,44,39,41,41,42,44,48,45,46,46,47,48],"trislength":28,"triTopoly":[0,0,0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,9,9,10,10],"baseVert":[0,7,11,15,19,23,27,31,35,39,45],"vertsCount":[7,4,4,4,4,4,4,4,4,6,4],"baseTri":[0,6,8,10,12,14,16,18,20,22,26],"triCount":[6,2,2,2,2,2,2,2,2,4,2]},"links":{"poly":[1,4,1,5,1,2],"cost":[1559.818603515625,1280.47998046875,3985.5],"type":[2,2,2],"pos":[-1177.992431640625,-1701.34765625,-140,-1`
+`162,-1701,-168,-1170.3438720703125,-2053.181396484375,-140,-1162,-2053,-168,-1354,-2069,-140,-1354,-2085,-189],"length":3}}],["4_1",{"tileId":"4_1","tx":4,"ty":1,"mesh":{"verts":[-802,-2085,-408,-1002,-2085,-408,-1002,-2213,-416,-794,-2213,-416,-922,-1981,-168,-1002,-1989,-168,-1002,-2053,-168,-818,-2053,-168,-1002,-1701,-168,-1002,-1773,-168,-922,-1781,-168,-818,-1701,-168,-922,-1781,-168,-922,-1981,-168,-818,-2053,-168,-818,-1701,-168,-962,-1965,-88,-1002,-1965,-88,-1002,-1973,-88,-938,-1973,-`
+`88,-938,-1973,-88,-938,-1941,-88,-962,-1965,-88,-1002,-1797,-168,-1002,-1965,-168,-946,-1965,-168,-946,-1797,-168,-1002,-1909,-68,-1002,-1941,-68,-970,-1949,-68,-962,-1901,-68,-970,-1877,-88,-1002,-1869,-88,-1002,-1885,-88,-962,-1885,-88,-938,-1845,-88,-970,-1877,-88,-962,-1885,-88,-938,-1909,-88,-1002,-1821,-68,-1002,-1845,-68,-962,-1853,-68,-962,-1813,-68,-962,-1797,-88,-938,-1821,-88,-938,-1789,-88,-650,-2077,-408,-642,-2053,-408,-738,-2045,-408,-634,-2085,-408,-650,-2077,-408,-738,-2045,-408`
+`,-754,-2077,-408,-754,-2077,-408,-802,-2085,-408,-794,-2213,-416,-626,-2213,-416,-634,-2085,-408,-794,-1965,-408,-786,-1989,-408,-770,-1973,-408,-626,-1717,-408,-650,-1701,-408,-786,-1701,-408,-490,-1709,-416,-498,-1701,-416,-618,-1701,-416,-626,-1717,-408,-730,-1989,-408,-738,-2045,-408,-642,-2053,-408,-730,-1989,-408,-642,-2053,-408,-618,-2069,-416,-490,-2069,-416,-770,-1973,-408,-730,-1989,-408,-490,-2069,-416,-490,-1709,-416,-626,-1717,-408,-786,-2037,-358,-786,-2061,-358,-762,-2061,-358,-76`
+`2,-2037,-358,-786,-2021,-189,-786,-2045,-189,-770,-2045,-194,-770,-2021,-194,-770,-1989,-358,-770,-2013,-358,-754,-2021,-358,-746,-1989,-358,-618,-2069,-416,-634,-2085,-408,-626,-2213,-416,-490,-2213,-416,-490,-2069,-416],"vertslength":97,"polys":[0,3,4,7,8,11,12,15,16,19,20,22,23,26,27,30,31,34,35,38,39,42,43,45,46,48,49,52,53,57,58,63,64,67,68,70,71,74,75,79,80,83,84,87,88,91,92,96],"polyslength":24,"regions":[4,5,5,5,10,10,6,7,9,9,8,12,2,2,2,1,1,1,1,1,14,15,18,3],"neighbors":[[[0],[0],[0],[1,`
+`14]],[[0],[0],[0],[1,3]],[[0],[0],[1,3],[0]],[[0],[1,1],[0],[1,2]],[[0],[0],[0],[1,5]],[[0],[0],[1,4]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,9]],[[0],[1,8],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[1,17],[1,13]],[[0],[1,12],[0],[1,14]],[[0],[1,0],[0],[1,23],[1,13]],[[0],[0],[1,19],[0],[0],[0]],[[0],[0],[0],[1,19]],[[0],[1,12],[1,18]],[[1,17],[0],[1,23],[1,19]],[[0],[1,18],[0],[1,16],[1,15]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,14],[0],[0],[1,18]]]},"de`
+`tail":{"verts":[-802,-2085,-408,-1002,-2085,-408,-1002,-2213,-416,-794,-2213,-416,-922,-1981,-168,-1002,-1989,-168,-1002,-2053,-168,-818,-2053,-168,-1002,-1701,-168,-1002,-1773,-168,-922,-1781,-168,-818,-1701,-168,-922,-1781,-168,-922,-1981,-168,-818,-2053,-168,-818,-1701,-168,-962,-1965,-88,-1002,-1965,-88,-1002,-1973,-88,-938,-1973,-88,-938,-1973,-88,-938,-1941,-88,-962,-1965,-88,-1002,-1797,-168,-1002,-1965,-168,-946,-1965,-168,-946,-1797,-168,-1002,-1909,-68,-1002,-1941,-68,-970,-1949,-68,-9`
+`62,-1901,-68,-990,-1937,-60,-970,-1877,-88,-1002,-1869,-88,-1002,-1885,-88,-962,-1885,-88,-938,-1845,-88,-970,-1877,-88,-962,-1885,-88,-938,-1909,-88,-1002,-1821,-68,-1002,-1845,-68,-962,-1853,-68,-962,-1813,-68,-990,-1841,-60,-962,-1797,-88,-938,-1821,-88,-938,-1789,-88,-650,-2077,-408,-642,-2053,-408,-738,-2045,-408,-634,-2085,-408,-650,-2077,-408,-738,-2045,-408,-754,-2077,-408,-754,-2077,-408,-802,-2085,-408,-794,-2213,-416,-626,-2213,-416,-634,-2085,-408,-794,-1965,-408,-786,-1989,-408,-770`
+`,-1973,-408,-637.076904296875,-1736.6922607421875,-408,-626,-1717,-416,-650,-1701,-408,-786,-1701,-408,-490,-1709,-416,-498,-1701,-416,-618,-1701,-416,-626,-1717,-416,-730,-1989,-408,-738,-2045,-408,-642,-2053,-408,-730,-1989,-408,-642,-2053,-408,-618,-2069,-416,-490,-2069,-416,-770,-1973,-408,-730,-1989,-408,-490,-2069,-416,-490,-1709,-416,-626,-1717,-416,-637.076904296875,-1736.6922607421875,-408,-614,-1745,-416,-786,-2037,-358,-786,-2061,-358,-762,-2061,-358,-762,-2037,-358,-786,-2021,-194,-7`
+`86,-2045,-194,-770,-2045,-194,-770,-2021,-194,-770,-1989,-358,-770,-2013,-358,-754,-2021,-358,-746,-1989,-358,-618,-2069,-416,-634,-2085,-416,-632.6666870117188,-2106.333251953125,-408,-626,-2213,-416,-490,-2213,-416,-490,-2069,-416],"vertslength":103,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,16,17,18,16,18,19,20,21,22,26,23,24,24,25,26,27,28,31,28,29,31,29,30,31,27,30,31,32,33,34,32,34,35,36,37,38,36,38,39,41,42,44,42,43,44,43,40,44,41,40,44,45,46,47,48,49,50,51,52,53,51,`
+`53,54,55,56,57,58,59,55,55,57,58,60,61,62,63,64,65,63,65,66,66,60,62,62,63,66,67,68,69,67,69,70,71,72,73,74,75,76,74,76,77,83,78,79,83,79,84,81,82,84,83,82,84,79,80,84,81,80,84,88,85,86,86,87,88,92,89,90,90,91,92,93,94,95,93,95,96,97,98,99,97,99,100,101,102,97,97,100,101],"trislength":58,"triTopoly":[0,0,1,1,2,2,3,3,4,4,5,6,6,7,7,7,7,8,8,9,9,10,10,10,10,11,12,13,13,14,14,14,15,15,15,15,15,16,16,17,18,18,19,19,19,19,19,19,20,20,21,21,22,22,23,23,23,23],"baseVert":[0,4,8,12,16,20,23,27,32,36,40,45`
+`,48,51,55,60,67,71,74,78,85,89,93,97],"vertsCount":[4,4,4,4,4,3,4,5,4,4,5,3,3,4,5,7,4,3,4,7,4,4,4,6],"baseTri":[0,2,4,6,8,10,11,13,17,19,21,25,26,27,29,32,37,39,40,42,48,50,52,54],"triCount":[2,2,2,2,2,1,2,4,2,2,4,1,1,2,3,5,2,1,2,6,2,2,2,4]},"links":{"poly":[4,7,5,9,7,8,8,10,9,11,10,11,13,20,13,22,20,22],"cost":[984,1536,984,1430.769287109375,864,792,4057.199951171875,4998,480],"type":[2,1,2,2,1,2,2,2,1],"pos":[-970,-1965,-88,-970,-1949,-68,-938,-1941,-88,-938,-1909,-88,-962,-1901,-68,-962,-1885`
+`,-88,-1002,-1869,-88,-997.3846435546875,-1845.923095703125,-68,-938,-1845,-88,-938,-1821,-88,-962,-1813,-68,-954,-1805,-88,-749.2000122070312,-2067.39990234375,-408,-762,-2061,-358,-738,-2045,-408,-754,-2021,-358,-762,-2037,-358,-754,-2021,-358],"length":9}}],["5_1",{"tileId":"5_1","tx":5,"ty":1,"mesh":{"verts":[-490,-2213,-416,-130,-2213,-414,-178,-2189,-416,-490,-2213,-416,-178,-2189,-416,-490,-1725,-416,22,-1901,-416,22,-1877,-416,-34,-1861,-416,-50,-1909,-416,-50,-1909,-416,-34,-1861,-416,-9`
+`8,-1805,-416,-186,-2101,-416,-42,-1997,-416,-58,-1989,-416,-58,-1989,-416,-50,-1909,-416,-98,-1805,-416,-58,-1989,-416,-98,-1805,-416,-138,-1701,-416,-458,-1701,-416,-186,-2101,-416,-146,-2173,-416,-18,-2213,-416,22,-2213,-416,22,-1997,-416,-42,-1997,-416,-186,-2101,-416,-90,-1797,-47,-50,-1837,-47,22,-1861,-45,22,-1701,-19,-130,-1701,-46,22,-1701,-416,-114,-1701,-416,-74,-1797,-416,22,-1853,-416,-34,-1917,-340,-42,-1941,-340,-42,-1981,-340,22,-1981,-340,22,-1917,-340,-26,-1973,-414,22,-1973,-41`
+`4,22,-1925,-414,-26,-1925,-414,14,-2213,-768,22,-2213,-768,22,-2189,-768],"vertslength":51,"polys":[0,2,3,5,6,9,10,12,13,15,16,18,19,23,24,29,30,34,35,38,39,43,44,47,48,50],"polyslength":13,"regions":[2,2,1,1,1,1,1,3,4,5,6,7,8],"neighbors":[[[0],[0],[1,1]],[[1,0],[0],[0]],[[0],[0],[1,3],[0]],[[1,2],[0],[1,5]],[[1,7],[0],[1,6]],[[0],[1,3],[1,6]],[[1,5],[0],[0],[0],[1,4]],[[0],[0],[0],[0],[1,4],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"de`
+`tail":{"verts":[-490,-2213,-416,-130,-2213,-414,-178,-2189,-416,-490,-2213,-416,-178,-2189,-416,-490,-1725,-416,22,-1901,-416,22,-1877,-416,-34,-1861,-416,-50,-1909,-416,-50,-1909,-416,-34,-1861,-416,-98,-1805,-416,-186,-2101,-416,-42,-1997,-416,-58,-1989,-416,-58,-1989,-416,-50,-1909,-416,-98,-1805,-416,-58,-1989,-416,-98,-1805,-416,-138,-1701,-416,-458,-1701,-416,-186,-2101,-416,-146,-2173,-416,-18,-2213,-416,22,-2213,-416,22,-1997,-416,-42,-1997,-416,-186,-2101,-416,-90,-1797,-44,-50,-1837,-4`
+`4,22,-1861,-43,22,-1746.7142333984375,-19,22,-1701,-19,-21.428571701049805,-1701,-21,-130,-1701,-44,22,-1701,-416,-114,-1701,-416,-74,-1797,-416,22,-1853,-416,-34,-1917,-340,-42,-1941,-340,-42,-1981,-340,22,-1981,-340,22,-1917,-340,-26,-1973,-414,22,-1973,-414,22,-1925,-414,-26,-1925,-414,14,-2213,-768,22,-2213,-768,22,-2189,-768],"vertslength":53,"tris":[0,1,2,3,4,5,6,7,8,6,8,9,10,11,12,13,14,15,16,17,18,19,20,21,23,19,21,21,22,23,24,25,26,28,29,24,26,27,28,24,26,28,33,34,35,30,31,32,33,35,36,3`
+`0,32,33,30,33,36,37,38,39,37,39,40,41,42,43,44,45,41,41,43,44,49,46,47,47,48,49,50,51,52],"trislength":27,"triTopoly":[0,1,2,2,3,4,5,6,6,6,7,7,7,7,8,8,8,8,8,9,9,10,10,10,11,11,12],"baseVert":[0,3,6,10,13,16,19,24,30,37,41,46,50],"vertsCount":[3,3,4,3,3,3,5,6,7,4,5,4,3],"baseTri":[0,1,2,4,5,6,7,10,14,19,21,24,26],"triCount":[1,1,2,1,1,1,3,4,5,2,3,2,1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["6_1",{"tileId":"6_1","tx":6,"ty":1,"mesh":{"verts":[414,-2061,-768,422,-2109,-768,`
+`454,-2109,-768,462,-2061,-768,70,-1845,-760,62,-1813,-768,22,-1797,-768,46,-2061,-768,62,-1973,-760,462,-2061,-768,534,-2061,-759,534,-1981,-768,446,-1973,-760,414,-2061,-768,462,-2061,-768,446,-1973,-760,150,-1965,-760,142,-1981,-760,150,-1965,-760,454,-1957,-760,534,-1829,-760,86,-1829,-760,70,-1845,-760,70,-1957,-760,22,-2213,-768,38,-2213,-768,46,-2061,-768,22,-1797,-768,62,-1973,-760,70,-1957,-760,70,-1845,-760,454,-1957,-760,534,-1957,-760,534,-1829,-760,142,-1981,-760,62,-1973,-760,46,-20`
+`61,-768,414,-2061,-768,150,-1965,-760,446,-1973,-760,454,-1957,-760,22,-1877,-416,22,-1901,-416,46,-1909,-416,78,-1885,-416,86,-1933,-416,78,-1885,-416,46,-1909,-416,46,-1989,-416,246,-2213,-416,238,-2101,-416,182,-2093,-416,182,-1933,-416,86,-1933,-416,46,-1989,-416,182,-2093,-416,182,-2093,-416,46,-1989,-416,22,-1997,-416,22,-2213,-416,246,-2213,-416,214,-1837,-47,270,-1773,-46,294,-1701,-46,22,-1701,-19,22,-1869,-46,142,-1869,-46,46,-1829,-416,22,-1821,-416,22,-1853,-416,126,-1861,-416,54,-18`
+`13,-416,46,-1829,-416,126,-1861,-416,22,-1701,-416,22,-1813,-416,54,-1813,-416,54,-1813,-416,126,-1861,-416,174,-1845,-416,254,-1773,-416,278,-1701,-416,22,-1701,-416,22,-1797,-768,62,-1813,-768,70,-1797,-768,22,-1717,-768,158,-1805,-768,174,-1789,-768,182,-1717,-768,22,-1717,-768,70,-1797,-768,470,-2213,-752,462,-2133,-752,398,-2133,-752,398,-2133,-752,390,-2085,-752,62,-2085,-752,398,-2133,-752,62,-2085,-752,62,-2213,-752,470,-2213,-752,214,-1917,-240,214,-1845,-240,182,-1869,-240,182,-1869,-2`
+`40,94,-1885,-240,94,-1917,-240,214,-1917,-240,110,-1885,-416,102,-1909,-416,206,-1909,-416,206,-1853,-416,182,-1717,-768,174,-1789,-768,182,-1805,-768,534,-1797,-768,534,-1717,-768,198,-2045,-368,198,-2085,-368,238,-2085,-368,246,-2045,-368,198,-1941,-320,198,-2029,-320,286,-2029,-320,286,-1941,-320,214,-1957,-414,214,-2013,-414,278,-2013,-414,278,-1957,-414,230,-1829,-416,230,-1925,-416,294,-1925,-416,278,-1781,-416,278,-1781,-416,294,-1925,-416,302,-1941,-416,534,-1925,-416,470,-1789,-416,302,`
+`-1701,-416,534,-1925,-416,534,-1797,-408,470,-1789,-416,470,-1789,-416,470,-1701,-416,302,-1701,-416,302,-2037,-416,262,-2045,-416,238,-2101,-416,246,-2213,-416,494,-2213,-416,502,-2165,-416,502,-2165,-416,534,-2157,-416,534,-1925,-416,302,-1941,-416,302,-2037,-416,534,-2085,-752,478,-2085,-752,478,-2125,-752,478,-2125,-752,462,-2133,-752,470,-2213,-752,478,-2125,-752,470,-2213,-752,534,-2213,-752,534,-2085,-752,534,-1725,-239,526,-1701,-235,486,-1701,-224,486,-1781,-224,534,-1781,-224,534,-2213`
+`,-288,534,-2173,-288,518,-2173,-288,510,-2213,-288,534,-1701,-416,518,-1701,-416,510,-1741,-414,534,-1757,-416,526,-2213,-415,534,-2213,-415,534,-2189,-415],"vertslength":183,"polys":[0,3,4,8,9,12,13,17,18,23,24,27,28,30,31,33,34,37,38,40,41,44,45,48,49,51,52,55,56,60,61,66,67,70,71,73,74,76,77,82,83,86,87,91,92,94,95,97,98,101,102,104,105,108,109,112,113,117,118,121,122,125,126,129,130,133,134,139,140,142,143,145,146,151,152,156,157,159,160,162,163,166,167,171,172,175,176,179,180,182],"polyslen`
+`gth":45,"regions":[3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,5,6,6,6,6,8,8,7,7,7,13,13,15,9,16,10,12,2,2,2,2,1,1,11,11,11,14,18,19,20],"neighbors":[[[0],[0],[0],[1,3]],[[0],[1,20],[1,5],[1,8],[1,6]],[[0],[0],[0],[1,3]],[[1,0],[1,2],[1,9],[0],[1,8]],[[1,9],[1,7],[0],[0],[1,6],[0]],[[0],[0],[1,1],[0]],[[0],[1,4],[1,1]],[[0],[0],[1,4]],[[0],[1,1],[0],[1,3]],[[1,3],[0],[1,4]],[[0],[0],[1,11],[0]],[[0],[1,10],[0],[1,13]],[[1,36],[0],[1,14]],[[0],[1,11],[1,14],[0]],[[1,13],[0],[0],[0],[1,12]],[[0],[0],[0],[0],[0]`
+`,[0]],[[0],[0],[0],[1,17]],[[0],[1,16],[1,19]],[[0],[0],[1,19]],[[1,17],[0],[0],[0],[0],[1,18]],[[1,1],[0],[1,21],[0]],[[0],[1,28],[0],[1,20],[0]],[[1,39],[0],[1,24]],[[0],[0],[1,24]],[[1,23],[0],[0],[1,22]],[[0],[0],[1,26]],[[0],[0],[0],[1,25]],[[0],[0],[0],[0]],[[1,21],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,33],[0]],[[1,32],[0],[1,37],[1,34],[1,35],[0]],[[0],[0],[1,33]],[[0],[0],[1,33]],[[0],[0],[1,12],[0],[0],[1,37]],[[0],[0],[1,33],[0],[1,36]],[[0]`
+`,[0],[1,40]],[[0],[1,22],[1,40]],[[1,39],[0],[0],[1,38]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[414,-2061,-768,422,-2109,-768,454,-2109,-768,462,-2061,-768,70,-1845,-760,62,-1813,-768,22,-1797,-768,46,-2061,-768,58,-1995,-768,62,-1973,-760,58,-1977,-760,462,-2061,-768,534,-2061,-765,534,-1981,-768,512,-1979,-760,446,-1973,-760,450,-1995,-768,414,-2061,-768,462,-2061,-768,450,-1995,-768,446,-1973,-760,150,-1965,-760,142,-1981,-760,187.33332824`
+`70703,-1994.3333740234375,-768,418,-2001,-768,150,-1965,-760,454,-1957,-760,534,-1829,-760,86,-1829,-760,70,-1845,-760,70,-1957,-760,22,-2213,-768,38,-2213,-768,46,-2061,-768,22,-1797,-768,62,-1973,-760,70,-1957,-760,70,-1845,-760,454,-1957,-760,534,-1957,-760,534,-1829,-760,142,-1981,-760,62,-1973,-760,58,-1995,-768,46,-2061,-768,414,-2061,-768,187.3333282470703,-1994.3333740234375,-768,150,-1965,-760,446,-1973,-760,454,-1957,-760,22,-1877,-416,22,-1901,-416,46,-1909,-416,78,-1885,-416,86,-1933`
+`,-416,78,-1885,-416,46,-1909,-416,46,-1989,-416,246,-2213,-416,238,-2101,-416,182,-2093,-416,182,-1933,-416,86,-1933,-416,46,-1989,-416,182,-2093,-416,182,-2093,-416,46,-1989,-416,22,-1997,-416,22,-2213,-416,246,-2213,-416,214,-1837,-46,270,-1773,-45,294,-1701,-46,158,-1701,-19,22,-1701,-19,22,-1764,-20,22,-1869,-44,142,-1869,-46,106,-1761,-19,46,-1829,-416,22,-1821,-416,22,-1853,-416,126,-1861,-416,54,-1813,-416,46,-1829,-416,126,-1861,-416,22,-1701,-416,22,-1813,-416,54,-1813,-416,54,-1813,-41`
+`6,126,-1861,-416,174,-1845,-416,254,-1773,-416,278,-1701,-416,22,-1701,-416,22,-1797,-768,62,-1813,-768,70,-1797,-768,22,-1717,-768,158,-1805,-768,174,-1789,-768,182,-1717,-768,22,-1717,-768,70,-1797,-768,470,-2213,-752,462,-2133,-752,398,-2133,-752,398,-2133,-752,390,-2085,-752,62,-2085,-752,398,-2133,-752,62,-2085,-752,62,-2213,-752,470,-2213,-752,214,-1917,-240,214,-1845,-240,182,-1869,-240,182,-1869,-240,94,-1885,-240,94,-1917,-240,214,-1917,-240,110,-1885,-416,102,-1909,-416,206,-1909,-416,`
+`206,-1853,-416,182,-1717,-768,174,-1789,-768,182,-1805,-768,534,-1797,-768,534,-1717,-768,198,-2045,-368,198,-2085,-368,238,-2085,-368,246,-2045,-368,198,-1941,-320,198,-2029,-320,286,-2029,-320,286,-1941,-320,214,-1957,-414,214,-2013,-414,278,-2013,-414,278,-1957,-414,230,-1829,-416,230,-1925,-416,294,-1925,-416,278,-1781,-416,278,-1781,-416,294,-1925,-416,302,-1941,-416,534,-1925,-416,515.7142944335938,-1886.142822265625,-408,479.1428527832031,-1808.4285888671875,-408,470,-1789,-416,302,-1701,`
+`-416,458,-1905,-416,534,-1925,-416,534,-1882.3333740234375,-408,534,-1797,-408,491.3333435058594,-1791.6666259765625,-408,470,-1789,-416,479.1428527832031,-1808.4285888671875,-408,515.7142944335938,-1886.142822265625,-408,470,-1789,-416,470,-1701,-416,302,-1701,-416,302,-2037,-416,262,-2045,-416,238,-2101,-416,246,-2213,-416,494,-2213,-416,502,-2165,-416,502,-2165,-416,534,-2157,-416,534,-1925,-416,302,-1941,-416,302,-2037,-416,534,-2085,-752,478,-2085,-752,478,-2125,-752,478,-2125,-752,462,-213`
+`3,-752,470,-2213,-752,478,-2125,-752,470,-2213,-752,534,-2213,-752,534,-2085,-752,534,-1725,-239,526,-1701,-235,486,-1701,-224,486,-1781,-224,534,-1781,-225,522,-1745,-239,534,-2213,-288,534,-2173,-288,518,-2173,-288,510,-2213,-288,534,-1701,-416,518,-1701,-416,510,-1741,-416,534,-1757,-416,526,-2213,-415,534,-2213,-415,534,-2189,-415],"vertslength":203,"tris":[0,1,2,0,2,3,4,5,6,4,6,10,8,9,10,4,9,10,6,7,10,8,7,10,14,15,16,13,14,16,16,11,12,12,13,16,21,22,23,20,21,24,20,19,24,17,18,24,19,18,24,21`
+`,23,24,17,23,24,28,29,30,28,30,25,28,25,26,26,27,28,31,32,33,31,33,34,35,36,37,38,39,40,41,42,43,46,41,43,46,43,44,44,45,46,47,48,49,50,51,52,50,52,53,54,55,56,54,56,57,58,59,60,61,62,63,61,63,64,65,66,67,68,69,65,65,67,68,70,71,72,70,72,73,73,74,78,75,74,78,75,76,78,77,76,78,77,70,78,73,70,78,79,80,81,79,81,82,83,84,85,86,87,88,89,90,91,91,92,93,94,89,91,91,93,94,95,96,97,95,97,98,99,100,101,103,99,101,101,102,103,104,105,106,107,108,109,110,111,112,110,112,113,114,115,116,117,118,119,117,119,1`
+`20,121,122,123,121,123,124,125,126,127,128,129,125,125,127,128,130,131,132,130,132,133,137,134,135,135,136,137,141,138,139,139,140,141,142,143,144,142,144,145,152,153,146,148,149,154,149,150,154,150,151,154,152,151,154,152,146,154,148,147,154,146,147,154,158,159,160,161,155,156,157,158,160,161,156,157,157,160,161,162,163,164,165,166,167,165,167,168,169,170,165,165,168,169,171,172,173,173,174,175,171,173,175,176,177,178,179,180,181,182,183,184,182,184,185,186,187,188,189,190,191,190,186,191,186,1`
+`88,191,189,188,191,192,193,194,192,194,195,196,197,198,196,198,199,200,201,202],"trislength":118,"triTopoly":[0,0,1,1,1,1,1,1,2,2,2,2,3,3,3,3,3,3,3,4,4,4,4,5,5,6,7,8,8,8,8,9,10,10,11,11,12,13,13,14,14,14,15,15,15,15,15,15,15,15,16,16,17,18,19,19,19,19,20,20,21,21,21,22,23,24,24,25,26,26,27,27,28,28,28,29,29,30,30,31,31,32,32,33,33,33,33,33,33,33,33,34,34,34,34,34,35,36,36,36,36,37,37,37,38,39,40,40,41,41,41,41,41,42,42,43,43,44],"baseVert":[0,4,11,17,25,31,35,38,41,47,50,54,58,61,65,70,79,83,86,`
+`89,95,99,104,107,110,114,117,121,125,130,134,138,142,146,155,162,165,171,176,179,182,186,192,196,200],"vertsCount":[4,7,6,8,6,4,3,3,6,3,4,4,3,4,5,9,4,3,3,6,4,5,3,3,4,3,4,4,5,4,4,4,4,9,7,3,6,5,3,3,4,6,4,4,3],"baseTri":[0,2,8,12,19,23,25,26,27,31,32,34,36,37,39,42,50,52,53,54,58,60,63,64,65,67,68,70,72,75,77,79,81,83,91,96,97,101,104,105,106,108,113,115,117],"triCount":[2,6,4,7,4,2,1,1,4,1,2,2,1,2,3,8,2,1,1,4,2,3,1,1,2,1,2,2,3,2,2,2,2,8,5,1,4,3,1,1,2,5,2,2,1]},"links":{"poly":[2,40,12,29,29,30],"c`
+`ost":[937.5,3611.52001953125,3840],"type":[1,2,2],"pos":[534,-2061,-759,534,-2085,-752,196.55999755859375,-2095.080078125,-416,198,-2085,-368,198,-2045,-368,198,-2029,-320],"length":3}}],["7_1",{"tileId":"7_1","tx":7,"ty":1,"mesh":{"verts":[582,-2069,-752,542,-2069,-752,534,-2077,-752,590,-2085,-752,654,-2133,-752,646,-2085,-752,590,-2085,-752,726,-2213,-752,718,-2133,-752,654,-2133,-752,654,-2133,-752,590,-2085,-752,534,-2077,-752,534,-2213,-752,726,-2213,-752,534,-2189,-415,534,-2213,-415,758,`
+`-2213,-415,758,-2189,-415,534,-2213,-288,774,-2213,-288,774,-2181,-288,534,-2173,-288,558,-2085,-416,582,-2077,-416,590,-2037,-416,534,-1797,-408,534,-2157,-416,558,-2157,-416,558,-2085,-416,534,-1797,-408,662,-2045,-416,662,-2157,-416,782,-2165,-416,1046,-1701,-416,886,-1701,-416,878,-1797,-416,782,-2165,-416,790,-2213,-416,1046,-2213,-416,534,-1797,-408,590,-2037,-416,630,-2029,-416,534,-1797,-408,630,-2029,-416,662,-2045,-416,878,-1797,-416,662,-2045,-416,782,-2165,-416,1046,-2213,-416,1046,-`
+`1701,-416,878,-1797,-416,678,-2109,-768,710,-2109,-768,710,-2069,-768,670,-2061,-768,670,-2061,-768,710,-2069,-768,742,-2061,-768,734,-1965,-768,670,-1965,-768,670,-1965,-768,662,-1949,-767,598,-1941,-768,590,-1981,-760,582,-2069,-752,670,-2061,-768,534,-1981,-768,534,-2069,-754,582,-2069,-752,590,-1981,-760,598,-1941,-768,662,-1949,-767,670,-1933,-760,598,-1901,-760,670,-1845,-760,662,-1829,-767,598,-1821,-768,670,-1845,-760,598,-1821,-768,590,-1845,-760,582,-1893,-760,598,-1901,-760,670,-1933,`
+`-760,534,-1957,-760,574,-1957,-760,582,-1893,-760,534,-1829,-760,590,-1845,-760,534,-1829,-760,582,-1893,-760,854,-1845,-640,670,-1845,-760,670,-1933,-760,878,-1933,-640,534,-1869,-448,582,-1869,-448,590,-1837,-448,534,-1797,-448,590,-1837,-448,758,-1837,-448,758,-1797,-448,534,-1797,-448,598,-1821,-768,662,-1829,-767,670,-1813,-768,590,-1797,-768,534,-1717,-768,534,-1797,-768,590,-1797,-768,670,-1813,-768,734,-1813,-768,742,-1717,-768,534,-1717,-768,590,-1797,-768,534,-1733,-240,534,-1781,-224,`
+`558,-1797,-226,598,-1733,-240,870,-1701,-224,606,-1701,-240,598,-1733,-240,870,-1781,-224,870,-1701,-224,598,-1733,-240,558,-1797,-226,534,-1701,-416,534,-1757,-416,822,-1757,-416,838,-1701,-416,590,-1717,-174,590,-1701,-174,542,-1701,-174,542,-1717,-174,574,-2157,-339,646,-2157,-339,646,-2101,-339,574,-2093,-339,582,-2109,-414,582,-2157,-414,630,-2157,-414,630,-2109,-414,646,-2053,-368,606,-2045,-368,598,-2077,-368,646,-2085,-368,734,-2125,-752,718,-2133,-752,726,-2213,-752,966,-2213,-752,966,-`
+`2085,-752,966,-2085,-752,734,-2085,-752,734,-2125,-752,838,-1957,-768,846,-1933,-768,766,-1925,-768,854,-1965,-768,838,-1957,-768,766,-1925,-768,734,-1965,-768,742,-2061,-768,862,-2061,-768,766,-1845,-768,838,-1829,-768,854,-1813,-768,862,-1717,-768,742,-1717,-768,734,-1813,-768,854,-1845,-768,838,-1829,-768,766,-1845,-768,766,-1925,-768,846,-1933,-768,862,-1933,-768,814,-1805,-448,790,-1797,-448,790,-1869,-448,838,-1861,-448,1022,-1949,-768,990,-1949,-768,966,-1965,-768,998,-2069,-768,966,-1965`
+`,-768,958,-1941,-768,870,-1949,-768,854,-1965,-768,862,-2061,-768,998,-2069,-768,998,-2069,-768,998,-2213,-768,1022,-2213,-768,1022,-1949,-768,982,-1933,-768,990,-1949,-768,1022,-1949,-768,1014,-1821,-768,862,-1933,-768,870,-1949,-768,958,-1941,-768,870,-1829,-768,854,-1845,-768,862,-1933,-768,958,-1941,-768,982,-1933,-768,1014,-1821,-768,878,-1789,-640,854,-1845,-640,878,-1933,-640,1022,-1933,-640,1022,-1717,-640,1022,-1717,-640,870,-1717,-640,878,-1789,-640,862,-1717,-768,854,-1813,-768,870,-1`
+`829,-768,1014,-1821,-768,1022,-1717,-768],"vertslength":219,"polys":[0,3,4,6,7,9,10,14,15,18,19,22,23,26,27,30,31,33,34,36,37,39,40,42,43,46,47,51,52,55,56,60,61,66,67,70,71,74,75,77,78,83,84,87,88,90,91,94,95,98,99,102,103,106,107,109,110,114,115,118,119,121,122,125,126,129,130,133,134,137,138,141,142,145,146,150,151,153,154,156,157,162,163,168,169,174,175,178,179,182,183,188,189,192,193,196,197,199,200,205,206,210,211,213,214,218],"polyslength":53,"regions":[9,9,9,9,24,20,1,1,1,1,1,1,1,1,10,10`
+`,10,10,12,12,12,12,12,12,18,18,11,11,11,15,15,15,19,25,17,21,22,5,5,6,6,7,13,23,8,8,8,2,2,2,3,3,4],"neighbors":[[[0],[0],[1,3],[0]],[[0],[0],[1,3]],[[1,37],[0],[1,3]],[[1,1],[1,0],[0],[0],[1,2]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,11],[1,7]],[[0],[0],[1,6],[0]],[[0],[0],[1,13]],[[0],[0],[1,13]],[[0],[0],[1,13]],[[1,6],[0],[1,12]],[[1,11],[0],[1,13],[0]],[[1,8],[1,10],[0],[1,9],[1,12]],[[0],[0],[1,15],[0]],[[1,14],[0],[1,40],[0],[1,16]],[[0],[1,18],[0],[1,17],[0],[1,15]],[[0],[0],[1,1`
+`6],[0]],[[1,16],[0],[1,20],[0]],[[0],[1,26],[1,20]],[[1,19],[0],[1,22],[0],[1,18],[1,23]],[[0],[0],[1,22],[0]],[[0],[1,21],[1,20]],[[0],[1,20],[0],[1,50]],[[0],[0],[1,25],[0]],[[0],[0],[0],[1,24]],[[1,19],[0],[1,28],[0]],[[0],[0],[1,28]],[[0],[1,41],[0],[1,27],[1,26]],[[0],[0],[1,31],[0]],[[0],[0],[1,31]],[[0],[1,30],[1,29],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,2],[0],[0],[1,38]],[[0],[0],[1,37]],[[0],[1,42],[1,40]],[[0],[1,39],[0]`
+`,[1,15],[0],[1,45]],[[1,42],[0],[1,52],[0],[1,28],[0]],[[0],[1,41],[0],[1,39],[0],[1,49]],[[0],[0],[0],[0]],[[1,47],[0],[1,45],[1,46]],[[0],[1,48],[0],[1,40],[0],[1,44]],[[0],[0],[0],[1,44]],[[0],[1,44],[0],[1,49]],[[0],[1,45],[1,49]],[[0],[1,42],[1,48],[0],[1,47],[1,52]],[[0],[1,23],[0],[0],[1,51]],[[0],[0],[1,50]],[[1,41],[0],[1,49],[0],[0]]]},"detail":{"verts":[582,-2069,-752,542,-2069,-752,534,-2077,-752,590,-2085,-752,654,-2133,-752,646,-2085,-752,590,-2085,-752,726,-2213,-752,718,-2133,-75`
+`2,654,-2133,-752,654,-2133,-752,590,-2085,-752,534,-2077,-752,534,-2213,-752,726,-2213,-752,534,-2189,-415,534,-2213,-415,758,-2213,-415,758,-2189,-415,534,-2213,-288,774,-2213,-288,774,-2181,-288,534,-2173,-288,558,-2085,-416,582,-2077,-416,590,-2037,-416,534,-1797,-408,534,-2157,-416,558,-2157,-416,558,-2085,-416,534,-1797,-408,534,-1887,-408,534,-1909.5,-416,662,-2045,-416,662,-2157,-416,782,-2165,-416,1046,-1701,-416,886,-1701,-416,878,-1797,-416,782,-2165,-416,790,-2213,-416,1046,-2213,-416`
+`,534,-1797,-408,590,-2037,-416,630,-2029,-416,534,-1797,-408,630,-2029,-416,662,-2045,-416,785.4285888671875,-1903.2857666015625,-416,800.8571166992188,-1885.5714111328125,-408,878,-1797,-416,855.066650390625,-1797,-408,786,-1889,-408,858,-1817,-408,662,-2045,-416,782,-2165,-416,1046,-2213,-416,1046,-1701,-416,878,-1797,-416,847.1428833007812,-1832.4285888671875,-408,800.8571166992188,-1885.5714111328125,-408,785.4285888671875,-1903.2857666015625,-416,818,-1913,-416,678,-2109,-768,710,-2109,-768`
+`,710,-2069,-768,670,-2061,-768,670,-2061,-768,710,-2069,-768,742,-2061,-768,734,-1965,-768,670,-1965,-768,670,-1965,-768,662,-1949,-767,598,-1941,-768,590,-1981,-768,584,-2047,-768,582,-2069,-759,604,-2067,-768,670,-2061,-768,534,-1981,-768,534,-2069,-764,582,-2069,-759,584,-2047,-768,590,-1981,-768,598,-1941,-768,640.6666870117188,-1946.3333740234375,-768,662,-1949,-760,670,-1933,-752,652,-1925,-768,598,-1901,-768,670,-1845,-752,662,-1829,-767,598,-1821,-768,652,-1839,-767,670,-1845,-752,652,-1`
+`839,-767,598,-1821,-768,590,-1845,-768,587.3333129882812,-1861,-760,582,-1893,-760,598,-1901,-768,652,-1925,-768,670,-1933,-752,534,-1957,-760,574,-1957,-760,576.6666870117188,-1935.6666259765625,-768,582,-1893,-760,534,-1829,-760,590,-1845,-768,571.3333129882812,-1839.6666259765625,-760,534,-1829,-760,582,-1893,-760,587.3333129882812,-1861,-760,854,-1845,-640,831,-1845,-648,693,-1845,-744,670,-1845,-752,670,-1933,-752,693.111083984375,-1933,-744,831.7777709960938,-1933,-648,878,-1933,-640,534,-`
+`1869,-448,582,-1869,-448,590,-1837,-448,534,-1797,-448,590,-1837,-448,758,-1837,-448,758,-1797,-448,534,-1797,-448,598,-1821,-768,662,-1829,-768,670,-1813,-768,590,-1797,-768,534,-1717,-768,534,-1797,-768,590,-1797,-768,670,-1813,-768,734,-1813,-768,742,-1717,-768,534,-1717,-768,590,-1797,-768,534,-1733,-240,534,-1781,-225,558,-1797,-224,598,-1733,-240,870,-1701,-224,848,-1701,-224,804,-1701,-240,606,-1701,-240,598,-1733,-240,802,-1709,-240,847.3333129882812,-1703.6666259765625,-224,870,-1781,-2`
+`24,870,-1701,-224,847.3333129882812,-1703.6666259765625,-224,802,-1709,-240,598,-1733,-240,558,-1797,-224,810,-1737,-240,534,-1701,-416,534,-1757,-416,822,-1757,-416,838,-1701,-416,590,-1717,-174,590,-1701,-174,542,-1701,-174,542,-1717,-174,574,-2157,-339,646,-2157,-339,646,-2101,-339,574,-2093,-339,582,-2109,-414,582,-2157,-414,630,-2157,-414,630,-2109,-414,646,-2053,-368,606,-2045,-368,598,-2077,-368,646,-2085,-368,734,-2125,-752,718,-2133,-752,726,-2213,-752,966,-2213,-752,966,-2085,-752,966,`
+`-2085,-752,734,-2085,-752,734,-2125,-752,838,-1957,-768,846,-1933,-768,766,-1925,-768,854,-1965,-768,838,-1957,-768,766,-1925,-768,734,-1965,-768,742,-2061,-768,862,-2061,-768,766,-1845,-768,838,-1829,-768,854,-1813,-768,862,-1717,-768,742,-1717,-768,734,-1813,-768,854,-1845,-768,838,-1829,-768,766,-1845,-768,766,-1925,-768,846,-1933,-768,862,-1933,-768,814,-1805,-448,790,-1797,-448,790,-1869,-448,838,-1861,-448,1022,-1949,-768,990,-1949,-768,966,-1965,-768,998,-2069,-768,966,-1965,-768,958,-194`
+`1,-768,870,-1949,-768,854,-1965,-768,862,-2061,-768,998,-2069,-768,998,-2069,-768,998,-2213,-768,1022,-2213,-768,1022,-1949,-768,982,-1933,-768,990,-1949,-768,1022,-1949,-768,1014,-1821,-768,862,-1933,-768,870,-1949,-768,958,-1941,-768,870,-1829,-768,854,-1845,-768,862,-1933,-768,958,-1941,-768,982,-1933,-768,1014,-1821,-768,878,-1789,-640,854,-1845,-640,878,-1933,-640,1022,-1933,-640,1022,-1717,-640,1022,-1717,-640,870,-1717,-640,878,-1789,-640,862,-1717,-768,854,-1813,-768,870,-1829,-768,1014,`
+`-1821,-768,1022,-1717,-768],"vertslength":253,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,10,11,12,10,12,13,10,13,14,18,15,16,16,17,18,19,20,21,19,21,22,23,24,25,23,25,26,27,28,29,32,27,29,31,32,29,29,30,31,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,46,48,52,48,49,52,51,49,52,51,45,52,46,45,52,49,50,53,50,51,53,51,49,53,55,56,62,55,54,62,60,61,62,54,61,62,56,57,62,57,58,62,60,59,62,58,59,62,63,64,65,63,65,66,67,68,69,70,71,67,67,69,70,76,77,78,72,73,74,75,76,78,72,74,75,75,78,79,72,75,79,81,82,83,83,84,80`
+`,80,81,83,87,88,89,86,87,89,85,86,89,85,89,90,94,91,92,92,93,94,97,98,99,99,100,101,97,99,101,102,103,95,102,95,96,96,97,101,96,101,102,104,105,106,104,106,107,104,107,108,113,109,110,112,113,110,110,111,112,116,117,118,116,118,119,121,114,115,120,121,115,115,116,119,115,119,120,122,123,124,122,124,125,126,127,128,126,128,129,130,131,132,130,132,133,134,135,136,137,138,139,140,141,137,137,139,140,142,143,144,142,144,145,152,146,147,152,147,148,151,152,148,151,148,149,149,150,151,153,154,155,158,`
+`153,159,153,155,159,156,155,159,156,157,159,158,157,159,160,161,162,160,162,163,167,164,165,165,166,167,168,169,170,168,170,171,175,172,173,173,174,175,176,177,178,176,178,179,180,181,182,180,182,183,180,183,184,185,186,187,188,189,190,191,192,193,191,193,194,191,194,195,191,195,196,197,198,199,202,197,199,199,200,201,199,201,202,203,204,205,207,208,203,205,206,207,203,205,207,209,210,211,209,211,212,213,214,215,213,215,216,217,218,219,219,220,221,217,219,221,217,221,222,223,224,225,223,225,226,`
+`227,228,229,227,229,230,231,232,233,234,235,236,236,237,238,234,236,238,234,238,239,240,241,242,240,242,243,240,243,244,245,246,247,248,249,250,250,251,252,248,250,252],"trislength":151,"triTopoly":[0,0,1,2,3,3,3,4,4,5,5,6,6,7,7,7,7,8,9,10,11,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,14,14,15,15,15,16,16,16,16,16,16,17,17,17,18,18,18,18,19,19,20,20,20,20,20,20,20,21,21,21,22,22,22,23,23,23,23,23,23,24,24,25,25,26,26,27,28,28,28,29,29,30,30,30,30,30,31,31,31,31,31,31,32,32,33,33,34,34,35`
+`,35,36,36,37,37,37,38,39,40,40,40,40,41,41,41,41,42,42,42,42,43,43,44,44,45,45,45,45,46,46,47,47,48,49,49,49,49,50,50,50,51,52,52,52],"baseVert":[0,4,7,10,15,19,23,27,33,36,39,42,45,54,63,67,72,80,85,91,95,104,109,114,122,126,130,134,137,142,146,153,160,164,168,172,176,180,185,188,191,197,203,209,213,217,223,227,231,234,240,245,248],"vertsCount":[4,3,3,5,4,4,4,6,3,3,3,3,9,9,4,5,8,5,6,4,9,5,5,8,4,4,4,3,5,4,7,7,4,4,4,4,4,5,3,3,6,6,6,4,4,6,4,4,3,6,5,3,5],"baseTri":[0,2,3,4,7,9,11,13,17,18,19,20,21,`
+`30,38,40,43,49,52,56,58,65,68,71,77,79,81,83,84,87,89,94,100,102,104,106,108,110,113,114,115,119,123,127,129,131,135,137,139,140,144,147,148],"triCount":[2,1,1,3,2,2,2,4,1,1,1,1,9,8,2,3,6,3,4,2,7,3,3,6,2,2,2,1,3,2,5,6,2,2,2,2,2,3,1,1,4,4,4,2,2,4,2,2,1,4,3,1,3]},"links":{"poly":[0,17,5,34,6,36,25,43,34,36],"cost":[4.166666507720947,4351.66650390625,3825.230712890625,1536,1640.8170166015625],"type":[1,2,2,1,2],"pos":[542,-2069,-752,542,-2069,-753.6666870117188,573.4228515625,-2174.314208984375,-28`
+`8,574,-2157,-339,582.6153564453125,-2073.923095703125,-416,598,-2077,-368,758,-1837,-448,790,-1837,-448,644.243896484375,-2100.804931640625,-339,646,-2085,-368],"length":5}}],["8_1",{"tileId":"8_1","tx":8,"ty":1,"mesh":{"verts":[1230,-2101,-408,1302,-2093,-408,1302,-2037,-408,1302,-2037,-408,1326,-2029,-415,1326,-1805,-415,1302,-1797,-416,1046,-2213,-416,1230,-2213,-408,1230,-2101,-408,1046,-2213,-416,1230,-2101,-408,1302,-2037,-408,1302,-1797,-416,1046,-1701,-416,1310,-1709,-416,1494,-1701,-416`
+`,1046,-1701,-416,1302,-1797,-416,1310,-1709,-416,1046,-1701,-416,1318,-1861,-640,1310,-1781,-640,1294,-1829,-640,1318,-1861,-640,1294,-1829,-640,1198,-1821,-635,1438,-1989,-640,1446,-1861,-640,1318,-1861,-640,1198,-1821,-635,1070,-1829,-636,1062,-1989,-640,1438,-1989,-640,1318,-1861,-640,1438,-1981,-500,1430,-1965,-500,1406,-1981,-500,1102,-1981,-500,1062,-1941,-500,1062,-1989,-500,1542,-1997,-500,1542,-1981,-500,1438,-1981,-500,1406,-1981,-500,1102,-1981,-500,1062,-1989,-500,1198,-1821,-635,119`
+`8,-1701,-639,1070,-1701,-639,1070,-1829,-636,1158,-1909,-500,1150,-1829,-500,1110,-1829,-500,1110,-1925,-500,1398,-1933,-500,1398,-1909,-500,1158,-1909,-500,1110,-1925,-500,1198,-1861,-500,1398,-1861,-500,1398,-1845,-500,1198,-1829,-500,1222,-1781,-500,1222,-1805,-500,1262,-1805,-500,1262,-1781,-500,1398,-1701,-480,1230,-1701,-480,1230,-1749,-480,1406,-1749,-480,1238,-1741,-768,1558,-1741,-768,1558,-1701,-768,1238,-1701,-768,1286,-1701,-640,1238,-1701,-640,1238,-1741,-640,1558,-1741,-640,1558,-1`
+`709,-640,1470,-2125,-416,1398,-2141,-416,1390,-2165,-416,1494,-2213,-416,1494,-2149,-416,1390,-2165,-416,1270,-2165,-416,1262,-2213,-416,1494,-2213,-416,1262,-2125,-164,1262,-2213,-164,1494,-2213,-164,1494,-2125,-164,1270,-2149,-368,1302,-2149,-368,1302,-2125,-368,1318,-2125,-352,1318,-2149,-352,1382,-2149,-352,1382,-2125,-352,1318,-1741,-212,1494,-1725,-212,1318,-1725,-212,1350,-1741,-416,1334,-1757,-416,1342,-1797,-416,1478,-1741,-416,1558,-1717,-416,1542,-1709,-416,1534,-1805,-416,1558,-2021,`
+`-416,1478,-1741,-416,1342,-1797,-416,1326,-1805,-415,1494,-1797,-416,1326,-2029,-415,1350,-2085,-416,1478,-2085,-416,1502,-2029,-415,1534,-1805,-416,1494,-1797,-416,1326,-1805,-415,1326,-2029,-415,1502,-2029,-415,1558,-2021,-416,1326,-1845,-624,1342,-1837,-624,1326,-1829,-624,1374,-1877,-228,1366,-1813,-228,1334,-1805,-228,1334,-2093,-228,1494,-1813,-228,1454,-1813,-228,1446,-1877,-228,1494,-2093,-228,1494,-2093,-228,1446,-1877,-228,1374,-1877,-228,1334,-2093,-228,1334,-1805,-228,1366,-1813,-228`
+`,1374,-1797,-228,1334,-1749,-228,1446,-1797,-228,1454,-1813,-228,1494,-1813,-228,1494,-1749,-228,1334,-1749,-228,1374,-1797,-228,1446,-1797,-228,1494,-1749,-228,1398,-1821,-182,1390,-1845,-182,1422,-1853,-182,1430,-1829,-182,1518,-1749,-480,1558,-1749,-480,1558,-1701,-480,1502,-1709,-480,1430,-1701,-480,1422,-1749,-480,1494,-1749,-480,1502,-1709,-480,1558,-1701,-480,1430,-1701,-480,1502,-1709,-480,1438,-1989,-640,1478,-2013,-639,1558,-2013,-639,1558,-1861,-640,1446,-1861,-640,1462,-2125,-640,144`
+`6,-2133,-640,1446,-2173,-640,1470,-2213,-640,1558,-2213,-640,1558,-2125,-640,1446,-1917,-500,1454,-1933,-500,1542,-1933,-500,1542,-1909,-500,1446,-1845,-500,1446,-1861,-500,1542,-1861,-500,1542,-1845,-500,1478,-2013,-639,1454,-2077,-640,1462,-2125,-640,1558,-2125,-640,1558,-2013,-639,1518,-1701,-88,1518,-2213,-88,1558,-2213,-88,1558,-1701,-88,1542,-2045,-288,1542,-2213,-288,1558,-2213,-288,1558,-2045,-288,1542,-2181,-168,1558,-2181,-168,1558,-2069,-168,1542,-2069,-168,1542,-2037,-168,1558,-2037,`
+`-168,1558,-1941,-168,1542,-1933,-168,1542,-1901,-168,1558,-1901,-168,1558,-1797,-168,1542,-1797,-168,1542,-1757,-168,1558,-1757,-168,1558,-1701,-168,1542,-1701,-168,1550,-2213,-415,1558,-2213,-415,1558,-2061,-415],"vertslength":218,"polys":[0,2,3,6,7,9,10,14,15,17,18,20,21,23,24,26,27,29,30,34,35,37,38,40,41,46,47,50,51,54,55,58,59,62,63,66,67,70,71,74,75,79,80,84,85,88,89,92,93,95,96,99,100,102,103,106,107,110,111,114,115,118,119,124,125,127,128,131,132,135,136,139,140,143,144,147,148,151,152,1`
+`55,156,159,160,163,164,166,167,171,172,177,178,181,182,185,186,190,191,194,195,198,199,202,203,206,207,210,211,214,215,217],"polyslength":55,"regions":[1,1,1,1,1,1,3,3,3,3,23,23,23,6,13,13,21,25,14,18,17,10,10,9,26,28,29,2,2,2,2,2,30,4,4,4,12,12,12,22,20,20,20,5,7,34,35,8,19,38,39,40,41,43,44],"neighbors":[[[0],[0],[1,3]],[[0],[1,31],[0],[1,3]],[[0],[0],[1,3]],[[1,2],[1,0],[1,1],[1,5],[0]],[[0],[0],[1,5]],[[0],[1,4],[1,3]],[[0],[0],[1,7]],[[1,6],[0],[1,9]],[[1,43],[0],[1,9]],[[1,13],[0],[0],[1,8`
+`],[1,7]],[[0],[0],[1,12]],[[0],[0],[1,12]],[[0],[0],[1,10],[0],[1,11],[0]],[[0],[0],[0],[1,9]],[[0],[0],[0],[1,15]],[[0],[0],[1,14],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[1,22],[0],[0]],[[0],[0],[0],[1,21]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,29],[0]],[[0],[0],[1,31],[0]],[[1,27],[0],[1,31],[0]],[[0],[0],[0],[1,31]],[[0],[1,29],[1,1],[1,30],[0],[1,28]],[[0],[0],[0]],[[0],[1,36],[0],[1,35`
+`]],[[1,37],[0],[1,35],[0]],[[1,34],[0],[1,33],[0]],[[1,33],[0],[1,38],[0]],[[0],[1,34],[0],[1,38]],[[1,36],[0],[1,37],[0]],[[0],[0],[0],[0]],[[0],[0],[1,42],[0]],[[0],[0],[0],[1,42]],[[0],[1,41],[1,40]],[[0],[1,47],[0],[0],[1,8]],[[0],[0],[0],[0],[0],[1,47]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,44],[0],[1,43]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[1230,-2101,-408,1302,-2093,-408,13`
+`02,-2037,-415,1287.5999755859375,-2049.800048828125,-408,1302,-2037,-415,1326,-2029,-415,1326,-1805,-415,1302,-1797,-416,1046,-2213,-416,1230,-2213,-408,1230,-2101,-408,1046,-2213,-416,1230,-2101,-408,1287.5999755859375,-2049.800048828125,-408,1302,-2037,-415,1302,-1797,-416,1046,-1701,-416,1178,-2105,-408,1310,-1709,-416,1494,-1701,-416,1046,-1701,-416,1302,-1797,-416,1310,-1709,-416,1046,-1701,-416,1318,-1861,-640,1310,-1781,-640,1294,-1829,-640,1318,-1861,-640,1294,-1829,-640,1198,-1821,-636,`
+`1438,-1989,-640,1446,-1861,-640,1318,-1861,-640,1198,-1821,-636,1070,-1829,-640,1062,-1989,-640,1438,-1989,-640,1318,-1861,-640,1438,-1981,-500,1430,-1965,-500,1406,-1981,-500,1102,-1981,-500,1062,-1941,-500,1062,-1989,-500,1542,-1997,-500,1542,-1981,-500,1438,-1981,-500,1406,-1981,-500,1102,-1981,-500,1062,-1989,-500,1198,-1821,-635,1198,-1701,-639,1070,-1701,-639,1070,-1829,-635,1158,-1909,-500,1150,-1829,-500,1110,-1829,-500,1110,-1925,-500,1398,-1933,-500,1398,-1909,-500,1158,-1909,-500,1110`
+`,-1925,-500,1198.6153564453125,-1927.4615478515625,-500,1220.769287109375,-1928.076904296875,-490,1242.923095703125,-1928.6922607421875,-500,1218,-1921,-490,1198,-1861,-500,1398,-1861,-500,1398,-1845,-500,1198,-1829,-500,1222,-1781,-500,1222,-1805,-500,1262,-1805,-500,1262,-1781,-500,1398,-1701,-480,1230,-1701,-480,1230,-1749,-480,1406,-1749,-480,1238,-1741,-768,1558,-1741,-768,1558,-1701,-768,1238,-1701,-768,1286,-1701,-640,1238,-1701,-640,1238,-1741,-640,1558,-1741,-640,1558,-1709,-640,1470,-2`
+`125,-416,1398,-2141,-416,1390,-2165,-416,1494,-2213,-416,1494,-2149,-416,1390,-2165,-416,1270,-2165,-416,1262,-2213,-416,1494,-2213,-416,1262,-2125,-164,1262,-2213,-164,1494,-2213,-164,1494,-2125,-164,1270,-2149,-368,1302,-2149,-368,1302,-2125,-368,1318,-2125,-352,1318,-2149,-352,1382,-2149,-352,1382,-2125,-352,1318,-1741,-212,1494,-1725,-212,1318,-1725,-212,1350,-1741,-416,1334,-1757,-416,1342,-1797,-416,1478,-1741,-416,1558,-1717,-416,1542,-1709,-416,1534,-1805,-416,1558,-2021,-416,1478,-1741,`
+`-416,1342,-1797,-416,1326,-1805,-416,1494,-1797,-416,1326,-2029,-416,1350,-2085,-416,1478,-2085,-416,1502,-2029,-415,1534,-1805,-416,1494,-1797,-416,1326,-1805,-416,1326,-2029,-416,1502,-2029,-415,1558,-2021,-416,1326,-1845,-624,1342,-1837,-624,1326,-1829,-624,1374,-1877,-228,1366,-1813,-228,1334,-1805,-228,1334,-2093,-228,1494,-1813,-228,1454,-1813,-228,1446,-1877,-228,1494,-2093,-228,1494,-2093,-228,1446,-1877,-228,1374,-1877,-228,1334,-2093,-228,1334,-1805,-228,1366,-1813,-228,1374,-1797,-228`
+`,1334,-1749,-228,1446,-1797,-228,1454,-1813,-228,1494,-1813,-228,1494,-1749,-228,1334,-1749,-228,1374,-1797,-228,1446,-1797,-228,1494,-1749,-228,1398,-1821,-182,1390,-1845,-182,1422,-1853,-182,1430,-1829,-182,1518,-1749,-480,1558,-1749,-480,1558,-1701,-480,1502,-1709,-480,1430,-1701,-480,1422,-1749,-480,1494,-1749,-480,1502,-1709,-480,1558,-1701,-480,1430,-1701,-480,1502,-1709,-480,1438,-1989,-640,1478,-2013,-640,1558,-2013,-640,1558,-1861,-640,1446,-1861,-640,1462,-2125,-640,1446,-2133,-640,144`
+`6,-2173,-640,1470,-2213,-640,1558,-2213,-640,1558,-2125,-640,1446,-1917,-500,1454,-1933,-500,1542,-1933,-500,1542,-1909,-500,1446,-1845,-500,1446,-1861,-500,1542,-1861,-500,1542,-1845,-500,1478,-2013,-639,1454,-2077,-640,1462,-2125,-640,1558,-2125,-640,1558,-2013,-639,1518,-1701,-88,1518,-2213,-88,1558,-2213,-88,1558,-1701,-88,1542,-2045,-288,1542,-2213,-288,1558,-2213,-288,1558,-2045,-288,1542,-2181,-168,1558,-2181,-168,1558,-2069,-168,1542,-2069,-168,1542,-2037,-168,1558,-2037,-168,1558,-1941,`
+`-168,1542,-1933,-168,1542,-1901,-168,1558,-1901,-168,1558,-1797,-168,1542,-1797,-168,1542,-1757,-162,1558,-1757,-162,1558,-1701,-168,1542,-1701,-168,1550,-2213,-415,1558,-2213,-415,1558,-2061,-415],"vertslength":225,"tris":[1,2,3,0,1,3,4,5,6,4,6,7,8,9,10,16,11,17,11,12,17,14,13,17,12,13,17,14,15,17,16,15,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,33,33,35,36,38,39,40,41,42,43,44,45,46,44,46,47,47,48,49,44,47,49,50,51,52,50,52,53,54,55,56,54,56,57,60,61,62,64,58,59,59,60,65,64`
+`,59,65,64,63,65,60,62,65,63,62,65,66,67,68,66,68,69,73,70,71,71,72,73,74,75,76,74,76,77,81,78,79,79,80,81,82,83,84,85,86,82,82,84,85,87,88,89,90,91,87,87,89,90,92,93,94,92,94,95,99,96,97,97,98,99,100,101,102,106,103,104,104,105,106,107,108,109,110,111,112,110,112,113,114,115,116,114,116,117,118,119,120,118,120,121,122,123,124,122,124,125,126,127,128,129,130,131,126,128,129,126,129,131,132,133,134,135,136,137,135,137,138,139,140,141,139,141,142,144,145,146,143,144,146,147,148,149,147,149,150,151,`
+`152,153,151,153,154,155,156,157,155,157,158,159,160,161,159,161,162,163,164,165,163,165,166,169,170,167,167,168,169,171,172,173,174,175,176,177,178,174,174,176,177,179,180,181,179,181,182,182,183,184,179,182,184,185,186,187,185,187,188,192,189,190,190,191,192,193,194,195,196,197,193,193,195,196,201,198,199,199,200,201,205,202,203,203,204,205,209,206,207,207,208,209,210,211,212,210,212,213,217,214,215,215,216,217,221,218,219,219,220,221,222,223,224],"trislength":117,"triTopoly":[0,0,1,1,2,3,3,3,3`
+`,3,3,4,5,6,7,8,9,9,9,10,11,12,12,12,12,13,13,14,14,15,15,15,15,15,15,15,16,16,17,17,18,18,19,19,20,20,20,21,21,21,22,22,23,23,24,25,25,26,27,27,28,28,29,29,30,30,31,31,31,31,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,40,40,41,41,42,43,43,43,44,44,44,44,45,45,46,46,47,47,47,48,48,49,49,50,50,51,51,52,52,53,53,54],"baseVert":[0,4,8,11,18,21,24,27,30,33,38,41,44,50,54,58,66,70,74,78,82,87,92,96,100,103,107,110,114,118,122,126,132,135,139,143,147,151,155,159,163,167,171,174,179,185,189,193,198,202`
+`,206,210,214,218,222],"vertsCount":[4,4,3,7,3,3,3,3,3,5,3,3,6,4,4,8,4,4,4,4,5,5,4,4,3,4,3,4,4,4,4,6,3,4,4,4,4,4,4,4,4,4,3,5,6,4,4,5,4,4,4,4,4,4,3],"baseTri":[0,2,4,5,11,12,13,14,15,16,19,20,21,25,27,29,36,38,40,42,44,47,50,52,54,55,57,58,60,62,64,66,70,71,73,75,77,79,81,83,85,87,89,90,93,97,99,101,104,106,108,110,112,114,116],"triCount":[2,2,1,6,1,1,1,1,1,3,1,1,4,2,2,7,2,2,2,2,3,3,2,2,1,2,1,2,2,2,2,4,1,2,2,2,2,2,2,2,2,2,1,3,4,2,2,3,2,2,2,2,2,2,1]},"links":{"poly":[6,32,18,41,22,24,24,25,26,36,33`
+`,39,50,51,51,52],"cost":[520.8712768554688,384,3840,768,516.9835815429688,3764.769287109375,1536,1536],"type":[2,1,2,2,2,2,1,1],"pos":[1316.4949951171875,-1845.950439453125,-640,1326,-1845,-624,1406,-1749,-480,1422,-1749,-480,1302,-2165,-416,1302,-2149,-368,1302,-2149,-368,1318,-2149,-352,1333.1475830078125,-1739.6229248046875,-212,1334,-1749,-228,1370.3077392578125,-1847.4615478515625,-228,1390,-1845,-182,1558,-2069,-168,1558,-2037,-168,1542,-1933,-168,1542,-1901,-168],"length":8}}],["9_1",{"ti`
+`leId":"9_1","tx":9,"ty":1,"mesh":{"verts":[1558,-2213,-640,1598,-2213,-640,1582,-2197,-640,1654,-1789,-640,1622,-1789,-640,1598,-1861,-640,1662,-1965,-640,1558,-2213,-640,1582,-2197,-640,1598,-2085,-640,1574,-2005,-639,1662,-1965,-640,1598,-1861,-640,1558,-1861,-640,1574,-2005,-639,1558,-2213,-640,1574,-2005,-639,1558,-1861,-640,1558,-2213,-415,1622,-2213,-415,1630,-2061,-415,1558,-2053,-415,1558,-2213,-288,1638,-2213,-288,1638,-2053,-288,1558,-2045,-288,1566,-1925,-88,1574,-1757,-88,1558,-1749,`
+`-88,1798,-2117,-88,1830,-2093,-88,1646,-1933,-88,1558,-2213,-88,1798,-2213,-88,1798,-2117,-88,1646,-1933,-88,1566,-1925,-88,1558,-2213,-88,1566,-1925,-88,1558,-1749,-88,2070,-2213,-416,2070,-2197,-416,2006,-2181,-416,2070,-1797,-416,2070,-1709,-416,2038,-1701,-416,1614,-1701,-416,2014,-1805,-416,2022,-2069,-416,2070,-2069,-416,2070,-1917,-416,2014,-1909,-416,2006,-2181,-416,2022,-2069,-416,2014,-1909,-416,1606,-1725,-416,1558,-1717,-416,1558,-2021,-416,1614,-1701,-416,1606,-1725,-416,1558,-2021,`
+`-416,1662,-2037,-416,2006,-2181,-416,2014,-1909,-416,2014,-1805,-416,1614,-1701,-416,1662,-2037,-416,1662,-2213,-416,2070,-2213,-416,2006,-2181,-416,1662,-2037,-416,1558,-1757,-168,1566,-1701,-168,1558,-1701,-168,1630,-1701,-480,1558,-1701,-480,1558,-1749,-480,1638,-1749,-480,1950,-2109,-88,1958,-2061,-88,1942,-2053,-88,1830,-2093,-88,1558,-1701,-88,1558,-1749,-88,1574,-1757,-88,1654,-1757,-88,1942,-1701,-88,1670,-1917,-88,1646,-1933,-88,1830,-2093,-88,1670,-1917,-88,1830,-2093,-88,1942,-2053,-8`
+`8,1942,-1701,-88,1670,-1773,-88,1942,-1701,-88,1654,-1757,-88,1670,-1773,-88,2022,-1741,-768,2046,-1733,-768,2046,-1701,-768,1558,-1701,-768,1558,-1741,-768,2022,-1741,-640,2046,-1733,-640,2046,-1701,-640,1622,-1701,-640,1558,-1709,-640,1558,-1741,-640,1582,-1917,-20,1598,-1909,-20,1582,-1901,-20,1582,-1853,-20,1598,-1837,-20,1582,-1829,-20,1598,-1909,-500,1598,-1933,-500,1654,-1933,-500,1654,-1909,-500,1614,-1837,-88,1638,-1821,-88,1638,-1781,-88,1598,-1781,-88,1638,-1893,-88,1638,-1845,-88,161`
+`4,-1837,-88,1598,-1901,-88,1598,-1901,-88,1614,-1837,-88,1598,-1781,-88,1598,-1893,0,1638,-1893,0,1638,-1861,0,1598,-1853,0,1654,-1861,-500,1654,-1789,-500,1622,-1789,-500,1598,-1861,-500,1598,-1821,0,1638,-1821,0,1638,-1789,0,1598,-1781,0,1702,-2069,-168,1662,-2069,-168,1662,-2165,-168,1702,-2173,-168,1662,-1933,-168,1662,-2013,-168,1702,-2013,-168,1702,-1933,-168,1662,-1813,-168,1662,-1901,-168,1702,-1901,-168,1702,-1797,-168,1702,-1757,-168,1702,-1701,-168,1662,-1701,-168,1662,-1757,-168,1662`
+`,-1701,-480,1662,-1741,-480,1694,-1741,-480,1710,-1709,-480,1710,-1709,-480,1718,-1741,-480,1854,-1749,-480,1846,-1701,-480,1846,-1701,-480,1662,-1701,-480,1710,-1709,-480,1894,-1853,-480,1902,-1837,-480,1686,-1837,-480,1686,-1917,-480,1974,-1973,-640,1966,-1853,-640,1694,-1845,-640,1694,-1981,-640,1902,-1837,-480,1894,-1853,-480,1894,-1877,-480,1966,-1853,-480,2030,-1837,-480,1974,-1837,-480,1966,-1853,-480,2022,-1901,-480,2070,-1989,-480,2070,-1909,-480,2022,-1901,-480,2022,-1901,-480,1966,-18`
+`53,-480,1894,-1877,-480,1702,-1989,-480,2070,-1989,-480,1734,-2069,-168,1734,-2181,-168,1774,-2181,-168,1774,-2069,-168,1734,-1933,-168,1734,-2013,-168,1774,-2013,-168,1774,-1933,-168,1734,-1797,-168,1734,-1901,-168,1774,-1901,-168,1774,-1797,-168,1734,-1725,-162,1734,-1757,-168,1774,-1757,-168,1774,-1725,-162,1806,-2125,-168,1838,-2069,-168,1806,-2069,-168,1806,-1933,-168,1806,-2013,-168,1838,-2013,-168,1838,-1933,-168,1806,-1797,-168,1806,-1901,-168,1838,-1901,-168,1838,-1797,-168,1814,-2125,7`
+`2,1814,-2213,72,1950,-2213,72,1950,-2125,72,1822,-2133,-88,1822,-2213,-88,1942,-2213,-88,1942,-2133,-88,1870,-2125,-162,1878,-2181,-163,1894,-2181,-162,1910,-2069,-168,1870,-2069,-162,1870,-1933,-162,1870,-2013,-162,1910,-2013,-168,1910,-1933,-168,1870,-1797,-162,1870,-1901,-162,1910,-1901,-168,1910,-1797,-168,1870,-1725,-162,1870,-1757,-162,1910,-1757,-168,1910,-1725,-168,2054,-1749,-480,2054,-1701,-480,1878,-1701,-480,1870,-1749,-480,1974,-2069,-168,1942,-2069,-168,1942,-2117,-168,1966,-2149,-`
+`168,1942,-1901,-168,1958,-1845,-168,1942,-1797,-168,1958,-2061,-88,1950,-2109,-88,1966,-2117,-88,2062,-2053,-88,1966,-2117,-88,1966,-2213,-88,2070,-2213,-88,2062,-2053,-88,2070,-1701,-88,2062,-1701,-88,2062,-2053,-88,2070,-2213,-88,2046,-2045,-20,2046,-1701,-20,1958,-1701,-20,1958,-2045,-20,1966,-1861,-88,1966,-2037,-88,2038,-2037,-88,2038,-1861,-88,1966,-1853,-640,1974,-1973,-640,2070,-1981,-640,2070,-1845,-640,2038,-1837,-88,2038,-1701,-88,1966,-1701,-88,1966,-1837,-88,2070,-2093,-224,2046,-20`
+`85,-224,2038,-2117,-224,2038,-2165,-224,2070,-2165,-224,2070,-2101,-414,2046,-2101,-414,2038,-2149,-414,2070,-2165,-414,2070,-2101,-318,2046,-2101,-318,2038,-2149,-318,2070,-2165,-318,2038,-1893,-224,2070,-1893,-224,2070,-1821,-224,2038,-1821,-224,2070,-1837,-480,2038,-1837,-480,2046,-1893,-480,2070,-1901,-480,2046,-1893,-414,2070,-1893,-414,2070,-1829,-414,2046,-1829,-414,2046,-1893,-318,2070,-1893,-318,2070,-1829,-318,2046,-1829,-318],"vertslength":313,"polys":[0,2,3,6,7,10,11,14,15,17,18,21,2`
+`2,25,26,28,29,31,32,36,37,39,40,42,43,47,48,51,52,54,55,57,58,61,62,66,67,70,71,73,74,77,78,81,82,86,87,89,90,94,95,97,98,102,103,108,109,111,112,114,115,118,119,122,123,126,127,129,130,133,134,137,138,141,142,145,146,149,150,153,154,157,158,161,162,165,166,168,169,172,173,176,177,180,181,184,185,187,188,192,193,196,197,200,201,204,205,208,209,211,212,215,216,219,220,223,224,227,228,232,233,236,237,240,241,244,245,248,249,252,253,255,256,259,260,263,264,267,268,271,272,275,276,279,280,283,284,28`
+`8,289,292,293,296,297,300,301,304,305,308,309,312],"polyslength":80,"regions":[7,7,7,7,7,13,11,3,3,3,3,1,1,1,1,1,1,1,1,50,17,2,2,2,2,2,26,39,51,52,56,24,24,24,22,23,25,27,28,29,30,31,31,31,14,4,6,6,6,6,32,33,34,40,59,41,42,9,12,35,36,37,43,18,44,63,8,8,8,10,15,5,16,45,66,67,46,68,69,70],"neighbors":[[[0],[0],[1,2]],[[0],[0],[1,3],[0]],[[1,0],[0],[0],[1,4]],[[1,1],[0],[1,4],[0]],[[1,2],[1,3],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,22],[1,10]],[[0],[1,23],[1,9]],[[0],[0],[1,8],[0],[1,10]]`
+`,[[1,9],[1,7],[0]],[[0],[0],[1,18]],[[0],[0],[0],[1,17],[0]],[[0],[0],[0],[1,14]],[[0],[1,13],[1,17]],[[0],[0],[1,16]],[[0],[1,15],[0],[1,17]],[[1,14],[0],[1,12],[1,16],[1,18]],[[0],[1,11],[1,17],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[1,66],[0],[1,24],[0]],[[0],[1,7],[0],[1,25],[0]],[[0],[1,8],[1,24]],[[1,23],[1,21],[0],[1,25],[0]],[[1,22],[0],[1,24]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,33]],[[0],[0],[1,33],[0]],[[1,32],[1`
+`,31],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,43]],[[0],[0],[0],[1,43]],[[0],[1,41],[1,42]],[[1,46],[0],[0],[0]],[[1,71],[0],[0],[0]],[[1,44],[0],[1,49],[0]],[[0],[0],[1,49],[0]],[[0],[0],[1,49]],[[1,47],[1,46],[0],[0],[1,48]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],`
+`[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[1,21],[0],[1,67],[0]],[[0],[0],[1,68],[1,66]],[[0],[0],[1,67],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,45],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1558,-2213,-640,1598,-2213,-640,1582,-2197,-640,1654,-1789,-640,1622,-1789,-640,1598,-18`
+`61,-640,1662,-1965,-640,1558,-2213,-640,1582,-2197,-640,1598,-2085,-640,1574,-2005,-640,1662,-1965,-640,1598,-1861,-640,1558,-1861,-640,1574,-2005,-640,1558,-2213,-640,1574,-2005,-640,1558,-1861,-640,1558,-2213,-415,1622,-2213,-415,1630,-2061,-415,1558,-2053,-415,1558,-2213,-288,1638,-2213,-288,1638,-2053,-288,1558,-2045,-288,1566,-1925,-88,1574,-1757,-88,1558,-1749,-88,1798,-2117,-88,1830,-2093,-88,1646,-1933,-88,1558,-2213,-88,1798,-2213,-88,1798,-2117,-88,1646,-1933,-88,1566,-1925,-88,1558,-2`
+`213,-88,1566,-1925,-88,1558,-1749,-88,2070,-2213,-416,2070,-2197,-416,2006,-2181,-416,2070,-1797,-416,2070,-1709,-416,2038,-1701,-416,1614,-1701,-416,2014,-1805,-416,2022,-2069,-416,2070,-2069,-416,2070,-1917,-416,2014,-1909,-416,2006,-2181,-416,2022,-2069,-416,2014,-1909,-416,1606,-1725,-416,1558,-1717,-416,1558,-2021,-416,1614,-1701,-416,1606,-1725,-416,1558,-2021,-416,1662,-2037,-416,2006,-2181,-416,2014,-1909,-416,2014,-1805,-416,1614,-1701,-416,1662,-2037,-416,1662,-2213,-416,2070,-2213,-41`
+`6,2006,-2181,-416,1662,-2037,-416,1558,-1757,-162,1566,-1701,-168,1558,-1701,-168,1630,-1701,-480,1558,-1701,-480,1558,-1749,-480,1638,-1749,-480,1950,-2109,-88,1958,-2061,-88,1942,-2053,-88,1830,-2093,-88,1558,-1701,-88,1558,-1749,-88,1574,-1757,-88,1654,-1757,-88,1942,-1701,-88,1670,-1917,-88,1646,-1933,-88,1830,-2093,-88,1670,-1917,-88,1830,-2093,-88,1942,-2053,-88,1942,-1701,-88,1670,-1773,-88,1942,-1701,-88,1654,-1757,-88,1670,-1773,-88,2022,-1741,-768,2046,-1733,-768,2046,-1701,-768,1558,-`
+`1701,-768,1558,-1741,-768,2022,-1741,-640,2046,-1733,-640,2046,-1701,-640,1622,-1701,-640,1558,-1709,-640,1558,-1741,-640,1582,-1917,-20,1598,-1909,-20,1582,-1901,-20,1582,-1853,-20,1598,-1837,-20,1582,-1829,-20,1598,-1909,-500,1598,-1933,-500,1654,-1933,-500,1654,-1909,-500,1614,-1837,-88,1638,-1821,-88,1638,-1781,-88,1598,-1781,-88,1638,-1893,-88,1638,-1845,-88,1614,-1837,-88,1598,-1901,-88,1598,-1901,-88,1614,-1837,-88,1598,-1781,-88,1598,-1893,8,1638,-1893,0,1638,-1861,0,1598,-1853,0,1654,-1`
+`861,-500,1654,-1789,-500,1622,-1789,-500,1598,-1861,-500,1598,-1821,8,1638,-1821,0,1638,-1789,0,1598,-1781,0,1702,-2069,-168,1662,-2069,-168,1662,-2165,-168,1702,-2173,-168,1662,-1933,-168,1662,-2013,-168,1702,-2013,-168,1702,-1933,-168,1662,-1813,-168,1662,-1901,-168,1702,-1901,-168,1702,-1797,-168,1702,-1757,-162,1702,-1701,-168,1662,-1701,-168,1662,-1757,-162,1662,-1701,-480,1662,-1741,-480,1694,-1741,-480,1710,-1709,-480,1710,-1709,-480,1718,-1741,-480,1854,-1749,-480,1846,-1701,-480,1846,-1`
+`701,-480,1662,-1701,-480,1710,-1709,-480,1894,-1853,-480,1902,-1837,-480,1686,-1837,-480,1686,-1917,-480,1974,-1973,-640,1966,-1853,-640,1694,-1845,-640,1694,-1981,-640,1902,-1837,-480,1894,-1853,-480,1894,-1877,-480,1966,-1853,-480,2030,-1837,-480,1974,-1837,-480,1966,-1853,-480,2022,-1901,-480,2070,-1989,-480,2070,-1909,-480,2022,-1901,-480,2022,-1901,-480,1966,-1853,-480,1894,-1877,-480,1702,-1989,-480,2070,-1989,-480,1734,-2069,-168,1734,-2181,-168,1774,-2181,-168,1774,-2069,-168,1734,-1933,`
+`-168,1734,-2013,-168,1774,-2013,-168,1774,-1933,-168,1734,-1797,-168,1734,-1901,-168,1774,-1901,-168,1774,-1797,-168,1734,-1725,-162,1734,-1757,-162,1774,-1757,-162,1774,-1725,-162,1806,-2125,-168,1838,-2069,-168,1806,-2069,-168,1806,-1933,-168,1806,-2013,-168,1838,-2013,-168,1838,-1933,-168,1806,-1797,-168,1806,-1901,-168,1838,-1901,-168,1838,-1797,-168,1814,-2125,72,1814,-2213,72,1950,-2213,72,1950,-2125,72,1822,-2133,-88,1822,-2213,-88,1942,-2213,-88,1942,-2133,-88,1870,-2125,-163,1878,-2181,`
+`-162,1894,-2181,-162,1910,-2069,-168,1870,-2069,-163,1870,-1933,-163,1870,-2013,-163,1910,-2013,-168,1910,-1933,-168,1870,-1797,-163,1870,-1901,-163,1910,-1901,-168,1910,-1797,-168,1870,-1725,-163,1870,-1757,-168,1910,-1757,-168,1910,-1725,-168,1882,-1745,-162,2054,-1749,-480,2054,-1701,-480,1878,-1701,-480,1870,-1749,-480,1974,-2069,-168,1942,-2069,-168,1942,-2117,-168,1966,-2149,-168,1942,-1901,-168,1958,-1845,-168,1942,-1797,-168,1958,-2061,-88,1950,-2109,-88,1966,-2117,-88,2062,-2053,-88,196`
+`6,-2117,-88,1966,-2213,-88,2070,-2213,-88,2062,-2053,-88,2070,-1701,-88,2062,-1701,-88,2062,-2053,-88,2070,-2213,-88,2046,-2045,-20,2046,-1701,-20,1958,-1701,-20,1958,-2045,-20,1966,-1861,-88,1966,-2037,-88,2038,-2037,-88,2038,-1861,-88,1966,-1853,-640,1974,-1973,-640,2070,-1981,-640,2070,-1845,-640,2038,-1837,-88,2038,-1701,-88,1966,-1701,-88,1966,-1837,-88,2070,-2093,-224,2046,-2085,-224,2038,-2117,-224,2038,-2165,-224,2070,-2165,-224,2070,-2101,-414,2046,-2101,-414,2038,-2149,-414,2070,-2165,`
+`-414,2070,-2101,-318,2046,-2101,-318,2038,-2149,-318,2070,-2165,-318,2038,-1893,-224,2070,-1893,-224,2070,-1821,-224,2038,-1821,-224,2070,-1837,-480,2038,-1837,-480,2046,-1893,-480,2070,-1901,-480,2046,-1893,-414,2070,-1893,-414,2070,-1829,-414,2046,-1829,-414,2046,-1893,-318,2070,-1893,-318,2070,-1829,-318,2046,-1829,-318],"vertslength":314,"tris":[0,1,2,3,4,5,3,5,6,7,8,9,7,9,10,11,12,13,11,13,14,15,16,17,18,19,20,18,20,21,22,23,24,22,24,25,26,27,28,29,30,31,32,33,34,34,35,36,32,34,36,37,38,39,`
+`40,41,42,43,44,45,47,43,45,45,46,47,48,49,50,48,50,51,52,53,54,55,56,57,58,59,60,58,60,61,62,63,64,66,62,64,64,65,66,67,68,69,67,69,70,71,72,73,74,75,76,74,76,77,78,79,80,78,80,81,82,83,84,82,84,85,82,85,86,87,88,89,90,91,92,93,94,90,90,92,93,95,96,97,98,99,100,101,102,98,98,100,101,103,104,105,106,107,108,103,105,106,103,106,108,109,110,111,112,113,114,118,115,116,116,117,118,119,120,121,119,121,122,123,124,125,123,125,126,127,128,129,130,131,132,130,132,133,134,135,136,134,136,137,138,139,140,`
+`138,140,141,142,143,144,142,144,145,149,146,147,147,148,149,150,151,152,150,152,153,157,154,155,155,156,157,158,159,160,158,160,161,165,162,163,163,164,165,166,167,168,169,170,171,169,171,172,173,174,175,173,175,176,177,178,179,177,179,180,181,182,183,181,183,184,185,186,187,188,189,190,192,188,190,190,191,192,196,193,194,194,195,196,200,197,198,198,199,200,204,201,202,202,203,204,208,205,206,206,207,208,209,210,211,215,212,213,213,214,215,219,216,217,217,218,219,223,220,221,221,222,223,227,224,`
+`225,225,226,227,228,229,230,231,232,228,228,230,231,236,233,234,234,235,236,240,237,238,238,239,240,244,241,245,241,242,245,242,243,245,244,243,245,246,247,248,246,248,249,250,251,252,250,252,253,254,255,256,257,258,259,257,259,260,261,262,263,261,263,264,265,266,267,265,267,268,272,269,270,270,271,272,276,273,274,274,275,276,277,278,279,277,279,280,284,281,282,282,283,284,285,286,287,287,288,289,285,287,289,290,291,292,290,292,293,294,295,296,294,296,297,301,298,299,299,300,301,302,303,304,302,`
+`304,305,309,306,307,307,308,309,313,310,311,311,312,313],"trislength":155,"triTopoly":[0,1,1,2,2,3,3,4,5,5,6,6,7,8,9,9,9,10,11,12,12,12,13,13,14,15,16,16,17,17,17,18,18,19,20,20,21,21,22,22,22,23,24,24,24,25,26,26,26,27,27,27,27,28,29,30,30,31,31,32,32,33,34,34,35,35,36,36,37,37,38,38,39,39,40,40,41,41,42,42,43,44,44,45,45,46,46,47,47,48,49,49,49,50,50,51,51,52,52,53,53,54,55,55,56,56,57,57,58,58,59,59,59,60,60,61,61,62,62,62,62,63,63,64,64,65,66,66,67,67,68,68,69,69,70,70,71,71,72,72,73,73,73,7`
+`4,74,75,75,76,76,77,77,78,78,79,79],"baseVert":[0,3,7,11,15,18,22,26,29,32,37,40,43,48,52,55,58,62,67,71,74,78,82,87,90,95,98,103,109,112,115,119,123,127,130,134,138,142,146,150,154,158,162,166,169,173,177,181,185,188,193,197,201,205,209,212,216,220,224,228,233,237,241,246,250,254,257,261,265,269,273,277,281,285,290,294,298,302,306,310],"vertsCount":[3,4,4,4,3,4,4,3,3,5,3,3,5,4,3,3,4,5,4,3,4,4,5,3,5,3,5,6,3,3,4,4,4,3,4,4,4,4,4,4,4,4,4,3,4,4,4,4,3,5,4,4,4,4,3,4,4,4,4,5,4,4,5,4,4,3,4,4,4,4,4,4,4,5`
+`,4,4,4,4,4,4],"baseTri":[0,1,3,5,7,8,10,12,13,14,17,18,19,22,24,25,26,28,31,33,34,36,38,41,42,45,46,49,53,54,55,57,59,61,62,64,66,68,70,72,74,76,78,80,81,83,85,87,89,90,93,95,97,99,101,102,104,106,108,110,113,115,117,121,123,125,126,128,130,132,134,136,138,140,143,145,147,149,151,153],"triCount":[1,2,2,2,1,2,2,1,1,3,1,1,3,2,1,1,2,3,2,1,2,2,3,1,3,1,3,4,1,1,2,2,2,1,2,2,2,2,2,2,2,2,2,1,2,2,2,2,1,3,2,2,2,2,1,2,2,2,2,3,2,2,4,2,2,1,2,2,2,2,2,2,2,3,2,2,2,2,2,2]},"links":{"poly":[20,41,28,34,29,34,29,36`
+`,34,36,37,50,38,39,38,51,39,52,40,53,42,63,47,77,50,54,51,52,51,55,52,56,54,59,55,56,55,60,56,61,59,64,60,61,61,65],"cost":[936.6486206054688,907.2000122070312,792,907.2000122070312,1536,1536,1536,1536,1536,1536,384,96,1536,1536,1536,1536,1590,1536,1590,1590,1536,1536,1536],"type":[1,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],"pos":[1637.351318359375,-1745.108154296875,-480,1662,-1741,-480,1591.5999755859375,-1905.800048828125,-20,1598,-1893,0,1590,-1845,-20,1598,-1853,0,1591.5999755859375,-18`
+`33.800048828125,-20,1598,-1821,0,1598,-1853,0,1598,-1821,0,1702,-2069,-168,1734,-2069,-168,1662,-1933,-168,1662,-1901,-168,1702,-2013,-168,1734,-2013,-168,1702,-1901,-168,1734,-1901,-168,1702,-1757,-168,1734,-1757,-168,1854,-1749,-480,1870,-1749,-480,2030,-1837,-480,2038,-1837,-480,1774,-2125,-168,1806,-2125,-168,1734,-1933,-168,1734,-1901,-168,1774,-2013,-168,1806,-2013,-168,1774,-1901,-168,1806,-1901,-168,1838,-2069,-168,1870,-2069,-162,1806,-1933,-168,1806,-1901,-168,1838,-2013,-168,1870,-201`
+`3,-162,1838,-1901,-168,1870,-1901,-162,1910,-2069,-168,1942,-2069,-168,1870,-1933,-162,1870,-1901,-162,1910,-1901,-168,1942,-1901,-168],"length":23}}],["10_1",{"tileId":"10_1","tx":10,"ty":1,"mesh":{"verts":[2070,-2197,-416,2070,-2213,-416,2110,-2213,-416,2110,-2197,-416,2070,-1701,-88,2070,-2213,-88,2134,-2213,-88,2134,-1701,-88,2070,-2173,-224,2110,-2173,-224,2118,-2157,-224,2118,-2101,-224,2070,-2093,-224,2102,-2165,-414,2110,-2133,-414,2110,-2109,-414,2070,-2101,-414,2070,-2165,-414,2102,-21`
+`65,-318,2110,-2133,-318,2110,-2109,-318,2070,-2101,-318,2070,-2165,-318,2118,-1933,-416,2110,-1917,-416,2070,-1917,-416,2070,-2069,-416,2110,-2069,-416,2118,-2037,-416,2070,-1989,-480,2134,-1989,-480,2134,-1933,-480,2070,-1917,-480,2198,-1965,-640,2190,-1853,-640,2070,-1845,-640,2070,-1981,-640,2070,-1893,-480,2110,-1893,-480,2118,-1845,-480,2070,-1837,-480,2070,-1829,-414,2070,-1893,-414,2110,-1893,-414,2110,-1829,-414,2070,-1829,-318,2070,-1893,-318,2110,-1893,-318,2110,-1829,-318,2070,-1821,-`
+`224,2070,-1893,-224,2118,-1893,-224,2118,-1821,-224,2070,-1797,-416,2110,-1797,-416,2118,-1757,-416,2118,-1709,-416,2070,-1709,-416,2078,-2069,-168,2078,-2181,-168,2118,-2181,-168,2118,-2069,-168,2118,-1933,-168,2078,-1933,-168,2078,-2005,-168,2118,-2037,-168,2078,-1797,-168,2078,-1901,-168,2118,-1901,-168,2118,-1797,-168,2118,-1757,-168,2118,-1701,-168,2078,-1701,-168,2078,-1757,-168,2150,-1933,-480,2158,-1981,-480,2190,-1973,-480,2182,-1845,-480,2166,-1837,-480,2158,-2189,-100,2158,-2213,-100,`
+`2294,-2213,-100,2294,-2189,-100,2350,-1805,-232,2326,-1805,-232,2326,-1941,-232,2350,-2157,-232,2230,-1957,-232,2222,-1797,-232,2158,-1789,-232,2158,-2173,-232,2350,-2157,-232,2326,-1941,-232,2310,-1957,-232,2350,-2157,-232,2310,-1957,-232,2230,-1957,-232,2158,-2173,-232,2310,-1781,-232,2326,-1805,-232,2350,-1805,-232,2350,-1741,-232,2158,-1789,-232,2222,-1797,-232,2230,-1781,-232,2158,-1741,-232,2230,-1781,-232,2310,-1781,-232,2350,-1741,-232,2158,-1741,-232,2502,-1725,-60,2502,-1701,-60,2158,-`
+`1701,-60,2158,-1725,-60,2238,-1797,-152,2238,-1941,-152,2302,-1941,-152,2302,-1797,-152,2254,-1805,-232,2254,-1925,-232,2294,-1925,-232,2294,-1805,-232,2486,-1757,-408,2462,-1741,-408,2382,-1741,-408,2382,-2157,-408,2486,-2157,-408,2526,-1845,-168,2526,-1701,-168,2518,-1701,-168,2518,-2157,-168,2518,-2157,-168,2582,-2157,-168,2582,-1861,-168,2526,-1845,-168,2550,-1845,-88,2582,-1845,-88,2582,-1701,-88,2550,-1701,-88,2558,-1829,-168,2582,-1829,-168,2582,-1709,-168,2558,-1709,-168],"vertslength":1`
+`43,"polys":[0,3,4,7,8,12,13,17,18,22,23,28,29,32,33,36,37,40,41,44,45,48,49,52,53,57,58,61,62,65,66,69,70,73,74,78,79,82,83,86,87,90,91,93,94,97,98,101,102,105,106,109,110,113,114,117,118,121,122,126,127,130,131,134,135,138,139,142],"polyslength":34,"regions":[25,5,9,17,18,10,8,2,13,14,15,11,12,16,19,20,21,23,31,1,1,1,1,4,4,4,32,6,22,3,7,7,24,36],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[`
+`0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,23],[0],[1,21],[0]],[[0],[1,24],[0],[1,22]],[[1,19],[0],[1,22]],[[1,21],[0],[1,20],[0]],[[0],[1,19],[0],[1,25]],[[1,20],[0],[1,25],[0]],[[0],[1,23],[0],[1,24]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[1,31]],[[0],[0`
+`],[0],[1,30]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2070,-2197,-416,2070,-2213,-416,2110,-2213,-416,2110,-2197,-416,2070,-1701,-88,2070,-2213,-88,2134,-2213,-88,2134,-1701,-88,2070,-2173,-224,2110,-2173,-224,2118,-2157,-224,2118,-2101,-224,2070,-2093,-224,2102,-2165,-414,2110,-2133,-414,2110,-2109,-414,2070,-2101,-414,2070,-2165,-414,2102,-2165,-318,2110,-2133,-318,2110,-2109,-318,2070,-2101,-318,2070,-2165,-318,2118,-1933,-416,2110,-1917,-416,2070,-1917,-416,2070,-2069,-416,2`
+`110,-2069,-416,2118,-2037,-416,2070,-1989,-480,2134,-1989,-480,2134,-1933,-480,2070,-1917,-480,2198,-1965,-640,2190,-1853,-640,2070,-1845,-640,2070,-1981,-640,2070,-1893,-480,2110,-1893,-480,2118,-1845,-480,2070,-1837,-480,2070,-1829,-414,2070,-1893,-414,2110,-1893,-414,2110,-1829,-414,2070,-1829,-318,2070,-1893,-318,2110,-1893,-318,2110,-1829,-318,2070,-1821,-224,2070,-1893,-224,2118,-1893,-224,2118,-1821,-224,2070,-1797,-416,2110,-1797,-416,2118,-1757,-416,2118,-1709,-416,2070,-1709,-416,2078,`
+`-2069,-168,2078,-2181,-168,2118,-2181,-168,2118,-2069,-168,2118,-1933,-168,2078,-1933,-168,2078,-2005,-168,2118,-2037,-168,2078,-1797,-168,2078,-1901,-168,2118,-1901,-168,2118,-1797,-168,2118,-1757,-168,2118,-1701,-168,2078,-1701,-168,2078,-1757,-168,2150,-1933,-480,2158,-1981,-480,2190,-1973,-480,2182,-1845,-480,2166,-1837,-480,2158,-2189,-100,2158,-2213,-100,2294,-2213,-100,2294,-2189,-100,2350,-1805,-232,2326,-1805,-232,2326,-1941,-232,2350,-2157,-232,2230,-1957,-232,2222,-1797,-232,2158,-178`
+`9,-232,2158,-2173,-232,2350,-2157,-232,2326,-1941,-232,2310,-1957,-232,2350,-2157,-232,2310,-1957,-232,2230,-1957,-232,2158,-2173,-232,2310,-1781,-232,2326,-1805,-232,2350,-1805,-232,2350,-1741,-232,2158,-1789,-232,2222,-1797,-232,2230,-1781,-232,2158,-1741,-232,2230,-1781,-232,2310,-1781,-232,2350,-1741,-232,2158,-1741,-232,2502,-1725,-60,2502,-1701,-60,2158,-1701,-60,2158,-1725,-60,2238,-1797,-152,2238,-1941,-152,2302,-1941,-152,2302,-1797,-152,2254,-1805,-232,2254,-1925,-232,2294,-1925,-232,2`
+`294,-1805,-232,2486,-1757,-408,2462,-1741,-408,2382,-1741,-408,2382,-2157,-408,2486,-2157,-408,2526,-1845,-168,2526,-1701,-168,2518,-1701,-168,2518,-2157,-168,2518,-2157,-168,2582,-2157,-168,2582,-1861,-168,2526,-1845,-168,2550,-1845,-88,2582,-1845,-88,2582,-1701,-88,2550,-1701,-88,2558,-1829,-168,2582,-1829,-168,2582,-1709,-168,2558,-1709,-168],"vertslength":143,"tris":[3,0,1,1,2,3,7,4,5,5,6,7,8,9,10,10,11,12,8,10,12,13,14,15,17,13,15,15,16,17,18,19,20,22,18,20,20,21,22,23,24,25,26,27,28,28,23,`
+`25,25,26,28,29,30,31,29,31,32,33,34,35,33,35,36,37,38,39,37,39,40,44,41,42,42,43,44,48,45,46,46,47,48,52,49,50,50,51,52,53,54,55,55,56,57,53,55,57,61,58,59,59,60,61,62,63,64,62,64,65,69,66,67,67,68,69,73,70,71,71,72,73,74,75,76,77,78,74,74,76,77,82,79,80,80,81,82,83,84,85,83,85,86,87,88,89,87,89,90,91,92,93,94,95,96,94,96,97,98,99,100,98,100,101,102,103,104,102,104,105,106,107,108,106,108,109,113,110,111,111,112,113,117,114,115,115,116,117,121,118,119,119,120,121,122,123,124,125,126,122,122,124,`
+`125,127,128,129,127,129,130,131,132,133,131,133,134,138,135,136,136,137,138,142,139,140,140,141,142],"trislength":75,"triTopoly":[0,0,1,1,2,2,2,3,3,3,4,4,4,5,5,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,12,13,13,14,14,15,15,16,16,17,17,17,18,18,19,19,20,20,21,22,22,23,23,24,24,25,25,26,26,27,27,28,28,29,29,29,30,30,31,31,32,32,33,33],"baseVert":[0,4,8,13,18,23,29,33,37,41,45,49,53,58,62,66,70,74,79,83,87,91,94,98,102,106,110,114,118,122,127,131,135,139],"vertsCount":[4,4,5,5,5,6,4,4,4,4,4,4,5,4,4,4,4`
+`,5,4,4,4,3,4,4,4,4,4,4,4,5,4,4,4,4],"baseTri":[0,2,4,7,10,13,17,19,21,23,25,27,29,32,34,36,38,40,43,45,47,49,50,52,54,56,58,60,62,64,67,69,71,73],"triCount":[2,2,3,3,3,4,2,2,2,2,2,2,3,2,2,2,2,3,2,2,2,1,2,2,2,2,2,2,2,3,2,2,2,2]},"links":{"poly":[6,17,6,8,13,14,14,15],"cost":[384,864,1536,1536],"type":[1,1,1,1],"pos":[2134,-1933,-480,2150,-1933,-480,2070,-1917,-480,2070,-1893,-480,2118,-2069,-168,2118,-2037,-168,2078,-1933,-168,2078,-1901,-168],"length":4}}],["11_1",{"tileId":"11_1","tx":11,"ty":1`
+`,"mesh":{"verts":[2646,-1701,-168,2630,-1701,-168,2630,-1853,-168,2646,-2157,-168,2630,-1853,-168,2582,-1861,-168,2582,-2157,-168,2646,-2157,-168,2582,-1701,-88,2582,-1845,-88,2614,-1845,-88,2614,-1701,-88,2582,-1709,-168,2582,-1829,-168,2598,-1829,-168,2598,-1709,-168],"vertslength":16,"polys":[0,3,4,7,8,11,12,15],"polyslength":4,"regions":[1,1,2,3],"neighbors":[[[0],[0],[1,1],[0]],[[0],[0],[0],[1,0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2646,-1701,-168,2630,-1701,-168,2630,`
+`-1853,-168,2646,-2157,-168,2630,-1853,-168,2582,-1861,-168,2582,-2157,-168,2646,-2157,-168,2582,-1701,-88,2582,-1845,-88,2614,-1845,-88,2614,-1701,-88,2582,-1709,-168,2582,-1829,-168,2598,-1829,-168,2598,-1709,-168],"vertslength":16,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,11,8,9,9,10,11,15,12,13,13,14,15],"trislength":8,"triTopoly":[0,0,1,1,2,2,3,3],"baseVert":[0,4,8,12],"vertsCount":[4,4,4,4],"baseTri":[0,2,4,6],"triCount":[2,2,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_1"`
+`,{"tileId":"12_1","tx":12,"ty":1,"mesh":{"verts":[3550,-1701,-352,3542,-1701,-352,3542,-2093,-352],"vertslength":3,"polys":[0,2],"polyslength":1,"regions":[2],"neighbors":[[[0],[0],[0]]]},"detail":{"verts":[3550,-1701,-352,3542,-1701,-352,3542,-2093,-352],"vertslength":3,"tris":[0,1,2],"trislength":1,"triTopoly":[0],"baseVert":[0],"vertsCount":[3],"baseTri":[0],"triCount":[1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["0_2",{"tileId":"0_2","tx":0,"ty":2,"mesh":{"verts":[-304`
+`2,-1701,-464,-3034,-1701,-464,-3034,-1277,-464,-3010,-1261,-164,-3010,-1285,-152,-2946,-1269,-152,-3026,-1189,-172,-3026,-1269,-172,-3010,-1261,-164,-3026,-1189,-172,-3010,-1261,-164,-2946,-1269,-152,-2538,-1269,-164,-2538,-1189,-164,-2898,-1453,-161,-2898,-1437,-157,-2962,-1429,-159,-2994,-1453,-168,-2994,-1453,-168,-2962,-1429,-159,-2954,-1317,-159,-2994,-1293,-168,-2954,-1317,-159,-2754,-1317,-168,-2746,-1293,-168,-2994,-1293,-168,-2538,-1189,-416,-2994,-1189,-408,-2994,-1261,-408,-2538,-1269`
+`,-416,-2754,-1421,-64,-2754,-1381,-64,-2802,-1413,-64,-2834,-1389,-64,-2818,-1333,-64,-2946,-1333,-64,-2946,-1421,-64,-2802,-1413,-64,-2834,-1389,-64,-2946,-1421,-64,-2946,-1421,-64,-2754,-1421,-64,-2802,-1413,-64,-2938,-1341,-144,-2938,-1413,-144,-2762,-1413,-144,-2762,-1341,-144,-2746,-1293,-168,-2754,-1317,-168,-2738,-1325,-168,-2690,-1317,-168,-2690,-1317,-168,-2538,-1317,-168,-2538,-1293,-168,-2746,-1293,-168,-2898,-1453,-161,-2538,-1453,-161,-2538,-1437,-157,-2698,-1437,-168,-2738,-1437,-1`
+`68,-2898,-1437,-157,-2698,-1437,-168,-2690,-1317,-168,-2738,-1325,-168,-2738,-1437,-168,-2778,-1349,-44,-2818,-1349,-44,-2818,-1381,-44,-2778,-1389,-44,-2770,-1341,-64,-2754,-1357,-64,-2754,-1333,-64,-2650,-1413,-64,-2682,-1389,-64,-2682,-1421,-64,-2538,-1421,-64,-2538,-1397,-64,-2578,-1413,-64,-2578,-1413,-64,-2610,-1389,-64,-2650,-1413,-64,-2578,-1413,-64,-2650,-1413,-64,-2682,-1421,-64,-2538,-1421,-64,-2666,-1413,-144,-2538,-1413,-144,-2538,-1341,-144,-2666,-1341,-144,-2658,-1349,-44,-2666,-1`
+`381,-44,-2626,-1389,-44,-2618,-1357,-44,-2618,-1341,-64,-2602,-1349,-64,-2594,-1333,-64,-2586,-1349,-44,-2586,-1389,-44,-2554,-1389,-44,-2546,-1349,-44],"vertslength":100,"polys":[0,2,3,5,6,8,9,13,14,17,18,21,22,25,26,29,30,32,33,36,37,39,40,42,43,46,47,50,51,54,55,60,61,64,65,68,69,71,72,74,75,77,78,80,81,84,85,88,89,92,93,95,96,99],"polyslength":27,"regions":[12,1,1,1,7,7,7,3,2,2,2,2,4,6,6,6,6,8,13,11,11,11,11,5,9,15,10],"neighbors":[[[0],[0],[0]],[[0],[0],[1,3]],[[0],[0],[1,3]],[[1,2],[1,1],[`
+`0],[0],[0]],[[1,15],[0],[1,5],[0]],[[1,4],[0],[1,6],[0]],[[0],[1,13],[0],[1,5]],[[0],[0],[0],[0]],[[0],[0],[1,11]],[[0],[0],[0],[1,10]],[[0],[1,9],[1,11]],[[0],[1,8],[1,10]],[[0],[0],[0],[0]],[[1,6],[0],[1,16],[1,14]],[[0],[0],[0],[1,13]],[[0],[0],[0],[1,16],[0],[1,4]],[[0],[1,13],[0],[1,15]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,22]],[[0],[0],[1,22]],[[0],[0],[1,22]],[[1,21],[1,19],[0],[1,20]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-3042,-`
+`1701,-464,-3034,-1701,-464,-3034,-1277,-464,-3010,-1261,-164,-3010,-1273,-164,-3010,-1285,-152,-2946,-1269,-164,-3026,-1189,-172,-3026,-1269,-172,-3010,-1261,-164,-3026,-1189,-172,-3010,-1261,-164,-2946,-1269,-164,-2538,-1269,-164,-2538,-1189,-164,-3002.761962890625,-1189,-164,-2898,-1453,-157,-2898,-1437,-157,-2940.666748046875,-1431.6666259765625,-157,-2962,-1429,-168,-2994,-1453,-168,-2974.800048828125,-1453,-168,-2955.60009765625,-1453,-157,-2994,-1453,-168,-2962,-1429,-168,-2954,-1317,-168,`
+`-2994,-1293,-168,-2954,-1317,-168,-2754,-1317,-168,-2746,-1293,-168,-2994,-1293,-168,-2538,-1189,-416,-2674.800048828125,-1189,-416,-2697.60009765625,-1189,-408,-2925.60009765625,-1189,-416,-2948.39990234375,-1189,-408,-2994,-1189,-408,-2994,-1261,-408,-2948.39990234375,-1261.800048828125,-408,-2925.60009765625,-1262.199951171875,-416,-2788.800048828125,-1264.5999755859375,-416,-2766,-1265,-408,-2538,-1269,-416,-2670,-1209,-416,-2790,-1233,-416,-2754,-1421,-64,-2754,-1381,-64,-2802,-1413,-64,-28`
+`34,-1389,-64,-2818,-1333,-64,-2946,-1333,-64,-2946,-1421,-64,-2802,-1413,-64,-2834,-1389,-64,-2946,-1421,-64,-2946,-1421,-64,-2754,-1421,-64,-2802,-1413,-64,-2938,-1341,-144,-2938,-1413,-144,-2762,-1413,-144,-2762,-1341,-144,-2746,-1293,-168,-2754,-1317,-168,-2738,-1325,-168,-2690,-1317,-168,-2690,-1317,-168,-2538,-1317,-168,-2538,-1293,-168,-2746,-1293,-168,-2898,-1453,-157,-2763,-1453,-157,-2740.5,-1453,-168,-2718,-1453,-168,-2695.5,-1453,-157,-2538,-1453,-157,-2538,-1437,-157,-2698,-1437,-157`
+`,-2718,-1437,-168,-2738,-1437,-168,-2760.857177734375,-1437,-168,-2783.71435546875,-1437,-157,-2898,-1437,-157,-2766,-1441,-157,-2698,-1437,-157,-2696.666748046875,-1417,-168,-2690,-1317,-168,-2738,-1325,-168,-2738,-1437,-168,-2718,-1437,-168,-2778,-1349,-44,-2818,-1349,-44,-2818,-1381,-44,-2778,-1389,-44,-2806,-1377,-36,-2770,-1341,-64,-2754,-1357,-64,-2754,-1333,-64,-2650,-1413,-64,-2682,-1389,-64,-2682,-1421,-64,-2538,-1421,-64,-2538,-1397,-64,-2578,-1413,-64,-2578,-1413,-64,-2610,-1389,-64,-`
+`2650,-1413,-64,-2578,-1413,-64,-2650,-1413,-64,-2682,-1421,-64,-2538,-1421,-64,-2666,-1413,-144,-2538,-1413,-144,-2538,-1341,-144,-2666,-1341,-144,-2658,-1349,-44,-2666,-1381,-44,-2626,-1389,-44,-2618,-1357,-44,-2654,-1377,-36,-2618,-1341,-64,-2602,-1349,-64,-2594,-1333,-64,-2586,-1349,-44,-2586,-1369,-36,-2586,-1389,-44,-2554,-1389,-44,-2546,-1349,-44],"vertslength":128,"tris":[0,1,2,6,3,4,4,5,6,7,8,9,15,10,11,15,11,12,12,13,14,12,14,15,19,20,21,19,21,22,18,19,22,16,17,18,16,18,22,23,24,25,23,2`
+`5,26,27,28,29,27,29,30,35,36,37,35,37,38,34,35,38,34,38,39,33,41,43,41,42,43,33,32,43,42,31,43,32,31,43,33,34,44,41,33,44,41,40,44,34,39,44,40,39,44,45,46,47,48,49,50,48,50,51,52,53,54,55,56,57,61,58,59,59,60,61,62,63,64,62,64,65,66,67,68,66,68,69,81,82,70,80,71,72,79,80,72,78,79,72,78,72,73,77,78,73,77,73,74,74,75,76,74,76,77,70,71,83,71,80,83,80,81,83,70,81,83,89,84,85,88,89,85,85,86,87,85,87,88,92,93,94,93,90,94,90,91,94,92,91,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,107,109,`
+`110,114,111,112,112,113,114,115,116,119,116,117,119,117,118,119,115,118,119,120,121,122,124,125,126,127,123,124,124,126,127],"trislength":79,"triTopoly":[0,1,1,2,3,3,3,3,4,4,4,4,4,5,5,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,8,9,9,10,11,12,12,13,13,14,14,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,17,17,17,17,18,19,20,21,22,22,23,23,24,24,24,24,25,26,26,26],"baseVert":[0,3,7,10,16,23,27,31,45,48,52,55,58,62,66,70,84,90,95,98,101,104,107,111,115,120,123],"vertsCount":[3,4,3,6,7,4,4,14,3,4,3,3,4,4,4`
+`,14,6,5,3,3,3,3,4,4,5,3,5],"baseTri":[0,1,3,4,8,13,15,17,31,32,34,35,36,38,40,42,55,59,63,64,65,66,67,69,71,75,76],"triCount":[1,2,1,4,5,2,2,14,1,2,1,1,2,2,2,13,4,4,1,1,1,1,2,2,4,1,3]},"links":{"poly":[1,5,3,14,8,17,8,18,17,18,19,24,20,26,24,25,24,26,25,26],"cost":[587.2941284179688,888,666.4615478515625,864,792,984,868.137939453125,907.2000122070312,1536,907.2000122070312],"type":[2,1,2,1,2,2,2,2,1,2],"pos":[-2996.823486328125,-1281.7059326171875,-152,-2994,-1293,-168,-2538,-1269,-164,-2538,-12`
+`93,-168,-2774.3076171875,-1394.5384521484375,-64,-2778,-1389,-44,-2754,-1381,-64,-2754,-1357,-64,-2778,-1349,-44,-2770,-1341,-64,-2675.60009765625,-1393.800048828125,-64,-2666,-1381,-44,-2549.034423828125,-1401.413818359375,-64,-2554,-1389,-44,-2618,-1357,-44,-2611.60009765625,-1344.199951171875,-64,-2618,-1357,-44,-2586,-1357,-44,-2598.800048828125,-1342.5999755859375,-64,-2586,-1349,-44],"length":10}}],["1_2",{"tileId":"1_2","tx":1,"ty":2,"mesh":{"verts":[-2474,-1429,-159,-2538,-1437,-157,-253`
+`8,-1453,-161,-2314,-1453,-168,-2538,-1293,-168,-2538,-1317,-168,-2474,-1325,-159,-2314,-1293,-168,-2474,-1325,-159,-2474,-1429,-159,-2314,-1453,-168,-2314,-1293,-168,-2538,-1333,-64,-2538,-1421,-64,-2490,-1421,-64,-2490,-1333,-64,-2538,-1341,-144,-2538,-1413,-144,-2498,-1413,-144,-2498,-1341,-144,-2538,-1269,-416,-2338,-1261,-408,-2354,-1245,-408,-2354,-1245,-408,-2354,-1189,-408,-2538,-1189,-416,-2538,-1269,-416,-2538,-1189,-164,-2538,-1269,-164,-2314,-1269,-164,-2314,-1189,-164,-2290,-1229,104`
+`,-2290,-1189,104,-2298,-1189,104,-2298,-1453,104,-2162,-1445,98,-2178,-1453,88,-2090,-1493,88,-2162,-1445,98,-2090,-1493,88,-2026,-1493,88,-2026,-1189,88,-2170,-1237,104,-2298,-1453,104,-2282,-1445,104,-2282,-1245,104,-2290,-1229,104,-2274,-1229,101,-2290,-1229,104,-2282,-1245,104,-2170,-1237,104,-2274,-1189,88,-2274,-1229,101,-2170,-1237,104,-2026,-1189,88,-2106,-1501,88,-2090,-1493,88,-2178,-1453,88,-2026,-1701,88,-2026,-1597,88,-2106,-1589,88,-2274,-1701,88,-2106,-1589,88,-2106,-1501,88,-2178`
+`,-1453,88,-2274,-1453,88,-2274,-1701,88,-2258,-1261,112,-2258,-1429,112,-2186,-1429,112,-2186,-1261,112,-2082,-1565,88,-2026,-1565,88,-2026,-1525,88,-2082,-1525,88],"vertslength":75,"polys":[0,3,4,7,8,11,12,15,16,19,20,22,23,26,27,30,31,34,35,37,38,42,43,46,47,50,51,54,55,57,58,61,62,66,67,70,71,74],"polyslength":19,"regions":[2,2,2,7,8,5,5,4,3,3,3,3,3,3,1,1,1,6,9],"neighbors":[[[0],[0],[0],[1,2]],[[0],[0],[1,2],[0]],[[0],[1,0],[0],[1,1]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,6]],[[0],`
+`[0],[0],[1,5]],[[0],[0],[0],[0]],[[0],[0],[0],[1,11]],[[0],[1,14],[1,10]],[[1,9],[0],[0],[1,13],[0]],[[0],[0],[1,12],[1,8]],[[0],[1,11],[0],[1,13]],[[0],[1,12],[1,10],[0]],[[0],[1,9],[1,16]],[[0],[0],[1,16],[0]],[[0],[1,14],[0],[0],[1,15]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2474,-1429,-168,-2538,-1437,-157,-2538,-1453,-157,-2493.199951171875,-1453,-157,-2470.800048828125,-1453,-168,-2314,-1453,-168,-2502,-1441,-157,-2538,-1293,-168,-2538,-1317,-168,-2474,-1325,-168,-2314,-`
+`1293,-168,-2474,-1325,-168,-2474,-1429,-168,-2314,-1453,-168,-2314,-1293,-168,-2538,-1333,-64,-2538,-1421,-64,-2490,-1421,-64,-2490,-1333,-64,-2538,-1341,-144,-2538,-1413,-144,-2498,-1413,-144,-2498,-1341,-144,-2538,-1269,-416,-2493.5556640625,-1267.22216796875,-408,-2338,-1261,-408,-2354,-1245,-408,-2492,-1263,-408,-2354,-1245,-408,-2354,-1189,-408,-2492,-1189,-408,-2538,-1189,-416,-2538,-1269,-416,-2492,-1263,-408,-2538,-1189,-164,-2538,-1269,-164,-2314,-1269,-164,-2314,-1189,-164,-2290,-1229,`
+`88,-2290,-1209,88,-2290,-1189,104,-2298,-1189,104,-2298,-1453,104,-2290.800048828125,-1251.4000244140625,104,-2162,-1445,95,-2178,-1453,98,-2160.39990234375,-1461,88,-2090,-1493,88,-2144,-1457,88,-2162,-1445,95,-2144,-1457,88,-2090,-1493,88,-2026,-1493,88,-2026,-1189,88,-2170,-1237,88,-2169.111083984375,-1260.111083984375,99,-2134,-1313,88,-2298,-1453,104,-2282,-1445,104,-2282,-1245,101,-2290,-1229,88,-2290.800048828125,-1251.4000244140625,104,-2274,-1229,88,-2290,-1229,88,-2282,-1245,101,-2192.`
+`39990234375,-1238.5999755859375,101,-2170,-1237,88,-2274,-1189,88,-2274,-1229,88,-2170,-1237,88,-2026,-1189,88,-2106,-1501,88,-2090,-1493,88,-2178,-1453,88,-2026,-1701,88,-2026,-1597,88,-2106,-1589,88,-2274,-1701,88,-2106,-1589,88,-2106,-1501,88,-2178,-1453,88,-2274,-1453,88,-2274,-1701,88,-2258,-1261,112,-2258,-1429,112,-2186,-1429,112,-2186,-1261,112,-2082,-1565,88,-2026,-1565,88,-2026,-1525,88,-2082,-1525,88],"vertslength":91,"tris":[0,4,5,0,1,6,4,0,6,4,3,6,1,2,6,3,2,6,7,8,9,7,9,10,11,12,13,1`
+`1,13,14,18,15,16,16,17,18,22,19,20,20,21,22,27,23,24,26,27,24,24,25,26,31,32,33,30,31,33,28,29,30,28,30,33,37,34,35,35,36,37,39,40,41,38,39,41,43,38,41,41,42,43,44,45,46,48,44,46,46,47,48,52,53,56,53,54,56,55,54,56,55,49,56,49,50,56,52,51,56,50,51,56,59,60,61,58,59,61,57,58,61,62,63,64,62,64,65,62,65,66,67,68,69,67,69,70,71,72,73,74,75,76,74,76,77,78,79,80,78,80,81,78,81,82,86,83,84,84,85,86,90,87,88,88,89,90],"trislength":55,"triTopoly":[0,0,0,0,0,0,1,1,2,2,3,3,4,4,5,5,5,6,6,6,6,7,7,8,8,8,8,9,9`
+`,9,10,10,10,10,10,10,10,11,11,11,12,12,12,13,13,14,15,15,16,16,16,17,17,18,18],"baseVert":[0,7,11,15,19,23,28,34,38,44,49,57,62,67,71,74,78,83,87],"vertsCount":[7,4,4,4,4,5,6,4,6,5,8,5,5,4,3,4,5,4,4],"baseTri":[0,6,8,10,12,14,17,21,23,27,30,37,40,43,45,46,48,51,53],"triCount":[6,2,2,2,2,3,4,2,4,3,7,3,3,2,1,2,3,2,2]},"links":{"poly":[1,7],"cost":[888],"type":[1],"pos":[-2538,-1293,-168,-2538,-1269,-164],"length":1}}],["2_2",{"tileId":"2_2","tx":2,"ty":2,"mesh":{"verts":[-1914,-1509,88,-1930,-1509`
+`,88,-1930,-1581,88,-1914,-1701,88,-1946,-1597,88,-2026,-1597,88,-2026,-1701,88,-1930,-1581,88,-1946,-1597,88,-2026,-1701,88,-1914,-1701,88,-2026,-1525,88,-2026,-1565,88,-1962,-1565,88,-1962,-1525,88,-1946,-1493,88,-1930,-1509,88,-1914,-1509,88,-1914,-1189,88,-2026,-1189,88,-2026,-1493,88,-1946,-1493,88,-1914,-1189,88,-1898,-1189,104,-1898,-1701,104,-1890,-1701,104,-1890,-1189,104,-1730,-1389,-408,-1874,-1389,-408,-1858,-1413,-408,-1858,-1413,-408,-1874,-1701,-408,-1730,-1701,-408,-1730,-1389,-40`
+`8,-1722,-1373,-132,-1722,-1349,-132,-1738,-1341,-132,-1874,-1373,-132,-1738,-1341,-132,-1722,-1317,-132,-1714,-1189,-132,-1874,-1189,-132,-1874,-1373,-132,-1690,-1269,-408,-1714,-1245,-408,-1722,-1261,-408,-1698,-1293,-408,-1698,-1293,-408,-1722,-1261,-408,-1746,-1253,-408,-1746,-1317,-408,-1746,-1253,-408,-1730,-1221,-408,-1722,-1189,-416,-1874,-1189,-416,-1754,-1357,-408,-1746,-1317,-408,-1746,-1253,-408,-1874,-1189,-416,-1874,-1357,-408,-1722,-1189,-416,-1730,-1221,-408,-1714,-1245,-408,-1690`
+`,-1269,-408,-1650,-1261,-416,-1650,-1261,-416,-1594,-1293,-408,-1514,-1293,-408,-1514,-1189,-416,-1722,-1189,-416,-1722,-1349,-132,-1722,-1373,-132,-1714,-1389,-140,-1602,-1389,-140,-1594,-1349,-140,-1706,-1325,-140,-1714,-1189,-132,-1722,-1317,-132,-1706,-1325,-140,-1594,-1349,-140,-1514,-1349,-140,-1514,-1189,-132,-1514,-1701,-140,-1514,-1573,-140,-1602,-1565,-140,-1714,-1701,-140,-1602,-1565,-140,-1602,-1389,-140,-1714,-1389,-140,-1714,-1701,-140,-1674,-1277,-358,-1674,-1293,-358,-1650,-1293,`
+`-358,-1650,-1277,-358,-1578,-1389,-64,-1554,-1365,-64,-1586,-1365,-64,-1578,-1429,-64,-1578,-1389,-64,-1586,-1365,-64,-1586,-1557,-64,-1514,-1557,-64,-1514,-1445,-64,-1546,-1453,-64,-1514,-1557,-64,-1546,-1453,-64,-1578,-1429,-64,-1586,-1557,-64,-1570,-1541,-140,-1514,-1541,-140,-1514,-1373,-140,-1570,-1373,-140,-1522,-1389,-44,-1554,-1389,-44,-1562,-1421,-44,-1522,-1429,-44],"vertslength":116,"polys":[0,3,4,6,7,10,11,14,15,18,19,22,23,26,27,29,30,33,34,37,38,42,43,46,47,50,51,54,55,59,60,64,65,`
+`69,70,75,76,81,82,85,86,89,90,93,94,96,97,100,101,103,104,107,108,111,112,115],"polyslength":28,"regions":[8,8,8,12,7,7,14,4,4,2,2,5,5,5,5,9,9,1,1,3,3,15,10,10,10,10,11,13],"neighbors":[[[1,4],[0],[1,2],[0]],[[0],[0],[1,2]],[[0],[1,1],[0],[1,0]],[[0],[0],[0],[0]],[[0],[1,0],[0],[1,5]],[[0],[0],[1,4],[0]],[[0],[0],[0],[0]],[[0],[0],[1,8]],[[0],[0],[0],[1,7]],[[1,17],[0],[1,10],[0]],[[0],[1,18],[0],[0],[1,9]],[[1,15],[0],[1,12],[0]],[[1,11],[0],[1,14],[0]],[[0],[1,15],[0],[1,14]],[[0],[1,12],[1,13`
+`],[0],[0]],[[1,13],[0],[1,11],[0],[1,16]],[[0],[0],[0],[0],[1,15]],[[1,9],[0],[1,20],[0],[1,18],[0]],[[1,10],[0],[1,17],[0],[0],[0]],[[0],[0],[1,20],[0]],[[0],[1,17],[0],[1,19]],[[0],[0],[0],[0]],[[0],[0],[1,23]],[[0],[1,22],[0],[1,25]],[[0],[0],[1,25]],[[1,24],[0],[1,23],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-1914,-1509,88,-1930,-1509,88,-1930,-1581,88,-1914,-1701,88,-1946,-1597,88,-2026,-1597,88,-2026,-1701,88,-1930,-1581,88,-1946,-1597,88,-2026,-1701,88,-1914,-1701,88,`
+`-2026,-1525,88,-2026,-1565,88,-1962,-1565,88,-1962,-1525,88,-1946,-1493,88,-1930,-1509,88,-1914,-1509,88,-1914,-1189,88,-2026,-1189,88,-2026,-1493,88,-1946,-1493,88,-1914,-1189,88,-1898,-1189,104,-1898,-1701,104,-1890,-1701,104,-1890,-1189,104,-1730,-1389,-408,-1874,-1389,-408,-1858,-1413,-408,-1858,-1413,-408,-1874,-1701,-408,-1730,-1701,-408,-1730,-1389,-408,-1722,-1373,-132,-1722,-1349,-132,-1738,-1341,-132,-1874,-1373,-132,-1738,-1341,-132,-1722,-1317,-132,-1714,-1189,-132,-1874,-1189,-132,-`
+`1874,-1373,-132,-1690,-1269,-416,-1714,-1245,-416,-1722,-1261,-408,-1710,-1277,-416,-1698,-1293,-408,-1698,-1293,-408,-1710,-1277,-416,-1722,-1261,-408,-1746,-1253,-408,-1746,-1317,-408,-1746,-1253,-408,-1730,-1221,-416,-1722,-1189,-416,-1874,-1189,-416,-1754,-1357,-408,-1746,-1317,-408,-1746,-1253,-408,-1874,-1189,-416,-1874,-1231,-408,-1874,-1357,-408,-1722,-1189,-416,-1730,-1221,-416,-1722,-1233,-408,-1714,-1245,-416,-1690,-1269,-416,-1650,-1261,-416,-1650,-1261,-416,-1594,-1293,-408,-1514,-1`
+`293,-408,-1514,-1272.199951171875,-416,-1514,-1189,-416,-1722,-1189,-416,-1722,-1349,-140,-1722,-1373,-140,-1714,-1389,-140,-1602,-1389,-140,-1594,-1349,-140,-1706,-1325,-140,-1714,-1189,-132,-1722,-1317,-132,-1706,-1325,-140,-1594,-1349,-140,-1514,-1349,-140,-1514,-1303.2857666015625,-132,-1514,-1189,-132,-1614,-1313,-132,-1514,-1701,-140,-1514,-1573,-140,-1602,-1565,-140,-1714,-1701,-140,-1602,-1565,-140,-1602,-1389,-140,-1714,-1389,-140,-1714,-1701,-140,-1674,-1277,-358,-1674,-1293,-358,-1650`
+`,-1293,-358,-1650,-1277,-358,-1578,-1389,-64,-1554,-1365,-64,-1586,-1365,-64,-1578,-1429,-64,-1578,-1389,-64,-1586,-1365,-64,-1586,-1557,-64,-1514,-1557,-64,-1514,-1445,-64,-1546,-1453,-64,-1514,-1557,-64,-1546,-1453,-64,-1578,-1429,-64,-1586,-1557,-64,-1570,-1541,-140,-1514,-1541,-140,-1514,-1373,-140,-1570,-1373,-140,-1522,-1389,-44,-1554,-1389,-44,-1562,-1421,-44,-1522,-1429,-44,-1550,-1417,-36],"vertslength":124,"tris":[0,1,2,0,2,3,4,5,6,10,7,8,8,9,10,14,11,12,12,13,14,15,16,17,15,17,18,19,2`
+`0,21,19,21,22,26,23,24,24,25,26,27,28,29,30,31,32,30,32,33,34,35,36,34,36,37,38,39,40,38,40,41,38,41,42,46,47,43,44,45,46,43,44,46,49,50,51,48,49,51,48,51,52,53,54,55,53,55,56,57,58,59,59,60,61,62,57,59,59,61,62,63,64,65,65,66,67,65,67,68,63,65,68,70,71,72,69,70,72,69,72,73,69,73,74,75,76,77,80,75,77,78,79,80,77,78,80,84,85,86,87,81,88,81,82,88,84,83,88,82,83,88,84,86,88,87,86,88,89,90,91,89,91,92,93,94,95,93,95,96,100,97,98,98,99,100,101,102,103,104,105,106,104,106,107,108,109,110,112,113,114,1`
+`11,112,114,118,115,116,116,117,118,121,122,123,122,119,123,119,120,123,121,120,123],"trislength":70,"triTopoly":[0,0,1,2,2,3,3,4,4,5,5,6,6,7,8,8,9,9,10,10,10,11,11,11,12,12,12,13,13,14,14,14,14,15,15,15,15,16,16,16,16,17,17,17,17,18,18,18,18,18,18,18,19,19,20,20,21,21,22,23,23,24,25,25,26,26,27,27,27,27],"baseVert":[0,4,7,11,15,19,23,27,30,34,38,43,48,53,57,63,69,75,81,89,93,97,101,104,108,111,115,119],"vertsCount":[4,3,4,4,4,4,4,3,4,4,5,5,5,4,6,6,6,6,8,4,4,4,3,4,3,4,4,5],"baseTri":[0,2,3,5,7,9,`
+`11,13,14,16,18,21,24,27,29,33,37,41,45,52,54,56,58,59,61,62,64,66],"triCount":[2,1,2,2,2,2,2,1,2,2,3,3,3,2,4,4,4,4,7,2,2,2,1,2,1,2,2,4]},"links":{"poly":[0,6,5,6,11,21,22,27],"cost":[768,768,4220.39990234375,1032],"type":[2,2,2,2],"pos":[-1914,-1701,88,-1898,-1701,104,-1914,-1189,88,-1898,-1189,104,-1690.800048828125,-1271.4000244140625,-408,-1674,-1277,-358,-1566,-1377,-64,-1554,-1389,-44],"length":4}}],["3_2",{"tileId":"3_2","tx":3,"ty":2,"mesh":{"verts":[-1474,-1565,-140,-1514,-1573,-140,-151`
+`4,-1701,-140,-1178,-1701,-140,-1434,-1325,-140,-1474,-1365,-140,-1474,-1565,-140,-1178,-1701,-140,-1186,-1325,-140,-1490,-1365,-64,-1514,-1365,-64,-1514,-1381,-64,-1498,-1397,-64,-1514,-1437,-64,-1514,-1557,-64,-1490,-1557,-64,-1498,-1397,-64,-1490,-1557,-64,-1490,-1365,-64,-1498,-1397,-64,-1474,-1365,-140,-1434,-1325,-140,-1450,-1317,-140,-1514,-1349,-140,-1474,-1365,-140,-1450,-1317,-140,-1442,-1301,-132,-1514,-1189,-132,-1178,-1261,-132,-1002,-1261,-132,-1002,-1189,-132,-1442,-1301,-132,-1186`
+`,-1301,-132,-1178,-1261,-132,-1514,-1189,-132,-1442,-1301,-132,-1178,-1261,-132,-1002,-1189,-132,-1210,-1293,-408,-1210,-1277,-408,-1258,-1269,-408,-1354,-1277,-408,-1514,-1189,-416,-1514,-1293,-408,-1498,-1277,-408,-1458,-1293,-408,-1362,-1293,-408,-1354,-1277,-408,-1514,-1189,-416,-1498,-1277,-408,-1458,-1293,-408,-1354,-1277,-408,-1258,-1189,-416,-1354,-1277,-408,-1258,-1269,-408,-1258,-1189,-416,-1242,-1189,-367,-1242,-1261,-362,-1218,-1261,-362,-1210,-1189,-367,-1202,-1213,-320,-1202,-1253,`
+`-320,-1170,-1253,-320,-1170,-1213,-320,-1162,-1301,-132,-1002,-1301,-132,-1002,-1293,-132,-1162,-1285,-132,-1154,-1669,-40,-1002,-1669,-40,-1002,-1293,-40,-1154,-1293,-40,-1066,-1261,-272,-1066,-1189,-272,-1154,-1189,-272,-1154,-1261,-272,-1074,-1237,-366,-1082,-1189,-366,-1138,-1189,-366,-1138,-1245,-366,-1050,-1261,-304,-1002,-1261,-304,-1002,-1189,-304,-1050,-1189,-301,-1026,-1245,-398,-1010,-1245,-398,-1010,-1197,-398],"vertslength":87,"polys":[0,3,4,8,9,12,13,16,17,19,20,22,23,27,28,30,31,3`
+`3,34,37,38,41,42,44,45,47,48,52,53,55,56,59,60,63,64,67,68,71,72,75,76,79,80,83,84,86],"polyslength":23,"regions":[1,1,9,9,9,3,3,3,3,3,4,4,4,4,4,11,8,15,2,5,6,7,16],"neighbors":[[[0],[0],[0],[1,1]],[[1,5],[0],[1,0],[0],[0]],[[0],[0],[0],[1,4]],[[0],[0],[1,4],[0]],[[0],[1,2],[1,3]],[[1,1],[0],[1,6]],[[0],[1,5],[0],[1,9],[0]],[[0],[0],[1,9]],[[0],[0],[1,9]],[[1,6],[1,8],[1,7],[0]],[[0],[0],[1,14],[0]],[[0],[0],[1,13]],[[0],[0],[1,13]],[[1,11],[0],[1,12],[1,14],[0]],[[1,10],[0],[1,13]],[[0],[0],[0]`
+`,[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[-1474,-1565,-140,-1514,-1573,-140,-1514,-1701,-140,-1178,-1701,-140,-1434,-1325,-140,-1474,-1365,-140,-1474,-1565,-140,-1178,-1701,-140,-1186,-1325,-140,-1490,-1365,-64,-1514,-1365,-64,-1514,-1381,-64,-1498,-1397,-64,-1514,-1437,-64,-1514,-1557,-64,-1490,-1557,-64,-1498,-1397,-64,-1490,-1557,-64,-1490,-1365,-64,-1498,-1397,-64,-1474,-1365,-140,-1447`
+`.3333740234375,-1338.3333740234375,-140,-1434,-1325,-132,-1450,-1317,-132,-1458,-1333,-140,-1514,-1349,-140,-1474,-1365,-140,-1458,-1333,-140,-1450,-1317,-132,-1442,-1301,-132,-1514,-1189,-132,-1514,-1303.2857666015625,-132,-1178,-1261,-132,-1002,-1261,-132,-1002,-1189,-132,-1442,-1301,-132,-1186,-1301,-132,-1178,-1261,-132,-1514,-1189,-132,-1442,-1301,-132,-1178,-1261,-132,-1002,-1189,-132,-1210,-1293,-408,-1210,-1277,-408,-1242,-1271.6666259765625,-408,-1258,-1269,-416,-1354,-1277,-416,-1333.4`
+`285888671875,-1279.2857666015625,-408,-1514,-1189,-416,-1514,-1272.199951171875,-416,-1514,-1293,-408,-1498,-1277,-416,-1458,-1293,-408,-1362,-1293,-408,-1354,-1277,-416,-1374.800048828125,-1280.199951171875,-408,-1514,-1189,-416,-1498,-1277,-416,-1458,-1293,-408,-1374.800048828125,-1280.199951171875,-408,-1354,-1277,-416,-1258,-1189,-416,-1354,-1277,-416,-1258,-1269,-416,-1258,-1189,-416,-1242,-1189,-367,-1242,-1261,-367,-1218,-1261,-367,-1210,-1189,-367,-1202,-1213,-320,-1202,-1253,-320,-1170,`
+`-1253,-320,-1170,-1213,-320,-1162,-1301,-132,-1002,-1301,-132,-1002,-1293,-132,-1162,-1285,-132,-1154,-1669,-40,-1002,-1669,-40,-1002,-1293,-40,-1154,-1293,-40,-1066,-1261,-272,-1066,-1189,-272,-1154,-1189,-272,-1154,-1261,-272,-1074,-1237,-366,-1082,-1189,-366,-1138,-1189,-366,-1138,-1245,-366,-1050,-1261,-304,-1002,-1261,-304,-1002,-1189,-304,-1050,-1189,-304,-1026,-1245,-398,-1010,-1245,-398,-1010,-1197,-398],"vertslength":96,"tris":[0,1,2,0,2,3,4,5,6,8,4,6,6,7,8,9,10,11,9,11,12,13,14,15,13,1`
+`5,16,17,18,19,21,22,23,21,23,24,20,21,24,25,26,27,25,27,28,25,28,29,31,25,29,29,30,31,32,33,34,35,36,37,38,39,40,38,40,41,42,43,44,42,44,45,45,46,47,42,45,47,49,50,51,48,49,51,53,54,55,52,53,55,58,59,60,56,57,58,56,58,60,56,60,61,62,63,64,65,66,67,65,67,68,72,69,70,70,71,72,73,74,75,73,75,76,80,77,78,78,79,80,84,81,82,82,83,84,85,86,87,85,87,88,92,89,90,90,91,92,93,94,95],"trislength":50,"triTopoly":[0,0,1,1,1,2,2,3,3,4,5,5,5,6,6,6,6,6,7,8,9,9,10,10,10,10,11,11,12,12,13,13,13,13,14,15,15,16,16,1`
+`7,17,18,18,19,19,20,20,21,21,22],"baseVert":[0,4,9,13,17,20,25,32,35,38,42,48,52,56,62,65,69,73,77,81,85,89,93],"vertsCount":[4,5,4,4,3,5,7,3,3,4,6,4,4,6,3,4,4,4,4,4,4,4,3],"baseTri":[0,2,5,7,9,10,13,18,19,20,22,26,28,30,34,35,37,39,41,43,45,47,49],"triCount":[2,3,2,2,1,3,5,1,1,2,4,2,2,4,1,2,2,2,2,2,2,2,1]},"links":{"poly":[10,15,15,16,16,19,19,21],"cost":[3340.053955078125,3069.528564453125,3840,1645.5],"type":[2,2,2,2],"pos":[-1243.729736328125,-1271.37841796875,-408,-1242,-1261,-362,-1216.926`
+`8798828125,-1251.3414306640625,-362.6707458496094,-1202,-1253,-320,-1170,-1253,-320,-1154,-1253,-272,-1066,-1189,-272,-1050,-1189,-301],"length":4}}],["4_2",{"tileId":"4_2","tx":4,"ty":2,"mesh":{"verts":[-930,-1349,-40,-938,-1293,-40,-1002,-1293,-40,-1002,-1669,-40,-786,-1669,-40,-786,-1349,-40,-930,-1349,-40,-1002,-1669,-40,-1002,-1301,-132,-922,-1301,-132,-1002,-1293,-132,-1002,-1189,-304,-1002,-1261,-304,-930,-1261,-353,-930,-1189,-353,-1002,-1189,-132,-1002,-1261,-132,-930,-1261,-132,-930,-1`
+`189,-132,-986,-1685,-24,-978,-1701,-37,-970,-1701,-37,-962,-1685,-24,-914,-1293,-24,-914,-1325,-24,-778,-1325,-24,-778,-1285,-24,-730,-1309,-408,-754,-1325,-408,-698,-1341,-416,-730,-1285,-408,-730,-1309,-408,-698,-1341,-416,-890,-1189,-414,-890,-1261,-416,-762,-1261,-416,-498,-1189,-416,-498,-1189,-416,-762,-1261,-416,-730,-1285,-408,-490,-1277,-416,-498,-1189,-416,-730,-1285,-408,-698,-1341,-416,-490,-1653,-416,-762,-1701,-408,-650,-1701,-408,-642,-1685,-408,-730,-1357,-408,-754,-1357,-408,-64`
+`2,-1685,-408,-618,-1701,-416,-506,-1701,-416,-730,-1357,-408],"vertslength":54,"polys":[0,3,4,7,8,10,11,14,15,18,19,22,23,26,27,29,30,32,33,36,37,39,40,44,45,49,50,53],"polyslength":14,"regions":[2,2,7,4,5,9,6,1,1,1,1,1,3,3],"neighbors":[[[0],[0],[0],[1,1]],[[0],[0],[1,0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,8]],[[0],[1,7],[1,11]],[[0],[0],[1,10],[0]],[[1,9],[0],[1,11]],[[0],[1,10],[1,8],[0],[0]],[[0],[0],[1,13],[0],[0]],[[0],[0],`
+`[0],[1,12]]]},"detail":{"verts":[-930,-1349,-40,-938,-1293,-40,-1002,-1293,-40,-1002,-1669,-40,-786,-1669,-40,-786,-1349,-40,-930,-1349,-40,-1002,-1669,-40,-1002,-1301,-132,-922,-1301,-132,-1002,-1293,-132,-1002,-1189,-306,-1002,-1261,-306,-930,-1261,-353,-930,-1189,-353,-942,-1225,-353,-942,-1249,-353,-1002,-1189,-132,-1002,-1261,-132,-930,-1261,-132,-930,-1189,-132,-986,-1685,-24,-978,-1701,-24,-970,-1701,-24,-962,-1685,-24,-914,-1293,-24,-914,-1325,-24,-778,-1325,-24,-778,-1285,-24,-730,-1309`
+`,-416,-754,-1325,-408,-735.3333129882812,-1330.3333740234375,-416,-698,-1341,-416,-730,-1285,-416,-730,-1309,-416,-698,-1341,-416,-890,-1189,-414,-890,-1261,-416,-762,-1261,-416,-498,-1189,-416,-498,-1189,-416,-762,-1261,-416,-730,-1285,-416,-490,-1277,-416,-498,-1189,-416,-730,-1285,-416,-698,-1341,-416,-490,-1653,-416,-762,-1701,-408,-650,-1701,-408,-642,-1685,-416,-724.1333618164062,-1378.86669921875,-416,-730,-1357,-408,-754,-1357,-408,-654,-1689,-408,-642,-1685,-416,-630,-1693,-408,-618,-17`
+`01,-416,-506,-1701,-416,-717.5555419921875,-1376.111083984375,-416,-730,-1357,-408,-724.1333618164062,-1378.86669921875,-416],"vertslength":62,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,13,14,15,14,11,15,12,11,15,12,13,16,13,15,16,15,12,16,20,17,18,18,19,20,21,22,23,21,23,24,25,26,27,25,27,28,29,30,31,29,31,32,33,34,35,36,37,38,36,38,39,40,41,42,44,45,46,43,44,46,43,46,47,51,52,53,48,49,54,49,50,54,48,53,54,50,51,54,53,51,54,59,60,61,56,57,58,55,56,58,59,61,55,55,58,59],"trislength":37,"triTopoly":[`
+`0,0,1,1,2,3,3,3,3,3,3,4,4,5,5,6,6,7,7,8,9,9,10,11,11,11,12,12,12,12,12,12,13,13,13,13,13],"baseVert":[0,4,8,11,17,21,25,29,33,36,40,43,48,55],"vertsCount":[4,4,3,6,4,4,4,4,3,4,3,5,7,7],"baseTri":[0,2,4,5,11,13,15,17,19,20,22,23,26,32],"triCount":[2,2,1,6,2,2,2,2,1,2,1,3,6,5]},"links":{"poly":[0,5],"cost":[1152],"type":[2],"pos":[-1002,-1669,-40,-986,-1685,-24],"length":1}}],["5_2",{"tileId":"5_2","tx":5,"ty":2,"mesh":{"verts":[-290,-1397,-416,-298,-1341,-416,-346,-1341,-416,-346,-1461,-416,-98,-`
+`1509,-416,-26,-1461,-416,-26,-1397,-416,-290,-1397,-416,-474,-1469,-416,-490,-1381,-416,-490,-1661,-416,-474,-1469,-416,-490,-1661,-416,-466,-1701,-416,-146,-1701,-416,-138,-1581,-416,-346,-1461,-416,-138,-1581,-416,-98,-1509,-416,-290,-1397,-416,-346,-1461,-416,-378,-1189,-288,-482,-1189,-288,-466,-1453,-288,-354,-1445,-288,-394,-1197,-415,-466,-1205,-415,-466,-1285,-415,-450,-1437,-415,-370,-1421,-415,-74,-1277,-640,-82,-1261,-640,-122,-1261,-640,-130,-1277,-640,-426,-1277,-640,-426,-1317,-640`
+`,-394,-1317,-640,-130,-1277,-640,22,-1333,-640,22,-1277,-640,-74,-1277,-640,-130,-1277,-640,-394,-1317,-640,-394,-1341,-640,22,-1189,-768,-82,-1205,-768,-90,-1221,-768,22,-1245,-768,-402,-1197,-768,-82,-1205,-768,22,-1189,-768,-418,-1189,-768,22,-1245,-768,-90,-1221,-768,-410,-1213,-768,-418,-1245,-768,-410,-1213,-768,-402,-1197,-768,-418,-1189,-768,-410,-1213,-768,-418,-1189,-768,-418,-1245,-768,-130,-1301,-488,-130,-1277,-488,-186,-1277,-488,-194,-1293,-488,-170,-1325,-488,-138,-1309,-488,-106`
+`,-1325,-488,22,-1325,-488,22,-1293,-488,-114,-1293,-488,-130,-1301,-488,-138,-1309,-488,22,-1293,-488,-122,-1277,-488,-114,-1293,-488,-170,-1325,-488,-194,-1293,-488,-234,-1277,-488,-370,-1277,-488,-362,-1325,-488,-346,-1341,-416,-298,-1341,-416,-298,-1189,-416,-362,-1189,-416,-202,-1205,-256,-266,-1189,-248,-282,-1189,-248,-282,-1381,-248,-170,-1317,-256,-170,-1229,-256,-106,-1381,-248,-114,-1333,-256,-170,-1317,-256,-282,-1381,-248,-170,-1229,-256,-82,-1221,-256,-74,-1189,-256,-186,-1189,-256,`
+`-202,-1205,-256,-26,-1333,-416,-26,-1357,-416,22,-1341,-416,-258,-1341,-416,-242,-1357,-416,-194,-1357,-416,-186,-1325,-416,-50,-1317,-416,-26,-1333,-416,22,-1341,-416,22,-1189,-416,-258,-1341,-416,-186,-1325,-416,-138,-1309,-416,-258,-1189,-416,-138,-1309,-416,-50,-1317,-416,22,-1189,-416,-258,-1189,-416,-146,-1277,-190,-138,-1301,-190,-98,-1301,-190,-90,-1253,-190,-122,-1245,-190,-58,-1485,-47,-106,-1541,-47,-130,-1605,-46,-130,-1701,-46,22,-1701,-19,22,-1453,-46,22,-1477,-416,-34,-1485,-416,-`
+`90,-1541,-416,-122,-1621,-416,-114,-1701,-416,22,-1701,-416,-74,-1189,-256,-82,-1221,-256,-58,-1261,-256,22,-1189,-256,-66,-1317,-256,-114,-1333,-256,-106,-1381,-248,-18,-1381,-248,-2,-1341,-256,-66,-1317,-256,-2,-1341,-256,22,-1349,-256,22,-1189,-256,-58,-1261,-256,-98,-1349,-384,-42,-1357,-384,-42,-1341,-384,-98,-1325,-384,22,-1429,-105,22,-1405,-103,-2,-1413,-117],"vertslength":158,"polys":[0,3,4,7,8,10,11,16,17,20,21,24,25,29,30,33,34,37,38,43,44,47,48,51,52,55,56,58,59,61,62,67,68,73,74,76,`
+`77,81,82,85,86,91,92,95,96,100,101,103,104,107,108,111,112,115,116,119,120,124,125,130,131,136,137,140,141,145,146,150,151,154,155,157],"polyslength":36,"regions":[1,1,1,1,1,5,9,10,10,10,12,12,12,12,12,16,16,16,16,11,6,6,6,4,4,4,4,4,14,2,3,8,8,8,20,22],"neighbors":[[[0],[1,19],[0],[1,4]],[[0],[0],[0],[1,4]],[[0],[0],[1,3]],[[1,2],[0],[0],[0],[1,4],[0]],[[0],[1,1],[1,0],[1,3]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[1,9]],[[0],[0],[1,9],[0]],[[0],[0],[1,7],[1,8],[0],[0]],[[1,11],[0]`
+`,[1,12],[0]],[[0],[1,10],[0],[1,13]],[[1,10],[0],[1,14],[0]],[[0],[1,11],[1,14]],[[1,13],[0],[1,12]],[[0],[0],[0],[1,18],[0],[1,16]],[[0],[0],[1,17],[0],[1,15],[0]],[[0],[0],[1,16]],[[1,15],[0],[0],[0],[0]],[[1,0],[0],[0],[0]],[[0],[0],[0],[1,21],[0],[1,22]],[[1,32],[0],[1,20],[0]],[[0],[1,31],[0],[0],[1,20]],[[0],[0],[1,25]],[[0],[0],[0],[1,26]],[[0],[1,23],[0],[1,27]],[[1,24],[0],[1,27],[0]],[[0],[1,25],[0],[1,26]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[1,2`
+`2],[0],[1,33],[0]],[[0],[1,21],[0],[0],[1,33]],[[1,32],[0],[0],[1,31],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[-290,-1397,-416,-298,-1341,-416,-346,-1341,-416,-346,-1461,-416,-98,-1509,-416,-26,-1461,-416,-26,-1397,-416,-290,-1397,-416,-474,-1469,-416,-490,-1381,-416,-490,-1661,-416,-474,-1469,-416,-490,-1661,-416,-466,-1701,-416,-146,-1701,-416,-138,-1581,-416,-346,-1461,-416,-138,-1581,-416,-98,-1509,-416,-290,-1397,-416,-346,-1461,-416,-378,-1189,-288,-482,-1189,-288,-466,-14`
+`53,-288,-354,-1445,-288,-394,-1197,-415,-466,-1205,-415,-466,-1285,-415,-450,-1437,-415,-370,-1421,-415,-74,-1277,-640,-82,-1261,-640,-122,-1261,-640,-130,-1277,-640,-426,-1277,-640,-426,-1317,-640,-394,-1317,-640,-130,-1277,-640,22,-1333,-640,22,-1277,-640,-74,-1277,-640,-130,-1277,-640,-394,-1317,-640,-394,-1341,-640,22,-1189,-768,-82,-1205,-768,-90,-1221,-768,22,-1245,-768,-402,-1197,-768,-82,-1205,-768,22,-1189,-768,-418,-1189,-768,22,-1245,-768,-90,-1221,-768,-410,-1213,-768,-418,-1245,-768`
+`,-410,-1213,-768,-402,-1197,-768,-418,-1189,-768,-410,-1213,-768,-418,-1189,-768,-418,-1245,-768,-130,-1301,-488,-130,-1277,-488,-186,-1277,-488,-194,-1293,-488,-170,-1325,-488,-138,-1309,-488,-106,-1325,-488,22,-1325,-488,22,-1293,-488,-114,-1293,-488,-130,-1301,-488,-138,-1309,-488,22,-1293,-488,-122,-1277,-488,-114,-1293,-488,-170,-1325,-488,-194,-1293,-488,-234,-1277,-488,-370,-1277,-488,-362,-1325,-488,-346,-1341,-416,-298,-1341,-416,-298,-1189,-416,-362,-1189,-416,-202,-1205,-256,-244.6666`
+`717529297,-1194.3333740234375,-256,-266,-1189,-248,-282,-1189,-248,-282,-1381,-248,-170,-1317,-256,-170,-1229,-256,-246,-1345,-256,-106,-1381,-248,-114,-1333,-256,-170,-1317,-256,-282,-1381,-248,-170,-1229,-256,-82,-1221,-256,-74,-1189,-256,-186,-1189,-256,-202,-1205,-256,-26,-1333,-416,-26,-1357,-416,22,-1341,-416,-258,-1341,-416,-242,-1357,-416,-194,-1357,-416,-186,-1325,-416,-50,-1317,-416,-26,-1333,-416,22,-1341,-416,22,-1189,-416,-258,-1341,-416,-186,-1325,-416,-138,-1309,-416,-258,-1189,-4`
+`16,-138,-1309,-416,-50,-1317,-416,22,-1189,-416,-258,-1189,-416,-146,-1277,-190,-138,-1301,-190,-98,-1301,-190,-90,-1253,-190,-122,-1245,-190,-58,-1485,-47,-106,-1541,-46,-130,-1605,-45,-130,-1701,-44,-21.428571701049805,-1701,-19,22,-1701,-19,22,-1588.272705078125,-20,22,-1453,-46,22,-1477,-416,-34,-1485,-416,-90,-1541,-416,-122,-1621,-416,-114,-1701,-416,22,-1701,-416,-74,-1189,-256,-82,-1221,-256,-58,-1261,-256,22,-1189,-256,-66,-1317,-256,-114,-1333,-256,-106,-1381,-248,-18,-1381,-248,-2,-13`
+`41,-256,-66,-1317,-256,-2,-1341,-256,22,-1349,-256,22,-1189,-256,-58,-1261,-256,-98,-1349,-384,-42,-1357,-384,-42,-1341,-384,-98,-1325,-384,22,-1429,-101,22,-1405,-103,-2,-1413,-111],"vertslength":162,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,11,12,13,16,11,13,14,15,16,13,14,16,17,18,19,17,19,20,21,22,23,21,23,24,25,26,27,27,28,29,25,27,29,30,31,32,30,32,33,34,35,36,34,36,37,38,39,40,38,40,41,41,42,43,38,41,43,44,45,46,44,46,47,51,48,49,49,50,51,53,54,55,52,53,55,56,57,58,59,60,61,67,62,63,64,65,66`
+`,66,67,63,63,64,66,71,72,73,71,73,68,68,69,70,68,70,71,74,75,76,77,78,79,79,80,81,77,79,81,82,83,84,82,84,85,87,88,89,89,90,93,90,91,93,91,92,93,92,86,93,89,87,93,86,87,93,94,95,96,94,96,97,101,102,98,98,99,100,98,100,101,103,104,105,106,107,108,106,108,109,110,111,112,110,112,113,114,115,116,114,116,117,118,119,120,118,120,121,122,123,124,125,126,122,122,124,125,131,132,133,127,128,129,129,130,131,133,134,127,129,131,133,127,129,133,135,136,137,137,138,139,137,139,140,135,137,140,141,142,143,14`
+`1,143,144,145,146,147,148,149,145,145,147,148,150,151,152,154,150,152,152,153,154,155,156,157,155,157,158,159,160,161],"trislength":91,"triTopoly":[0,0,1,1,2,3,3,3,3,4,4,5,5,6,6,6,7,7,8,8,9,9,9,9,10,10,11,11,12,12,13,14,15,15,15,15,16,16,16,16,17,18,18,18,19,19,20,20,20,20,20,20,20,21,21,22,22,22,23,24,24,25,25,26,26,27,27,28,28,28,29,29,29,29,29,29,30,30,30,30,31,31,32,32,32,33,33,33,34,34,35],"baseVert":[0,4,8,11,17,21,25,30,34,38,44,48,52,56,59,62,68,74,77,82,86,94,98,103,106,110,114,118,122,`
+`127,135,141,145,150,155,159],"vertsCount":[4,4,3,6,4,4,5,4,4,6,4,4,4,3,3,6,6,3,5,4,8,4,5,3,4,4,4,4,5,8,6,4,5,5,4,3],"baseTri":[0,2,4,5,9,11,13,16,18,20,24,26,28,30,31,32,36,40,41,44,46,53,55,58,59,61,63,65,67,70,76,80,82,85,88,90],"triCount":[2,2,1,4,2,2,3,2,2,4,2,2,2,1,1,4,4,1,3,2,7,2,3,1,2,2,2,2,3,6,4,2,3,3,2,1]},"links":{"poly":[23,34,29,35],"cost":[1920,6085.5],"type":[2,2],"pos":[-26,-1357,-416,-42,-1357,-384,22,-1453,-46,22,-1429,-105],"length":2}}],["6_2",{"tileId":"6_2","tx":6,"ty":2,"me`
+`sh":{"verts":[246,-1573,-416,246,-1541,-416,174,-1533,-416,278,-1701,-416,278,-1589,-416,246,-1573,-416,174,-1533,-416,174,-1477,-416,22,-1477,-416,278,-1701,-416,246,-1573,-416,174,-1533,-416,22,-1477,-416,22,-1701,-416,246,-1437,-55,190,-1437,-55,182,-1461,-47,246,-1541,-42,246,-1541,-42,182,-1461,-47,134,-1445,-46,22,-1453,-45,22,-1701,-19,294,-1701,-46,294,-1701,-46,278,-1549,-47,246,-1541,-42,166,-1621,-608,190,-1621,-608,190,-1605,-608,158,-1597,-608,86,-1573,-608,86,-1549,-608,62,-1533,-6`
+`08,54,-1573,-608,166,-1685,-608,182,-1669,-608,166,-1621,-608,166,-1685,-608,166,-1621,-608,158,-1597,-608,86,-1573,-608,54,-1573,-608,22,-1685,-608,158,-1597,-608,174,-1573,-608,86,-1573,-608,54,-1573,-608,22,-1549,-608,22,-1685,-608,254,-1549,-768,246,-1517,-767,150,-1517,-767,142,-1549,-768,22,-1565,-768,22,-1661,-768,94,-1669,-768,142,-1549,-768,326,-1669,-759,318,-1565,-764,254,-1549,-768,142,-1549,-768,94,-1669,-768,102,-1685,-768,246,-1421,-90,246,-1405,-90,22,-1405,-99,22,-1429,-101,118,`
+`-1429,-247,174,-1445,-248,182,-1421,-246,38,-1389,-248,30,-1437,-248,118,-1429,-247,182,-1421,-246,110,-1229,-256,142,-1269,-256,246,-1189,-256,22,-1381,-248,38,-1389,-248,182,-1421,-246,246,-1421,-248,110,-1325,-256,62,-1325,-256,78,-1221,-256,110,-1229,-256,246,-1189,-256,22,-1189,-256,246,-1189,-256,142,-1269,-256,110,-1325,-256,246,-1421,-248,30,-1277,-256,78,-1221,-256,22,-1189,-256,30,-1277,-256,22,-1189,-256,22,-1381,-248,62,-1325,-256,30,-1277,-256,22,-1381,-248,230,-1357,-416,246,-1341,`
+`-416,246,-1189,-416,22,-1189,-416,22,-1341,-416,46,-1357,-416,302,-1317,-640,302,-1277,-640,22,-1277,-640,22,-1325,-640,22,-1325,-488,166,-1317,-488,22,-1293,-488,22,-1189,-768,22,-1245,-768,246,-1245,-768,246,-1189,-768,54,-1397,-416,30,-1413,-416,30,-1437,-416,118,-1429,-416,174,-1397,-416,118,-1429,-416,174,-1445,-416,174,-1397,-416,78,-1245,-160,54,-1277,-160,62,-1301,-160,110,-1301,-160,118,-1269,-160,534,-1365,-642,534,-1341,-642,502,-1357,-642,486,-1341,-642,438,-1341,-642,430,-1357,-642,`
+`502,-1357,-642,534,-1365,-642,502,-1357,-642,430,-1357,-642,310,-1357,-641,134,-1357,-641,86,-1357,-641,86,-1509,-641,142,-1341,-640,134,-1357,-641,86,-1509,-641,302,-1509,-641,310,-1357,-641,310,-1357,-641,430,-1357,-642,414,-1341,-642,142,-1341,-640,118,-1373,-776,94,-1381,-776,118,-1405,-776,118,-1469,-776,94,-1493,-776,126,-1493,-770,118,-1469,-776,126,-1493,-770,150,-1517,-767,246,-1517,-767,294,-1493,-776,262,-1477,-776,246,-1517,-767,262,-1477,-776,262,-1405,-776,118,-1405,-776,118,-1469,`
+`-776,150,-1517,-767,246,-1517,-767,262,-1405,-776,118,-1373,-776,262,-1405,-776,294,-1397,-776,294,-1285,-769,262,-1277,-776,118,-1309,-776,118,-1373,-776,262,-1277,-776,94,-1285,-776,118,-1309,-776,198,-1517,-416,222,-1509,-416,198,-1493,-416,246,-1445,-416,230,-1445,-416,222,-1477,-416,246,-1493,-416,222,-1397,-416,222,-1421,-416,246,-1421,-416,246,-1397,-416,254,-1605,-608,230,-1597,-608,230,-1613,-608,246,-1629,-608,230,-1453,-144,230,-1477,-146,246,-1477,-140,246,-1453,-139,238,-1661,-608,2`
+`54,-1645,-608,238,-1637,-608,302,-1189,-625,262,-1189,-625,262,-1229,-625,302,-1245,-625,262,-1229,-625,238,-1253,-628,302,-1245,-625,254,-1685,-608,294,-1677,-608,262,-1669,-608,270,-1621,-608,254,-1645,-608,278,-1653,-608,294,-1629,-608,262,-1541,-608,310,-1549,-608,318,-1533,-608,534,-1517,74,534,-1461,77,414,-1453,75,278,-1517,74,398,-1253,74,390,-1189,74,278,-1189,74,422,-1341,75,534,-1341,75,534,-1253,74,398,-1253,74,414,-1453,75,422,-1341,75,398,-1253,74,278,-1189,74,278,-1517,74,278,-141`
+`3,-743,278,-1469,-736,294,-1469,-735,294,-1413,-743,350,-1445,-416,350,-1469,-416,406,-1469,-416,406,-1397,-416,502,-1397,-416,502,-1453,-416,534,-1453,-416,534,-1205,-416,318,-1365,-416,326,-1437,-416,350,-1445,-416,406,-1397,-416,422,-1381,-416,326,-1221,-416,278,-1221,-416,278,-1341,-416,334,-1205,-416,326,-1221,-416,278,-1341,-416,278,-1341,-416,318,-1365,-416,422,-1381,-416,486,-1381,-416,534,-1205,-416,334,-1205,-416,486,-1381,-416,502,-1397,-416,534,-1205,-416,286,-1189,-128,286,-1245,-12`
+`8,302,-1253,-128,318,-1189,-128,286,-1317,-128,302,-1501,-128,318,-1501,-128,318,-1189,-128,302,-1253,-128,302,-1237,-760,302,-1189,-760,286,-1189,-760,286,-1237,-760,294,-1581,-608,294,-1605,-608,310,-1613,-608,334,-1565,-593,310,-1613,-608,302,-1669,-608,318,-1677,-608,422,-1581,-540,390,-1573,-556,334,-1565,-593,390,-1541,-561,334,-1541,-593,334,-1565,-593,390,-1573,-556,294,-1549,-368,294,-1565,-368,318,-1565,-368,318,-1549,-368,374,-1581,-416,358,-1549,-416,302,-1581,-416,374,-1581,-416,302`
+`,-1581,-416,310,-1701,-416,470,-1701,-416,470,-1581,-416,342,-1541,-748,318,-1565,-764,326,-1669,-759,534,-1669,-640,534,-1557,-640,494,-1541,-652,534,-1357,-768,534,-1189,-770,342,-1189,-768,334,-1357,-768,342,-1685,-588,462,-1685,-513,462,-1589,-513,534,-1493,-177,534,-1477,-177,342,-1477,-176,342,-1501,-177,534,-1493,-128,534,-1469,-128,342,-1469,-128,342,-1501,-128,382,-1253,-633,382,-1189,-636,342,-1189,-636,342,-1317,-640,406,-1277,-640,382,-1253,-633,342,-1317,-640,534,-1317,-640,534,-127`
+`7,-640,406,-1277,-640,342,-1317,-640,342,-1317,-488,534,-1317,-488,534,-1277,-488,342,-1277,-488,374,-1549,-368,374,-1565,-368,470,-1565,-368,470,-1549,-368,414,-1549,-540,446,-1573,-524,462,-1541,-513,454,-1581,-768,438,-1541,-768,422,-1541,-768,422,-1677,-768,486,-1541,-768,462,-1541,-768,454,-1581,-768,486,-1677,-768,486,-1541,-768,454,-1581,-768,422,-1677,-768,438,-1221,74,438,-1189,74,422,-1189,74,422,-1221,74,438,-1453,-608,534,-1453,-608,534,-1413,-608,438,-1413,-608,438,-1437,92,534,-143`
+`7,92,534,-1365,92,438,-1365,92,462,-1237,149,534,-1237,132,534,-1189,131,454,-1189,149,486,-1213,74,486,-1189,74,462,-1189,74,462,-1221,74,486,-1701,-224,526,-1701,-235,534,-1637,-239,534,-1549,-239,486,-1549,-224,494,-1453,-526,534,-1453,-526,534,-1413,-526,494,-1413,-526,534,-1549,-416,518,-1549,-416,510,-1597,-416,510,-1701,-416,534,-1701,-416],"vertslength":398,"polys":[0,2,3,5,6,8,9,13,14,17,18,23,24,26,27,30,31,34,35,37,38,43,44,46,47,49,50,53,54,57,58,63,64,67,68,70,71,74,75,77,78,83,84,8`
+`7,88,91,92,94,95,97,98,100,101,106,107,110,111,113,114,117,118,122,123,125,126,130,131,133,134,137,138,141,142,144,145,149,150,153,154,156,157,159,160,162,163,165,166,168,169,174,175,180,181,183,184,186,187,190,191,194,195,198,199,202,203,205,206,209,210,212,213,215,216,219,220,222,223,226,227,229,230,233,234,238,239,242,243,246,247,250,251,255,256,258,259,261,262,267,268,270,271,274,275,279,280,283,284,287,288,293,294,297,298,301,302,304,305,309,310,315,316,319,320,322,323,326,327,330,331,334,3`
+`35,337,338,341,342,345,346,349,350,352,353,356,357,359,360,363,364,367,368,371,372,375,376,379,380,383,384,388,389,392,393,397],"polyslength":101,"regions":[2,2,2,2,1,1,1,13,13,13,13,13,13,6,6,6,33,10,10,10,10,10,10,10,10,10,5,24,34,19,25,25,21,4,4,4,4,4,4,7,7,7,7,7,7,7,7,39,45,46,48,49,50,26,26,53,30,54,9,9,9,9,62,3,3,3,3,3,3,3,31,31,64,14,14,14,65,12,12,11,8,17,67,32,16,16,16,27,68,69,18,18,18,71,28,15,22,74,23,29,75],"neighbors":[[[0],[0],[1,3]],[[0],[0],[1,3]],[[0],[0],[1,3]],[[1,1],[1,0],[1`
+`,2],[0],[0]],[[0],[0],[1,5],[0]],[[1,4],[0],[0],[0],[0],[1,6]],[[0],[0],[1,5]],[[0],[0],[0],[1,10]],[[0],[0],[0],[1,10]],[[0],[0],[1,10]],[[1,9],[1,7],[1,11],[1,8],[1,12],[0]],[[0],[0],[1,10]],[[0],[0],[1,10]],[[0],[1,44],[0],[1,15]],[[0],[0],[1,15],[0]],[[1,79],[0],[1,13],[1,14],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,18]],[[0],[0],[1,17],[1,20]],[[0],[1,22],[1,21]],[[0],[1,18],[0],[1,22],[0],[1,25]],[[0],[1,19],[0],[1,23]],[[1,19],[0],[1,20],[0]],[[0],[1,21],[1,24]],[[1,23],[0],[1,25]],[[0],[1,`
+`24],[1,20]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,31],[0]],[[0],[0],[1,30]],[[0],[0],[0],[0],[0]],[[0],[0],[1,35]],[[0],[0],[1,35],[0]],[[1,33],[1,34],[1,38],[0]],[[0],[0],[1,37]],[[0],[1,36],[0],[0],[1,38]],[[1,35],[0],[0],[1,37]],[[0],[0],[1,44]],[[0],[0],[1,41]],[[1,40],[0],[1,44]],[[0],[0],[1,43]],[[1,42],[0],[1,44]],[[0],[1,41],[1,13],[1,43],[1,45],[1,39]],[[0],[0],[0],[1,46],[0],[1,44]],[[0],[0],[1,45]],[[0],[0],[0]],[[0],[0],[0],[0]],[`
+`[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,54],[0]],[[0],[0],[1,53]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,61],[0]],[[0],[0],[1,61]],[[0],[0],[0],[1,61]],[[0],[1,60],[1,59],[0],[1,58]],[[0],[0],[0],[0]],[[0],[0],[0],[1,65]],[[0],[0],[0],[1,69]],[[0],[0],[1,63],[0],[1,68]],[[0],[0],[1,67]],[[0],[1,66],[1,68]],[[0],[1,65],[0],[1,69],[0],[1,67]],[[0],[1,64],[1,68]],[[0],[0],[1,71],[0]],[[0],[0],[0],[1,70],[0]],[[0],[0],[0],[0]],[[0],[0],[1,74]`
+`,[0]],[[0],[0],[0],[0],[1,75],[1,73]],[[0],[0],[1,74],[0]],[[0],[0],[0],[0]],[[0],[0],[1,78]],[[1,77],[0],[0],[0],[0]],[[0],[1,15],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,85]],[[0],[1,84],[1,86]],[[0],[0],[1,85],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,92]],[[0],[0],[1,92]],[[0],[1,91],[1,90],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]`
+`],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]]]},"detail":{"verts":[246,-1573,-416,246,-1541,-416,174,-1533,-416,278,-1701,-416,278,-1589,-416,246,-1573,-416,174,-1533,-416,174,-1477,-416,22,-1477,-416,278,-1701,-416,246,-1573,-416,174,-1533,-416,22,-1477,-416,22,-1701,-416,246,-1437,-55,190,-1437,-55,182,-1461,-55,246,-1541,-44,246,-1541,-44,182,-1461,-55,134,-1445,-46,22,-1453,-46,22,-1565.727294921875,-21,22,-1701,-19,180.6666717529297,-1701,-21,294,-1701,-46,106,-1569,-19,178,-1617,-22,294,-1701,`
+`-46,278,-1549,-47,246,-1541,-44,166,-1621,-608,190,-1621,-608,190,-1605,-608,158,-1597,-608,86,-1573,-608,86,-1549,-608,62,-1533,-608,54,-1573,-608,166,-1685,-608,182,-1669,-608,166,-1621,-608,166,-1685,-608,166,-1621,-608,158,-1597,-608,86,-1573,-608,54,-1573,-608,22,-1685,-608,158,-1597,-608,174,-1573,-608,86,-1573,-608,54,-1573,-608,22,-1549,-608,22,-1685,-608,254,-1549,-768,246,-1517,-767,150,-1517,-767,142,-1549,-768,22,-1565,-768,22,-1661,-768,94,-1669,-768,142,-1549,-768,326,-1669,-764,31`
+`8,-1565,-768,254,-1549,-768,142,-1549,-768,94,-1669,-768,102,-1685,-768,246,-1421,-90,246,-1405,-90,66.80000305175781,-1405,-90,22,-1405,-97,22,-1429,-95,118,-1429,-246,174,-1445,-246,182,-1421,-246,38,-1389,-248,30,-1437,-247,118,-1429,-246,182,-1421,-246,110,-1229,-256,142,-1269,-256,246,-1189,-256,22,-1381,-248,38,-1389,-248,182,-1421,-246,246,-1421,-248,110,-1325,-256,62,-1325,-256,78,-1221,-256,110,-1229,-256,246,-1189,-256,22,-1189,-256,246,-1189,-256,142,-1269,-256,110,-1325,-256,246,-142`
+`1,-248,246,-1374.5999755859375,-248,246,-1351.4000244140625,-256,30,-1277,-256,78,-1221,-256,22,-1189,-256,30,-1277,-256,22,-1189,-256,22,-1338.3333740234375,-256,22,-1381,-248,23.600000381469727,-1360.199951171875,-255,62,-1325,-256,30,-1277,-256,23.600000381469727,-1360.199951171875,-255,22,-1381,-248,230,-1357,-416,246,-1341,-416,246,-1189,-416,22,-1189,-416,22,-1341,-416,46,-1357,-416,302,-1317,-640,302,-1277,-640,22,-1277,-640,22,-1325,-640,22,-1325,-488,166,-1317,-488,22,-1293,-488,22,-118`
+`9,-768,22,-1245,-768,246,-1245,-768,246,-1189,-768,54,-1397,-416,30,-1413,-416,30,-1437,-416,118,-1429,-416,174,-1397,-416,118,-1429,-416,174,-1445,-416,174,-1397,-416,78,-1245,-160,54,-1277,-160,62,-1301,-160,110,-1301,-160,118,-1269,-160,534,-1365,-642,534,-1341,-642,502,-1357,-642,486,-1341,-642,438,-1341,-642,430,-1357,-642,502,-1357,-642,534,-1365,-642,502,-1357,-642,430,-1357,-642,310,-1357,-646,134,-1357,-640,86,-1357,-641,86,-1509,-641,142,-1341,-640,134,-1357,-640,86,-1509,-641,302,-150`
+`9,-641,310,-1357,-646,310,-1357,-646,430,-1357,-642,414,-1341,-642,142,-1341,-640,118,-1373,-776,94,-1381,-776,118,-1405,-776,118,-1469,-776,94,-1493,-776,126,-1493,-776,118,-1469,-776,126,-1493,-776,150,-1517,-768,246,-1517,-770,294,-1493,-776,262,-1477,-776,246,-1517,-770,262,-1477,-776,262,-1405,-776,118,-1405,-776,118,-1469,-776,150,-1517,-768,246,-1517,-770,262,-1405,-776,118,-1373,-776,154,-1481,-776,262,-1405,-776,294,-1397,-776,294,-1285,-769,262,-1277,-776,118,-1309,-776,118,-1373,-776,`
+`262,-1277,-776,94,-1285,-776,118,-1309,-776,198,-1517,-416,222,-1509,-416,198,-1493,-416,246,-1445,-416,230,-1445,-416,222,-1477,-416,246,-1493,-416,222,-1397,-416,222,-1421,-416,246,-1421,-416,246,-1397,-416,254,-1605,-608,230,-1597,-608,230,-1613,-608,246,-1629,-608,230,-1453,-139,230,-1477,-137,246,-1477,-137,246,-1453,-139,238,-1661,-608,254,-1645,-608,238,-1637,-608,302,-1189,-625,262,-1189,-625,262,-1229,-625,302,-1245,-625,262,-1229,-625,238,-1253,-628,302,-1245,-625,254,-1685,-608,294,-1`
+`677,-608,262,-1669,-608,270,-1621,-608,254,-1645,-608,278,-1653,-608,294,-1629,-608,262,-1541,-608,310,-1549,-608,318,-1533,-608,534,-1517,74,534,-1461,77,414,-1453,74,278,-1517,74,398,-1253,74,390,-1189,74,278,-1189,74,422,-1341,74,534,-1341,74,534,-1253,74,398,-1253,74,414,-1453,74,422,-1341,74,398,-1253,74,278,-1189,74,278,-1517,74,278,-1413,-743,278,-1450.3333740234375,-743,278,-1469,-734,294,-1469,-734,294,-1450.3333740234375,-743,294,-1413,-743,350,-1445,-416,350,-1469,-416,406,-1469,-416,`
+`406,-1397,-416,502,-1397,-416,502,-1453,-416,534,-1453,-416,534,-1205,-416,318,-1365,-416,326,-1437,-416,350,-1445,-416,406,-1397,-416,422,-1381,-416,326,-1221,-416,278,-1221,-416,278,-1341,-416,334,-1205,-416,326,-1221,-416,278,-1341,-416,278,-1341,-416,318,-1365,-416,422,-1381,-416,486,-1381,-416,534,-1205,-416,334,-1205,-416,486,-1381,-416,502,-1397,-416,534,-1205,-416,286,-1189,-128,286,-1245,-128,302,-1253,-128,318,-1189,-128,286,-1317,-128,302,-1501,-128,318,-1501,-128,318,-1189,-128,302,-`
+`1253,-128,302,-1237,-760,302,-1189,-760,286,-1189,-760,286,-1237,-760,294,-1581,-608,294,-1605,-608,310,-1613,-604,334,-1565,-588,310,-1613,-604,302,-1669,-608,318,-1677,-599,422,-1581,-540,406,-1577,-540,390,-1573,-561,334,-1565,-588,390,-1541,-561,334,-1541,-588,334,-1565,-588,390,-1573,-561,294,-1549,-368,294,-1565,-368,318,-1565,-368,318,-1549,-368,374,-1581,-416,358,-1549,-416,302,-1581,-416,374,-1581,-416,302,-1581,-416,310,-1701,-416,470,-1701,-416,470,-1581,-416,342,-1541,-743,318,-1565,`
+`-753,319.6000061035156,-1585.800048828125,-759,326,-1669,-753,349.1111145019531,-1669,-743,487.77777099609375,-1669,-647,534,-1669,-640,534,-1557,-640,514,-1549,-640,494,-1541,-652,472.28570556640625,-1541,-657,498,-1561,-641,534,-1357,-768,534,-1189,-770,342,-1189,-768,334,-1357,-768,342,-1685,-583,422,-1685,-529,462,-1685,-513,462,-1589,-513,450,-1601,-513,534,-1493,-177,534,-1477,-177,342,-1477,-176,342,-1501,-177,534,-1493,-128,534,-1469,-128,342,-1469,-128,342,-1501,-128,382,-1253,-636,382,`
+`-1189,-636,342,-1189,-636,342,-1274.3333740234375,-633,342,-1317,-640,406,-1277,-640,394,-1265,-628,382,-1253,-636,342,-1317,-640,534,-1317,-640,534,-1277,-640,406,-1277,-640,342,-1317,-640,342,-1317,-488,534,-1317,-488,534,-1277,-488,342,-1277,-488,374,-1549,-368,374,-1565,-368,470,-1565,-368,470,-1549,-368,414,-1549,-535,446,-1573,-513,462,-1541,-513,446,-1543.6666259765625,-513,454,-1581,-768,438,-1541,-768,422,-1541,-768,422,-1677,-768,486,-1541,-768,462,-1541,-768,454,-1581,-768,486,-1677,-`
+`768,486,-1541,-768,454,-1581,-768,422,-1677,-768,438,-1221,74,438,-1189,74,422,-1189,74,422,-1221,74,438,-1453,-608,534,-1453,-608,534,-1413,-608,438,-1413,-608,438,-1437,92,534,-1437,92,534,-1365,92,438,-1365,92,462,-1237,145,534,-1237,131,534,-1189,131,454,-1189,147,486,-1213,74,486,-1189,74,462,-1189,74,462,-1221,74,486,-1701,-224,526,-1701,-235,534,-1637,-239,534,-1549,-239,486,-1549,-224,494,-1453,-526,534,-1453,-526,534,-1413,-526,494,-1413,-526,534,-1549,-416,518,-1549,-416,510,-1597,-416`
+`,510,-1701,-416,534,-1701,-416],"vertslength":423,"tris":[0,1,2,3,4,5,6,7,8,9,10,11,11,12,13,9,11,13,14,15,16,14,16,17,18,19,26,20,19,26,22,21,26,20,21,26,22,23,26,18,26,27,24,25,27,18,25,27,26,23,27,24,23,27,28,29,30,31,32,33,31,33,34,38,35,36,36,37,38,39,40,41,42,43,44,44,45,46,42,44,46,42,46,47,48,49,50,51,52,53,54,55,56,54,56,57,58,59,60,58,60,61,65,66,67,62,63,64,64,65,67,62,64,67,70,71,72,68,69,70,68,70,72,73,74,75,76,77,78,76,78,79,80,81,82,88,83,84,87,88,84,85,86,87,84,85,87,92,89,90,90,`
+`91,92,95,96,97,95,97,98,94,95,98,93,94,98,99,100,101,104,105,106,104,106,102,102,103,104,109,110,107,107,108,109,111,112,113,114,115,116,116,111,113,113,114,116,117,118,119,117,119,120,121,122,123,127,124,125,125,126,127,128,129,130,128,130,131,128,131,132,133,134,135,136,137,138,139,140,136,136,138,139,141,142,143,144,145,146,144,146,147,148,149,150,148,150,151,152,153,154,155,156,157,158,159,155,155,157,158,160,161,162,160,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,183`
+`,184,185,181,182,185,183,182,185,184,179,185,181,180,185,179,180,185,186,187,188,186,188,189,189,190,191,186,189,191,192,193,194,195,196,197,198,199,200,198,200,201,205,202,203,203,204,205,206,207,208,206,208,209,213,210,211,211,212,213,214,215,216,217,218,219,217,219,220,221,222,223,224,225,226,227,228,229,227,229,230,231,232,233,234,235,236,234,236,237,238,239,240,241,242,243,241,243,244,245,246,247,245,247,248,245,248,249,251,252,253,251,253,254,255,250,251,251,254,255,256,257,258,256,258,259`
+`,260,261,262,260,262,263,264,265,266,266,267,268,264,266,268,269,270,271,272,273,274,275,276,277,275,277,278,280,275,278,278,279,280,281,282,283,284,285,286,284,286,287,288,289,290,292,288,290,290,291,292,296,293,294,294,295,296,297,298,299,297,299,300,301,302,303,305,306,307,304,305,307,307,301,303,303,304,307,308,309,310,308,310,311,315,312,313,313,314,315,316,317,318,319,320,321,322,323,319,319,321,322,324,325,326,326,327,328,324,326,328,334,324,328,328,329,334,334,329,335,329,330,335,331,330`
+`,335,331,332,335,334,333,335,332,333,335,336,337,338,336,338,339,340,341,344,343,340,344,341,342,344,343,342,344,345,346,347,345,347,348,349,350,351,349,351,352,356,357,353,353,354,355,353,355,356,361,358,359,359,360,361,362,363,364,362,364,365,369,366,367,367,368,369,373,370,371,371,372,373,375,376,377,374,375,377,378,379,380,378,380,381,382,383,384,387,388,385,385,386,387,392,389,390,390,391,392,396,393,394,394,395,396,400,397,398,398,399,400,401,402,403,401,403,404,405,406,407,405,407,408,409`
+`,410,411,411,412,413,409,411,413,417,414,415,415,416,417,418,419,420,420,421,422,418,420,422],"trislength":226,"triTopoly":[0,1,2,3,3,3,4,4,5,5,5,5,5,5,5,5,5,5,6,7,7,8,8,9,10,10,10,10,11,12,13,13,14,14,15,15,15,15,16,16,16,17,18,18,19,20,20,20,20,21,21,22,22,22,22,23,24,24,24,25,25,26,26,26,26,27,27,28,29,29,30,30,30,31,32,32,32,33,34,34,35,35,36,37,37,37,38,38,39,40,41,42,43,44,44,44,44,44,44,45,45,45,45,46,47,48,48,49,49,50,50,51,51,52,53,53,54,55,56,56,57,58,58,59,60,60,61,61,61,62,62,62,62,6`
+`3,63,64,64,65,65,65,66,67,68,68,68,68,69,70,70,71,71,71,72,72,73,73,74,74,74,74,74,75,75,76,76,77,78,78,78,79,79,79,79,79,79,79,79,79,79,79,80,80,81,81,81,81,82,82,83,83,84,84,84,85,85,86,86,87,87,88,88,89,89,90,90,91,92,92,93,93,94,94,95,95,96,96,97,97,98,98,98,99,99,100,100,100],"baseVert":[0,3,6,9,14,18,28,31,35,39,42,48,51,54,58,62,68,73,76,80,83,89,93,99,102,107,111,117,121,124,128,133,136,141,144,148,152,155,160,164,167,170,173,176,179,186,192,195,198,202,206,210,214,217,221,224,227,231,23`
+`4,238,241,245,250,256,260,264,269,272,275,281,284,288,293,297,301,308,312,316,319,324,336,340,345,349,353,358,362,366,370,374,378,382,385,389,393,397,401,405,409,414,418],"vertsCount":[3,3,3,5,4,10,3,4,4,3,6,3,3,4,4,6,5,3,4,3,6,4,6,3,5,4,6,4,3,4,5,3,5,3,4,4,3,5,4,3,3,3,3,3,7,6,3,3,4,4,4,4,3,4,3,3,4,3,4,3,4,5,6,4,4,5,3,3,6,3,4,5,4,4,7,4,4,3,5,12,4,5,4,4,5,4,4,4,4,4,4,3,4,4,4,4,4,4,5,4,5],"baseTri":[0,1,2,3,6,8,18,19,21,23,24,28,29,30,32,34,38,41,42,44,45,49,51,55,56,59,61,65,67,68,70,73,74,77,78,`
+`80,82,83,86,88,89,90,91,92,93,99,103,104,105,107,109,111,113,114,116,117,118,120,121,123,124,126,129,133,135,137,140,141,142,146,147,149,152,154,156,161,163,165,166,169,180,182,186,188,190,193,195,197,199,201,203,205,206,208,210,212,214,216,218,221,223],"triCount":[1,1,1,3,2,10,1,2,2,1,4,1,1,2,2,4,3,1,2,1,4,2,4,1,3,2,4,2,1,2,3,1,3,1,2,2,1,3,2,1,1,1,1,1,6,4,1,1,2,2,2,2,1,2,1,1,2,1,2,1,2,3,4,2,2,3,1,1,4,1,2,3,2,2,5,2,2,1,3,11,2,4,2,2,3,2,2,2,2,2,2,1,2,2,2,2,2,2,3,2,3]},"links":{"poly":[4,16,27,54,`
+`37,57,42,62,50,56,50,52,52,55,52,56,55,56,55,74,56,73,57,73,60,96,61,95,74,81,74,89,76,77,77,88,81,89,93,97],"cost":[2221.5,1080,2649.179931640625,2707.199951171875,361.8461608886719,172.8000030517578,768,384,564.7058715820312,192,691.2000122070312,977.4720458984375,5430,901.8045654296875,1036.07177734375,806.2463989257812,3509.169189453125,3532.800048828125,902.1210327148438,864],"type":[2,1,2,2,1,1,1,1,1,1,1,1,2,2,2,1,2,2,1,1],"pos":[246,-1437,-55,246,-1421,-90,238,-1277,-640,238,-1253,-628,30`
+`2,-1509,-641,305.67999267578125,-1534.760009765625,-608,271.6000061035156,-1481.800048828125,-776,278,-1469,-736,246,-1629,-608,258.9230651855469,-1637.6153564453125,-608,246,-1629,-608,241.1999969482422,-1638.5999755859375,-608,246,-1653,-608,262,-1669,-608,238,-1645,-608,254,-1645,-608,273.29412841796875,-1671.823486328125,-608,278,-1653,-608,294,-1677,-608,302,-1669,-608,294,-1629,-608,303.6000061035156,-1609.800048828125,-608,310,-1549,-608,318.82757568359375,-1571.0689697265625,-598.6896362`
+`304688,534,-1253,74,534,-1237,132,420.3756408691406,-1363.7410888671875,75,438,-1365,92,326.97125244140625,-1668.7188720703125,-602.1341552734375,342,-1685,-588,422,-1581,-540,434.4800109863281,-1564.3599853515625,-529.760009765625,318,-1565,-368,320.953857421875,-1570.169189453125,-416,367.6000061035156,-1568.199951171875,-416,374,-1565,-368,460.04876708984375,-1590.56103515625,-514.219482421875,446,-1573,-524,438,-1221,74,462,-1221,74],"length":20}}],["7_2",{"tileId":"7_2","tx":7,"ty":2,"mesh"`
+`:{"verts":[846,-1701,-416,870,-1693,-415,870,-1597,-415,838,-1589,-416,838,-1589,-416,822,-1541,-416,734,-1549,-416,566,-1541,-416,534,-1549,-416,534,-1701,-416,846,-1701,-416,838,-1589,-416,734,-1549,-416,726,-1469,-415,566,-1469,-415,566,-1541,-416,734,-1549,-416,534,-1549,-640,534,-1669,-640,574,-1685,-640,934,-1685,-640,926,-1557,-640,886,-1541,-640,534,-1549,-240,534,-1629,-240,598,-1629,-240,598,-1629,-240,606,-1701,-240,870,-1701,-224,598,-1629,-240,870,-1701,-224,870,-1549,-224,534,-1549`
+`,-240,758,-1517,-272,758,-1477,-272,742,-1493,-272,742,-1493,-272,534,-1485,-272,534,-1517,-272,758,-1517,-272,670,-1357,74,630,-1357,74,630,-1453,81,678,-1461,78,630,-1453,81,534,-1461,77,534,-1517,74,678,-1461,78,894,-1517,74,870,-1461,77,678,-1461,78,534,-1517,74,534,-1493,-177,734,-1501,-177,534,-1477,-177,950,-1469,-128,534,-1469,-128,534,-1493,-128,950,-1501,-128,670,-1453,-608,750,-1453,-608,734,-1413,-608,534,-1413,-608,534,-1453,-608,614,-1453,-608,614,-1453,-608,622,-1533,-608,662,-153`
+`3,-608,670,-1453,-608,678,-1461,-526,734,-1461,-526,726,-1413,-526,534,-1413,-526,606,-1453,-526,534,-1413,-526,534,-1453,-526,606,-1453,-526,614,-1517,-526,670,-1517,-526,678,-1461,-526,606,-1453,-526,886,-1469,-416,934,-1469,-416,934,-1445,-416,878,-1389,-416,734,-1453,-416,790,-1453,-416,798,-1389,-416,878,-1389,-416,934,-1445,-416,966,-1437,-416,958,-1189,-414,566,-1469,-415,726,-1469,-415,734,-1453,-416,534,-1453,-416,622,-1189,-416,534,-1205,-416,534,-1453,-416,734,-1453,-416,798,-1389,-41`
+`6,958,-1189,-414,798,-1389,-416,878,-1389,-416,958,-1189,-414,534,-1365,92,534,-1437,92,606,-1437,92,606,-1365,92,678,-1341,-642,534,-1341,-642,534,-1365,-642,726,-1365,-642,574,-1245,-770,566,-1189,-770,534,-1189,-770,534,-1357,-768,878,-1269,-768,870,-1229,-768,814,-1221,-769,814,-1221,-769,814,-1189,-769,718,-1189,-769,814,-1221,-769,718,-1189,-769,710,-1245,-769,878,-1269,-768,950,-1357,-768,950,-1269,-768,878,-1269,-768,710,-1245,-769,574,-1245,-770,534,-1357,-768,950,-1357,-768,878,-1269,-`
+`768,622,-1341,84,630,-1357,74,670,-1357,74,678,-1341,78,702,-1253,74,694,-1189,74,598,-1189,74,598,-1245,74,622,-1341,84,678,-1341,78,598,-1245,74,534,-1253,74,534,-1341,75,622,-1341,84,878,-1341,81,886,-1253,74,702,-1253,74,678,-1341,78,950,-1189,-640,902,-1189,-640,894,-1277,-640,950,-1317,-640,894,-1277,-640,534,-1277,-640,534,-1317,-640,950,-1317,-640,918,-1261,-488,926,-1189,-488,910,-1189,-488,902,-1277,-488,958,-1189,-488,950,-1189,-488,942,-1269,-488,950,-1317,-488,942,-1269,-488,918,-12`
+`61,-488,902,-1277,-488,950,-1317,-488,902,-1277,-488,534,-1277,-488,534,-1317,-488,950,-1317,-488,534,-1189,129,534,-1237,130,582,-1237,120,582,-1189,119,542,-1645,-174,542,-1701,-174,590,-1701,-174,590,-1645,-174,558,-1653,-224,558,-1677,-224,582,-1677,-224,582,-1653,-224,606,-1189,-565,614,-1213,-563,670,-1213,-565,686,-1189,-563,662,-1205,-702,670,-1189,-702,638,-1189,-702,622,-1205,-702,694,-1365,92,694,-1437,92,862,-1437,92,862,-1365,92,742,-1405,-526,726,-1413,-526,734,-1461,-526,798,-1461`
+`,-526,798,-1405,-526,742,-1341,-526,742,-1405,-526,798,-1405,-526,862,-1389,-526,870,-1341,-526,742,-1221,74,742,-1189,74,726,-1189,74,726,-1221,74,950,-1501,-177,950,-1477,-176,742,-1477,-177,734,-1493,-177,750,-1405,-608,734,-1413,-608,750,-1453,-608,750,-1405,-608,750,-1453,-608,790,-1453,-608,798,-1389,-608,750,-1357,-608,798,-1389,-608,1046,-1389,-608,1046,-1349,-608,750,-1357,-608,766,-1237,149,886,-1237,120,886,-1189,119,758,-1189,149,790,-1213,74,790,-1189,74,766,-1189,74,766,-1221,74,83`
+`0,-1453,-416,854,-1453,-416,854,-1429,-416,870,-1213,-718,870,-1189,-718,830,-1189,-718,830,-1213,-718,1038,-1341,-526,870,-1341,-526,862,-1389,-526,1038,-1397,-526,870,-1597,-415,870,-1693,-415,878,-1701,-416,1046,-1701,-416,1038,-1541,-416,910,-1549,-416,1046,-1189,-408,1038,-1189,-408,1038,-1541,-416,1046,-1701,-416,886,-1453,81,870,-1461,77,894,-1517,74,902,-1245,74,886,-1253,74,878,-1341,81,1014,-1189,74,902,-1189,74,902,-1245,74,886,-1453,81,894,-1517,74,1014,-1517,74,1014,-1189,74,902,-12`
+`45,74,878,-1341,81,942,-1245,-766,942,-1197,-766,902,-1197,-766,894,-1245,-766,990,-1557,-638,974,-1541,-640,926,-1557,-640,1046,-1669,-640,1046,-1557,-640,990,-1557,-638,926,-1557,-640,934,-1685,-640,974,-1501,-128,990,-1501,-128,998,-1469,-128,998,-1189,-128,974,-1189,-128,982,-1189,-520,974,-1189,-520,974,-1269,-520],"vertslength":295,"polys":[0,3,4,6,7,12,13,16,17,22,23,25,26,28,29,32,33,35,36,39,40,43,44,47,48,51,52,54,55,58,59,64,65,68,69,73,74,76,77,80,81,84,85,87,88,91,92,95,96,101,102,1`
+`04,105,108,109,112,113,116,117,119,120,122,123,126,127,129,130,134,135,138,139,144,145,148,149,152,153,156,157,160,161,164,165,168,169,172,173,176,177,180,181,184,185,188,189,192,193,196,197,200,201,205,206,210,211,214,215,218,219,221,222,226,227,230,231,234,235,238,239,241,242,245,246,249,250,255,256,259,260,262,263,265,266,268,269,274,275,278,279,281,282,286,287,291,292,294],"polyslength":73,"regions":[2,2,2,2,7,3,3,3,26,26,11,11,11,27,25,16,16,12,12,12,1,1,1,1,1,1,10,28,4,4,4,4,4,4,9,9,9,9,19`
+`,19,22,22,22,22,23,20,29,32,34,13,15,15,38,39,17,17,17,21,40,42,43,18,6,6,5,5,5,5,24,8,8,44,45],"neighbors":[[[0],[1,62],[0],[1,2]],[[0],[0],[1,2]],[[0],[0],[0],[1,0],[1,1],[1,3]],[[1,23],[0],[1,2],[0]],[[0],[0],[0],[1,70],[0],[0]],[[0],[0],[1,7]],[[0],[0],[1,7]],[[1,6],[0],[0],[1,5]],[[0],[0],[1,9]],[[0],[0],[0],[1,8]],[[1,34],[0],[1,11],[0]],[[0],[0],[1,12],[1,10]],[[1,64],[0],[1,11],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,54],[0],[0],[0],[1,16]],[[0],[0],[0],[1,15]],[[0],[1,50],[0],[1,18`
+`],[1,19]],[[0],[0],[1,17]],[[0],[0],[1,17],[0]],[[0],[0],[1,22],[0]],[[0],[0],[1,24]],[[1,20],[0],[0],[1,25]],[[1,3],[0],[1,24],[0]],[[0],[0],[1,23],[1,21],[1,25],[0]],[[0],[1,22],[1,24]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,33]],[[0],[0],[1,31]],[[0],[0],[1,31]],[[1,30],[0],[1,33],[1,29]],[[0],[0],[1,33]],[[0],[1,28],[0],[1,32],[1,31]],[[0],[1,10],[0],[1,35]],[[0],[0],[0],[1,36],[1,34],[1,37]],[[0],[0],[0],[1,35]],[[1,65],[0],[1,35],[0]],[[0],[0],[1,39],[0]],[[0],[0],[0],[1,38]],`
+`[[0],[0],[0],[1,42]],[[0],[0],[1,42],[0]],[[0],[1,40],[1,43],[1,41]],[[0],[0],[0],[1,42]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,17],[0],[0],[1,51]],[[0],[1,50],[0],[1,61],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,15],[1,55]],[[1,54],[0],[0],[1,56],[0]],[[0],[0],[0],[1,55]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,51],[0],[0]],[[1,0],[0],[0],[1,63],[0],[0]],[[0],[0],[1,62],[0]],`
+`[[0],[1,12],[1,67]],[[0],[1,37],[1,67]],[[0],[0],[1,67]],[[1,64],[0],[0],[1,66],[1,65],[0]],[[0],[0],[0],[0]],[[0],[0],[1,70]],[[0],[0],[1,69],[1,4],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[846,-1701,-415,870,-1693,-415,870,-1597,-415,838,-1589,-416,838,-1589,-416,822,-1541,-416,734,-1549,-416,566,-1541,-415,534,-1549,-416,534,-1701,-416,846,-1701,-415,838,-1589,-416,734,-1549,-416,726,-1469,-415,566,-1469,-415,566,-1541,-415,734,-1549,-416,534,-1549,-640,534,-1669,-640,574,`
+`-1685,-640,934,-1685,-640,926,-1557,-640,886,-1541,-640,618,-1625,-634,534,-1549,-240,534,-1629,-240,598,-1629,-240,598,-1629,-240,606,-1701,-240,804,-1701,-240,848,-1701,-224,870,-1701,-224,847.3333129882812,-1695,-224,802,-1683,-240,598,-1629,-240,802,-1683,-240,847.3333129882812,-1695,-224,870,-1701,-224,870,-1549,-224,847.5999755859375,-1549,-224,802.7999877929688,-1549,-240,534,-1549,-240,758,-1517,-272,758,-1477,-272,742,-1493,-272,742,-1493,-272,534,-1485,-272,534,-1517,-272,758,-1517,-27`
+`2,670,-1357,74,630,-1357,74,630,-1453,74,678,-1461,77,630,-1453,74,534,-1461,77,534,-1517,74,678,-1461,77,894,-1517,74,870,-1461,77,678,-1461,77,534,-1517,74,534,-1493,-176,734,-1501,-177,534,-1477,-176,950,-1469,-128,534,-1469,-128,534,-1493,-128,950,-1501,-128,670,-1453,-608,750,-1453,-608,734,-1413,-608,534,-1413,-608,534,-1453,-608,614,-1453,-608,614,-1453,-608,622,-1533,-608,662,-1533,-608,670,-1453,-608,678,-1461,-526,734,-1461,-526,726,-1413,-526,534,-1413,-526,606,-1453,-526,534,-1413,-5`
+`26,534,-1453,-526,606,-1453,-526,614,-1517,-526,670,-1517,-526,678,-1461,-526,606,-1453,-526,886,-1469,-416,934,-1469,-416,934,-1445,-416,878,-1389,-416,734,-1453,-416,790,-1453,-416,798,-1389,-416,878,-1389,-416,934,-1445,-416,966,-1437,-416,958,-1189,-414,566,-1469,-416,726,-1469,-416,734,-1453,-416,534,-1453,-416,622,-1189,-416,534,-1205,-416,534,-1453,-416,734,-1453,-416,798,-1389,-416,958,-1189,-414,798,-1389,-416,878,-1389,-416,958,-1189,-414,534,-1365,92,534,-1437,92,606,-1437,92,606,-136`
+`5,92,678,-1341,-642,534,-1341,-642,534,-1365,-642,726,-1365,-642,574,-1245,-770,566,-1189,-770,534,-1189,-770,534,-1357,-768,878,-1269,-768,870,-1229,-768,814,-1221,-769,814,-1221,-769,814,-1189,-769,718,-1189,-769,814,-1221,-769,718,-1189,-769,710,-1245,-769,878,-1269,-768,950,-1357,-768,950,-1269,-768,878,-1269,-768,710,-1245,-769,574,-1245,-770,534,-1357,-768,950,-1357,-768,878,-1269,-768,622,-1341,74,630,-1357,74,670,-1357,75,678,-1341,74,702,-1253,74,694,-1189,74,598,-1189,74,598,-1245,74,6`
+`22,-1341,74,678,-1341,74,598,-1245,74,534,-1253,74,534,-1341,74,622,-1341,74,878,-1341,74,886,-1253,74,702,-1253,74,678,-1341,74,950,-1189,-640,902,-1189,-640,894,-1277,-640,950,-1317,-640,894,-1277,-640,534,-1277,-640,534,-1317,-640,950,-1317,-640,918,-1261,-488,926,-1189,-488,910,-1189,-488,902,-1277,-488,958,-1189,-488,950,-1189,-488,942,-1269,-488,950,-1317,-488,942,-1269,-488,918,-1261,-488,902,-1277,-488,950,-1317,-488,902,-1277,-488,534,-1277,-488,534,-1317,-488,950,-1317,-488,534,-1189,1`
+`27,534,-1237,127,582,-1237,119,582,-1189,119,542,-1645,-174,542,-1701,-174,590,-1701,-174,590,-1645,-174,558,-1653,-224,558,-1677,-224,582,-1677,-224,582,-1653,-224,606,-1189,-565,614,-1213,-565,670,-1213,-563,686,-1189,-563,662,-1205,-702,670,-1189,-702,638,-1189,-702,622,-1205,-702,694,-1365,92,694,-1437,92,862,-1437,92,862,-1365,92,742,-1405,-526,726,-1413,-526,734,-1461,-526,798,-1461,-526,798,-1405,-526,742,-1341,-526,742,-1405,-526,798,-1405,-526,862,-1389,-526,870,-1341,-526,742,-1221,74,`
+`742,-1189,74,726,-1189,74,726,-1221,74,950,-1501,-177,950,-1477,-176,742,-1477,-177,734,-1493,-177,750,-1405,-608,734,-1413,-608,750,-1453,-608,750,-1405,-608,750,-1453,-608,790,-1453,-608,798,-1389,-608,750,-1357,-608,798,-1389,-608,1046,-1389,-608,1046,-1349,-608,750,-1357,-608,766,-1237,145,886,-1237,119,886,-1189,119,758,-1189,147,790,-1213,74,790,-1189,74,766,-1189,74,766,-1221,74,830,-1453,-416,854,-1453,-416,854,-1429,-416,870,-1213,-718,870,-1189,-718,830,-1189,-718,830,-1213,-718,1038,-`
+`1341,-526,870,-1341,-526,862,-1389,-526,1038,-1397,-526,870,-1597,-416,870,-1693,-416,878,-1701,-416,1046,-1701,-416,1039.142822265625,-1563.857177734375,-416,1038,-1541,-408,1016.6666870117188,-1542.3333740234375,-416,910,-1549,-416,1046,-1189,-408,1038,-1189,-408,1038,-1541,-408,1039.142822265625,-1563.857177734375,-416,1046,-1701,-416,1046,-1561.3636474609375,-416,1046,-1538.0909423828125,-408,886,-1453,74,870,-1461,74,894,-1517,74,902,-1245,74,886,-1253,74,878,-1341,74,1014,-1189,74,902,-118`
+`9,74,902,-1245,74,886,-1453,74,894,-1517,74,1014,-1517,74,1014,-1189,74,902,-1245,74,878,-1341,74,942,-1245,-766,942,-1197,-766,902,-1197,-766,894,-1245,-766,990,-1557,-640,974,-1541,-640,926,-1557,-640,1046,-1669,-640,1046,-1557,-640,990,-1557,-640,926,-1557,-640,934,-1685,-640,974,-1501,-128,990,-1501,-128,998,-1469,-128,998,-1189,-128,974,-1189,-128,982,-1189,-520,974,-1189,-520,974,-1269,-520],"vertslength":309,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,10,11,12,12,7,9,9,10,12,13,14,15,13,15,16,20,21,2`
+`2,22,17,23,17,18,23,19,18,23,19,20,23,22,20,23,24,25,26,30,31,32,29,30,32,29,32,33,28,29,33,27,28,33,35,36,37,37,38,39,37,39,40,35,37,40,34,35,40,34,40,41,42,43,44,45,46,47,45,47,48,49,50,51,49,51,52,53,54,55,53,55,56,57,58,59,57,59,60,61,62,63,64,65,66,64,66,67,68,69,70,71,72,73,73,68,70,70,71,73,74,75,76,74,76,77,78,79,80,82,78,80,80,81,82,83,84,85,86,87,88,86,88,89,90,91,92,90,92,93,94,95,96,97,98,99,97,99,100,101,102,103,101,103,104,107,108,109,105,106,107,105,107,109,105,109,110,111,112,113`
+`,117,114,115,115,116,117,118,119,120,118,120,121,122,123,124,122,124,125,126,127,128,129,130,131,132,133,134,132,134,135,136,137,138,139,140,141,142,143,139,139,141,142,144,145,146,144,146,147,149,150,151,152,153,148,148,149,151,148,151,152,154,155,156,154,156,157,158,159,160,158,160,161,162,163,164,162,164,165,166,167,168,166,168,169,170,171,172,170,172,173,174,175,176,174,176,177,178,179,180,178,180,181,182,183,184,182,184,185,189,186,187,187,188,189,193,190,191,191,192,193,197,194,195,195,196`
+`,197,198,199,200,198,200,201,202,203,204,202,204,205,209,206,207,207,208,209,210,211,212,213,214,210,210,212,213,215,216,217,217,218,219,215,217,219,223,220,221,221,222,223,224,225,226,224,226,227,228,229,230,231,232,233,234,235,231,231,233,234,236,237,238,236,238,239,240,241,242,240,242,243,244,245,246,244,246,247,248,249,250,254,251,252,252,253,254,255,256,257,255,257,258,263,264,265,259,260,261,263,265,266,266,259,261,262,263,266,261,262,266,270,271,272,269,270,272,269,272,273,273,267,268,268`
+`,269,273,274,275,276,277,278,279,280,281,282,283,284,285,287,288,283,286,287,283,283,285,286,289,290,291,289,291,292,293,294,295,296,297,298,298,299,300,296,298,300,301,302,303,303,304,305,301,303,305,306,307,308],"trislength":164,"triTopoly":[0,0,1,2,2,2,2,3,3,4,4,4,4,4,4,5,6,6,6,6,6,7,7,7,7,7,7,8,9,9,10,10,11,11,12,12,13,14,14,15,15,15,15,16,16,17,17,17,18,19,19,20,20,21,22,22,23,23,24,24,24,24,25,26,26,27,27,28,28,29,30,31,31,32,33,33,33,34,34,35,35,35,35,36,36,37,37,38,38,39,39,40,40,41,41,4`
+`2,42,43,43,44,44,45,45,46,46,47,47,48,48,49,49,50,50,50,51,51,51,52,52,53,53,54,55,55,55,56,56,57,57,58,58,59,60,60,61,61,62,62,62,62,62,62,63,63,63,63,63,64,65,66,67,67,67,67,68,68,69,70,70,70,71,71,71,72],"baseVert":[0,4,7,13,17,24,27,34,42,45,49,53,57,61,64,68,74,78,83,86,90,94,97,101,105,111,114,118,122,126,129,132,136,139,144,148,154,158,162,166,170,174,178,182,186,190,194,198,202,206,210,215,220,224,228,231,236,240,244,248,251,255,259,267,274,277,280,283,289,293,296,301,306],"vertsCount":[`
+`4,3,6,4,7,3,7,8,3,4,4,4,4,3,4,6,4,5,3,4,4,3,4,4,6,3,4,4,4,3,3,4,3,5,4,6,4,4,4,4,4,4,4,4,4,4,4,4,4,4,5,5,4,4,3,5,4,4,4,3,4,4,8,7,3,3,3,6,4,3,5,5,3],"baseTri":[0,2,3,7,9,15,16,21,27,28,30,32,34,36,37,39,43,45,48,49,51,53,54,56,58,62,63,65,67,69,70,71,73,74,77,79,83,85,87,89,91,93,95,97,99,101,103,105,107,109,111,114,117,119,121,122,125,127,129,131,132,134,136,142,147,148,149,150,154,156,157,160,163],"triCount":[2,1,4,2,6,1,5,6,1,2,2,2,2,1,2,4,2,3,1,2,2,1,2,2,4,1,2,2,2,1,1,2,1,3,2,4,2,2,2,2,2,2,2,2`
+`,2,2,2,2,2,2,3,3,2,2,1,3,2,2,2,1,2,2,6,5,1,1,1,4,2,1,3,3,1]},"links":{"poly":[4,16,13,53,29,60,35,44,37,57,41,72,52,58],"cost":[1792.9251708984375,94.63722229003906,4017.34130859375,3421.5,3558,1920,864],"type":[2,1,2,2,2,2,1],"pos":[662.29736328125,-1546.0841064453125,-640,662,-1533,-608,733.0536499023438,-1500.886474609375,-177,734,-1493,-177,828.5599975585938,-1223.0799560546875,-768.739990234375,830,-1213,-718,598,-1189,74,582,-1189,119,886,-1253,74,886,-1237,120,958,-1189,-488,974,-1189,-52`
+`0,742,-1221,74,766,-1221,74],"length":7}}],["8_2",{"tileId":"8_2","tx":8,"ty":2,"mesh":{"verts":[1246,-1429,-416,1262,-1405,-416,1166,-1397,-416,1070,-1397,-408,1062,-1301,-408,1046,-1285,-408,1046,-1701,-416,1430,-1701,-416,1414,-1677,-416,1302,-1685,-416,1246,-1429,-416,1166,-1397,-416,1070,-1397,-408,1046,-1701,-416,1302,-1685,-416,1046,-1701,-416,1430,-1701,-416,1302,-1685,-416,1046,-1549,-640,1046,-1669,-640,1070,-1677,-640,1070,-1541,-640,1070,-1677,-640,1070,-1701,-640,1198,-1701,-640,120`
+`6,-1389,-640,1190,-1277,-640,1070,-1269,-645,1070,-1541,-640,1070,-1677,-640,1198,-1701,-640,1182,-1389,-416,1166,-1397,-416,1262,-1405,-416,1046,-1189,-408,1046,-1285,-408,1062,-1301,-408,1174,-1293,-416,1174,-1293,-416,1182,-1389,-416,1262,-1405,-416,1374,-1389,-416,1174,-1293,-416,1374,-1389,-416,1382,-1189,-416,1046,-1189,-408,1070,-1221,-522,1142,-1213,-527,1126,-1189,-538,1054,-1189,-538,1062,-1189,-698,1070,-1269,-645,1190,-1277,-640,1198,-1189,-698,1062,-1213,-768,1110,-1205,-768,1062,-1`
+`197,-768,1094,-1309,-340,1094,-1381,-340,1166,-1381,-340,1166,-1309,-340,1102,-1325,-414,1102,-1373,-414,1150,-1373,-414,1150,-1325,-414,1182,-1237,-511,1198,-1237,-511,1198,-1189,-538,1150,-1189,-538,1230,-1189,-480,1230,-1701,-480,1398,-1701,-480,1398,-1189,-480,1558,-1389,-768,1542,-1373,-768,1238,-1365,-768,1238,-1701,-768,1558,-1701,-768,1278,-1229,-640,1558,-1229,-640,1558,-1189,-640,1238,-1189,-640,1238,-1701,-640,1278,-1701,-640,1278,-1229,-640,1238,-1189,-640,1558,-1357,-768,1558,-1325,`
+`-768,1542,-1317,-768,1238,-1365,-768,1542,-1373,-768,1542,-1317,-768,1558,-1293,-768,1558,-1189,-768,1238,-1189,-768,1238,-1365,-768,1318,-1677,-288,1342,-1677,-288,1342,-1621,-288,1262,-1429,-288,1342,-1621,-288,1406,-1613,-288,1414,-1589,-288,1374,-1405,-288,1262,-1429,-288,1326,-1661,-415,1406,-1645,-415,1406,-1629,-415,1358,-1421,-415,1294,-1429,-415,1286,-1469,-415,1318,-1277,-676,1318,-1677,-676,1518,-1677,-676,1518,-1277,-676,1414,-1645,-257,1406,-1629,-257,1366,-1629,-257,1350,-1661,-257`
+`,1438,-1661,-416,1414,-1677,-416,1430,-1701,-416,1494,-1701,-416,1438,-1629,-416,1438,-1661,-416,1494,-1701,-416,1494,-1589,-416,1374,-1389,-416,1438,-1629,-416,1494,-1589,-416,1374,-1389,-416,1494,-1589,-416,1558,-1581,-408,1558,-1189,-416,1382,-1189,-416,1430,-1189,-480,1430,-1701,-480,1558,-1701,-480,1558,-1189,-480,1518,-1605,-88,1518,-1701,-88,1558,-1701,-88,1558,-1605,-88,1542,-1629,-168,1542,-1701,-168,1558,-1701,-168,1558,-1629,-168,1542,-1685,-268,1558,-1685,-268,1558,-1629,-268,1542,-1`
+`629,-268,1558,-1685,-414,1558,-1637,-414,1542,-1661,-414,1558,-1685,-340,1558,-1637,-340,1542,-1661,-340],"vertslength":157,"polys":[0,2,3,6,7,9,10,14,15,17,18,21,22,24,25,30,31,33,34,37,38,41,42,45,46,49,50,53,54,56,57,60,61,64,65,68,69,72,73,77,78,81,82,85,86,90,91,95,96,99,100,104,105,110,111,114,115,118,119,122,123,126,127,129,130,134,135,138,139,142,143,146,147,150,151,153,154,156],"polyslength":39,"regions":[3,3,3,3,3,8,8,8,5,5,5,5,18,9,23,13,15,19,7,1,14,14,2,2,11,11,12,6,20,4,4,4,4,10,17`
+`,25,26,27,28],"neighbors":[[[0],[1,8],[1,3]],[[0],[1,9],[0],[1,3]],[[1,29],[0],[1,4]],[[1,0],[0],[1,1],[1,4],[0]],[[0],[1,2],[1,3]],[[0],[0],[1,7],[0]],[[0],[0],[1,7]],[[0],[1,13],[0],[1,5],[1,6],[0]],[[0],[1,0],[1,10]],[[0],[1,1],[0],[1,11]],[[0],[1,8],[0],[1,11]],[[1,10],[1,32],[0],[1,9]],[[0],[0],[0],[0]],[[0],[1,7],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,22],[0],[0],[0]],[[0],[0],[0],[1,21]],[[0],[0],[1,20],[0]],[[0],[0],[1,23],[`
+`1,19],[0]],[[0],[0],[0],[0],[1,22]],[[0],[0],[1,25],[0]],[[0],[0],[0],[0],[1,24]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,2],[0],[1,30]],[[0],[1,29],[0],[1,31]],[[0],[1,30],[1,32]],[[1,31],[0],[0],[0],[1,11]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[1246,-1429,-416,1262,-1405,-416,1166,-1397,-416,1070,-1397,-408,1062,-1301,-408,1046,-1285,-408,1046,-1701,-416,1430,-1701,-416,1414,-1677,-`
+`416,1302,-1685,-416,1246,-1429,-416,1166,-1397,-416,1089.199951171875,-1397,-416,1070,-1397,-408,1046,-1701,-416,1302,-1685,-416,1082,-1401,-416,1046,-1701,-416,1430,-1701,-416,1302,-1685,-416,1046,-1549,-640,1046,-1669,-640,1070,-1677,-640,1070,-1541,-640,1070,-1677,-640,1070,-1701,-640,1198,-1701,-640,1206,-1389,-640,1190,-1277,-640,1070,-1269,-645,1070,-1541,-640,1070,-1677,-640,1198,-1701,-640,1182,-1389,-416,1166,-1397,-416,1262,-1405,-416,1046,-1189,-408,1046,-1285,-408,1062,-1301,-408,108`
+`4.4000244140625,-1299.4000244140625,-416,1174,-1293,-416,1082.5714111328125,-1218.7142333984375,-416,1174,-1293,-416,1182,-1389,-416,1262,-1405,-416,1374,-1389,-416,1174,-1293,-416,1374,-1389,-416,1382,-1189,-416,1090.800048828125,-1189,-416,1046,-1189,-408,1082.5714111328125,-1218.7142333984375,-416,1070,-1221,-527,1142,-1213,-527,1134,-1201,-527,1126,-1189,-538,1054,-1189,-538,1062,-1205,-538,1114,-1209,-533,1062,-1189,-698,1064,-1209,-693,1070,-1269,-655,1190,-1277,-650,1196,-1211,-693,1198,-`
+`1189,-698,1062,-1213,-768,1110,-1205,-768,1062,-1197,-768,1094,-1309,-340,1094,-1381,-340,1166,-1381,-340,1166,-1309,-340,1102,-1325,-414,1102,-1373,-414,1150,-1373,-414,1150,-1325,-414,1182,-1237,-517,1198,-1237,-517,1198,-1205,-538,1198,-1189,-538,1150,-1189,-538,1160.6666259765625,-1205,-538,1230,-1189,-480,1230,-1701,-480,1398,-1701,-480,1398,-1189,-480,1558,-1389,-768,1542,-1373,-768,1238,-1365,-768,1238,-1701,-768,1558,-1701,-768,1278,-1229,-640,1558,-1229,-640,1558,-1189,-640,1238,-1189,-`
+`640,1238,-1701,-640,1278,-1701,-640,1278,-1229,-640,1238,-1189,-640,1558,-1357,-768,1558,-1325,-768,1542,-1317,-768,1238,-1365,-768,1542,-1373,-768,1542,-1317,-768,1558,-1293,-768,1558,-1189,-768,1238,-1189,-768,1238,-1365,-768,1318,-1677,-288,1342,-1677,-288,1342,-1621,-288,1262,-1429,-288,1342,-1621,-288,1406,-1613,-288,1414,-1589,-288,1374,-1405,-288,1262,-1429,-288,1326,-1661,-415,1406,-1645,-415,1406,-1629,-415,1358,-1421,-415,1294,-1429,-415,1286,-1469,-415,1318,-1277,-676,1318,-1677,-676,`
+`1518,-1677,-676,1518,-1277,-676,1414,-1645,-257,1406,-1629,-257,1366,-1629,-257,1350,-1661,-257,1438,-1661,-416,1414,-1677,-416,1430,-1701,-416,1494,-1701,-416,1438,-1629,-416,1438,-1661,-416,1494,-1701,-416,1494,-1611.4000244140625,-416,1494,-1589,-408,1475.3333740234375,-1602.3333740234375,-416,1374,-1389,-416,1438,-1629,-416,1475.3333740234375,-1602.3333740234375,-416,1494,-1589,-408,1482,-1569,-416,1374,-1389,-416,1482,-1569,-416,1494,-1589,-408,1558,-1581,-408,1558,-1534.88232421875,-416,15`
+`58,-1189,-416,1382,-1189,-416,1506,-1553,-408,1430,-1189,-480,1430,-1701,-480,1558,-1701,-480,1558,-1189,-480,1518,-1605,-88,1518,-1701,-88,1558,-1701,-88,1558,-1605,-88,1542,-1629,-168,1542,-1701,-168,1558,-1701,-168,1558,-1629,-168,1542,-1685,-268,1558,-1685,-268,1558,-1629,-268,1542,-1629,-268,1558,-1685,-414,1558,-1637,-414,1542,-1661,-414,1558,-1685,-340,1558,-1637,-340,1542,-1661,-340],"vertslength":177,"tris":[0,1,2,3,4,5,3,5,6,7,8,9,15,10,16,13,12,16,10,11,16,12,11,16,13,14,16,15,14,16,1`
+`7,18,19,23,20,21,21,22,23,24,25,26,27,28,29,30,31,32,27,29,30,27,30,32,33,34,35,37,38,39,41,36,37,41,37,39,39,40,41,42,43,44,42,44,45,49,50,51,49,51,46,46,47,48,46,48,49,55,56,57,52,53,58,53,54,58,54,55,58,52,57,58,55,57,58,64,59,60,63,64,60,60,61,62,60,62,63,65,66,67,71,68,69,69,70,71,75,72,73,73,74,75,76,77,78,76,78,79,79,80,81,76,79,81,85,82,83,83,84,85,86,87,88,89,90,86,86,88,89,91,92,93,91,93,94,95,96,97,95,97,98,99,100,101,103,99,101,101,102,103,104,105,106,104,106,107,104,107,108,109,110,`
+`111,109,111,112,113,114,115,113,115,116,113,116,117,118,119,120,121,122,123,123,118,120,120,121,123,127,124,125,125,126,127,128,129,130,128,130,131,132,133,134,132,134,135,139,140,141,139,141,136,139,136,137,137,138,139,144,145,146,143,144,146,142,143,146,152,153,147,147,151,152,147,148,154,151,147,154,151,150,154,148,149,154,150,149,154,158,155,156,156,157,158,162,159,160,160,161,162,166,163,164,164,165,166,170,167,168,168,169,170,171,172,173,174,175,176],"trislength":102,"triTopoly":[0,1,1,2,3`
+`,3,3,3,3,3,4,5,5,6,7,7,7,7,8,9,9,9,9,10,10,11,11,11,11,12,12,12,12,12,12,13,13,13,13,14,15,15,16,16,17,17,17,17,18,18,19,19,19,20,20,21,21,22,22,22,23,23,23,24,24,25,25,25,26,26,26,26,27,27,28,28,29,29,30,30,30,30,31,31,31,32,32,32,32,32,32,32,33,33,34,34,35,35,36,36,37,38],"baseVert":[0,3,7,10,17,20,24,27,33,36,42,46,52,59,65,68,72,76,82,86,91,95,99,104,109,113,118,124,128,132,136,142,147,155,159,163,167,171,174],"vertsCount":[3,4,3,7,3,4,3,6,3,6,4,6,7,6,3,4,4,6,4,5,4,4,5,5,4,5,6,4,4,4,6,5,8,4,`
+`4,4,4,3,3],"baseTri":[0,1,3,4,10,11,13,14,18,19,23,25,29,35,39,40,42,44,48,50,53,55,57,60,63,65,68,72,74,76,78,82,85,92,94,96,98,100,101],"triCount":[1,2,1,6,1,2,1,4,1,4,2,4,6,4,1,2,2,4,2,3,2,2,3,3,2,3,4,2,2,2,4,3,7,2,2,2,2,1,1]},"links":{"poly":[12,17,17,18,18,33,24,28],"cost":[619.0317993164062,2977.5,1536,1537.5],"type":[1,2,1,2],"pos":[1142,-1213,-527,1158.6153564453125,-1201.923095703125,-530.7307739257812,1198,-1237,-511,1230,-1237,-480,1398,-1701,-480,1430,-1701,-480,1342,-1661,-288,1350,`
+`-1661,-257],"length":4}}],["9_2",{"tileId":"9_2","tx":9,"ty":2,"mesh":{"verts":[1662,-1573,-768,1630,-1573,-768,1622,-1589,-768,1678,-1701,-768,1678,-1589,-768,1622,-1589,-768,1598,-1573,-768,1558,-1557,-768,1558,-1701,-768,1678,-1701,-768,1558,-1189,-480,1558,-1701,-480,1630,-1701,-480,1630,-1189,-480,1558,-1701,-168,1566,-1701,-168,1558,-1629,-168,2054,-1637,-88,2062,-1701,-88,2070,-1701,-88,2070,-1605,-88,1950,-1637,-88,2054,-1637,-88,2070,-1605,-88,1558,-1605,-88,1558,-1701,-88,1942,-1701,-8`
+`8,1950,-1637,-88,1558,-1605,-88,1606,-1653,-268,1598,-1629,-268,1558,-1629,-268,1558,-1693,-268,1590,-1693,-268,1590,-1693,-414,1598,-1661,-414,1598,-1645,-414,1558,-1637,-414,1558,-1685,-414,1590,-1693,-340,1598,-1661,-340,1598,-1645,-340,1558,-1637,-340,1558,-1685,-340,1630,-1229,-416,1630,-1189,-416,1558,-1189,-416,1558,-1581,-408,1742,-1629,-416,1630,-1629,-416,1614,-1701,-416,2030,-1701,-416,2030,-1629,-416,1910,-1621,-416,1742,-1629,-416,1614,-1701,-416,2006,-1189,-408,1646,-1189,-392,1646`
+`,-1229,-401,1646,-1229,-401,1630,-1229,-416,1558,-1581,-408,1750,-1589,-415,1742,-1629,-416,1910,-1621,-416,1918,-1581,-408,1558,-1581,-408,1750,-1589,-415,1918,-1581,-408,1918,-1581,-408,2006,-1581,-408,2006,-1189,-408,1646,-1229,-401,1558,-1581,-408,1630,-1573,-768,1662,-1573,-768,1678,-1557,-768,1606,-1557,-768,1558,-1557,-768,1598,-1573,-768,1606,-1557,-768,1558,-1189,-768,1558,-1557,-768,1606,-1557,-768,1678,-1557,-768,1678,-1557,-768,1678,-1701,-768,2046,-1701,-768,2046,-1197,-768,2022,-11`
+`89,-768,1558,-1189,-768,1678,-1557,-768,2046,-1701,-768,1878,-1229,-640,1862,-1197,-640,1838,-1189,-640,1558,-1189,-640,1558,-1229,-640,1630,-1237,-640,1662,-1237,-640,1878,-1229,-640,1838,-1189,-640,1558,-1325,-640,1558,-1373,-640,1630,-1373,-640,1630,-1317,-640,1662,-1237,-640,1630,-1237,-640,1630,-1317,-640,1630,-1373,-640,1630,-1701,-640,1662,-1701,-640,1630,-1645,-168,1630,-1629,-168,1598,-1629,-168,1598,-1653,-168,1694,-1205,-480,1678,-1189,-480,1662,-1189,-480,1726,-1189,-480,1702,-1189,-`
+`480,1694,-1205,-480,1726,-1213,-480,1758,-1237,-480,1750,-1213,-480,1726,-1213,-480,1758,-1237,-480,1726,-1213,-480,1694,-1205,-480,1846,-1701,-480,1846,-1245,-480,1758,-1237,-480,1758,-1237,-480,1694,-1205,-480,1662,-1189,-480,1662,-1701,-480,1846,-1701,-480,1662,-1629,-168,1662,-1701,-168,1702,-1701,-168,1702,-1629,-168,2070,-1237,-60,2054,-1221,-60,2030,-1237,-60,2070,-1581,-60,2006,-1213,-60,2014,-1189,-60,1686,-1189,-60,2030,-1237,-60,2006,-1213,-60,1686,-1189,-60,1686,-1581,-60,2070,-1581,`
+`-60,1734,-1629,-168,1734,-1669,-168,1774,-1669,-168,1774,-1629,-168,1806,-1629,-168,1806,-1669,-168,1838,-1669,-168,1838,-1629,-168,1878,-1189,-640,1862,-1197,-640,1878,-1229,-640,2014,-1237,-640,2022,-1189,-640,2046,-1197,-640,2022,-1189,-640,2014,-1237,-640,2014,-1701,-640,2046,-1701,-640,1870,-1629,-168,1870,-1669,-168,1910,-1669,-168,1910,-1629,-168,2046,-1589,-480,2038,-1589,-480,2006,-1621,-480,2054,-1701,-480,2054,-1701,-480,2006,-1621,-480,1982,-1589,-480,1878,-1573,-480,1878,-1701,-480,`
+`2006,-1213,-480,2014,-1189,-480,1990,-1189,-480,1974,-1253,-480,2046,-1229,-480,2006,-1213,-480,1974,-1253,-480,1878,-1573,-480,1982,-1589,-480,2046,-1589,-480,1974,-1253,-480,1878,-1237,-481,1878,-1573,-480,1942,-1669,-168,1974,-1629,-168,1942,-1629,-168,1958,-1653,-20,1958,-1701,-20,2046,-1701,-20,2046,-1653,-20,1966,-1661,-88,1966,-1701,-88,2038,-1701,-88,2038,-1661,-88,2046,-1629,-168,2006,-1629,-168,2006,-1645,-168,2046,-1653,-168,2046,-1581,-408,2070,-1581,-408,2070,-1277,-408,2046,-1261,-`
+`408,2046,-1245,-352,2070,-1261,-352,2070,-1237,-352,2046,-1221,-352,2054,-1685,-268,2070,-1685,-268,2070,-1629,-268,2054,-1629,-268],"vertslength":224,"polys":[0,4,5,9,10,13,14,16,17,20,21,24,25,28,29,33,34,38,39,43,44,47,48,50,51,55,56,58,59,61,62,65,66,68,69,73,74,77,78,80,81,84,85,87,88,92,93,95,96,101,102,105,106,111,112,115,116,118,119,122,123,125,126,128,129,131,132,136,137,140,141,144,145,147,148,152,153,156,157,160,161,165,166,170,171,174,175,178,179,183,184,187,188,193,194,196,197,199,2`
+`00,203,204,207,208,211,212,215,216,219,220,223],"polyslength":55,"regions":[7,7,9,21,8,8,8,13,18,19,2,2,2,2,2,2,2,2,1,1,1,1,1,10,10,10,10,22,4,4,4,4,4,4,14,3,3,3,15,20,12,12,16,6,6,5,5,5,28,11,17,29,31,32,33],"neighbors":[[[1,18],[0],[1,1],[0],[0]],[[0],[1,19],[0],[0],[1,0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,5]],[[0],[1,4],[0],[1,6]],[[0],[0],[1,5],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[1,14]],[[0],[0],[1,12]],[[0],[0],[1,15],[1,11],[0]`
+`],[[0],[0],[1,17]],[[0],[1,10],[1,17]],[[0],[1,12],[0],[1,16]],[[0],[1,15],[1,17]],[[0],[0],[1,13],[1,14],[1,16]],[[1,0],[0],[1,20],[0]],[[1,1],[0],[1,20]],[[0],[1,19],[1,18],[1,22]],[[0],[0],[1,22]],[[0],[0],[1,20],[1,21],[0]],[[1,40],[0],[1,24]],[[0],[0],[1,26],[0],[1,23],[0]],[[0],[0],[1,26],[0]],[[1,24],[0],[1,25],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,33]],[[0],[0],[1,31],[0]],[[0],[0],[1,31]],[[1,30],[1,29],[1,33]],[[0],[0],[1,33]],[[1,31],[1,28],[0],[0],[1,32]],[[0],[0],[0],[0]],[[0],`
+`[0],[1,37],[0]],[[0],[0],[1,37]],[[0],[1,36],[0],[0],[1,35]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,23],[0],[1,41],[0]],[[0],[1,40],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,44],[0]],[[1,43],[0],[1,46],[0],[0]],[[0],[0],[0],[1,46]],[[0],[1,45],[1,47],[1,44],[0],[0]],[[0],[0],[1,46]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1662,-1573,-768,1630,-1573,-768,1622,-1589,-768,1678,-1701,-768,167`
+`8,-1589,-768,1622,-1589,-768,1598,-1573,-768,1558,-1557,-768,1558,-1701,-768,1678,-1701,-768,1558,-1189,-480,1558,-1701,-480,1630,-1701,-480,1630,-1189,-480,1558,-1701,-168,1566,-1701,-168,1558,-1629,-168,2054,-1637,-88,2062,-1701,-88,2070,-1701,-88,2070,-1605,-88,1950,-1637,-88,2054,-1637,-88,2070,-1605,-88,1558,-1605,-88,1558,-1701,-88,1942,-1701,-88,1950,-1637,-88,1558,-1605,-88,1606,-1653,-268,1598,-1629,-268,1558,-1629,-268,1558,-1693,-268,1590,-1693,-268,1590,-1693,-414,1598,-1661,-414,159`
+`8,-1645,-414,1558,-1637,-414,1558,-1685,-414,1590,-1693,-340,1598,-1661,-340,1598,-1645,-340,1558,-1637,-340,1558,-1685,-340,1630,-1229,-399,1630,-1209,-395,1630,-1189,-416,1558,-1189,-416,1558,-1534.88232421875,-416,1558,-1581,-408,1567.5999755859375,-1534.066650390625,-416,1625.199951171875,-1252.4666748046875,-416,1742,-1629,-416,1630,-1629,-416,1614,-1701,-416,2030,-1701,-416,2030,-1629,-416,1910,-1621,-416,1742,-1629,-416,1614,-1701,-416,2006,-1189,-408,1961,-1189,-414,1826,-1189,-401,1646,`
+`-1189,-400,1646,-1229,-407,1803.5,-1211.5,-402,1961,-1194,-414,1646,-1229,-407,1630,-1229,-399,1625.199951171875,-1252.4666748046875,-416,1567.5999755859375,-1534.066650390625,-416,1558,-1581,-408,1569,-1537,-416,1635,-1273,-416,1640.5,-1251,-406,1750,-1589,-416,1742,-1629,-416,1910,-1621,-416,1918,-1581,-408,1897,-1582,-416,1558,-1581,-408,1707.3333740234375,-1587.22216796875,-408,1750,-1589,-416,1897,-1582,-416,1918,-1581,-408,1895.5,-1581,-416,1918,-1581,-408,2006,-1581,-408,2006,-1189,-408,1`
+`961,-1194,-414,1803.5,-1211.5,-402,1646,-1229,-407,1640.5,-1251,-406,1635,-1273,-416,1569,-1537,-416,1558,-1581,-408,1715.5,-1581,-408,1738,-1581,-416,1895.5,-1581,-416,1618,-1545,-408,1834,-1569,-416,1642,-1257,-416,1978,-1473,-416,1882,-1257,-416,1570,-1545,-408,1714,-1569,-408,1630,-1573,-768,1662,-1573,-768,1678,-1557,-768,1606,-1557,-768,1558,-1557,-768,1598,-1573,-768,1606,-1557,-768,1558,-1189,-768,1558,-1557,-768,1606,-1557,-768,1678,-1557,-768,1678,-1557,-768,1678,-1701,-768,2046,-1701,`
+`-768,2046,-1197,-768,2022,-1189,-768,1558,-1189,-768,1678,-1557,-768,2046,-1701,-768,1878,-1229,-640,1862,-1197,-640,1838,-1189,-640,1558,-1189,-640,1558,-1229,-640,1630,-1237,-640,1662,-1237,-640,1878,-1229,-640,1838,-1189,-640,1558,-1325,-640,1558,-1373,-640,1630,-1373,-640,1630,-1317,-640,1662,-1237,-640,1630,-1237,-640,1630,-1317,-640,1630,-1373,-640,1630,-1701,-640,1662,-1701,-640,1630,-1645,-168,1630,-1629,-168,1598,-1629,-168,1598,-1653,-168,1694,-1205,-480,1678,-1189,-480,1662,-1189,-480`
+`,1726,-1189,-480,1702,-1189,-480,1694,-1205,-480,1726,-1213,-480,1758,-1237,-480,1750,-1213,-480,1726,-1213,-480,1758,-1237,-480,1726,-1213,-480,1694,-1205,-480,1846,-1701,-480,1846,-1245,-480,1758,-1237,-480,1758,-1237,-480,1694,-1205,-480,1662,-1189,-480,1662,-1701,-480,1846,-1701,-480,1662,-1629,-168,1662,-1701,-168,1702,-1701,-168,1702,-1629,-168,2070,-1237,-60,2054,-1221,-60,2030,-1237,-60,2070,-1581,-60,2006,-1213,-60,2014,-1189,-60,1686,-1189,-60,2030,-1237,-60,2006,-1213,-60,1686,-1189,-`
+`60,1686,-1581,-60,2070,-1581,-60,1734,-1629,-168,1734,-1669,-168,1774,-1669,-168,1774,-1629,-168,1806,-1629,-168,1806,-1669,-168,1838,-1669,-168,1838,-1629,-168,1878,-1189,-640,1862,-1197,-640,1878,-1229,-640,2014,-1237,-640,2022,-1189,-640,2046,-1197,-640,2022,-1189,-640,2014,-1237,-640,2014,-1701,-640,2046,-1701,-640,1870,-1629,-168,1870,-1669,-168,1910,-1669,-168,1910,-1629,-168,2046,-1589,-480,2038,-1589,-480,2006,-1621,-480,2054,-1701,-480,2054,-1701,-480,2006,-1621,-480,1982,-1589,-480,187`
+`8,-1573,-480,1878,-1701,-480,2006,-1213,-480,2014,-1189,-480,1990,-1189,-480,1974,-1253,-480,2046,-1229,-480,2006,-1213,-480,1974,-1253,-480,1878,-1573,-480,1982,-1589,-480,2046,-1589,-480,1974,-1253,-480,1878,-1237,-481,1878,-1573,-480,1942,-1669,-168,1974,-1629,-168,1942,-1629,-168,1958,-1653,-20,1958,-1701,-20,2046,-1701,-20,2046,-1653,-20,1966,-1661,-88,1966,-1701,-88,2038,-1701,-88,2038,-1661,-88,2046,-1629,-168,2006,-1629,-168,2006,-1645,-168,2046,-1653,-168,2046,-1581,-408,2070,-1581,-408`
+`,2070,-1277,-408,2046,-1261,-408,2046,-1245,-352,2070,-1261,-352,2070,-1237,-352,2046,-1221,-352,2054,-1685,-268,2070,-1685,-268,2070,-1629,-268,2054,-1629,-268],"vertslength":256,"tris":[0,1,2,4,0,2,2,3,4,5,6,7,5,7,8,5,8,9,13,10,11,11,12,13,14,15,16,17,18,19,17,19,20,21,22,23,21,23,24,25,26,27,25,27,28,29,30,31,32,33,29,29,31,32,34,35,36,38,34,36,36,37,38,39,40,41,43,39,41,41,42,43,51,44,45,48,49,50,51,45,46,51,46,47,48,50,51,47,48,51,52,53,54,55,56,57,55,57,58,55,58,59,66,60,61,66,61,62,65,66,`
+`62,63,64,65,62,63,65,74,67,68,74,68,69,73,74,69,70,71,72,70,72,73,69,70,73,77,78,79,79,75,76,76,77,79,83,84,85,82,83,85,81,82,85,80,81,85,96,95,99,94,93,99,98,97,100,92,93,101,93,90,101,90,91,101,92,91,101,87,88,102,87,86,102,100,98,102,86,98,102,89,90,103,102,100,103,90,93,103,102,88,103,89,88,103,94,95,104,95,99,104,99,94,104,97,100,105,99,96,105,97,96,105,99,93,105,100,103,105,93,103,105,106,107,108,106,108,109,110,111,112,113,114,115,113,115,116,117,118,119,120,121,122,120,122,123,120,123,12`
+`4,125,126,127,128,129,130,128,130,131,131,132,133,128,131,133,134,135,136,134,136,137,138,139,140,138,140,141,141,142,143,138,141,143,144,145,146,144,146,147,148,149,150,151,152,153,151,153,154,155,156,157,158,159,160,161,162,163,165,166,167,164,165,167,164,167,168,172,169,170,170,171,172,173,174,175,173,175,176,177,178,179,180,181,182,183,184,180,180,182,183,188,185,186,186,187,188,192,189,190,190,191,192,193,194,195,193,195,196,193,196,197,198,199,200,200,201,202,198,200,202,206,203,204,204,20`
+`5,206,207,208,209,207,209,210,211,212,213,213,214,215,211,213,215,216,217,218,216,218,219,220,221,222,223,224,225,225,220,222,222,223,225,226,227,228,229,230,231,235,232,233,233,234,235,239,236,237,237,238,239,240,241,242,240,242,243,244,245,246,244,246,247,248,249,250,248,250,251,255,252,253,253,254,255],"trislength":153,"triTopoly":[0,0,0,1,1,1,2,2,3,4,4,5,5,6,6,7,7,7,8,8,8,9,9,9,10,10,10,10,10,10,11,12,12,12,13,13,13,13,13,14,14,14,14,14,14,15,15,15,16,16,16,16,17,17,17,17,17,17,17,17,17,17,1`
+`7,17,17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,19,20,20,21,22,22,22,23,24,24,24,24,25,25,26,26,26,26,27,27,28,29,29,30,31,32,33,33,33,34,34,35,35,36,37,37,37,38,38,39,39,40,40,40,41,41,41,42,42,43,43,44,44,44,45,45,46,46,46,46,47,48,49,49,50,50,51,51,52,52,53,53,54,54],"baseVert":[0,5,10,14,17,21,25,29,34,39,44,52,55,60,67,75,80,86,106,110,113,117,120,125,128,134,138,144,148,151,155,158,161,164,169,173,177,180,185,189,193,198,203,207,211,216,220,226,229,232,236,240,244,248,252],"vertsCount":[`
+`5,5,4,3,4,4,4,5,5,5,8,3,5,7,8,5,6,20,4,3,4,3,5,3,6,4,6,4,3,4,3,3,3,5,4,4,3,5,4,4,5,5,4,4,5,4,6,3,3,4,4,4,4,4,4],"baseTri":[0,3,6,8,9,11,13,15,18,21,24,30,31,34,39,45,48,52,77,79,80,82,83,86,87,91,93,97,99,100,102,103,104,105,108,110,112,113,116,118,120,123,126,128,130,133,135,139,140,141,143,145,147,149,151],"triCount":[3,3,2,1,2,2,2,3,3,3,6,1,3,5,6,3,4,25,2,1,2,1,3,1,4,2,4,2,1,2,1,1,1,3,2,2,1,3,2,2,3,3,2,2,3,2,4,1,1,2,2,2,2,2,2]},"links":{"poly":[2,33,2,28,27,34,32,44,34,38,38,39,39,42,42,48,47`
+`,32,48,51,52,53],"cost":[1536,1536,1536,1536,1536,1536,1536,1536,1537.429443359375,1536,4969.84619140625],"type":[1,1,1,1,1,1,1,1,1,1,2],"pos":[1630,-1701,-480,1662,-1701,-480,1630,-1189,-480,1662,-1189,-480,1630,-1645,-168,1662,-1645,-168,1846,-1701,-480,1878,-1701,-480,1702,-1669,-168,1734,-1669,-168,1774,-1669,-168,1806,-1669,-168,1838,-1669,-168,1870,-1669,-168,1910,-1669,-168,1942,-1669,-168,1878,-1245,-480.9761962890625,1846,-1245,-480,1974,-1629,-168,2006,-1629,-168,2062.615478515625,-127`
+`2.076904296875,-408,2070,-1261,-352],"length":11}}],["10_2",{"tileId":"10_2","tx":10,"ty":2,"mesh":{"verts":[2070,-1605,-88,2070,-1701,-88,2134,-1701,-88,2134,-1605,-88,2070,-1629,-268,2070,-1685,-268,2118,-1685,-268,2118,-1629,-268,2070,-1629,-414,2070,-1677,-414,2102,-1677,-414,2102,-1629,-414,2070,-1629,-340,2070,-1677,-340,2102,-1677,-340,2102,-1629,-340,2158,-1701,-408,2206,-1701,-408,2214,-1637,-408,2158,-1589,-408,2158,-1589,-408,2214,-1637,-408,2294,-1637,-408,2294,-1637,-408,2302,-1701,`
+`-408,2478,-1701,-408,2158,-1589,-408,2294,-1637,-408,2478,-1701,-408,2486,-1397,-408,2238,-1381,-408,2070,-1285,-408,2070,-1581,-408,2158,-1589,-408,2238,-1381,-408,2102,-1261,-60,2094,-1237,-60,2070,-1245,-60,2430,-1349,-60,2398,-1365,-60,2390,-1397,-60,2502,-1333,-60,2190,-1317,-60,2102,-1261,-60,2070,-1245,-60,2070,-1581,-60,2158,-1589,-60,2358,-1397,-60,2350,-1365,-60,2190,-1317,-60,2158,-1589,-60,2390,-1397,-60,2358,-1397,-60,2158,-1589,-60,2390,-1397,-60,2158,-1589,-60,2158,-1701,-60,2502,`
+`-1701,-60,2502,-1333,-60,2118,-1293,-352,2070,-1245,-352,2070,-1269,-352,2214,-1357,-352,2350,-1381,-352,2222,-1349,-352,2214,-1357,-352,2222,-1349,-352,2118,-1293,-352,2070,-1269,-352,2078,-1629,-168,2078,-1701,-168,2118,-1701,-168,2118,-1629,-168,2102,-1189,-60,2086,-1189,-60,2086,-1205,-60,2134,-1189,-60,2110,-1213,-60,2134,-1253,-60,2294,-1325,-60,2318,-1325,-60,2334,-1189,-60,2134,-1189,-352,2118,-1213,-352,2166,-1269,-352,2310,-1325,-352,2334,-1189,-352,2222,-1653,-314,2222,-1701,-314,2286`
+`,-1701,-314,2286,-1653,-314,2350,-1229,-352,2350,-1269,-352,2366,-1261,-352,2374,-1285,-352,2350,-1285,-352,2342,-1317,-352,2398,-1309,-352,2398,-1253,-352,2350,-1229,-352,2366,-1261,-352,2398,-1253,-352,2366,-1261,-352,2374,-1285,-352,2398,-1309,-352,2390,-1213,-60,2390,-1189,-60,2358,-1189,-60,2358,-1213,-60,2438,-1309,-352,2486,-1309,-352,2486,-1189,-352,2414,-1189,-352,2430,-1325,-60,2502,-1309,-60,2502,-1189,-60,2414,-1189,-60,2422,-1341,-408,2430,-1365,-408,2470,-1365,-408,2486,-1341,-408,`
+`2430,-1373,-352,2478,-1373,-352,2486,-1357,-352,2518,-1701,-168,2534,-1701,-168,2542,-1685,-168,2526,-1653,-168,2542,-1685,-168,2582,-1685,-168,2582,-1661,-168,2526,-1653,-168,2518,-1701,-168,2526,-1653,-168,2526,-1501,-168,2518,-1325,-168,2582,-1493,-168,2582,-1301,-168,2518,-1325,-168,2526,-1501,-168,2550,-1645,-88,2582,-1645,-88,2582,-1509,-88,2550,-1509,-88,2558,-1637,-168,2582,-1637,-168,2582,-1517,-168,2558,-1517,-168],"vertslength":148,"polys":[0,3,4,7,8,11,12,15,16,19,20,22,23,25,26,30,3`
+`1,34,35,37,38,41,42,46,47,50,51,53,54,58,59,61,62,64,65,68,69,72,73,75,76,81,82,86,87,90,91,93,94,97,98,100,101,104,105,108,109,112,113,116,117,120,121,123,124,127,128,131,132,135,136,139,140,143,144,147],"polyslength":38,"regions":[7,9,12,13,2,2,2,2,2,1,1,1,1,1,1,17,17,17,11,19,3,4,10,14,14,14,14,32,6,5,15,37,8,8,8,8,16,40],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,5],[0]],[[1,4],[0],[1,7]],[[0],[0],[1,7]],[[1,5],[1,6],[0],[0],[1,8]],[[0],[`
+`0],[1,7],[0]],[[0],[0],[1,11]],[[0],[0],[1,14],[0]],[[0],[1,9],[0],[0],[1,12]],[[0],[0],[1,11],[1,13]],[[0],[1,12],[1,14]],[[1,13],[0],[0],[0],[1,10]],[[0],[0],[1,17]],[[0],[0],[1,17]],[[1,16],[0],[1,15],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,25]],[[0],[0],[0],[1,26]],[[0],[1,23],[1,26]],[[1,25],[0],[1,24],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,33],[1,34]],[`
+`[0],[0],[0],[1,32]],[[1,32],[0],[1,35],[0]],[[0],[0],[1,34],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2070,-1605,-88,2070,-1701,-88,2134,-1701,-88,2134,-1605,-88,2070,-1629,-268,2070,-1685,-268,2118,-1685,-268,2118,-1629,-268,2070,-1629,-414,2070,-1677,-414,2102,-1677,-414,2102,-1629,-414,2070,-1629,-340,2070,-1677,-340,2102,-1677,-340,2102,-1629,-340,2158,-1701,-408,2206,-1701,-408,2214,-1637,-408,2158,-1589,-408,2158,-1589,-408,2214,-1637,-408,2294,-1637,-408,2294,-1637,-40`
+`8,2302,-1701,-408,2478,-1701,-408,2158,-1589,-408,2294,-1637,-408,2478,-1701,-408,2486,-1397,-408,2238,-1381,-408,2070,-1285,-408,2070,-1581,-408,2158,-1589,-408,2238,-1381,-408,2102,-1261,-60,2094,-1237,-60,2070,-1245,-60,2430,-1349,-60,2398,-1365,-60,2390,-1397,-60,2502,-1333,-60,2190,-1317,-60,2102,-1261,-60,2070,-1245,-60,2070,-1581,-60,2158,-1589,-60,2358,-1397,-60,2350,-1365,-60,2190,-1317,-60,2158,-1589,-60,2390,-1397,-60,2358,-1397,-60,2158,-1589,-60,2390,-1397,-60,2158,-1589,-60,2158,-1`
+`701,-60,2502,-1701,-60,2502,-1333,-60,2118,-1293,-352,2070,-1245,-352,2070,-1269,-352,2214,-1357,-352,2350,-1381,-352,2222,-1349,-352,2214,-1357,-352,2222,-1349,-352,2118,-1293,-352,2070,-1269,-352,2078,-1629,-168,2078,-1701,-168,2118,-1701,-168,2118,-1629,-168,2102,-1189,-60,2086,-1189,-60,2086,-1205,-60,2134,-1189,-60,2110,-1213,-60,2134,-1253,-60,2294,-1325,-60,2318,-1325,-60,2334,-1189,-60,2134,-1189,-352,2118,-1213,-352,2166,-1269,-352,2310,-1325,-352,2334,-1189,-352,2222,-1653,-314,2222,-1`
+`701,-314,2286,-1701,-314,2286,-1653,-314,2350,-1229,-352,2350,-1269,-352,2366,-1261,-352,2374,-1285,-352,2350,-1285,-352,2342,-1317,-352,2398,-1309,-352,2398,-1253,-352,2350,-1229,-352,2366,-1261,-352,2398,-1253,-352,2366,-1261,-352,2374,-1285,-352,2398,-1309,-352,2390,-1213,-60,2390,-1189,-60,2358,-1189,-60,2358,-1213,-60,2438,-1309,-352,2486,-1309,-352,2486,-1189,-352,2414,-1189,-352,2430,-1325,-60,2502,-1309,-60,2502,-1189,-60,2414,-1189,-60,2422,-1341,-408,2430,-1365,-408,2470,-1365,-408,248`
+`6,-1341,-408,2430,-1373,-352,2478,-1373,-352,2486,-1357,-352,2518,-1701,-168,2534,-1701,-168,2542,-1685,-168,2526,-1653,-168,2542,-1685,-168,2582,-1685,-168,2582,-1661,-168,2526,-1653,-168,2518,-1701,-168,2526,-1653,-168,2526,-1501,-168,2518,-1325,-168,2582,-1493,-168,2582,-1301,-168,2518,-1325,-168,2526,-1501,-168,2550,-1645,-88,2582,-1645,-88,2582,-1509,-88,2550,-1509,-88,2558,-1637,-168,2582,-1637,-168,2582,-1517,-168,2558,-1517,-168],"vertslength":148,"tris":[3,0,1,1,2,3,7,4,5,5,6,7,11,8,9,9`
+`,10,11,15,12,13,13,14,15,16,17,18,16,18,19,20,21,22,23,24,25,30,26,27,27,28,29,27,29,30,32,33,34,31,32,34,35,36,37,38,39,40,38,40,41,42,43,44,45,46,42,42,44,45,47,48,49,47,49,50,51,52,53,54,55,56,57,58,54,54,56,57,59,60,61,62,63,64,65,66,67,65,67,68,72,69,70,70,71,72,73,74,75,76,77,78,79,80,81,76,78,79,76,79,81,82,83,84,82,84,85,82,85,86,90,87,88,88,89,90,91,92,93,94,95,96,94,96,97,98,99,100,101,102,103,101,103,104,108,105,106,106,107,108,109,110,111,109,111,112,113,114,115,113,115,116,117,118,1`
+`19,117,119,120,121,122,123,124,125,126,124,126,127,128,129,130,128,130,131,132,133,134,132,134,135,138,139,136,136,137,138,143,140,141,141,142,143,147,144,145,145,146,147],"trislength":72,"triTopoly":[0,0,1,1,2,2,3,3,4,4,5,6,7,7,7,8,8,9,10,10,11,11,11,12,12,13,14,14,14,15,16,17,17,18,18,19,20,20,20,20,21,21,21,22,22,23,24,24,25,26,26,27,27,28,28,29,29,30,30,31,32,32,33,33,34,34,35,35,36,36,37,37],"baseVert":[0,4,8,12,16,20,23,26,31,35,38,42,47,51,54,59,62,65,69,73,76,82,87,91,94,98,101,105,109,1`
+`13,117,121,124,128,132,136,140,144],"vertsCount":[4,4,4,4,4,3,3,5,4,3,4,5,4,3,5,3,3,4,4,3,6,5,4,3,4,3,4,4,4,4,4,3,4,4,4,4,4,4],"baseTri":[0,2,4,6,8,10,11,12,15,17,18,20,23,25,26,29,30,31,33,35,36,40,43,45,46,48,49,51,53,55,57,59,60,62,64,66,68,70],"triCount":[2,2,2,2,2,1,1,3,2,1,2,3,2,1,3,1,1,2,2,1,4,3,2,1,2,1,2,2,2,2,2,1,2,2,2,2,2,2]},"links":{"poly":[8,17],"cost":[4823.630859375],"type":[2],"pos":[2209.5693359375,-1364.75390625,-408,2214,-1357,-352],"length":1}}],["11_2",{"tileId":"11_2","tx":`
+`11,"ty":2,"mesh":{"verts":[2622,-1685,-168,2630,-1701,-168,2646,-1701,-168,2630,-1653,-168,2582,-1661,-168,2582,-1685,-168,2622,-1685,-168,2630,-1653,-168,2630,-1501,-168,2630,-1653,-168,2646,-1701,-168,2646,-1253,-168,2582,-1293,-168,2582,-1493,-168,2630,-1501,-168,2646,-1253,-168,2582,-1509,-88,2582,-1645,-88,2614,-1645,-88,2614,-1509,-88,2582,-1517,-168,2582,-1637,-168,2598,-1637,-168,2598,-1517,-168,2598,-1189,-352,2582,-1189,-352,2582,-1205,-352,2590,-1237,-352,2614,-1245,-352,2670,-1189,-3`
+`52,2646,-1189,-352,2654,-1205,-352],"vertslength":32,"polys":[0,3,4,7,8,11,12,15,16,19,20,23,24,28,29,31],"polyslength":8,"regions":[1,1,1,1,2,3,5,10],"neighbors":[[[0],[0],[1,2],[1,1]],[[0],[0],[1,0],[0]],[[0],[1,0],[0],[1,3]],[[0],[0],[1,2],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[2622,-1685,-168,2630,-1701,-168,2646,-1701,-168,2630,-1653,-168,2582,-1661,-168,2582,-1685,-168,2622,-1685,-168,2630,-1653,-168,2630,-1501,-168,2630,-1653,-168`
+`,2646,-1701,-168,2646,-1253,-168,2582,-1293,-168,2582,-1493,-168,2630,-1501,-168,2646,-1253,-168,2582,-1509,-88,2582,-1645,-88,2614,-1645,-88,2614,-1509,-88,2582,-1517,-168,2582,-1637,-168,2598,-1637,-168,2598,-1517,-168,2598,-1189,-352,2582,-1189,-352,2582,-1205,-352,2590,-1237,-352,2614,-1245,-352,2670,-1189,-352,2646,-1189,-352,2654,-1205,-352],"vertslength":32,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,19,16,17,17,18,19,23,20,21,21,22,23,24,25,26,24,26,27,24,27,28,29,30`
+`,31],"trislength":16,"triTopoly":[0,0,1,1,2,2,3,3,4,4,5,5,6,6,6,7],"baseVert":[0,4,8,12,16,20,24,29],"vertsCount":[4,4,4,4,4,4,5,3],"baseTri":[0,2,4,6,8,10,12,15],"triCount":[2,2,2,2,2,2,3,1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_2",{"tileId":"12_2","tx":12,"ty":2,"mesh":{"verts":[3542,-1189,-352,3542,-1701,-352,3550,-1701,-352,3550,-1189,-352],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[3542,-1189,`
+`-352,3542,-1701,-352,3550,-1701,-352,3550,-1189,-352],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["0_3",{"tileId":"0_3","tx":0,"ty":3,"mesh":{"verts":[-3026,-677,-172,-3026,-1189,-172,-2538,-1189,-164,-2538,-677,-164,-2538,-677,-400,-2562,-677,-400,-2570,-717,-400,-2578,-885,-404,-2538,-885,-400,-2538,-1189,-416,-2706,-1157,-408,-2714,-1181,-408,-`
+`2730,-1149,-408,-2706,-1157,-408,-2538,-1189,-416,-2538,-925,-416,-2586,-901,-415,-2586,-901,-415,-2578,-885,-404,-2570,-717,-400,-2602,-701,-416,-2602,-701,-416,-2602,-677,-416,-2994,-677,-416,-2994,-1189,-408,-2538,-1189,-416,-2714,-1181,-408,-2994,-1189,-408,-2714,-1181,-408,-2738,-1173,-408,-2962,-1157,-408,-2962,-1157,-408,-2738,-1173,-408,-2730,-1149,-408,-2994,-1157,-408,-2962,-1157,-408,-2730,-1149,-408,-2586,-901,-415,-2602,-701,-416,-2994,-677,-416,-2570,-885,-256,-2538,-885,-256,-2538`
+`,-677,-256,-2570,-677,-256,-2538,-909,-356,-2538,-901,-356,-2562,-901,-356],"vertslength":47,"polys":[0,3,4,8,9,11,12,16,17,20,21,23,24,26,27,30,31,33,34,39,40,43,44,46],"polyslength":12,"regions":[1,2,2,2,2,2,2,2,2,2,3,4],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,4],[0],[0]],[[1,3],[0],[1,6]],[[0],[1,2],[0],[0],[1,9]],[[0],[1,1],[0],[1,9]],[[0],[0],[1,9]],[[0],[1,2],[1,7]],[[1,6],[0],[1,8],[0]],[[1,7],[0],[1,9]],[[0],[1,8],[1,3],[1,4],[1,5],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"ver`
+`ts":[-3026,-677,-172,-3026,-1189,-172,-3002.761962890625,-1189,-164,-2538,-1189,-164,-2538,-677,-164,-3002.761962890625,-677,-164,-2538,-677,-400,-2562,-677,-400,-2570,-717,-400,-2578,-885,-400,-2538,-885,-400,-2538,-1189,-416,-2706,-1157,-416,-2714,-1181,-408,-2670,-1183,-416,-2730,-1149,-416,-2718,-1153,-408,-2706,-1157,-416,-2538,-1189,-416,-2538,-925,-416,-2554,-917,-400,-2586,-901,-404,-2598,-921.6666870117188,-416,-2574,-913,-416,-2586,-901,-404,-2578,-885,-400,-2570,-717,-400,-2586,-709,-`
+`400,-2602,-701,-416,-2602,-701,-416,-2602,-677,-416,-2994,-677,-416,-2994,-1189,-408,-2948.39990234375,-1189,-408,-2925.60009765625,-1189,-416,-2788.800048828125,-1189,-416,-2766,-1189,-408,-2538,-1189,-416,-2670,-1183,-416,-2714,-1181,-408,-2760.666748046875,-1182.3333740234375,-408,-2784,-1183,-416,-2924,-1187,-416,-2947.333251953125,-1187.6666259765625,-408,-2994,-1189,-408,-2947.333251953125,-1187.6666259765625,-408,-2924,-1187,-416,-2784,-1183,-416,-2760.666748046875,-1182.3333740234375,-40`
+`8,-2714,-1181,-408,-2738,-1173,-408,-2782.800048828125,-1169.800048828125,-416,-2962,-1157,-416,-2962,-1157,-416,-2782.800048828125,-1169.800048828125,-416,-2738,-1173,-408,-2730,-1149,-416,-2753.199951171875,-1149.800048828125,-408,-2776.39990234375,-1150.5999755859375,-416,-2994,-1157,-408,-2962,-1157,-416,-2776.39990234375,-1150.5999755859375,-416,-2753.199951171875,-1149.800048828125,-408,-2730,-1149,-416,-2598,-921.6666870117188,-416,-2586,-901,-404,-2602,-701,-416,-2994,-677,-416,-2994,-11`
+`34.142822265625,-416,-2570,-885,-256,-2538,-885,-256,-2538,-677,-256,-2570,-677,-256,-2538,-909,-356,-2538,-901,-356,-2562,-901,-356],"vertslength":76,"tris":[5,0,1,5,1,2,5,2,3,3,4,5,6,7,8,8,9,10,6,8,10,12,13,14,11,12,14,19,20,22,16,17,18,15,16,18,18,19,22,15,18,22,20,21,23,21,22,23,22,20,23,25,26,27,25,27,28,24,25,28,29,30,31,43,32,33,43,33,34,42,43,34,39,40,41,38,39,41,42,34,35,41,42,35,41,35,36,38,41,36,36,37,38,48,49,50,48,50,51,47,48,51,52,44,45,52,45,46,46,47,51,46,51,52,55,56,57,55,57,58,`
+`54,55,58,53,54,58,61,62,63,68,59,60,68,60,61,64,65,66,68,61,63,63,64,66,67,68,63,63,66,67,72,69,70,70,71,72,73,74,75],"trislength":53,"triTopoly":[0,0,0,0,1,1,1,2,2,3,3,3,3,3,3,3,3,4,4,4,5,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,8,8,8,8,9,9,9,9,9,9,9,9,10,10,11],"baseVert":[0,6,11,15,24,29,32,44,53,59,69,73],"vertsCount":[6,5,4,9,5,3,12,9,6,10,4,3],"baseTri":[0,4,7,9,17,20,21,31,38,42,50,52],"triCount":[4,3,2,8,3,1,10,7,4,8,2,1]},"links":{"poly":[1,11],"cost":[3288],"type":[2],"pos":[-2538,-885,-400,-`
+`2538,-901,-356],"length":1}}],["1_3",{"tileId":"1_3","tx":1,"ty":3,"mesh":{"verts":[-2354,-1189,-408,-2466,-1157,-408,-2474,-1181,-408,-2490,-1149,-408,-2466,-1157,-408,-2354,-1189,-408,-2330,-1149,-408,-2538,-1189,-416,-2354,-1189,-408,-2474,-1181,-408,-2538,-1189,-416,-2474,-1181,-408,-2498,-1173,-408,-2538,-1189,-416,-2498,-1173,-408,-2490,-1149,-408,-2538,-917,-416,-2490,-1149,-408,-2330,-1149,-408,-2290,-1133,-416,-2330,-925,-416,-2354,-909,-416,-2538,-917,-416,-2498,-853,-164,-2538,-821,-1`
+`64,-2538,-1189,-164,-2314,-1189,-164,-2314,-845,-164,-2498,-853,-164,-2538,-1189,-164,-2514,-845,-400,-2522,-677,-400,-2538,-677,-400,-2538,-885,-400,-2202,-893,-401,-2026,-885,-400,-2026,-845,-400,-2514,-845,-400,-2306,-885,-400,-2330,-925,-416,-2290,-1133,-416,-2242,-1117,-416,-2162,-925,-416,-2202,-893,-401,-2298,-901,-406,-2242,-1117,-416,-2170,-1133,-416,-2162,-925,-416,-2514,-845,-400,-2538,-885,-400,-2306,-885,-400,-2306,-885,-400,-2298,-901,-406,-2202,-893,-401,-2538,-885,-256,-2522,-885`
+`,-256,-2522,-829,-256,-2538,-821,-256,-2538,-805,-164,-2522,-797,-164,-2538,-781,-164,-2538,-773,-164,-2522,-765,-164,-2538,-741,-164,-2538,-733,-164,-2522,-725,-164,-2538,-709,-164,-2538,-693,-164,-2522,-677,-164,-2538,-677,-164,-2514,-837,-30,-2026,-837,-30,-2026,-677,-36,-2514,-677,-30,-2498,-885,-256,-2026,-885,-256,-2026,-853,-256,-2498,-853,-256,-2298,-1189,104,-2290,-1189,104,-2282,-1157,104,-2298,-1149,104,-2282,-1157,104,-2026,-1157,104,-2026,-1149,104,-2298,-1149,104,-2298,-1133,-140,-`
+`2026,-1133,-140,-2026,-853,-140,-2298,-853,-140,-2274,-1173,88,-2274,-1189,88,-2026,-1189,88,-2026,-1173,88,-2026,-909,-416,-2146,-909,-416,-2162,-925,-416,-2170,-1133,-416,-2026,-1133,-416],"vertslength":99,"polys":[0,2,3,6,7,9,10,12,13,16,17,22,23,25,26,29,30,33,34,38,39,44,45,47,48,50,51,53,54,57,58,60,61,63,64,66,67,69,70,73,74,77,78,81,82,85,86,89,90,93,94,98],"polyslength":26,"regions":[1,1,1,1,1,1,4,4,3,3,3,3,3,3,8,9,10,11,12,6,7,15,15,2,16,5],"neighbors":[[[1,1],[0],[1,2]],[[0],[1,0],[0]`
+`,[1,5]],[[0],[1,0],[1,3]],[[1,2],[0],[1,4]],[[1,3],[0],[1,5],[0]],[[1,1],[0],[1,10],[0],[0],[1,4]],[[0],[0],[1,7]],[[0],[0],[1,6],[0]],[[0],[0],[0],[1,12]],[[0],[0],[0],[1,12],[1,13]],[[1,5],[0],[1,11],[0],[1,13],[0]],[[0],[1,25],[1,10]],[[1,8],[0],[1,9]],[[0],[1,10],[1,9]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,22],[0]],[[0],[0],[0],[1,21]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,11],[0],[0]]]},"detail":{`
+`"verts":[-2354,-1189,-408,-2466,-1157,-408,-2474,-1181,-408,-2490,-1149,-416,-2466,-1157,-408,-2354,-1189,-408,-2330,-1149,-416,-2454,-1153,-408,-2334,-1153,-408,-2538,-1189,-416,-2492,-1189,-408,-2354,-1189,-408,-2474,-1181,-408,-2538,-1189,-416,-2474,-1181,-408,-2498,-1173,-408,-2538,-1189,-416,-2498,-1173,-408,-2490,-1149,-416,-2538,-917,-416,-2490,-1149,-416,-2330,-1149,-416,-2290,-1133,-416,-2330,-925,-416,-2354,-909,-416,-2538,-917,-416,-2498,-853,-164,-2538,-821,-164,-2538,-1189,-164,-231`
+`4,-1189,-164,-2314,-845,-164,-2498,-853,-164,-2538,-1189,-164,-2514,-845,-400,-2522,-677,-400,-2538,-677,-400,-2538,-885,-400,-2202,-893,-400,-2026,-885,-400,-2026,-845,-400,-2514,-845,-400,-2306,-885,-400,-2330,-925,-416,-2290,-1133,-416,-2242,-1117,-416,-2162,-925,-416,-2175.333251953125,-914.3333129882812,-416,-2188.666748046875,-903.6666870117188,-400,-2202,-893,-400,-2298,-901,-401,-2242,-1117,-416,-2170,-1133,-416,-2162,-925,-416,-2514,-845,-400,-2538,-885,-400,-2306,-885,-400,-2306,-885,-`
+`400,-2298,-901,-401,-2202,-893,-400,-2538,-885,-256,-2522,-885,-256,-2522,-829,-256,-2538,-821,-256,-2538,-805,-164,-2522,-797,-164,-2538,-781,-164,-2538,-773,-164,-2522,-765,-164,-2538,-741,-164,-2538,-733,-164,-2522,-725,-164,-2538,-709,-164,-2538,-693,-164,-2522,-677,-164,-2538,-677,-164,-2514,-837,-30,-2026,-837,-30,-2026,-814.1428833007812,-36,-2026,-677,-36,-2514,-677,-36,-2514,-814.1428833007812,-36,-2498,-885,-256,-2026,-885,-256,-2026,-853,-256,-2498,-853,-256,-2298,-1189,104,-2290,-118`
+`9,104,-2282,-1157,104,-2298,-1149,104,-2282,-1157,104,-2026,-1157,104,-2026,-1149,104,-2298,-1149,104,-2298,-1133,-140,-2026,-1133,-140,-2026,-853,-140,-2298,-853,-140,-2274,-1173,88,-2274,-1189,88,-2026,-1189,88,-2026,-1173,88,-2026,-909,-416,-2146,-909,-416,-2162,-925,-416,-2170,-1133,-416,-2026,-1133,-416],"vertslength":106,"tris":[0,1,2,6,3,7,3,4,7,5,4,7,5,6,8,6,7,8,7,5,8,12,9,10,10,11,12,13,14,15,16,17,18,16,18,19,20,21,22,23,24,25,20,22,23,20,23,25,26,27,28,29,30,31,29,31,32,33,34,35,33,35`
+`,36,37,38,39,41,37,39,39,40,41,46,47,48,46,48,49,45,46,49,45,49,42,42,43,44,42,44,45,50,51,52,53,54,55,56,57,58,59,60,61,59,61,62,63,64,65,66,67,68,69,70,71,72,73,74,80,75,76,80,76,77,80,77,78,78,79,80,84,81,82,82,83,84,85,86,87,85,87,88,89,90,91,89,91,92,96,93,94,94,95,96,100,97,98,98,99,100,101,102,103,103,104,105,101,103,105],"trislength":56,"triTopoly":[0,1,1,1,1,1,1,2,2,3,4,4,5,5,5,5,6,7,7,8,8,9,9,9,10,10,10,10,10,10,11,12,13,14,14,15,16,17,18,19,19,19,19,20,20,21,21,22,22,23,23,24,24,25,25`
+`,25],"baseVert":[0,3,9,13,16,20,26,29,33,37,42,50,53,56,59,63,66,69,72,75,81,85,89,93,97,101],"vertsCount":[3,6,4,3,4,6,3,4,4,5,8,3,3,3,4,3,3,3,3,6,4,4,4,4,4,5],"baseTri":[0,1,7,9,10,12,16,17,19,21,24,30,31,32,33,35,36,37,38,39,43,45,47,49,51,53],"triCount":[1,6,2,1,2,4,1,2,2,3,6,1,1,1,2,1,1,1,1,4,2,2,2,2,2,3]},"links":{"poly":[6,15,7,23,15,16,16,17,17,18,21,24],"cost":[384,1248,96,96,384,587.2941284179688],"type":[1,2,1,1,1,2],"pos":[-2538,-821,-164,-2538,-805,-164,-2314,-1133,-164,-2298,-1133,`
+`-140,-2538,-781,-164,-2538,-773,-164,-2538,-741,-164,-2538,-733,-164,-2538,-709,-164,-2538,-693,-164,-2285.294189453125,-1170.176513671875,104,-2274,-1173,88],"length":6}}],["2_3",{"tileId":"2_3","tx":2,"ty":3,"mesh":{"verts":[-2026,-1173,88,-2026,-1189,88,-1914,-1189,88,-1914,-1173,88,-1906,-1157,104,-1898,-1189,104,-1890,-1189,104,-1890,-1149,104,-2026,-1149,104,-2026,-1157,104,-1906,-1157,104,-1890,-1149,104,-1802,-885,-400,-1690,-885,-400,-1730,-845,-400,-2026,-845,-400,-2026,-885,-400,-1906`
+`,-893,-401,-1802,-885,-400,-1730,-845,-400,-1586,-1189,-416,-1594,-1077,-416,-1650,-1077,-416,-1586,-1189,-416,-1650,-1077,-416,-1706,-1037,-416,-1882,-1133,-416,-1874,-1189,-416,-1706,-1037,-416,-1690,-909,-416,-1810,-917,-416,-1930,-925,-416,-2026,-909,-416,-2026,-1133,-416,-1906,-893,-401,-1930,-925,-416,-2026,-1133,-416,-1882,-1133,-416,-1706,-1037,-416,-1810,-917,-416,-1810,-917,-416,-1802,-885,-400,-1906,-893,-401,-1594,-1189,-132,-1602,-1077,-132,-1690,-1053,-132,-1882,-1133,-132,-1874,-1`
+`189,-132,-2026,-1133,-140,-1882,-1133,-132,-1690,-1053,-132,-1690,-845,-140,-2026,-853,-140,-2026,-853,-256,-2026,-885,-256,-1690,-885,-256,-1690,-845,-256,-2026,-677,-36,-2026,-837,-30,-1690,-837,-30,-1690,-677,-36,-1682,-1053,118,-1514,-1053,112,-1514,-677,112,-1682,-677,118,-1514,-1061,-132,-1586,-1061,-132,-1602,-1077,-132,-1594,-1189,-132,-1514,-1189,-132,-1514,-1061,-416,-1578,-1061,-416,-1594,-1077,-416,-1586,-1189,-416,-1514,-1189,-416],"vertslength":75,"polys":[0,3,4,7,8,11,12,14,15,19,`
+`20,22,23,27,28,30,31,33,34,39,40,42,43,47,48,52,53,56,57,60,61,64,65,69,70,74],"polyslength":18,"regions":[8,9,9,2,2,2,2,2,2,2,2,1,1,7,4,3,5,6],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[1,2]],[[0],[0],[1,1],[0]],[[0],[0],[1,4]],[[0],[0],[1,10],[1,3],[0]],[[1,17],[0],[1,6]],[[1,5],[0],[1,9],[0],[0]],[[0],[0],[1,9]],[[0],[0],[1,9]],[[0],[1,8],[0],[1,6],[1,7],[1,10]],[[0],[1,4],[1,9]],[[1,16],[0],[1,12],[0],[0]],[[0],[1,11],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],`
+`[0],[1,11],[0],[0]],[[0],[0],[1,5],[0],[0]]]},"detail":{"verts":[-2026,-1173,88,-2026,-1189,88,-1914,-1189,88,-1914,-1173,88,-1906,-1157,104,-1898,-1189,104,-1890,-1189,104,-1890,-1149,104,-2026,-1149,104,-2026,-1157,104,-1906,-1157,104,-1890,-1149,104,-1802,-885,-400,-1690,-885,-400,-1730,-845,-400,-2026,-845,-400,-2026,-885,-400,-1906,-893,-400,-1802,-885,-400,-1730,-845,-400,-1586,-1189,-416,-1594,-1077,-416,-1650,-1077,-416,-1586,-1189,-416,-1650,-1077,-416,-1706,-1037,-416,-1882,-1133,-416,`
+`-1874,-1189,-416,-1706,-1037,-416,-1690,-909,-416,-1770,-914.3333129882812,-416,-1790,-915.6666870117188,-400,-1810,-917,-411,-1930,-925,-416,-2006.800048828125,-912.2000122070312,-416,-2026,-909,-400,-2026,-931.4000244140625,-416,-2026,-1133,-416,-1906,-893,-400,-1918,-909,-401,-1930,-925,-416,-2026,-1133,-416,-1882,-1133,-416,-1706,-1037,-416,-1810,-917,-411,-1810,-917,-411,-1806,-901,-400,-1802,-885,-400,-1906,-893,-400,-1594,-1189,-132,-1602,-1077,-132,-1668,-1059,-132,-1690,-1053,-140,-1711`
+`.3333740234375,-1061.888916015625,-132,-1882,-1133,-132,-1874,-1189,-132,-2026,-1133,-140,-1902.5714111328125,-1133,-140,-1882,-1133,-132,-1711.3333740234375,-1061.888916015625,-132,-1690,-1053,-140,-1690,-845,-140,-2026,-853,-140,-1726,-1049,-140,-1894,-1121,-132,-2026,-853,-256,-2026,-885,-256,-1690,-885,-256,-1690,-845,-256,-2026,-677,-36,-2026,-814.1428833007812,-36,-2026,-837,-30,-1690,-837,-30,-1690,-814.1428833007812,-36,-1690,-677,-36,-1682,-1053,112,-1514,-1053,112,-1514,-677,112,-1682,`
+`-677,112,-1514,-1061,-132,-1586,-1061,-132,-1602,-1077,-132,-1594,-1189,-132,-1514,-1189,-132,-1514,-1061,-416,-1578,-1061,-416,-1594,-1077,-416,-1586,-1189,-416,-1514,-1189,-416],"vertslength":89,"tris":[3,0,1,1,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,15,16,17,17,18,19,15,17,19,20,21,22,23,24,25,25,26,27,23,25,27,30,31,32,30,32,28,28,29,30,34,35,36,33,34,36,33,36,37,44,38,39,44,39,40,43,44,40,40,41,42,40,42,43,48,45,46,46,47,48,51,52,53,50,51,53,49,50,53,53,54,55,49,53,55,61,62,63,59,60,63,61,6`
+`0,63,59,58,63,56,57,64,57,58,64,63,58,64,63,62,64,56,62,64,65,66,67,65,67,68,70,71,72,70,72,73,74,69,70,70,73,74,78,75,76,76,77,78,79,80,81,81,82,83,79,81,83,84,85,86,86,87,88,84,86,88],"trislength":55,"triTopoly":[0,0,1,1,2,2,3,4,4,4,5,6,6,6,7,7,7,8,8,8,9,9,9,9,9,10,10,11,11,11,11,11,12,12,12,12,12,12,12,12,12,13,13,14,14,14,14,15,15,16,16,16,17,17,17],"baseVert":[0,4,8,12,15,20,23,28,33,38,45,49,56,65,69,75,79,84],"vertsCount":[4,4,4,3,5,3,5,5,5,7,4,7,9,4,6,4,5,5],"baseTri":[0,2,4,6,7,10,11,14`
+`,17,20,25,27,32,41,43,47,49,52],"triCount":[2,2,2,1,3,1,3,3,3,5,2,5,9,2,4,2,3,3]},"links":{"poly":[0,2],"cost":[768],"type":[2],"pos":[-2026,-1173,88,-2026,-1157,104],"length":1}}],["3_3",{"tileId":"3_3","tx":3,"ty":3,"mesh":{"verts":[-1322,-1029,-416,-1354,-1029,-416,-1362,-1061,-416,-1258,-1149,-416,-1242,-1141,-416,-1258,-1189,-416,-1258,-1149,-416,-1362,-1061,-416,-1514,-1061,-416,-1514,-1189,-416,-1322,-869,-132,-1354,-853,-132,-1354,-1053,-132,-1354,-1053,-132,-1514,-1061,-132,-1514,-1189,`
+`-132,-1002,-1189,-132,-1002,-869,-132,-1322,-869,-132,-1354,-1053,-132,-1002,-1189,-132,-1514,-677,112,-1514,-1053,112,-1362,-1053,112,-1362,-677,112,-1346,-957,-400,-1346,-1013,-400,-1314,-1013,-401,-1306,-957,-407,-1298,-941,-416,-1306,-957,-407,-1314,-1013,-401,-1322,-869,-416,-1354,-853,-416,-1354,-933,-416,-1322,-869,-416,-1354,-933,-416,-1298,-941,-416,-1010,-869,-416,-1314,-1013,-401,-1322,-1029,-416,-1242,-1141,-416,-1298,-941,-416,-1314,-1013,-401,-1242,-1141,-416,-1002,-1141,-416,-1002`
+`,-933,-416,-1010,-869,-416,-1354,-829,-52,-1002,-829,-52,-1002,-677,-52,-1354,-677,-52,-1242,-1157,-363,-1242,-1189,-367,-1170,-1189,-367,-1170,-1157,-363,-1154,-1157,-272,-1154,-1189,-272,-1066,-1189,-272,-1066,-1157,-272,-1050,-1157,-304,-1050,-1189,-301,-1002,-1189,-304,-1002,-1157,-304],"vertslength":64,"polys":[0,4,5,9,10,12,13,16,17,20,21,24,25,28,29,31,32,34,35,38,39,41,42,47,48,51,52,55,56,59,60,63],"polyslength":16,"regions":[4,4,1,1,1,3,2,2,2,2,2,2,5,6,7,8],"neighbors":[[[0],[0],[1,1],`
+`[0],[1,10]],[[0],[1,0],[0],[0],[0]],[[0],[0],[1,4]],[[0],[0],[0],[1,4]],[[0],[1,2],[1,3],[0]],[[0],[0],[0],[0]],[[0],[0],[1,7],[0]],[[0],[1,6],[1,11]],[[0],[0],[1,9]],[[1,8],[0],[1,11],[0]],[[0],[1,0],[1,11]],[[1,7],[1,10],[0],[0],[0],[1,9]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-1322,-1029,-416,-1354,-1029,-416,-1362,-1061,-416,-1258,-1149,-416,-1242,-1141,-416,-1258,-1189,-416,-1258,-1149,-416,-1362,-1061,-416,-1514,-1061,-416,-1514,-1189,`
+`-416,-1322,-869,-132,-1354,-853,-132,-1354,-1053,-132,-1354,-1053,-132,-1514,-1061,-132,-1514,-1189,-132,-1002,-1189,-132,-1002,-869,-132,-1322,-869,-132,-1354,-1053,-132,-1002,-1189,-132,-1514,-677,112,-1514,-1053,112,-1362,-1053,112,-1362,-677,112,-1346,-957,-416,-1346,-975.6666870117188,-400,-1346,-1013,-400,-1314,-1013,-412,-1306,-957,-416,-1298,-941,-416,-1306,-957,-416,-1314,-1013,-412,-1322,-869,-416,-1354,-853,-416,-1354,-933,-416,-1322,-869,-416,-1354,-933,-416,-1298,-941,-416,-1010,-86`
+`9,-416,-1314,-1013,-412,-1322,-1029,-407,-1308.6666259765625,-1047.6666259765625,-416,-1242,-1141,-416,-1298,-941,-416,-1314,-1013,-412,-1242,-1141,-416,-1002,-1141,-416,-1002,-933,-416,-1010,-869,-416,-1354,-829,-52,-1002,-829,-52,-1002,-677,-52,-1354,-677,-52,-1242,-1157,-363,-1242,-1189,-367,-1170,-1189,-367,-1170,-1157,-363,-1154,-1157,-272,-1154,-1189,-272,-1066,-1189,-272,-1066,-1157,-272,-1050,-1157,-304,-1050,-1189,-304,-1002,-1189,-304,-1002,-1157,-304],"vertslength":66,"tris":[0,1,2,3,`
+`4,0,0,2,3,5,6,7,7,8,9,5,7,9,10,11,12,13,14,15,13,15,16,17,18,19,17,19,20,24,21,22,22,23,24,29,25,26,26,27,28,26,28,29,30,31,32,33,34,35,36,37,38,36,38,39,40,41,42,40,42,43,44,45,46,47,48,49,44,46,47,44,47,49,53,50,51,51,52,53,57,54,55,55,56,57,61,58,59,59,60,61,65,62,63,63,64,65],"trislength":34,"triTopoly":[0,0,0,1,1,1,2,3,3,4,4,5,5,6,6,6,7,8,9,9,10,10,11,11,11,11,12,12,13,13,14,14,15,15],"baseVert":[0,5,10,13,17,21,25,30,33,36,40,44,50,54,58,62],"vertsCount":[5,5,3,4,4,4,5,3,3,4,4,6,4,4,4,4],"`
+`baseTri":[0,3,6,7,9,11,13,16,17,18,20,22,26,28,30,32],"triCount":[3,3,1,2,2,2,3,1,1,2,2,4,2,2,2,2]},"links":{"poly":[0,13,14,15],"cost":[4520.7001953125,1645.5],"type":[2,2],"pos":[-1248.4000244140625,-1144.199951171875,-416,-1242,-1157,-363,-1066,-1189,-272,-1050,-1189,-301],"length":2}}],["4_3",{"tileId":"4_3","tx":4,"ty":3,"mesh":{"verts":[-1002,-1157,-304,-1002,-1189,-304,-930,-1189,-353,-930,-1157,-353,-922,-853,-132,-946,-845,-132,-962,-869,-132,-930,-901,-132,-906,-893,-140,-906,-893,-140`
+`,-906,-829,-140,-922,-853,-132,-930,-901,-132,-962,-869,-132,-1002,-869,-132,-1002,-1189,-132,-930,-1189,-132,-858,-917,-416,-890,-917,-416,-898,-981,-414,-898,-1141,-416,-890,-1189,-414,-522,-749,-416,-490,-741,-416,-490,-677,-416,-626,-677,-416,-930,-973,-416,-930,-941,-416,-1002,-941,-416,-1002,-1141,-416,-898,-981,-414,-930,-973,-416,-1002,-1141,-416,-898,-1141,-416,-530,-1101,-408,-506,-1077,-416,-522,-749,-416,-626,-677,-416,-858,-917,-416,-890,-1189,-414,-890,-1189,-414,-514,-1189,-416,-5`
+`30,-1173,-408,-890,-1189,-414,-530,-1173,-408,-530,-1101,-408,-1002,-677,-52,-1002,-829,-52,-962,-829,-52,-962,-677,-52,-994,-869,-369,-994,-925,-369,-930,-925,-369,-930,-853,-369,-930,-677,-416,-938,-677,-416,-938,-765,-416,-498,-853,-767,-490,-829,-767,-490,-677,-768,-914,-677,-768,-914,-853,-768,-866,-853,-640,-866,-677,-640,-914,-677,-640,-914,-853,-640,-906,-677,-288,-914,-677,-288,-914,-773,-288,-866,-677,-416,-890,-677,-416,-890,-709,-416,-842,-709,-416,-834,-701,-416,-834,-701,-416,-842,`
+`-709,-416,-858,-757,-416,-818,-677,-416,-834,-701,-416,-858,-757,-416,-890,-885,-416,-682,-677,-416,-858,-757,-416,-890,-749,-416,-890,-885,-416,-874,-717,-368,-874,-741,-368,-858,-741,-368,-858,-717,-368,-514,-1101,-155,-514,-1173,-155,-490,-1173,-155,-490,-757,-155,-506,-757,-155],"vertslength":94,"polys":[0,3,4,8,9,11,12,16,17,21,22,25,26,29,30,33,34,39,40,42,43,45,46,49,50,53,54,56,57,61,62,65,66,68,69,73,74,76,77,81,82,84,85,88,89,93],"polyslength":23,"regions":[8,4,4,4,1,1,1,1,1,1,1,7,5,11`
+`,2,6,12,3,3,3,3,13,15],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,3],[0],[1,2]],[[0],[0],[1,1]],[[1,1],[0],[0],[0],[0]],[[0],[0],[1,7],[0],[1,8]],[[0],[0],[0],[1,8]],[[0],[0],[0],[1,7]],[[0],[1,6],[0],[1,4]],[[0],[0],[1,5],[0],[1,4],[1,10]],[[0],[0],[1,10]],[[1,9],[0],[1,8]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,18],[0]],[[1,17],[0],[1,19]],[[0],[1,18],[1,20],[0],[0]],[[0],[0],[1,19]],[[0],[0],[0],[0]],[[0],[0],[0]`
+`,[0],[0]]]},"detail":{"verts":[-1002,-1157,-306,-1002,-1189,-306,-930,-1189,-353,-930,-1157,-353,-942,-1177,-353,-922,-853,-140,-946,-845,-132,-962,-869,-132,-930,-901,-132,-906,-893,-140,-926,-865,-132,-906,-893,-140,-906,-829,-140,-922,-853,-140,-930,-901,-132,-962,-869,-132,-1002,-869,-132,-1002,-1189,-132,-930,-1189,-132,-858,-917,-416,-890,-917,-416,-898,-981,-414,-898,-1141,-416,-890,-1189,-414,-522,-749,-416,-490,-741,-416,-490,-677,-416,-626,-677,-416,-930,-973,-416,-930,-941,-416,-1002,`
+`-941,-416,-1002,-1141,-416,-898,-981,-414,-930,-973,-416,-1002,-1141,-416,-898,-1141,-416,-530,-1101,-416,-506,-1077,-416,-516.2857055664062,-866.1428833007812,-416,-517.4285888671875,-842.7142944335938,-408,-520.8571166992188,-772.4285888671875,-408,-522,-749,-416,-626,-677,-416,-858,-917,-416,-890,-1189,-414,-566,-841,-408,-890,-1189,-414,-584.5,-1189,-416,-537.5,-1189,-408,-514,-1189,-416,-530,-1173,-408,-575,-1175,-408,-597.5,-1176,-416,-890,-1189,-414,-597.5,-1176,-416,-575,-1175,-408,-530,`
+`-1173,-408,-530,-1119,-408,-530,-1101,-416,-566,-1129,-408,-1002,-677,-52,-1002,-829,-52,-962,-829,-52,-962,-677,-52,-994,-869,-369,-994,-925,-369,-930,-925,-369,-930,-853,-369,-930,-677,-416,-938,-677,-416,-938,-765,-416,-498,-853,-767,-490,-829,-766,-490,-677,-768,-914,-677,-768,-914,-853,-768,-866,-853,-640,-866,-677,-640,-914,-677,-640,-914,-853,-640,-906,-677,-288,-914,-677,-288,-914,-773,-288,-866,-677,-416,-890,-677,-416,-890,-709,-416,-842,-709,-416,-834,-701,-416,-834,-701,-416,-842,-70`
+`9,-416,-858,-757,-416,-818,-677,-416,-834,-701,-416,-858,-757,-416,-890,-885,-416,-682,-677,-416,-858,-757,-416,-890,-749,-416,-890,-885,-416,-874,-717,-368,-874,-741,-368,-858,-741,-368,-858,-717,-368,-514,-1101,-155,-514,-1173,-155,-490,-1173,-156,-490,-757,-155,-506,-757,-155],"vertslength":108,"tris":[1,2,4,2,3,4,3,0,4,1,0,4,7,8,10,5,6,10,7,6,10,8,9,10,5,9,10,11,12,13,14,15,16,17,18,14,14,16,17,19,20,21,21,22,23,19,21,23,24,25,26,24,26,27,28,29,30,28,30,31,35,32,33,33,34,35,43,44,36,36,37,45`
+`,38,37,45,38,39,45,40,39,45,42,41,45,40,41,45,42,43,45,36,43,45,48,49,50,48,50,51,47,48,51,47,51,52,46,47,52,53,54,59,58,53,59,54,55,59,58,57,59,55,56,59,57,56,59,63,60,61,61,62,63,64,65,66,64,66,67,68,69,70,71,72,73,74,75,71,71,73,74,79,76,77,77,78,79,80,81,82,86,87,83,83,84,85,83,85,86,88,89,90,91,92,93,95,91,93,93,94,95,96,97,98,102,99,100,100,101,102,103,104,105,106,107,103,103,105,106],"trislength":66,"triTopoly":[0,0,0,0,1,1,1,1,1,2,3,3,3,4,4,4,5,5,6,6,7,7,8,8,8,8,8,8,8,8,8,9,9,9,9,9,10,10`
+`,10,10,10,10,11,11,12,12,13,14,14,14,15,15,16,17,17,17,18,19,19,19,20,21,21,22,22,22],"baseVert":[0,5,11,14,19,24,28,32,36,46,53,60,64,68,71,76,80,83,88,91,96,99,103],"vertsCount":[5,6,3,5,5,4,4,4,10,7,7,4,4,3,5,4,3,5,3,5,3,4,5],"baseTri":[0,4,9,10,13,16,18,20,22,31,36,42,44,46,47,50,52,53,56,57,60,61,63],"triCount":[4,5,1,3,3,2,2,2,9,5,6,2,2,1,3,2,1,3,1,3,1,2,3]},"links":{"poly":[0,7,6,12,17,21],"cost":[6337.5,3697.5,3552],"type":[2,2,2],"pos":[-930,-1157,-353,-930,-1141,-416,-930,-941,-416,-93`
+`0,-925,-369,-874,-709,-416,-874,-717,-368],"length":3}}],["5_3",{"tileId":"5_3","tx":5,"ty":3,"mesh":{"verts":[-218,-1173,-155,-210,-1189,-155,-202,-1189,-155,-186,-1173,-155,-106,-725,-172,22,-717,-172,22,-677,-172,22,-1173,-156,22,-1069,-172,-106,-1061,-172,-186,-1173,-155,-434,-893,-172,-338,-893,-172,-274,-773,-155,-490,-757,-155,-218,-1173,-155,-186,-1173,-155,-106,-1061,-172,-338,-1053,-172,-434,-1053,-172,-490,-1173,-155,-274,-773,-155,-338,-893,-172,-338,-1053,-172,-106,-1061,-172,-106,-`
+`725,-172,-106,-725,-172,22,-677,-172,-266,-677,-160,-274,-773,-155,-434,-1053,-172,-434,-893,-172,-490,-757,-155,-490,-1173,-155,-466,-773,-768,-466,-813,-768,-418,-861,-768,-290,-1045,-766,-234,-1037,-772,-234,-973,-772,-418,-861,-768,-426,-1029,-768,-490,-677,-768,-490,-813,-768,-466,-773,-768,-122,-725,-772,22,-733,-766,22,-677,-768,-490,-677,-768,-466,-773,-768,-418,-861,-768,-234,-973,-772,-170,-909,-772,-170,-741,-772,-490,-677,-768,-170,-741,-772,-122,-725,-772,22,-677,-768,-346,-765,-415`
+`,-298,-709,-416,-298,-677,-416,-490,-677,-416,-490,-741,-416,-434,-765,-415,-338,-797,-416,-346,-765,-415,-434,-765,-415,-442,-797,-416,-298,-805,-416,-338,-797,-416,-442,-797,-416,-258,-1117,-416,-274,-965,-416,-298,-957,-416,-306,-1117,-416,-474,-1125,-416,-314,-1133,-416,-306,-1117,-416,-298,-805,-416,-442,-797,-416,-482,-805,-416,-474,-1125,-416,-306,-1117,-416,-298,-957,-416,-458,-861,-240,-458,-1061,-240,-442,-1061,-240,-442,-861,-240,-282,-1085,-766,-290,-1045,-766,-426,-1029,-768,-426,-1`
+`189,-768,22,-1189,-768,22,-1077,-766,-282,-1085,-766,-426,-1189,-768,-354,-933,-92,-338,-917,-100,-354,-901,-92,-354,-933,-92,-354,-901,-92,-418,-901,-92,-426,-965,-92,-418,-1053,-92,-354,-1045,-92,-394,-909,-240,-370,-901,-240,-370,-861,-240,-410,-861,-240,-394,-933,-240,-394,-909,-240,-410,-861,-240,-410,-1061,-240,-370,-1061,-240,-370,-941,-240,-394,-933,-240,-410,-1061,-240,-338,-821,-240,-306,-837,-240,-306,-813,-240,22,-1189,-416,22,-1181,-416,-10,-1173,-416,-114,-1181,-416,-170,-1189,-416`
+`,-258,-1117,-416,-242,-1133,-416,-106,-1141,-416,-106,-1141,-416,-114,-1181,-416,-10,-1173,-416,-10,-1141,-416,-106,-1141,-416,-10,-1141,-416,22,-1133,-416,-242,-909,-414,-274,-965,-416,-258,-1117,-416,-106,-1141,-416,22,-1133,-416,22,-845,-415,22,-845,-415,-242,-861,-416,-242,-909,-414,22,-677,-416,6,-677,-416,-10,-693,-416,22,-845,-415,-258,-701,-416,-258,-845,-414,-242,-861,-416,-82,-677,-416,-194,-677,-416,-258,-701,-416,-242,-861,-416,22,-845,-415,-10,-693,-416,22,-901,-87,22,-877,-87,-34,-`
+`877,-87,-42,-909,-87,22,-1053,-92,22,-1013,-96,-26,-1005,-87,-90,-1053,-91,-42,-909,-87,-34,-877,-87,-50,-845,-96,-42,-781,-96,22,-773,-96,22,-733,-90,-90,-733,-90,-50,-973,-96,-42,-909,-87,-50,-845,-96,-90,-733,-90,-90,-1053,-91,-90,-1053,-91,-26,-1005,-87,-50,-973,-96,-50,-845,-96,-42,-781,-96,-90,-733,-90,-34,-997,-176,-2,-997,-176,-2,-981,-176,-34,-949,-176,-26,-877,-176,-2,-877,-176,-2,-861,-176,-18,-981,-22,22,-981,-22,22,-933,-22,-18,-933,-22,22,-805,-22,-18,-805,-22,-18,-845,-22,22,-853,`
+`-22],"vertslength":198,"polys":[0,3,4,6,7,10,11,14,15,20,21,25,26,29,30,33,34,36,37,41,42,44,45,47,48,53,54,57,58,63,64,67,68,70,71,74,75,77,78,83,84,87,88,91,92,95,96,98,99,104,105,108,109,112,113,116,117,119,120,124,125,127,128,131,132,134,135,140,141,143,144,147,148,150,151,156,157,160,161,164,165,167,168,171,172,176,177,179,180,182,183,186,187,189,190,193,194,197],"polyslength":49,"regions":[4,4,4,4,4,4,4,4,3,3,3,3,3,3,7,5,5,5,5,5,21,6,6,8,8,13,13,13,26,1,1,1,1,1,1,2,2,2,9,9,9,9,9,9,9,28,29,`
+`14,15],"neighbors":[[[0],[0],[0],[1,4]],[[0],[0],[1,6]],[[0],[0],[1,4],[0]],[[0],[1,5],[0],[1,7]],[[1,0],[1,2],[1,5],[0],[1,7],[0]],[[1,3],[0],[1,4],[0],[1,6]],[[1,1],[0],[0],[1,5]],[[0],[1,3],[0],[1,4]],[[0],[0],[1,12]],[[0],[0],[1,12],[0],[1,21]],[[0],[0],[1,12]],[[0],[0],[1,13]],[[1,10],[1,8],[1,9],[0],[0],[1,13]],[[1,12],[0],[1,11],[0]],[[0],[0],[0],[0],[0],[1,15]],[[0],[1,14],[0],[1,16]],[[0],[1,15],[1,19]],[[1,33],[0],[1,19],[0]],[[0],[0],[1,19]],[[1,16],[0],[0],[1,18],[1,17],[0]],[[0],[0]`
+`,[0],[0]],[[0],[1,9],[0],[1,22]],[[0],[0],[1,21],[0]],[[0],[0],[1,24]],[[1,23],[0],[0],[0],[0],[0]],[[0],[0],[0],[1,26]],[[0],[1,25],[0],[1,27]],[[0],[0],[1,26],[0]],[[0],[0],[0]],[[0],[0],[1,31],[0],[0]],[[0],[0],[1,33]],[[0],[1,29],[0],[1,32]],[[1,31],[0],[1,33]],[[0],[1,17],[1,30],[1,32],[0],[1,34]],[[1,37],[0],[1,33]],[[0],[0],[1,37],[0]],[[0],[0],[1,37]],[[0],[0],[1,36],[1,34],[1,35],[0]],[[0],[0],[1,40],[0]],[[0],[0],[1,43],[0]],[[1,38],[0],[1,42]],[[0],[0],[0],[1,44]],[[0],[1,40],[1,44],[`
+`0],[1,43]],[[1,39],[0],[1,42]],[[0],[1,41],[1,42]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-218,-1173,-159,-210,-1189,-155,-202,-1189,-155,-186,-1173,-159,-106,-725,-172,22,-717,-172,22,-677,-172,22,-1173,-159,22,-1131.4000244140625,-172,22,-1069,-172,-106,-1061,-172,-159.3333282470703,-1135.6666259765625,-172,-186,-1173,-159,-30,-1137,-172,-434,-893,-172,-338,-893,-172,-306,-833,-172,-274,-773,-160,-490,-757,-155,-474,-795.8571166992188,-160,-466`
+`,-815.2857055664062,-172,-454,-833,-172,-334,-785,-155,-218,-1173,-159,-186,-1173,-159,-159.3333282470703,-1135.6666259765625,-172,-106,-1061,-172,-338,-1053,-172,-434,-1053,-172,-471.3333435058594,-1133,-172,-490,-1173,-159,-430,-1137,-172,-238,-1137,-172,-274,-773,-160,-306,-833,-172,-338,-893,-172,-338,-1053,-172,-106,-1061,-172,-106,-725,-172,-232,-761,-172,-278,-785,-156,-106,-725,-172,22,-677,-172,-243.84616088867188,-677,-172,-266,-677,-165,-274,-773,-160,-232,-761,-172,-434,-1053,-172,-4`
+`34,-893,-172,-466,-815.2857055664062,-172,-474,-795.8571166992188,-160,-490,-757,-155,-490,-826.3333129882812,-165,-490,-1149.888916015625,-165,-490,-1173,-159,-471.3333435058594,-1133,-172,-466,-773,-768,-466,-813,-768,-418,-861,-768,-290,-1045,-772,-234,-1037,-772,-234,-973,-772,-418,-861,-768,-426,-1029,-768,-294,-1033,-766,-490,-677,-768,-490,-813,-768,-466,-773,-768,-122,-725,-772,1.4285714626312256,-731.8571166992188,-772,22,-733,-766,22,-677,-768,-490,-677,-768,-466,-773,-768,-418,-861,-7`
+`68,-234,-973,-772,-170,-909,-772,-170,-741,-772,-490,-677,-768,-170,-741,-772,-122,-725,-772,22,-677,-768,-346,-765,-416,-298,-709,-416,-298,-677,-416,-490,-677,-416,-490,-741,-416,-434,-765,-416,-338,-797,-416,-346,-765,-415,-434,-765,-415,-442,-797,-416,-298,-805,-416,-338,-797,-416,-442,-797,-416,-258,-1117,-416,-274,-965,-416,-298,-957,-416,-306,-1117,-416,-474,-1125,-416,-314,-1133,-416,-306,-1117,-416,-298,-805,-416,-442,-797,-416,-482,-805,-416,-474,-1125,-416,-306,-1117,-416,-298,-957,-4`
+`16,-458,-861,-240,-458,-1061,-240,-442,-1061,-240,-442,-861,-240,-282,-1085,-766,-290,-1045,-766,-426,-1029,-768,-426,-1189,-768,22,-1189,-768,22,-1077,-766,-282,-1085,-766,-426,-1189,-768,-354,-933,-100,-338,-917,-100,-354,-901,-92,-354,-933,-100,-354,-901,-92,-418,-901,-92,-426,-965,-92,-418,-1053,-92,-354,-1045,-92,-354,-955.4000244140625,-92,-394,-909,-240,-370,-901,-240,-370,-861,-240,-410,-861,-240,-394,-933,-240,-394,-909,-240,-410,-861,-240,-410,-1061,-240,-370,-1061,-240,-370,-941,-240,`
+`-394,-933,-240,-410,-1061,-240,-338,-821,-240,-306,-837,-240,-306,-813,-240,22,-1189,-416,22,-1181,-416,-10,-1173,-415,-114,-1181,-416,-170,-1189,-416,-258,-1117,-416,-242,-1133,-416,-106,-1141,-416,-106,-1141,-416,-114,-1181,-416,-10,-1173,-415,-10,-1141,-416,-106,-1141,-416,-10,-1141,-416,22,-1133,-416,-242,-909,-416,-274,-965,-414,-258,-1117,-416,-106,-1141,-416,22,-1133,-416,22,-845,-416,22,-845,-416,-242,-861,-416,-242,-909,-416,22,-677,-416,6,-677,-416,-10,-693,-416,22,-845,-415,-258,-701,`
+`-416,-258,-845,-414,-242,-861,-416,-82,-677,-416,-194,-677,-416,-258,-701,-416,-242,-861,-416,22,-845,-415,-10,-693,-416,22,-901,-87,22,-877,-87,-34,-877,-87,-42,-909,-87,22,-1053,-92,22,-1013,-96,-26,-1005,-96,-90,-1053,-91,-42,-909,-87,-34,-877,-87,-50,-845,-96,-42,-781,-96,22,-773,-96,22,-733,-90,-90,-733,-90,-50,-973,-96,-42,-909,-87,-50,-845,-96,-90,-733,-90,-90,-778.7142944335938,-96,-90,-1053,-91,-90,-1053,-91,-26,-1005,-96,-50,-973,-96,-50,-845,-96,-44.66666793823242,-802.3333129882812,-`
+`87,-42,-781,-96,-90,-733,-90,-34,-997,-176,-2,-997,-176,-2,-981,-176,-34,-949,-176,-26,-877,-176,-2,-877,-176,-2,-861,-176,-18,-981,-22,22,-981,-22,22,-933,-22,-18,-933,-22,22,-805,-22,-18,-805,-22,-18,-845,-22,22,-853,-22],"vertslength":225,"tris":[0,1,2,0,2,3,4,5,6,10,11,13,7,8,13,10,9,13,8,9,13,11,12,13,7,12,13,14,15,21,14,20,21,17,18,22,18,19,22,21,20,22,19,20,22,17,16,22,21,15,22,16,15,22,30,23,31,30,29,31,27,28,31,29,28,31,26,27,32,26,25,32,23,24,32,25,24,32,27,31,32,23,31,32,39,34,35,38,3`
+`9,35,35,36,37,35,37,38,39,33,40,33,34,40,34,39,40,43,44,45,43,45,46,43,46,41,41,42,43,53,54,55,50,51,52,49,50,52,48,49,52,53,55,47,47,48,52,47,52,53,56,57,58,63,59,64,59,60,64,61,60,64,61,62,64,63,62,64,65,66,67,69,70,71,68,69,71,72,73,74,75,76,77,74,75,77,72,74,77,78,79,80,78,80,81,82,83,84,85,86,87,87,82,84,84,85,87,88,89,90,88,90,91,92,93,94,96,97,98,95,96,98,99,100,101,102,103,104,105,106,107,107,102,104,104,105,107,111,108,109,109,110,111,112,113,114,112,114,115,116,117,118,116,118,119,120,`
+`121,122,123,124,125,129,123,125,129,125,126,126,127,128,126,128,129,130,131,132,130,132,133,134,135,136,134,136,137,138,139,140,138,140,141,142,143,144,145,146,147,145,147,148,145,148,149,150,151,152,155,156,153,153,154,155,157,158,159,160,161,162,162,163,164,160,162,164,160,164,165,166,167,168,169,170,171,169,171,172,173,174,175,176,177,178,180,181,176,176,178,179,176,179,180,182,183,184,182,184,185,186,187,188,186,188,189,190,191,192,193,194,195,193,195,196,199,200,201,197,198,199,197,199,201,`
+`197,201,202,203,204,205,206,207,208,206,208,209,210,211,212,210,212,213,214,215,216,220,217,218,218,219,220,221,222,223,221,223,224],"trislength":134,"triTopoly":[0,0,1,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,6,6,6,6,7,7,7,7,7,7,7,8,9,9,9,9,9,10,11,11,12,12,12,12,13,13,14,14,14,14,15,15,16,17,17,18,19,19,19,19,20,20,21,21,22,22,23,24,24,24,24,24,25,25,26,26,27,27,28,29,29,29,30,31,31,32,33,33,33,33,34,35,35,36,37,37,37,37,38,38,39,39,40,41,41,42,42,42,42,43,44,44,45,45,46`
+`,47,47,48,48],"baseVert":[0,4,7,14,23,33,41,47,56,59,65,68,72,78,82,88,92,95,99,102,108,112,116,120,123,130,134,138,142,145,150,153,157,160,166,169,173,176,182,186,190,193,197,203,206,210,214,217,221],"vertsCount":[4,3,7,9,10,8,6,9,3,6,3,4,6,4,6,4,3,4,3,6,4,4,4,3,7,4,4,4,3,5,3,4,3,6,3,4,3,6,4,4,3,4,6,3,4,4,3,4,4],"baseTri":[0,2,3,9,18,28,35,39,46,47,52,53,55,59,61,65,67,68,70,71,75,77,79,81,82,87,89,91,93,94,97,98,100,101,105,106,108,109,113,115,117,118,120,124,125,127,129,130,132],"triCount":[2`
+`,1,6,9,10,7,4,7,1,5,1,2,4,2,4,2,1,2,1,4,2,2,2,1,5,2,2,2,1,3,1,2,1,4,1,2,1,4,2,2,1,2,4,1,2,2,1,2,2]},"links":{"poly":[20,26],"cost":[1536],"type":[1],"pos":[-442,-1061,-240,-410,-1061,-240],"length":1}}],["6_3",{"tileId":"6_3","tx":6,"ty":3,"mesh":{"verts":[70,-1085,-766,62,-957,-766,22,-957,-766,22,-1189,-768,246,-1189,-768,246,-1085,-766,70,-1085,-766,22,-1189,-768,22,-1189,-416,46,-1189,-416,22,-1181,-416,22,-1069,-172,22,-1173,-156,246,-1173,-156,246,-1069,-172,222,-677,-416,174,-677,-416,166`
+`,-693,-416,222,-829,-416,246,-813,-416,246,-701,-416,150,-957,-416,158,-973,-416,246,-957,-416,246,-877,-416,222,-861,-416,166,-693,-416,94,-677,-416,22,-677,-416,150,-957,-416,222,-861,-416,222,-829,-416,166,-693,-416,22,-677,-416,118,-965,-416,22,-1133,-416,118,-1133,-416,118,-965,-416,22,-677,-416,54,-989,-87,22,-1005,-96,22,-1053,-92,158,-933,-96,62,-941,-96,54,-989,-87,246,-1053,-91,246,-941,-96,158,-933,-96,54,-989,-87,22,-1053,-92,22,-877,-87,22,-901,-87,46,-901,-87,54,-869,-87,54,-869,-8`
+`7,46,-901,-87,62,-941,-96,62,-813,-96,54,-869,-87,62,-941,-96,158,-933,-96,158,-813,-96,22,-821,-766,62,-821,-766,70,-781,-766,22,-677,-768,198,-749,-772,246,-741,-772,246,-677,-768,70,-781,-766,190,-781,-772,198,-749,-772,70,-781,-766,198,-749,-772,246,-677,-768,22,-677,-768,22,-733,-90,22,-773,-96,46,-773,-96,158,-733,-90,158,-813,-96,174,-789,-96,158,-733,-90,46,-773,-96,62,-813,-96,166,-717,-101,158,-733,-90,174,-789,-96,230,-805,-96,246,-677,-92,22,-677,-172,22,-717,-172,166,-717,-101,246,-`
+`677,-92,238,-1189,-416,230,-1173,-416,86,-1173,-416,70,-1189,-416,190,-877,-288,182,-829,-288,134,-821,-288,134,-1133,-288,166,-1133,-288,182,-1061,-288,246,-1069,-288,246,-877,-288,190,-877,-288,182,-1061,-288,134,-821,-288,182,-829,-288,190,-813,-288,246,-701,-288,222,-677,-288,134,-677,-288,190,-813,-288,246,-813,-288,246,-701,-288,278,-1109,-399,270,-1045,-404,246,-1037,-416,230,-1133,-416,246,-1037,-416,246,-957,-416,158,-973,-416,158,-1125,-416,230,-1133,-416,174,-677,-172,166,-677,-172,16`
+`6,-717,-172,182,-885,-112,182,-917,-112,238,-917,-112,238,-885,-112,182,-885,68,182,-917,68,238,-917,68,238,-885,68,230,-1093,-286,198,-1093,-286,198,-1125,-286,230,-1133,-286,198,-829,68,198,-861,68,238,-861,68,238,-829,68,246,-717,-172,246,-677,-172,198,-677,-172,198,-717,-172,270,-853,-625,302,-845,-625,302,-677,-625,262,-677,-625,302,-1189,-625,302,-949,-625,270,-941,-625,262,-1189,-625,270,-941,-625,270,-853,-625,262,-677,-625,262,-1189,-625,270,-1133,-404,270,-1149,-404,294,-1149,-393,294,`
+`-1133,-393,398,-741,74,390,-677,74,270,-677,74,278,-1189,74,390,-1189,74,398,-1053,74,398,-1053,74,534,-1053,74,534,-741,74,398,-741,74,422,-917,-392,414,-893,-391,382,-893,-391,350,-925,-392,286,-1037,-393,270,-1045,-404,278,-1109,-399,326,-1109,-392,454,-917,-392,422,-917,-392,350,-925,-392,286,-1037,-393,326,-1109,-392,462,-1157,-392,350,-1173,-392,446,-1173,-392,462,-1157,-392,326,-1109,-392,350,-925,-392,286,-909,-391,286,-1037,-393,294,-677,-432,278,-677,-432,270,-885,-432,310,-885,-432,30`
+`2,-1189,-432,310,-1189,-432,310,-1157,-432,278,-1165,-432,294,-933,-276,310,-901,-276,278,-901,-276,278,-1037,-276,278,-1117,-276,302,-1117,-276,294,-1013,-276,294,-933,-276,278,-901,-276,278,-1005,-276,294,-1013,-276,302,-1117,-276,294,-933,-276,294,-1013,-276,286,-1189,-760,302,-1189,-760,302,-1165,-760,286,-1053,-122,286,-1189,-128,318,-1189,-128,318,-1045,-122,286,-957,-432,286,-1013,-432,302,-1013,-432,302,-957,-432,326,-677,-128,286,-677,-128,302,-741,-128,302,-741,-128,286,-949,-128,318,-`
+`957,-128,326,-677,-128,302,-677,-744,294,-677,-744,294,-693,-744,318,-677,-416,318,-733,-413,334,-749,-397,438,-757,-392,494,-757,-406,534,-677,-400,494,-757,-406,494,-1101,-416,534,-1101,-416,534,-677,-400,334,-877,-280,350,-869,-280,334,-853,-280,382,-893,-391,414,-893,-391,422,-877,-392,334,-837,-392,478,-877,-392,478,-765,-392,438,-757,-392,334,-749,-397,334,-837,-392,422,-877,-392,342,-677,-768,342,-1189,-768,534,-1189,-770,534,-677,-768,342,-677,-636,342,-1189,-636,382,-1189,-636,382,-677,`
+`-636,422,-917,-432,414,-893,-432,382,-893,-432,342,-925,-432,446,-1173,-432,462,-1157,-432,462,-1117,-432,430,-1109,-432,350,-1173,-432,454,-917,-432,422,-917,-432,342,-925,-432,430,-1029,-432,462,-1021,-432,350,-1173,-432,430,-1109,-432,430,-1029,-432,342,-925,-432,350,-1101,-124,534,-1101,-124,534,-1061,-124,342,-1053,-124,382,-893,-432,414,-893,-432,422,-877,-432,342,-845,-432,342,-845,-432,422,-877,-432,470,-877,-432,470,-773,-432,342,-773,-432,350,-893,-252,350,-1189,-252,478,-1189,-251,478`
+`,-893,-251,422,-1085,74,422,-1189,74,438,-1189,74,438,-1085,74,422,-1085,32,422,-1109,32,526,-1109,32,526,-1085,32,438,-709,74,438,-677,74,422,-677,74,422,-709,74,454,-1077,149,454,-1189,149,534,-1189,131,534,-1069,132,462,-725,149,534,-725,132,534,-677,131,454,-677,149,462,-1085,74,462,-1189,74,486,-1189,74,486,-1085,74,486,-701,74,486,-677,74,462,-677,74,462,-709,74,494,-1181,-289,534,-1181,-289,534,-1141,-289,494,-1141,-289,534,-1125,-339,534,-1117,-339,494,-1117,-339,502,-1181,-414,534,-1181`
+`,-414,534,-1133,-414,502,-1133,-414],"vertslength":344,"polys":[0,3,4,7,8,10,11,14,15,20,21,25,26,28,29,34,35,38,39,41,42,44,45,49,50,53,54,56,57,61,62,65,66,68,69,71,72,75,76,79,80,84,85,89,90,93,94,97,98,103,104,107,108,113,114,116,117,120,121,125,126,128,129,132,133,136,137,140,141,144,145,148,149,152,153,156,157,160,161,164,165,170,171,174,175,178,179,182,183,188,189,192,193,195,196,199,200,203,204,206,207,210,211,214,215,217,218,220,221,224,225,228,229,231,232,235,236,238,239,244,245,248,24`
+`9,251,252,255,256,261,262,265,266,269,270,273,274,278,279,283,284,287,288,291,292,295,296,300,301,304,305,308,309,312,313,316,317,320,321,324,325,328,329,332,333,336,337,339,340,343],"polyslength":84,"regions":[15,15,37,16,2,2,2,2,2,5,5,5,10,10,10,17,17,17,17,18,18,19,19,41,11,11,12,12,13,13,42,31,32,33,34,22,23,23,23,45,1,1,4,4,4,4,4,25,46,47,47,47,47,48,35,50,27,27,56,8,8,58,6,6,3,28,9,9,9,9,29,14,14,7,59,60,61,20,21,62,63,30,64,36],"neighbors":[[[0],[0],[0],[1,1]],[[0],[0],[1,0],[0]],[[0],[0]`
+`,[0]],[[0],[0],[0],[0]],[[0],[0],[1,7],[0],[0],[0]],[[0],[1,29],[0],[0],[1,7]],[[0],[0],[1,7]],[[1,5],[0],[1,4],[1,6],[1,8],[0]],[[0],[0],[1,7],[0]],[[0],[0],[1,11]],[[1,14],[0],[1,11]],[[0],[0],[1,10],[1,9],[0]],[[0],[0],[1,13],[0]],[[1,12],[0],[1,14]],[[0],[1,13],[1,10],[0],[1,20]],[[0],[0],[1,18],[0]],[[0],[0],[1,18]],[[0],[0],[1,18]],[[1,17],[1,16],[0],[1,15]],[[0],[0],[1,20],[0]],[[0],[1,21],[1,19],[0],[1,14]],[[0],[1,20],[0],[0],[1,22]],[[0],[0],[1,21],[0]],[[0],[0],[0],[0]],[[0],[1,26],[0`
+`],[0],[0],[1,25]],[[0],[0],[1,24],[0]],[[1,24],[0],[1,27],[0],[0],[0]],[[0],[0],[1,26]],[[1,43],[0],[1,29],[0]],[[0],[1,5],[0],[0],[1,28]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,38]],[[0],[0],[1,38],[0]],[[0],[1,36],[0],[1,37]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[1,41]],[[0],[0],[0],[1,40]],[[0],[1,62],[0],[1,44]],[[0],[1,28],[0],[1,44]],[[0],[1,42],[1,46],[1,43],[1,45],[0]],[[0],[0],[1,44],[0]],[[0],[0],[1,44]]`
+`,[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,51]],[[0],[0],[1,52],[0]],[[1,49],[0],[0],[1,52]],[[0],[1,51],[1,50]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,57]],[[0],[0],[0],[1,56]],[[0],[0],[0]],[[0],[0],[1,63],[0],[1,60],[0]],[[0],[0],[0],[1,59]],[[0],[0],[0]],[[1,42],[0],[1,63],[0]],[[0],[0],[1,59],[0],[1,62],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,71],[0],[1,68]],[[0],[0],[0],[1,69],[0]],[[0],[1,66],[1,69],[0],[0]],[[1,67],[0],[1,68],[0]],[[0],[0],[0],[0]],[`
+`[1,66],[0],[1,72],[0]],[[1,71],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[70,-1085,-766,62,-957,-766,22,-957,-766,22,-1189,-768,246,-1189,-768,246,-1085,-766,70,-1085,-766,22,-1189,-768,22,-1189,-416,46,-1189,-416,22,-1181,-416,22,-1069,-172,22,-1131.4000244140625,-172,22,-1173,-159,246,-1173,-159,246,-1131.40`
+`00244140625,-172,246,-1069,-172,222,-677,-416,174,-677,-416,166,-693,-416,222,-829,-416,246,-813,-416,246,-701,-416,150,-957,-416,158,-973,-416,246,-957,-416,246,-877,-416,222,-861,-416,166,-693,-416,94,-677,-416,22,-677,-416,150,-957,-416,222,-861,-416,222,-829,-416,166,-693,-416,22,-677,-416,118,-965,-416,22,-1133,-416,118,-1133,-416,118,-965,-416,22,-677,-416,54,-989,-96,22,-1005,-87,22,-1021,-96,22,-1053,-92,32.66666793823242,-1031.6666259765625,-96,43.33333206176758,-1010.3333129882812,-87,`
+`34,-1017,-96,158,-933,-96,62,-941,-96,54,-989,-96,246,-1053,-91,246,-941,-96,158,-933,-96,54,-989,-96,43.33333206176758,-1010.3333129882812,-87,32.66666793823242,-1031.6666259765625,-96,22,-1053,-92,22,-877,-87,22,-901,-87,46,-901,-96,54,-869,-96,54,-869,-96,46,-901,-96,62,-941,-96,62,-813,-96,54,-869,-96,62,-941,-96,158,-933,-96,158,-813,-96,22,-821,-768,62,-821,-766,70,-781,-772,60.400001525878906,-760.2000122070312,-766,22,-677,-768,198,-749,-772,246,-741,-772,246,-677,-768,70,-781,-772,190,-`
+`781,-772,198,-749,-772,70,-781,-772,198,-749,-772,246,-677,-768,22,-677,-768,60.400001525878906,-760.2000122070312,-766,22,-733,-90,22,-773,-96,46,-773,-96,158,-733,-90,158,-813,-96,174,-789,-96,158,-733,-90,46,-773,-96,62,-813,-96,166,-717,-96,158,-733,-101,163.3333282470703,-751.6666870117188,-90,174,-789,-96,230,-805,-96,246,-677,-92,194,-745,-90,22,-677,-172,22,-717,-172,42.57143020629883,-717,-172,83.71428680419922,-717,-155,166,-717,-96,246,-677,-92,178.8000030517578,-677,-92,66.8000030517`
+`5781,-677,-165,238,-1189,-416,230,-1173,-416,86,-1173,-416,70,-1189,-416,190,-877,-288,182,-829,-288,134,-821,-288,134,-1133,-288,166,-1133,-288,182,-1061,-288,246,-1069,-288,246,-877,-288,190,-877,-288,182,-1061,-288,134,-821,-288,182,-829,-288,190,-813,-288,246,-701,-288,222,-677,-288,134,-677,-288,190,-813,-288,246,-813,-288,246,-701,-288,278,-1109,-404,270,-1045,-409,246,-1037,-416,230,-1133,-416,246,-1037,-416,246,-957,-416,158,-973,-416,158,-1125,-416,230,-1133,-416,174,-677,-172,166,-677,`
+`-172,166,-717,-172,182,-885,-112,182,-917,-112,238,-917,-112,238,-885,-112,182,-885,68,182,-917,68,238,-917,68,238,-885,68,230,-1093,-286,198,-1093,-286,198,-1125,-286,230,-1133,-286,198,-829,68,198,-861,68,238,-861,68,238,-829,68,246,-717,-172,246,-677,-172,198,-677,-172,198,-717,-172,270,-853,-625,302,-845,-625,302,-677,-625,262,-677,-625,302,-1189,-625,302,-949,-625,270,-941,-625,262,-1189,-625,270,-941,-625,270,-853,-625,262,-677,-625,262,-1189,-625,270,-1133,-399,270,-1149,-399,294,-1149,-3`
+`93,294,-1133,-393,398,-741,74,390,-677,74,270,-677,74,278,-1189,74,390,-1189,74,398,-1053,74,398,-1053,74,534,-1053,74,534,-741,74,398,-741,74,422,-917,-392,414,-893,-391,382,-893,-391,350,-925,-392,286,-1037,-392,270,-1045,-393,275.3333435058594,-1087.6666259765625,-399,278,-1109,-393,326,-1109,-392,454,-917,-392,422,-917,-392,350,-925,-392,286,-1037,-392,326,-1109,-392,462,-1157,-392,350,-1173,-392,446,-1173,-392,462,-1157,-392,326,-1109,-392,350,-925,-392,286,-909,-391,286,-1037,-392,294,-677`
+`,-432,278,-677,-432,270,-885,-432,310,-885,-432,302,-1189,-432,310,-1189,-432,310,-1157,-432,278,-1165,-432,294,-933,-276,310,-901,-276,278,-901,-276,278,-1037,-276,278,-1117,-276,302,-1117,-276,294,-1013,-276,294,-933,-276,278,-901,-276,278,-1005,-276,294,-1013,-276,302,-1117,-276,294,-933,-276,294,-1013,-276,286,-1189,-760,302,-1189,-760,302,-1165,-760,286,-1053,-122,286,-1189,-122,318,-1189,-122,318,-1045,-122,286,-957,-432,286,-1013,-432,302,-1013,-432,302,-957,-432,326,-677,-128,286,-677,-1`
+`28,302,-741,-128,302,-741,-128,286,-949,-128,318,-957,-128,326,-677,-128,302,-677,-744,294,-677,-744,294,-693,-744,318,-677,-416,318,-733,-416,334,-749,-407,417.20001220703125,-755.4000244140625,-403,438,-757,-416,475.3333435058594,-757,-415,494,-757,-401,534,-677,-400,512.4000244140625,-677,-400,469.20001220703125,-677,-416,450,-745,-416,426,-745,-414,474,-745,-415,330,-721,-416,494,-757,-401,494,-871.6666870117188,-401,494,-917.5333251953125,-416,494,-1101,-416,534,-1101,-416,534,-959.66668701`
+`17188,-416,534,-865.4444580078125,-400,534,-677,-400,334,-877,-280,350,-869,-280,334,-853,-280,382,-893,-392,414,-893,-392,422,-877,-392,334,-837,-392,478,-877,-392,478,-765,-392,438,-757,-392,334,-749,-397,334,-837,-392,422,-877,-392,342,-677,-768,342,-1189,-768,534,-1189,-770,534,-677,-768,342,-677,-636,342,-1189,-636,382,-1189,-636,382,-677,-636,422,-917,-432,414,-893,-432,382,-893,-432,342,-925,-432,446,-1173,-432,462,-1157,-432,462,-1117,-432,430,-1109,-432,350,-1173,-432,454,-917,-432,422,`
+`-917,-432,342,-925,-432,430,-1029,-432,462,-1021,-432,350,-1173,-432,430,-1109,-432,430,-1029,-432,342,-925,-432,350,-1101,-124,534,-1101,-124,534,-1061,-124,342,-1053,-124,382,-893,-432,414,-893,-432,422,-877,-432,342,-845,-432,342,-845,-432,422,-877,-432,470,-877,-432,470,-773,-432,342,-773,-432,350,-893,-252,350,-1189,-256,478,-1189,-251,478,-893,-251,422,-1085,74,422,-1189,74,438,-1189,74,438,-1085,74,422,-1085,32,422,-1109,32,526,-1109,32,526,-1085,32,438,-709,74,438,-677,74,422,-677,74,422`
+`,-709,74,454,-1077,149,454,-1189,147,534,-1189,131,534,-1069,132,462,-725,145,534,-725,131,534,-677,131,454,-677,147,462,-1085,74,462,-1189,74,486,-1189,74,486,-1085,74,486,-701,74,486,-677,74,462,-677,74,462,-709,74,494,-1181,-289,534,-1181,-289,534,-1141,-289,494,-1141,-289,534,-1125,-339,534,-1117,-339,494,-1117,-339,502,-1181,-414,534,-1181,-414,534,-1133,-414,502,-1133,-414],"vertslength":373,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,12,13,14,12,14,15,16,11,12,12,15,16,17,18,19,22,17,19,20,21,`
+`22,19,20,22,23,24,25,25,26,27,23,25,27,28,29,30,31,32,33,36,31,33,33,34,35,33,35,36,37,38,39,37,39,40,43,44,45,41,42,46,45,46,47,46,42,47,42,43,47,45,43,47,48,49,50,53,54,55,53,55,56,51,52,53,53,56,57,51,53,57,58,59,60,58,60,61,62,63,64,65,66,67,68,69,65,65,67,68,71,72,73,70,71,73,70,73,74,75,76,77,78,79,80,84,85,81,81,82,83,81,83,84,86,87,88,86,88,89,90,91,92,93,94,90,90,92,93,99,100,101,100,95,101,99,98,101,98,97,101,95,96,101,97,96,101,102,103,104,109,102,104,109,104,105,106,107,108,105,106,1`
+`08,105,108,109,110,111,112,110,112,113,114,115,116,117,118,119,119,114,116,116,117,119,121,122,123,120,121,123,124,125,126,127,128,129,124,126,127,124,127,129,130,131,132,133,134,135,133,135,136,137,138,139,140,141,137,137,139,140,142,143,144,148,145,146,146,147,148,152,149,150,150,151,152,153,154,155,153,155,156,160,157,158,158,159,160,164,161,162,162,163,164,165,166,167,165,167,168,169,170,171,169,171,172,173,174,175,173,175,176,180,177,178,178,179,180,181,182,183,184,185,186,186,181,183,183,1`
+`84,186,190,187,188,188,189,190,191,192,193,191,193,194,195,196,197,197,198,199,195,197,199,200,201,202,202,203,204,200,202,204,200,204,205,206,207,208,206,208,209,210,211,212,213,214,215,213,215,216,217,218,219,217,219,220,221,222,223,224,225,226,224,226,227,230,231,228,228,229,230,232,233,234,235,236,237,238,239,240,238,240,241,245,242,243,243,244,245,246,247,248,249,250,251,249,251,252,253,254,255,262,263,264,261,260,266,266,265,267,265,256,267,259,260,267,266,260,267,265,266,268,262,264,268,2`
+`65,264,268,266,261,268,262,261,268,267,256,269,256,257,269,258,257,269,258,259,269,267,259,269,273,274,275,272,273,275,271,272,275,271,275,276,270,271,276,270,276,277,278,279,280,281,282,283,281,283,284,285,286,287,289,290,285,287,288,289,285,287,289,294,291,292,292,293,294,298,295,296,296,297,298,299,300,301,299,301,302,303,304,305,303,305,306,303,306,307,308,309,310,311,312,308,308,310,311,313,314,315,313,315,316,317,318,319,317,319,320,321,322,323,321,323,324,325,326,327,328,329,325,325,327,3`
+`28,333,330,331,331,332,333,337,334,335,335,336,337,341,338,339,339,340,341,345,342,343,343,344,345,346,347,348,346,348,349,350,351,352,350,352,353,357,354,355,355,356,357,358,359,360,358,360,361,365,362,363,363,364,365,366,367,368,372,369,370,370,371,372],"trislength":211,"triTopoly":[0,0,1,1,2,3,3,3,3,4,4,4,4,5,5,5,6,7,7,7,7,8,8,9,9,9,9,9,9,10,11,11,11,11,11,12,12,13,14,14,14,15,15,15,16,17,18,18,18,19,19,20,20,20,21,21,21,21,21,21,22,22,22,22,22,22,23,23,24,24,24,24,25,25,26,26,26,26,27,28,28,`
+`29,29,29,30,31,31,32,32,33,33,34,34,35,35,36,36,37,37,38,38,39,39,40,40,40,40,41,41,42,42,43,43,43,44,44,44,44,45,45,46,47,47,48,48,49,50,50,51,51,52,53,54,54,55,55,56,57,57,58,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,60,60,60,60,60,60,61,62,62,63,63,63,63,64,64,65,65,66,66,67,67,67,68,68,68,69,69,70,70,71,71,72,72,72,73,73,74,74,75,75,76,76,77,77,78,78,79,79,80,80,81,81,82,83,83],"baseVert":[0,4,8,11,17,23,28,31,37,41,48,51,58,62,65,70,75,78,81,86,90,95,102,110,114,120,124,130,133,137,14`
+`2,145,149,153,157,161,165,169,173,177,181,187,191,195,200,206,210,213,217,221,224,228,232,235,238,242,246,249,253,256,270,278,281,285,291,295,299,303,308,313,317,321,325,330,334,338,342,346,350,354,358,362,366,369],"vertsCount":[4,4,3,6,6,5,3,6,4,7,3,7,4,3,5,5,3,3,5,4,5,7,8,4,6,4,6,3,4,5,3,4,4,4,4,4,4,4,4,4,6,4,4,5,6,4,3,4,4,3,4,4,3,3,4,4,3,4,3,14,8,3,4,6,4,4,4,5,5,4,4,4,5,4,4,4,4,4,4,4,4,4,3,4],"baseTri":[0,2,4,5,9,13,16,17,21,23,29,30,35,37,38,41,44,45,46,49,51,54,60,66,68,72,74,78,79,81,84,85`
+`,87,89,91,93,95,97,99,101,103,107,109,111,114,118,120,121,123,125,126,128,130,131,132,134,136,137,139,140,156,162,163,165,169,171,173,175,178,181,183,185,187,190,192,194,196,198,200,202,204,206,208,209],"triCount":[2,2,1,4,4,3,1,4,2,6,1,5,2,1,3,3,1,1,3,2,3,6,6,2,4,2,4,1,2,3,1,2,2,2,2,2,2,2,2,2,4,2,2,3,4,2,1,2,2,1,2,2,1,1,2,2,1,2,1,16,6,1,2,4,2,2,2,3,3,2,2,2,3,2,2,2,2,2,2,2,2,2,1,2]},"links":{"poly":[2,23,39,48,41,77,41,78,61,73,73,81,74,79,76,80,81,82],"cost":[864,1656,5430,5430,1867.19995117187`
+`5,2550,864,864,4134],"type":[1,2,2,2,2,2,1,1,2],"pos":[46,-1189,-416,70,-1189,-416,270,-1149,-404,278,-1165,-432,534,-1053,74,534,-1069,132,534,-741,74,534,-725,132,340.3999938964844,-873.7999877929688,-280,350,-893,-252,478,-1181,-251,494,-1181,-289,438,-1189,74,462,-1189,74,438,-709,74,462,-709,74,534,-1141,-289,534,-1125,-339],"length":9}}],["7_3",{"tileId":"7_3","tx":7,"ty":3,"mesh":{"verts":[534,-1189,-770,566,-1189,-770,574,-1093,-769,534,-677,-768,718,-1189,-769,814,-1189,-769,822,-1157,-`
+`769,710,-1093,-769,1038,-1021,-767,1038,-925,-767,998,-925,-768,990,-1029,-768,950,-1069,-768,990,-1029,-768,998,-925,-768,950,-885,-768,822,-1157,-769,950,-1165,-768,950,-1069,-768,710,-1093,-769,710,-1093,-769,950,-1069,-768,950,-885,-768,950,-677,-768,534,-677,-768,574,-1093,-769,534,-1069,130,534,-1189,129,582,-1189,119,582,-1069,120,534,-1133,-414,534,-1181,-414,550,-1181,-414,550,-1133,-414,534,-1117,-339,534,-1125,-339,550,-1133,-339,558,-1117,-339,550,-1133,-339,550,-1189,-339,558,-1189,`
+`-339,558,-1117,-339,590,-757,-406,566,-677,-400,534,-677,-400,534,-1101,-416,966,-677,-414,854,-677,-400,838,-733,-400,966,-677,-414,838,-733,-400,782,-765,-400,614,-1117,-416,630,-1189,-416,958,-1189,-414,590,-757,-406,534,-1101,-416,614,-1117,-416,782,-765,-400,534,-1061,-124,534,-1101,-124,942,-1101,-124,950,-1053,-124,598,-1061,74,694,-1061,74,734,-1053,74,718,-1013,74,574,-1013,74,534,-1053,74,574,-1013,74,566,-925,74,534,-917,74,534,-1053,74,694,-1061,74,598,-1061,74,598,-1189,74,694,-1189`
+`,74,534,-917,74,566,-925,74,574,-909,74,598,-837,74,590,-813,74,534,-797,74,686,-837,74,670,-821,74,622,-821,74,598,-837,74,574,-909,74,710,-909,74,614,-805,74,622,-821,74,670,-821,74,678,-805,74,534,-797,74,590,-813,74,614,-805,74,598,-733,74,534,-741,74,678,-805,74,702,-813,74,710,-741,74,694,-733,74,598,-733,74,614,-805,74,678,-805,74,694,-733,74,694,-677,74,598,-677,74,534,-677,129,534,-725,130,582,-725,120,582,-677,119,574,-1189,-367,590,-1189,-367,614,-1181,-367,606,-1133,-367,574,-1133,-3`
+`67,582,-989,148,598,-981,148,582,-933,148,710,-709,-207,734,-741,-207,766,-749,-207,822,-733,-207,838,-677,-207,582,-677,-207,590,-725,-207,614,-741,-207,678,-741,-207,710,-709,-207,838,-677,-207,582,-677,-207,710,-709,-207,686,-1189,-565,686,-1149,-565,654,-1125,-565,606,-1141,-565,598,-1189,-565,622,-725,-315,662,-717,-315,654,-677,-315,598,-677,-315,654,-1149,-702,630,-1141,-702,638,-1157,-702,654,-1149,-702,638,-1157,-702,614,-1173,-702,646,-1189,-702,670,-1189,-702,614,-949,148,614,-965,148`
+`,686,-965,148,686,-949,148,742,-997,74,718,-1013,74,734,-1053,74,902,-1061,74,910,-677,74,902,-677,74,894,-741,74,910,-797,74,702,-813,74,686,-837,74,710,-909,74,742,-925,74,902,-1061,74,910,-1053,74,910,-941,74,742,-925,74,742,-997,74,710,-741,74,702,-813,74,742,-925,74,910,-941,74,910,-797,74,894,-741,74,910,-941,74,1014,-941,74,1014,-797,74,910,-797,74,726,-989,148,726,-933,148,710,-933,148,702,-989,148,726,-1085,74,726,-1189,74,742,-1189,74,742,-1085,74,726,-1085,32,726,-1109,32,830,-1109,32`
+`,830,-1085,32,742,-709,74,742,-677,74,726,-677,74,726,-709,74,886,-1189,119,886,-1069,120,766,-1069,149,758,-1189,149,766,-725,149,886,-725,120,886,-677,119,758,-677,149,766,-1085,74,766,-1189,74,790,-1189,74,790,-1085,74,798,-717,-313,798,-677,-313,766,-677,-313,766,-725,-313,790,-701,74,790,-677,74,766,-677,74,766,-709,74,830,-1173,-718,830,-1189,-718,862,-1189,-718,870,-1173,-718,902,-677,-640,902,-1189,-640,950,-1189,-640,950,-677,-640,918,-1101,74,910,-1053,74,902,-1061,74,902,-1189,74,1014`
+`,-1189,74,1014,-1093,74,918,-1101,74,902,-1189,74,934,-1157,-488,950,-1189,-488,958,-1189,-488,910,-1189,-488,926,-1189,-488,934,-1157,-488,926,-1077,-488,910,-1101,-488,918,-765,-488,918,-677,-488,910,-677,-488,910,-1069,-488,958,-677,-488,950,-677,-488,942,-773,-488,926,-1077,-488,934,-1157,-488,958,-1189,-488,942,-773,-488,918,-765,-488,910,-1069,-488,926,-1077,-488,934,-949,148,934,-1085,148,998,-1085,148,998,-949,148,998,-789,148,998,-677,148,934,-677,148,934,-789,148,958,-957,-128,998,-949`
+`,-128,982,-821,-128,982,-821,-128,998,-677,-128,958,-677,-128,958,-957,-128,974,-1189,-520,982,-1189,-520,982,-701,-520,974,-1045,-122,974,-1189,-128,998,-1189,-128,998,-1045,-122,998,-909,-648,998,-1037,-648,1022,-1037,-648,1030,-909,-648,1038,-1189,-408,1046,-1189,-408,1046,-1173,-408,1046,-773,-240,1046,-677,-240,1038,-677,-240],"vertslength":287,"polys":[0,3,4,7,8,11,12,15,16,19,20,25,26,29,30,33,34,37,38,41,42,45,46,48,49,54,55,58,59,62,63,68,69,72,73,76,77,82,83,88,89,92,93,97,98,101,102,1`
+`07,108,111,112,116,117,119,120,124,125,129,130,132,133,137,138,141,142,144,145,149,150,153,154,157,158,161,162,165,166,170,171,176,177,180,181,184,185,188,189,192,193,196,197,200,201,204,205,208,209,212,213,216,217,220,221,224,225,228,229,232,233,235,236,240,241,244,245,250,251,254,255,258,259,262,263,265,266,269,270,272,273,276,277,280,281,283,284,286],"polyslength":68,"regions":[2,2,2,2,2,2,14,27,28,28,1,1,1,1,20,6,6,6,7,7,4,4,4,4,21,24,30,10,10,10,13,15,25,25,34,3,3,3,3,3,3,37,39,40,41,5,16,4`
+`2,26,43,44,17,8,8,19,19,19,19,19,11,12,23,23,45,46,47,48,50],"neighbors":[[[0],[0],[1,5],[0]],[[0],[0],[1,4],[0]],[[0],[0],[1,3],[0]],[[0],[1,2],[0],[1,5]],[[0],[0],[1,5],[1,1]],[[1,4],[1,3],[0],[0],[1,0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,9],[0]],[[0],[0],[0],[1,8]],[[0],[0],[0],[1,13]],[[0],[0],[1,12]],[[1,11],[0],[1,13],[0],[0],[0]],[[1,10],[0],[1,12],[0]],[[0],[0],[0],[0]],[[1,17],[0],[1,35],[0],[1,16],[0]],[[0],[1,18],[0],[1,15]],[[1,15],[0],[0],[0]],[[1,16],[0],[1,19],[0]`
+`,[1,21],[0]],[[0],[1,20],[0],[1,18],[0],[1,37]],[[0],[1,19],[0],[1,23]],[[1,18],[0],[1,23],[0],[0]],[[0],[1,39],[0],[1,23]],[[1,21],[1,20],[1,22],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0],[1,29]],[[0],[0],[0],[0],[1,29]],[[0],[1,28],[1,27]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,33]],[[1,32],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,15],[0],[1,38]],[[0],[0],[1,39],[0]],[[0],[1,19],[0],[1,39]],[[1,52],[0],[1,39],[0],[1,35]],[[1,22],[1,37],[1`
+`,38],[1,40],[1,36],[0]],[[0],[0],[0],[1,39]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,38],[0],[1,53]],[[0],[0],[1,52],[0]],[[0],[0],[1,57]],[[0],[0],[1,57],[0],[0]],[[0],[0],[0],[1,58]],[[0],[0],[1,58],[1,55],[1,54],[0]],[[0],[1,56],[0],[1,57]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,62]],[[0],[0],[0],[1,61]],[[0],[0],[0]],[[`
+`0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[534,-1189,-770,566,-1189,-770,574,-1093,-772,534,-677,-768,718,-1189,-769,814,-1189,-769,822,-1157,-766,710,-1093,-772,1038,-1021,-767,1038,-925,-767,998,-925,-768,990,-1029,-768,950,-1069,-768,990,-1029,-768,998,-925,-768,950,-885,-768,822,-1157,-766,950,-1165,-768,950,-1069,-768,710,-1093,-772,710,-1093,-772,950,-1069,-768,950,-885,-768,950,-677,-768,534,-677,-768,574,-1093,-772,534,-1069,128,534,-1189,127,582,-`
+`1189,119,582,-1069,120,534,-1133,-414,534,-1181,-414,550,-1181,-414,550,-1133,-414,534,-1117,-339,534,-1125,-339,550,-1133,-339,558,-1117,-339,550,-1133,-339,550,-1189,-339,558,-1189,-339,558,-1117,-339,590,-757,-416,584,-737,-400,566,-677,-400,534,-677,-400,534,-865.4444580078125,-400,534,-959.6666870117188,-416,534,-1101,-416,556.4000244140625,-963.4000244140625,-416,571.3333129882812,-871.6666870117188,-400,570,-753,-400,966,-677,-414,898.7999877929688,-677,-416,854,-677,-400,838,-733,-400,85`
+`9.3333129882812,-723.6666870117188,-400,880.6666870117188,-714.3333129882812,-414,966,-677,-414,880.6666870117188,-714.3333129882812,-414,859.3333129882812,-723.6666870117188,-400,838,-733,-400,782,-765,-400,732.5882568359375,-868.5294189453125,-400,693.058837890625,-951.3529663085938,-416,614,-1117,-416,630,-1189,-416,958,-1189,-414,794,-961,-416,842,-865,-400,866,-793,-400,590,-757,-416,571.3333129882812,-871.6666870117188,-400,556.4000244140625,-963.4000244140625,-416,534,-1101,-416,614,-1117`
+`,-416,693.058837890625,-951.3529663085938,-416,732.5882568359375,-868.5294189453125,-400,782,-765,-400,696.6666870117188,-761.4444580078125,-400,675.3333129882812,-760.5555419921875,-416,666,-793,-416,594,-889,-416,534,-1061,-124,534,-1101,-124,942,-1101,-124,950,-1053,-124,598,-1061,74,694,-1061,74,734,-1053,74,718,-1013,74,574,-1013,74,534,-1053,74,574,-1013,74,566,-925,74,534,-917,74,534,-1053,74,694,-1061,74,598,-1061,74,598,-1189,74,694,-1189,74,534,-917,74,566,-925,74,574,-909,74,598,-837,`
+`74,590,-813,74,534,-797,74,686,-837,74,670,-821,74,622,-821,74,598,-837,74,574,-909,74,710,-909,74,614,-805,74,622,-821,74,670,-821,74,678,-805,74,534,-797,74,590,-813,74,614,-805,74,598,-733,74,534,-741,74,678,-805,74,702,-813,74,710,-741,74,694,-733,74,598,-733,74,614,-805,74,678,-805,74,694,-733,74,694,-677,74,598,-677,74,534,-677,127,534,-725,127,582,-725,119,582,-677,119,574,-1189,-367,590,-1189,-367,614,-1181,-367,606,-1133,-367,574,-1133,-367,582,-989,148,598,-981,148,582,-933,148,710,-70`
+`9,-207,734,-741,-207,766,-749,-207,822,-733,-207,838,-677,-207,582,-677,-207,590,-725,-207,614,-741,-207,678,-741,-207,710,-709,-207,838,-677,-207,582,-677,-207,710,-709,-207,686,-1189,-565,686,-1149,-565,654,-1125,-565,606,-1141,-565,598,-1189,-565,622,-725,-315,662,-717,-315,654,-677,-315,598,-677,-315,654,-1149,-702,630,-1141,-702,638,-1157,-702,654,-1149,-702,638,-1157,-702,614,-1173,-702,646,-1189,-702,670,-1189,-702,614,-949,148,614,-965,148,686,-965,148,686,-949,148,742,-997,74,718,-1013,`
+`74,734,-1053,74,902,-1061,74,910,-677,74,902,-677,74,894,-741,74,910,-797,74,702,-813,74,686,-837,74,710,-909,74,742,-925,74,902,-1061,74,910,-1053,74,910,-941,74,742,-925,74,742,-997,74,710,-741,74,702,-813,74,742,-925,74,910,-941,74,910,-797,74,894,-741,74,910,-941,74,1014,-941,74,1014,-797,74,910,-797,74,726,-989,148,726,-933,148,710,-933,148,702,-989,148,726,-1085,74,726,-1189,74,742,-1189,74,742,-1085,74,726,-1085,32,726,-1109,32,830,-1109,32,830,-1085,32,742,-709,74,742,-677,74,726,-677,74`
+`,726,-709,74,886,-1189,119,886,-1069,120,766,-1069,146,758,-1189,147,766,-725,145,886,-725,119,886,-677,119,758,-677,147,766,-1085,74,766,-1189,74,790,-1189,74,790,-1085,74,798,-717,-313,798,-677,-313,766,-677,-313,766,-725,-313,790,-701,74,790,-677,74,766,-677,74,766,-709,74,830,-1173,-718,830,-1189,-718,862,-1189,-718,870,-1173,-718,902,-677,-640,902,-1189,-640,950,-1189,-640,950,-677,-640,918,-1101,74,910,-1053,74,902,-1061,74,902,-1189,74,1014,-1189,74,1014,-1093,74,918,-1101,74,902,-1189,74`
+`,934,-1157,-488,950,-1189,-488,958,-1189,-488,910,-1189,-488,926,-1189,-488,934,-1157,-488,926,-1077,-488,910,-1101,-488,918,-765,-488,918,-677,-488,910,-677,-488,910,-1069,-488,958,-677,-488,950,-677,-488,942,-773,-488,926,-1077,-488,934,-1157,-488,958,-1189,-488,942,-773,-488,918,-765,-488,910,-1069,-488,926,-1077,-488,934,-949,148,934,-1085,148,998,-1085,148,998,-949,148,998,-789,148,998,-677,148,934,-677,148,934,-789,148,958,-957,-128,998,-949,-128,982,-821,-128,982,-821,-128,998,-677,-128,9`
+`58,-677,-128,958,-957,-128,974,-1189,-520,982,-1189,-520,982,-701,-520,974,-1045,-122,974,-1189,-122,998,-1189,-122,998,-1045,-122,998,-909,-648,998,-1037,-648,1022,-1037,-648,1030,-909,-648,1038,-1189,-408,1046,-1189,-408,1046,-1173,-408,1046,-773,-240,1046,-677,-240,1038,-677,-240],"vertslength":311,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,16,17,18,16,18,19,20,21,22,25,20,22,22,23,24,22,24,25,29,26,27,27,28,29,33,30,31,31,32,33,34,35,36,34,36,37,38,39,40,38,40,41,43,44,`
+`45,47,48,49,47,49,50,46,47,50,45,46,51,42,43,51,45,43,51,46,50,51,42,50,51,54,55,56,54,56,57,53,54,57,52,53,57,66,67,68,63,64,68,66,65,68,64,65,68,68,63,69,62,63,69,68,67,69,58,67,69,58,59,70,69,58,70,59,60,70,60,61,70,69,62,70,61,62,70,77,78,79,73,74,75,73,75,76,79,77,81,72,71,81,79,80,81,71,80,81,72,73,82,73,76,82,72,81,82,76,77,82,81,77,82,83,84,85,83,85,86,88,89,90,91,92,87,91,87,88,88,90,91,93,94,95,93,95,96,100,97,98,98,99,100,101,102,103,104,105,106,101,103,104,101,104,106,107,108,109,107`
+`,109,110,112,107,110,110,111,112,113,114,115,113,115,116,117,118,119,120,121,117,117,119,120,124,125,122,122,123,124,126,127,128,128,129,130,130,131,126,126,128,130,135,132,133,133,134,135,136,137,138,138,139,140,136,138,140,141,142,143,144,145,146,146,147,148,144,146,148,149,150,151,151,152,153,149,151,153,154,155,156,157,158,159,159,160,161,157,159,161,162,163,164,162,164,165,166,167,168,169,170,171,172,173,169,169,171,172,177,174,175,175,176,177,178,179,180,178,180,181,182,183,184,182,184,185`
+`,186,187,188,186,188,189,190,191,192,192,193,194,190,192,194,195,196,197,198,199,200,197,198,200,195,197,200,204,201,202,202,203,204,205,206,207,205,207,208,212,209,210,210,211,212,216,213,214,214,215,216,220,217,218,218,219,220,221,222,223,221,223,224,225,226,227,225,227,228,232,229,230,230,231,232,233,234,235,233,235,236,237,238,239,237,239,240,241,242,243,241,243,244,248,245,246,246,247,248,249,250,251,249,251,252,253,254,255,253,255,256,257,258,259,260,261,262,262,263,264,260,262,264,265,266`
+`,267,265,267,268,269,270,271,272,273,274,269,271,272,269,272,274,277,278,275,275,276,277,282,279,280,280,281,282,286,283,284,284,285,286,287,288,289,290,291,292,290,292,293,294,295,296,300,297,298,298,299,300,301,302,303,301,303,304,305,306,307,308,309,310],"trislength":181,"triTopoly":[0,0,1,1,2,2,3,3,4,4,5,5,5,5,6,6,7,7,8,8,9,9,10,10,10,10,10,10,10,10,10,11,11,11,11,12,12,12,12,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,13,13,14,14,15,15,15,15,16,16,17,17,18,18,18,18,19,19,19,`
+`19,20,20,21,21,21,22,22,23,23,23,23,24,24,25,25,25,26,27,27,27,28,28,28,29,30,30,30,31,31,32,33,33,33,34,34,35,35,36,36,37,37,38,38,38,39,39,39,39,40,40,41,41,42,42,43,43,44,44,45,45,46,46,47,47,48,48,49,49,50,50,51,51,52,52,53,53,54,55,55,55,56,56,57,57,57,57,58,58,59,59,60,60,61,62,62,63,64,64,65,65,66,67],"baseVert":[0,4,8,12,16,20,26,30,34,38,42,52,58,71,83,87,93,97,101,107,113,117,122,126,132,136,141,144,149,154,157,162,166,169,174,178,182,186,190,195,201,205,209,213,217,221,225,229,233,237`
+`,241,245,249,253,257,260,265,269,275,279,283,287,290,294,297,301,305,308],"vertsCount":[4,4,4,4,4,6,4,4,4,4,10,6,13,12,4,6,4,4,6,6,4,5,4,6,4,5,3,5,5,3,5,4,3,5,4,4,4,4,5,6,4,4,4,4,4,4,4,4,4,4,4,4,4,4,3,5,4,6,4,4,4,3,4,3,4,4,3,3],"baseTri":[0,2,4,6,8,10,14,16,18,20,22,31,35,49,61,63,67,69,71,75,79,81,84,86,90,92,95,96,99,102,103,106,108,109,112,114,116,118,120,123,127,129,131,133,135,137,139,141,143,145,147,149,151,153,155,156,159,161,165,167,169,171,172,174,175,177,179,180],"triCount":[2,2,2,2,2,`
+`4,2,2,2,2,9,4,14,12,2,4,2,2,4,4,2,3,2,4,2,3,1,3,3,1,3,2,1,3,2,2,2,2,3,4,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,3,2,4,2,2,2,1,2,1,2,2,1,1]},"links":{"poly":[1,50,6,15,8,25,12,25,21,24,35,45,36,46,42,47,44,49,54,63],"cost":[4104.7939453125,3321.6923828125,1944,3792.37060546875,3321.6923828125,3288.89599609375,3321.6923828125,864,864,1920],"type":[2,2,2,2,2,2,2,1,1,2],"pos":[818.7058715820312,-1170.176513671875,-769,830,-1173,-718,582,-1069,120,583.2307739257812,-1059.15380859375,74,558,-1117,-339,574,-1133`
+`,-367,617.0117797851562,-1130.552978515625,-416,606,-1133,-367,583.2307739257812,-734.8461303710938,74,582,-725,120,886.416259765625,-1060.2579345703125,74,886,-1069,120,895.8461303710938,-726.2307739257812,74,886,-725,120,742,-1189,74,766,-1189,74,742,-709,74,766,-709,74,958,-1189,-488,974,-1189,-520],"length":10}}],["8_3",{"tileId":"8_3","tx":8,"ty":3,"mesh":{"verts":[1254,-845,-408,1238,-813,-408,1190,-805,-408,1046,-805,-408,1046,-1189,-408,1342,-1189,-416,1342,-1189,-416,1318,-845,-408,1254`
+`,-845,-408,1390,-773,-240,1390,-717,-240,1222,-725,-240,1046,-773,-240,1222,-725,-240,1214,-677,-240,1046,-677,-240,1046,-773,-240,1078,-1133,-575,1054,-1149,-565,1054,-1189,-543,1118,-1189,-543,1206,-1045,-608,1102,-1045,-608,1094,-1061,-608,1142,-1189,-543,1198,-1189,-543,1086,-1037,-608,1102,-1045,-608,1206,-1045,-608,1246,-1037,-608,1238,-909,-608,1190,-901,-608,1190,-901,-608,1198,-861,-608,1062,-861,-608,1054,-1061,-608,1086,-1037,-608,1142,-1189,-543,1094,-1061,-608,1078,-1053,-608,1054,-`
+`1061,-608,1054,-1061,-608,1078,-1053,-608,1086,-1037,-608,1350,-749,-416,1366,-741,-416,1374,-677,-416,1054,-677,-416,1198,-749,-416,1054,-773,-416,1190,-773,-416,1198,-749,-416,1054,-677,-416,1062,-1189,-703,1198,-1189,-703,1190,-1037,-768,1062,-1029,-768,1230,-1021,-768,1238,-925,-768,1190,-917,-768,1190,-1037,-768,1230,-1021,-768,1190,-917,-768,1070,-869,-768,1062,-1029,-768,1190,-917,-768,1198,-869,-768,1070,-869,-768,1262,-917,-768,1238,-925,-768,1230,-1021,-768,1478,-1021,-767,1478,-933,-7`
+`67,1398,-909,-768,1398,-909,-768,1398,-677,-640,1262,-677,-640,1262,-917,-768,1374,-709,-160,1374,-677,-160,1230,-677,-160,1230,-709,-160,1470,-1037,-608,1470,-909,-608,1454,-925,-608,1470,-1037,-608,1454,-925,-608,1406,-909,-608,1262,-877,-603,1238,-909,-608,1246,-1037,-608,1398,-797,-555,1262,-797,-555,1262,-877,-603,1406,-909,-608,1254,-813,-360,1254,-829,-360,1278,-829,-360,1278,-813,-360,1334,-733,-507,1398,-733,-513,1366,-717,-497,1334,-733,-507,1366,-717,-497,1366,-693,-486,1262,-693,-486`
+`,1262,-749,-518,1262,-749,-518,1334,-757,-523,1334,-733,-507,1262,-717,-768,1262,-733,-768,1398,-733,-768,1398,-709,-768,1294,-813,-312,1294,-829,-312,1326,-829,-312,1326,-813,-312,1422,-805,-416,1342,-813,-408,1318,-845,-408,1342,-1189,-416,1558,-677,-416,1422,-677,-416,1422,-805,-416,1422,-805,-416,1342,-1189,-416,1558,-1189,-416,1558,-677,-416,1422,-1189,-480,1558,-1189,-480,1558,-1181,-480,1494,-1053,-608,1558,-1053,-608,1558,-909,-608,1494,-909,-608,1502,-1045,-768,1558,-1045,-768,1558,-909`
+`,-768,1502,-909,-768],"vertslength":140,"polys":[0,5,6,8,9,12,13,16,17,20,21,25,26,31,32,36,37,40,41,43,44,48,49,52,53,56,57,59,60,64,65,67,68,73,74,77,78,81,82,84,85,90,91,94,95,98,99,101,102,106,107,109,110,113,114,117,118,121,122,124,125,128,129,131,132,135,136,139],"polyslength":34,"regions":[2,2,8,8,13,3,3,3,3,3,9,9,7,6,6,6,5,5,14,4,4,4,16,11,11,11,17,18,1,1,1,25,10,12],"neighbors":[[[0],[0],[0],[0],[0],[1,1]],[[1,28],[0],[1,0]],[[0],[0],[1,3],[0]],[[0],[0],[0],[1,2]],[[0],[0],[0],[0]],[[1,`
+`6],[0],[1,8],[0],[0]],[[0],[1,5],[0],[1,20],[0],[1,7]],[[0],[0],[0],[1,9],[1,6]],[[1,5],[0],[1,9],[0]],[[1,8],[0],[1,7]],[[0],[0],[0],[1,11],[0]],[[0],[0],[1,10],[0]],[[0],[0],[1,14],[0]],[[1,16],[0],[1,14]],[[0],[1,13],[1,15],[0],[1,12]],[[0],[0],[1,14]],[[0],[1,13],[0],[0],[0],[1,17]],[[0],[0],[0],[1,16]],[[0],[0],[0],[0]],[[0],[0],[1,20]],[[1,19],[0],[1,21],[0],[1,6],[0]],[[0],[0],[1,20],[0]],[[0],[0],[0],[0]],[[0],[0],[1,24]],[[1,23],[0],[0],[0],[1,25]],[[0],[0],[1,24]],[[0],[0],[0],[0]],[[0`
+`],[0],[0],[0]],[[0],[0],[1,1],[1,30]],[[0],[0],[1,30]],[[1,28],[0],[0],[1,29]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1254,-845,-408,1238,-813,-408,1190,-805,-408,1046,-805,-408,1046,-1189,-408,1091.5384521484375,-1189,-416,1342,-1189,-416,1271.5999755859375,-913.7999877929688,-416,1265.7332763671875,-890.8666381835938,-408,1250,-889,-408,1342,-1189,-416,1322.800048828125,-913.7999877929688,-416,1321.199951171875,-890.8666381835938,-408,1318,-845,-408,1254,-845,-4`
+`08,1265.7332763671875,-890.8666381835938,-408,1271.5999755859375,-913.7999877929688,-416,1290,-889,-408,1390,-773,-240,1390,-717,-240,1222,-725,-240,1046,-773,-240,1222,-725,-240,1214,-677,-240,1046,-677,-240,1046,-773,-240,1078,-1133,-575,1054,-1149,-575,1054,-1189,-549,1118,-1189,-543,1206,-1045,-608,1102,-1045,-608,1094,-1061,-608,1110,-1103.6666259765625,-602,1142,-1189,-549,1198,-1189,-549,1202.5714111328125,-1106.7142333984375,-602,1086,-1037,-608,1102,-1045,-608,1206,-1045,-608,1246,-1037`
+`,-608,1238,-909,-608,1190,-901,-608,1190,-901,-608,1198,-861,-608,1062,-861,-608,1054,-1061,-608,1086,-1037,-608,1142,-1189,-549,1110,-1103.6666259765625,-602,1094,-1061,-608,1078,-1053,-608,1054,-1061,-608,1079.142822265625,-1097.5714111328125,-607,1054,-1061,-608,1078,-1053,-608,1086,-1037,-608,1350,-749,-416,1366,-741,-416,1374,-677,-416,1054,-677,-416,1198,-749,-416,1054,-773,-416,1190,-773,-416,1198,-749,-416,1054,-677,-416,1062,-1189,-709,1198,-1189,-709,1193.4285888671875,-1102.1428222656`
+`25,-762,1190,-1037,-768,1062,-1029,-768,1062,-1097.5714111328125,-767,1230,-1021,-768,1238,-925,-768,1190,-917,-768,1190,-1037,-768,1230,-1021,-768,1190,-917,-768,1070,-869,-768,1062,-1029,-768,1190,-917,-768,1198,-869,-768,1070,-869,-768,1262,-917,-768,1238,-925,-768,1230,-1021,-768,1478,-1021,-767,1478,-933,-767,1398,-909,-768,1398,-909,-768,1398,-885.7999877929688,-768,1398,-700.2000122070312,-641,1398,-677,-640,1262,-677,-640,1262,-698.8181762695312,-641,1262,-873.3636474609375,-758,1262,-91`
+`7,-768,1374,-709,-160,1374,-677,-160,1230,-677,-160,1230,-709,-160,1470,-1037,-608,1470,-909,-608,1454,-925,-608,1470,-1037,-608,1454,-925,-608,1406,-909,-608,1303.142822265625,-886.1428833007812,-608,1262,-877,-598,1238,-909,-608,1246,-1037,-608,1398,-797,-555,1262,-797,-555,1262,-837,-571,1262,-877,-598,1303.142822265625,-886.1428833007812,-608,1406,-909,-608,1404.4000244140625,-886.5999755859375,-608,1399.5999755859375,-819.4000244140625,-561,1394,-873,-598,1254,-813,-360,1254,-829,-360,1278,`
+`-829,-360,1278,-813,-360,1334,-733,-502,1376.6666259765625,-733,-502,1398,-733,-513,1366,-717,-491,1334,-733,-502,1366,-717,-491,1366,-693,-486,1262,-693,-486,1262,-749,-513,1262,-749,-513,1316,-755,-518,1334,-757,-507,1334,-733,-502,1262,-717,-768,1262,-733,-768,1398,-733,-768,1398,-709,-768,1294,-813,-312,1294,-829,-312,1326,-829,-312,1326,-813,-312,1422,-805,-416,1342,-813,-408,1318,-845,-408,1321.199951171875,-890.8666381835938,-408,1322.800048828125,-913.7999877929688,-416,1342,-1189,-416,1`
+`398.4705810546875,-917.941162109375,-416,1403.176513671875,-895.3529663085938,-408,1407.88232421875,-872.7647094726562,-416,1402,-889,-408,1558,-677,-416,1422,-677,-416,1422,-805,-416,1422,-805,-416,1407.88232421875,-872.7647094726562,-416,1403.176513671875,-895.3529663085938,-408,1398.4705810546875,-917.941162109375,-416,1342,-1189,-416,1558,-1189,-416,1558,-677,-416,1422,-1189,-480,1558,-1189,-480,1558,-1181,-480,1494,-1053,-608,1558,-1053,-608,1558,-909,-608,1494,-909,-608,1502,-1045,-768,155`
+`8,-1045,-768,1558,-909,-768,1502,-909,-768],"vertslength":176,"tris":[0,1,2,3,4,5,5,6,7,2,3,9,2,0,9,7,8,9,0,8,9,3,5,9,7,5,9,10,11,16,16,11,17,11,12,17,13,12,17,16,15,17,13,14,17,15,14,17,18,19,20,18,20,21,22,23,24,22,24,25,26,27,28,26,28,29,31,32,33,34,35,36,30,31,33,36,30,33,33,34,36,37,38,39,39,40,41,39,41,42,37,39,42,43,44,45,45,46,47,43,45,47,50,51,52,50,52,53,49,50,53,48,49,53,54,55,56,57,58,59,61,57,59,59,60,61,62,63,64,62,64,65,69,70,71,68,69,71,68,71,66,66,67,68,72,73,74,75,76,77,79,75,7`
+`7,77,78,79,80,81,82,83,84,85,86,87,88,88,83,85,85,86,88,92,93,94,91,92,94,96,89,90,95,96,90,91,94,95,90,91,95,100,97,98,98,99,100,101,102,103,107,108,109,104,105,106,106,107,109,106,109,110,104,106,110,113,114,115,112,113,115,118,111,112,112,115,118,115,116,119,118,115,119,116,117,119,118,117,119,123,120,121,121,122,123,125,126,127,124,125,127,128,129,130,131,132,128,128,130,131,134,135,136,133,134,136,137,138,139,137,139,140,144,141,142,142,143,144,149,150,151,145,146,154,151,152,154,145,153,15`
+`4,152,153,154,151,149,154,149,148,154,146,147,154,148,147,154,155,156,157,164,158,159,164,159,160,164,160,161,161,162,163,161,163,164,165,166,167,171,168,169,169,170,171,175,172,173,173,174,175],"trislength":112,"triTopoly":[0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,3,3,4,4,5,5,5,5,5,6,6,6,6,7,7,7,8,8,8,8,9,10,10,10,11,11,12,12,12,12,13,14,14,14,15,16,16,16,16,17,17,17,17,17,17,18,18,19,20,20,20,20,20,21,21,21,21,21,21,21,21,22,22,23,23,24,24,24,25,25,26,26,27,27,28,28,28,28,28,28,28,28,28,29,30,30,30`
+`,30,30,31,32,32,33,33],"baseVert":[0,10,18,22,26,30,37,43,48,54,57,62,66,72,75,80,83,89,97,101,104,111,120,124,128,133,137,141,145,155,158,165,168,172],"vertsCount":[10,8,4,4,4,7,6,5,6,3,5,4,6,3,5,3,6,8,4,3,7,9,4,4,5,4,4,4,10,3,7,3,4,4],"baseTri":[0,9,16,18,20,22,27,31,34,38,39,42,44,48,49,52,53,57,63,65,66,71,79,81,83,86,88,90,92,101,102,107,108,110],"triCount":[9,7,2,2,2,5,4,3,4,1,3,2,4,1,3,1,4,6,2,1,5,8,2,2,3,2,2,2,9,1,5,1,2,2]},"links":{"poly":[0,22,4,8,19,32,22,27],"cost":[3532.800048828125`
+`,635.2532348632812,864,3840],"type":[2,1,1,2],"pos":[1247.5999755859375,-832.2000122070312,-408,1254,-829,-360,1118,-1189,-543,1134.297119140625,-1177.7957763671875,-548.6896362304688,1470,-1037,-608,1494,-1037,-608,1278,-829,-360,1294,-829,-312],"length":4}}],["9_3",{"tileId":"9_3","tx":9,"ty":3,"mesh":{"verts":[1558,-1189,-480,1638,-1189,-480,1558,-1181,-480,1558,-677,-416,1558,-1189,-416,1630,-1189,-416,1630,-677,-416,1558,-1053,-608,1662,-1045,-608,1646,-1029,-608,1646,-1029,-608,1646,-909,-`
+`608,1558,-909,-608,1558,-1053,-608,1782,-1037,-768,1774,-917,-768,1558,-909,-768,1558,-1045,-768,1950,-877,-344,1926,-845,-352,1894,-829,-352,1838,-837,-352,1798,-885,-352,1798,-885,-352,1838,-837,-352,1790,-813,-352,1798,-885,-352,1790,-813,-352,1662,-805,-352,1646,-1189,-390,1798,-1149,-386,2006,-1189,-408,2006,-1157,-408,1798,-1149,-386,1646,-1189,-390,1670,-909,-608,1662,-957,-608,1670,-1013,-608,1806,-1013,-608,1814,-901,-608,1798,-749,-352,1822,-725,-352,1822,-677,-352,1662,-677,-352,1662,`
+`-805,-352,1662,-805,-352,1790,-813,-352,1798,-749,-352,1798,-885,-60,1950,-877,-60,1926,-853,-60,1798,-885,-60,1926,-853,-60,1934,-821,-60,1934,-821,-60,1990,-789,-60,2054,-677,-60,1686,-677,-60,1798,-885,-60,2022,-1189,-60,2006,-1157,-60,1798,-1149,-60,1686,-1189,-60,1686,-1189,-60,1798,-1149,-60,1798,-885,-60,1686,-677,-60,1774,-917,-768,1782,-1037,-768,1902,-1037,-768,1902,-909,-768,1814,-749,-276,1806,-813,-276,1878,-821,-276,1886,-757,-276,1998,-1045,-608,1998,-941,-608,1982,-949,-608,1974,`
+`-1021,-608,1966,-1053,-608,1974,-1021,-608,1982,-949,-608,1958,-901,-608,1822,-901,-608,1822,-1029,-608,1830,-1053,-608,1966,-1053,-608,1982,-949,-608,1870,-765,-350,1830,-757,-350,1822,-805,-350,1870,-813,-350,1894,-829,-352,1926,-845,-352,1942,-813,-343,1902,-749,-352,1894,-829,-352,1942,-813,-343,1990,-789,-336,1822,-677,-352,1822,-725,-352,1902,-749,-352,1902,-749,-352,1990,-789,-336,2054,-677,-336,1822,-677,-352,1926,-1069,-48,1934,-1029,-48,1902,-1013,-48,1846,-1021,-48,1846,-1021,-48,1846`
+`,-957,-48,1830,-949,-48,1830,-1125,-48,1998,-1125,-48,1982,-1069,-48,1926,-1069,-48,1926,-1069,-48,1846,-1021,-48,1830,-1125,-48,1998,-1125,-48,1830,-909,-48,1830,-949,-48,1846,-957,-48,1894,-957,-48,1958,-909,-48,1902,-1013,-48,1934,-1029,-48,1966,-1021,-48,1958,-909,-48,1894,-957,-48,1902,-1037,-768,1998,-1037,-768,1998,-917,-768,1902,-909,-768,2014,-1101,-48,2038,-1133,-60,2070,-1117,-60,2046,-1037,-60,2022,-1037,-60,2070,-917,-60,2046,-917,-60,2046,-1037,-60,2070,-1117,-60,2022,-1037,-60,202`
+`2,-909,-60,1982,-909,-60,2014,-1101,-48,2030,-965,-352,1998,-965,-352,1990,-1005,-352,2022,-1045,-352,2014,-821,-352,1998,-845,-352,1998,-861,-352,2070,-877,-352,2070,-845,-352,2014,-781,-60,2022,-805,-60,2070,-813,-60,2070,-701,-60,2046,-1045,-352,2030,-1053,-352,2070,-1117,-352,2070,-957,-352,2038,-965,-352,2046,-1045,-352,2070,-1117,-352,2070,-1165,-352,2070,-1149,-352,2054,-1149,-352],"vertslength":171,"polys":[0,2,3,6,7,9,10,13,14,17,18,22,23,25,26,30,31,34,35,39,40,44,45,47,48,50,51,53,54,`
+`58,59,62,63,66,67,70,71,74,75,78,79,81,82,87,88,91,92,94,95,98,99,101,102,105,106,109,110,113,114,116,117,120,121,125,126,130,131,134,135,139,140,143,144,147,148,151,152,156,157,160,161,163,164,167,168,170],"polyslength":43,"regions":[23,12,11,11,4,2,2,2,2,9,5,5,1,1,1,1,1,6,14,3,3,3,17,8,8,8,8,10,10,10,10,13,13,7,15,15,15,22,20,16,21,21,42],"neighbors":[[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,3]],[[0],[0],[0],[1,2]],[[1,17],[0],[0],[0]],[[0],[1,23],[0],[1,6],[0]],[[1,5],[0],[1,7]],[[1,6],[1,`
+`11],[0],[1,8],[0]],[[0],[0],[1,7],[0]],[[0],[0],[0],[0],[0]],[[0],[1,25],[0],[0],[1,11]],[[1,7],[0],[1,10]],[[0],[0],[1,13]],[[1,12],[0],[1,14]],[[0],[0],[0],[1,16],[1,13]],[[0],[0],[1,16],[0]],[[1,15],[0],[1,14],[0]],[[1,4],[0],[1,33],[0]],[[0],[0],[0],[0]],[[0],[0],[1,20],[0]],[[0],[1,19],[1,21]],[[0],[0],[0],[0],[1,20],[0]],[[0],[0],[0],[0]],[[1,5],[0],[1,24]],[[0],[1,23],[0],[1,26]],[[1,10],[0],[1,26]],[[1,24],[0],[0],[1,25]],[[0],[1,32],[0],[1,30]],[[0],[1,31],[0],[1,30]],[[0],[0],[1,30]],[`
+`[1,27],[1,28],[0],[1,29]],[[0],[1,28],[0],[1,32],[0]],[[1,27],[0],[0],[1,31],[0]],[[0],[0],[0],[1,17]],[[0],[0],[1,35],[0],[1,36]],[[0],[0],[1,34],[0]],[[0],[0],[0],[1,34]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,41]],[[0],[0],[1,40],[0]],[[0],[0],[0]]]},"detail":{"verts":[1558,-1189,-480,1638,-1189,-480,1558,-1181,-480,1558,-677,-416,1558,-1189,-416,1630,-1189,-416,1630,-677,-416,1558,-1053,-608,1662,-1045,-608,1646,-1029,-608,1646,-1029,-608,1646,-909,-608,1558,-9`
+`09,-608,1558,-1053,-608,1782,-1037,-768,1774,-917,-768,1558,-909,-768,1558,-1045,-768,1950,-877,-344,1926,-845,-352,1894,-829,-352,1838,-837,-352,1798,-885,-352,1906.5714111328125,-879.2857055664062,-352,1930,-873,-343,1798,-885,-352,1838,-837,-352,1790,-813,-352,1798,-885,-352,1790,-813,-352,1662,-805,-352,1652.5882568359375,-1030.88232421875,-352,1646,-1189,-396,1798,-1149,-384,1798,-1039,-354,2006,-1189,-408,2006,-1157,-408,1959.77783203125,-1155.22216796875,-411,1798,-1149,-384,1646,-1189,-3`
+`96,1803.5,-1189,-395,1961,-1189,-412,1670,-909,-608,1662,-957,-608,1670,-1013,-608,1806,-1013,-608,1814,-901,-608,1798,-749,-352,1822,-725,-352,1822,-677,-352,1662,-677,-352,1662,-805,-352,1662,-805,-352,1790,-813,-352,1798,-749,-352,1798,-885,-60,1950,-877,-60,1926,-853,-60,1798,-885,-60,1926,-853,-60,1934,-821,-60,1934,-821,-60,1990,-789,-60,2054,-677,-60,1686,-677,-60,1798,-885,-60,2022,-1189,-60,2006,-1157,-60,1798,-1149,-60,1686,-1189,-60,1686,-1189,-60,1798,-1149,-60,1798,-885,-60,1686,-67`
+`7,-60,1774,-917,-768,1782,-1037,-768,1902,-1037,-768,1902,-909,-768,1814,-749,-276,1806,-813,-276,1878,-821,-276,1886,-757,-276,1998,-1045,-608,1998,-941,-608,1982,-949,-608,1974,-1021,-608,1966,-1053,-608,1974,-1021,-608,1982,-949,-608,1958,-901,-608,1822,-901,-608,1822,-1029,-608,1830,-1053,-608,1966,-1053,-608,1982,-949,-608,1870,-765,-350,1830,-757,-350,1822,-805,-350,1870,-813,-350,1894,-829,-352,1926,-845,-352,1942,-813,-343,1902,-749,-352,1894,-829,-352,1942,-813,-343,1990,-789,-336,1937.`
+`199951171875,-765,-352,1822,-677,-352,1822,-725,-352,1902,-749,-352,1902,-749,-352,1937.199951171875,-765,-352,1990,-789,-336,2054,-677,-336,2030.800048828125,-677,-336,1984.4000244140625,-677,-352,1822,-677,-352,2026,-681,-344,1954,-753,-344,1926,-1069,-48,1934,-1029,-48,1902,-1013,-48,1846,-1021,-48,1846,-1021,-48,1846,-957,-48,1830,-949,-48,1830,-1125,-48,1998,-1125,-48,1982,-1069,-48,1926,-1069,-48,1926,-1069,-48,1846,-1021,-48,1830,-1125,-48,1998,-1125,-48,1830,-909,-48,1830,-949,-48,1846,-`
+`957,-48,1894,-957,-48,1958,-909,-48,1902,-1013,-48,1934,-1029,-48,1966,-1021,-48,1958,-909,-48,1894,-957,-48,1902,-1037,-768,1998,-1037,-768,1998,-917,-768,1902,-909,-768,2014,-1101,-60,2038,-1133,-60,2070,-1117,-60,2046,-1037,-60,2022,-1037,-60,2026,-1049,-47,2070,-917,-60,2046,-917,-60,2046,-1037,-60,2070,-1117,-60,2022,-1037,-60,2022,-909,-60,1982,-909,-60,2014,-1101,-60,2030,-965,-352,1998,-965,-352,1990,-1005,-352,2022,-1045,-352,2014,-821,-352,1998,-845,-352,1998,-861,-352,2070,-877,-352,2`
+`070,-845,-352,2014,-781,-60,2022,-805,-60,2070,-813,-60,2070,-701,-60,2046,-1045,-352,2030,-1053,-352,2070,-1117,-352,2070,-957,-352,2038,-965,-352,2046,-1045,-352,2070,-1117,-352,2070,-1165,-352,2070,-1149,-352,2054,-1149,-352],"vertslength":185,"tris":[0,1,2,6,3,4,4,5,6,7,8,9,10,11,12,10,12,13,14,15,16,14,16,17,23,19,20,23,20,21,21,22,23,23,18,24,18,19,24,19,23,24,25,26,27,28,29,30,32,33,34,31,32,34,31,34,28,28,30,31,41,35,36,41,36,37,40,41,37,38,39,40,37,38,40,42,43,44,42,44,45,42,45,46,47,48`
+`,49,47,49,50,47,50,51,52,53,54,55,56,57,58,59,60,61,62,63,65,61,63,63,64,65,66,67,68,66,68,69,70,71,72,70,72,73,74,75,76,74,76,77,78,79,80,78,80,81,83,84,85,82,83,85,86,87,88,90,91,92,93,94,89,89,90,92,89,92,93,95,96,97,95,97,98,99,100,101,104,105,106,104,106,102,102,103,104,107,108,109,110,115,116,114,115,117,112,113,117,114,113,117,111,112,118,112,117,118,115,117,118,115,110,118,111,110,118,119,120,121,119,121,122,123,124,125,123,125,126,127,128,129,130,131,132,130,132,133,134,135,136,134,136,`
+`137,134,137,138,139,140,141,143,139,141,141,142,143,144,145,146,144,146,147,148,149,150,151,152,153,152,148,153,148,150,153,151,150,153,154,155,156,154,156,157,158,159,160,158,160,161,162,163,164,162,164,165,166,167,168,169,170,166,166,168,169,171,172,173,171,173,174,175,176,177,178,179,180,178,180,181,182,183,184],"trislength":103,"triTopoly":[0,1,1,2,3,3,4,4,5,5,5,5,5,5,6,7,7,7,7,7,8,8,8,8,8,9,9,9,10,10,10,11,12,13,14,14,14,15,15,16,16,17,17,18,18,19,19,20,21,21,21,21,22,22,23,24,24,24,25,26,2`
+`6,26,26,26,26,26,26,26,27,27,28,28,29,30,30,31,31,31,32,32,32,33,33,34,34,34,34,34,35,35,36,36,37,37,38,38,38,39,39,40,41,41,42],"baseVert":[0,3,7,10,14,18,25,28,35,42,47,52,55,58,61,66,70,74,78,82,86,89,95,99,102,107,110,119,123,127,130,134,139,144,148,154,158,162,166,171,175,178,182],"vertsCount":[3,4,3,4,4,7,3,7,7,5,5,3,3,3,5,4,4,4,4,4,3,6,4,3,5,3,9,4,4,3,4,5,5,4,6,4,4,4,5,4,3,4,3],"baseTri":[0,1,3,4,6,8,14,15,20,25,28,31,32,33,34,37,39,41,43,45,47,48,52,54,55,58,59,68,70,72,73,75,78,81,83,88`
+`,90,92,94,97,99,100,102],"triCount":[1,2,1,2,2,6,1,5,5,3,3,1,1,1,3,2,2,2,2,2,1,4,2,1,3,1,9,2,2,1,2,3,3,2,5,2,2,2,3,2,1,2,1]},"links":{"poly":[1,7,2,9,9,21,37,41],"cost":[1398,1200,96,96],"type":[2,1,1,1],"pos":[1630,-1189,-416,1646,-1189,-390,1650,-1033,-608,1670,-1013,-608,1814,-901,-608,1822,-901,-608,2030,-965,-352,2038,-965,-352],"length":4}}],["10_3",{"tileId":"10_3","tx":10,"ty":3,"mesh":{"verts":[2070,-1141,-352,2070,-1157,-352,2126,-1141,-352,2134,-1109,-352,2294,-981,-352,2286,-949,-352`
+`,2246,-933,-352,2070,-957,-352,2070,-1109,-352,2334,-957,-60,2310,-941,-60,2070,-917,-60,2070,-1109,-60,2070,-869,-352,2086,-861,-352,2070,-845,-352,2222,-877,-60,2310,-877,-60,2302,-837,-60,2198,-677,-60,2078,-677,-60,2070,-693,-60,2070,-821,-60,2222,-877,-60,2302,-837,-60,2222,-877,-352,2310,-877,-352,2310,-853,-352,2214,-701,-352,2166,-733,-352,2094,-829,-352,2110,-877,-60,2150,-869,-60,2110,-861,-60,2310,-1141,-352,2342,-1133,-352,2342,-1093,-352,2310,-1141,-352,2342,-1093,-352,2310,-1021,-3`
+`52,2142,-1189,-352,2142,-1189,-352,2318,-1189,-352,2310,-1141,-352,2310,-1141,-60,2342,-1133,-60,2358,-981,-60,2142,-1189,-60,2318,-1189,-60,2310,-1141,-60,2142,-1189,-60,2310,-1141,-60,2358,-981,-60,2238,-1053,-352,2222,-1045,-352,2158,-1085,-352,2142,-1149,-352,2150,-1101,-60,2182,-1117,-60,2262,-1029,-60,2270,-677,-352,2230,-677,-352,2310,-805,-352,2246,-717,-60,2302,-805,-60,2310,-781,-60,2278,-685,-60,2302,-957,-352,2334,-949,-352,2302,-941,-352,2310,-677,-352,2302,-677,-352,2302,-701,-352,`
+`2310,-677,-60,2302,-677,-60,2302,-701,-60,2310,-909,-60,2342,-917,-60,2342,-901,-60,2326,-1021,-352,2350,-1061,-352,2358,-981,-352,2374,-709,-60,2366,-677,-60,2334,-677,-60,2334,-797,-60,2334,-797,-60,2366,-885,-60,2382,-885,-60,2446,-709,-60,2374,-709,-60,2358,-1165,-60,2382,-1165,-60,2382,-1093,-60,2366,-1077,-60,2366,-1133,-352,2382,-1141,-352,2382,-1093,-352,2366,-1085,-352,2502,-1165,-352,2502,-1133,-352,2494,-1069,-352,2398,-973,-352,2390,-1013,-352,2406,-1165,-352,2494,-1077,-60,2398,-981`
+`,-60,2390,-1013,-60,2406,-1165,-60,2494,-1165,-60,2414,-909,-352,2454,-893,-352,2414,-893,-352,2414,-909,-60,2494,-885,-60,2494,-781,-60,2566,-1053,-352,2574,-1021,-352,2558,-1013,-352,2558,-1013,-352,2542,-933,-352,2422,-957,-352,2566,-1053,-352,2422,-957,-60,2494,-997,-60,2494,-941,-60,2478,-765,-60,2494,-709,-60,2470,-709,-60,2438,-813,-60,2446,-797,-352,2502,-709,-352,2470,-709,-352,2470,-1029,-60,2486,-1045,-60,2494,-1029,-60,2582,-1125,-352,2582,-1085,-352,2518,-1053,-352,2582,-901,-352,25`
+`82,-893,-352,2518,-909,-352,2526,-861,-352,2550,-869,-352,2550,-853,-352,2526,-845,-352,2582,-957,-352,2574,-933,-352,2550,-933,-352,2582,-1005,-352,2582,-693,-60,2582,-677,-60,2558,-677,-60,2550,-693,-60,2574,-1189,-352,2582,-1189,-352,2582,-1165,-352,2558,-1149,-352,2582,-685,-352,2582,-677,-352,2558,-677,-352],"vertslength":161,"polys":[0,3,4,8,9,12,13,15,16,18,19,24,25,30,31,33,34,36,37,40,41,43,44,46,47,49,50,52,53,56,57,59,60,62,63,66,67,69,70,72,73,75,76,78,79,81,82,85,86,90,91,94,95,98,9`
+`9,104,105,109,110,112,113,115,116,118,119,122,123,125,126,129,130,132,133,135,136,138,139,141,142,145,146,149,150,153,154,157,158,160],"polyslength":44,"regions":[20,4,2,21,1,1,3,27,5,5,5,6,6,6,13,14,15,16,39,40,41,43,44,7,7,47,48,8,9,52,11,10,10,12,17,18,60,19,64,66,67,68,69,71],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,5]],[[0],[0],[0],[0],[1,4],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,9]],[[1,8],[0],[0],[1,10]],[[0],[0],[1`
+`,9]],[[0],[0],[1,13]],[[0],[0],[1,13]],[[1,12],[1,11],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,24]],[[0],[0],[0],[0],[1,23]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,32]],[[0],[0],[0],[1,31]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],`
+`[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[2070,-1141,-352,2070,-1157,-352,2126,-1141,-352,2134,-1109,-352,2294,-981,-352,2286,-949,-352,2246,-933,-352,2070,-957,-352,2070,-1109,-352,2334,-957,-60,2310,-941,-60,2070,-917,-60,2070,-1109,-60,2070,-869,-352,2086,-861,-352,2070,-845,-352,2222,-877,-60,2310,-877,-60,2302,-837,-60,2198,-677,-60,2078,-677,-60,2070,-693,-60,2070,-821,-60,2222,-877,-60,2302,-837,-60,2222,-877,-352,2310,-877,-352,2310,-853,-352,221`
+`4,-701,-352,2166,-733,-352,2094,-829,-352,2110,-877,-60,2150,-869,-60,2110,-861,-60,2310,-1141,-352,2342,-1133,-352,2342,-1093,-352,2310,-1141,-352,2342,-1093,-352,2310,-1021,-352,2142,-1189,-352,2142,-1189,-352,2318,-1189,-352,2310,-1141,-352,2310,-1141,-60,2342,-1133,-60,2358,-981,-60,2142,-1189,-60,2318,-1189,-60,2310,-1141,-60,2142,-1189,-60,2310,-1141,-60,2358,-981,-60,2238,-1053,-352,2222,-1045,-352,2158,-1085,-352,2142,-1149,-352,2150,-1101,-60,2182,-1117,-60,2262,-1029,-60,2270,-677,-352`
+`,2230,-677,-352,2310,-805,-352,2246,-717,-60,2302,-805,-60,2310,-781,-60,2278,-685,-60,2302,-957,-352,2334,-949,-352,2302,-941,-352,2310,-677,-352,2302,-677,-352,2302,-701,-352,2310,-677,-60,2302,-677,-60,2302,-701,-60,2310,-909,-60,2342,-917,-60,2342,-901,-60,2326,-1021,-352,2350,-1061,-352,2358,-981,-352,2374,-709,-60,2366,-677,-60,2334,-677,-60,2334,-797,-60,2334,-797,-60,2366,-885,-60,2382,-885,-60,2446,-709,-60,2374,-709,-60,2358,-1165,-60,2382,-1165,-60,2382,-1093,-60,2366,-1077,-60,2366,-`
+`1133,-352,2382,-1141,-352,2382,-1093,-352,2366,-1085,-352,2502,-1165,-352,2502,-1133,-352,2494,-1069,-352,2398,-973,-352,2390,-1013,-352,2406,-1165,-352,2494,-1077,-60,2398,-981,-60,2390,-1013,-60,2406,-1165,-60,2494,-1165,-60,2414,-909,-352,2454,-893,-352,2414,-893,-352,2414,-909,-60,2494,-885,-60,2494,-781,-60,2566,-1053,-352,2574,-1021,-352,2558,-1013,-352,2558,-1013,-352,2542,-933,-352,2422,-957,-352,2566,-1053,-352,2422,-957,-60,2494,-997,-60,2494,-941,-60,2478,-765,-60,2494,-709,-60,2470,-`
+`709,-60,2438,-813,-60,2446,-797,-352,2502,-709,-352,2470,-709,-352,2470,-1029,-60,2486,-1045,-60,2494,-1029,-60,2582,-1125,-352,2582,-1085,-352,2518,-1053,-352,2582,-901,-352,2582,-893,-352,2518,-909,-352,2526,-861,-352,2550,-869,-352,2550,-853,-352,2526,-845,-352,2582,-957,-352,2574,-933,-352,2550,-933,-352,2582,-1005,-352,2582,-693,-60,2582,-677,-60,2558,-677,-60,2550,-693,-60,2574,-1189,-352,2582,-1189,-352,2582,-1165,-352,2558,-1149,-352,2582,-685,-352,2582,-677,-352,2558,-677,-352],"vertsle`
+`ngth":161,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,4,7,8,9,10,11,9,11,12,13,14,15,16,17,18,19,20,21,19,21,22,23,24,19,19,22,23,25,26,27,28,29,30,30,25,27,27,28,30,31,32,33,34,35,36,37,38,39,37,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,53,55,56,57,58,59,60,61,62,63,64,65,63,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,82,84,85,86,87,88,89,90,86,86,88,89,91,92,93,91,93,94,95,96,97,95,97,98,99,100,101,101,102,103,104,99,101,101,103,104,105,106,107,108,109,105,105,107,108,110,111,112,`
+`113,114,115,116,117,118,119,120,121,119,121,122,123,124,125,126,127,128,126,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,142,144,145,146,147,148,146,148,149,150,151,152,150,152,153,154,155,156,154,156,157,158,159,160],"trislength":73,"triTopoly":[0,0,1,1,1,2,2,3,4,5,5,5,5,6,6,6,6,7,8,9,9,10,11,12,13,14,14,15,16,17,17,18,19,20,21,22,23,23,24,24,24,25,25,26,26,27,27,27,27,28,28,28,29,30,31,32,32,33,34,34,35,36,37,38,39,39,40,40,41,41,42,42,43],"baseVert":[0,4,9,13,16,19,25,3`
+`1,34,37,41,44,47,50,53,57,60,63,67,70,73,76,79,82,86,91,95,99,105,110,113,116,119,123,126,130,133,136,139,142,146,150,154,158],"vertsCount":[4,5,4,3,3,6,6,3,3,4,3,3,3,3,4,3,3,4,3,3,3,3,3,4,5,4,4,6,5,3,3,3,4,3,4,3,3,3,3,4,4,4,4,3],"baseTri":[0,2,5,7,8,9,13,17,18,19,21,22,23,24,25,27,28,29,31,32,33,34,35,36,38,41,43,45,49,52,53,54,55,57,58,60,61,62,63,64,66,68,70,72],"triCount":[2,3,2,1,1,4,4,1,1,2,1,1,1,1,2,1,1,2,1,1,1,1,1,2,3,2,2,4,3,1,1,1,2,1,2,1,1,1,1,2,2,2,2,1]},"links":{"poly":[0,14,1,18,9,2`
+`2,31,40],"cost":[457.4117736816406,276.70587158203125,320.6597900390625,480],"type":[1,1,1,1],"pos":[2126,-1141,-352,2142.941162109375,-1145.2353515625,-352,2288.823486328125,-960.2941284179688,-352,2302,-957,-352,2312.63916015625,-1026.9381103515625,-352,2326,-1021,-352,2574,-1021,-352,2582,-1005,-352],"length":4}}],["11_3",{"tileId":"11_3","tx":11,"ty":3,"mesh":{"verts":[2582,-1173,-352,2582,-1189,-352,2590,-1189,-352,2598,-1173,-352,2638,-1189,-352,2670,-1189,-352,2694,-1157,-352,2582,-1093,-`
+`352,2582,-1133,-352,2582,-1029,-352,2614,-1077,-352,2582,-981,-352,2670,-677,-60,2582,-677,-60,2582,-693,-60,2678,-693,-60,2582,-685,-352,2622,-677,-352,2582,-677,-352,2702,-789,-352,2678,-749,-352,2670,-765,-352,2686,-805,-352,2678,-1125,-352,2702,-1117,-352,2702,-1093,-352,2694,-1077,-64,2742,-1077,-64,2750,-1061,-64,2766,-909,-64,2694,-917,-64,2694,-821,-64,2726,-813,-64,2734,-781,-64,2694,-725,-64,3094,-677,-60,2702,-677,-60,2702,-693,-60,3094,-701,-60,2710,-773,-352,2726,-789,-352,2726,-765`
+`,-352,2734,-1053,-352,2726,-1077,-352,2742,-1077,-352,2742,-1021,-352,2718,-1021,-352,2718,-1037,-352,2734,-1053,-352,2742,-1021,-352,2734,-1053,-352,2742,-1077,-352,2766,-933,-352,2766,-933,-352,2726,-933,-352,2742,-1021,-352,2942,-765,-64,2726,-717,-64,2758,-797,-64,2822,-837,-64,2934,-837,-64,2942,-765,-64,3094,-765,-64,3094,-709,-64,2726,-717,-64,2878,-973,-64,2894,-965,-64,2838,-941,-64,2830,-965,-64,2878,-973,-64,2830,-965,-64,2790,-965,-64,2774,-1069,-64,2878,-1069,-64,2798,-869,-64,2798,`
+`-909,-64,2822,-909,-64,2822,-861,-64,2822,-861,-64,2822,-909,-64,2838,-941,-64,2822,-837,-64,2822,-861,-64,2838,-941,-64,2894,-965,-64,2942,-869,-64,2934,-837,-64,3094,-965,-64,3094,-869,-64,2942,-869,-64,2894,-965,-64,2894,-981,-24,2894,-1069,-24,3086,-1069,-24,3086,-981,-24,2902,-989,-64,2902,-1061,-64,3070,-1061,-64,3070,-989,-64,2950,-845,-24,3094,-845,-24,3094,-789,-24,2950,-789,-24,2958,-797,-64,2958,-837,-64,3078,-837,-64,3078,-797,-64,3094,-685,-344,3094,-677,-344,3022,-677,-344],"vertsl`
+`ength":110,"polys":[0,3,4,8,9,11,12,15,16,18,19,22,23,25,26,30,31,34,35,38,39,41,42,44,45,48,49,52,53,55,56,60,61,64,65,68,69,73,74,77,78,80,81,86,87,90,91,94,95,98,99,102,103,106,107,109],"polyslength":28,"regions":[11,8,12,15,16,38,39,5,9,42,46,47,47,47,47,1,1,3,3,2,2,2,2,4,6,7,10,56],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],`
+`[1,13]],[[0],[0],[0],[1,13]],[[1,12],[1,11],[0],[1,14]],[[0],[0],[1,13]],[[1,16],[0],[0],[1,21],[0]],[[0],[0],[0],[1,15]],[[0],[1,21],[0],[1,18]],[[1,17],[0],[0],[0],[0]],[[0],[0],[1,20],[0]],[[1,19],[0],[1,21]],[[0],[1,20],[1,17],[1,22],[0],[1,15]],[[0],[0],[1,21],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[2582,-1173,-352,2582,-1189,-352,2590,-1189,-352,2598,-1173,-352,2638,-1189,-352,2670,-1189,-352,2694,-1157,-352,2582,-1093`
+`,-352,2582,-1133,-352,2582,-1029,-352,2614,-1077,-352,2582,-981,-352,2670,-677,-60,2582,-677,-60,2582,-693,-60,2678,-693,-60,2582,-685,-352,2622,-677,-352,2582,-677,-352,2702,-789,-352,2678,-749,-352,2670,-765,-352,2686,-805,-352,2678,-1125,-352,2702,-1117,-352,2702,-1093,-352,2694,-1077,-64,2742,-1077,-64,2750,-1061,-64,2766,-909,-64,2694,-917,-64,2694,-821,-64,2726,-813,-64,2734,-781,-64,2694,-725,-64,3094,-677,-60,2702,-677,-60,2702,-693,-60,3094,-701,-60,2710,-773,-352,2726,-789,-352,2726,-7`
+`65,-352,2734,-1053,-352,2726,-1077,-352,2742,-1077,-352,2742,-1021,-352,2718,-1021,-352,2718,-1037,-352,2734,-1053,-352,2742,-1021,-352,2734,-1053,-352,2742,-1077,-352,2766,-933,-352,2766,-933,-352,2726,-933,-352,2742,-1021,-352,2942,-765,-64,2726,-717,-64,2758,-797,-64,2822,-837,-64,2934,-837,-64,2942,-765,-64,3094,-765,-64,3094,-709,-64,2726,-717,-64,2878,-973,-64,2894,-965,-64,2838,-941,-64,2830,-965,-64,2878,-973,-64,2830,-965,-64,2790,-965,-64,2774,-1069,-64,2878,-1069,-64,2798,-869,-64,279`
+`8,-909,-64,2822,-909,-64,2822,-861,-64,2822,-861,-64,2822,-909,-64,2838,-941,-64,2822,-837,-64,2822,-861,-64,2838,-941,-64,2894,-965,-64,2942,-869,-64,2934,-837,-64,3094,-965,-64,3094,-869,-64,2942,-869,-64,2894,-965,-64,2894,-981,-24,2894,-1069,-24,3086,-1069,-24,3086,-981,-24,2902,-989,-64,2902,-1061,-64,3070,-1061,-64,3070,-989,-64,2950,-845,-24,3094,-845,-24,3094,-789,-24,2950,-789,-24,2958,-797,-64,2958,-837,-64,3078,-837,-64,3078,-797,-64,3094,-685,-344,3094,-677,-344,3022,-677,-344],"vert`
+`slength":110,"tris":[0,1,2,0,2,3,4,5,6,7,8,4,4,6,7,9,10,11,12,13,14,12,14,15,16,17,18,19,20,21,19,21,22,23,24,25,26,27,28,30,26,28,28,29,30,31,32,33,31,33,34,35,36,37,35,37,38,39,40,41,42,43,44,45,46,47,45,47,48,49,50,51,49,51,52,53,54,55,57,58,59,59,60,56,56,57,59,61,62,63,61,63,64,67,68,65,65,66,67,69,70,71,73,69,71,71,72,73,74,75,76,74,76,77,78,79,80,81,82,83,85,86,81,83,84,85,81,83,85,87,88,89,87,89,90,94,91,92,92,93,94,98,95,96,96,97,98,102,99,100,100,101,102,106,103,104,104,105,106,107,108`
+`,109],"trislength":54,"triTopoly":[0,0,1,1,1,2,3,3,4,5,5,6,7,7,7,8,8,9,9,10,11,12,12,13,13,14,15,15,15,16,16,17,17,18,18,18,19,19,20,21,21,21,21,22,22,23,23,24,24,25,25,26,26,27],"baseVert":[0,4,9,12,16,19,23,26,31,35,39,42,45,49,53,56,61,65,69,74,78,81,87,91,95,99,103,107],"vertsCount":[4,5,3,4,3,4,3,5,4,4,3,3,4,4,3,5,4,4,5,4,3,6,4,4,4,4,4,3],"baseTri":[0,2,5,6,8,9,11,12,15,17,19,20,21,23,25,26,29,31,33,36,38,39,43,45,47,49,51,53],"triCount":[2,3,1,2,1,2,1,3,2,2,1,1,2,2,1,3,2,2,3,2,1,4,2,2,2,2,`
+`2,1]},"links":{"poly":[5,10,6,11,9,16,9,15,15,25,17,23],"cost":[341.6470642089844,1248,120,852.7493896484375,2568.58544921875,2707.199951171875],"type":[1,1,1,1,2,2],"pos":[2697.058837890625,-780.7647094726562,-352,2710,-773,-352,2702,-1093,-352,2726,-1077,-352,3094,-701,-60,3094,-709,-64,2726.4794921875,-693.4995727539062,-60,2726,-717,-64,2939.46337890625,-787.8292846679688,-64,2950,-789,-24,2887.60009765625,-968.2000122070312,-64,2894,-981,-24],"length":6}}],["12_3",{"tileId":"12_3","tx":12,"`
+`ty":3,"mesh":{"verts":[3134,-789,-64,3110,-789,-64,3110,-853,-64,3094,-973,-64,3102,-1069,-64,3134,-1069,-64,3134,-789,-64,3110,-853,-64,3110,-853,-64,3094,-861,-64,3094,-973,-64,3094,-773,-64,3110,-789,-64,3134,-789,-64,3150,-757,-60,3094,-677,-60,3526,-677,-60,3094,-677,-60,3150,-757,-60,3150,-757,-60,3158,-1069,-60,3526,-1069,-60,3526,-677,-60,3166,-901,-335,3214,-909,-330,3222,-893,-341,3198,-813,-344,3166,-829,-344,3094,-677,-344,3094,-685,-344,3158,-685,-344,3502,-677,-352,3158,-685,-344,3`
+`166,-725,-344,3198,-741,-344,3502,-677,-352,3198,-813,-344,3222,-893,-341,3238,-901,-344,3502,-1005,-352,3502,-677,-352,3198,-741,-344,3238,-901,-344,3238,-1005,-352,3502,-1005,-352,3270,-1069,-304,3254,-1037,-304,3214,-1029,-304,3166,-1069,-304,3214,-1029,-304,3214,-909,-330,3166,-901,-335,3166,-1069,-304,3166,-741,-296,3166,-813,-296,3182,-813,-296,3182,-741,-296,3286,-1013,-304,3254,-1037,-304,3270,-1069,-304,3446,-1077,-304,3446,-1037,-304,3438,-1013,-304,3502,-1069,-304,3502,-1037,-304,3446`
+`,-1037,-304,3446,-1077,-304,3542,-677,-352,3542,-1189,-352,3550,-1189,-352,3550,-677,-352],"vertslength":71,"polys":[0,2,3,7,8,10,11,15,16,18,19,22,23,27,28,31,32,35,36,41,42,44,45,48,49,52,53,56,57,62,63,66,67,70],"polyslength":17,"regions":[5,5,5,1,1,1,2,2,2,2,2,4,4,6,3,3,8],"neighbors":[[[1,3],[0],[1,1]],[[0],[0],[0],[1,0],[1,2]],[[0],[0],[1,1]],[[0],[1,0],[0],[1,4],[0]],[[0],[1,3],[1,5]],[[0],[0],[0],[1,4]],[[1,12],[0],[1,9],[0],[0]],[[0],[0],[1,8],[0]],[[0],[0],[1,9],[1,7]],[[1,6],[0],[1,10`
+`],[0],[1,8],[0]],[[0],[0],[1,9]],[[1,14],[0],[1,12],[0]],[[0],[1,6],[0],[1,11]],[[0],[0],[0],[0]],[[0],[1,11],[0],[1,15],[0],[0]],[[0],[0],[1,14],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[3134,-789,-64,3110,-789,-64,3110,-853,-64,3094,-973,-64,3102,-1069,-64,3134,-1069,-64,3134,-789,-64,3110,-853,-64,3110,-853,-64,3094,-861,-64,3094,-973,-64,3094,-773,-64,3110,-789,-64,3134,-789,-60,3150,-757,-60,3094,-677,-60,3526,-677,-60,3094,-677,-60,3150,-757,-60,3150,-757,-60,3158,-1069,-60,3526,-1069,-6`
+`0,3526,-677,-60,3166,-901,-344,3214,-909,-344,3222,-893,-344,3198,-813,-344,3166,-829,-344,3094,-677,-344,3094,-685,-344,3158,-685,-344,3226.800048828125,-683.4000244140625,-344,3249.7333984375,-682.8666381835938,-352,3502,-677,-352,3230,-677,-352,3207.333251953125,-677,-344,3158,-685,-344,3166,-725,-344,3198,-741,-344,3244.769287109375,-731.1538696289062,-352,3502,-677,-352,3249.7333984375,-682.8666381835938,-352,3226.800048828125,-683.4000244140625,-344,3218,-705,-344,3198,-813,-344,3222,-893,`
+`-344,3238,-901,-352,3502,-1005,-352,3502,-677,-352,3244.769287109375,-731.1538696289062,-352,3198,-741,-344,3238,-901,-352,3238,-1005,-352,3502,-1005,-352,3270,-1069,-304,3254,-1037,-304,3214,-1029,-304,3166,-1069,-304,3214,-1029,-304,3214,-969,-304,3214,-909,-330,3166,-901,-335,3166,-922,-330,3166,-964,-304,3166,-1069,-304,3166,-741,-296,3166,-813,-296,3182,-813,-296,3182,-741,-296,3286,-1013,-304,3254,-1037,-304,3270,-1069,-304,3446,-1077,-304,3446,-1037,-304,3438,-1013,-304,3502,-1069,-304,35`
+`02,-1037,-304,3446,-1037,-304,3446,-1077,-304,3542,-677,-352,3542,-1189,-352,3550,-1189,-352,3550,-677,-352],"vertslength":83,"tris":[0,1,2,3,4,5,6,7,3,3,5,6,8,9,10,11,12,13,11,13,14,11,14,15,16,17,18,19,20,21,19,21,22,23,24,25,26,27,23,23,25,26,28,29,30,35,28,30,35,30,31,34,35,31,34,31,32,32,33,34,39,40,41,39,41,43,41,42,43,36,42,43,39,38,43,36,37,43,38,37,43,44,45,46,49,50,44,49,44,46,48,49,46,46,47,48,51,52,53,54,55,56,54,56,57,60,61,62,60,62,63,59,60,63,58,59,63,58,63,64,68,65,66,66,67,68,72`
+`,73,74,69,70,71,74,69,71,71,72,74,75,76,77,75,77,78,82,79,80,80,81,82],"trislength":50,"triTopoly":[0,1,1,1,2,3,3,3,4,5,5,6,6,6,7,7,7,7,7,7,8,8,8,8,8,8,8,9,9,9,9,9,10,11,11,12,12,12,12,12,13,13,14,14,14,14,15,15,16,16],"baseVert":[0,3,8,11,16,19,23,28,36,44,51,54,58,65,69,75,79],"vertsCount":[3,5,3,5,3,4,5,8,8,7,3,4,7,4,6,4,4],"baseTri":[0,1,4,5,8,9,11,14,20,27,32,33,35,40,42,46,48],"triCount":[1,3,1,3,1,2,3,6,7,5,1,2,5,2,4,2,2]},"links":{"poly":[6,13],"cost":[3532.800048828125],"type":[2],"pos"`
+`:[3185.199951171875,-819.4000244140625,-344,3182,-813,-296],"length":1}}],["0_4",{"tileId":"0_4","tx":0,"ty":4,"mesh":{"verts":[-3002,-221,-408,-2994,-229,-408,-2970,-237,-408,-2866,-221,-408,-2866,-261,-408,-2858,-229,-408,-2866,-221,-408,-2970,-237,-408,-2978,-269,-408,-3002,-261,-408,-2994,-229,-408,-3002,-221,-408,-3002,-221,-408,-3010,-165,-408,-3018,-165,-408,-3010,-293,-416,-3002,-261,-408,-2978,-269,-408,-3002,-261,-408,-3010,-293,-416,-2850,-277,-408,-2866,-261,-408,-2978,-269,-408,-301`
+`0,-293,-416,-2706,-269,-416,-2818,-261,-408,-2850,-277,-408,-2850,-277,-408,-3010,-293,-416,-3042,-301,-424,-3042,-509,-424,-2738,-501,-416,-2706,-269,-416,-3026,-517,-172,-3026,-677,-172,-2538,-677,-164,-2538,-517,-172,-2866,-213,-272,-2866,-165,-272,-3002,-165,-272,-3002,-213,-272,-2930,-581,-416,-2994,-581,-416,-2994,-677,-416,-2594,-677,-416,-2722,-605,-416,-2594,-677,-416,-2594,-589,-416,-2722,-605,-416,-2978,-165,-408,-2978,-189,-408,-2890,-189,-408,-2882,-165,-408,-2866,-221,-408,-2858,-2`
+`29,-408,-2826,-229,-408,-2866,-165,-408,-2586,-277,-416,-2586,-189,-416,-2602,-165,-416,-2866,-165,-408,-2826,-229,-408,-2706,-269,-416,-2826,-229,-408,-2818,-261,-408,-2706,-269,-416,-2810,-533,-416,-2746,-541,-416,-2746,-525,-416,-2730,-525,-314,-2730,-541,-323,-2650,-557,-330,-2642,-525,-313,-2722,-501,-312,-2634,-501,-311,-2634,-485,-311,-2722,-469,-300,-2722,-453,-370,-2650,-469,-370,-2618,-461,-365,-2610,-373,-370,-2706,-365,-370,-2594,-293,-365,-2650,-277,-370,-2698,-277,-365,-2706,-365,-`
+`370,-2610,-373,-370,-2602,-565,-416,-2594,-533,-416,-2618,-525,-416,-2626,-565,-416,-2562,-429,-400,-2562,-501,-400,-2538,-501,-400,-2618,-501,-416,-2594,-501,-416,-2586,-421,-416,-2586,-317,-416,-2538,-165,-400,-2570,-165,-400,-2570,-309,-400,-2562,-429,-400,-2538,-501,-400,-2570,-309,-400,-2586,-317,-416,-2586,-421,-416,-2562,-429,-400,-2570,-573,-256,-2570,-677,-256,-2538,-677,-256,-2538,-573,-256,-2570,-501,-256,-2538,-501,-256,-2538,-437,-256,-2570,-437,-256,-2562,-677,-400,-2538,-677,-400,`
+`-2538,-589,-400,-2562,-573,-400,-2538,-533,-256,-2538,-525,-256,-2562,-525,-256],"vertslength":122,"polys":[0,3,4,8,9,11,12,16,17,19,20,23,24,26,27,32,33,36,37,40,41,45,46,48,49,52,53,56,57,62,63,65,66,68,69,72,73,76,77,81,82,86,87,90,91,93,94,97,98,102,103,106,107,110,111,114,115,118,119,121],"polyslength":30,"regions":[1,1,1,1,1,1,1,1,2,9,7,7,15,3,3,3,17,11,10,4,5,12,8,8,8,8,13,14,20,21],"neighbors":[[[1,2],[0],[1,1],[0]],[[0],[1,13],[1,0],[0],[1,5]],[[0],[1,0],[1,3]],[[0],[0],[0],[1,4],[1,2]]`
+`,[[0],[1,3],[1,5]],[[0],[1,1],[1,4],[1,7]],[[1,15],[0],[1,7]],[[1,5],[0],[0],[0],[0],[1,6]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,11],[0]],[[0],[0],[1,10]],[[0],[0],[0],[0]],[[1,1],[0],[1,14],[0]],[[0],[0],[0],[1,13],[1,15],[0]],[[0],[1,6],[1,14]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,20],[0]],[[0],[0],[0],[1,19],[0]],[[0],[0],[0],[0]],[[0],[0],[1,24]],[[0],[0],[1,25],[0]],[[0],[0],[1,25],[1,22],[0]],[[0],[1,23],[0],[1,24]],[[0],[0],[0],[0]],[[0],[0],[0]`
+`,[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[-3002,-221,-408,-2994,-229,-408,-2970,-237,-408,-2866,-221,-408,-2866,-261,-408,-2858,-229,-408,-2866,-221,-408,-2970,-237,-408,-2978,-269,-408,-3002,-261,-408,-2994,-229,-408,-3002,-221,-408,-3002,-221,-408,-3010,-165,-408,-3018,-165,-408,-3011.333251953125,-271.6666564941406,-408,-3010,-293,-416,-3002,-261,-408,-2978,-269,-408,-3002,-261,-408,-3010,-293,-416,-2850,-277,-408,-2866,-261,-408,-2978,-269,-408,-3010,-293,-416,-2706,-269,-41`
+`6,-2818,-261,-416,-2850,-277,-408,-2808.857177734375,-274.71429443359375,-416,-2850,-277,-408,-3010,-293,-416,-3026,-297,-408,-3042,-301,-424,-3042,-509,-424,-3018.615478515625,-508.3846130371094,-416,-2738,-501,-416,-2706,-269,-416,-2808.857177734375,-274.71429443359375,-416,-2838,-281,-408,-3030,-305,-416,-3026,-517,-172,-3026,-677,-172,-3002.761962890625,-677,-164,-2538,-677,-164,-2538,-562.7142944335938,-164,-2538,-517,-172,-3014,-569,-164,-2866,-213,-272,-2866,-165,-272,-3002,-165,-272,-300`
+`2,-213,-272,-2930,-581,-416,-2994,-581,-416,-2994,-677,-416,-2594,-677,-416,-2722,-605,-416,-2594,-677,-416,-2594,-589,-416,-2722,-605,-416,-2978,-165,-408,-2978,-189,-408,-2890,-189,-408,-2882,-165,-408,-2866,-221,-408,-2858,-229,-408,-2826,-229,-408,-2866,-165,-408,-2586,-277,-416,-2586,-189,-416,-2602,-165,-416,-2800,-165,-416,-2822,-165,-408,-2866,-165,-408,-2826,-229,-408,-2806,-235.6666717529297,-416,-2706,-269,-416,-2826,-229,-408,-2818,-261,-416,-2706,-269,-416,-2806,-235.6666717529297,-`
+`416,-2810,-533,-416,-2746,-541,-416,-2746,-525,-416,-2730,-525,-314,-2730,-541,-314,-2650,-557,-317,-2642,-525,-313,-2722,-501,-311,-2634,-501,-311,-2634,-485,-311,-2722,-469,-300,-2722,-485,-300,-2710,-489,-311,-2722,-453,-370,-2650,-469,-370,-2618,-461,-365,-2610,-373,-370,-2706,-365,-370,-2594,-293,-365,-2650,-277,-370,-2698,-277,-370,-2706,-365,-370,-2610,-373,-365,-2602,-565,-416,-2594,-533,-416,-2618,-525,-416,-2626,-565,-416,-2562,-429,-400,-2562,-501,-400,-2538,-501,-400,-2618,-501,-416,`
+`-2594,-501,-416,-2588,-441,-416,-2586,-421,-408,-2586,-337.79998779296875,-408,-2586,-317,-400,-2590,-340,-416,-2538,-165,-400,-2570,-165,-400,-2570,-309,-400,-2562,-429,-400,-2538,-501,-400,-2570,-309,-400,-2586,-317,-400,-2586,-337.79998779296875,-408,-2586,-421,-408,-2562,-429,-400,-2570,-573,-256,-2570,-677,-256,-2538,-677,-256,-2538,-573,-256,-2570,-501,-256,-2538,-501,-256,-2538,-437,-256,-2570,-437,-256,-2562,-677,-400,-2538,-677,-400,-2538,-589,-400,-2562,-573,-400,-2538,-533,-256,-2538,`
+`-525,-256,-2562,-525,-256],"vertslength":142,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,4,7,8,9,10,11,15,16,17,15,17,12,12,13,14,12,14,15,18,19,20,21,22,23,21,23,24,26,27,28,25,26,28,35,36,37,37,29,38,29,30,38,34,35,38,37,35,38,34,38,39,32,33,39,34,33,39,32,31,39,38,30,39,31,30,39,42,43,44,42,44,46,40,41,46,42,41,46,44,45,46,40,45,46,50,47,48,48,49,50,51,52,53,55,51,53,53,54,55,56,57,58,59,60,61,59,61,62,63,64,65,63,65,66,71,72,73,70,71,73,70,73,74,67,68,69,70,74,75,75,67,69,69,70,75,79,76,77,77,78,79,80,8`
+`1,82,83,84,85,83,85,86,91,87,92,87,88,92,88,89,92,91,90,92,89,90,92,93,94,95,95,96,97,93,95,97,98,99,100,101,102,98,98,100,101,103,104,105,103,105,106,107,108,109,114,115,116,110,111,112,113,114,116,110,112,113,110,113,116,117,118,119,119,120,121,117,119,121,122,123,124,124,125,126,122,124,126,130,127,128,128,129,130,134,131,132,132,133,134,135,136,137,135,137,138,139,140,141],"trislength":86,"triTopoly":[0,0,1,1,1,2,3,3,3,3,4,5,5,6,6,7,7,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,9,9,10,10,10,11,12,12,13,13`
+`,14,14,14,14,14,14,14,15,15,16,17,17,18,18,18,18,18,19,19,19,20,20,20,21,21,22,23,23,23,23,23,24,24,24,25,25,25,26,26,27,27,28,28,29],"baseVert":[0,4,9,12,18,21,25,29,40,47,51,56,59,63,67,76,80,83,87,93,98,103,107,110,117,122,127,131,135,139],"vertsCount":[4,5,3,6,3,4,4,11,7,4,5,3,4,4,9,4,3,4,6,5,5,4,3,7,5,5,4,4,4,3],"baseTri":[0,2,5,6,10,11,13,15,26,32,34,37,38,40,42,49,51,52,54,59,62,65,67,68,73,76,79,81,83,85],"triCount":[2,3,1,4,1,2,2,11,6,2,3,1,2,2,7,2,1,2,5,3,3,2,1,5,3,3,2,2,2,1]},"links":`
+`{"poly":[7,20,12,13,14,24,18,19,19,23],"cost":[4023.48828125,384,768,5478.5244140625,3971.958740234375],"type":[2,1,2,2,2],"pos":[-2706.93359375,-275.7677917480469,-416,-2698,-277,-365,-2882,-165,-408,-2866,-165,-408,-2586,-277,-416,-2570,-277,-400,-2634,-485,-311,-2638.705810546875,-466.1764831542969,-368.23529052734375,-2618,-461,-365,-2611.247802734375,-462.17431640625,-416],"length":5}}],["1_4",{"tileId":"1_4","tx":1,"ty":4,"mesh":{"verts":[-2538,-589,-400,-2538,-677,-400,-2522,-677,-400,-25`
+`22,-589,-400,-2538,-677,-164,-2530,-677,-164,-2538,-669,-164,-2538,-653,-164,-2522,-645,-164,-2538,-629,-164,-2538,-613,-164,-2522,-605,-164,-2538,-589,-164,-2538,-573,-350,-2522,-573,-350,-2538,-557,-350,-2538,-573,-164,-2522,-565,-164,-2538,-549,-164,-2538,-533,-172,-2522,-525,-172,-2538,-517,-172,-2538,-501,-400,-2522,-501,-400,-2514,-421,-400,-2522,-165,-400,-2538,-165,-400,-2538,-461,-256,-2522,-453,-256,-2538,-437,-256,-2514,-165,-30,-2514,-677,-30,-2026,-677,-36,-2026,-165,-36],"vertsleng`
+`th":34,"polys":[0,3,4,6,7,9,10,12,13,15,16,18,19,21,22,26,27,29,30,33],"polyslength":10,"regions":[2,3,4,5,6,7,8,9,10,1],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2538,-589,-400,-2538,-677,-400,-2522,-677,-400,-2522,-589,-400,-2538,-677,-164,-2530,-677,-164,-2538,-669,-164,-2538,-653,-164,-2522,-645,-164,-2538,-629,-164,-2538,-613,-164,-2522,-605,-1`
+`64,-2538,-589,-164,-2538,-573,-350,-2522,-573,-350,-2538,-557,-350,-2538,-573,-164,-2522,-565,-164,-2538,-549,-164,-2538,-533,-172,-2522,-525,-172,-2538,-517,-172,-2538,-501,-400,-2522,-501,-400,-2514,-421,-400,-2522,-165,-400,-2538,-165,-400,-2538,-461,-256,-2522,-453,-256,-2538,-437,-256,-2514,-165,-36,-2514,-677,-36,-2026,-677,-36,-2026,-165,-36],"vertslength":34,"tris":[3,0,1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,24,25,26,22,24,26,27,28,29,33,30,31,31,32,33],"trislen`
+`gth":14,"triTopoly":[0,0,1,2,3,4,5,6,7,7,7,8,9,9],"baseVert":[0,4,7,10,13,16,19,22,27,30],"vertsCount":[4,3,3,3,3,3,3,5,3,4],"baseTri":[0,2,3,4,5,6,7,8,11,12],"triCount":[2,1,1,1,1,1,1,3,1,2]},"links":{"poly":[0,4,1,2,2,3,3,5,5,6],"cost":[4134,384,384,384,480],"type":[2,1,1,1,1],"pos":[-2538,-589,-400,-2538,-573,-350,-2538,-669,-164,-2538,-653,-164,-2538,-629,-164,-2538,-613,-164,-2538,-589,-164,-2538,-573,-164,-2538,-549,-164,-2538,-533,-172],"length":5}}],["2_4",{"tileId":"2_4","tx":2,"ty":4,"`
+`mesh":{"verts":[-2026,-165,-36,-2026,-677,-36,-1690,-677,-36,-1690,-165,-36,-1682,-165,118,-1682,-677,118,-1514,-677,112,-1514,-165,112],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2026,-165,-36,-2026,-677,-36,-1690,-677,-36,-1690,-165,-36,-1682,-165,112,-1682,-677,112,-1514,-677,112,-1514,-165,112],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"ve`
+`rtsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_4",{"tileId":"3_4","tx":3,"ty":4,"mesh":{"verts":[-1394,-285,112,-1378,-293,112,-1386,-277,112,-1410,-253,112,-1362,-253,112,-1378,-253,112,-1386,-277,112,-1378,-293,112,-1410,-309,112,-1394,-285,112,-1410,-253,112,-1514,-237,112,-1362,-253,112,-1378,-293,112,-1386,-317,112,-1362,-677,112,-1362,-677,112,-1386,-317,112,-1410,-309,112,-1514,-677,112,-1362,-677,112,-1410,-309,112,-151`
+`4,-237,112,-1410,-253,112,-1362,-253,112,-1362,-165,112,-1514,-165,112,-1514,-237,112,-1122,-677,-52,-1130,-645,-52,-1146,-637,-52,-1354,-677,-52,-1130,-613,-52,-1138,-525,-52,-1170,-525,-52,-1146,-637,-52,-1130,-613,-52,-1170,-525,-52,-1202,-493,-52,-1202,-493,-52,-1202,-333,-52,-1354,-317,-52,-1354,-677,-52,-1146,-637,-52,-1202,-493,-52,-1354,-317,-52,-1210,-205,-52,-1194,-181,-52,-1186,-165,-52,-1354,-165,-52,-1170,-325,-52,-1194,-213,-52,-1210,-205,-52,-1354,-165,-52,-1354,-317,-52,-1202,-33`
+`3,-52,-1186,-165,-52,-1194,-181,-52,-1194,-213,-52,-1114,-325,-52,-1002,-349,-52,-1002,-165,-52,-1114,-325,-52,-1098,-349,-52,-1002,-349,-52,-1194,-213,-52,-1170,-325,-52,-1114,-325,-52,-1186,-341,28,-1186,-485,28,-1122,-485,28,-1122,-341,28,-1170,-349,-52,-1170,-469,-52,-1130,-469,-52,-1130,-349,-52,-1114,-621,-52,-1130,-645,-52,-1122,-677,-52,-1002,-677,-52,-1138,-525,-52,-1130,-613,-52,-1114,-621,-52,-1098,-485,-52,-1002,-349,-52,-1098,-349,-52,-1098,-485,-52,-1098,-485,-52,-1114,-621,-52,-10`
+`02,-677,-52,-1002,-349,-52],"vertslength":91,"polys":[0,3,4,7,8,11,12,15,16,18,19,22,23,27,28,31,32,34,35,38,39,41,42,45,46,49,50,55,56,61,62,64,65,67,68,71,72,75,76,79,80,83,84,86,87,90],"polyslength":23,"regions":[4,4,4,4,4,4,6,1,1,1,1,1,2,2,3,3,3,7,8,5,5,5,5],"neighbors":[[[0],[1,1],[0],[1,2]],[[0],[0],[1,0],[1,3]],[[0],[1,0],[1,6],[1,5]],[[1,1],[0],[1,4],[0]],[[1,3],[0],[1,5]],[[0],[1,4],[1,2],[0]],[[0],[0],[0],[0],[1,2]],[[1,19],[0],[1,11],[0]],[[1,20],[0],[1,9]],[[0],[1,8],[0],[1,11]],[[0]`
+`,[1,13],[1,11]],[[1,7],[1,9],[1,10],[0]],[[0],[1,14],[0],[1,13]],[[1,16],[0],[1,12],[0],[1,10],[0]],[[1,12],[0],[1,16],[1,15],[0],[0]],[[0],[1,21],[1,14]],[[1,13],[0],[1,14]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,7],[0],[1,22]],[[1,8],[0],[1,22],[0]],[[1,15],[0],[1,22]],[[1,20],[1,19],[0],[1,21]]]},"detail":{"verts":[-1394,-285,112,-1378,-293,112,-1386,-277,112,-1410,-253,112,-1362,-253,112,-1378,-253,112,-1386,-277,112,-1378,-293,112,-1410,-309,112,-1394,-285,112,-1410,-253,112,-1514,-237`
+`,112,-1362,-253,112,-1378,-293,112,-1386,-317,112,-1362,-677,112,-1362,-677,112,-1386,-317,112,-1410,-309,112,-1514,-677,112,-1362,-677,112,-1410,-309,112,-1514,-237,112,-1410,-253,112,-1362,-253,112,-1362,-165,112,-1514,-165,112,-1514,-237,112,-1122,-677,-52,-1130,-645,-52,-1146,-637,-52,-1354,-677,-52,-1130,-613,-52,-1138,-525,-52,-1170,-525,-52,-1146,-637,-52,-1130,-613,-52,-1170,-525,-52,-1202,-493,-52,-1202,-493,-52,-1202,-333,-52,-1354,-317,-52,-1354,-677,-52,-1146,-637,-52,-1202,-493,-52,`
+`-1354,-317,-52,-1210,-205,-52,-1194,-181,-52,-1186,-165,-52,-1354,-165,-52,-1170,-325,-52,-1194,-213,-52,-1210,-205,-52,-1354,-165,-52,-1354,-317,-52,-1202,-333,-52,-1186,-165,-52,-1194,-181,-52,-1194,-213,-52,-1114,-325,-52,-1002,-349,-52,-1002,-165,-52,-1114,-325,-52,-1098,-349,-52,-1002,-349,-52,-1194,-213,-52,-1170,-325,-52,-1114,-325,-52,-1186,-341,28,-1186,-485,28,-1122,-485,28,-1122,-341,28,-1170,-349,-52,-1170,-469,-52,-1130,-469,-52,-1130,-349,-52,-1114,-621,-52,-1130,-645,-52,-1122,-67`
+`7,-52,-1002,-677,-52,-1138,-525,-52,-1130,-613,-52,-1114,-621,-52,-1098,-485,-52,-1002,-349,-52,-1098,-349,-52,-1098,-485,-52,-1098,-485,-52,-1114,-621,-52,-1002,-677,-52,-1002,-349,-52],"vertslength":91,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,16,17,18,19,20,21,19,21,22,23,24,25,26,27,23,23,25,26,28,29,30,28,30,31,32,33,34,35,36,37,35,37,38,39,40,41,42,43,44,42,44,45,46,47,48,46,48,49,50,51,52,55,50,52,54,55,52,52,53,54,56,57,58,56,58,59,59,60,61,56,59,61,62,63,64,65,66,`
+`67,71,68,69,69,70,71,75,72,73,73,74,75,76,77,78,76,78,79,80,81,82,80,82,83,84,85,86,87,88,89,87,89,90],"trislength":45,"triTopoly":[0,0,1,1,2,2,3,3,4,5,5,6,6,6,7,7,8,9,9,10,11,11,12,12,13,13,13,13,14,14,14,14,15,16,17,17,18,18,19,19,20,20,21,22,22],"baseVert":[0,4,8,12,16,19,23,28,32,35,39,42,46,50,56,62,65,68,72,76,80,84,87],"vertsCount":[4,4,4,4,3,4,5,4,3,4,3,4,4,6,6,3,3,4,4,4,4,3,4],"baseTri":[0,2,4,6,8,9,11,14,16,17,19,20,22,24,28,32,33,34,36,38,40,42,43],"triCount":[2,2,2,2,1,2,3,2,1,2,1,2,`
+`2,4,4,1,1,2,2,2,2,1,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_4",{"tileId":"4_4","tx":4,"ty":4,"mesh":{"verts":[-1002,-165,-52,-1002,-677,-52,-962,-677,-52,-962,-165,-52,-954,-517,-528,-938,-501,-528,-938,-437,-528,-954,-421,-528,-938,-165,-416,-938,-677,-416,-930,-677,-416,-930,-165,-416,-914,-389,-640,-914,-525,-640,-858,-533,-640,-730,-525,-640,-738,-397,-640,-642,-677,-768,-666,-621,-768,-754,-621,-768,-914,-677,-768,-770,-605,-768,-770,-549,-768,-914,-549,-768,-7`
+`54,-621,-768,-770,-605,-768,-914,-549,-768,-914,-677,-768,-866,-605,-640,-834,-597,-648,-826,-549,-656,-858,-533,-640,-914,-677,-640,-866,-677,-640,-866,-605,-640,-866,-605,-640,-858,-533,-640,-914,-525,-640,-914,-677,-640,-914,-165,-288,-914,-677,-288,-906,-677,-288,-906,-165,-288,-906,-381,-488,-914,-413,-488,-882,-397,-488,-882,-397,-488,-762,-421,-488,-762,-405,-488,-906,-381,-488,-890,-677,-416,-866,-677,-416,-850,-637,-416,-890,-597,-416,-538,-229,-408,-514,-237,-408,-490,-165,-408,-890,-5`
+`97,-416,-850,-637,-416,-826,-637,-416,-810,-589,-416,-810,-589,-416,-826,-637,-416,-810,-677,-416,-802,-509,-416,-810,-589,-416,-810,-677,-416,-674,-677,-416,-802,-509,-416,-754,-301,-416,-762,-333,-416,-802,-509,-416,-762,-333,-416,-842,-349,-416,-890,-501,-416,-890,-501,-416,-842,-349,-416,-874,-277,-416,-490,-493,-416,-490,-165,-408,-522,-261,-408,-490,-493,-416,-522,-261,-408,-546,-253,-408,-754,-301,-416,-802,-509,-416,-674,-677,-416,-890,-165,-416,-890,-501,-416,-874,-277,-416,-890,-165,-4`
+`16,-874,-277,-416,-866,-253,-416,-890,-165,-416,-866,-253,-416,-778,-237,-416,-490,-165,-408,-890,-165,-416,-778,-237,-416,-538,-229,-408,-546,-253,-408,-538,-229,-408,-778,-237,-416,-754,-301,-416,-890,-517,-340,-890,-581,-340,-826,-581,-340,-818,-517,-340,-882,-525,-414,-882,-573,-414,-834,-573,-414,-834,-525,-414,-786,-325,-340,-770,-301,-340,-786,-253,-340,-858,-277,-340,-842,-333,-340,-850,-653,-368,-850,-677,-368,-826,-677,-368,-826,-653,-368,-794,-317,-414,-786,-285,-414,-794,-261,-414,-8`
+`42,-285,-414,-834,-317,-414,-826,-549,-656,-834,-597,-648,-666,-597,-760,-658,-549,-768,-730,-525,-640,-666,-525,-640,-674,-397,-640,-738,-397,-640,-738,-421,-488,-722,-429,-488,-714,-413,-488,-490,-317,-640,-602,-317,-640,-602,-357,-640,-634,-389,-640,-674,-397,-640,-666,-525,-640,-490,-317,-640,-602,-357,-640,-634,-389,-640,-666,-525,-640,-490,-525,-640,-658,-549,-768,-666,-597,-760,-650,-605,-768,-490,-549,-768,-650,-605,-768,-666,-621,-768,-642,-677,-768,-490,-677,-768,-490,-549,-768,-618,-6`
+`77,-416,-490,-677,-416,-490,-549,-416,-490,-421,-488,-490,-381,-488,-618,-381,-488,-490,-341,-488,-490,-325,-488,-538,-333,-488],"vertslength":166,"polys":[0,3,4,7,8,11,12,16,17,20,21,23,24,27,28,31,32,34,35,38,39,42,43,45,46,49,50,53,54,56,57,60,61,63,64,67,68,70,71,74,75,77,78,80,81,86,87,89,90,92,93,95,96,99,100,103,104,107,108,111,112,116,117,120,121,125,126,129,130,133,134,136,137,139,140,142,143,147,148,151,152,156,157,159,160,162,163,165],"polyslength":44,"regions":[12,16,17,3,5,5,5,8,8,8`
+`,18,19,19,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,9,13,10,20,14,11,4,22,2,2,2,6,6,7,15,25],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,9],[0],[1,34],[0]],[[1,40],[0],[1,6],[0]],[[0],[0],[1,6]],[[0],[1,5],[0],[1,4]],[[0],[1,33],[0],[1,9]],[[0],[0],[1,9]],[[1,7],[1,3],[0],[1,8]],[[0],[0],[0],[0]],[[0],[0],[1,12]],[[0],[0],[0],[1,11]],[[0],[0],[1,15],[0]],[[0],[0],[1,26]],[[1,13],[0],[1,16],[0]],[[1,15],[0],[1,17]],[[0],[1,16],[0],[1,22]],[[1,22],[0],[1,19]],[[1,18],[0],[1,20],`
+`[0]],[[1,19],[0],[1,23]],[[0],[0],[1,22]],[[1,21],[0],[1,27],[1,18],[1,17],[0]],[[0],[1,20],[1,24]],[[1,23],[0],[1,25]],[[1,24],[0],[1,26]],[[0],[1,25],[1,27],[1,14]],[[0],[1,26],[0],[1,22]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[1,7],[0],[1,39],[0]],[[0],[1,37],[0],[1,3]],[[0],[0],[0]],[[0],[0],[1,38]],[[0],[1,34],[1,38]],[[1,36],[0],[1,37],[0],[0]],[[1,33],[0],[1,40],[0]],[[0],[1,4],[0],[0],[1,39]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0`
+`],[0]]]},"detail":{"verts":[-1002,-165,-52,-1002,-677,-52,-962,-677,-52,-962,-165,-52,-954,-517,-528,-938,-501,-528,-938,-437,-528,-954,-421,-528,-938,-165,-416,-938,-677,-416,-930,-677,-416,-930,-165,-416,-914,-389,-640,-914,-525,-640,-858,-533,-640,-730,-525,-640,-738,-397,-640,-642,-677,-768,-666,-621,-768,-754,-621,-768,-914,-677,-768,-770,-605,-768,-770,-549,-768,-914,-549,-768,-754,-621,-768,-770,-605,-768,-914,-549,-768,-914,-677,-768,-866,-605,-640,-834,-597,-648,-831.3333129882812,-581,`
+`-656,-826,-549,-656,-842,-541,-656,-858,-533,-640,-914,-677,-640,-866,-677,-640,-866,-605,-640,-866,-605,-640,-858,-533,-640,-914,-525,-640,-914,-677,-640,-914,-165,-288,-914,-677,-288,-906,-677,-288,-906,-165,-288,-906,-381,-488,-914,-413,-488,-882,-397,-488,-882,-397,-488,-762,-421,-488,-762,-405,-488,-906,-381,-488,-890,-677,-416,-866,-677,-416,-850,-637,-416,-890,-597,-416,-538,-229,-408,-514,-237,-408,-490,-165,-408,-890,-597,-416,-850,-637,-416,-826,-637,-416,-810,-589,-416,-810,-589,-416,`
+`-826,-637,-416,-810,-677,-416,-802,-509,-416,-810,-589,-416,-810,-677,-416,-674,-677,-416,-802,-509,-416,-754,-301,-416,-762,-333,-416,-802,-509,-416,-762,-333,-416,-842,-349,-416,-890,-501,-416,-890,-501,-416,-842,-349,-416,-874,-277,-416,-490,-493,-416,-490,-282.1428527832031,-416,-490,-258.71429443359375,-408,-490,-165,-408,-522,-261,-408,-518.7999877929688,-284.20001220703125,-416,-490,-493,-416,-518.7999877929688,-284.20001220703125,-416,-522,-261,-408,-546,-253,-408,-569.111083984375,-258.`
+`3333435058594,-416,-754,-301,-416,-802,-509,-416,-674,-677,-416,-890,-165,-416,-890,-501,-416,-874,-277,-416,-890,-165,-416,-874,-277,-416,-866,-253,-416,-890,-165,-416,-866,-253,-416,-778,-237,-416,-490,-165,-408,-560.5882568359375,-165,-408,-584.11767578125,-165,-416,-890,-165,-416,-778,-237,-416,-581.6363525390625,-230.4545440673828,-416,-538,-229,-408,-546,-253,-408,-538,-229,-408,-581.6363525390625,-230.4545440673828,-416,-778,-237,-416,-754,-301,-416,-569.111083984375,-258.3333435058594,-4`
+`16,-890,-517,-340,-890,-581,-340,-826,-581,-340,-818,-517,-340,-882,-525,-414,-882,-573,-414,-834,-573,-414,-834,-525,-414,-786,-325,-340,-770,-301,-340,-786,-253,-340,-858,-277,-340,-842,-333,-340,-850,-653,-368,-850,-677,-368,-826,-677,-368,-826,-653,-368,-794,-317,-414,-786,-285,-414,-794,-261,-414,-842,-285,-414,-834,-317,-414,-826,-549,-664,-834,-597,-664,-771,-597,-696,-687,-597,-760,-666,-597,-760,-663.3333129882812,-581,-768,-658,-549,-768,-721,-549,-736,-822,-585,-664,-702,-585,-744,-73`
+`0,-525,-640,-666,-525,-640,-674,-397,-640,-738,-397,-640,-738,-421,-488,-722,-429,-488,-714,-413,-488,-490,-317,-640,-602,-317,-640,-602,-357,-640,-634,-389,-640,-674,-397,-640,-666,-525,-640,-490,-317,-640,-602,-357,-640,-634,-389,-640,-666,-525,-640,-490,-525,-640,-658,-549,-768,-666,-597,-768,-650,-605,-768,-490,-549,-768,-650,-605,-768,-666,-621,-768,-642,-677,-768,-490,-677,-768,-490,-549,-768,-618,-677,-416,-490,-677,-416,-490,-549,-416,-490,-421,-488,-490,-381,-488,-618,-381,-488,-490,-34`
+`1,-488,-490,-325,-488,-538,-333,-488],"vertslength":184,"tris":[3,0,1,1,2,3,4,5,6,4,6,7,11,8,9,9,10,11,12,13,14,14,15,16,12,14,16,17,18,19,17,19,20,21,22,23,24,25,26,24,26,27,28,29,30,30,31,32,30,32,33,28,30,33,34,35,36,37,38,39,37,39,40,44,41,42,42,43,44,45,46,47,48,49,50,48,50,51,52,53,54,52,54,55,56,57,58,60,61,62,59,60,62,63,64,65,66,67,68,66,68,69,70,71,72,75,76,73,73,74,75,77,78,79,82,83,84,81,82,84,81,84,85,80,81,85,87,88,89,87,89,90,87,90,91,86,87,91,92,93,86,86,91,92,94,95,96,97,98,99,1`
+`00,101,102,109,103,104,109,104,105,108,109,105,107,108,105,105,106,107,115,110,111,115,111,112,114,115,112,112,113,114,116,117,118,116,118,119,123,120,121,121,122,123,124,125,126,127,128,124,124,126,127,132,129,130,130,131,132,133,134,135,136,137,133,133,135,136,141,142,143,138,140,145,138,139,146,139,140,146,140,138,146,140,141,147,145,140,147,145,144,147,141,143,147,144,143,147,148,149,150,148,150,151,152,153,154,155,156,157,158,159,160,161,162,163,163,164,165,161,163,165,166,167,168,166,168,1`
+`69,170,171,172,170,172,173,170,173,174,175,176,177,178,179,180,181,182,183],"trislength":98,"triTopoly":[0,0,1,1,2,2,3,3,3,4,4,5,6,6,7,7,7,7,8,9,9,10,10,11,12,12,13,13,14,15,15,16,17,17,18,19,19,20,21,21,21,21,22,22,22,22,22,22,23,24,25,26,26,26,26,26,27,27,27,27,28,28,29,29,30,30,30,31,31,32,32,32,33,33,33,33,33,33,33,33,33,33,34,34,35,36,37,38,38,38,39,39,40,40,40,41,42,43],"baseVert":[0,4,8,12,17,21,24,28,34,37,41,45,48,52,56,59,63,66,70,73,77,80,86,94,97,100,103,110,116,120,124,129,133,138,1`
+`48,152,155,158,161,166,170,175,178,181],"vertsCount":[4,4,4,5,4,3,4,6,3,4,4,3,4,4,3,4,3,4,3,4,3,6,8,3,3,3,7,6,4,4,5,4,5,10,4,3,3,3,5,4,5,3,3,3],"baseTri":[0,2,4,6,9,11,12,14,18,19,21,23,24,26,28,29,31,32,34,35,37,38,42,48,49,50,51,56,60,62,64,67,69,72,82,84,85,86,87,90,92,95,96,97],"triCount":[2,2,2,3,2,1,2,4,1,2,2,1,2,2,1,2,1,2,1,2,1,4,6,1,1,1,5,4,2,2,3,2,3,10,2,1,1,1,3,2,3,1,1,1]},"links":{"poly":[10,28,12,35,13,31],"cost":[4440,864,3508.965576171875],"type":[2,1,2],"pos":[-906,-581,-288,-890,`
+`-581,-340,-762,-421,-488,-738,-421,-488,-855.5172119140625,-650.7930908203125,-416,-850,-653,-368],"length":3}}],["5_4",{"tileId":"5_4","tx":5,"ty":4,"mesh":{"verts":[-346,-677,-768,-362,-645,-768,-402,-645,-768,-490,-677,-768,-410,-629,-768,-346,-621,-768,-338,-549,-768,-490,-677,-768,-402,-645,-768,-410,-629,-768,-490,-549,-768,-490,-677,-768,-410,-629,-768,-338,-549,-768,-298,-309,-416,-338,-269,-416,-362,-269,-416,-362,-421,-416,-298,-677,-416,-490,-541,-416,-490,-677,-416,-298,-677,-416,-36`
+`2,-421,-416,-418,-357,-640,-418,-317,-640,-490,-317,-640,-386,-389,-640,-418,-357,-640,-490,-317,-640,-490,-525,-640,-298,-525,-640,-298,-525,-640,-322,-389,-640,-386,-389,-640,-394,-397,-416,-394,-165,-408,-490,-165,-408,-490,-485,-416,-434,-437,-488,-394,-397,-488,-394,-381,-488,-490,-381,-488,-490,-421,-488,-490,-325,-488,-490,-341,-488,-418,-341,-488,-418,-317,-488,-322,-381,-488,-362,-381,-488,-362,-421,-488,-322,-461,-488,-362,-421,-488,-394,-461,-488,-322,-461,-488,-338,-637,-768,-362,-64`
+`5,-768,-346,-677,-768,22,-677,-768,-338,-549,-768,-346,-621,-768,-338,-637,-768,22,-677,-768,22,-549,-768,-42,-237,-224,-50,-165,-224,-362,-165,-224,-362,-245,-224,-362,-221,68,-362,-237,68,-50,-237,68,-50,-221,68,-50,-197,68,-50,-165,68,-362,-165,68,-362,-197,68,-346,-237,-416,22,-237,-416,22,-165,-416,-346,-165,-416,-306,-373,-640,-322,-389,-640,-298,-525,-640,22,-525,-640,22,-381,-640,-298,-269,-368,-322,-269,-368,-322,-285,-368,-298,-293,-368,-74,-533,-172,-58,-525,-172,-58,-485,-172,-226,-5`
+`01,-172,-226,-525,-172,-226,-501,-172,-234,-445,-172,-266,-437,-160,-266,-677,-160,-226,-605,-172,-226,-525,-172,-58,-597,-172,-226,-605,-172,-266,-677,-160,22,-677,-172,22,-269,-172,6,-269,-172,6,-533,-172,22,-677,-172,-18,-573,-172,-58,-597,-172,22,-677,-172,6,-533,-172,-18,-573,-172,22,-677,-172,-266,-437,-160,-234,-445,-172,-226,-429,-172,-266,-269,-160,-226,-429,-172,-130,-437,-172,-98,-421,-172,-98,-269,-172,-266,-269,-160,-218,-461,-256,-250,-469,-256,-250,-645,-256,-42,-349,-256,-218,-34`
+`9,-256,-218,-461,-256,-218,-461,-256,-250,-645,-256,-42,-637,-256,-42,-349,-256,-242,-469,-324,-242,-621,-311,-226,-621,-311,-226,-469,-324,-218,-541,-106,-218,-589,-106,-154,-589,-106,-154,-541,-106,-218,-445,-106,-218,-485,-106,-154,-485,-106,-154,-445,-106,-2,-461,-416,6,-509,-416,22,-509,-416,22,-285,-416,6,-285,-416,-2,-365,-416,-186,-677,-416,-90,-677,-416,-82,-637,-416,-202,-637,-416,-82,-637,-416,-42,-637,-416,-42,-469,-416,-202,-349,-416,-202,-637,-416,-2,-365,-416,-42,-349,-416,-202,-3`
+`49,-416,-42,-469,-416,-2,-461,-416,-186,-549,-156,-186,-573,-156,-162,-573,-156,-162,-549,-156,-186,-453,-156,-186,-477,-156,-162,-477,-156,-162,-453,-156,-82,-429,-120,-130,-445,-120,-138,-477,-120,-50,-469,-120,-74,-549,-120,-138,-549,-120,-138,-581,-120,-58,-581,-120,-42,-525,-120,-74,-549,-120,-58,-581,-120,-18,-549,-120,-50,-469,-120,-42,-525,-120,-18,-549,-120,-82,-333,-120,-82,-429,-120,-50,-469,-120,-10,-333,-120,-82,-333,-120,-50,-469,-120,-18,-549,-120,-34,-221,124,-34,-245,124,-18,-24`
+`5,124,-18,-221,124,-18,-197,124,-18,-165,124,-34,-165,124,-34,-197,124,22,-165,-224,-10,-165,-224,6,-181,-224,6,-181,-224,-2,-237,-224,22,-245,-224,22,-165,-224,6,-645,-256,22,-645,-256,22,-269,-256,6,-269,-256,22,-637,-416,22,-629,-416,6,-629,-416,22,-237,68,22,-221,68,6,-221,68,6,-197,68,22,-197,68,22,-165,68,6,-165,68],"vertslength":223,"polys":[0,3,4,6,7,9,10,13,14,18,19,22,23,25,26,30,31,33,34,37,38,42,43,46,47,50,51,53,54,57,58,62,63,66,67,70,71,74,75,78,79,83,84,87,88,92,93,98,99,102,103,`
+`106,107,109,110,112,113,116,117,121,122,124,125,127,128,131,132,135,136,139,140,143,144,149,150,153,154,158,159,163,164,167,168,171,172,175,176,179,180,183,184,186,187,189,190,193,194,197,198,201,202,204,205,208,209,212,213,215,216,218,219,222],"polyslength":56,"regions":[8,8,8,8,2,2,5,5,5,9,14,23,15,15,7,7,11,26,22,12,6,27,10,10,10,10,10,10,4,4,1,1,1,31,17,19,3,3,3,3,34,35,13,13,13,13,13,13,40,41,43,43,44,45,46,47],"neighbors":[[[1,14],[0],[1,2],[0]],[[0],[1,15],[1,3]],[[1,0],[0],[1,3]],[[0],[1`
+`,2],[1,1],[0]],[[0],[0],[0],[1,5],[0]],[[0],[0],[1,4],[0]],[[0],[0],[1,7]],[[0],[1,6],[0],[0],[1,8]],[[1,20],[0],[1,7]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,13],[0]],[[0],[0],[1,12]],[[0],[1,0],[0],[1,15]],[[1,1],[0],[1,14],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,8],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,23],[0]],[[0],[1,28],[0],[1,24],[0],[1,22]],[[0],[1,23],[0],[1,26]],[[0],[0],[1,27],[0]],[[0],[1,24],[1,`
+`27]],[[0],[1,26],[1,25]],[[1,23],[0],[1,29],[0]],[[0],[0],[0],[0],[1,28]],[[0],[0],[1,32]],[[0],[0],[1,32]],[[1,30],[0],[0],[1,31]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[1,39]],[[0],[0],[1,38],[0]],[[0],[0],[1,39],[0],[1,37]],[[0],[0],[1,38],[0],[1,36]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,46]],[[0],[0],[0],[1,44]],[[0],[1,43],[0],[1,45]],[[0],[1,44],[1,47]],[[0],[1,42],[1,47]],[[0],[1,46],[1,45],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[`
+`0],[1,51]],[[0],[0],[0],[1,50]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-346,-677,-768,-362,-645,-768,-402,-645,-768,-490,-677,-768,-410,-629,-768,-346,-621,-768,-338,-549,-768,-490,-677,-768,-402,-645,-768,-410,-629,-768,-490,-549,-768,-490,-677,-768,-410,-629,-768,-338,-549,-768,-298,-309,-416,-338,-269,-416,-362,-269,-416,-362,-421,-416,-298,-677,-416,-490,-541,-416,-490,-677,-416,-298,-677,-416,-362,-421,-416,-418,-357,-640,-418,-317,-640,-490,-31`
+`7,-640,-386,-389,-640,-418,-357,-640,-490,-317,-640,-490,-525,-640,-298,-525,-640,-298,-525,-640,-322,-389,-640,-386,-389,-640,-394,-397,-416,-394,-165,-408,-490,-165,-408,-490,-256.4285583496094,-408,-490,-279.28570556640625,-416,-490,-485,-416,-454,-257,-408,-434,-437,-488,-394,-397,-488,-394,-381,-488,-490,-381,-488,-490,-421,-488,-490,-325,-488,-490,-341,-488,-418,-341,-488,-418,-317,-488,-322,-381,-488,-362,-381,-488,-362,-421,-488,-322,-461,-488,-362,-421,-488,-394,-461,-488,-322,-461,-488`
+`,-338,-637,-768,-362,-645,-768,-346,-677,-768,22,-677,-768,-338,-549,-768,-346,-621,-768,-338,-637,-768,22,-677,-768,22,-549,-768,-42,-237,-224,-50,-165,-224,-362,-165,-224,-362,-245,-224,-362,-221,68,-362,-237,68,-50,-237,68,-50,-221,68,-50,-197,68,-50,-165,68,-362,-165,68,-362,-197,68,-346,-237,-416,22,-237,-416,22,-165,-416,-346,-165,-416,-306,-373,-640,-322,-389,-640,-298,-525,-640,22,-525,-640,22,-381,-640,-298,-269,-368,-322,-269,-368,-322,-285,-368,-298,-293,-368,-74,-533,-172,-58,-525,-1`
+`72,-58,-485,-172,-226,-501,-172,-226,-525,-172,-226,-501,-172,-234,-445,-172,-266,-437,-165,-266,-677,-165,-226,-605,-172,-226,-525,-172,-58,-597,-172,-226,-605,-172,-266,-677,-165,-243.84616088867188,-677,-172,22,-677,-172,22,-269,-172,6,-269,-172,6,-533,-172,22,-677,-172,-18,-573,-172,-58,-597,-172,22,-677,-172,6,-533,-172,-18,-573,-172,22,-677,-172,-266,-437,-165,-234,-445,-172,-226,-429,-172,-266,-269,-165,-226,-429,-172,-130,-437,-172,-98,-421,-172,-98,-269,-172,-245,-269,-172,-266,-269,-16`
+`5,-218,-461,-256,-250,-469,-256,-250,-645,-256,-42,-349,-256,-218,-349,-256,-218,-461,-256,-198.44444274902344,-448.5555419921875,-248,-159.3333282470703,-423.6666564941406,-256,-158,-401,-248,-134,-401,-248,-218,-461,-256,-250,-645,-256,-42,-637,-256,-42,-349,-256,-159.3333282470703,-423.6666564941406,-256,-178.88888549804688,-436.1111145019531,-248,-190,-585,-248,-190,-465,-248,-70,-585,-248,-94,-417,-248,-242,-469,-324,-242,-555.8571166992188,-324,-242,-577.5714111328125,-311,-242,-621,-311,-`
+`226,-621,-311,-226,-577.5714111328125,-311,-226,-555.8571166992188,-324,-226,-469,-324,-218,-541,-106,-218,-589,-106,-154,-589,-106,-154,-541,-106,-218,-445,-106,-218,-485,-106,-154,-485,-106,-154,-445,-106,-2,-461,-416,6,-509,-416,22,-509,-416,22,-285,-416,6,-285,-416,-2,-365,-416,-186,-677,-415,-90,-677,-415,-82,-637,-416,-202,-637,-416,-82,-637,-416,-42,-637,-416,-42,-469,-415,-202,-349,-416,-202,-637,-416,-2,-365,-416,-42,-349,-416,-202,-349,-416,-42,-469,-415,-2,-461,-416,-186,-549,-156,-18`
+`6,-573,-156,-162,-573,-156,-162,-549,-156,-186,-453,-156,-186,-477,-156,-162,-477,-156,-162,-453,-156,-82,-429,-120,-130,-445,-120,-138,-477,-120,-50,-469,-120,-74,-549,-120,-138,-549,-120,-138,-581,-120,-58,-581,-120,-42,-525,-120,-74,-549,-120,-58,-581,-120,-18,-549,-120,-50,-469,-120,-42,-525,-120,-18,-549,-120,-82,-333,-120,-82,-429,-120,-50,-469,-120,-10,-333,-120,-82,-333,-120,-50,-469,-120,-18,-549,-120,-34,-221,124,-34,-245,124,-18,-245,124,-18,-221,124,-18,-197,124,-18,-165,124,-34,-165`
+`,124,-34,-197,124,22,-165,-224,-10,-165,-224,6,-181,-224,6,-181,-224,-2,-237,-224,22,-245,-224,22,-165,-224,6,-645,-256,22,-645,-256,22,-621.5,-248,22,-574.5,-248,22,-551,-256,22,-504,-256,22,-480.5,-248,22,-386.5,-256,22,-363,-248,22,-316,-248,22,-269,-256,6,-269,-256,6,-316,-248,6,-363,-248,6,-386.5,-256,6,-433.5,-248,6,-480.5,-248,6,-504,-256,6,-621.5,-248,18,-561,-248,22,-637,-416,22,-629,-416,6,-629,-416,22,-237,68,22,-221,68,6,-221,68,6,-197,68,22,-197,68,22,-165,68,6,-165,68],"vertslength`
+`":258,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,10,11,12,10,12,13,14,15,16,14,16,17,14,17,18,19,20,21,19,21,22,23,24,25,26,27,28,26,28,29,26,29,30,31,32,33,38,39,34,34,35,40,38,34,40,38,37,40,35,36,40,37,36,40,41,42,43,44,45,41,41,43,44,46,47,48,46,48,49,50,51,52,50,52,53,54,55,56,57,58,59,57,59,60,61,62,63,65,61,63,63,64,65,66,67,68,66,68,69,73,70,71,71,72,73,77,74,75,75,76,77,81,78,79,79,80,81,82,83,84,84,85,86,82,84,86,87,88,89,87,89,90,91,92,93,94,95,91,91,93,94,101,96,97,101,97,98,99,100,101,98,99,10`
+`1,103,104,105,102,103,105,102,105,106,107,108,109,107,109,110,111,112,113,114,115,116,117,118,119,117,119,120,121,122,123,125,126,121,121,123,124,121,124,125,127,128,129,131,132,133,131,133,135,134,133,135,134,130,136,135,134,136,130,131,136,135,131,136,137,138,143,138,139,143,137,143,144,137,142,144,141,142,144,139,140,145,144,143,145,139,143,145,140,141,146,145,140,146,141,144,146,145,144,146,149,150,151,149,151,152,148,149,152,148,152,153,154,147,148,148,153,154,158,155,156,156,157,158,162,15`
+`9,160,160,161,162,163,164,165,166,167,168,168,163,165,165,166,168,169,170,171,169,171,172,173,174,175,177,173,175,175,176,177,181,182,178,181,178,179,179,180,181,186,183,184,184,185,186,190,187,188,188,189,190,191,192,193,191,193,194,195,196,197,195,197,198,202,199,200,200,201,202,203,204,205,206,207,208,209,210,211,209,211,212,216,213,214,214,215,216,220,217,218,218,219,220,221,222,223,224,225,226,224,226,227,246,228,229,246,229,230,246,230,231,237,238,239,237,239,240,236,237,240,236,240,241,23`
+`5,236,241,235,241,242,235,242,243,235,243,244,234,235,244,233,234,244,233,244,245,232,233,245,232,245,247,245,246,247,246,231,247,232,231,247,248,249,250,251,252,253,257,254,255,255,256,257],"trislength":154,"triTopoly":[0,0,1,2,3,3,4,4,4,5,5,6,7,7,7,8,9,9,9,9,9,9,10,10,10,11,11,12,12,13,14,14,15,15,15,16,16,17,17,18,18,19,19,20,20,20,21,21,22,22,22,23,23,23,23,24,24,24,25,25,26,27,28,28,29,29,29,29,30,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,34,34,35,35,36,36,3`
+`6,36,37,37,38,38,38,39,39,39,40,40,41,41,42,42,43,43,44,44,45,46,47,47,48,48,49,49,50,51,51,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,53,54,55,55],"baseVert":[0,4,7,10,14,19,23,26,31,34,41,46,50,54,57,61,66,70,74,78,82,87,91,96,102,107,111,114,117,121,127,130,137,147,155,159,163,169,173,178,183,187,191,195,199,203,206,209,213,217,221,224,228,248,251,254],"vertsCount":[4,3,3,4,5,4,3,5,3,7,5,4,4,3,4,5,4,4,4,4,5,4,5,6,5,4,3,3,4,6,3,7,10,8,4,4,6,4,5,5,4,4,4,4,4,3,3,4,4,4,3,4,20,3,3,4]`
+`,"baseTri":[0,2,3,4,6,9,11,12,15,16,22,25,27,29,30,32,35,37,39,41,43,46,48,51,55,58,60,61,62,64,68,69,76,88,94,96,98,102,104,107,110,112,114,116,118,120,121,122,124,126,128,129,131,150,151,152],"triCount":[2,1,1,2,3,2,1,3,1,6,3,2,2,1,2,3,2,2,2,2,3,2,3,4,3,2,1,1,2,4,1,7,12,6,2,2,4,2,3,3,2,2,2,2,2,1,1,2,2,2,1,2,19,1,1,2]},"links":{"poly":[4,21,10,12,17,48,18,49,22,43,34,43,35,42,48,54,49,55,51,52],"cost":[3648,1536,5088,5088,4440,678,678,5568,5568,2400],"type":[2,1,2,2,2,2,2,2,2,2],"pos":[-330,-27`
+`7,-416,-322,-269,-368,-394,-397,-488,-362,-397,-488,-50,-237,68,-34,-237,124,-50,-165,68,-34,-165,124,-74,-533,-172,-74,-549,-120,-154,-549,-106,-138,-549,-120,-154,-477,-106,-138,-477,-120,-18,-221,124,6,-221,68,-18,-197,124,6,-197,68,22,-245,-224,22,-269,-256],"length":10}}],["6_4",{"tileId":"6_4","tx":6,"ty":4,"mesh":{"verts":[22,-549,-768,22,-677,-768,246,-677,-768,254,-549,-768,22,-677,-172,70,-677,-171,70,-645,-172,190,-645,-172,198,-677,-172,246,-677,-172,246,-605,-172,230,-597,-172,230,-`
+`357,-172,246,-349,-172,246,-285,-172,230,-357,-172,246,-285,-172,230,-269,-172,22,-269,-172,22,-677,-172,70,-645,-172,70,-645,-172,190,-645,-172,230,-597,-172,230,-357,-172,270,-237,-256,238,-253,-256,214,-269,-256,270,-637,-256,214,-269,-256,22,-269,-256,22,-645,-256,270,-637,-256,38,-621,-416,22,-629,-416,22,-637,-416,262,-637,-416,262,-621,-416,222,-613,-416,206,-253,-415,118,-245,-415,110,-285,-416,222,-285,-416,262,-461,-416,262,-277,-416,222,-285,-416,230,-469,-416,110,-285,-416,22,-285,-4`
+`16,22,-509,-416,38,-517,-416,230,-469,-416,222,-285,-416,38,-517,-416,38,-621,-416,222,-613,-416,230,-469,-416,302,-525,-640,302,-389,-640,254,-389,-640,302,-525,-640,254,-389,-640,246,-373,-640,22,-381,-640,22,-525,-640,22,-165,-224,22,-245,-224,46,-245,-224,46,-165,-224,22,-237,-416,30,-165,-416,22,-165,-416,22,-221,68,22,-237,68,158,-237,68,158,-221,68,22,-165,68,22,-197,68,126,-197,68,126,-165,68,182,-221,-248,238,-253,-256,270,-237,-256,270,-165,-256,62,-165,-256,62,-245,-256,182,-221,-248,`
+`270,-165,-256,62,-165,-416,62,-197,-416,118,-245,-415,206,-253,-415,262,-189,-416,262,-165,-416,150,-189,68,166,-197,68,166,-181,68,150,-173,68,198,-237,68,214,-229,68,198,-221,68,198,-165,68,222,-197,68,262,-197,68,302,-165,68,310,-237,68,302,-165,68,262,-197,68,286,-245,68,534,-245,68,534,-237,68,310,-237,68,286,-245,68,230,-549,-628,230,-565,-628,262,-573,-625,302,-549,-630,262,-573,-625,262,-677,-625,302,-677,-625,302,-549,-630,238,-525,-396,238,-605,-396,254,-605,-396,254,-525,-396,534,-413`
+`,74,510,-413,74,510,-453,74,510,-453,74,478,-453,74,470,-525,74,310,-525,74,302,-445,74,278,-429,74,270,-677,74,534,-413,74,510,-453,74,470,-525,74,534,-541,74,398,-541,74,534,-541,74,470,-525,74,310,-525,74,390,-677,74,398,-541,74,310,-525,74,270,-677,74,278,-677,-432,294,-677,-432,294,-477,-432,302,-557,-768,278,-557,-768,278,-629,-760,302,-637,-760,278,-429,74,302,-445,74,310,-429,74,326,-349,74,278,-325,74,454,-429,74,470,-397,74,422,-389,74,406,-349,74,350,-325,74,326,-349,74,422,-389,74,42`
+`2,-389,74,326,-349,74,310,-429,74,454,-429,74,326,-325,-128,302,-317,-128,286,-373,-128,302,-501,-128,326,-677,-128,286,-373,-128,286,-493,-128,302,-501,-128,302,-501,-128,286,-677,-128,326,-677,-128,302,-381,-488,286,-381,-488,286,-405,-488,302,-413,-488,302,-277,-128,302,-317,-128,326,-325,-128,534,-317,-128,534,-285,-128,302,-165,-72,302,-205,-72,326,-205,-72,358,-197,-72,366,-165,-72,430,-205,-72,438,-165,-72,406,-165,-72,414,-205,-72,430,-205,-72,414,-205,-72,390,-221,-72,462,-229,-72,358,-`
+`197,-72,326,-205,-72,334,-229,-72,390,-221,-72,390,-221,-72,334,-229,-72,462,-229,-72,334,-557,-416,318,-565,-416,318,-677,-416,470,-405,-416,470,-325,-416,390,-309,-414,326,-341,-416,318,-501,-416,334,-509,-416,470,-405,-416,334,-509,-416,334,-557,-416,534,-677,-400,534,-413,-416,470,-405,-416,334,-557,-416,318,-677,-416,406,-165,-272,318,-165,-272,318,-221,-276,414,-221,-272,382,-493,74,382,-461,74,334,-461,74,334,-501,74,374,-357,-488,358,-341,-488,334,-373,-488,342,-445,-488,374,-461,-488,33`
+`4,-277,74,446,-325,74,446,-269,74,342,-549,-768,342,-677,-768,534,-677,-768,534,-549,-768,414,-541,-628,414,-565,-628,430,-533,-628,390,-533,-633,414,-541,-628,430,-533,-628,398,-325,-640,342,-373,-640,342,-677,-636,382,-677,-636,390,-533,-633,342,-373,-640,534,-165,-640,398,-165,-640,398,-325,-640,430,-533,-628,534,-533,-640,462,-445,148,382,-445,148,398,-469,148,398,-469,148,382,-509,148,462,-509,148,462,-445,148,382,-181,143,382,-205,149,398,-205,148,398,-181,142,390,-285,-432,534,-293,-432,5`
+`34,-261,-432,390,-253,-432,422,-573,74,422,-677,74,438,-677,74,438,-573,74,446,-165,-288,430,-173,-288,430,-221,-288,478,-221,-288,494,-165,-288,438,-189,-414,438,-221,-414,478,-221,-402,478,-189,-414,454,-373,74,534,-389,74,534,-373,74,446,-357,74,454,-565,149,454,-677,149,534,-677,131,534,-557,132,462,-573,74,462,-677,74,486,-677,74,486,-573,74,534,-269,118,462,-269,118,462,-325,118,534,-341,118,534,-165,-72,470,-165,-72,478,-205,-72,534,-221,-72,486,-397,-339,534,-397,-339,534,-325,-339,486,-`
+`325,-339,494,-349,-414,494,-381,-414,534,-381,-414,534,-333,-414,534,-181,-272,502,-181,-272,502,-213,-272,534,-221,-272,534,-213,-415,534,-189,-415,510,-189,-415],"vertslength":327,"polys":[0,3,4,6,7,11,12,14,15,20,21,24,25,28,29,32,33,38,39,42,43,46,47,52,53,56,57,59,60,64,65,68,69,71,72,75,76,79,80,83,84,87,88,93,94,97,98,100,101,104,105,108,109,112,113,116,117,120,121,124,125,127,128,130,131,134,135,138,139,142,143,146,147,149,150,153,154,158,159,161,162,165,166,169,170,174,175,177,178,180,1`
+`81,184,185,189,190,194,195,198,199,202,203,206,207,209,210,212,213,218,219,221,222,226,227,230,231,234,235,239,240,242,243,246,247,249,250,254,255,258,259,263,264,266,267,270,271,274,275,278,279,282,283,287,288,291,292,295,296,299,300,303,304,307,308,311,312,315,316,319,320,323,324,326],"polyslength":81,"regions":[7,3,3,3,3,3,1,1,2,2,2,2,2,6,6,36,37,38,32,13,13,10,39,40,24,24,24,25,25,41,9,9,9,9,9,9,44,45,11,11,11,11,27,27,27,46,21,20,20,20,20,20,4,4,4,4,16,29,33,22,8,5,5,5,5,17,17,55,34,57,18,3`
+`5,60,12,61,15,30,23,31,65,66],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,4]],[[0],[0],[0],[0],[1,5]],[[0],[0],[1,4]],[[1,3],[0],[0],[0],[1,1],[1,5]],[[0],[1,2],[0],[1,4]],[[1,19],[0],[1,7],[0]],[[0],[0],[0],[1,6]],[[0],[0],[0],[0],[0],[1,12]],[[1,21],[0],[1,11],[0]],[[0],[0],[1,11],[0]],[[0],[0],[0],[1,12],[1,10],[1,9]],[[0],[1,8],[0],[1,11]],[[0],[0],[1,14]],[[1,13],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,6],[0],[1,20]],[[0],[0],[1,19],[0]],[`
+`[0],[0],[1,9],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,25],[0]],[[0],[1,24],[0],[1,26]],[[0],[0],[1,25],[0]],[[0],[0],[1,28],[0]],[[0],[0],[0],[1,27]],[[0],[0],[0],[0]],[[0],[0],[1,33]],[[0],[0],[1,33]],[[0],[1,38],[0],[1,35]],[[1,30],[1,31],[1,34],[0]],[[0],[1,33],[0],[1,35]],[[0],[1,34],[1,32],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[1,32],[0],[1,41],[0],[0]],[[0],[0],[1,41]],[[0],[0],[1,41],[0]],[[1,40],[1,38],[0],[1,39]],[[1,46],[0],[1,43],[1,44],[0]],[[0],[0],[1,42]],[[0],[0],`
+`[1,42]],[[0],[0],[0],[0]],[[0],[1,42],[0],[0],[0]],[[0],[0],[1,50],[0],[0]],[[0],[0],[0],[1,49]],[[1,48],[0],[1,51],[0]],[[1,47],[0],[1,51],[0]],[[1,50],[0],[1,49]],[[0],[0],[1,55]],[[0],[0],[0],[0],[0],[1,54]],[[1,53],[0],[1,55]],[[0],[0],[1,54],[1,52],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,62]],[[0],[1,61],[1,64],[0],[1,63]],[[0],[0],[1,62],[0]],[[0],[0],[1,62],[0],[0]],[[0],[0],[1,66]],[[0],[0],[0],[1,65]],[[0],[0],[0],[0]],[`
+`[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[22,-549,-768,22,-677,-768,246,-677,-768,254,-549,-768,22,-677,-172,70,-677,-171,70,-645,-172,190,-645,-172,198,-677,-172,246,-677,-172,246,-605,-172,230,-597,-172,230,-357,-172,246,-349,-172,246,-285,-172,230,-357,-172,246,-285,-172,230,-269,-172`
+`,22,-269,-172,22,-677,-172,70,-645,-172,70,-645,-172,190,-645,-172,230,-597,-172,230,-357,-172,270,-237,-256,238,-253,-256,214,-269,-256,221,-315,-248,228,-361,-248,231.5,-384,-256,270,-637,-256,214,-269,-256,22,-269,-256,22,-316,-248,22,-363,-248,22,-386.5,-256,22,-433.5,-248,22,-480.5,-248,22,-504,-256,22,-621.5,-248,22,-645,-256,270,-637,-256,231.5,-384,-256,228,-361,-248,106,-489,-248,130,-393,-256,226,-441,-248,58,-321,-256,106,-345,-248,226,-585,-248,38,-621,-416,22,-629,-416,22,-637,-416,`
+`262,-637,-416,262,-621,-416,222,-613,-416,206,-253,-415,118,-245,-415,110,-285,-416,222,-285,-416,262,-461,-416,262,-277,-416,222,-285,-416,230,-469,-416,110,-285,-416,22,-285,-416,22,-509,-416,38,-517,-416,230,-469,-416,222,-285,-416,38,-517,-416,38,-621,-416,222,-613,-416,230,-469,-416,302,-525,-640,302,-389,-640,254,-389,-640,302,-525,-640,254,-389,-640,246,-373,-640,22,-381,-640,22,-525,-640,22,-165,-224,22,-245,-224,46,-245,-224,46,-165,-224,22,-237,-416,30,-165,-416,22,-165,-416,22,-221,68`
+`,22,-237,68,158,-237,68,158,-221,68,22,-165,68,22,-197,68,126,-197,68,126,-165,68,182,-221,-248,200.6666717529297,-231.6666717529297,-256,238,-253,-256,270,-237,-256,270,-165,-256,62,-165,-256,62,-245,-256,182,-221,-248,270,-165,-256,154.44444274902344,-165,-256,131.3333282470703,-165,-248,62,-165,-416,62,-197,-416,118,-245,-416,206,-253,-416,262,-189,-416,262,-165,-416,150,-189,68,166,-197,68,166,-181,68,150,-173,68,198,-237,68,214,-229,68,198,-221,68,198,-165,68,222,-197,68,262,-197,68,302,-16`
+`5,68,310,-237,68,302,-165,68,262,-197,68,286,-245,68,534,-245,68,534,-237,68,310,-237,68,286,-245,68,230,-549,-628,230,-565,-628,262,-573,-625,302,-549,-630,262,-573,-625,262,-677,-624,302,-677,-624,302,-549,-630,238,-525,-396,238,-605,-396,254,-605,-396,254,-525,-396,534,-413,74,510,-413,74,510,-453,74,510,-453,74,478,-453,74,470,-525,74,310,-525,74,302,-445,74,278,-429,74,270,-677,74,534,-413,74,510,-453,74,470,-525,74,534,-541,74,398,-541,74,534,-541,74,470,-525,74,310,-525,74,390,-677,74,398`
+`,-541,74,310,-525,74,270,-677,74,278,-677,-432,294,-677,-432,294,-477,-432,302,-557,-768,278,-557,-768,278,-629,-768,302,-637,-760,302,-617,-768,278,-429,74,302,-445,74,310,-429,74,326,-349,74,278,-325,74,454,-429,74,470,-397,74,422,-389,74,406,-349,74,350,-325,74,326,-349,74,422,-389,74,422,-389,74,326,-349,74,310,-429,74,454,-429,74,326,-325,-128,302,-317,-128,286,-373,-128,302,-501,-128,326,-677,-128,286,-373,-128,286,-493,-128,302,-501,-128,302,-501,-128,286,-677,-128,326,-677,-128,302,-381,`
+`-488,286,-381,-488,286,-405,-488,302,-413,-488,302,-277,-128,302,-317,-128,326,-325,-128,534,-317,-128,534,-285,-128,302,-165,-72,302,-205,-72,326,-205,-72,358,-197,-72,366,-165,-72,430,-205,-72,438,-165,-72,406,-165,-72,414,-205,-72,430,-205,-72,414,-205,-72,390,-221,-72,462,-229,-72,358,-197,-72,326,-205,-72,334,-229,-72,390,-221,-72,390,-221,-72,334,-229,-72,462,-229,-72,334,-557,-416,318,-565,-416,318,-677,-416,470,-405,-416,470,-325,-416,390,-309,-414,326,-341,-416,318,-501,-416,334,-509,-4`
+`16,470,-405,-416,334,-509,-416,334,-557,-416,534,-677,-400,534,-567,-400,534,-501,-416,534,-413,-416,470,-405,-416,334,-557,-416,318,-677,-416,469.20001220703125,-677,-416,512.4000244140625,-677,-400,498,-521,-416,498,-641,-401,406,-165,-272,318,-165,-272,318,-221,-272,414,-221,-272,382,-493,74,382,-461,74,334,-461,74,334,-501,74,374,-357,-488,358,-341,-488,334,-373,-488,342,-445,-488,374,-461,-488,334,-277,74,446,-325,74,446,-269,74,342,-549,-768,342,-677,-768,534,-677,-768,534,-549,-768,414,-5`
+`41,-640,414,-553,-628,414,-565,-628,422,-549,-640,430,-533,-640,390,-533,-640,414,-541,-640,430,-533,-640,398,-325,-640,342,-373,-640,342,-677,-636,382,-677,-636,390,-533,-640,342,-373,-640,378,-545,-633,534,-165,-640,398,-165,-640,398,-325,-640,430,-533,-640,534,-533,-640,462,-445,148,382,-445,148,398,-469,148,398,-469,148,382,-509,148,462,-509,148,462,-445,148,382,-181,142,382,-205,146,398,-205,146,398,-181,142,390,-285,-432,534,-293,-432,534,-261,-432,390,-253,-432,422,-573,74,422,-677,74,438`
+`,-677,74,438,-573,74,446,-165,-288,430,-173,-288,430,-221,-288,478,-221,-288,494,-165,-288,438,-189,-414,438,-221,-414,478,-221,-414,478,-189,-414,454,-373,74,534,-389,74,534,-373,74,446,-357,74,454,-565,149,454,-677,147,534,-677,131,534,-557,132,462,-573,74,462,-677,74,486,-677,74,486,-573,74,534,-269,118,462,-269,118,462,-325,118,534,-341,118,534,-165,-72,470,-165,-72,478,-205,-72,534,-221,-72,486,-397,-339,534,-397,-339,534,-325,-339,486,-325,-339,494,-349,-414,494,-381,-414,534,-381,-414,534`
+`,-333,-414,534,-181,-272,502,-181,-272,502,-213,-272,534,-221,-272,534,-213,-415,534,-189,-415,510,-189,-415],"vertslength":358,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,10,11,7,7,9,10,12,13,14,15,16,17,15,17,18,19,20,15,15,18,19,21,22,23,21,23,24,25,26,27,25,27,28,25,28,29,25,29,30,25,30,31,39,40,45,39,38,45,38,37,45,37,36,46,37,45,46,44,43,46,42,43,47,43,46,47,45,46,47,32,33,48,34,33,48,34,35,48,32,48,49,46,44,49,32,44,49,48,35,49,46,36,49,35,36,49,42,47,50,47,45,50,45,40,50,42,41,50,40,41,50,51,52,53,5`
+`4,55,56,56,51,53,53,54,56,57,58,59,57,59,60,63,64,61,61,62,63,67,68,69,69,70,65,65,66,67,65,67,69,71,72,73,71,73,74,75,76,77,78,79,80,80,81,82,78,80,82,86,83,84,84,85,86,87,88,89,93,90,91,91,92,93,97,94,95,95,96,97,99,100,101,98,99,101,98,101,102,108,103,104,107,108,104,105,106,107,104,105,107,109,110,111,112,113,114,109,111,112,109,112,114,115,116,117,115,117,118,119,120,121,122,123,124,122,124,125,128,129,126,126,127,128,130,131,132,130,132,133,134,135,136,134,136,137,138,139,140,138,140,141,1`
+`45,142,143,143,144,145,146,147,148,149,150,151,152,153,154,152,154,155,157,158,159,156,157,159,160,161,162,160,162,163,164,165,166,164,166,167,168,169,170,173,174,175,175,171,172,172,173,175,176,177,178,176,178,179,176,179,180,181,182,183,184,185,186,184,186,187,188,189,190,188,190,191,192,193,194,192,194,195,192,195,196,197,198,199,200,201,202,203,204,205,203,205,206,207,208,209,209,210,211,207,209,211,212,213,214,214,215,216,212,214,216,220,217,218,218,219,220,221,222,223,221,223,224,225,226,2`
+`27,225,227,228,229,230,231,232,233,234,235,236,237,238,239,240,235,237,238,235,238,240,241,242,243,246,247,248,249,250,251,248,249,253,245,246,253,248,246,253,251,252,254,252,244,254,245,244,254,245,253,254,251,249,254,253,249,254,255,256,257,255,257,258,259,260,261,259,261,262,263,264,265,265,266,267,263,265,267,268,269,270,274,271,272,272,273,274,276,277,278,275,276,278,275,278,279,280,281,282,284,280,282,282,283,284,287,288,289,288,285,289,285,286,289,287,286,289,290,291,292,292,293,294,290,2`
+`92,294,295,296,297,298,299,300,298,300,301,305,302,303,303,304,305,306,307,308,306,308,309,313,310,311,311,312,313,314,315,316,314,316,317,314,317,318,322,319,320,320,321,322,323,324,325,323,325,326,327,328,329,327,329,330,334,331,332,332,333,334,335,336,337,335,337,338,339,340,341,339,341,342,346,343,344,344,345,346,347,348,349,347,349,350,351,352,353,351,353,354,355,356,357],"trislength":205,"triTopoly":[0,0,1,2,2,2,3,4,4,4,4,5,5,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,8,8,8,8,`
+`9,9,10,10,11,11,11,11,12,12,13,14,14,14,15,15,16,17,17,18,18,19,19,19,20,20,20,20,21,21,21,21,22,22,23,24,24,25,25,26,26,27,27,28,28,29,29,30,31,32,32,33,33,34,34,35,35,36,37,37,37,38,38,38,39,40,40,41,41,42,42,42,43,44,45,45,46,46,46,47,47,47,48,48,49,49,50,50,51,52,53,53,53,53,54,55,55,55,55,55,55,55,55,55,55,55,56,56,57,57,58,58,58,59,60,60,61,61,61,62,62,62,63,63,63,63,64,64,64,65,66,66,67,67,68,68,69,69,70,70,70,71,71,72,72,73,73,74,74,75,75,76,76,77,77,78,78,79,79,80],"baseVert":[0,4,7,12,`
+`15,21,25,32,51,57,61,65,71,75,78,83,87,90,94,98,103,109,115,119,122,126,130,134,138,142,146,149,152,156,160,164,168,171,176,181,184,188,192,197,200,203,207,212,217,221,225,229,232,235,241,244,255,259,263,268,271,275,280,285,290,295,298,302,306,310,314,319,323,327,331,335,339,343,347,351,355],"vertsCount":[4,3,5,3,6,4,7,19,6,4,4,6,4,3,5,4,3,4,4,5,6,6,4,3,4,4,4,4,4,4,3,3,4,4,4,4,3,5,5,3,4,4,5,3,3,4,5,5,4,4,4,3,3,6,3,11,4,4,5,3,4,5,5,5,5,3,4,4,4,4,5,4,4,4,4,4,4,4,4,4,3],"baseTri":[0,2,3,6,7,11,13,1`
+`8,41,45,47,49,53,55,56,59,61,62,64,66,69,73,77,79,80,82,84,86,88,90,92,93,94,96,98,100,102,103,106,109,110,112,114,117,118,119,121,124,127,129,131,133,134,135,139,140,151,153,155,158,159,161,164,167,171,174,175,177,179,181,183,186,188,190,192,194,196,198,200,202,204],"triCount":[2,1,3,1,4,2,5,23,4,2,2,4,2,1,3,2,1,2,2,3,4,4,2,1,2,2,2,2,2,2,1,1,2,2,2,2,1,3,3,1,2,2,3,1,1,2,3,3,2,2,2,1,1,4,1,11,2,2,3,1,2,3,3,4,3,1,2,2,2,2,3,2,2,2,2,2,2,2,2,2,1]},"links":{"poly":[7,15,8,29,13,27,33,73,48,76,56,70,59,`
+`75,69,74,70,79],"cost":[2400,899.076904296875,1014,5430,1536,768,3288,864,603.1697998046875],"type":[2,2,1,2,1,2,2,1,2],"pos":[22,-269,-256,22,-245,-224,251.23077392578125,-618.8461303710938,-416,254,-605,-396,302,-525,-640,302,-549,-630,534,-541,74,534,-557,132,438,-165,-72,470,-165,-72,414,-221,-272,430,-221,-288,446,-325,74,462,-325,118,438,-677,74,462,-677,74,490.3773498535156,-177.6792449951172,-288,502,-181,-272],"length":9}}],["7_4",{"tileId":"7_4","tx":7,"ty":4,"mesh":{"verts":[558,-541,`
+`-765,534,-549,-768,534,-677,-768,806,-677,-768,798,-549,-768,726,-541,-765,558,-541,-765,726,-541,-765,726,-293,-655,558,-293,-655,694,-605,-409,702,-645,-400,718,-621,-400,566,-677,-400,598,-629,-400,590,-605,-406,694,-605,-409,718,-621,-400,782,-605,-400,534,-677,-400,566,-677,-400,590,-605,-406,574,-405,-416,534,-413,-416,774,-333,-416,574,-325,-416,574,-405,-416,590,-605,-406,694,-605,-409,782,-605,-400,534,-557,130,534,-677,129,582,-677,119,582,-557,120,614,-533,74,622,-549,74,670,-549,74,6`
+`86,-533,74,662,-397,74,582,-397,74,574,-421,74,614,-533,74,686,-533,74,678,-429,74,574,-421,74,534,-413,74,534,-541,74,614,-533,74,726,-293,-655,742,-285,-640,750,-165,-640,534,-165,-640,542,-285,-640,558,-293,-655,534,-533,-640,542,-285,-640,534,-165,-640,534,-325,-339,534,-397,-339,558,-397,-339,558,-325,-339,534,-389,74,558,-381,74,534,-373,74,534,-381,-414,550,-341,-414,534,-333,-414,534,-269,118,534,-341,118,574,-357,118,758,-349,118,758,-269,118,822,-317,-128,806,-293,-128,766,-293,-128,53`
+`4,-317,-128,766,-293,-128,750,-269,-128,534,-269,-128,534,-317,-128,670,-261,-432,614,-261,-432,606,-277,-432,702,-277,-432,606,-277,-432,534,-261,-432,534,-293,-432,702,-277,-432,606,-277,-432,534,-293,-432,894,-285,-432,894,-253,-432,710,-261,-432,702,-277,-432,894,-285,-432,1006,-245,68,1046,-213,68,1046,-165,68,982,-165,68,982,-237,68,982,-237,68,534,-237,68,534,-245,68,1006,-245,68,766,-205,-72,814,-205,-72,822,-165,-72,534,-229,-72,758,-229,-72,766,-205,-72,534,-165,-72,534,-229,-72,766,-2`
+`05,-72,822,-165,-72,766,-205,-416,814,-205,-416,822,-165,-416,558,-237,-416,758,-237,-416,766,-205,-416,550,-165,-416,558,-237,-416,766,-205,-416,822,-165,-416,558,-261,-772,558,-397,-772,726,-397,-772,726,-261,-772,830,-677,-207,822,-637,-207,790,-621,-207,750,-621,-207,710,-661,-207,630,-621,-207,590,-645,-207,582,-677,-207,710,-661,-207,694,-637,-207,582,-677,-207,830,-677,-207,710,-661,-207,694,-565,74,670,-549,74,622,-549,74,598,-565,74,598,-677,74,694,-677,74,638,-637,-315,606,-653,-315,60`
+`6,-677,-315,646,-677,-315,670,-645,-315,830,-365,74,734,-389,74,726,-421,74,726,-421,74,678,-429,74,686,-533,74,870,-381,74,830,-365,74,726,-421,74,686,-533,74,878,-541,74,726,-573,74,726,-677,74,742,-677,74,742,-573,74,774,-637,-313,734,-653,-313,758,-677,-313,814,-677,-313,878,-317,-640,902,-333,-640,950,-325,-640,950,-253,-640,886,-197,-640,886,-197,-640,878,-165,-640,750,-165,-640,742,-285,-640,798,-333,-640,878,-317,-640,742,-533,-640,798,-533,-640,798,-333,-640,742,-285,-640,886,-677,119,8`
+`86,-557,120,766,-557,149,758,-677,149,766,-573,74,766,-677,74,790,-677,74,790,-573,74,926,-413,-416,934,-325,-407,870,-309,-414,774,-333,-416,782,-605,-400,838,-637,-400,846,-677,-400,966,-677,-414,782,-605,-400,838,-637,-400,966,-677,-414,966,-421,-416,926,-413,-416,774,-269,74,774,-341,74,878,-317,74,958,-269,74,950,-461,-768,862,-477,-768,862,-525,-768,862,-525,-768,798,-549,-768,806,-677,-768,862,-525,-768,806,-677,-768,950,-677,-768,950,-461,-768,822,-269,-128,806,-293,-128,822,-317,-128,95`
+`0,-317,-128,982,-269,-128,1006,-461,-128,1046,-461,-128,1046,-317,-128,990,-317,-128,966,-333,-128,982,-269,-128,950,-317,-128,966,-333,-128,990,-317,-128,982,-493,-128,1006,-461,-128,966,-333,-128,958,-677,-128,998,-677,-128,982,-493,-128,966,-333,-128,814,-477,-671,814,-517,-671,846,-517,-671,846,-477,-671,854,-461,-768,862,-477,-768,950,-461,-768,814,-261,-768,814,-461,-768,854,-461,-768,950,-461,-768,950,-261,-768,822,-165,-416,814,-205,-416,838,-229,-416,966,-229,-416,982,-165,-416,958,-229`
+`,-72,974,-197,-72,958,-165,-72,822,-165,-72,814,-205,-72,838,-229,-72,902,-677,74,910,-677,74,918,-637,74,902,-549,74,982,-301,74,918,-333,74,918,-365,74,1014,-317,74,982,-301,74,918,-365,74,902,-549,74,918,-637,74,1014,-645,74,870,-381,74,878,-541,74,902,-549,74,918,-365,74,902,-677,-640,950,-677,-640,950,-637,-640,942,-381,-640,950,-325,-640,902,-333,-640,902,-677,-640,950,-637,-640,990,-597,-640,942,-381,-640,902,-333,-640,990,-597,-640,1022,-597,-640,1022,-413,-640,982,-413,-640,990,-597,-64`
+`0,982,-413,-640,942,-381,-640,934,-565,-488,918,-557,-488,910,-557,-488,950,-677,-488,958,-677,-488,958,-645,-488,950,-597,-488,974,-541,-488,950,-541,-488,934,-565,-488,974,-581,-488,990,-589,-488,974,-581,-488,934,-565,-488,910,-557,-488,934,-589,-488,950,-597,-488,958,-645,-488,998,-605,-488,934,-589,-488,950,-597,-488,998,-605,-488,1030,-605,-488,990,-589,-488,910,-677,-488,918,-677,-488,934,-589,-488,910,-557,-488,1030,-461,-488,1014,-461,-488,990,-589,-488,1030,-605,-488,910,-557,-488,918,`
+`-557,-488,926,-533,-488,918,-405,-488,910,-325,-488,998,-453,-488,1014,-461,-488,1030,-461,-488,1030,-405,-488,998,-405,-488,990,-429,-488,918,-349,-488,934,-333,-488,910,-325,-488,926,-533,-488,950,-541,-488,974,-541,-488,974,-493,-488,998,-453,-488,990,-429,-488,958,-413,-488,918,-405,-488,926,-533,-488,974,-493,-488,966,-381,-488,990,-397,-488,966,-325,-488,918,-405,-488,918,-349,-488,910,-325,-488,966,-381,-488,966,-325,-488,942,-405,-488,958,-413,-488,958,-413,-488,942,-405,-488,918,-405,-4`
+`88,910,-229,-488,934,-221,-488,910,-205,-488,926,-309,-488,950,-317,-488,958,-277,-488,934,-653,148,934,-677,148,998,-677,148,998,-653,148,1014,-245,-256,1046,-245,-256,1046,-165,-248,1014,-165,-256,1046,-285,-128,1046,-277,-128,1030,-269,-128,1046,-165,-416,1030,-165,-416,1030,-205,-416,1046,-213,-416,1038,-677,-240,1046,-677,-240,1046,-501,-240],"vertslength":393,"polys":[0,5,6,9,10,12,13,15,16,18,19,23,24,29,30,33,34,37,38,43,44,47,48,53,54,56,57,60,61,63,64,66,67,71,72,75,76,79,80,83,84,86,8`
+`7,90,91,94,95,99,100,103,104,106,107,109,110,113,114,116,117,119,120,123,124,127,128,132,133,137,138,140,141,146,147,151,152,154,155,157,158,162,163,166,167,170,171,175,176,181,182,185,186,189,190,193,194,198,199,201,202,206,207,210,211,213,214,216,217,220,221,225,226,230,231,234,235,237,238,241,242,245,246,248,249,253,254,258,259,264,265,268,269,271,272,277,278,281,282,284,285,287,288,292,293,296,297,299,300,302,303,306,307,310,311,315,316,318,319,323,324,327,328,331,332,336,337,342,343,345,346`
+`,349,350,355,356,358,359,361,362,365,366,368,369,371,372,374,375,378,379,382,383,385,386,389,390,392],"polyslength":97,"regions":[3,3,1,1,1,1,1,26,9,9,9,4,4,36,37,38,15,27,27,31,31,31,31,25,25,19,19,19,18,18,18,10,24,24,24,14,32,6,6,6,40,33,7,7,7,12,41,2,2,2,20,8,8,8,16,16,16,16,16,34,11,11,21,22,5,5,5,5,13,13,13,13,13,28,28,28,28,28,28,28,28,17,17,17,17,17,17,17,17,17,43,44,45,35,48,49,50],"neighbors":[[[0],[0],[0],[1,52],[0],[1,1]],[[1,0],[0],[1,11],[0]],[[0],[0],[1,4]],[[0],[0],[1,5]],[[1,2],`
+`[0],[1,6]],[[0],[1,3],[1,6],[0],[0]],[[0],[0],[1,5],[0],[1,4],[1,47]],[[0],[0],[0],[0]],[[0],[1,35],[0],[1,9]],[[0],[0],[1,10],[1,8],[1,38],[0]],[[0],[0],[0],[1,9]],[[0],[1,43],[0],[1,12],[0],[1,1]],[[0],[1,11],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0],[0]],[[1,54],[0],[1,18],[0]],[[0],[0],[0],[1,17]],[[0],[0],[1,21],[0]],[[0],[0],[1,21]],[[1,19],[1,20],[0],[1,22]],[[0],[0],[1,21],[0]],[[0],[0],[0],[0],[1,24]],[[0],[0],[0],[1,23]],[[0],[1,63],[1,27]],[[0],[0],[1,27]],[[`
+`0],[1,26],[1,25],[0]],[[0],[1,62],[1,30]],[[0],[0],[1,30]],[[0],[1,29],[1,28],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[1,34]],[[0],[0],[1,34],[0],[0]],[[0],[1,32],[1,33]],[[0],[1,8],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[1,39]],[[0],[1,9],[1,39]],[[0],[1,37],[1,38],[0],[1,67]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,69],[0],[0],[1,43]],[[0],[0],[1,11],[1,44],[0],[1,42]],[[0],[0],[1,43],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,6],[1,49]],[[0],[0],[1,49]],[[0],[1,48],`
+`[0],[0],[1,47]],[[0],[0],[0],[0]],[[1,60],[0],[1,53]],[[0],[1,0],[1,53]],[[1,52],[0],[0],[1,51]],[[0],[1,17],[0],[1,56],[0]],[[0],[0],[0],[1,56],[1,57]],[[1,54],[0],[1,55],[0]],[[0],[1,55],[1,58]],[[0],[0],[1,57],[0]],[[0],[0],[0],[0]],[[0],[1,51],[1,61]],[[0],[0],[1,60],[0],[0]],[[1,28],[0],[0],[0],[0]],[[0],[0],[0],[1,25],[0],[0]],[[0],[0],[1,66],[0]],[[0],[0],[1,66]],[[0],[1,65],[1,67],[1,64],[0],[0]],[[1,39],[0],[1,66],[0]],[[0],[0],[1,70]],[[0],[1,42],[1,70]],[[1,68],[0],[1,72],[1,69],[0]],`
+`[[0],[0],[0],[1,72]],[[1,71],[0],[1,70]],[[0],[1,81],[1,76]],[[0],[0],[1,77],[0]],[[1,84],[0],[1,76],[0]],[[0],[1,75],[1,73],[1,79],[1,78]],[[1,74],[0],[1,78]],[[0],[1,77],[0],[1,80],[1,76]],[[0],[0],[1,76],[0]],[[1,82],[0],[1,78],[0]],[[1,73],[0],[1,85],[1,87],[0]],[[0],[1,80],[0],[0],[0],[1,85]],[[0],[0],[1,87]],[[0],[1,75],[0],[1,85]],[[1,82],[0],[1,89],[1,81],[1,84],[0]],[[0],[0],[1,88]],[[0],[1,83],[1,81]],[[1,86],[0],[1,89],[0]],[[1,88],[0],[1,85]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],`
+`[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[558,-541,-761,534,-549,-765,534,-677,-768,806,-677,-768,798,-549,-768,744,-543,-768,726,-541,-761,618,-569,-768,558,-541,-761,726,-541,-761,726,-315.5454406738281,-662,726,-293,-658,558,-293,-658,558,-315.5454406738281,-662,694,-605,-400,702,-645,-400,718,-621,-400,566,-677,-400,587.3333129882812,-645,-400,598,-629,-416,590,-605,-416,584,-623,-400,694,-605,-400,718,-621,-400,782,-605,-400,534,-677,-400,566,`
+`-677,-400,584,-623,-400,590,-605,-416,574,-405,-416,534,-413,-416,534,-501,-416,534,-567,-400,570,-545,-405,570,-593,-400,774,-333,-416,574,-325,-416,574,-405,-416,590,-605,-416,673.2000122070312,-605,-416,694,-605,-400,782,-605,-400,780.6666870117188,-559.6666870117188,-401,778.6666870117188,-491.6666564941406,-416,706,-545,-405,534,-557,128,534,-677,127,582,-677,119,582,-557,120,614,-533,74,622,-549,74,670,-549,74,686,-533,74,662,-397,74,582,-397,74,574,-421,74,614,-533,74,686,-533,74,678,-429`
+`,74,574,-421,74,534,-413,74,534,-541,74,614,-533,74,726,-293,-640,742,-285,-640,750,-165,-640,534,-165,-640,540.6666870117188,-265,-639,542,-285,-646,558,-293,-651,705,-293,-651,570,-257,-639,714,-257,-639,534,-533,-640,541.272705078125,-307.5454406738281,-640,542,-285,-646,540.6666870117188,-265,-639,534,-165,-640,534,-325,-339,534,-397,-339,558,-397,-339,558,-325,-339,534,-389,74,558,-381,74,534,-373,74,534,-381,-414,550,-341,-414,534,-333,-414,534,-269,118,534,-341,118,574,-357,118,758,-349,1`
+`18,758,-269,118,822,-317,-128,806,-293,-128,766,-293,-128,534,-317,-128,766,-293,-128,750,-269,-128,534,-269,-128,534,-317,-128,670,-261,-432,614,-261,-432,606,-277,-432,702,-277,-432,606,-277,-432,534,-261,-432,534,-293,-432,702,-277,-432,606,-277,-432,534,-293,-432,894,-285,-432,894,-253,-432,710,-261,-432,702,-277,-432,894,-285,-432,1006,-245,68,1046,-213,68,1046,-165,68,982,-165,68,982,-237,68,982,-237,68,534,-237,68,534,-245,68,1006,-245,68,766,-205,-72,814,-205,-72,822,-165,-72,534,-229,-7`
+`2,758,-229,-72,766,-205,-72,534,-165,-72,534,-229,-72,766,-205,-72,822,-165,-72,766,-205,-416,814,-205,-416,822,-165,-416,558,-237,-416,758,-237,-416,766,-205,-416,550,-165,-416,558,-237,-416,766,-205,-416,822,-165,-416,558,-261,-772,558,-397,-772,726,-397,-772,726,-261,-772,830,-677,-207,822,-637,-207,790,-621,-207,750,-621,-207,710,-661,-207,630,-621,-207,590,-645,-207,582,-677,-207,710,-661,-207,694,-637,-207,582,-677,-207,830,-677,-207,710,-661,-207,694,-565,74,670,-549,74,622,-549,74,598,-5`
+`65,74,598,-677,74,694,-677,74,638,-637,-315,606,-653,-315,606,-677,-315,646,-677,-315,670,-645,-315,830,-365,74,734,-389,74,726,-421,74,726,-421,74,678,-429,74,686,-533,74,870,-381,74,830,-365,74,726,-421,74,686,-533,74,878,-541,74,726,-573,74,726,-677,74,742,-677,74,742,-573,74,774,-637,-313,734,-653,-313,758,-677,-313,814,-677,-313,878,-317,-640,902,-333,-640,950,-325,-640,950,-253,-640,886,-197,-640,886,-197,-640,878,-165,-640,750,-165,-640,742,-285,-640,798,-333,-640,878,-317,-640,742,-533,-`
+`640,798,-533,-640,798,-333,-640,742,-285,-640,886,-677,119,886,-557,120,766,-557,146,758,-677,147,766,-573,74,766,-677,74,790,-677,74,790,-573,74,926,-413,-416,934,-325,-407,912.6666870117188,-319.6666564941406,-416,870,-309,-414,774,-333,-416,778.6666870117188,-491.6666564941406,-416,780.6666870117188,-559.6666870117188,-401,782,-605,-400,808.1818237304688,-570.0908813476562,-400,860.5454711914062,-500.2727355957031,-416,930,-353,-416,838,-637,-400,846,-677,-400,866,-677,-400,886,-677,-416,966,`
+`-677,-415,880.6666870117188,-650.3333129882812,-414,859.3333129882812,-643.6666870117188,-400,782,-605,-400,838,-637,-400,859.3333129882812,-643.6666870117188,-400,880.6666870117188,-650.3333129882812,-414,966,-677,-415,966,-421,-416,926,-413,-416,860.5454711914062,-500.2727355957031,-416,808.1818237304688,-570.0908813476562,-400,866,-569,-400,866,-641,-400,774,-269,74,774,-341,74,878,-317,74,958,-269,74,950,-461,-768,862,-477,-768,862,-525,-768,862,-525,-768,798,-549,-768,806,-677,-768,862,-525`
+`,-768,806,-677,-768,950,-677,-768,950,-461,-768,822,-269,-128,806,-293,-128,822,-317,-128,950,-317,-128,982,-269,-128,1006,-461,-128,1046,-461,-128,1046,-317,-128,990,-317,-128,966,-333,-128,982,-269,-128,950,-317,-128,966,-333,-128,990,-317,-128,982,-493,-128,1006,-461,-128,966,-333,-128,958,-677,-128,998,-677,-128,982,-493,-128,966,-333,-128,814,-477,-671,814,-517,-671,846,-517,-671,846,-477,-671,854,-461,-767,862,-477,-768,950,-461,-768,814,-261,-768,814,-461,-768,854,-461,-767,950,-461,-768,`
+`950,-261,-768,822,-165,-416,814,-205,-416,838,-229,-416,966,-229,-416,982,-165,-416,958,-229,-72,974,-197,-72,958,-165,-72,822,-165,-72,814,-205,-72,838,-229,-72,902,-677,74,910,-677,74,918,-637,74,902,-549,74,982,-301,74,918,-333,74,918,-365,74,1014,-317,74,982,-301,74,918,-365,74,902,-549,74,918,-637,74,1014,-645,74,870,-381,74,878,-541,74,902,-549,74,918,-365,74,902,-677,-640,950,-677,-640,950,-637,-640,942,-381,-640,950,-325,-640,902,-333,-640,902,-677,-640,950,-637,-640,990,-597,-640,942,-3`
+`81,-640,902,-333,-640,990,-597,-640,1022,-597,-640,1022,-413,-640,982,-413,-640,990,-597,-640,982,-413,-640,942,-381,-640,934,-565,-488,918,-557,-488,910,-557,-488,950,-677,-488,958,-677,-488,958,-645,-488,950,-597,-488,974,-541,-488,950,-541,-488,934,-565,-488,974,-581,-488,990,-589,-488,974,-581,-488,934,-565,-488,910,-557,-488,934,-589,-488,950,-597,-488,958,-645,-488,998,-605,-488,934,-589,-488,950,-597,-488,998,-605,-488,1030,-605,-488,990,-589,-488,910,-677,-488,918,-677,-488,934,-589,-488`
+`,910,-557,-488,1030,-461,-488,1014,-461,-488,990,-589,-488,1030,-605,-488,910,-557,-488,918,-557,-488,926,-533,-488,918,-405,-488,910,-325,-488,998,-453,-488,1014,-461,-488,1030,-461,-488,1030,-405,-488,998,-405,-488,990,-429,-488,918,-349,-488,934,-333,-488,910,-325,-488,926,-533,-488,950,-541,-488,974,-541,-488,974,-493,-488,998,-453,-488,990,-429,-488,958,-413,-488,918,-405,-488,926,-533,-488,974,-493,-488,966,-381,-488,990,-397,-488,966,-325,-488,918,-405,-488,918,-349,-488,910,-325,-488,966`
+`,-381,-488,966,-325,-488,942,-405,-488,958,-413,-488,958,-413,-488,942,-405,-488,918,-405,-488,910,-229,-488,934,-221,-488,910,-205,-488,926,-309,-488,950,-317,-488,958,-277,-488,934,-653,148,934,-677,148,998,-677,148,998,-653,148,1014,-245,-256,1046,-245,-256,1046,-165,-248,1014,-165,-256,1046,-285,-128,1046,-277,-128,1030,-269,-128,1046,-165,-416,1030,-165,-416,1030,-205,-416,1046,-213,-416,1038,-677,-240,1046,-677,-240,1046,-501,-240],"vertslength":430,"tris":[3,4,5,3,5,6,6,0,7,0,1,7,2,1,7,2,`
+`3,7,6,3,7,10,11,12,10,12,13,13,8,9,9,10,13,14,15,16,19,20,21,18,19,21,17,18,21,22,23,24,25,26,27,29,30,31,28,29,33,29,31,33,31,32,33,32,25,34,25,27,34,28,27,34,28,33,34,32,33,34,40,41,42,35,36,37,43,35,37,38,39,44,43,42,44,39,40,44,42,40,44,43,37,44,38,37,44,48,45,46,46,47,48,49,50,51,49,51,52,53,54,55,58,53,55,56,57,58,55,56,58,59,60,61,59,61,62,67,68,69,69,70,71,66,67,71,69,67,71,66,65,71,65,71,72,71,70,72,65,64,72,70,63,72,64,63,72,74,75,76,74,76,77,73,74,77,81,78,79,79,80,81,82,83,84,85,86,8`
+`7,88,89,90,90,91,92,88,90,92,93,94,95,93,95,96,97,98,99,97,99,100,101,102,103,101,103,104,105,106,107,108,109,110,108,110,111,112,113,114,112,114,115,120,116,117,117,118,119,117,119,120,121,122,123,121,123,124,125,126,127,128,129,130,131,132,133,131,133,134,135,136,137,138,139,140,141,142,143,141,143,144,148,145,146,146,147,148,149,150,151,151,152,153,149,151,153,154,155,156,157,158,154,154,156,157,159,160,161,162,163,164,162,164,165,167,162,165,165,166,167,169,170,171,171,172,168,168,169,171,17`
+`3,174,175,176,177,178,179,180,181,183,179,181,181,182,183,187,184,185,185,186,187,188,189,190,188,190,191,192,193,194,192,194,195,192,195,196,200,201,202,197,198,199,197,199,200,197,200,202,203,204,205,203,205,206,207,208,209,207,209,210,214,211,212,212,213,214,221,222,223,220,221,223,220,223,224,220,224,215,215,218,219,215,219,220,215,216,225,216,217,225,217,218,225,215,218,225,231,232,226,226,227,228,231,226,228,231,228,229,229,230,231,238,239,240,240,241,242,236,237,242,240,238,242,237,238,24`
+`2,241,233,243,236,235,243,233,234,243,235,234,243,236,242,243,241,242,243,244,245,246,244,246,247,248,249,250,251,252,253,254,255,256,254,256,257,258,259,260,258,260,261,258,261,262,265,266,267,267,263,264,264,265,267,269,270,271,268,269,271,272,273,274,275,276,277,275,277,278,282,279,280,280,281,282,283,284,285,286,287,288,288,289,290,286,288,290,291,292,293,291,293,294,291,294,295,296,297,298,299,300,301,301,296,298,298,299,301,302,303,304,302,304,305,306,307,308,309,310,311,312,313,314,309,31`
+`1,312,309,312,314,315,316,317,315,317,318,319,320,321,322,323,324,325,326,327,327,328,329,325,327,329,330,331,332,330,332,333,334,335,336,337,338,339,340,341,342,340,342,343,344,345,346,344,346,347,350,351,352,349,350,352,348,349,352,353,354,355,358,359,360,357,358,360,356,357,360,361,362,363,361,363,364,365,366,367,365,367,368,369,370,371,369,371,372,369,372,373,374,375,376,378,379,374,377,378,374,374,376,377,380,381,382,383,384,385,383,385,386,387,388,389,392,387,389,392,389,390,390,391,392,39`
+`3,394,395,396,397,398,401,402,399,399,400,401,403,404,405,406,407,408,409,410,411,415,412,413,413,414,415,419,416,417,417,418,419,420,421,422,423,424,425,423,425,426,427,428,429],"trislength":245,"triTopoly":[0,0,0,0,0,0,0,1,1,1,1,2,3,3,3,4,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,7,7,8,8,9,9,9,9,10,10,11,11,11,11,11,11,11,11,11,11,12,12,12,13,13,14,15,16,16,16,17,17,18,18,19,19,20,21,21,22,22,23,23,23,24,24,25,26,27,27,28,29,30,30,31,31,32,32,32,33,33,33,34,35,35,35,35,36,36,36,37,38,39,39,39,40,4`
+`0,41,41,42,42,42,43,43,43,43,44,44,45,45,46,46,47,47,47,47,47,47,47,47,47,47,48,48,48,48,48,49,49,49,49,49,49,49,49,49,49,49,50,50,51,52,53,53,54,54,54,55,55,55,56,56,57,58,58,59,59,60,61,61,61,62,62,62,63,63,63,63,64,64,65,66,66,66,66,67,67,68,69,70,70,70,71,71,72,73,74,74,75,75,76,76,76,77,78,78,78,79,79,80,80,81,81,81,82,82,82,82,83,84,84,85,85,85,85,86,87,88,88,89,90,91,92,92,93,93,94,95,95,96],"baseVert":[0,8,14,17,22,25,35,45,49,53,59,63,73,78,82,85,88,93,97,101,105,108,112,116,121,125,128`
+`,131,135,138,141,145,149,154,159,162,168,173,176,179,184,188,192,197,203,207,211,215,226,233,244,248,251,254,258,263,268,272,275,279,283,286,291,296,302,306,309,315,319,322,325,330,334,337,340,344,348,353,356,361,365,369,374,380,383,387,393,396,399,403,406,409,412,416,420,423,427],"vertsCount":[8,6,3,5,3,10,10,4,4,6,4,10,5,4,3,3,5,4,4,4,3,4,4,5,4,3,3,4,3,3,4,4,5,5,3,6,5,3,3,5,4,4,5,6,4,4,4,11,7,11,4,3,3,4,5,5,4,3,4,4,3,5,5,6,4,3,6,4,3,3,5,4,3,3,4,4,5,3,5,4,4,5,6,3,4,6,3,3,4,3,3,3,4,4,3,4,3],"bas`
+`eTri":[0,7,11,12,15,16,26,35,37,39,43,45,55,58,60,61,62,65,67,69,71,72,74,76,79,81,82,83,85,86,87,89,91,94,97,98,102,105,106,107,110,112,114,117,121,123,125,127,137,142,153,155,156,157,159,162,165,167,168,170,172,173,176,179,183,185,186,190,192,193,194,197,199,200,201,203,205,208,209,212,214,216,219,223,224,226,230,231,232,234,235,236,237,239,241,242,244],"triCount":[7,4,1,3,1,10,9,2,2,4,2,10,3,2,1,1,3,2,2,2,1,2,2,3,2,1,1,2,1,1,2,2,3,3,1,4,3,1,1,3,2,2,3,4,2,2,2,10,5,11,2,1,1,2,3,3,2,1,2,2,1,3,3,`
+`4,2,1,4,2,1,1,3,2,1,1,2,2,3,1,3,2,2,3,4,1,2,4,1,1,2,1,1,1,2,2,1,2,1]},"links":{"poly":[7,10,16,50,39,45,40,46,44,59,81,91],"cost":[3816.53466796875,3288,3654,864,1825.5,768],"type":[2,2,2,1,2,1],"pos":[582,-557,120,579.9406127929688,-536.4059448242188,74,758,-341,118,774,-341,74,878,-541,74,886,-557,120,742,-677,74,766,-677,74,798,-517,-640,814,-517,-671,910,-325,-488,926,-309,-488],"length":6}}],["8_4",{"tileId":"8_4","tx":8,"ty":4,"mesh":{"verts":[1214,-637,-240,1230,-629,-240,1174,-573,-240,1`
+`214,-677,-240,1214,-637,-240,1174,-573,-240,1046,-557,-240,1046,-677,-240,1302,-373,-127,1302,-405,-127,1318,-413,-128,1358,-301,-128,1334,-309,-128,1046,-557,-240,1110,-565,-231,1118,-549,-220,1046,-501,-240,1358,-301,-128,1318,-413,-128,1310,-501,-128,1358,-549,-128,1118,-549,-220,1358,-549,-128,1310,-501,-128,1046,-501,-240,1382,-293,-416,1326,-277,-416,1310,-293,-416,1390,-437,-416,1238,-565,-415,1326,-565,-415,1334,-549,-416,1390,-437,-416,1190,-541,-416,1166,-253,-415,1078,-253,-415,1078,-`
+`277,-416,1174,-285,-416,1414,-541,-415,1414,-445,-415,1390,-437,-416,1334,-549,-416,1174,-285,-416,1078,-277,-416,1046,-285,-416,1046,-541,-416,1062,-557,-416,1190,-541,-416,1310,-293,-416,1174,-285,-416,1046,-285,-416,1046,-541,-416,1190,-541,-416,1390,-437,-416,1102,-269,80,1110,-301,80,1238,-309,80,1286,-269,80,1286,-269,80,1238,-309,80,1238,-413,80,1278,-477,80,1238,-413,80,1046,-421,80,1046,-477,80,1278,-477,80,1046,-469,-128,1118,-469,-128,1126,-405,-128,1126,-405,-128,1190,-413,-128,1198,`
+`-317,-128,1046,-309,-128,1046,-469,-128,1126,-405,-128,1198,-317,-128,1046,-317,148,1046,-405,148,1222,-405,148,1222,-317,148,1046,-301,80,1062,-301,80,1046,-293,80,1046,-165,-248,1046,-245,-256,1238,-245,-256,1238,-165,-256,1046,-237,68,1046,-245,68,1070,-245,68,1094,-213,68,1046,-165,-416,1046,-221,-416,1078,-253,-415,1166,-253,-415,1222,-205,-416,1222,-165,-416,1046,-205,68,1094,-165,68,1046,-165,68,1334,-589,-416,1326,-565,-415,1238,-565,-415,1206,-613,-416,1366,-677,-416,1374,-589,-416,1334`
+`,-589,-416,1206,-613,-416,1054,-677,-416,1206,-613,-416,1054,-613,-416,1054,-677,-416,1054,-421,-598,1054,-597,-598,1086,-605,-610,1094,-365,-607,1054,-293,-248,1054,-349,-248,1190,-349,-248,1190,-293,-248,1070,-621,-640,1070,-637,-640,1078,-653,-640,1254,-653,-640,1102,-613,-640,1254,-653,-640,1262,-677,-640,1398,-677,-640,1398,-629,-640,1254,-653,-640,1398,-629,-640,1422,-621,-640,1422,-621,-640,1422,-389,-640,1398,-365,-640,1110,-365,-640,1102,-613,-640,1254,-653,-640,1430,-333,-640,1422,-205`
+`,-640,1390,-189,-640,1070,-197,-640,1070,-333,-640,1118,-189,68,1126,-213,68,1150,-213,68,1158,-181,68,1142,-165,68,1142,-165,68,1158,-181,68,1190,-189,68,1246,-165,68,1254,-221,68,1246,-165,68,1190,-189,68,1150,-245,68,1558,-165,68,1470,-165,68,1462,-221,68,1558,-229,68,1558,-229,68,1462,-221,68,1254,-221,68,1150,-245,68,1198,-557,-240,1174,-573,-240,1230,-629,-240,1198,-501,-240,1198,-557,-240,1230,-629,-240,1390,-637,-240,1310,-493,-240,1390,-269,-240,1310,-269,-240,1310,-493,-240,1390,-637,-`
+`240,1278,-413,-128,1302,-405,-127,1302,-373,-127,1278,-365,-128,1278,-365,-128,1278,-309,-128,1198,-317,-128,1190,-413,-128,1278,-413,-128,1190,-413,-128,1190,-469,-128,1278,-469,-128,1278,-413,-128,1270,-293,-248,1238,-293,-248,1246,-309,-248,1246,-309,-248,1222,-317,-248,1214,-373,-248,1270,-373,-248,1270,-373,-248,1270,-293,-248,1246,-309,-248,1230,-645,-160,1230,-677,-160,1374,-677,-160,1374,-645,-160,1558,-229,-448,1558,-197,-448,1254,-197,-448,1254,-253,-448,1454,-205,148,1454,-165,148,126`
+`2,-165,148,1262,-205,148,1446,-197,68,1446,-165,68,1278,-165,68,1278,-197,68,1406,-637,-223,1422,-621,-223,1422,-365,-223,1406,-349,-223,1414,-445,-415,1414,-541,-415,1430,-557,-416,1542,-557,-416,1558,-541,-407,1558,-445,-407,1542,-557,-416,1558,-445,-407,1542,-429,-416,1430,-421,-416,1414,-445,-415,1430,-557,-416,1558,-677,-416,1558,-565,-416,1542,-557,-416,1438,-621,-416,1422,-677,-416,1542,-557,-416,1430,-557,-416,1438,-621,-416,1558,-293,-416,1542,-269,-416,1526,-309,-416,1558,-413,-416,143`
+`0,-421,-416,1542,-429,-416,1558,-413,-416,1526,-309,-416,1422,-309,-416,1422,-269,-368,1422,-293,-368,1526,-293,-368,1526,-269,-368,1422,-205,-640,1430,-333,-640,1558,-333,-640,1558,-197,-640,1542,-389,-640,1558,-373,-640,1558,-365,-640,1454,-365,-640,1558,-397,-640,1542,-389,-640,1454,-365,-640,1454,-653,-640,1558,-653,-640],"vertslength":265,"polys":[0,2,3,7,8,12,13,16,17,20,21,24,25,28,29,33,34,37,38,41,42,44,45,47,48,53,54,57,58,61,62,65,66,68,69,71,72,75,76,79,80,82,83,86,87,90,91,96,97,99,`
+`100,103,104,108,109,111,112,115,116,119,120,124,125,128,129,131,132,137,138,142,143,147,148,151,152,155,156,159,160,163,164,166,167,171,172,175,176,179,180,184,185,188,189,191,192,195,196,198,199,202,203,206,207,210,211,214,215,218,219,221,222,224,225,230,231,235,236,238,239,242,243,247,248,251,252,255,256,259,260,264],"polyslength":65,"regions":[11,11,17,17,17,17,2,2,2,2,2,2,2,19,19,19,12,12,12,15,31,16,32,13,33,9,9,9,28,21,1,1,1,1,4,18,18,18,18,18,5,5,5,8,8,8,26,26,26,29,23,27,30,37,3,3,3,7,7,`
+`10,10,38,6,14,14],"neighbors":[[[0],[1,40],[1,1]],[[0],[1,0],[0],[0],[0]],[[1,43],[0],[1,4],[0],[0]],[[0],[0],[1,5],[0]],[[1,2],[0],[1,5],[0]],[[0],[1,4],[0],[1,3]],[[0],[0],[1,12],[0]],[[1,25],[0],[1,9],[1,12],[0]],[[1,23],[0],[1,10],[0]],[[1,54],[0],[1,7],[0]],[[1,8],[0],[1,12]],[[0],[0],[1,12]],[[0],[1,10],[0],[1,11],[1,7],[1,6]],[[0],[0],[1,14],[0]],[[1,13],[0],[1,15],[0]],[[0],[0],[0],[1,14]],[[0],[0],[1,18]],[[0],[1,44],[1,18]],[[0],[1,16],[1,17],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[`
+`0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,8],[0],[0],[0]],[[0],[0],[0]],[[0],[1,7],[0],[1,26]],[[0],[0],[1,25],[1,27],[0]],[[0],[0],[1,26]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,33],[0]],[[0],[0],[0],[1,32]],[[1,31],[0],[1,33]],[[0],[0],[0],[0],[1,30],[1,32]],[[1,62],[0],[0],[0],[0]],[[0],[0],[0],[1,36],[0]],[[1,35],[0],[1,37],[0]],[[0],[1,36],[0],[1,39]],[[0],[0],[1,39],[0]],[[1,38],[0],[1,37],[0]],[[0],[1,0],[1,41]],[[0],[1,40],[0],[1,42],[0]],[[0],[0],[1,41],[0]],[[0],[1,2],[0],`
+`[1,44]],[[0],[0],[1,17],[1,45],[1,43]],[[0],[0],[0],[1,44]],[[0],[0],[1,48]],[[0],[0],[0],[1,48]],[[0],[1,46],[1,47]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,9],[0],[1,56]],[[0],[0],[1,56]],[[1,55],[0],[1,60],[0],[1,54],[1,58]],[[0],[0],[1,58],[0],[0]],[[1,56],[0],[1,57]],[[0],[0],[1,60],[0]],[[1,56],[0],[1,59],[0],[0]],[[0],[0],[0],[0]],[[1,34],[0],[0],[0]],[[0],[0],[0],[1,64]],[[0],[1,63],[0],[0],[0]]]},"detail":{"verts":[1214,-637,-240,123`
+`0,-629,-240,1174,-573,-240,1214,-677,-240,1214,-637,-240,1174,-573,-240,1046,-557,-240,1046,-677,-240,1302,-373,-128,1302,-405,-128,1318,-413,-128,1358,-301,-128,1334,-309,-128,1046,-557,-240,1088.6666259765625,-562.3333129882812,-236,1110,-565,-220,1118,-549,-215,1082,-525,-240,1046,-501,-240,1358,-301,-128,1318,-413,-128,1310,-501,-128,1358,-549,-128,1118,-549,-215,1248.9090576171875,-549,-129,1358,-549,-128,1310,-501,-128,1244,-501,-135,1090,-501,-236,1046,-501,-240,1082,-525,-240,1382,-293,-`
+`416,1326,-277,-416,1310,-293,-416,1390,-437,-416,1238,-565,-416,1326,-565,-416,1334,-549,-416,1390,-437,-416,1190,-541,-416,1166,-253,-415,1078,-253,-415,1078,-277,-415,1174,-285,-416,1414,-541,-415,1414,-445,-415,1390,-437,-416,1334,-549,-416,1174,-285,-416,1078,-277,-415,1046,-285,-416,1046,-541,-416,1062,-557,-416,1190,-541,-416,1310,-293,-416,1174,-285,-416,1046,-285,-416,1046,-541,-416,1190,-541,-416,1390,-437,-416,1102,-269,80,1110,-301,80,1238,-309,80,1286,-269,80,1286,-269,80,1238,-309,8`
+`0,1238,-413,80,1278,-477,80,1238,-413,80,1046,-421,80,1046,-477,80,1278,-477,80,1046,-469,-128,1118,-469,-126,1126,-405,-128,1126,-405,-128,1190,-413,-128,1198,-317,-128,1046,-309,-128,1046,-469,-128,1126,-405,-128,1198,-317,-128,1046,-317,148,1046,-405,148,1222,-405,148,1222,-317,148,1046,-301,80,1062,-301,80,1046,-293,80,1046,-165,-248,1046,-245,-256,1238,-245,-256,1238,-165,-256,1195.3333740234375,-165,-248,1131.3333740234375,-165,-248,1110,-165,-256,1088.6666259765625,-165,-248,1202,-185,-25`
+`6,1046,-237,68,1046,-245,68,1070,-245,68,1094,-213,68,1046,-165,-416,1046,-221,-416,1078,-253,-416,1166,-253,-416,1222,-205,-416,1222,-165,-416,1046,-205,68,1094,-165,68,1046,-165,68,1334,-589,-416,1326,-565,-415,1238,-565,-415,1206,-613,-416,1366,-677,-416,1374,-589,-416,1334,-589,-416,1206,-613,-416,1054,-677,-416,1206,-613,-416,1054,-613,-416,1054,-677,-416,1054,-421,-601,1054,-597,-601,1086,-605,-610,1092.54541015625,-408.6363525390625,-610,1093.272705078125,-386.81817626953125,-616,1094,-36`
+`5,-607,1080.6666259765625,-383.6666564941406,-616,1090,-377,-607,1054,-293,-248,1054,-349,-248,1190,-349,-248,1190,-293,-248,1070,-621,-640,1070,-637,-640,1078,-653,-640,1254,-653,-640,1102,-613,-640,1254,-653,-640,1262,-677,-640,1398,-677,-640,1398,-629,-640,1254,-653,-640,1398,-629,-640,1422,-621,-640,1422,-621,-640,1422,-389,-640,1398,-365,-640,1110,-365,-640,1102,-613,-640,1254,-653,-640,1430,-333,-640,1422,-205,-640,1390,-189,-640,1070,-197,-640,1070,-333,-640,1118,-189,68,1126,-213,68,1150`
+`,-213,68,1158,-181,68,1142,-165,68,1142,-165,68,1158,-181,68,1190,-189,68,1246,-165,68,1254,-221,68,1246,-165,68,1190,-189,68,1150,-245,68,1558,-165,68,1470,-165,68,1462,-221,68,1558,-229,68,1558,-229,68,1462,-221,68,1254,-221,68,1150,-245,68,1198,-557,-240,1174,-573,-240,1230,-629,-240,1198,-501,-240,1198,-557,-240,1230,-629,-240,1390,-637,-240,1310,-493,-240,1390,-269,-240,1310,-269,-240,1310,-493,-240,1390,-637,-240,1278,-413,-127,1302,-405,-127,1302,-373,-127,1278,-365,-128,1278,-365,-128,12`
+`78,-309,-128,1198,-317,-128,1190,-413,-128,1278,-413,-127,1190,-413,-128,1190,-469,-128,1278,-469,-128,1278,-413,-127,1270,-293,-248,1238,-293,-248,1246,-309,-248,1246,-309,-248,1222,-317,-248,1214,-373,-248,1270,-373,-248,1270,-373,-248,1270,-293,-248,1246,-309,-248,1230,-645,-160,1230,-677,-160,1374,-677,-160,1374,-645,-160,1558,-229,-448,1558,-197,-448,1254,-197,-448,1254,-253,-448,1454,-205,148,1454,-165,148,1262,-165,148,1262,-205,148,1446,-197,68,1446,-165,68,1278,-165,68,1278,-197,68,1406`
+`,-637,-223,1422,-621,-223,1422,-365,-223,1406,-349,-223,1414,-445,-416,1414,-541,-416,1430,-557,-416,1542,-557,-407,1558,-541,-407,1558,-445,-407,1542,-557,-407,1558,-445,-407,1542,-429,-416,1430,-421,-416,1414,-445,-416,1430,-557,-416,1519.5999755859375,-557,-416,1522,-473,-416,1558,-677,-416,1558,-565,-416,1542,-557,-416,1438,-621,-416,1422,-677,-416,1542,-557,-416,1430,-557,-416,1438,-621,-416,1558,-293,-416,1542,-269,-416,1526,-309,-416,1558,-413,-416,1430,-421,-416,1542,-429,-416,1558,-413,`
+`-416,1526,-309,-416,1422,-309,-416,1422,-269,-368,1422,-293,-368,1526,-293,-368,1526,-269,-368,1422,-205,-640,1430,-333,-640,1558,-333,-640,1558,-197,-640,1542,-389,-640,1558,-373,-640,1558,-365,-640,1454,-365,-640,1558,-397,-640,1542,-389,-640,1454,-365,-640,1454,-653,-640,1558,-653,-640],"vertslength":282,"tris":[0,1,2,3,4,5,5,6,7,3,5,7,8,9,10,11,12,8,8,10,11,14,15,16,14,16,17,13,14,17,13,17,18,20,21,22,19,20,22,28,29,30,28,30,23,24,25,26,24,26,27,23,24,27,23,27,28,31,32,33,31,33,34,35,36,37,3`
+`9,35,37,37,38,39,40,41,42,40,42,43,44,45,46,44,46,47,48,49,50,51,52,53,54,55,56,58,59,54,56,57,58,54,56,58,60,61,62,60,62,63,64,65,66,64,66,67,68,69,70,68,70,71,72,73,74,75,76,77,78,79,80,78,80,81,85,82,83,83,84,85,86,87,88,96,89,90,95,96,90,94,95,90,90,91,94,93,94,97,94,91,97,91,92,97,93,92,97,98,99,100,98,100,101,102,103,104,105,106,107,102,104,105,102,105,107,108,109,110,111,112,113,111,113,114,115,116,117,115,117,118,115,118,119,120,121,122,126,127,129,126,129,123,123,124,125,123,125,126,127`
+`,128,130,128,129,130,129,127,130,134,131,132,132,133,134,135,136,137,139,135,137,137,138,139,140,141,142,140,142,143,144,145,146,147,148,149,151,152,147,149,150,151,147,149,151,153,154,155,155,156,157,153,155,157,158,159,160,161,162,158,158,160,161,163,164,165,163,165,166,167,168,169,167,169,170,171,172,173,171,173,174,175,176,177,175,177,178,179,180,181,182,183,184,186,182,184,184,185,186,187,188,189,187,189,190,191,192,193,191,193,194,195,196,197,198,199,195,195,197,198,203,200,201,201,202,203`
+`,204,205,206,207,208,209,207,209,210,211,212,213,217,214,215,215,216,217,218,219,220,218,220,221,225,222,223,223,224,225,229,226,227,227,228,229,230,231,232,230,232,233,234,235,236,237,238,239,241,242,247,243,242,247,243,244,247,245,244,247,245,246,247,241,240,247,246,240,247,248,249,250,251,252,248,248,250,251,253,254,255,256,257,258,256,258,259,261,262,263,263,264,260,260,261,263,268,265,266,266,267,268,269,270,271,269,271,272,273,274,275,273,275,276,277,278,279,280,281,277,277,279,280],"trisl`
+`ength":155,"triTopoly":[0,1,1,1,2,2,2,3,3,3,3,4,4,5,5,5,5,5,5,6,6,7,7,7,8,8,9,9,10,11,12,12,12,12,13,13,14,14,15,15,16,17,18,18,19,19,20,21,21,21,21,21,21,21,21,22,22,23,23,23,23,24,25,25,26,26,26,27,28,28,28,28,28,28,28,29,29,30,30,30,31,31,32,33,33,33,33,34,34,34,35,35,35,36,36,37,37,38,38,39,39,40,41,41,41,42,42,43,43,44,44,44,45,45,46,47,47,48,49,49,50,50,51,51,52,52,53,53,54,55,56,56,56,56,56,56,56,57,57,57,58,59,59,60,60,60,61,61,62,62,63,63,64,64,64],"baseVert":[0,3,8,13,19,23,31,35,40,44`
+`,48,51,54,60,64,68,72,75,78,82,86,89,98,102,108,111,115,120,123,131,135,140,144,147,153,158,163,167,171,175,179,182,187,191,195,200,204,207,211,214,218,222,226,230,234,237,240,248,253,256,260,265,269,273,277],"vertsCount":[3,5,5,6,4,8,4,5,4,4,3,3,6,4,4,4,3,3,4,4,3,9,4,6,3,4,5,3,8,4,5,4,3,6,5,5,4,4,4,4,3,5,4,4,5,4,3,4,3,4,4,4,4,4,3,3,8,5,3,4,5,4,4,4,5],"baseTri":[0,1,4,7,11,13,19,21,24,26,28,29,30,34,36,38,40,41,42,44,46,47,55,57,61,62,64,67,68,75,77,80,82,83,87,90,93,95,97,99,101,102,105,107,109`
+`,112,114,115,117,118,120,122,124,126,128,129,130,137,140,141,143,146,148,150,152],"triCount":[1,3,3,4,2,6,2,3,2,2,1,1,4,2,2,2,1,1,2,2,1,8,2,4,1,2,3,1,7,2,3,2,1,4,3,3,2,2,2,2,1,3,2,2,3,2,1,2,1,2,2,2,2,2,1,1,7,3,1,2,3,2,2,2,3]},"links":{"poly":[28,30,29,47,41,53,59,61],"cost":[1553.2940673828125,1105.9200439453125,817.5,3508.965576171875],"type":[2,1,2,2],"pos":[1086,-605,-610,1088.823486328125,-616.2941284179688,-640,1190,-349,-248,1216.8800048828125,-352.8399963378906,-248,1390,-637,-240,1406,-6`
+`37,-223,1531.5172119140625,-295.2069091796875,-416,1526,-293,-368],"length":4}}],["9_4",{"tileId":"9_4","tx":9,"ty":4,"mesh":{"verts":[1558,-565,-416,1558,-677,-416,1630,-677,-416,1630,-565,-416,1558,-365,-640,1558,-373,-640,1574,-381,-640,1638,-389,-640,1622,-365,-640,1622,-613,-640,1638,-389,-640,1574,-381,-640,1558,-397,-640,1558,-653,-640,1614,-637,-640,1806,-365,-640,1646,-365,-640,1638,-389,-640,1622,-613,-640,1638,-621,-640,1806,-653,-640,1806,-653,-640,1638,-621,-640,1630,-645,-640,1558,`
+`-653,-640,1806,-653,-640,1630,-645,-640,1558,-653,-640,1630,-645,-640,1614,-637,-640,1558,-445,-401,1558,-541,-401,1662,-549,-352,1662,-437,-352,1662,-677,-352,2054,-677,-336,2070,-661,-336,1662,-549,-352,1662,-677,-352,2070,-661,-336,2070,-269,-352,1662,-269,-352,1662,-437,-352,1558,-413,-416,1630,-413,-416,1630,-309,-416,1558,-301,-416,1822,-325,-640,1814,-197,-640,1558,-197,-640,1558,-333,-640,1558,-197,-448,1558,-229,-448,1654,-229,-448,1654,-197,-448,1558,-165,68,1558,-229,68,1646,-229,68,1`
+`646,-165,68,1566,-269,-368,1566,-293,-368,1630,-293,-368,1630,-269,-368,1814,-661,-448,1814,-373,-448,1670,-373,-448,1654,-661,-448,1654,-269,-448,1654,-357,-448,1670,-373,-448,1814,-373,-448,1814,-261,-448,1678,-237,-412,2070,-237,-412,2070,-165,-412,1678,-165,-412,1686,-677,-60,2054,-677,-60,2070,-661,-60,2070,-261,-60,1686,-261,-60],"vertslength":81,"polys":[0,3,4,8,9,14,15,20,21,23,24,26,27,29,30,33,34,36,37,42,43,46,47,50,51,54,55,58,59,62,63,66,67,71,72,75,76,80],"polyslength":19,"regions"`
+`:[7,3,3,3,3,3,3,1,1,1,8,6,11,10,12,4,5,9,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,2],[0],[0]],[[1,3],[1,1],[0],[0],[1,6],[0]],[[0],[0],[1,2],[0],[1,4],[0]],[[1,3],[0],[1,5]],[[0],[1,4],[1,6]],[[1,5],[0],[1,2]],[[0],[0],[1,9],[0]],[[0],[0],[1,9]],[[0],[1,8],[0],[0],[0],[1,7]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,16],[0],[0]],[[0],[0],[1,15],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]]]},"detail":{"verts":[1558,-565,-416,1558,-6`
+`77,-416,1630,-677,-416,1630,-565,-416,1558,-365,-640,1558,-373,-640,1574,-381,-640,1638,-389,-640,1622,-365,-640,1622,-613,-640,1638,-389,-640,1574,-381,-640,1558,-397,-640,1558,-653,-640,1614,-637,-640,1806,-365,-640,1646,-365,-640,1638,-389,-640,1622,-613,-640,1638,-621,-640,1806,-653,-640,1806,-653,-640,1638,-621,-640,1630,-645,-640,1558,-653,-640,1806,-653,-640,1630,-645,-640,1558,-653,-640,1630,-645,-640,1614,-637,-640,1558,-445,-396,1558,-541,-396,1620.4000244140625,-545.7999877929688,-359`
+`,1662,-549,-352,1662,-437,-352,1620.4000244140625,-440.20001220703125,-359,1618,-465,-359,1662,-677,-352,1984.823486328125,-677,-352,2054,-677,-336,2070,-661,-337,2002,-663.6666870117188,-352,1662,-549,-352,1662,-677,-352,2002,-663.6666870117188,-352,2070,-661,-337,2070,-591.8235473632812,-352,2070,-269,-352,1662,-269,-352,1662,-437,-352,2034,-641,-343,1558,-413,-416,1630,-413,-416,1630,-309,-416,1558,-301,-416,1822,-325,-640,1814,-197,-640,1558,-197,-640,1558,-333,-640,1558,-197,-448,1558,-229,`
+`-448,1654,-229,-448,1654,-197,-448,1558,-165,68,1558,-229,68,1646,-229,68,1646,-165,68,1566,-269,-368,1566,-293,-368,1630,-293,-368,1630,-269,-368,1814,-661,-448,1814,-373,-448,1670,-373,-448,1654,-661,-448,1654,-269,-448,1654,-357,-448,1670,-373,-448,1814,-373,-448,1814,-261,-448,1678,-237,-412,2070,-237,-412,2070,-165,-412,1678,-165,-412,1686,-677,-60,2054,-677,-60,2070,-661,-60,2070,-261,-60,1686,-261,-60],"vertslength":89,"tris":[3,0,1,1,2,3,4,5,6,8,4,6,6,7,8,13,14,9,10,11,12,9,10,12,9,12,13`
+`,15,16,17,18,19,20,15,17,18,15,18,20,21,22,23,24,25,26,27,28,29,30,31,36,32,31,36,32,33,36,34,33,36,34,35,36,30,35,36,39,40,41,38,39,41,37,38,41,44,46,47,42,43,44,49,42,44,47,48,49,44,47,49,44,45,50,45,46,50,46,44,50,51,52,53,51,53,54,55,56,57,55,57,58,62,59,60,60,61,62,66,63,64,64,65,66,70,67,68,68,69,70,71,72,73,71,73,74,75,76,77,77,78,79,75,77,79,83,80,81,81,82,83,84,85,86,86,87,88,84,86,88],"trislength":53,"triTopoly":[0,0,1,1,1,2,2,2,2,3,3,3,3,4,5,6,7,7,7,7,7,7,8,8,8,9,9,9,9,9,9,9,9,10,10,1`
+`1,11,12,12,13,13,14,14,15,15,16,16,16,17,17,18,18,18],"baseVert":[0,4,9,15,21,24,27,30,37,42,51,55,59,63,67,71,75,80,84],"vertsCount":[4,5,6,6,3,3,3,7,5,9,4,4,4,4,4,4,5,4,5],"baseTri":[0,2,5,9,13,14,15,16,22,25,33,35,37,39,41,43,45,48,50],"triCount":[2,3,4,4,1,1,1,6,3,8,2,2,2,2,2,2,3,2,3]},"links":{"poly":[10,14],"cost":[3573.0732421875],"type":[2],"pos":[1565.0244140625,-301.7804870605469,-416,1566,-293,-368],"length":1}}],["10_4",{"tileId":"10_4","tx":10,"ty":4,"mesh":{"verts":[2070,-653,-336,`
+`2142,-597,-336,2174,-525,-352,2070,-269,-352,2174,-525,-352,2214,-541,-344,2430,-525,-336,2438,-269,-352,2070,-269,-352,2174,-525,-60,2214,-541,-60,2318,-525,-60,2070,-261,-60,2070,-653,-60,2142,-597,-60,2174,-525,-60,2070,-261,-60,2318,-525,-60,2582,-533,-60,2582,-261,-60,2070,-261,-60,2070,-165,-412,2070,-237,-412,2582,-237,-412,2582,-165,-412,2198,-677,-60,2166,-621,-60,2142,-621,-60,2086,-677,-60,2222,-677,-352,2230,-677,-352,2238,-661,-352,2198,-613,-352,2238,-661,-352,2254,-677,-352,2270,-`
+`677,-352,2238,-581,-352,2206,-589,-352,2198,-613,-352,2310,-557,-60,2278,-557,-60,2262,-589,-60,2294,-677,-60,2310,-677,-60,2278,-605,-352,2294,-677,-352,2310,-677,-352,2310,-581,-352,2334,-565,-60,2334,-677,-60,2366,-677,-60,2366,-565,-60,2390,-677,-352,2462,-677,-352,2478,-613,-352,2398,-565,-352,2494,-565,-60,2430,-549,-60,2390,-549,-60,2390,-677,-60,2454,-677,-60,2430,-525,-336,2502,-549,-336,2518,-533,-344,2430,-525,-336,2518,-533,-344,2582,-533,-352,2582,-269,-352,2438,-269,-352,2486,-677,`
+`-352,2526,-677,-352,2510,-653,-352,2510,-653,-352,2542,-661,-352,2558,-637,-352,2558,-597,-352,2526,-581,-352,2486,-677,-352,2510,-653,-352,2526,-581,-352,2582,-677,-352,2582,-669,-352,2566,-661,-352,2558,-677,-352,2558,-677,-60,2582,-677,-60,2582,-645,-60],"vertslength":87,"polys":[0,3,4,8,9,12,13,16,17,20,21,24,25,28,29,32,33,38,39,43,44,47,48,51,52,55,56,60,61,63,64,68,69,71,72,76,77,79,80,83,84,86],"polyslength":21,"regions":[2,2,1,1,1,6,7,9,9,10,11,12,5,4,3,3,8,8,8,22,23],"neighbors":[[[0],`
+`[0],[1,1],[0]],[[0],[0],[1,15],[0],[1,0]],[[0],[0],[1,4],[1,3]],[[0],[0],[1,2],[0]],[[0],[0],[0],[1,2]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,8],[0]],[[0],[0],[0],[0],[0],[1,7]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[1,15]],[[1,14],[0],[0],[0],[1,1]],[[0],[0],[1,18]],[[0],[0],[0],[0],[1,18]],[[1,16],[1,17],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[2070,-653,-336,2124,-611,-336,2142,-597,-344,2174,-525,`
+`-352,2070,-269,-352,2070,-585.2352905273438,-352,2154,-545,-344,2174,-525,-352,2214,-541,-344,2257.199951171875,-537.7999877929688,-344,2278.800048828125,-536.2000122070312,-336,2408.39990234375,-526.5999755859375,-336,2430,-525,-344,2431.45458984375,-478.4545593261719,-352,2438,-269,-352,2070,-269,-352,2274,-505,-343,2174,-525,-60,2214,-541,-60,2318,-525,-60,2070,-261,-60,2070,-653,-60,2142,-597,-60,2174,-525,-60,2070,-261,-60,2318,-525,-60,2582,-533,-60,2582,-261,-60,2070,-261,-60,2070,-165,-4`
+`12,2070,-237,-412,2582,-237,-412,2582,-165,-412,2198,-677,-60,2166,-621,-60,2142,-621,-60,2086,-677,-60,2222,-677,-352,2230,-677,-352,2238,-661,-352,2198,-613,-352,2238,-661,-352,2254,-677,-352,2270,-677,-352,2238,-581,-352,2206,-589,-352,2198,-613,-352,2310,-557,-60,2278,-557,-60,2262,-589,-60,2294,-677,-60,2310,-677,-60,2278,-605,-352,2294,-677,-352,2310,-677,-352,2310,-581,-352,2334,-565,-60,2334,-677,-60,2366,-677,-60,2366,-565,-60,2390,-677,-352,2462,-677,-352,2478,-613,-352,2398,-565,-352,`
+`2494,-565,-60,2430,-549,-60,2390,-549,-60,2390,-677,-60,2454,-677,-60,2430,-525,-344,2466,-537,-336,2502,-549,-344,2518,-533,-343,2430,-525,-344,2518,-533,-343,2582,-533,-352,2582,-269,-352,2438,-269,-352,2431.45458984375,-478.4545593261719,-352,2486,-677,-352,2526,-677,-352,2510,-653,-352,2510,-653,-352,2542,-661,-352,2558,-637,-352,2558,-597,-352,2526,-581,-352,2486,-677,-352,2510,-653,-352,2526,-581,-352,2582,-677,-352,2582,-669,-352,2566,-661,-352,2558,-677,-352,2558,-677,-60,2582,-677,-60,2`
+`582,-645,-60],"vertslength":97,"tris":[5,0,1,5,1,2,5,2,6,2,3,6,3,4,6,5,4,6,11,12,13,13,14,16,10,9,16,7,8,16,9,8,16,13,11,16,10,11,16,14,15,16,7,15,16,17,18,19,17,19,20,21,22,23,21,23,24,25,26,27,25,27,28,32,29,30,30,31,32,33,34,35,33,35,36,37,38,39,37,39,40,41,42,43,44,45,46,44,46,41,41,43,44,47,48,49,49,50,51,47,49,51,52,53,54,52,54,55,59,56,57,57,58,59,60,61,62,60,62,63,64,65,66,66,67,68,64,66,68,70,71,72,69,70,72,78,73,74,78,74,75,76,77,78,75,76,78,79,80,81,82,83,84,84,85,86,82,84,86,87,88,89`
+`,90,91,92,90,92,93,94,95,96],"trislength":57,"triTopoly":[0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,8,8,9,9,9,10,10,11,11,12,12,13,13,13,14,14,15,15,15,15,16,17,17,17,18,19,19,20],"baseVert":[0,7,17,21,25,29,33,37,41,47,52,56,60,64,69,73,79,82,87,90,94],"vertsCount":[7,10,4,4,4,4,4,4,6,5,4,4,4,5,4,6,3,5,3,4,3],"baseTri":[0,6,15,17,19,21,23,25,27,31,34,36,38,40,43,45,49,50,53,54,56],"triCount":[6,9,2,2,2,2,2,2,4,3,2,2,2,3,2,4,1,3,1,2,1]},"links":{"poly":[],"cost":[],"type":[],"pos`
+`":[],"length":0}}],["11_4",{"tileId":"11_4","tx":11,"ty":4,"mesh":{"verts":[2606,-621,-60,2582,-629,-60,2582,-677,-60,2662,-677,-60,2694,-677,-336,2750,-677,-344,2742,-629,-344,2614,-605,-336,2582,-533,-352,2614,-605,-336,2742,-629,-344,2838,-621,-344,2798,-365,-352,2582,-269,-352,2798,-365,-352,2798,-269,-352,2582,-269,-352,2838,-621,-344,2846,-373,-352,2798,-365,-352,2582,-533,-60,2614,-605,-60,2694,-677,-60,3094,-677,-60,3094,-261,-60,2582,-261,-60,2582,-165,-412,2582,-237,-412,3094,-237,-412`
+`,3094,-165,-412,2590,-677,-352,2614,-677,-352,2614,-653,-352,2774,-637,-248,2774,-677,-248,2830,-677,-248,2830,-637,-248,2926,-349,-257,2918,-269,-257,2814,-269,-256,2814,-357,-256,2830,-285,-350,2830,-341,-350,2894,-341,-350,2894,-285,-350,2982,-637,-344,2974,-661,-344,3094,-677,-344,3094,-365,-352,2886,-677,-344,3094,-677,-344,2974,-661,-344,2886,-677,-344,2974,-661,-344,2950,-653,-344,2878,-645,-344,2878,-645,-344,2950,-653,-344,2958,-629,-344,2846,-373,-352,2838,-621,-344,2846,-373,-352,2958`
+`,-629,-344,2982,-637,-344,3094,-365,-352,2846,-653,-296,2846,-677,-296,2862,-677,-296,2862,-653,-296,2934,-269,-208,2934,-341,-208,2950,-341,-208,2950,-269,-208,2966,-269,-239,2966,-341,-239,3030,-341,-239,3030,-269,-239,3078,-309,-293,3094,-301,-299,3094,-269,-299,3046,-269,-293,3038,-357,-293,3078,-357,-293,3078,-309,-293,3046,-269,-293],"vertslength":85,"polys":[0,3,4,7,8,13,14,16,17,19,20,25,26,29,30,32,33,36,37,40,41,44,45,48,49,51,52,55,56,60,61,64,65,68,69,72,73,76,77,80,81,84],"polysleng`
+`th":21,"regions":[8,2,2,2,2,1,5,11,9,4,7,3,3,3,3,3,14,15,6,10,10],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,2],[0]],[[0],[1,1],[0],[1,4],[1,3],[0]],[[0],[0],[1,2]],[[1,14],[0],[1,2]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,12],[0],[1,15]],[[0],[1,11],[1,13]],[[1,12],[0],[1,14],[0]],[[1,13],[0],[1,15],[1,4],[0]],[[1,14],[0],[1,11],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,20]],[[0],[0`
+`],[1,19],[0]]]},"detail":{"verts":[2606,-621,-60,2582,-629,-60,2582,-677,-60,2662,-677,-60,2694,-677,-344,2750,-677,-344,2744.666748046875,-645,-344,2742,-629,-352,2614,-605,-344,2662,-648.2000122070312,-337,2582,-533,-352,2590,-551,-344,2614,-605,-344,2742,-629,-352,2838,-621,-352,2798,-365,-352,2582,-269,-352,2798,-365,-352,2798,-269,-352,2582,-269,-352,2838,-621,-352,2846,-373,-352,2798,-365,-352,2582,-533,-60,2614,-605,-60,2694,-677,-60,3094,-677,-60,3094,-261,-60,2582,-261,-60,2582,-165,-41`
+`2,2582,-237,-412,3094,-237,-412,3094,-165,-412,2590,-677,-352,2614,-677,-352,2614,-653,-352,2774,-637,-248,2774,-677,-248,2830,-677,-248,2830,-637,-248,2926,-349,-257,2918,-269,-257,2814,-269,-256,2814,-357,-256,2830,-285,-350,2830,-341,-350,2894,-341,-350,2894,-285,-350,2982,-637,-344,2974,-661,-344,3094,-677,-344,3094,-632.4285888671875,-344,3094,-610.1428833007812,-352,3094,-365,-352,2990.615478515625,-616.076904296875,-352,2886,-677,-344,3094,-677,-344,2974,-661,-344,2886,-677,-344,2974,-661`
+`,-344,2950,-653,-344,2878,-645,-344,2878,-645,-344,2950,-653,-344,2958,-629,-352,2846,-373,-352,2838,-621,-352,2846,-373,-352,2958,-629,-352,2982,-637,-344,2990.615478515625,-616.076904296875,-352,3094,-365,-352,2846,-653,-296,2846,-677,-296,2862,-677,-296,2862,-653,-296,2934,-269,-208,2934,-341,-208,2950,-341,-208,2950,-269,-208,2966,-269,-239,2966,-341,-239,3030,-341,-239,3030,-269,-239,3078,-309,-299,3094,-301,-299,3094,-269,-299,3046,-269,-293,3038,-357,-293,3078,-357,-293,3078,-309,-299,304`
+`6,-269,-293],"vertslength":92,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,9,4,7,7,8,9,11,12,13,10,11,13,10,13,14,15,16,10,10,14,15,17,18,19,20,21,22,23,24,25,28,23,25,25,26,27,25,27,28,32,29,30,30,31,32,33,34,35,39,36,37,37,38,39,40,41,42,40,42,43,47,44,45,45,46,47,54,48,49,54,49,50,54,50,51,54,51,52,52,53,54,55,56,57,60,61,58,58,59,60,62,63,64,66,62,64,64,65,66,68,69,70,67,68,70,67,70,71,75,72,73,73,74,75,79,76,77,77,78,79,83,80,81,81,82,83,84,85,86,84,86,87,88,89,90,88,90,91],"trislength":50,"triTopoly":[`
+`0,0,1,1,1,1,2,2,2,2,2,3,4,5,5,5,5,6,6,7,8,8,9,9,10,10,11,11,11,11,11,12,13,13,14,14,14,15,15,15,16,16,17,17,18,18,19,19,20,20],"baseVert":[0,4,10,17,20,23,29,33,36,40,44,48,55,58,62,67,72,76,80,84,88],"vertsCount":[4,6,7,3,3,6,4,3,4,4,4,7,3,4,5,5,4,4,4,4,4],"baseTri":[0,2,6,11,12,13,17,19,20,22,24,26,31,32,34,37,40,42,44,46,48],"triCount":[2,4,5,1,1,4,2,1,2,2,2,5,1,2,3,3,2,2,2,2,2]},"links":{"poly":[8,16,9,17,11,20,12,16,17,18,18,20],"cost":[3840,3981.697998046875,5701.5,4320,1825.5,4506.9833984`
+`375],"type":[2,2,2,2,2,2],"pos":[2830,-677,-248,2846,-677,-296,2918.158447265625,-270.58416748046875,-257,2934,-269,-208,3094,-365,-352,3078,-357,-293,2886,-677,-344,2862,-677,-296,2950,-341,-208,2966,-341,-239,3030,-341,-239,3039.376953125,-341.8524475097656,-293],"length":6}}],["12_4",{"tileId":"12_4","tx":12,"ty":4,"mesh":{"verts":[3366,-677,-352,3358,-549,-352,3326,-549,-352,3230,-637,-344,3222,-661,-343,3230,-637,-344,3326,-549,-352,3310,-525,-352,3206,-629,-344,3206,-629,-344,3310,-525,-35`
+`2,3302,-421,-352,3094,-413,-352,3094,-677,-344,3094,-677,-344,3366,-677,-352,3222,-661,-343,3094,-677,-344,3222,-661,-343,3198,-653,-344,3094,-677,-344,3198,-653,-344,3206,-629,-344,3526,-517,-60,3542,-509,-64,3526,-261,-60,3094,-677,-60,3526,-677,-60,3526,-517,-60,3526,-261,-60,3094,-261,-60,3142,-301,-352,3094,-317,-352,3094,-413,-352,3302,-421,-352,3318,-397,-352,3318,-397,-352,3502,-381,-352,3502,-269,-352,3142,-269,-352,3142,-301,-352,3094,-269,-299,3094,-301,-299,3126,-301,-299,3126,-269,-`
+`299,3094,-165,-412,3094,-237,-412,3510,-237,-412,3510,-165,-412,3334,-429,-237,3334,-477,-237,3342,-525,-237,3486,-509,-237,3502,-501,-237,3502,-413,-237,3390,-501,-308,3422,-501,-308,3422,-477,-308,3374,-461,-308,3342,-469,-308,3350,-517,-308,3374,-461,-308,3342,-437,-308,3374,-461,-308,3422,-477,-308,3414,-429,-308,3342,-437,-308,3358,-549,-352,3366,-677,-352,3502,-677,-352,3502,-533,-352,3478,-469,-335,3494,-493,-335,3494,-453,-335,3478,-421,-335,3542,-677,-352,3550,-677,-352,3550,-525,-352,3`
+`550,-165,-352,3542,-165,-352,3542,-189,-352],"vertslength":81,"polys":[0,4,5,8,9,13,14,16,17,19,20,22,23,25,26,30,31,35,36,40,41,44,45,48,49,54,55,58,59,62,63,66,67,70,71,74,75,77,78,80],"polyslength":20,"regions":[3,3,3,3,3,3,1,1,2,2,8,6,5,7,7,7,4,10,11,13],"neighbors":[[[1,16],[0],[1,1],[0],[1,3]],[[1,0],[0],[1,2],[0]],[[1,1],[0],[1,8],[0],[1,5]],[[0],[1,0],[1,4]],[[1,3],[0],[1,5]],[[1,4],[0],[1,2]],[[0],[0],[1,7]],[[0],[0],[1,6],[0],[0]],[[0],[0],[1,2],[0],[1,9]],[[0],[0],[0],[0],[1,8]],[[0],`
+`[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[1,15],[0]],[[0],[0],[1,15],[0]],[[1,13],[0],[0],[1,14]],[[1,0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[3366,-677,-352,3358,-549,-352,3326,-549,-352,3230,-637,-352,3222,-661,-344,3242.571533203125,-663.2857055664062,-352,3230,-637,-352,3326,-549,-352,3310,-525,-352,3206,-629,-352,3206,-629,-352,3310,-525,-352,3302,-421,-352,3094,-413,-352,3094,-611,-352,3094,-633,-344,3094,-677,-344,3187.333`
+`251953125,-637,-344,3094,-677,-344,3366,-677,-352,3242.571533203125,-663.2857055664062,-352,3222,-661,-344,3094,-677,-344,3222,-661,-344,3198,-653,-344,3094,-677,-344,3198,-653,-344,3206,-629,-352,3187.333251953125,-637,-344,3526,-517,-64,3542,-509,-64,3526,-261,-60,3094,-677,-60,3526,-677,-60,3526,-517,-64,3526,-261,-60,3094,-261,-60,3142,-301,-352,3094,-317,-352,3094,-413,-352,3302,-421,-352,3318,-397,-352,3318,-397,-352,3502,-381,-352,3502,-269,-352,3142,-269,-352,3142,-301,-352,3094,-269,-29`
+`9,3094,-301,-299,3126,-301,-299,3126,-269,-299,3094,-165,-412,3094,-237,-412,3510,-237,-412,3510,-165,-412,3334,-429,-237,3334,-477,-237,3342,-525,-237,3486,-509,-237,3502,-501,-237,3502,-413,-237,3390,-501,-308,3422,-501,-308,3422,-477,-308,3374,-461,-308,3342,-469,-308,3350,-517,-308,3374,-461,-308,3342,-437,-308,3374,-461,-308,3422,-477,-308,3414,-429,-308,3342,-437,-308,3358,-549,-352,3366,-677,-352,3502,-677,-352,3502,-533,-352,3478,-469,-335,3494,-493,-335,3494,-453,-335,3478,-421,-335,354`
+`2,-677,-352,3550,-677,-352,3550,-525,-352,3550,-165,-352,3542,-165,-352,3542,-189,-352],"vertslength":87,"tris":[3,4,5,0,1,2,3,5,0,0,2,3,6,7,8,6,8,9,15,16,17,14,15,17,14,17,10,10,11,12,13,14,10,10,12,13,20,21,18,18,19,20,22,23,24,26,27,28,25,26,28,29,30,31,32,33,34,34,35,36,32,34,36,37,38,39,40,41,37,37,39,40,45,46,42,42,43,44,42,44,45,50,47,48,48,49,50,54,51,52,52,53,54,55,56,57,58,59,60,55,57,58,55,58,60,61,62,63,61,63,64,67,68,65,65,66,67,69,70,71,69,71,72,73,74,75,73,75,76,77,78,79,77,79,80,`
+`81,82,83,84,85,86],"trislength":47,"triTopoly":[0,0,0,0,1,1,2,2,2,2,2,2,3,3,4,5,5,6,7,7,7,8,8,8,9,9,9,10,10,11,11,12,12,12,12,13,13,14,14,15,15,16,16,17,17,18,19],"baseVert":[0,6,10,18,22,25,29,32,37,42,47,51,55,61,65,69,73,77,81,84],"vertsCount":[6,4,8,4,3,4,3,5,5,5,4,4,6,4,4,4,4,4,3,3],"baseTri":[0,4,6,12,14,15,17,18,21,24,27,29,31,35,37,39,41,43,45,46],"triCount":[4,2,6,2,1,2,1,3,3,3,2,2,4,2,2,2,2,2,1,1]},"links":{"poly":[8,10],"cost":[4251.89990234375],"type":[2],"pos":[3127.60009765625,-305`
+`.79998779296875,-352,3126,-301,-299],"length":1}}],["0_5",{"tileId":"0_5","tx":0,"ty":5,"mesh":{"verts":[-3018,-165,-408,-3010,-165,-408,-3010,-29,-408,-2858,-117,-408,-2874,-125,-408,-2858,-165,-408,-2818,-5,-408,-2826,-29,-408,-2858,-165,-408,-2602,-165,-416,-2586,-149,-416,-2586,11,-416,-2858,-117,-408,-2858,-165,-408,-2826,-29,-408,-2850,-21,-408,-2866,-5,-408,-2858,-117,-408,-2850,-21,-408,-2866,-5,-408,-2850,-21,-408,-2842,3,-408,-3010,-13,-408,-2866,-5,-408,-2842,3,-408,-3010,347,-408,-28`
+`42,3,-408,-2818,-5,-408,-2586,11,-416,-2538,19,-416,-2538,347,-416,-3010,347,-408,-3002,-13,-272,-3002,-165,-272,-2866,-165,-272,-2866,-13,-272,-2858,-165,-408,-2874,-125,-408,-2890,-117,-408,-2978,-165,-408,-2890,-117,-408,-2890,-37,-408,-2978,-37,-408,-2978,-165,-408,-2570,3,-400,-2570,-165,-400,-2538,-165,-400,-2538,3,-400],"vertslength":48,"polys":[0,2,3,5,6,11,12,15,16,18,19,21,22,25,26,31,32,35,36,39,40,43,44,47],"polyslength":12,"regions":[5,1,1,1,1,1,1,1,2,3,3,4],"neighbors":[[[0],[0],[0`
+`]],[[0],[1,9],[1,3]],[[0],[1,3],[0],[0],[0],[1,7]],[[1,1],[1,2],[0],[1,4]],[[0],[1,3],[1,5]],[[1,4],[0],[1,6]],[[0],[1,5],[1,7],[0]],[[0],[1,2],[0],[0],[0],[1,6]],[[0],[0],[0],[0]],[[1,1],[0],[1,10],[0]],[[0],[0],[0],[1,9]],[[0],[0],[0],[0]]]},"detail":{"verts":[-3018,-165,-408,-3010,-165,-408,-3010,-29,-408,-2858,-117,-408,-2874,-125,-408,-2858,-165,-408,-2818,-5,-416,-2826,-29,-408,-2858,-165,-408,-2811.45458984375,-165,-416,-2602,-165,-416,-2586,-149,-416,-2586,11,-416,-2822,-153,-408,-2858,-`
+`117,-408,-2858,-165,-408,-2826,-29,-408,-2850,-21,-408,-2866,-5,-408,-2858,-117,-408,-2850,-21,-408,-2866,-5,-408,-2850,-21,-408,-2842,3,-408,-3010,-13,-408,-2866,-5,-408,-2842,3,-408,-3010,347,-408,-2842,3,-408,-2818,-5,-416,-2586,11,-416,-2538,19,-416,-2538,347,-416,-3010,347,-408,-2806,7,-416,-3002,-13,-272,-3002,-165,-272,-2866,-165,-272,-2866,-13,-272,-2858,-165,-408,-2874,-125,-408,-2890,-117,-408,-2978,-165,-408,-2890,-117,-408,-2890,-37,-408,-2978,-37,-408,-2978,-165,-408,-2570,3,-400,-2`
+`570,-165,-400,-2538,-165,-400,-2538,3,-400],"vertslength":51,"tris":[0,1,2,3,4,5,6,7,9,10,11,12,6,9,10,6,10,12,7,8,13,8,9,13,9,7,13,16,17,14,14,15,16,18,19,20,21,22,23,24,25,26,24,26,27,31,32,34,31,30,34,28,29,34,30,29,34,32,33,34,28,33,34,38,35,36,36,37,38,39,40,41,39,41,42,43,44,45,43,45,46,50,47,48,48,49,50],"trislength":29,"triTopoly":[0,1,2,2,2,2,2,2,2,3,3,4,5,6,6,7,7,7,7,7,7,8,8,9,9,10,10,11,11],"baseVert":[0,3,6,14,18,21,24,28,35,39,43,47],"vertsCount":[3,3,8,4,3,3,4,7,4,4,4,4],"baseTri":`
+`[0,1,2,9,11,12,13,15,21,23,25,27],"triCount":[1,1,7,2,1,1,2,6,2,2,2,2]},"links":{"poly":[0,6,2,11],"cost":[384,768],"type":[1,2],"pos":[-3010,-29,-408,-3010,-13,-408,-2586,-149,-416,-2570,-149,-400],"length":2}}],["1_5",{"tileId":"1_5","tx":1,"ty":5,"mesh":{"verts":[-2538,3,-400,-2538,-165,-400,-2522,-165,-400,-2514,3,-400,-2490,331,-408,-2522,347,-408,-2538,347,-416,-2522,187,-408,-2514,211,-408,-2490,331,-408,-2514,211,-408,-2490,203,-408,-2026,347,-408,-2482,347,-408,-2522,187,-408,-2538,347,`
+`-416,-2538,19,-416,-2498,179,-408,-2522,187,-408,-2538,19,-416,-2490,203,-408,-2498,179,-408,-2538,19,-416,-2026,19,-416,-2026,347,-408,-2514,-165,-30,-2026,-165,-36,-2026,-133,-36,-2514,-125,-30,-2514,-133,112,-2026,-133,112,-2026,11,112,-2514,11,112],"vertslength":33,"polys":[0,3,4,8,9,13,14,16,17,19,20,24,25,28,29,32],"polyslength":8,"regions":[4,1,1,1,1,1,3,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,3],[0],[1,2]],[[1,1],[0],[1,5],[0],[0]],[[1,1],[0],[1,4]],[[0],[1,3],[1,5]],[[0],[1,4],[0]`
+`,[0],[1,2]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2538,3,-400,-2538,-165,-400,-2522,-165,-400,-2514,3,-400,-2490,331,-408,-2522,347,-408,-2538,347,-408,-2522,187,-408,-2514,211,-408,-2490,331,-408,-2514,211,-408,-2490,203,-408,-2026,347,-408,-2482,347,-408,-2522,187,-408,-2538,347,-408,-2538,19,-416,-2526,145,-416,-2526,175,-408,-2498,179,-408,-2522,187,-408,-2526,145,-416,-2538,19,-416,-2490,203,-408,-2498,179,-408,-2538,19,-416,-2026,19,-416,-2026,89.28571319580078,-416,-20`
+`26,112.71428680419922,-408,-2026,347,-408,-2094,103,-416,-2514,-165,-36,-2026,-165,-36,-2026,-133,-36,-2490.761962890625,-125.38095092773438,-36,-2514,-125,-30,-2514,-133,112,-2026,-133,112,-2026,11,112,-2514,11,112],"vertslength":40,"tris":[0,1,2,0,2,3,4,5,6,7,8,4,4,6,7,9,10,11,13,9,11,11,12,13,17,14,18,14,15,18,15,16,18,17,16,18,19,20,21,19,21,22,23,24,30,25,24,30,28,27,30,25,26,30,27,26,30,28,29,30,23,29,30,34,35,31,32,33,34,31,32,34,39,36,37,37,38,39],"trislength":26,"triTopoly":[0,0,1,1,1,2`
+`,2,2,3,3,3,3,4,4,5,5,5,5,5,5,5,6,6,6,7,7],"baseVert":[0,4,9,14,19,23,31,36],"vertsCount":[4,5,5,5,4,8,5,4],"baseTri":[0,2,5,8,12,14,21,24],"triCount":[2,3,3,4,2,7,3,2]},"links":{"poly":[0,3],"cost":[768],"type":[2],"pos":[-2538,3,-400,-2538,19,-416],"length":1}}],["2_5",{"tileId":"2_5","tx":2,"ty":5,"mesh":{"verts":[-2026,-133,-36,-2026,-165,-36,-1690,-165,-36,-1690,-133,-36,-1690,-133,118,-1682,-165,118,-1514,-165,112,-1514,11,112,-2026,11,112,-2026,-133,112,-1690,-133,118,-1514,11,112,-2026,34`
+`7,-408,-2026,19,-416,-1514,19,-416,-1514,347,-408],"vertslength":16,"polys":[0,3,4,7,8,11,12,15],"polyslength":4,"regions":[3,2,2,1],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[1,2]],[[0],[0],[1,1],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2026,-133,-36,-2026,-165,-36,-1690,-165,-36,-1690,-133,-36,-1690,-133,112,-1686,-149,118,-1682,-165,112,-1514,-165,112,-1514,11,112,-2026,11,112,-2026,-133,112,-1690,-133,112,-1514,11,112,-2026,347,-408,-2026,112.71428680419922,-408,-2026,89.28571319580078`
+`,-416,-2026,19,-416,-1514,19,-416,-1514,89.28571319580078,-416,-1514,112.71428680419922,-408,-1514,347,-408,-1966,103,-416],"vertslength":22,"tris":[3,0,1,1,2,3,5,6,7,4,5,7,4,7,8,9,10,11,9,11,12,17,18,21,14,15,21,17,16,21,15,16,21,18,19,21,19,20,21,14,13,21,20,13,21],"trislength":15,"triTopoly":[0,0,1,1,1,2,2,3,3,3,3,3,3,3,3],"baseVert":[0,4,9,13],"vertsCount":[4,5,4,9],"baseTri":[0,2,5,7],"triCount":[2,3,2,8]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_5",{"tileId":"3_5",`
+`"tx":3,"ty":5,"mesh":{"verts":[-1514,11,112,-1514,-165,112,-1362,-165,112,-1362,11,112,-1514,347,-408,-1514,19,-416,-1002,19,-416,-1002,347,-408,-1354,-13,-52,-1354,-165,-52,-1002,-165,-52,-1002,-13,-52],"vertslength":12,"polys":[0,3,4,7,8,11],"polyslength":3,"regions":[2,1,3],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-1514,11,112,-1514,-165,112,-1362,-165,112,-1362,11,112,-1514,347,-408,-1514,112.71428680419922,-408,-1514,89.28571319580078,-416,-151`
+`4,19,-416,-1002,19,-416,-1002,89.28571319580078,-416,-1002,112.71428680419922,-408,-1002,347,-408,-1454,103,-416,-1354,-13,-52,-1354,-165,-52,-1002,-165,-52,-1002,-13,-52],"vertslength":17,"tris":[3,0,1,1,2,3,8,9,12,5,6,12,8,7,12,6,7,12,9,10,12,10,11,12,5,4,12,11,4,12,16,13,14,14,15,16],"trislength":12,"triTopoly":[0,0,1,1,1,1,1,1,1,1,2,2],"baseVert":[0,4,13],"vertsCount":[4,9,4],"baseTri":[0,2,10],"triCount":[2,8,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_5",{"tileId"`
+`:"4_5","tx":4,"ty":5,"mesh":{"verts":[-1002,-13,-52,-1002,-165,-52,-962,-165,-52,-962,-13,-52,-890,147,-408,-906,163,-408,-1002,171,-408,-1002,19,-416,-898,19,-416,-538,283,-408,-514,275,-408,-490,347,-408,-1002,171,-408,-906,163,-408,-882,179,-408,-874,155,-408,-890,147,-408,-898,19,-416,-490,-165,-408,-514,19,-408,-522,-5,-408,-490,-165,-408,-522,-5,-408,-546,3,-408,-890,-165,-416,-1002,347,-408,-1002,171,-408,-882,179,-408,-546,259,-408,-538,283,-408,-874,155,-408,-898,19,-416,-890,-165,-416,`
+`-546,3,-408,-538,27,-408,-546,259,-408,-514,19,-408,-490,-165,-408,-490,347,-408,-522,251,-408,-538,27,-408,-514,19,-408,-522,251,-408,-546,259,-408,-538,283,-408,-490,347,-408,-1002,347,-408,-546,259,-408,-882,179,-408,-874,155,-408,-938,-165,-416,-930,-165,-416,-930,-21,-416,-906,3,-288,-938,3,-288,-914,-13,-288,-914,-13,-288,-914,-165,-288,-906,-165,-288,-906,3,-288],"vertslength":60,"polys":[0,3,4,8,9,11,12,14,15,17,18,20,21,24,25,29,30,35,36,39,40,43,44,46,47,49,50,52,53,55,56,59],"polyslen`
+`gth":16,"regions":[3,2,1,1,1,1,1,1,1,1,1,1,1,5,6,6],"neighbors":[[[0],[0],[0],[0]],[[0],[1,3],[0],[0],[1,4]],[[0],[0],[1,11]],[[1,1],[0],[1,7]],[[0],[1,1],[1,8]],[[1,9],[0],[1,6]],[[1,5],[0],[1,8],[0]],[[0],[1,3],[1,12],[0],[1,11]],[[1,4],[0],[1,6],[0],[1,10],[1,12]],[[1,5],[0],[0],[1,10]],[[0],[1,9],[0],[1,8]],[[1,2],[0],[1,7]],[[1,7],[0],[1,8]],[[0],[0],[0]],[[0],[0],[1,15]],[[0],[0],[0],[1,14]]]},"detail":{"verts":[-1002,-13,-52,-1002,-165,-52,-962,-165,-52,-962,-13,-52,-890,147,-408,-906,163`
+`,-408,-1002,171,-408,-1002,19,-416,-898,19,-416,-892.6666870117188,104.33333587646484,-416,-918,127,-408,-538,283,-408,-514,275,-408,-490,347,-408,-1002,171,-408,-906,163,-408,-882,179,-408,-874,155,-408,-890,147,-408,-892.6666870117188,104.33333587646484,-416,-898,19,-416,-882,109.66666412353516,-416,-490,-165,-408,-514,19,-408,-522,-5,-408,-490,-165,-408,-522,-5,-408,-546,3,-408,-567.5,-7.5,-416,-890,-165,-416,-584.11767578125,-165,-416,-560.5882568359375,-165,-408,-566,-153,-416,-1002,347,-40`
+`8,-1002,171,-408,-882,179,-408,-837.2000122070312,189.6666717529297,-416,-568.4000244140625,253.6666717529297,-416,-546,259,-408,-538,283,-408,-584.4000244140625,289.3999938964844,-416,-677.2000122070312,302.20001220703125,-416,-846,207,-408,-874,155,-408,-882,109.66666412353516,-416,-898,19,-416,-890,-165,-416,-567.5,-7.5,-416,-546,3,-408,-538,27,-408,-546,259,-408,-567.8666381835938,252.06666564941406,-416,-830.2666625976562,168.86666870117188,-416,-514,19,-408,-490,-165,-408,-490,347,-408,-52`
+`2,251,-408,-538,27,-408,-514,19,-408,-522,251,-408,-546,259,-408,-538,283,-408,-490,347,-408,-559.8181762695312,347,-408,-583.0908813476562,347,-416,-1002,347,-408,-839.5999755859375,324.6000061035156,-416,-584.4000244140625,289.3999938964844,-416,-546,259,-408,-568.4000244140625,253.6666717529297,-416,-837.2000122070312,189.6666717529297,-416,-882,179,-408,-874,155,-408,-830.2666625976562,168.86666870117188,-416,-567.8666381835938,252.06666564941406,-416,-938,-165,-416,-930,-165,-416,-930,-21,-`
+`416,-906,3,-288,-938,3,-288,-914,-13,-288,-914,-13,-288,-914,-165,-288,-906,-165,-288,-906,3,-288],"vertslength":85,"tris":[3,0,1,1,2,3,7,8,9,5,6,10,9,4,10,5,4,10,6,7,10,9,7,10,11,12,13,14,15,16,21,17,18,21,18,19,19,20,21,22,23,24,26,27,28,28,29,30,30,31,32,28,30,32,31,25,32,28,26,32,25,26,32,37,38,39,37,39,40,37,40,41,36,37,41,36,41,42,41,33,42,33,34,42,36,35,42,34,35,42,47,48,49,52,43,44,52,44,45,49,50,51,47,49,51,52,45,46,47,51,52,46,47,52,55,56,53,53,54,55,57,58,59,57,59,60,61,62,63,61,63,64`
+`,67,61,64,66,67,64,64,65,66,74,68,69,70,71,72,70,72,73,73,74,69,69,70,73,75,76,77,78,79,80,81,82,83,81,83,84],"trislength":56,"triTopoly":[0,0,1,1,1,1,1,1,2,3,4,4,4,5,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,9,9,10,10,11,11,11,11,11,12,12,12,12,12,13,14,15,15],"baseVert":[0,4,11,14,17,22,25,33,43,53,57,61,68,75,78,81],"vertsCount":[4,7,3,3,5,3,8,10,10,4,4,7,7,3,3,4],"baseTri":[0,2,8,9,10,13,14,21,30,38,40,42,47,52,53,54],"triCount":[2,6,1,1,3,1,7,9,8,2,2,5,5,1,1,2]},"links":{"poly":[],"co`
+`st":[],"type":[],"pos":[],"length":0}}],["5_5",{"tileId":"5_5","tx":5,"ty":5,"mesh":{"verts":[-490,347,-408,-490,-165,-408,-394,-165,-408,-394,347,-408,-362,-165,-224,-306,-165,-224,-322,131,-224,-362,147,-224,-242,-165,68,-258,-125,68,-314,-117,68,-362,-165,68,-314,-117,68,-322,123,68,-362,139,68,-362,-165,68,-362,139,68,-322,123,68,-314,139,68,-210,139,68,-202,123,68,-106,123,68,-106,347,68,-362,347,68,-362,139,68,-314,139,68,-210,139,68,-106,347,68,-362,147,-224,-322,131,-224,-306,139,-224,-3`
+`06,347,-224,-362,347,-224,22,59,-416,-50,43,-416,-74,35,-416,22,-165,-416,22,-165,-416,-74,35,-416,-82,51,-416,-346,67,-416,-346,-165,-416,-58,59,-416,-50,43,-416,22,59,-416,-346,67,-416,-82,51,-416,-58,59,-416,-58,59,-416,22,59,-416,22,347,-416,-346,347,-416,-346,67,-416,-298,-109,119,-258,-109,119,-258,-77,148,-306,-77,148,-306,-77,148,-258,-77,148,-218,-69,148,-218,123,148,-306,123,148,-258,-93,-160,-258,-77,-160,-298,-77,-160,-298,-101,-160,-298,115,68,-298,-61,68,-226,-61,68,-226,115,68,-15`
+`4,347,-160,-298,347,-160,-298,43,-160,14,147,-160,-2,155,-160,-82,115,-160,-82,51,-160,14,-149,-160,14,147,-160,-82,51,-160,-274,-149,-160,-202,-77,68,-242,-85,68,-258,-125,68,-50,-29,68,14,-29,68,14,3,68,-98,3,68,-202,-77,68,-258,-125,68,-242,-165,68,-50,-165,68,-50,-29,68,-98,3,68,-98,3,68,-106,123,68,-202,123,68,-202,-77,68,-34,-37,124,-34,-165,124,-18,-165,124,-18,-37,124,-18,-165,-224,22,-165,-224,22,-149,-224,6,-85,68,6,-165,68,22,-165,68,22,-77,68],"vertslength":109,"polys":[0,3,4,7,8,11,`
+`12,15,16,18,19,22,23,27,28,32,33,36,37,41,42,44,45,47,48,52,53,56,57,61,62,65,66,69,70,72,73,76,77,80,81,83,84,87,88,93,94,97,98,101,102,104,105,108],"polyslength":27,"regions":[7,10,12,12,3,3,3,11,2,2,1,1,1,8,8,14,9,6,4,4,5,5,5,5,17,18,19],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[1,7],[0]],[[1,22],[0],[1,3],[0]],[[0],[1,4],[0],[1,2]],[[1,3],[0],[1,6]],[[0],[1,23],[0],[1,6]],[[0],[1,4],[0],[1,5],[0]],[[1,1],[0],[0],[0],[0]],[[1,10],[0],[1,9],[0]],[[1,8],[0],[1,11],[0],[0]],[[0],[1,8],[1,12]],[[1`
+`,9],[0],[1,12]],[[1,10],[0],[0],[0],[1,11]],[[0],[0],[1,14],[0]],[[1,13],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,19]],[[0],[1,18],[0],[0]],[[0],[0],[1,22]],[[0],[0],[0],[1,22]],[[1,20],[1,2],[0],[0],[1,21],[1,23]],[[0],[1,5],[0],[1,22]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-490,347,-408,-490,-165,-408,-394,-165,-408,-394,347,-408,-362,-165,-224,-306,-165,-224,-322,131,-224,-362,147,-224,-242,-165,68,-258,-125,68,-314,-1`
+`17,68,-362,-165,68,-314,-117,68,-322,123,68,-362,139,68,-362,-165,68,-362,139,68,-322,123,68,-314,139,68,-210,139,68,-202,123,68,-106,123,68,-106,347,68,-362,347,68,-362,139,68,-314,139,68,-210,139,68,-106,347,68,-362,147,-224,-322,131,-224,-306,139,-224,-306,347,-224,-362,347,-224,22,59,-416,-50,43,-416,-74,35,-416,22,-165,-416,22,-165,-416,-74,35,-416,-82,51,-416,-346,67,-416,-346,-165,-416,-58,59,-416,-50,43,-416,22,59,-416,-346,67,-416,-82,51,-416,-58,59,-416,-58,59,-416,22,59,-416,22,347,-4`
+`16,-346,347,-416,-346,67,-416,-298,-109,124,-258,-109,124,-258,-93,148,-258,-77,148,-306,-77,148,-306,-77,148,-258,-77,148,-218,-69,148,-218,123,148,-306,123,148,-258,-93,-160,-258,-77,-160,-298,-77,-160,-298,-101,-160,-298,115,68,-298,-61,68,-226,-61,68,-226,115,68,-154,347,-160,-298,347,-160,-298,43,-160,14,147,-160,-2,155,-160,-82,115,-160,-82,51,-160,14,-149,-160,14,147,-160,-82,51,-160,-274,-149,-160,-202,-77,68,-242,-85,68,-258,-125,68,-50,-29,68,14,-29,68,14,3,68,-98,3,68,-202,-77,68,-258`
+`,-125,68,-242,-165,68,-50,-165,68,-50,-29,68,-98,3,68,-98,3,68,-106,123,68,-202,123,68,-202,-77,68,-34,-37,124,-34,-165,124,-18,-165,124,-18,-37,124,-18,-165,-224,22,-165,-224,22,-149,-224,6,-85,68,6,-165,68,22,-165,68,22,-77,68],"vertslength":110,"tris":[3,0,1,1,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,12,14,15,16,17,18,19,20,21,19,21,22,23,24,25,23,25,26,23,26,27,28,29,30,31,32,28,28,30,31,33,34,35,33,35,36,37,38,39,39,40,41,37,39,41,42,43,44,45,46,47,48,49,50,51,52,48,48,50,51,53,54,55,53,55,5`
+`6,53,56,57,58,59,60,60,61,62,58,60,62,63,64,65,63,65,66,70,67,68,68,69,70,71,72,73,74,75,76,74,76,77,78,79,80,78,80,81,82,83,84,85,86,87,85,87,88,89,90,91,93,94,89,89,91,92,89,92,93,95,96,97,95,97,98,102,99,100,100,101,102,103,104,105,106,107,108,106,108,109],"trislength":56,"triTopoly":[0,0,1,1,2,2,3,3,4,5,5,6,6,6,7,7,7,8,8,9,9,9,10,11,12,12,12,13,13,13,14,14,14,15,15,16,16,17,18,18,19,19,20,21,21,22,22,22,22,23,23,24,24,25,26,26],"baseVert":[0,4,8,12,16,19,23,28,33,37,42,45,48,53,58,63,67,71,7`
+`4,78,82,85,89,95,99,103,106],"vertsCount":[4,4,4,4,3,4,5,5,4,5,3,3,5,5,5,4,4,3,4,4,3,4,6,4,4,3,4],"baseTri":[0,2,4,6,8,9,11,14,17,19,22,23,24,27,30,33,35,37,38,40,42,43,45,49,51,53,54],"triCount":[2,2,2,2,1,2,3,3,2,3,1,1,3,3,3,2,2,1,2,2,1,2,4,2,2,1,2]},"links":{"poly":[2,13,15,19,21,24,24,26],"cost":[4057.02001953125,1112.9925537109375,4800,5568],"type":[2,1,2,2],"pos":[-299.44000244140625,-119.08000183105469,68,-298,-109,119,-258,-93,-160,-238.34971618652344,-111.8642807006836,-160,-34,-29,68,-`
+`34,-37,124,-18,-165,124,6,-165,68],"length":4}}],["6_5",{"tileId":"6_5","tx":6,"ty":5,"mesh":{"verts":[22,-165,-416,30,-165,-416,22,139,-416,22,-165,-224,46,-165,-224,46,-61,-224,22,-45,-224,70,-109,68,46,-61,68,22,-61,68,22,-165,68,134,-165,68,94,-101,68,70,-109,68,22,-165,68,22,-5,-224,46,-5,-224,30,91,-224,86,171,-224,198,171,-224,214,203,-224,270,347,-224,22,347,-224,214,203,-224,270,203,-224,270,347,-224,30,91,-224,86,171,-224,22,347,-224,22,-5,-224,30,91,-224,22,347,-224,78,187,-416,198,18`
+`7,-416,206,211,-416,46,211,-416,78,187,-416,206,211,-416,262,347,-416,22,347,-416,22,347,-416,22,187,-416,46,211,-416,206,211,-416,262,211,-416,262,347,-416,30,347,68,22,275,68,22,243,68,302,-165,68,302,-45,68,278,-45,68,302,-165,68,278,-45,68,270,-13,68,70,19,68,190,-165,68,70,19,68,270,-13,68,294,-5,68,310,347,68,30,347,68,22,243,68,294,-5,68,310,-21,68,310,347,68,70,-69,68,86,-69,68,86,-45,68,54,11,68,38,-5,68,262,11,-416,294,19,-415,294,147,-415,86,155,-416,62,131,-416,62,-165,-416,262,-165,`
+`-416,262,11,-416,62,131,-416,262,147,-256,278,171,-256,238,155,-256,262,147,-256,238,155,-256,86,155,-256,62,131,-256,62,-165,-256,270,-165,-256,222,51,148,150,43,148,150,-13,148,278,-13,148,278,347,148,222,347,148,222,51,148,278,-13,148,326,219,-416,302,195,-416,310,155,-416,422,195,-416,406,267,-416,406,267,-416,326,267,-416,326,219,-416,462,-77,-416,502,-165,-416,534,-165,-416,310,155,-416,294,147,-415,294,19,-415,422,-37,-416,534,203,-415,422,195,-416,310,155,-416,462,-77,-416,534,-165,-416,`
+`310,155,-416,422,-37,-416,462,-77,-416,534,-165,-72,534,35,-72,310,35,-72,302,-165,-72,310,123,-72,310,35,-72,534,35,-72,534,123,-72,302,347,-72,302,299,-72,334,267,-72,534,347,-72,334,235,-72,302,195,-72,310,123,-72,534,123,-72,534,347,-72,334,267,-72,334,235,-72,534,123,-72,302,347,-419,302,299,-416,326,267,-416,406,267,-416,414,331,-416,414,331,-416,502,323,-416,502,347,-419,302,347,-419,414,-141,-272,430,-157,-272,462,-149,-272,470,-117,-272,398,-37,-272,374,-93,-272,358,-93,-276,374,-29,-28`
+`8,326,-13,-288,318,-165,-272,358,-93,-276,318,-165,-272,406,-165,-272,414,-141,-272,374,-93,-272,334,-37,-414,334,-69,-414,358,-69,-414,358,-37,-414,390,-165,-415,398,-165,-415,398,-141,-415,534,-165,-640,534,-101,-640,470,-101,-636,398,-165,-640,470,-101,-636,470,-13,-636,406,-5,-636,398,-165,-640,406,-5,-636,470,-13,-636,478,3,-640,422,107,-640,398,83,-640,478,3,-640,534,3,-640,534,115,-640,422,107,-640,406,-21,-509,406,-77,-509,470,-77,-509,470,-21,-509,422,315,-340,422,243,-340,478,243,-340,`
+`478,315,-340,430,-133,-415,454,-133,-415,454,-117,-415,430,227,-384,430,211,-384,478,211,-384,478,227,-384,430,299,-410,430,251,-410,478,251,-410,478,299,-410,462,-165,-288,486,-165,-288,486,-149,-288,494,219,-320,534,219,-320,534,307,-320,494,307,-320,510,227,-414,534,227,-414,534,291,-414,510,291,-414,534,307,-644,534,347,-644,526,347,-644],"vertslength":221,"polys":[0,2,3,6,7,10,11,14,15,17,18,22,23,25,26,28,29,31,32,34,35,39,40,42,43,45,46,48,49,51,52,56,57,62,63,65,66,70,71,75,76,79,80,82,8`
+`3,88,89,92,93,96,97,101,102,104,105,107,108,111,112,116,117,119,120,123,124,127,128,131,132,135,136,139,140,144,145,148,149,154,155,158,159,163,164,167,168,170,171,174,175,178,179,183,184,187,188,191,192,195,196,198,199,202,203,206,207,209,210,213,214,217,218,220],"polyslength":56,"regions":[21,22,15,15,8,8,8,8,8,9,9,9,9,1,1,1,1,1,23,6,6,7,7,14,14,2,2,2,2,2,2,3,4,5,5,5,11,11,12,12,12,25,26,13,13,10,10,17,18,28,29,19,30,20,32,34],"neighbors":[[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,3]],[[`
+`0],[0],[1,2],[0]],[[0],[0],[1,8]],[[0],[0],[1,6],[0],[1,7]],[[0],[0],[1,5]],[[0],[1,5],[1,8]],[[1,4],[1,7],[0]],[[0],[0],[1,10]],[[0],[1,9],[1,12],[0],[1,11]],[[0],[0],[1,10]],[[0],[0],[1,10]],[[0],[0],[1,16]],[[0],[0],[1,15]],[[1,14],[0],[1,16],[0],[0]],[[1,15],[0],[1,17],[0],[1,13],[0]],[[0],[0],[1,16]],[[0],[0],[0],[0],[0]],[[0],[1,28],[0],[0],[1,20]],[[0],[0],[1,19],[0]],[[0],[0],[1,22]],[[1,21],[0],[0],[0],[0],[0]],[[0],[0],[0],[1,24]],[[0],[0],[1,23],[0]],[[0],[0],[1,29],[0],[1,26]],[[1,36`
+`],[0],[1,25]],[[0],[0],[1,29]],[[0],[1,19],[0],[1,30]],[[0],[1,25],[1,30],[1,27],[0]],[[1,28],[0],[1,29]],[[0],[1,32],[0],[0]],[[0],[1,31],[0],[1,34]],[[0],[0],[1,35],[0]],[[0],[0],[1,32],[1,35]],[[1,33],[0],[1,34],[0]],[[0],[0],[1,26],[0],[1,37]],[[0],[0],[0],[1,36]],[[0],[0],[0],[0],[0],[1,40]],[[0],[0],[0],[1,40]],[[1,39],[0],[0],[1,38],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,44],[0]],[[0],[1,45],[0],[1,43]],[[1,44],[0],[1,46],[0],[0]],[[0],[0],[0],[1,45]],[[0],[0],[0],[0]],[[0],[0],`
+`[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[22,-165,-416,30,-165,-416,22,139,-416,22,-165,-224,46,-165,-224,46,-61,-224,22,-45,-224,70,-109,68,46,-61,68,22,-61,68,22,-165,68,134,-165,68,94,-101,68,70,-109,68,22,-165,68,22,-5,-224,46,-5,-224,30,91,-224,86,171,-224,198,171,-224,214,203,-224,270,347,-224,22,347,-224,106,255,-216,214,203,-224,270,203,-224,270,347,-224,30,91,-224,86,171,-224,22,347,-22`
+`4,22,-5,-224,30,91,-224,22,347,-224,78,187,-416,198,187,-416,206,211,-416,46,211,-416,78,187,-416,206,211,-416,262,347,-416,22,347,-416,22,347,-416,22,187,-416,46,211,-416,206,211,-416,262,211,-416,262,347,-416,30,347,68,22,275,68,22,243,68,302,-165,68,302,-45,68,278,-45,68,302,-165,68,278,-45,68,270,-13,68,70,19,68,190,-165,68,70,19,68,270,-13,68,294,-5,68,310,347,68,30,347,68,22,243,68,294,-5,68,310,-21,68,310,347,68,70,-69,68,86,-69,68,86,-45,68,54,11,68,38,-5,68,262,11,-415,294,19,-415,294,1`
+`47,-415,86,155,-416,62,131,-416,62,-165,-416,262,-165,-416,262,11,-415,62,131,-416,262,147,-256,278,171,-256,238,155,-256,262,147,-256,238,155,-256,86,155,-256,62,131,-256,62,-165,-256,108.22222137451172,-165,-248,154.44444274902344,-165,-256,270,-165,-256,146,-129,-248,146,87,-248,222,51,148,150,43,148,150,-13,148,278,-13,148,278,347,148,222,347,148,222,51,148,278,-13,148,326,219,-416,302,195,-416,310,155,-416,422,195,-416,406,267,-416,406,267,-416,326,267,-416,326,219,-416,462,-77,-416,502,-16`
+`5,-416,534,-165,-416,310,155,-416,294,147,-416,294,19,-416,422,-37,-416,534,203,-415,422,195,-416,310,155,-416,462,-77,-416,534,-165,-416,310,155,-416,422,-37,-416,462,-77,-416,534,-165,-72,534,35,-72,310,35,-72,302,-165,-72,310,123,-72,310,35,-72,534,35,-72,534,123,-72,302,347,-72,302,299,-72,334,267,-72,534,347,-72,334,235,-72,302,195,-72,310,123,-72,534,123,-72,534,347,-72,334,267,-72,334,235,-72,534,123,-72,302,347,-419,302,299,-416,326,267,-416,406,267,-416,414,331,-419,414,331,-419,502,323`
+`,-418,502,347,-419,302,347,-419,414,-141,-272,430,-157,-272,462,-149,-272,470,-117,-272,398,-37,-272,374,-93,-272,358,-93,-272,363.3333435058594,-71.66666412353516,-288,374,-29,-288,326,-13,-288,321.4285583496094,-99.85713958740234,-288,320.28570556640625,-121.57142639160156,-272,318,-165,-272,330,-105,-276,358,-93,-272,318,-165,-272,406,-165,-272,414,-141,-272,374,-93,-272,334,-37,-414,334,-69,-414,358,-69,-414,358,-37,-414,390,-165,-415,398,-165,-415,398,-141,-415,534,-165,-640,534,-101,-640,4`
+`70,-101,-636,398,-165,-640,470,-101,-636,470,-13,-636,406,-5,-636,398,-165,-640,406,-5,-640,470,-13,-640,478,3,-640,422,107,-640,398,83,-640,478,3,-640,534,3,-640,534,115,-640,422,107,-640,406,-21,-509,406,-77,-509,470,-77,-509,470,-21,-509,422,315,-340,422,243,-340,478,243,-340,478,315,-340,430,-133,-415,454,-133,-415,454,-117,-415,430,227,-384,430,211,-384,478,211,-384,478,227,-384,430,299,-410,430,251,-410,478,251,-410,478,299,-410,462,-165,-288,486,-165,-288,486,-149,-288,494,219,-320,534,21`
+`9,-320,534,307,-320,494,307,-320,510,227,-414,534,227,-414,534,291,-414,510,291,-414,534,307,-644,534,347,-644,526,347,-644],"vertslength":230,"tris":[0,1,2,3,4,5,3,5,6,7,8,9,7,9,10,11,12,13,11,13,14,15,16,17,21,22,23,22,18,23,21,20,23,18,19,23,20,19,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,40,36,38,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,57,53,55,55,56,57,58,59,60,62,63,58,58,60,61,58,61,62,64,65,66,67,68,69,69,70,71,67,69,71,72,73,74,74,75,76,72,74,76,77,78,79,77,79,80,81,`
+`82,83,90,89,92,89,88,92,90,91,92,87,86,93,84,85,93,86,85,93,87,88,93,92,88,93,92,91,93,84,91,93,94,95,96,94,96,97,98,99,100,98,100,101,102,103,104,105,106,102,102,104,105,107,108,109,110,111,112,113,114,115,113,115,116,117,118,119,120,121,117,117,119,120,122,123,124,125,126,127,125,127,128,132,129,130,130,131,132,133,134,135,133,135,136,137,138,139,137,139,140,141,142,143,141,143,144,145,146,147,147,148,149,145,147,149,150,151,152,150,152,153,154,155,156,154,156,157,159,154,157,157,158,159,161,1`
+`62,163,160,161,163,160,163,164,166,160,167,160,164,167,164,165,167,166,165,167,171,172,168,170,171,168,168,169,170,176,173,174,174,175,176,177,178,179,180,181,182,180,182,183,184,185,186,184,186,187,188,189,190,191,192,188,188,190,191,193,194,195,193,195,196,200,197,198,198,199,200,204,201,202,202,203,204,205,206,207,211,208,209,209,210,211,215,212,213,213,214,215,216,217,218,222,219,220,220,221,222,226,223,224,224,225,226,227,228,229],"trislength":122,"triTopoly":[0,1,1,2,2,3,3,4,5,5,5,5,5,6,7,`
+`8,9,10,10,10,11,12,13,14,15,15,15,16,16,16,16,17,18,18,18,19,19,19,20,20,21,22,22,22,22,22,22,22,22,22,22,23,23,24,24,25,25,25,26,27,28,28,29,29,29,30,31,31,32,32,33,33,34,34,35,35,36,36,36,37,37,38,38,38,38,39,39,39,39,39,39,39,40,40,40,41,41,42,43,43,44,44,45,45,45,46,46,47,47,48,48,49,50,50,51,51,52,53,53,54,54,55],"baseVert":[0,3,7,11,15,18,24,27,30,33,36,41,44,47,50,53,58,64,67,72,77,81,84,94,98,102,107,110,113,117,122,125,129,133,137,141,145,150,154,160,168,173,177,180,184,188,193,197,201,`
+`205,208,212,216,219,223,227],"vertsCount":[3,4,4,4,3,6,3,3,3,3,5,3,3,3,3,5,6,3,5,5,4,3,10,4,4,5,3,3,4,5,3,4,4,4,4,4,5,4,6,8,5,4,3,4,4,5,4,4,4,3,4,4,3,4,4,3],"baseTri":[0,1,3,5,7,8,13,14,15,16,17,20,21,22,23,24,27,31,32,35,38,40,41,51,53,55,58,59,60,62,65,66,68,70,72,74,76,79,81,85,92,95,97,98,100,102,105,107,109,111,112,114,116,117,119,121],"triCount":[1,2,2,2,1,5,1,1,1,1,3,1,1,1,1,3,4,1,3,3,2,1,10,2,2,3,1,1,2,3,1,2,2,2,2,2,3,2,4,7,3,2,1,2,2,3,2,2,2,1,2,2,1,2,2,1]},"links":{"poly":[1,22,4,22,25,`
+`50,38,52,48,50,48,53],"cost":[1920,1920,1726.87060546875,649.8461303710938,3288,984],"type":[2,2,2,2,2,2],"pos":[46,-165,-224,62,-165,-256,46,-5,-224,62,-5,-256,418.98822021484375,208.55294799804688,-416,430,211,-384,462,-149,-272,469.3846130371094,-160.07691955566406,-288,430,243,-340,430,227,-384,478,243,-340,494,243,-320],"length":6}}],["7_5",{"tileId":"7_5","tx":7,"ty":5,"mesh":{"verts":[822,-165,-640,798,-109,-640,758,-109,-640,758,-109,-640,750,-93,-640,534,-101,-640,758,-109,-640,534,-101`
+`,-640,534,-165,-640,822,-165,-640,966,283,-416,982,291,-416,982,347,-419,790,347,-419,790,307,-416,974,-165,-416,974,-125,-416,902,-117,-416,534,-165,-416,902,-29,-416,974,-21,-416,982,19,-416,974,155,-416,966,283,-416,790,307,-416,758,291,-415,726,307,-415,606,307,-415,606,211,-415,1030,11,-416,1030,-165,-416,1046,-165,-416,1046,147,-416,790,307,-416,758,291,-415,606,211,-415,534,-165,-416,902,-117,-416,902,-29,-416,982,19,-416,1030,11,-416,1046,147,-416,974,155,-416,606,211,-415,534,203,-415,5`
+`34,-165,-416,958,267,-72,982,291,-72,982,347,-72,662,323,-72,702,291,-72,958,267,-72,702,291,-72,702,227,-72,966,211,-72,982,203,-72,966,211,-72,702,227,-72,670,203,-72,630,323,-72,662,323,-72,982,347,-72,534,347,-72,958,-141,-72,982,-141,-72,982,203,-72,534,-165,-72,966,-165,-72,958,-141,-72,590,291,-72,630,323,-72,534,347,-72,582,243,-72,590,291,-72,534,347,-72,582,243,-72,534,347,-72,534,-165,-72,614,203,-72,582,243,-72,534,-165,-72,670,203,-72,614,203,-72,534,-165,-72,958,-141,-72,982,203,-7`
+`2,806,3,-640,822,-21,-636,878,-21,-636,886,83,-640,862,107,-640,758,3,-640,806,3,-640,862,107,-640,758,115,-637,750,-13,-640,758,3,-640,758,115,-637,758,347,-564,534,347,-564,534,-5,-640,534,307,-320,534,219,-320,590,219,-320,590,307,-320,534,291,-414,534,227,-414,574,227,-414,574,291,-414,534,347,-644,534,307,-644,758,307,-644,766,347,-644,686,291,128,662,307,128,614,299,128,598,243,128,614,219,128,686,227,128,734,315,-370,750,307,-370,758,323,-370,822,-85,-636,798,-109,-640,822,-165,-640,878,-`
+`21,-636,822,-21,-636,822,-85,-636,822,-165,-640,878,-165,-640,822,-21,-509,822,-77,-509,878,-77,-509,878,-21,-509,918,-37,-307,918,-109,-307,966,-109,-307,966,-37,-307,926,-45,-414,926,-93,-414,974,-93,-414,974,-45,-414,990,-149,68,974,-165,68,1046,-165,68,1046,347,68,990,347,68,990,-149,68,1046,-165,68,1014,347,-256,1014,-165,-256,1046,-165,-248,1046,347,-256,1022,187,-416,1046,187,-416,1046,347,-416,1022,347,-416],"vertslength":157,"polys":[0,2,3,5,6,9,10,14,15,18,19,24,25,28,29,32,33,38,39,42`
+`,43,45,46,50,51,54,55,58,59,62,63,65,66,68,69,71,72,74,75,77,78,80,81,85,86,90,91,94,95,100,101,104,105,108,109,112,113,118,119,121,122,124,125,129,130,133,134,137,138,141,142,144,145,148,149,152,153,156],"polyslength":39,"regions":[6,6,6,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,3,3,3,8,12,13,4,16,5,5,9,11,14,10,10,15,20],"neighbors":[[[1,30],[0],[1,2]],[[0],[0],[1,2]],[[1,1],[0],[0],[1,0]],[[0],[0],[0],[0],[1,5]],[[0],[0],[1,8],[0]],[[0],[0],[1,9],[0],[1,3],[1,8]],[[0],[0],[0],[1,8]],[[0],[0],[0],`
+`[1,9]],[[0],[1,6],[1,10],[1,4],[0],[1,5]],[[0],[1,7],[0],[1,5]],[[0],[0],[1,8]],[[0],[0],[1,14],[0],[1,12]],[[1,11],[0],[1,13],[0]],[[0],[1,12],[0],[1,21]],[[0],[1,11],[0],[1,17]],[[0],[0],[1,21]],[[0],[0],[1,21]],[[0],[1,14],[1,18]],[[0],[1,17],[1,19]],[[1,18],[0],[1,20]],[[0],[1,19],[1,21]],[[0],[1,20],[1,16],[1,15],[1,13]],[[0],[1,31],[0],[0],[1,23]],[[0],[1,22],[0],[1,24]],[[0],[1,23],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0],[0]],[[0],[0],[0`
+`]],[[0],[1,0],[1,31]],[[1,22],[0],[1,30],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,36]],[[0],[0],[1,35],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[822,-165,-640,798,-109,-640,758,-109,-640,758,-109,-640,750,-93,-640,534,-101,-640,758,-109,-640,534,-101,-640,534,-165,-640,822,-165,-640,966,283,-416,982,291,-416,982,347,-419,790,347,-419,790,307,-416,974,-165,-416,974,-125,-416,902,-117,-416,534,-165,-416,902,-29,-416,974,-21,-416,982,19,-416,974`
+`,155,-416,966,283,-416,790,307,-416,758,291,-415,726,307,-415,606,307,-415,606,211,-415,1030,11,-416,1030,-165,-416,1046,-165,-416,1046,147,-416,790,307,-416,758,291,-415,606,211,-415,534,-165,-416,902,-117,-416,902,-29,-416,982,19,-416,1030,11,-416,1046,147,-416,974,155,-416,606,211,-415,534,203,-415,534,-165,-416,958,267,-72,982,291,-72,982,347,-72,662,323,-72,702,291,-72,958,267,-72,702,291,-72,702,227,-72,966,211,-72,982,203,-72,966,211,-72,702,227,-72,670,203,-72,630,323,-72,662,323,-72,982`
+`,347,-72,534,347,-72,958,-141,-72,982,-141,-72,982,203,-72,534,-165,-72,966,-165,-72,958,-141,-72,590,291,-72,630,323,-72,534,347,-72,582,243,-72,590,291,-72,534,347,-72,582,243,-72,534,347,-72,534,-165,-72,614,203,-72,582,243,-72,534,-165,-72,670,203,-72,614,203,-72,534,-165,-72,958,-141,-72,982,203,-72,806,3,-640,822,-21,-636,878,-21,-636,886,83,-640,862,107,-640,758,3,-640,806,3,-640,862,107,-640,758,115,-634,770,111,-640,750,-13,-640,758,3,-640,758,115,-634,758,347,-564,534,347,-564,534,112.`
+`33333587646484,-637,534,-5,-640,534,307,-320,534,219,-320,590,219,-320,590,307,-320,534,291,-414,534,227,-414,574,227,-414,574,291,-414,534,347,-644,534,307,-644,758,307,-644,766,347,-644,686,291,128,662,307,128,614,299,128,598,243,128,614,219,128,686,227,128,734,315,-370,750,307,-370,758,323,-370,822,-85,-636,798,-109,-636,822,-165,-640,878,-21,-636,822,-21,-636,822,-85,-636,822,-165,-640,878,-165,-640,822,-21,-509,822,-77,-509,878,-77,-509,878,-21,-509,918,-37,-307,918,-109,-307,966,-109,-307,`
+`966,-37,-307,926,-45,-414,926,-93,-414,974,-93,-414,974,-45,-414,990,-149,68,974,-165,68,1046,-165,68,1046,347,68,990,347,68,990,-149,68,1046,-165,68,1014,347,-256,1014,-165,-256,1046,-165,-248,1046,-141.72727966308594,-256,1046,-95.18181610107422,-248,1046,-48.6363639831543,-256,1046,-2.090909004211426,-248,1046,44.45454406738281,-256,1046,91,-248,1046,137.5454559326172,-256,1046,184.09091186523438,-256,1046,207.36363220214844,-248,1046,230.63636779785156,-256,1046,277.18182373046875,-256,1046,`
+`300.4545593261719,-248,1046,347,-256,1022,187,-416,1046,187,-416,1046,347,-416,1022,347,-416],"vertslength":171,"tris":[0,1,2,3,4,5,6,7,8,6,8,9,10,11,12,13,14,10,10,12,13,15,16,17,15,17,18,19,20,21,19,21,22,22,23,24,19,22,24,25,26,27,25,27,28,29,30,31,29,31,32,33,34,35,36,37,38,38,33,35,35,36,38,39,40,41,39,41,42,43,44,45,46,47,48,49,50,46,46,48,49,53,54,51,51,52,53,55,56,57,55,57,58,62,59,60,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,81,81,83,84,86,87,88,89,90`
+`,86,86,88,89,92,93,95,93,94,95,94,91,95,92,91,95,96,97,98,101,102,96,101,96,98,98,99,100,98,100,101,106,103,104,104,105,106,110,107,108,108,109,110,111,112,113,111,113,114,115,116,117,117,118,119,120,115,117,117,119,120,121,122,123,124,125,126,127,128,129,129,130,131,127,129,131,135,132,133,133,134,135,139,136,137,137,138,139,143,140,141,141,142,143,144,145,146,147,148,149,147,149,150,152,153,154,165,166,151,164,165,151,152,154,155,152,155,156,163,164,151,162,163,151,161,162,151,152,156,157,160,`
+`161,151,152,157,158,152,158,159,159,160,151,151,152,159,170,167,168,168,169,170],"trislength":94,"triTopoly":[0,1,2,2,3,3,3,4,4,5,5,5,5,6,6,7,7,8,8,8,8,9,9,10,11,11,11,12,12,13,13,14,14,15,16,17,18,19,20,21,21,21,22,22,22,23,23,23,23,24,24,24,24,24,25,25,26,26,27,27,28,28,28,28,29,30,31,31,31,32,32,33,33,34,34,35,36,36,37,37,37,37,37,37,37,37,37,37,37,37,37,37,38,38],"baseVert":[0,3,6,10,15,19,25,29,33,39,43,46,51,55,59,63,66,69,72,75,78,81,86,91,96,103,107,111,115,121,124,127,132,136,140,144,14`
+`7,151,167],"vertsCount":[3,3,4,5,4,6,4,4,6,4,3,5,4,4,4,3,3,3,3,3,3,5,5,5,7,4,4,4,6,3,3,5,4,4,4,3,4,16,4],"baseTri":[0,1,2,4,7,9,13,15,17,21,23,24,27,29,31,33,34,35,36,37,38,39,42,45,49,54,56,58,60,64,65,66,69,71,73,75,76,78,92],"triCount":[1,1,2,3,2,4,2,2,4,2,1,3,2,2,2,1,1,1,1,1,1,3,3,4,5,2,2,2,4,1,1,3,2,2,2,1,2,14,2]},"links":{"poly":[3,29],"cost":[4877.759765625],"type":[2],"pos":[790,323,-417.20001220703125,758,323,-370],"length":1}}],["8_5",{"tileId":"8_5","tx":8,"ty":5,"mesh":{"verts":[1222`
+`,131,-416,1206,147,-416,1046,147,-416,1046,-165,-416,1222,-165,-416,1238,-61,-256,1254,-53,-256,1238,51,-256,1238,-61,-256,1238,51,-256,1222,91,-256,1046,99,-248,1046,-165,-248,1238,-165,-256,1214,27,68,1158,27,68,1126,11,68,1158,-93,68,1126,11,68,1118,35,68,1046,51,68,1046,-165,68,1094,-165,68,1158,-93,68,1150,43,68,1158,27,68,1214,27,68,1254,131,68,1046,51,68,1118,35,68,1150,43,68,1046,347,68,1046,299,68,1134,307,68,1254,347,68,1118,347,160,1078,347,160,1078,235,88,1126,219,80,1126,219,80,1078`
+`,235,88,1046,219,68,1254,347,68,1134,307,68,1142,219,68,1254,131,68,1126,219,80,1046,219,68,1046,51,68,1150,43,68,1254,131,68,1142,219,68,1254,171,-256,1262,187,-256,1246,187,-256,1230,171,-256,1214,155,-256,1222,91,-256,1238,107,-256,1230,139,-256,1214,155,-256,1046,187,-256,1046,99,-248,1214,155,-256,1230,171,-256,1046,187,-256,1246,259,-416,1238,299,-416,1222,307,-416,1222,251,-416,1214,187,-416,1222,251,-416,1222,307,-416,1222,347,-416,1046,347,-416,1046,187,-416,1222,187,-256,1262,187,-256,`
+`1254,331,-256,1046,347,-256,1046,187,-256,1046,187,-256,1230,171,-256,1222,187,-256,1254,331,-256,1334,347,-256,1046,347,-256,1246,-165,68,1246,-109,68,1182,-109,68,1150,-165,68,1262,187,-256,1254,171,-256,1262,155,-256,1470,163,-256,1462,187,-256,1262,155,-256,1230,139,-256,1238,107,-256,1558,203,-256,1502,187,-256,1494,163,-256,1558,-53,-256,1262,155,-256,1238,107,-256,1238,51,-256,1262,155,-256,1238,51,-256,1254,-53,-256,1558,-53,-256,1494,163,-256,1470,163,-256,1254,307,-416,1238,299,-416,12`
+`46,259,-416,1254,307,-416,1246,259,-416,1254,179,-416,1478,347,-416,1254,347,-416,1254,307,-416,1254,179,-416,1478,179,-416,1270,155,-416,1254,139,-416,1254,-45,-416,1558,-53,-416,1518,163,-416,1558,347,-416,1510,347,-416,1518,163,-416,1558,-53,-416,1262,-117,148,1262,-165,148,1454,-165,148,1454,-117,148,1278,-125,68,1278,-165,68,1446,-165,68,1446,-125,68,1558,-165,68,1558,-109,68,1502,-101,68,1470,-165,68,1502,-101,68,1502,99,68,1470,107,68,1470,-165,68,1470,203,-256,1502,187,-256,1558,203,-256`
+`,1558,347,-256,1462,347,-256,1470,107,68,1502,99,68,1510,115,68,1470,347,68,1510,115,68,1558,115,68,1558,347,68,1470,347,68,1518,-93,148,1558,-93,148,1558,99,148,1518,99,148,1526,-85,68,1558,-85,68,1558,91,68,1526,91,68],"vertslength":169,"polys":[0,4,5,7,8,13,14,17,18,23,24,27,28,30,31,34,35,38,39,41,42,45,46,51,52,56,57,62,63,65,66,69,70,75,76,80,81,83,84,86,87,90,91,95,96,98,99,102,103,105,106,111,112,114,115,117,118,122,123,127,128,131,132,135,136,139,140,143,144,147,148,152,153,156,157,160,`
+`161,164,165,168],"polyslength":40,"regions":[7,2,2,10,10,6,6,6,6,6,6,6,5,5,5,9,9,3,3,3,14,1,1,1,1,1,8,8,8,4,4,15,16,13,13,12,11,11,17,18],"neighbors":[[[0],[0],[0],[0],[0]],[[0],[1,25],[1,2]],[[1,1],[0],[1,13],[0],[0],[0]],[[1,5],[0],[1,4],[0]],[[0],[1,6],[0],[0],[0],[1,3]],[[0],[1,3],[0],[1,11]],[[1,4],[0],[1,11]],[[0],[0],[1,10],[0]],[[0],[0],[1,9],[0]],[[1,8],[0],[1,11]],[[1,7],[0],[1,11],[0]],[[1,9],[0],[1,6],[1,5],[1,10],[0]],[[1,21],[0],[0],[1,14],[0]],[[0],[1,22],[0],[1,14],[0],[1,2]],[[1`
+`,12],[1,18],[1,13]],[[1,26],[0],[1,16],[0]],[[0],[1,15],[0],[0],[0],[0]],[[0],[0],[1,19],[0],[1,18]],[[1,14],[0],[1,17]],[[0],[0],[1,17]],[[0],[0],[0],[0]],[[1,12],[0],[1,25],[0],[0]],[[0],[1,13],[1,24]],[[1,35],[0],[1,25],[0]],[[1,22],[0],[1,25]],[[1,24],[1,1],[0],[1,23],[0],[1,21]],[[0],[1,15],[1,27]],[[1,26],[0],[1,28]],[[0],[0],[1,27],[0],[0]],[[0],[0],[0],[1,30],[0]],[[0],[0],[1,29],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,34],[0]],[[0],[1,36],[0],[1,33]],[[0],[1,23],[0],[0],[0]`
+`],[[1,34],[0],[1,37],[0]],[[0],[0],[0],[1,36]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1222,131,-416,1206,147,-416,1046,147,-416,1046,-165,-416,1222,-165,-416,1238,-61,-256,1254,-53,-256,1238,51,-256,1238,-61,-256,1238,51,-256,1222,91,-256,1134,95,-256,1112,96,-248,1090,97,-256,1046,99,-256,1046,55,-256,1046,33,-248,1046,-11,-256,1046,-55,-256,1046,-77,-248,1046,-143,-256,1046,-165,-248,1195.3333740234375,-165,-248,1238,-165,-256,1082,15,-248,1178,-153,-256,1058,-57,-256,1214,27`
+`,68,1158,27,68,1126,11,68,1158,-93,68,1126,11,68,1118,35,68,1046,51,68,1046,-165,68,1094,-165,68,1158,-93,68,1150,43,68,1158,27,68,1214,27,68,1254,131,68,1046,51,68,1118,35,68,1150,43,68,1046,347,68,1046,299,68,1134,307,68,1254,347,68,1118,347,68,1078,347,68,1078,324.6000061035156,68,1078,302.20001220703125,136,1078,279.79998779296875,120,1078,235,96,1126,219,68,1124.6666259765625,240.3333282470703,96,1122,283,128,1120.6666259765625,304.3333435058594,68,1114,303,136,1114,327,68,1126,219,68,1078,`
+`235,96,1046,219,80,1106,219,80,1254,347,68,1134,307,68,1142,219,68,1254,131,68,1126,219,68,1106,219,80,1046,219,80,1046,177,68,1046,51,68,1150,43,68,1254,131,68,1142,219,68,1254,171,-256,1262,187,-256,1246,187,-256,1230,171,-256,1214,155,-256,1222,91,-256,1238,107,-256,1230,139,-256,1214,155,-256,1046,187,-256,1046,99,-248,1178,93,-248,1106,103,-256,1214,155,-256,1230,171,-256,1046,187,-256,1246,259,-416,1238,299,-416,1222,307,-416,1222,251,-416,1214,187,-416,1222,251,-416,1222,307,-416,1222,347`
+`,-416,1046,347,-416,1046,187,-416,1222,187,-256,1262,187,-256,1254,331,-256,1046,347,-256,1046,301.28570556640625,-248,1046,278.4285583496094,-256,1046,187,-248,1090,187,-256,1112,187,-248,1134,187,-256,1178,187,-248,1106,247,-256,1130,199,-248,1202,223,-256,1046,187,-248,1069,185,-256,1230,171,-256,1222,187,-256,1178,187,-248,1156,187,-256,1254,331,-256,1334,347,-256,1046,347,-256,1246,-165,68,1246,-109,68,1182,-109,68,1150,-165,68,1262,187,-256,1254,171,-256,1262,155,-256,1470,163,-256,1462,18`
+`7,-256,1262,155,-256,1230,139,-256,1238,107,-256,1558,203,-256,1502,187,-256,1494,163,-256,1538.800048828125,11.800000190734863,-256,1545.199951171875,-9.800000190734863,-248,1558,-53,-256,1558,-29.727272033691406,-248,1558,16.81818199157715,-256,1558,40.09090805053711,-248,1558,86.63636016845703,-256,1558,109.90908813476562,-248,1558,133.18182373046875,-256,1554,55,-248,1554,127,-248,1262,155,-256,1238,107,-256,1238,51,-256,1262,155,-256,1238,51,-256,1254,-53,-256,1558,-53,-256,1551.59997558593`
+`75,-31.399999618530273,-248,1538.800048828125,11.800000190734863,-256,1494,163,-256,1470,163,-256,1274,-17,-248,1274,127,-248,1254,307,-416,1238,299,-416,1246,259,-416,1254,307,-416,1246,259,-416,1254,179,-416,1478,347,-416,1254,347,-416,1254,307,-416,1254,179,-416,1478,179,-416,1270,155,-416,1254,139,-416,1254,-45,-416,1558,-53,-416,1518,163,-416,1558,347,-416,1510,347,-416,1518,163,-416,1558,-53,-416,1262,-117,148,1262,-165,148,1454,-165,148,1454,-117,148,1278,-125,68,1278,-165,68,1446,-165,68`
+`,1446,-125,68,1558,-165,68,1558,-109,68,1502,-101,68,1470,-165,68,1502,-101,68,1502,99,68,1470,107,68,1470,-165,68,1470,203,-256,1502,187,-256,1558,203,-256,1558,347,-256,1462,347,-256,1470,107,68,1502,99,68,1510,115,68,1470,347,68,1510,115,68,1558,115,68,1558,347,68,1470,347,68,1518,-93,148,1558,-93,148,1558,99,148,1518,99,148,1526,-85,68,1558,-85,68,1558,91,68,1526,91,68],"vertslength":221,"tris":[0,1,2,3,4,0,0,2,3,5,6,7,13,14,15,9,10,11,17,16,24,11,12,24,12,13,24,16,15,24,13,15,24,11,9,24,8,9`
+`,24,19,20,25,22,21,25,20,21,25,22,23,25,8,23,25,8,24,26,19,25,26,8,25,26,19,18,26,24,17,26,18,17,26,27,28,29,27,29,30,31,32,33,34,35,36,36,31,33,33,34,36,37,38,39,37,39,40,41,42,43,44,45,46,44,46,47,53,54,55,52,53,55,52,55,56,51,50,58,51,52,58,56,52,58,56,57,58,50,58,59,48,49,59,50,49,59,58,57,59,48,57,59,63,60,61,61,62,63,64,65,66,64,66,67,69,70,71,68,69,71,75,68,71,71,72,73,75,71,73,73,74,75,76,77,78,76,78,79,76,79,80,81,82,83,81,83,84,87,81,84,86,87,88,87,84,88,84,85,88,86,85,88,89,90,91,92,9`
+`3,94,92,94,95,96,97,98,96,98,99,100,101,96,96,99,100,106,107,113,110,109,113,107,108,113,109,108,113,106,105,113,105,104,113,111,112,114,113,110,114,111,110,114,112,102,115,102,103,115,104,103,115,104,113,115,112,114,115,113,114,115,118,119,120,118,120,121,121,116,117,117,118,121,122,123,124,125,126,127,125,127,128,129,130,131,133,129,131,131,132,133,134,135,136,141,142,143,140,141,143,140,143,144,140,144,145,137,138,139,140,146,147,148,137,139,139,140,147,140,145,149,145,146,149,146,140,149,147`
+`,148,150,148,139,150,139,147,150,151,152,153,159,160,161,156,155,162,156,157,162,157,158,162,161,159,162,158,159,162,161,154,163,162,161,163,154,155,163,162,155,163,164,165,166,167,168,169,170,171,172,172,173,174,170,172,174,175,176,177,179,175,177,177,178,179,180,181,182,180,182,183,187,184,185,185,186,187,191,188,189,189,190,191,192,193,194,192,194,195,196,197,198,196,198,199,200,201,202,200,202,203,200,203,204,205,206,207,205,207,208,209,210,211,209,211,212,216,213,214,214,215,216,220,217,218`
+`,218,219,220],"trislength":154,"triTopoly":[0,0,0,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,4,4,4,4,5,5,6,7,7,8,8,8,8,8,8,8,8,8,8,8,8,9,9,10,10,11,11,11,11,11,11,12,12,12,13,13,13,13,13,13,13,14,15,15,16,16,16,16,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,18,18,18,18,19,20,20,21,21,21,22,23,23,23,23,23,23,23,23,23,23,23,23,23,23,24,25,25,25,25,25,25,25,25,25,25,26,27,28,28,28,29,29,29,30,30,31,31,32,32,33,33,34,34,35,35,35,36,36,37,37,38,38,39,39],"baseVert":[0,5,8,27,31,37,41,44,48,60,64,`
+`68,76,81,89,92,96,102,116,122,125,129,134,137,151,154,164,167,170,175,180,184,188,192,196,200,205,209,213,217],"vertsCount":[5,3,19,4,6,4,3,4,12,4,4,8,5,8,3,4,6,14,6,3,4,5,3,14,3,10,3,3,5,5,4,4,4,4,4,5,4,4,4,4],"baseTri":[0,3,4,24,26,30,32,33,35,47,49,51,57,60,67,68,70,74,89,93,94,96,99,100,114,115,125,126,127,130,133,135,137,139,141,143,146,148,150,152],"triCount":[3,1,20,2,4,2,1,2,12,2,2,6,3,7,1,2,4,15,4,1,2,3,1,14,1,10,1,1,3,3,2,2,2,2,2,3,2,2,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos"`
+`:[],"length":0}}],["9_5",{"tileId":"9_5","tx":9,"ty":5,"mesh":{"verts":[1622,-101,68,1558,-109,68,1558,-165,68,1646,-165,68,1646,99,68,1622,99,68,1622,-101,68,1646,-165,68,1558,99,148,1558,-93,148,1606,-93,148,1606,99,148,1558,91,68,1558,-85,68,1598,-85,68,1598,91,68,1558,347,-256,1558,-53,-256,1646,-53,-256,1646,347,-256,1638,155,-416,1622,171,-416,1558,179,-416,1558,-45,-416,1638,-45,-416,1614,115,68,1622,99,68,1646,99,68,1646,347,68,1558,347,68,1558,115,68,1614,115,68,1646,347,68,1558,179,-41`
+`6,1622,171,-416,1638,187,-416,1638,347,-416,1558,347,-416,1758,-109,-408,1750,-85,-408,1718,-77,-412,1758,-109,-408,1718,-77,-412,1678,-61,-412,1678,-165,-412,1870,-165,-412,1846,-109,-408,1758,-109,-408,1678,-165,-412,1678,-61,-412,1718,-77,-412,1750,-61,-408,1718,51,-412,1678,67,-412,1750,-61,-408,1750,43,-408,1718,51,-412,1678,67,-412,1718,51,-412,1758,91,-408,1678,347,-412,1854,91,-408,1886,51,-412,2070,67,-408,2070,347,-408,1678,347,-412,1678,347,-412,1758,91,-408,1854,91,-408,1726,51,-248,`
+`1726,-69,-248,1750,-69,-248,1750,51,-248,1734,43,-356,1734,-61,-356,1750,-61,-356,1750,43,-356,1790,-45,-232,1766,-29,-232,1766,-93,-232,1814,-21,-232,1790,-45,-232,1766,-93,-232,1846,-93,-232,1846,75,-218,1814,75,-232,1814,-21,-232,1846,-93,-232,1822,67,-408,1798,67,-408,1798,43,-408,1806,19,-408,1838,51,-408,1798,43,-408,1782,51,-408,1806,19,-408,1806,-21,-408,1774,-29,-408,1774,-85,-408,1838,-85,-408,1838,51,-408,1806,19,-408,1806,-21,-408,1838,-85,-408,1886,-77,-412,1846,-109,-408,1870,-165,`
+`-412,2070,-165,-412,1886,51,-412,1862,43,-408,1862,-61,-408,1886,-77,-412,2070,-165,-412,2070,67,-408,1862,-69,-248,1886,-69,-248,1886,-45,-248,1886,51,-248,1862,51,-248,1862,-21,-248,1886,-29,-248],"vertslength":121,"polys":[0,3,4,7,8,11,12,15,16,19,20,24,25,28,29,32,33,37,38,40,41,44,45,48,49,53,54,56,57,60,61,65,66,68,69,72,73,76,77,79,80,83,84,87,88,92,93,95,96,99,100,103,104,107,108,113,114,116,117,120],"polyslength":30,"regions":[10,10,12,13,3,5,4,4,6,7,7,7,8,8,1,1,1,18,19,17,17,17,16,16,1`
+`6,16,2,2,25,26],"neighbors":[[[0],[0],[0],[1,1]],[[1,6],[0],[1,0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,8],[0],[0],[0]],[[0],[1,1],[0],[1,7]],[[0],[0],[1,6],[0]],[[1,5],[0],[0],[0],[0]],[[0],[0],[1,10]],[[1,9],[1,12],[0],[1,11]],[[1,26],[0],[1,10],[0]],[[1,10],[0],[1,13],[1,14],[0]],[[0],[0],[1,12]],[[1,12],[0],[1,16],[0]],[[0],[1,27],[0],[0],[1,16]],[[1,14],[0],[1,15]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,20]],[[0],[1,19],[0],[1,21]],[[0],[0],[1,20],[0]],`
+`[[0],[0],[1,23],[1,25],[0]],[[0],[0],[1,22]],[[0],[0],[0],[1,25]],[[1,22],[0],[1,24],[0]],[[0],[1,11],[0],[1,27]],[[0],[0],[0],[1,26],[0],[1,15]],[[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1622,-101,68,1558,-109,68,1558,-165,68,1646,-165,68,1646,99,68,1622,99,68,1622,-101,68,1646,-165,68,1558,99,148,1558,-93,148,1606,-93,148,1606,99,148,1558,91,68,1558,-85,68,1598,-85,68,1598,91,68,1558,347,-256,1558,135.23529052734375,-256,1558,111.70587921142578,-248,1558,88.17646789550781,-256,1558,`
+`64.64705657958984,-248,1558,17.58823585510254,-256,1558,-29.47058868408203,-248,1558,-53,-256,1646,-53,-256,1646,347,-256,1570,127,-248,1638,155,-416,1622,171,-416,1558,179,-416,1558,-45,-416,1638,-45,-416,1614,115,68,1622,99,68,1646,99,68,1646,347,68,1558,347,68,1558,115,68,1614,115,68,1646,347,68,1558,179,-416,1622,171,-416,1638,187,-416,1638,347,-416,1558,347,-416,1758,-109,-408,1750,-85,-408,1718,-77,-412,1758,-109,-408,1718,-77,-412,1678,-61,-412,1678,-165,-412,1870,-165,-412,1846,-109,-408`
+`,1758,-109,-408,1678,-165,-412,1678,-61,-412,1718,-77,-408,1750,-61,-408,1718,51,-413,1678,67,-412,1750,-61,-408,1750,43,-408,1718,51,-413,1678,67,-412,1718,51,-408,1758,91,-408,1678,347,-412,1854,91,-408,1886,51,-412,2070,67,-408,2070,347,-408,1678,347,-412,1678,347,-412,1758,91,-408,1854,91,-408,1726,51,-248,1726,-69,-248,1750,-69,-248,1750,51,-248,1734,43,-356,1734,-61,-356,1750,-61,-356,1750,43,-356,1790,-45,-232,1766,-29,-232,1766,-93,-232,1814,-21,-232,1790,-45,-232,1766,-93,-232,1846,-93,`
+`-232,1822,-39,-226,1846,75,-218,1830,75,-218,1814,75,-232,1814,-21,-232,1822,-39,-226,1846,-93,-232,1846,33,-232,1822,67,-408,1798,67,-408,1798,43,-408,1806,19,-408,1838,51,-408,1798,43,-408,1782,51,-408,1806,19,-408,1806,-21,-408,1774,-29,-408,1774,-85,-408,1838,-85,-408,1838,51,-408,1806,19,-408,1806,-21,-408,1838,-85,-408,1886,-77,-412,1846,-109,-408,1870,-165,-412,2070,-165,-412,1886,51,-412,1862,43,-408,1862,-61,-408,1886,-77,-412,2070,-165,-412,2070,67,-408,1862,-69,-248,1886,-69,-248,1886`
+`,-45,-248,1886,51,-248,1862,51,-248,1862,-21,-248,1886,-29,-248],"vertslength":132,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,11,8,9,9,10,11,15,12,13,13,14,15,22,23,24,21,22,24,20,21,24,25,16,17,24,25,26,25,17,26,17,18,26,18,19,26,24,20,26,19,20,26,27,28,29,30,31,27,27,29,30,32,33,34,32,34,35,36,37,38,36,38,39,40,41,42,42,43,44,40,42,44,45,46,47,48,49,50,48,50,51,52,53,54,52,54,55,56,57,58,59,60,56,56,58,59,61,62,63,64,65,66,64,66,67,68,69,70,68,70,71,68,71,72,73,74,75,79,76,77,77,78,79,83,80,81,81,82,83,8`
+`4,85,86,91,87,88,91,88,89,89,90,91,98,92,93,98,93,94,98,94,95,98,95,96,96,97,98,99,100,101,103,99,101,101,102,103,104,105,106,107,108,109,107,109,110,111,112,113,111,113,114,115,116,117,115,117,118,119,120,121,119,121,122,124,119,122,122,123,124,125,126,127,128,129,130,128,130,131],"trislength":73,"triTopoly":[0,0,1,1,2,2,3,3,4,4,4,4,4,4,4,4,4,4,5,5,5,6,6,7,7,8,8,8,9,10,10,11,11,12,12,12,13,14,14,15,15,15,16,17,17,18,18,19,20,20,20,21,21,21,21,21,22,22,22,23,24,24,25,25,26,26,27,27,27,27,28,29,2`
+`9],"baseVert":[0,4,8,12,16,27,32,36,40,45,48,52,56,61,64,68,73,76,80,84,87,92,99,104,107,111,115,119,125,128],"vertsCount":[4,4,4,4,11,5,4,4,5,3,4,4,5,3,4,5,3,4,4,3,5,7,5,3,4,4,4,6,3,4],"baseTri":[0,2,4,6,8,18,21,23,25,28,29,31,33,36,37,39,42,43,45,47,48,51,56,59,60,62,64,66,70,71],"triCount":[2,2,2,2,10,3,2,2,3,1,2,2,3,1,2,3,1,2,2,1,3,5,3,1,2,2,2,4,1,2]},"links":{"poly":[17,19,20,28,21,29,28,29],"cost":[768,1632,1110,384],"type":[2,2,2,1],"pos":[1750,-69,-248,1766,-69,-232,1846,-93,-232,1862,-6`
+`9,-248,1846,-21,-226,1862,-21,-248,1886,-45,-248,1886,-29,-248],"length":4}}],["10_5",{"tileId":"10_5","tx":10,"ty":5,"mesh":{"verts":[2414,-45,-412,2430,-21,-412,2430,-5,-412,2414,27,-412,2390,27,-412,2390,-45,-412,2358,-21,-412,2294,-13,-408,2286,-29,-408,2366,-53,-412,2430,-53,-412,2414,-45,-412,2390,-45,-412,2366,-53,-412,2438,-165,-412,2102,-101,-408,2070,-93,-406,2070,-165,-412,2366,-53,-412,2286,-29,-408,2254,-21,-408,2102,-101,-408,2070,-165,-412,2438,-165,-412,2110,67,-408,2070,67,-406,`
+`2070,-69,-408,2110,-69,-408,2254,-21,-408,2270,19,-408,2254,-21,-408,2110,-69,-408,2102,-101,-408,2254,315,-408,2286,323,-408,2294,347,-408,2070,347,-408,2070,211,-406,2070,91,-408,2102,99,-408,2102,203,-408,2070,347,-408,2070,235,-408,2110,235,-408,2254,283,-408,2254,315,-408,2102,203,-408,2102,99,-408,2110,67,-408,2110,235,-408,2110,235,-408,2110,67,-408,2270,19,-408,2278,19,-408,2278,275,-408,2254,283,-408,2078,-21,-208,2078,-93,-208,2102,-93,-208,2102,-21,-208,2078,83,-208,2078,11,-208,2102,`
+`11,-208,2102,83,-208,2078,283,-208,2078,211,-208,2102,211,-208,2102,283,-208,2102,315,-208,2102,347,-208,2078,347,-208,2078,315,-208,2262,-21,-110,2262,-93,-110,2278,-93,-111,2278,-21,-111,2278,19,-111,2278,83,-111,2262,83,-110,2262,11,-110,2262,283,-110,2262,211,-110,2278,211,-111,2278,283,-111,2278,347,-111,2262,347,-110,2262,315,-110,2382,43,-412,2390,27,-412,2414,27,-412,2422,43,-412,2526,99,-408,2558,83,-408,2582,83,-412,2558,235,-408,2550,203,-408,2294,347,-408,2286,323,-408,2294,283,-408,`
+`2526,243,-408,2582,347,-412,2278,19,-408,2294,-13,-408,2358,-21,-412,2358,35,-412,2422,43,-412,2446,27,-412,2526,59,-406,2278,19,-408,2358,35,-412,2382,43,-412,2422,43,-412,2526,59,-406,2526,99,-408,2382,43,-412,2526,243,-408,2558,235,-408,2582,347,-412,2278,19,-408,2382,43,-412,2526,99,-408,2518,211,-406,2294,283,-408,2278,275,-408,2582,83,-412,2582,347,-412,2558,235,-408,2526,99,-408,2550,203,-408,2518,211,-406,2518,211,-406,2526,243,-408,2294,283,-408,2382,-29,-298,2382,-45,-288,2430,-45,-289`
+`,2430,-29,-298,2398,35,-311,2390,19,-307,2414,11,-314,2422,27,-308,2582,-77,-412,2558,-77,-408,2550,-101,-408,2582,-165,-412,2518,-69,-408,2534,-61,-408,2446,-37,-412,2518,-69,-408,2446,-37,-412,2430,-53,-412,2526,-101,-406,2582,-165,-412,2550,-101,-408,2526,-101,-406,2526,-101,-406,2430,-53,-412,2438,-165,-412,2582,-165,-412,2430,-5,-412,2430,-21,-412,2446,-37,-412,2582,83,-412,2558,83,-408,2550,59,-408,2534,-61,-408,2558,-77,-408,2582,-77,-412,2446,27,-412,2430,-5,-412,2446,-37,-412,2534,-61,-`
+`408,2550,59,-408,2526,59,-406,2526,-21,-208,2526,-93,-208,2550,-93,-208,2550,-21,-208,2526,83,-208,2526,11,-208,2550,11,-208,2550,83,-208,2526,283,-208,2526,211,-208,2550,211,-208,2550,283,-208,2550,315,-208,2550,347,-208,2526,347,-208,2526,315,-208],"vertslength":190,"polys":[0,5,6,9,10,14,15,17,18,23,24,29,30,32,33,36,37,40,41,45,46,49,50,55,56,59,60,63,64,67,68,71,72,75,76,79,80,83,84,86,87,90,91,95,96,100,101,104,105,107,108,110,111,114,115,117,118,123,124,126,127,129,130,132,133,136,137,140`
+`,141,144,145,147,148,151,152,154,155,158,159,161,162,167,168,173,174,177,178,181,182,185,186,189],"polyslength":46,"regions":[3,3,3,3,3,3,3,1,1,1,1,1,10,11,14,15,19,20,21,22,2,2,2,2,2,2,2,2,2,2,2,2,28,30,6,6,6,6,6,4,4,4,40,41,44,45],"neighbors":[[[0],[1,39],[0],[1,20],[0],[1,2]],[[1,23],[0],[1,4],[0]],[[0],[1,0],[0],[1,4],[1,38]],[[0],[0],[1,4]],[[1,1],[0],[1,6],[1,3],[0],[1,2]],[[0],[0],[0],[1,6],[0],[1,11]],[[1,5],[0],[1,4]],[[0],[1,22],[0],[1,9]],[[0],[0],[1,10],[0]],[[0],[0],[1,11],[0],[1,7]`
+`],[[1,8],[0],[1,11],[0]],[[1,10],[1,5],[0],[1,28],[0],[1,9]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[1,0],[0],[1,26]],[[0],[1,40],[1,29],[0],[1,30]],[[1,7],[0],[1,31],[1,27],[0]],[[0],[1,1],[0],[1,25]],[[0],[1,41],[1,26]],[[1,23],[0],[1,28]],[[1,24],[0],[1,28],[1,20]],[[0],[1,29],[1,22]],[[1,25],[1,26],[1,30],[1,31],[0],[1,11]],[[0],[1,27],[1,21]],[[1,21],[0],[1,28]],[[0],[1,22],[1,28]],[[0]`
+`,[0],[0],[0]],[[0],[0],[0],[0]],[[1,40],[0],[1,37],[0]],[[0],[1,41],[1,36]],[[1,35],[0],[1,38],[0]],[[1,34],[0],[1,38]],[[1,36],[1,2],[0],[1,37]],[[1,0],[0],[1,41]],[[1,21],[0],[1,41],[0],[1,34],[0]],[[0],[1,39],[1,35],[1,40],[0],[1,24]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2414,-45,-412,2430,-21,-412,2430,-5,-412,2414,27,-412,2390,27,-412,2390,-45,-412,2358,-21,-412,2294,-13,-412,2286,-29,-408,2366,-53,-412,2430,-53,-412,2414,-45,-412,2390`
+`,-45,-412,2366,-53,-412,2438,-165,-412,2102,-101,-408,2070,-93,-406,2070,-165,-412,2366,-53,-412,2286,-29,-408,2254,-21,-408,2102,-101,-408,2070,-165,-412,2438,-165,-412,2110,67,-412,2070,67,-406,2070,-69,-408,2110,-69,-412,2254,-21,-408,2270,19,-408,2254,-21,-408,2110,-69,-412,2102,-101,-408,2254,315,-408,2286,323,-408,2294,347,-408,2070,347,-408,2070,211,-406,2070,91,-408,2102,99,-408,2102,203,-408,2070,347,-408,2070,235,-408,2110,235,-412,2254,283,-408,2254,315,-408,2102,203,-408,2102,99,-408`
+`,2110,67,-412,2110,235,-412,2110,235,-412,2110,67,-412,2270,19,-408,2278,19,-408,2278,275,-408,2254,283,-408,2078,-21,-209,2078,-93,-209,2102,-93,-208,2102,-21,-208,2078,83,-209,2078,11,-209,2102,11,-208,2102,83,-208,2078,283,-209,2078,211,-209,2102,211,-208,2102,283,-208,2102,315,-208,2102,347,-208,2078,347,-209,2078,315,-209,2262,-21,-111,2262,-93,-111,2278,-93,-111,2278,-21,-111,2278,19,-111,2278,83,-111,2262,83,-111,2262,11,-111,2262,283,-111,2262,211,-111,2278,211,-111,2278,283,-111,2278,34`
+`7,-111,2262,347,-111,2262,315,-111,2382,43,-412,2390,27,-412,2414,27,-412,2422,43,-412,2526,99,-408,2558,83,-412,2582,83,-412,2558,235,-412,2550,203,-408,2294,347,-412,2286,323,-408,2294,283,-412,2526,243,-408,2582,347,-412,2278,19,-408,2294,-13,-412,2358,-21,-412,2358,35,-412,2422,43,-412,2446,27,-412,2526,59,-406,2278,19,-408,2358,35,-412,2382,43,-412,2422,43,-412,2526,59,-406,2526,99,-408,2382,43,-412,2526,243,-408,2558,235,-412,2582,347,-412,2278,19,-408,2382,43,-412,2526,99,-408,2518,211,-4`
+`08,2294,283,-412,2278,275,-408,2582,83,-412,2582,347,-412,2558,235,-412,2526,99,-408,2550,203,-408,2518,211,-408,2518,211,-408,2526,243,-408,2294,283,-412,2382,-29,-303,2382,-45,-303,2398,-45,-311,2414,-45,-298,2430,-45,-298,2430,-29,-298,2414,-29,-298,2398,-29,-311,2398,35,-311,2390,19,-311,2414,11,-308,2422,27,-308,2582,-77,-412,2558,-77,-412,2550,-101,-408,2582,-165,-412,2518,-69,-408,2534,-61,-408,2446,-37,-412,2518,-69,-408,2446,-37,-412,2430,-53,-412,2526,-101,-408,2582,-165,-412,2550,-101`
+`,-408,2526,-101,-408,2526,-101,-408,2430,-53,-412,2438,-165,-412,2582,-165,-412,2430,-5,-412,2430,-21,-412,2446,-37,-412,2582,83,-412,2558,83,-412,2550,59,-408,2534,-61,-408,2558,-77,-412,2582,-77,-412,2446,27,-412,2430,-5,-412,2446,-37,-412,2534,-61,-408,2550,59,-408,2526,59,-408,2526,-21,-209,2526,-93,-209,2550,-93,-208,2550,-21,-208,2526,83,-209,2526,11,-209,2550,11,-208,2550,83,-208,2526,283,-209,2526,211,-209,2550,211,-208,2550,283,-208,2550,315,-208,2550,347,-208,2526,347,-209,2526,315,-20`
+`9],"vertslength":194,"tris":[0,1,2,2,3,4,5,0,2,2,4,5,6,7,8,6,8,9,10,11,12,10,12,13,10,13,14,15,16,17,18,19,20,20,21,22,23,18,20,20,22,23,24,25,26,24,26,27,27,28,29,24,27,29,30,31,32,33,34,35,33,35,36,38,39,40,37,38,40,41,42,43,43,44,45,41,43,45,49,46,47,47,48,49,51,52,53,54,55,50,54,50,51,51,53,54,59,56,57,57,58,59,63,60,61,61,62,63,67,64,65,65,66,67,71,68,69,69,70,71,75,72,73,73,74,75,76,77,78,76,78,79,83,80,81,81,82,83,84,85,86,87,88,89,87,89,90,91,92,93,94,95,91,91,93,94,96,97,98,96,98,99,96,`
+`99,100,101,102,103,101,103,104,105,106,107,108,109,110,111,112,113,111,113,114,115,116,117,119,120,121,121,122,123,123,118,119,119,121,123,124,125,126,127,128,129,130,131,132,140,133,134,140,134,135,140,135,136,136,137,138,136,138,139,136,139,140,141,142,143,141,143,144,145,146,147,145,147,148,149,150,151,152,153,154,152,154,155,156,157,158,159,160,161,159,161,162,163,164,165,166,167,168,169,170,171,168,169,171,166,168,171,172,173,174,176,177,172,172,174,175,172,175,176,181,178,179,179,180,181,1`
+`85,182,183,183,184,185,189,186,187,187,188,189,193,190,191,191,192,193],"trislength":102,"triTopoly":[0,0,0,0,1,1,2,2,2,3,4,4,4,4,5,5,5,5,6,7,7,8,8,9,9,9,10,10,11,11,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,20,20,21,21,21,22,22,22,23,23,24,25,26,26,27,28,28,28,28,29,30,31,32,32,32,32,32,32,33,33,34,34,35,36,36,37,38,38,39,40,40,40,40,41,41,41,41,42,42,43,43,44,44,45,45],"baseVert":[0,6,10,15,18,24,30,33,37,41,46,50,56,60,64,68,72,76,80,84,87,91,96,101,105,108,111,115,118,124,127,130,13`
+`3,141,145,149,152,156,159,163,166,172,178,182,186,190],"vertsCount":[6,4,5,3,6,6,3,4,4,5,4,6,4,4,4,4,4,4,4,3,4,5,5,4,3,3,4,3,6,3,3,3,8,4,4,3,4,3,4,3,6,6,4,4,4,4],"baseTri":[0,4,6,9,10,14,18,19,21,23,26,28,32,34,36,38,40,42,44,46,47,49,52,55,57,58,59,61,62,66,67,68,69,75,77,79,80,82,83,85,86,90,94,96,98,100],"triCount":[4,2,3,1,4,4,1,2,2,3,2,4,2,2,2,2,2,2,2,1,2,3,3,2,1,1,2,1,4,1,1,1,6,2,2,1,2,1,2,1,4,4,2,2,2,2]},"links":{"poly":[12,13,14,15,42,43,44,45],"cost":[1536,1536,1536,1536],"type":[1,1,1,`
+`1],"pos":[2102,-21,-208,2102,11,-208,2102,283,-208,2102,315,-208,2550,-21,-208,2550,11,-208,2550,283,-208,2550,315,-208],"length":4}}],["11_5",{"tileId":"11_5","tx":11,"ty":5,"mesh":{"verts":[2686,-21,-408,2686,11,-408,2582,27,-412,2742,-165,-412,2718,-29,-408,2686,-21,-408,2686,-21,-408,2582,27,-412,2582,-165,-412,2742,-165,-412,2814,43,-412,2838,27,-412,2862,27,-412,2886,43,-412,2726,19,-408,2734,3,-408,2806,19,-412,2814,43,-412,2886,43,-412,2894,19,-412,2966,19,-408,3094,19,-412,2998,275,-408`
+`,2958,283,-408,3094,347,-412,3006,315,-408,2998,275,-408,3094,19,-412,2966,19,-408,3006,3,-408,3094,19,-412,2686,11,-408,2726,19,-408,2814,43,-412,2726,275,-408,2686,283,-408,2582,27,-412,2966,323,-408,3006,315,-408,3094,347,-412,2582,347,-412,2694,323,-408,2734,315,-408,2966,323,-408,3094,347,-412,2686,283,-408,2694,323,-408,2582,347,-412,2582,27,-412,2814,43,-412,2886,43,-412,2958,283,-408,2966,323,-408,2734,315,-408,2726,275,-408,2694,115,-110,2694,-125,-110,2726,-125,-110,2726,115,-110,2726,`
+`179,-110,2718,347,-114,2702,347,-114,2694,179,-110,2862,-45,-412,2878,-21,-412,2878,-5,-412,2862,27,-412,2838,27,-412,2830,-53,-412,2878,-53,-412,2862,-45,-412,2830,-53,-412,2886,-165,-412,2806,19,-412,2734,3,-408,2718,-29,-408,2806,-45,-412,2830,-53,-412,2806,-45,-412,2718,-29,-408,2742,-165,-412,2886,-165,-412,2830,-29,-298,2830,-45,-288,2878,-45,-289,2878,-29,-298,2846,35,-311,2838,19,-307,2862,11,-314,2870,27,-308,2878,-5,-412,2878,-21,-412,2894,-29,-412,2966,-29,-408,2966,19,-408,2894,19,-4`
+`12,2966,-29,-408,2894,-29,-412,2878,-53,-412,2886,-165,-412,3094,-165,-412,2998,-29,-408,3094,19,-412,3006,3,-408,2998,-29,-408,3094,-165,-412,2974,-21,-110,2974,-93,-110,2990,-93,-111,2990,-21,-111,2974,83,-110,2974,19,-110,2990,19,-111,2990,83,-111,2974,283,-110,2974,211,-110,2990,211,-111,2990,283,-111,2990,323,-111,2990,347,-111,2974,347,-110,2974,323,-110],"vertslength":122,"polys":[0,2,3,5,6,9,10,13,14,17,18,23,24,27,28,30,31,36,37,39,40,44,45,48,49,54,55,58,59,62,63,68,69,72,73,76,77,81,8`
+`2,85,86,89,90,95,96,101,102,105,106,109,110,113,114,117,118,121],"polyslength":28,"regions":[2,2,2,1,1,1,1,1,1,1,1,1,1,7,8,3,3,3,3,14,16,4,4,4,25,26,27,28],"neighbors":[[[0],[1,8],[1,2]],[[1,18],[0],[1,2]],[[1,0],[0],[0],[1,1]],[[0],[1,15],[0],[1,12]],[[0],[1,17],[0],[1,8]],[[0],[1,21],[1,7],[1,6],[0],[1,12]],[[1,9],[0],[1,5],[0]],[[0],[1,23],[1,5]],[[0],[1,4],[1,12],[0],[1,11],[1,0]],[[0],[1,6],[1,10]],[[1,11],[0],[1,12],[1,9],[0]],[[0],[1,10],[0],[1,8]],[[1,3],[1,5],[0],[1,10],[0],[1,8]],[[0],`
+`[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,21],[0],[1,3],[0],[1,16]],[[0],[1,15],[1,18],[1,22]],[[1,4],[0],[1,18],[0]],[[0],[1,17],[1,1],[0],[1,16]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,15],[0],[1,22],[0],[1,5],[0]],[[1,21],[0],[1,16],[0],[1,23],[0]],[[1,7],[0],[1,22],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2686,-21,-412,2686,11,-412,2582,27,-412,2742,-165,-412,2718,-29,-408,2686,-21,-412,2686,-21,-412,2582,27,-412,2582,-165,-412,2742,-165`
+`,-412,2814,43,-412,2838,27,-412,2862,27,-412,2886,43,-412,2726,19,-408,2734,3,-412,2806,19,-412,2814,43,-412,2886,43,-412,2894,19,-412,2966,19,-408,3094,19,-412,2998,275,-408,2958,283,-412,3094,347,-412,3006,315,-412,2998,275,-408,3094,19,-412,2966,19,-408,3006,3,-412,3094,19,-412,2686,11,-408,2726,19,-408,2814,43,-412,2726,275,-408,2686,283,-412,2582,27,-412,2966,323,-408,3006,315,-412,3094,347,-412,2582,347,-412,2694,323,-408,2734,315,-412,2966,323,-408,3094,347,-412,2686,283,-412,2694,323,-40`
+`8,2582,347,-412,2582,27,-412,2814,43,-412,2886,43,-412,2958,283,-412,2966,323,-408,2734,315,-412,2726,275,-408,2694,115,-114,2694,-125,-110,2726,-125,-110,2726,115,-110,2726,179,-110,2718,347,-114,2702,347,-114,2694,179,-110,2862,-45,-412,2878,-21,-412,2878,-5,-412,2862,27,-412,2838,27,-412,2830,-53,-412,2878,-53,-412,2862,-45,-412,2830,-53,-412,2886,-165,-412,2806,19,-412,2734,3,-412,2718,-29,-408,2806,-45,-412,2830,-53,-412,2806,-45,-412,2718,-29,-408,2742,-165,-412,2886,-165,-412,2830,-29,-30`
+`3,2830,-45,-303,2846,-45,-311,2862,-45,-298,2878,-45,-298,2878,-29,-298,2862,-29,-298,2846,-29,-311,2846,35,-311,2838,19,-311,2862,11,-308,2870,27,-308,2878,-5,-412,2878,-21,-412,2894,-29,-412,2966,-29,-408,2966,19,-408,2894,19,-412,2966,-29,-408,2894,-29,-412,2878,-53,-412,2886,-165,-412,3094,-165,-412,2998,-29,-408,3094,19,-412,3006,3,-412,2998,-29,-408,3094,-165,-412,2974,-21,-111,2974,-93,-111,2990,-93,-111,2990,-21,-111,2974,83,-111,2974,19,-111,2990,19,-111,2990,83,-111,2974,283,-111,2974,`
+`211,-111,2990,211,-111,2990,283,-111,2990,323,-111,2990,347,-111,2974,347,-111,2974,323,-111],"vertslength":126,"tris":[0,1,2,3,4,5,6,7,8,6,8,9,10,11,12,10,12,13,14,15,16,14,16,17,18,19,20,18,20,21,22,23,18,18,21,22,24,25,26,24,26,27,28,29,30,31,32,33,36,31,33,33,34,35,33,35,36,37,38,39,40,41,42,42,43,44,40,42,44,45,46,47,45,47,48,51,52,53,51,53,54,54,49,50,50,51,54,58,55,56,56,57,58,59,60,61,59,61,62,63,64,65,65,66,67,68,63,65,65,67,68,69,70,71,69,71,72,73,74,75,73,75,76,77,78,79,77,79,80,77,80`
+`,81,89,82,83,89,83,84,89,84,85,85,86,87,85,87,88,85,88,89,90,91,92,90,92,93,94,95,96,99,94,96,99,96,97,97,98,99,100,101,102,105,100,102,105,102,103,103,104,105,106,107,108,106,108,109,113,110,111,111,112,113,117,114,115,115,116,117,121,118,119,119,120,121,125,122,123,123,124,125],"trislength":70,"triTopoly":[0,1,2,2,3,3,4,4,5,5,5,5,6,6,7,8,8,8,8,9,10,10,10,11,11,12,12,12,12,13,13,14,14,15,15,15,15,16,16,17,17,18,18,18,19,19,19,19,19,19,20,20,21,21,21,21,22,22,22,22,23,23,24,24,25,25,26,26,27,27]`
+`,"baseVert":[0,3,6,10,14,18,24,28,31,37,40,45,49,55,59,63,69,73,77,82,90,94,100,106,110,114,118,122],"vertsCount":[3,3,4,4,4,6,4,3,6,3,5,4,6,4,4,6,4,4,5,8,4,6,6,4,4,4,4,4],"baseTri":[0,1,2,4,6,8,12,14,15,19,20,23,25,29,31,33,37,39,41,44,50,52,56,60,62,64,66,68],"triCount":[1,1,2,2,2,4,2,1,4,1,3,2,4,2,2,4,2,2,3,6,2,4,4,2,2,2,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_5",{"tileId":"12_5","tx":12,"ty":5,"mesh":{"verts":[3206,-165,-412,3190,-109,-408,3166,-101,-408,3094`
+`,-165,-412,3198,59,-408,3174,83,-408,3094,91,-412,3094,-165,-412,3166,-69,-408,3094,-165,-412,3166,-101,-408,3166,-69,-408,3166,-69,-408,3198,-61,-408,3198,59,-408,3094,91,-412,3174,83,-408,3198,99,-408,3198,203,-408,3174,227,-408,3094,235,-412,3094,235,-412,3174,227,-408,3206,243,-408,3214,347,-408,3094,347,-412,3382,-165,-408,3366,-149,-408,3302,-149,-408,3206,-165,-412,3214,-77,-408,3190,-109,-408,3206,-165,-412,3302,-149,-408,3294,-85,-408,3190,-21,-14,3190,-93,-14,3206,-93,-13,3206,-21,-13,`
+`3206,83,-13,3190,83,-14,3190,19,-14,3206,11,-13,3190,283,-14,3190,211,-14,3206,211,-13,3206,283,-13,3198,347,-140,3190,347,-140,3190,243,-140,3206,347,-13,3190,347,-14,3198,315,-13,3374,-77,-408,3382,-77,-408,3406,-61,-408,3302,-69,-408,3214,-77,-408,3294,-85,-408,3302,-69,-408,3198,-61,-408,3214,83,-408,3198,59,-408,3198,-61,-408,3406,-61,-408,3406,59,-408,3382,83,-408,3214,83,-408,3198,-61,-408,3302,-69,-408,3198,203,-408,3198,99,-408,3214,83,-408,3382,83,-408,3406,99,-408,3406,203,-408,3382,2`
+`27,-408,3214,227,-408,3198,203,-408,3214,83,-408,3382,83,-408,3406,203,-408,3214,347,-408,3206,243,-408,3214,227,-408,3382,227,-408,3414,243,-408,3422,347,-408,3318,-93,-408,3318,-125,-408,3350,-125,-408,3350,-93,-408,3342,-93,-292,3358,-101,-292,3358,-85,-292,3390,-101,-404,3382,-77,-408,3374,-77,-408,3366,-149,-408,3390,-101,-404,3366,-149,-408,3382,-165,-408,3510,-165,-412,3414,-101,-408,3510,99,-412,3422,83,-408,3406,59,-408,3422,-69,-408,3510,-165,-412,3406,59,-408,3406,-61,-408,3422,-69,-4`
+`08,3422,-69,-408,3414,-101,-408,3510,-165,-412,3390,-21,105,3390,-93,105,3406,-93,104,3406,-21,104,3406,19,104,3406,83,104,3390,83,105,3390,11,105,3390,283,105,3390,211,105,3406,211,104,3406,283,104,3406,347,104,3390,347,105,3390,315,105,3406,347,-205,3398,347,-205,3398,243,-205,3406,347,-48,3398,347,-48,3398,243,-48,3406,99,-408,3422,83,-408,3510,99,-412,3510,243,-412,3422,227,-408,3406,203,-408,3414,243,-408,3422,227,-408,3510,243,-412,3510,347,-412,3422,347,-408,3542,347,-352,3542,-165,-352,3`
+`550,-165,-352,3550,347,-352],"vertslength":151,"polys":[0,3,4,8,9,11,12,14,15,20,21,25,26,29,30,34,35,38,39,42,43,46,47,49,50,52,53,56,57,60,61,63,64,69,70,72,73,75,76,81,82,87,88,91,92,94,95,98,99,103,104,108,109,111,112,114,115,118,119,122,123,126,127,129,130,132,133,135,136,141,142,146,147,150],"polyslength":37,"regions":[5,5,5,5,7,8,10,10,13,16,18,19,21,2,2,2,2,1,1,1,3,24,26,4,4,4,4,4,28,29,30,31,34,35,6,9,36],"neighbors":[[[1,7],[0],[1,2],[0]],[[0],[1,4],[0],[1,2],[1,3]],[[1,0],[0],[1,1]],[`
+`[0],[1,15],[1,1]],[[1,1],[0],[1,17],[0],[1,5],[0]],[[1,4],[0],[1,20],[0],[0]],[[1,24],[0],[1,7],[0]],[[0],[1,0],[1,6],[0],[1,14]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[1,23],[0],[1,16],[0]],[[1,7],[0],[1,16],[0]],[[0],[1,3],[1,16]],[[1,26],[0],[1,19],[1,15],[1,14],[1,13]],[[1,4],[0],[1,19]],[[0],[1,34],[1,19]],[[1,20],[0],[1,17],[1,16],[1,18],[0]],[[1,5],[0],[1,19],[0],[1,35],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[1,13],[0],[1,24]],[[1,23],[1,6],`
+`[0],[1,27],[0]],[[1,34],[0],[1,26],[1,27],[0]],[[1,16],[0],[1,25]],[[0],[1,24],[1,25]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[1,25],[0],[1,35],[0],[1,18]],[[0],[1,34],[0],[0],[1,20]],[[0],[0],[0],[0]]]},"detail":{"verts":[3206,-165,-412,3190,-109,-408,3166,-101,-412,3094,-165,-412,3198,59,-408,3174,83,-408,3094,91,-412,3094,-165,-412,3166,-69,-408,3094,-165,-412,3166,-101,-412,3166,-69,-408,3166,-69,-408,3198,-61,-408,3198,59,-408,30`
+`94,91,-412,3174,83,-408,3198,99,-408,3198,203,-408,3174,227,-408,3094,235,-412,3094,235,-412,3174,227,-408,3206,243,-408,3214,347,-408,3094,347,-412,3382,-165,-408,3366,-149,-408,3302,-149,-408,3206,-165,-412,3214,-77,-412,3190,-109,-404,3206,-165,-412,3302,-149,-408,3294,-85,-408,3190,-21,-13,3190,-93,-13,3206,-93,-13,3206,-21,-13,3206,83,-13,3190,83,-13,3190,19,-13,3206,11,-13,3190,283,-13,3190,211,-13,3206,211,-13,3206,283,-13,3198,347,-140,3190,347,-140,3190,243,-140,3206,347,-13,3190,347,-1`
+`3,3198,315,-13,3374,-77,-408,3382,-77,-408,3406,-61,-408,3302,-69,-408,3214,-77,-412,3294,-85,-408,3302,-69,-408,3198,-61,-408,3214,83,-412,3198,59,-408,3198,-61,-408,3406,-61,-408,3406,59,-408,3382,83,-408,3214,83,-412,3198,-61,-408,3302,-69,-408,3198,203,-408,3198,99,-408,3214,83,-412,3382,83,-408,3406,99,-408,3406,203,-408,3382,227,-408,3214,227,-412,3198,203,-408,3214,83,-412,3382,83,-408,3406,203,-408,3214,347,-412,3206,243,-408,3214,227,-412,3382,227,-408,3414,243,-408,3422,347,-408,3318,-`
+`93,-408,3318,-125,-408,3350,-125,-408,3350,-93,-408,3342,-93,-292,3358,-101,-292,3358,-85,-292,3390,-101,-408,3382,-77,-408,3374,-77,-408,3366,-149,-408,3390,-101,-408,3366,-149,-408,3382,-165,-408,3510,-165,-412,3414,-101,-408,3510,99,-412,3422,83,-412,3406,59,-408,3422,-69,-412,3510,-165,-412,3406,59,-408,3406,-61,-408,3422,-69,-412,3422,-69,-412,3414,-101,-408,3510,-165,-412,3390,-21,104,3390,-93,104,3406,-93,104,3406,-21,104,3406,19,104,3406,83,104,3390,83,104,3390,11,104,3390,283,104,3390,2`
+`11,104,3406,211,104,3406,283,104,3406,347,104,3390,347,104,3390,315,104,3406,347,-205,3398,347,-205,3398,243,-205,3406,347,-48,3398,347,-48,3398,243,-48,3406,99,-408,3422,83,-412,3510,99,-412,3510,243,-412,3422,227,-412,3406,203,-408,3414,243,-408,3422,227,-412,3510,243,-412,3510,347,-412,3422,347,-412,3542,347,-352,3542,-165,-352,3550,-165,-352,3550,347,-352],"vertslength":151,"tris":[0,1,2,0,2,3,4,5,6,8,4,6,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,15,17,18,15,18,20,21,22,23,25,21,23,23,24,25,2`
+`6,27,28,26,28,29,30,31,32,33,34,30,30,32,33,38,35,36,36,37,38,39,40,41,39,41,42,46,43,44,44,45,46,47,48,49,50,51,52,53,54,55,53,55,56,57,58,59,57,59,60,61,62,63,64,65,66,68,69,64,66,67,68,64,66,68,70,71,72,73,74,75,77,78,79,80,81,76,80,76,77,77,79,80,82,83,84,85,86,87,82,84,85,82,85,87,91,88,89,89,90,91,92,93,94,95,96,97,95,97,98,99,100,101,103,99,101,101,102,103,104,105,106,104,106,107,104,107,108,109,110,111,112,113,114,118,115,116,116,117,118,119,120,121,119,121,122,126,123,124,124,125,126,12`
+`7,128,129,130,131,132,133,134,135,136,137,138,139,140,141,141,136,138,138,139,141,142,143,144,144,145,146,142,144,146,150,147,148,148,149,150],"trislength":77,"triTopoly":[0,0,1,1,1,2,3,4,4,4,4,5,5,5,6,6,7,7,7,8,8,9,9,10,10,11,12,13,13,14,14,15,16,16,16,16,17,18,19,19,19,19,20,20,20,20,21,21,22,23,23,24,24,24,25,25,25,26,27,28,28,29,29,30,30,31,32,33,34,34,34,34,35,35,35,36,36],"baseVert":[0,4,9,12,15,21,26,30,35,39,43,47,50,53,57,61,64,70,73,76,82,88,92,95,99,104,109,112,115,119,123,127,130,133`
+`,136,142,147],"vertsCount":[4,5,3,3,6,5,4,5,4,4,4,3,3,4,4,3,6,3,3,6,6,4,3,4,5,5,3,3,4,4,4,3,3,3,6,5,4],"baseTri":[0,2,5,6,7,11,14,16,19,21,23,25,26,27,29,31,32,36,37,38,42,46,48,49,51,54,57,58,59,61,63,65,66,67,68,72,75],"triCount":[2,3,1,1,4,3,2,3,2,2,2,1,1,2,2,1,4,1,1,4,4,2,1,2,3,3,1,1,2,2,2,1,1,1,4,3,2]},"links":{"poly":[8,9],"cost":[1536],"type":[1],"pos":[3206,-21,-13,3206,11,-13],"length":1}}],["0_6",{"tileId":"0_6","tx":0,"ty":6,"mesh":{"verts":[-2538,347,-416,-2818,451,-408,-2826,419,-40`
+`8,-3010,347,-408,-2538,347,-416,-2826,419,-408,-3010,347,-408,-2826,419,-408,-2850,427,-408,-3010,347,-408,-2850,427,-408,-2842,459,-408,-3010,859,-408,-3010,859,-408,-2842,459,-408,-2818,451,-408,-2818,451,-408,-2538,347,-416,-2538,859,-416,-3010,859,-408],"vertslength":20,"polys":[0,2,3,5,6,8,9,12,13,15,16,19],"polyslength":6,"regions":[1,1,1,1,1,1],"neighbors":[[[1,5],[0],[1,1]],[[0],[1,0],[1,2]],[[1,1],[0],[1,3]],[[1,2],[0],[1,4],[0]],[[1,3],[0],[1,5]],[[1,0],[0],[0],[1,4]]]},"detail":{"vert`
+`s":[-2538,347,-416,-2818,451,-416,-2826,419,-408,-2803.84619140625,413.4615478515625,-416,-3010,347,-408,-2538,347,-416,-2803.84619140625,413.4615478515625,-416,-2826,419,-408,-2830,407,-408,-3010,347,-408,-2826,419,-408,-2850,427,-408,-3010,347,-408,-2850,427,-408,-2842,459,-408,-3010,859,-408,-3010,859,-408,-2842,459,-408,-2818,451,-416,-2828.105224609375,472.47369384765625,-408,-2818,451,-416,-2538,347,-416,-2538,859,-416,-3010,859,-408,-2828.105224609375,472.47369384765625,-408,-2806,479,-41`
+`6],"vertslength":26,"tris":[1,2,3,0,1,3,6,7,8,7,4,8,4,5,8,6,5,8,9,10,11,12,13,14,12,14,15,17,18,19,16,17,19,21,22,25,24,20,25,21,20,25,22,23,25,24,23,25],"trislength":16,"triTopoly":[0,0,1,1,1,1,2,3,3,4,4,5,5,5,5,5],"baseVert":[0,4,9,12,16,20],"vertsCount":[4,5,3,4,4,6],"baseTri":[0,2,6,7,9,11],"triCount":[2,4,1,2,2,5]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_6",{"tileId":"1_6","tx":1,"ty":6,"mesh":{"verts":[-2538,347,-416,-2522,347,-408,-2514,483,-408,-2522,563,-408,-2`
+`538,859,-416,-2498,707,-408,-2482,683,-408,-2522,827,-408,-2522,699,-408,-2498,707,-408,-2538,859,-416,-2522,563,-408,-2522,699,-408,-2538,859,-416,-2538,347,-416,-2026,859,-408,-2490,843,-408,-2498,819,-408,-2482,683,-408,-2514,851,-408,-2490,843,-408,-2026,859,-408,-2538,859,-416,-2482,563,-408,-2522,563,-408,-2514,483,-408,-2482,459,-408,-2522,827,-408,-2514,851,-408,-2538,859,-416,-2498,819,-408,-2522,827,-408,-2482,683,-408,-2482,459,-408,-2482,347,-408,-2026,347,-408,-2026,859,-408,-2482,6`
+`83,-408,-2482,563,-408],"vertslength":39,"polys":[0,3,4,7,8,10,11,14,15,18,19,22,23,26,27,29,30,32,33,38],"polyslength":10,"regions":[1,1,1,1,1,1,1,1,1,1],"neighbors":[[[0],[0],[1,6],[1,3]],[[1,2],[0],[1,8],[1,7]],[[0],[1,1],[1,3]],[[0],[1,2],[0],[1,0]],[[1,5],[0],[1,8],[1,9]],[[0],[1,4],[0],[1,7]],[[0],[1,0],[0],[1,9]],[[0],[1,5],[1,1]],[[0],[1,1],[1,4]],[[0],[0],[0],[1,4],[0],[1,6]]]},"detail":{"verts":[-2538,347,-408,-2522,347,-408,-2514,483,-408,-2522,563,-408,-2538,859,-408,-2498,707,-408,-`
+`2482,683,-408,-2522,827,-408,-2522,699,-408,-2498,707,-408,-2538,859,-408,-2522,563,-408,-2522,699,-408,-2538,859,-408,-2538,347,-408,-2026,859,-408,-2490,843,-408,-2498,819,-408,-2482,683,-408,-2514,851,-408,-2490,843,-408,-2026,859,-408,-2538,859,-408,-2482,563,-408,-2522,563,-408,-2514,483,-408,-2482,459,-408,-2522,827,-408,-2514,851,-408,-2538,859,-408,-2498,819,-408,-2522,827,-408,-2482,683,-408,-2482,459,-408,-2482,347,-408,-2026,347,-408,-2026,859,-408,-2482,683,-408,-2482,563,-408],"vert`
+`slength":39,"tris":[0,1,2,0,2,3,5,6,7,4,5,7,8,9,10,11,12,13,11,13,14,16,17,18,15,16,18,19,20,21,19,21,22,23,24,25,23,25,26,27,28,29,30,31,32,33,34,35,38,33,35,36,37,38,35,36,38],"trislength":19,"triTopoly":[0,0,1,1,2,3,3,4,4,5,5,6,6,7,8,9,9,9,9],"baseVert":[0,4,8,11,15,19,23,27,30,33],"vertsCount":[4,4,3,4,4,4,4,3,3,6],"baseTri":[0,2,4,5,7,9,11,13,14,15],"triCount":[2,2,1,2,2,2,2,1,1,4]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_6",{"tileId":"2_6","tx":2,"ty":6,"mesh":{"v`
+`erts":[-2026,859,-408,-2026,347,-408,-1514,347,-408,-1514,859,-408],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[-2026,859,-408,-2026,347,-408,-1514,347,-408,-1514,859,-408],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_6",{"tileId":"3_6","tx":3,"ty":6,"mesh":{"verts":[-1514,859,-`
+`408,-1514,347,-408,-1002,347,-408,-1002,859,-408],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[-1514,859,-408,-1514,347,-408,-1002,347,-408,-1002,859,-408],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_6",{"tileId":"4_6","tx":4,"ty":6,"mesh":{"verts":[-538,795,-408,-514,787,-408,-`
+`490,859,-416,-490,347,-408,-514,531,-408,-522,507,-408,-490,347,-408,-522,507,-408,-546,515,-408,-1002,347,-408,-514,531,-408,-490,347,-408,-490,859,-416,-522,763,-408,-538,539,-408,-514,531,-408,-522,763,-408,-538,539,-408,-522,763,-408,-546,771,-408,-1002,859,-408,-1002,347,-408,-546,515,-408,-538,795,-408,-490,859,-416,-1002,859,-408,-546,771,-408,-538,795,-408,-1002,859,-408],"vertslength":29,"polys":[0,2,3,5,6,9,10,13,14,16,17,22,23,25,26,28],"polyslength":8,"regions":[1,1,1,1,1,1,1,1],"nei`
+`ghbors":[[[0],[0],[1,6]],[[1,3],[0],[1,2]],[[1,1],[0],[1,5],[0]],[[1,1],[0],[0],[1,4]],[[0],[1,3],[1,5]],[[1,4],[0],[1,7],[0],[1,2],[0]],[[1,0],[0],[1,7]],[[0],[1,6],[1,5]]]},"detail":{"verts":[-538,795,-408,-514,787,-408,-490,859,-416,-490,347,-408,-514,531,-408,-522,507,-408,-490,347,-408,-522,507,-408,-546,515,-408,-567.7142944335938,507,-416,-1002,347,-408,-839.0908813476562,347,-416,-583.0908813476562,347,-416,-559.8181762695312,347,-408,-558,479,-408,-514,531,-408,-490,347,-408,-490,812.45`
+`45288085938,-408,-490,859,-416,-522,763,-408,-538,539,-408,-514,531,-408,-522,763,-408,-538,539,-408,-522,763,-408,-546,771,-408,-568.7999877929688,775.4000244140625,-416,-1002,859,-408,-1002,347,-408,-828.2857055664062,411,-416,-567.7142944335938,507,-416,-546,515,-408,-558,767,-408,-558,527,-408,-538,795,-408,-490,859,-416,-839.0908813476562,859,-416,-862.3636474609375,859,-408,-1002,859,-408,-862.7999877929688,839.7999877929688,-408,-839.5999755859375,836.5999755859375,-416,-584.4000244140625`
+`,801.4000244140625,-416,-558,831,-408,-546,771,-408,-538,795,-408,-584.4000244140625,801.4000244140625,-416,-677.2000122070312,814.2000122070312,-416,-1002,859,-408,-842.4000244140625,828.2000122070312,-408,-819.5999755859375,823.7999877929688,-416,-568.7999877929688,775.4000244140625,-416],"vertslength":51,"tris":[0,1,2,3,4,5,9,10,11,13,6,14,6,7,14,9,8,14,7,8,14,13,12,14,9,11,14,12,11,14,17,18,19,17,19,15,15,16,17,20,21,22,27,28,29,26,27,29,23,24,32,24,25,32,26,25,32,30,31,33,23,31,33,23,32,33,`
+`26,32,33,26,29,33,30,29,33,37,38,39,36,37,39,36,39,40,36,40,41,35,36,42,36,41,42,41,34,42,35,34,42,50,43,44,50,44,45,50,45,46,49,50,46,48,49,46,46,47,48],"trislength":39,"triTopoly":[0,1,2,2,2,2,2,2,2,2,3,3,3,4,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,7,7,7,7,7,7],"baseVert":[0,3,6,15,20,23,34,43],"vertsCount":[3,3,9,5,3,11,9,8],"baseTri":[0,1,2,10,13,14,25,33],"triCount":[1,1,8,3,1,11,8,6]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_6",{"tileId":"5_6","tx":5,"ty":6,"mesh":{"`
+`verts":[-490,859,-416,-490,347,-408,-394,347,-408,-394,859,-416,-298,691,-224,-178,699,-224,-170,739,-224,-362,739,-224,-362,347,-224,-306,347,-224,-298,691,-224,-362,739,-224,-106,387,68,-90,395,68,-146,531,68,-106,347,68,-106,387,68,-146,531,68,-290,531,68,-362,347,68,-290,531,68,-298,683,68,-362,691,68,-362,347,68,-362,691,68,-298,683,68,-290,699,68,-362,859,68,-130,571,68,-18,563,68,22,603,68,22,859,68,-138,699,68,-362,859,68,-290,699,68,-138,699,68,22,859,68,-346,723,-416,-346,347,-416,22,3`
+`47,-416,22,723,-416,-10,627,-160,14,579,-160,14,643,-160,-10,627,-160,14,643,-160,-298,643,-160,-298,643,-160,-298,347,-160,-146,347,-160,-10,627,-160,-274,675,68,-274,555,68,-154,555,68,-154,675,68,-170,739,-224,-178,699,-224,-162,691,-224,22,739,-224,-170,675,-224,-274,675,-224,-274,651,-224,22,643,-224,22,739,-224,-162,691,-224,-170,675,-224,22,643,-224,22,459,68,22,467,68,-2,475,68,-2,395,68,-2,475,68,14,547,68,-10,539,68,-2,475,68,-10,539,68,-18,563,68,-90,395,68,-2,395,68,-2,475,68,-18,563`
+`,68,-130,571,68,-146,531,68],"vertslength":83,"polys":[0,3,4,7,8,11,12,14,15,19,20,23,24,27,28,32,33,36,37,40,41,43,44,46,47,50,51,54,55,58,59,62,63,66,67,70,71,73,74,76,77,82],"polyslength":21,"regions":[7,9,9,4,4,4,3,3,3,1,2,2,2,6,8,8,8,5,5,5,5],"neighbors":[[[0],[0],[0],[0]],[[0],[1,14],[0],[1,2]],[[0],[0],[1,1],[0]],[[0],[1,20],[1,4]],[[0],[1,3],[0],[1,5],[0]],[[0],[1,6],[0],[1,4]],[[1,5],[0],[1,8],[0]],[[1,20],[0],[0],[1,8],[0]],[[1,6],[0],[1,7],[0]],[[0],[0],[0],[0]],[[0],[0],[1,11]],[[1,1`
+`0],[0],[1,12]],[[0],[0],[0],[1,11]],[[0],[0],[0],[0]],[[1,1],[0],[1,16],[0]],[[0],[0],[0],[1,16]],[[1,14],[0],[1,15],[0]],[[0],[0],[1,20],[0]],[[0],[0],[1,19]],[[1,18],[0],[1,20]],[[0],[1,17],[1,19],[1,7],[0],[1,3]]]},"detail":{"verts":[-490,859,-416,-490,812.4545288085938,-408,-490,347,-408,-394,347,-408,-394,812.4545288085938,-408,-394,859,-416,-298,691,-224,-178,699,-224,-170,739,-224,-362,739,-224,-362,347,-224,-306,347,-224,-298,691,-224,-362,739,-224,-106,387,68,-90,395,68,-146,531,68,-106`
+`,347,68,-106,387,68,-146,531,68,-290,531,68,-362,347,68,-290,531,68,-298,683,68,-362,691,68,-362,347,68,-362,691,68,-298,683,68,-290,699,68,-362,859,68,-130,571,68,-18,563,68,22,603,68,22,859,68,-138,699,68,-362,859,68,-290,699,68,-138,699,68,22,859,68,-346,723,-416,-346,347,-416,22,347,-416,22,723,-416,-10,627,-160,14,579,-160,14,643,-160,-10,627,-160,14,643,-160,-298,643,-160,-298,643,-160,-298,347,-160,-146,347,-160,-10,627,-160,-274,675,68,-274,555,68,-154,555,68,-154,675,68,-170,739,-224,-1`
+`78,699,-224,-162,691,-224,22,739,-224,-170,675,-224,-274,675,-224,-274,651,-224,22,643,-224,22,739,-224,-162,691,-224,-170,675,-224,22,643,-224,22,459,68,22,467,68,-2,475,68,-2,395,68,-2,475,68,14,547,68,-10,539,68,-2,475,68,-10,539,68,-18,563,68,-90,395,68,-2,395,68,-2,475,68,-18,563,68,-130,571,68,-146,531,68],"vertslength":85,"tris":[5,0,1,4,5,1,4,1,2,2,3,4,6,7,8,6,8,9,10,11,12,10,12,13,14,15,16,17,18,19,17,19,20,17,20,21,22,23,24,22,24,25,26,27,28,26,28,29,30,31,32,34,30,32,32,33,34,35,36,37`
+`,35,37,38,42,39,40,40,41,42,43,44,45,46,47,48,49,50,51,49,51,52,56,53,54,54,55,56,57,58,59,57,59,60,61,62,63,61,63,64,66,67,68,65,66,68,69,70,71,69,71,72,73,74,75,76,77,78,79,80,81,82,83,84,81,82,84,79,81,84],"trislength":43,"triTopoly":[0,0,0,0,1,1,2,2,3,4,4,4,5,5,6,6,7,7,7,8,8,9,9,10,11,12,12,13,13,14,14,15,15,16,16,17,17,18,19,20,20,20,20],"baseVert":[0,6,10,14,17,22,26,30,35,39,43,46,49,53,57,61,65,69,73,76,79],"vertsCount":[6,4,4,3,5,4,4,5,4,4,3,3,4,4,4,4,4,4,3,3,6],"baseTri":[0,4,6,8,9,12,`
+`14,16,19,21,23,24,25,27,29,31,33,35,37,38,39],"triCount":[4,2,2,1,3,2,2,3,2,2,1,1,2,2,2,2,2,2,1,1,4]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["6_6",{"tileId":"6_6","tx":6,"ty":6,"mesh":{"verts":[262,347,-416,166,379,-416,158,355,-416,150,387,-416,166,379,-416,262,347,-416,262,723,-416,22,347,-416,262,347,-416,158,355,-416,22,347,-416,158,355,-416,142,363,-416,22,347,-416,142,363,-416,150,387,-416,262,723,-416,22,723,-416,86,579,-224,86,587,-224,70,587,-224,46,531,-224,30,4`
+`59,-224,270,443,-224,270,507,-224,238,491,-224,238,443,-224,238,443,-224,238,491,-224,214,491,-216,214,411,-224,238,443,-224,214,491,-216,94,539,-224,270,347,-224,270,419,-224,214,411,-224,30,459,-224,46,531,-224,22,555,-224,214,491,-216,230,507,-224,94,539,-224,30,459,-224,22,555,-224,22,347,-224,38,419,-224,30,459,-224,22,347,-224,94,539,-224,38,419,-224,22,347,-224,270,347,-224,214,411,-224,174,691,-224,174,699,-224,166,715,-224,150,691,-224,150,651,-224,158,675,-224,150,691,-224,70,627,-224,`
+`102,611,-224,118,619,-224,70,627,-224,150,691,-224,166,715,-224,70,627,-224,166,715,-224,190,739,-224,22,739,-224,22,739,-224,22,571,-224,70,627,-224,22,595,68,38,579,68,70,627,68,70,627,68,94,619,68,206,755,68,22,595,68,70,627,68,206,755,68,22,859,68,206,755,68,366,859,68,22,859,68,310,347,68,310,371,68,278,363,68,254,379,68,262,395,68,206,411,68,206,411,68,214,483,81,86,523,68,30,347,68,254,379,68,206,411,68,30,347,68,278,363,68,30,347,68,310,347,68,278,363,68,30,491,68,46,483,68,86,595,68,374`
+`,755,68,374,763,68,350,771,68,286,771,68,294,731,68,350,771,68,366,787,68,366,811,68,286,771,68,294,731,68,286,771,68,174,667,68,214,483,81,278,347,148,214,483,81,222,347,148,278,347,148,174,667,68,86,523,68,214,483,81,86,587,-224,86,579,-224,102,571,-224,134,611,-224,118,619,-224,102,611,-224,198,691,-224,174,699,-224,174,691,-224,158,675,-224,150,651,-224,158,635,-224,198,691,-224,174,691,-224,230,507,-224,238,491,-224,270,507,-224,134,611,-224,102,571,-224,94,539,-224,158,635,-224,270,739,-22`
+`4,198,691,-224,158,635,-224,94,539,-224,230,507,-224,270,507,-224,534,771,-480,526,859,-480,494,859,-480,502,707,-480,446,707,-480,438,731,-480,326,731,-480,534,619,-480,534,771,-480,502,707,-480,502,627,-480,502,627,-480,502,707,-480,446,707,-480,502,627,-480,446,707,-480,326,731,-480,302,595,-474,302,347,-421,502,347,-421,534,723,-72,510,723,-72,494,699,-72,534,347,-72,462,699,-72,438,723,-72,398,707,-72,382,659,-72,350,715,-72,398,707,-72,438,723,-72,334,723,-72,350,715,-72,334,723,-72,310,69`
+`1,-72,342,675,-72,494,699,-72,462,699,-72,382,659,-72,302,347,-72,534,347,-72,342,675,-72,310,691,-72,302,347,-72,382,659,-72,342,675,-72,302,347,-72,358,691,126,382,691,128,382,747,148,358,755,147,374,763,68,374,755,68,382,739,68,390,779,68,374,763,68,382,739,68,534,739,68,534,859,68,486,859,68,366,811,68,366,787,68,390,779,68,486,859,68,534,859,-80,486,859,-80,486,771,-80,534,755,-80,494,763,-192,534,763,-192,534,859,-192,494,859,-192,526,347,-644,534,347,-644,534,603,-644],"vertslength":229,"`
+`polys":[0,2,3,6,7,9,10,12,13,17,18,22,23,26,27,29,30,33,34,36,37,39,40,42,43,45,46,48,49,53,54,57,58,63,64,66,67,70,71,73,74,76,77,79,80,83,84,86,87,89,90,92,93,96,97,100,101,103,104,106,107,111,112,115,116,120,121,123,124,126,127,132,133,135,136,140,141,143,144,147,148,153,154,157,158,160,161,164,165,167,168,173,174,177,178,181,182,185,186,189,190,194,195,197,198,200,201,204,205,207,208,213,214,217,218,221,222,225,226,228],"polyslength":60,"regions":[1,1,1,1,1,6,6,6,6,6,6,6,6,6,6,9,9,9,9,9,4,4,`
+`4,4,8,8,8,8,8,13,5,5,5,5,5,7,7,7,7,7,7,3,3,3,3,3,2,2,2,2,2,2,2,16,10,10,10,11,12,17],"neighbors":[[[1,1],[0],[1,2]],[[0],[1,0],[0],[1,4]],[[0],[1,0],[1,3]],[[1,2],[0],[1,4]],[[1,3],[0],[1,1],[0],[0]],[[1,35],[0],[0],[1,10],[0]],[[0],[1,38],[1,7],[0]],[[1,6],[0],[1,8]],[[0],[1,7],[1,11],[1,14]],[[0],[0],[1,14]],[[1,5],[0],[1,12]],[[0],[1,40],[1,8]],[[1,10],[0],[1,13]],[[0],[1,12],[1,14]],[[0],[1,13],[0],[1,9],[1,8]],[[1,36],[0],[1,17],[0]],[[1,37],[0],[1,17],[0],[1,35],[0]],[[1,16],[1,15],[1,18]]`
+`,[[1,17],[0],[0],[1,19]],[[0],[0],[1,18]],[[0],[0],[1,22]],[[0],[0],[1,22]],[[1,20],[1,21],[1,23],[0]],[[0],[0],[1,22]],[[0],[0],[1,28]],[[0],[0],[1,27]],[[0],[1,34],[0],[1,27]],[[1,25],[1,26],[1,28],[0]],[[0],[1,24],[1,27]],[[0],[0],[0]],[[1,54],[0],[1,31],[1,32],[0]],[[0],[1,56],[0],[1,30]],[[1,30],[0],[1,34],[1,33],[0]],[[0],[0],[1,32]],[[0],[1,26],[1,32]],[[1,5],[0],[1,39],[0],[1,16],[0]],[[0],[1,15],[1,37]],[[1,16],[0],[1,40],[1,36],[0]],[[0],[1,6],[1,40]],[[1,35],[0],[1,40],[0]],[[0],[1,37`
+`],[1,39],[1,11],[1,38],[0]],[[0],[0],[0],[1,43]],[[0],[0],[1,45]],[[0],[1,41],[1,44],[0]],[[1,43],[0],[1,45]],[[1,44],[1,42],[0],[0],[0],[0]],[[0],[0],[1,50],[0]],[[0],[1,48],[0],[1,50]],[[0],[1,47],[0],[1,49]],[[1,48],[0],[1,51],[0]],[[0],[1,47],[1,52],[0],[1,46]],[[1,49],[0],[1,52]],[[0],[1,51],[1,50]],[[0],[0],[0],[0]],[[1,30],[0],[1,55]],[[0],[1,54],[0],[0],[0],[1,56]],[[1,31],[0],[1,55],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[262,347,-416,166,379,-416,158`
+`,355,-416,150,387,-416,166,379,-416,262,347,-416,262,723,-416,22,347,-416,262,347,-416,158,355,-416,22,347,-416,158,355,-416,142,363,-416,22,347,-416,142,363,-416,150,387,-416,262,723,-416,22,723,-416,86,579,-224,86,587,-224,70,587,-224,46,531,-224,30,459,-224,270,443,-224,270,507,-224,238,491,-224,238,443,-224,238,443,-224,238,491,-224,214,491,-224,214,411,-224,238,443,-224,214,491,-224,94,539,-224,169,459,-224,184,443,-216,202,471,-216,270,347,-224,270,419,-224,214,411,-224,30,459,-224,46,531,`
+`-224,22,555,-224,214,491,-224,230,507,-224,94,539,-224,30,459,-224,22,555,-224,22,347,-224,38,419,-224,30,459,-224,22,347,-224,94,539,-224,38,419,-224,22,347,-224,270,347,-224,214,411,-224,184,443,-216,169,459,-224,106,455,-216,174,691,-224,174,699,-224,166,715,-224,150,691,-224,150,651,-224,158,675,-224,150,691,-224,118,665.4000244140625,-216,86,639.7999877929688,-216,70,627,-224,102,611,-224,118,619,-216,106,623,-224,70,627,-224,86,639.7999877929688,-216,150,691,-224,166,715,-224,134,685.66668`
+`70117188,-224,118,671,-216,86,641.6666870117188,-216,70,627,-224,86,641.6666870117188,-216,166,715,-224,190,739,-224,22,739,-224,82,663,-224,22,739,-224,22,571,-224,70,627,-224,22,595,68,38,579,68,70,627,68,70,627,68,94,619,68,206,755,68,22,595,68,70,627,68,206,755,68,22,859,68,206,755,68,366,859,68,22,859,68,310,347,68,310,371,68,278,363,68,254,379,68,262,395,68,206,411,68,206,411,68,214,483,68,86,523,68,30,347,68,254,379,68,206,411,68,30,347,68,278,363,68,30,347,68,310,347,68,278,363,68,30,491`
+`,68,46,483,68,86,595,68,374,755,68,374,763,68,350,771,68,286,771,68,294,731,68,350,771,68,366,787,68,366,811,68,286,771,68,294,731,68,286,771,68,174,667,68,209,506,68,214,483,76,268.8571472167969,366.4285583496094,148,278,347,148,278.941162109375,369.5882263183594,148,279.8823547363281,392.1764831542969,136,283.6470642089844,482.5294189453125,72,234,479,72,214,483,76,219.3333282470703,392.3333435058594,141,222,347,148,278,347,148,268.8571472167969,366.4285583496094,148,226,407,120,226,383,136,17`
+`4,667,68,86,523,68,192.6666717529297,489.6666564941406,68,214,483,76,209,506,68,86,587,-224,86,579,-224,102,571,-224,134,611,-224,118,619,-224,102,611,-224,198,691,-224,174,699,-224,174,691,-224,158,675,-224,150,651,-224,158,635,-224,184.6666717529297,672.3333129882812,-216,198,691,-224,174,691,-224,230,507,-224,238,491,-224,270,507,-224,134,611,-224,102,571,-224,94,539,-224,158,635,-224,270,739,-224,198,691,-224,184.6666717529297,672.3333129882812,-216,158,635,-224,94,539,-224,230,507,-224,270,`
+`507,-224,202,639,-216,534,771,-480,526,859,-480,494,859,-480,502,707,-480,446,707,-480,438,731,-480,326,731,-480,534,619,-480,534,771,-480,502,707,-480,502,627,-480,502,627,-480,502,707,-480,446,707,-480,502,627,-480,446,707,-480,326,731,-480,302,595,-478,302,347,-423,502,347,-423,534,723,-72,510,723,-72,494,699,-72,534,347,-72,462,699,-72,438,723,-72,398,707,-72,382,659,-72,350,715,-72,398,707,-72,438,723,-72,334,723,-72,350,715,-72,334,723,-72,310,691,-72,342,675,-72,494,699,-72,462,699,-72,38`
+`2,659,-72,302,347,-72,534,347,-72,342,675,-72,310,691,-72,302,347,-72,382,659,-72,342,675,-72,302,347,-72,358,691,138,382,691,136,382,747,148,358,755,147,358,733.6666870117188,150,374,763,68,374,755,68,382,739,68,390,779,68,374,763,68,382,739,68,534,739,68,534,859,68,486,859,68,366,811,68,366,787,68,390,779,68,486,859,68,534,859,-80,486,859,-80,486,771,-80,534,755,-80,494,763,-192,534,763,-192,534,859,-192,494,859,-192,526,347,-644,534,347,-644,534,603,-644],"vertslength":260,"tris":[0,1,2,3,4,5`
+`,3,5,6,7,8,9,10,11,12,13,14,15,17,13,15,15,16,17,18,19,20,18,20,21,18,21,22,25,26,23,23,24,25,27,28,29,35,30,31,32,33,34,31,32,36,32,34,36,34,35,36,31,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,55,56,57,55,57,58,54,55,58,58,52,59,52,53,59,53,54,59,58,54,59,60,61,62,60,62,63,64,65,66,68,69,70,64,66,67,64,67,71,68,70,72,70,71,72,71,67,72,68,67,72,79,73,74,75,76,77,75,77,78,78,79,74,74,75,78,83,84,85,81,82,85,83,82,85,84,80,85,81,80,85,86,87,88,89,90,91,92,93,94,95,96,97,95,97,98,99,100,101`
+`,102,103,104,105,106,107,108,109,110,108,110,111,112,113,114,112,114,115,116,117,118,119,120,121,122,123,124,124,125,126,122,124,126,127,128,129,127,129,130,136,137,138,136,138,139,131,132,133,133,134,140,131,133,140,140,134,141,134,135,141,136,135,141,136,139,141,140,139,141,144,145,146,142,143,147,146,142,147,143,144,148,144,146,148,146,147,148,143,147,148,151,152,153,150,151,153,149,150,153,154,155,156,157,158,159,159,154,156,156,157,159,160,161,162,166,167,168,166,168,163,163,164,165,163,165`
+`,166,169,170,171,172,173,174,172,174,175,180,181,182,179,180,182,179,182,183,182,176,183,176,177,183,179,178,183,177,178,183,184,185,186,184,186,187,188,189,190,193,194,191,191,192,193,195,196,197,198,199,200,198,200,201,201,202,203,198,201,203,204,205,206,204,206,207,208,209,210,208,210,211,215,212,213,213,214,215,216,217,218,216,218,219,220,221,222,224,220,222,222,223,224,225,226,227,228,229,230,233,234,235,235,231,232,232,233,235,236,237,238,239,240,241,242,243,244,244,239,241,241,242,244,245`
+`,246,247,245,247,248,249,250,251,249,251,252,256,253,254,254,255,256,257,258,259],"trislength":148,"triTopoly":[0,1,1,2,3,4,4,4,5,5,5,6,6,7,8,8,8,8,8,8,9,10,11,12,13,14,14,14,14,14,14,14,15,15,16,16,16,16,16,16,16,16,17,17,17,17,17,18,18,18,18,18,19,20,21,22,22,23,24,25,26,26,27,27,28,29,30,30,30,31,31,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,34,34,34,35,35,35,35,36,37,37,37,37,38,39,39,40,40,40,40,40,40,40,41,41,42,43,43,44,45,45,45,45,46,46,47,47,48,48,49,49,50,50,50,51,52,53,53,53,5`
+`4,55,55,55,55,56,56,57,57,58,58,59],"baseVert":[0,3,7,10,13,18,23,27,30,37,40,43,46,49,52,60,64,73,80,86,89,92,95,99,102,105,108,112,116,119,122,127,131,142,149,154,160,163,169,172,176,184,188,191,195,198,204,208,212,216,220,225,228,231,236,239,245,249,253,257],"vertsCount":[3,4,3,3,5,5,4,3,7,3,3,3,3,3,8,4,9,7,6,3,3,3,4,3,3,3,4,4,3,3,5,4,11,7,5,6,3,6,3,4,8,4,3,4,3,6,4,4,4,4,5,3,3,5,3,6,4,4,4,3],"baseTri":[0,1,3,4,5,8,11,13,14,20,21,22,23,24,25,32,34,42,47,52,53,54,55,57,58,59,60,62,64,65,66,69,7`
+`1,81,88,91,95,96,100,101,103,110,112,113,115,116,120,122,124,126,128,131,132,133,136,137,141,143,145,147],"triCount":[1,2,1,1,3,3,2,1,6,1,1,1,1,1,7,2,8,5,5,1,1,1,2,1,1,1,2,2,1,1,3,2,10,7,3,4,1,4,1,2,7,2,1,2,1,4,2,2,2,2,3,1,1,3,1,4,2,2,2,1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_6",{"tileId":"7_6","tx":7,"ty":6,"mesh":{"verts":[534,603,-644,534,347,-644,766,347,-644,766,603,-644,758,627,-480,790,627,-480,798,707,-480,774,739,-480,774,739,-480,790,859,-480,622,859,-480,`
+`622,771,-480,758,627,-480,758,627,-480,622,771,-480,534,763,-480,534,347,-562,758,347,-562,958,723,-72,846,723,-72,830,699,-72,950,643,-72,950,643,-72,830,699,-72,798,699,-72,982,619,-72,950,643,-72,798,699,-72,534,723,-72,534,347,-72,982,347,-72,798,699,-72,782,723,-72,534,723,-72,990,731,68,1006,755,68,902,827,68,806,859,68,534,859,68,534,739,68,774,755,-80,798,771,-80,798,859,-80,534,859,-80,534,755,-80,534,859,-192,534,763,-192,782,763,-192,790,859,-192,542,851,-404,542,779,-404,606,779,-404`
+`,606,851,-404,550,835,-478,550,787,-478,598,787,-478,598,835,-478,838,707,-480,798,707,-480,790,627,-480,966,731,-480,846,731,-480,838,707,-480,966,731,-480,838,707,-480,790,627,-480,790,347,-421,982,347,-421,1006,843,68,1046,811,68,1046,859,68,926,859,68,982,819,68,1006,843,68,1046,859,68,926,859,68,1006,843,68,1046,731,68,1006,755,68,990,731,68,990,347,68,1046,347,68,1046,763,68,1046,787,68,1006,803,68,1014,675,-256,1014,347,-256,1046,347,-256,1046,675,-256,1014,755,-160,1030,803,-160,1014,803`
+`,-160,1022,667,-416,1022,347,-416,1046,347,-416,1046,667,-416,1022,699,-416,1046,699,-416,1046,795,-416,1022,795,-416],"vertslength":100,"polys":[0,3,4,7,8,12,13,17,18,21,22,24,25,30,31,33,34,39,40,44,45,48,49,52,53,56,57,59,60,62,63,67,68,70,71,73,74,76,77,81,82,84,85,88,89,91,92,95,96,99],"polyslength":25,"regions":[3,2,2,2,1,1,1,1,5,6,7,8,10,4,4,4,11,11,11,9,13,12,14,15,16],"neighbors":[[[0],[0],[0],[0]],[[0],[1,13],[0],[1,2]],[[0],[0],[0],[1,3],[1,1]],[[1,2],[0],[0],[0],[0]],[[0],[0],[1,5],[`
+`0]],[[1,4],[0],[1,6]],[[0],[1,5],[1,7],[0],[0],[0]],[[0],[0],[1,6]],[[1,19],[0],[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,1],[1,15]],[[0],[0],[1,15]],[[1,14],[1,13],[0],[0],[0]],[[0],[0],[1,18]],[[0],[0],[1,18]],[[0],[1,17],[1,16]],[[0],[1,8],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[534,603,-644,534,347,-644,766,347,-644,766,603,-644,758,627,-480,790,627,-480,798,`
+`707,-480,774,739,-480,774,739,-480,790,859,-480,622,859,-480,622,771,-480,758,627,-480,758,627,-480,622,771,-480,534,763,-480,534,601.2222290039062,-479,534,347,-559,758,347,-559,758,580.3333129882812,-484,958,723,-72,846,723,-72,830,699,-72,950,643,-72,950,643,-72,830,699,-72,798,699,-72,982,619,-72,950,643,-72,798,699,-72,534,723,-72,534,347,-72,982,347,-72,798,699,-72,782,723,-72,534,723,-72,990,731,68,1006,755,68,902,827,68,806,859,68,534,859,68,534,739,68,774,755,-80,798,771,-80,798,859,-80`
+`,534,859,-80,534,755,-80,534,859,-192,534,763,-192,759.4545288085938,763,-192,782,763,-184,790,859,-192,542,851,-404,542,779,-404,606,779,-404,606,851,-404,550,835,-478,550,787,-478,598,787,-478,598,835,-478,838,707,-480,798,707,-480,790,627,-480,966,731,-480,846,731,-480,838,707,-480,966,731,-480,838,707,-480,790,627,-480,790,347,-423,982,347,-423,971.6470336914062,595.4705810546875,-478,1006,843,68,1046,811,68,1046,859,68,926,859,68,982,819,68,1006,843,68,1046,859,68,926,859,68,1006,843,68,104`
+`6,731,68,1006,755,68,990,731,68,990,347,68,1046,347,68,1046,763,68,1046,787,68,1006,803,68,1014,675,-256,1014,347,-256,1046,347,-256,1046,393.8571472167969,-248,1046,440.71429443359375,-256,1046,487.5714416503906,-248,1046,534.4285888671875,-256,1046,581.2857055664062,-248,1046,675,-256,1014,755,-160,1030,803,-160,1014,803,-160,1022,667,-416,1022,347,-416,1046,347,-416,1046,667,-416,1022,699,-416,1046,699,-416,1046,795,-416,1022,795,-416],"vertslength":109,"tris":[3,0,1,1,2,3,4,5,6,4,6,7,9,10,11`
+`,8,9,11,8,11,12,14,15,16,19,13,14,19,14,16,17,18,19,16,17,19,20,21,22,20,22,23,24,25,26,27,28,29,32,27,29,29,30,31,29,31,32,33,34,35,36,37,38,36,38,39,39,40,41,36,39,41,42,43,44,45,46,42,42,44,45,49,50,51,47,48,49,47,49,51,55,52,53,53,54,55,59,56,57,57,58,59,60,61,62,63,64,65,66,67,68,71,66,68,69,70,71,68,69,71,72,73,74,75,76,77,78,79,80,81,82,83,85,81,83,83,84,85,86,87,88,90,91,92,90,92,93,96,97,89,90,93,94,95,96,89,90,94,95,89,90,95,98,99,100,104,101,102,102,103,104,108,105,106,106,107,108],"t`
+`rislength":59,"triTopoly":[0,0,1,1,2,2,2,3,3,3,3,3,4,4,5,6,6,6,6,7,8,8,8,8,9,9,9,10,10,10,11,11,12,12,13,14,15,15,15,15,16,17,18,19,19,19,20,21,21,21,21,21,21,21,22,23,23,24,24],"baseVert":[0,4,8,13,20,24,27,33,36,42,47,52,56,60,63,66,72,75,78,81,86,89,98,101,105],"vertsCount":[4,4,5,7,4,3,6,3,6,5,5,4,4,3,3,6,3,3,3,5,3,9,3,4,4],"baseTri":[0,2,4,7,12,14,15,19,20,24,27,30,32,34,35,36,40,41,42,43,46,47,54,55,57],"triCount":[2,2,3,5,2,1,4,1,4,3,3,2,2,1,1,4,1,1,1,3,1,7,1,2,2]},"links":{"poly":[],"cos`
+`t":[],"type":[],"pos":[],"length":0}}],["8_6",{"tileId":"8_6","tx":8,"ty":6,"mesh":{"verts":[1246,451,-416,1238,491,-416,1222,499,-416,1222,443,-416,1182,699,-416,1222,699,-416,1222,731,-416,1094,739,-416,1046,795,-416,1046,699,-416,1070,691,-416,1094,739,-416,1102,755,-408,1174,675,-416,1182,699,-416,1094,739,-416,1070,691,-416,1102,755,-408,1222,755,-336,1222,795,-336,1046,795,-416,1174,675,-416,1070,691,-416,1046,667,-416,1222,667,-416,1046,347,-416,1222,347,-416,1222,443,-416,1222,499,-416,1`
+`222,667,-416,1046,667,-416,1454,507,-256,1462,491,-256,1502,555,-256,1486,563,-256,1262,363,-256,1270,387,-256,1238,443,-256,1046,347,-256,1454,507,-256,1486,563,-256,1486,627,-256,1238,627,-256,1246,507,-256,1238,627,-256,1230,675,-256,1046,675,-256,1046,347,-256,1230,483,-256,1246,507,-256,1046,347,-256,1286,347,-256,1262,363,-256,1046,347,-256,1238,443,-256,1230,483,-256,1078,475,68,1046,483,68,1046,347,68,1182,571,68,1102,683,68,1046,723,68,1046,507,68,1086,507,68,1078,475,68,1046,347,68,125`
+`4,347,68,1230,467,68,1182,571,68,1086,507,68,1046,755,68,1094,731,68,1046,779,68,1046,811,68,1062,811,68,1070,843,68,1046,859,68,1118,811,68,1086,795,68,1134,739,68,1462,515,68,1470,499,68,1518,579,68,1502,587,68,1502,739,68,1558,747,68,1558,859,68,1134,739,68,1118,707,68,1206,611,68,1118,811,68,1134,739,68,1206,611,68,1502,739,68,1558,859,68,1462,515,68,1502,587,68,1502,739,68,1206,611,68,1246,515,68,1558,859,68,1046,859,68,1070,843,68,1118,811,68,1102,691,-160,1118,699,-160,1102,707,-160,1254,`
+`499,-416,1238,491,-416,1246,451,-416,1254,499,-416,1246,451,-416,1254,379,-416,1254,499,-416,1254,379,-416,1478,379,-416,1478,619,-416,1254,619,-416,1558,555,-256,1502,555,-256,1462,491,-256,1462,347,-256,1558,347,-256,1558,579,68,1518,579,68,1470,499,68,1470,347,68,1558,347,68,1510,547,-416,1510,347,-416,1558,347,-416,1558,547,-416,1526,595,148,1558,595,148,1558,731,148,1526,731,148,1534,603,68,1558,603,68,1558,723,68,1534,723,68],"vertslength":140,"polys":[0,3,4,7,8,12,13,16,17,20,21,24,25,30,`
+`31,34,35,38,39,43,44,49,50,52,53,55,56,58,59,63,64,69,70,72,73,76,77,79,80,83,84,86,87,89,90,94,95,99,100,103,104,106,107,109,110,112,113,117,118,122,123,127,128,131,132,135,136,139],"polyslength":34,"regions":[4,4,4,4,4,4,4,2,2,2,2,2,2,5,5,5,11,1,1,1,1,1,1,1,1,14,3,3,3,7,6,9,10,15],"neighbors":[[[1,26],[0],[1,6],[0]],[[0],[0],[0],[1,3]],[[0],[0],[1,3],[0],[1,4]],[[0],[1,1],[1,2],[1,5]],[[0],[0],[0],[1,2]],[[1,3],[0],[1,6],[0]],[[0],[0],[1,0],[0],[1,5],[0]],[[0],[1,29],[0],[1,9]],[[0],[0],[1,12]`
+`,[1,11]],[[1,7],[0],[0],[1,10],[0]],[[0],[0],[0],[1,12],[0],[1,9]],[[0],[0],[1,8]],[[1,8],[0],[1,10]],[[0],[0],[1,15]],[[0],[0],[0],[0],[1,15]],[[1,13],[0],[0],[0],[1,14],[0]],[[0],[0],[0]],[[0],[0],[1,24],[0]],[[0],[0],[1,22]],[[0],[1,30],[0],[1,23]],[[0],[0],[1,22]],[[0],[0],[1,22]],[[1,18],[1,21],[1,23],[1,20],[1,24]],[[1,19],[0],[1,22],[0],[0]],[[0],[1,17],[0],[1,22]],[[0],[0],[0]],[[0],[1,0],[1,27]],[[1,26],[0],[1,28]],[[1,27],[0],[0],[0],[0]],[[0],[1,7],[0],[0],[0]],[[0],[1,19],[0],[0],[0]`
+`],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1246,451,-416,1238,491,-416,1222,499,-416,1222,443,-416,1182,699,-416,1222,699,-416,1222,731,-416,1200.6666259765625,732.3333129882812,-344,1094,739,-408,1111.5999755859375,731,-400,1129.199951171875,723,-416,1202,711,-416,1178,711,-416,1046,795,-416,1046,699,-416,1070,691,-416,1086,723,-416,1094,739,-408,1102,755,-408,1083.3333740234375,768.3333129882812,-416,1174,675,-416,1182,699,-416,1129.199951171875,723,-416,1111.`
+`5999755859375,731,-400,1094,739,-408,1086,723,-416,1070,691,-416,1102,755,-408,1142,755,-376,1222,755,-336,1222,795,-336,1200,795,-344,1090,795,-416,1046,795,-416,1083.3333740234375,768.3333129882812,-416,1130,767,-392,1174,675,-416,1070,691,-416,1046,667,-416,1222,667,-416,1046,347,-416,1222,347,-416,1222,443,-416,1222,499,-416,1222,667,-416,1046,667,-416,1454,507,-256,1462,491,-256,1502,555,-256,1486,563,-256,1262,363,-256,1270,387,-256,1238,443,-256,1195.3333740234375,421.6666564941406,-256,1`
+`174,411,-248,1152.6666259765625,400.3333435058594,-256,1131.3333740234375,389.6666564941406,-248,1046,347,-256,1178,383,-248,1454,507,-256,1486,563,-256,1486,627,-256,1238,627,-256,1246,507,-256,1238,627,-256,1230,675,-256,1046,675,-256,1046,628.1428833007812,-256,1046,604.7142944335938,-248,1046,557.8571166992188,-256,1046,511,-248,1046,464.1428527832031,-256,1046,417.28570556640625,-248,1046,347,-256,1082.800048828125,374.20001220703125,-256,1101.199951171875,387.79998779296875,-248,1138,415,-`
+`256,1230,483,-256,1246,507,-256,1106,407,-248,1178,479,-248,1046,347,-256,1286,347,-256,1262,363,-256,1046,347,-256,1088.6666259765625,368.3333435058594,-256,1110,379,-248,1152.6666259765625,400.3333435058594,-256,1174,411,-248,1195.3333740234375,421.6666564941406,-256,1238,443,-256,1230,483,-256,1138,415,-256,1119.5999755859375,401.3999938964844,-248,1154,407,-256,1078,475,68,1046,483,68,1046,347,68,1182,571,68,1102,683,68,1046,723,68,1046,507,68,1086,507,68,1078,475,68,1046,347,68,1254,347,68,`
+`1230,467,68,1182,571,68,1086,507,68,1046,755,68,1094,731,68,1046,779,68,1046,811,68,1062,811,68,1070,843,68,1046,859,68,1118,811,68,1086,795,68,1134,739,68,1462,515,68,1470,499,68,1518,579,68,1502,587,68,1502,739,68,1558,747,68,1558,859,68,1134,739,68,1118,707,68,1206,611,68,1118,811,68,1134,739,68,1206,611,68,1502,739,68,1558,859,68,1462,515,68,1502,587,68,1502,739,68,1206,611,68,1246,515,68,1558,859,68,1046,859,68,1070,843,68,1118,811,68,1102,691,-160,1118,699,-160,1102,707,-160,1254,499,-416,`
+`1238,491,-416,1246,451,-416,1254,499,-416,1246,451,-416,1254,379,-416,1254,499,-416,1254,379,-416,1478,379,-416,1478,619,-416,1254,619,-416,1558,555,-256,1502,555,-256,1462,491,-256,1462,347,-256,1558,347,-256,1558,579,68,1518,579,68,1470,499,68,1470,347,68,1558,347,68,1510,547,-416,1510,347,-416,1558,347,-416,1558,547,-416,1526,595,148,1558,595,148,1558,731,148,1526,731,148,1534,603,68,1558,603,68,1558,723,68,1534,723,68],"vertslength":179,"tris":[0,1,2,0,2,3,8,9,10,7,8,10,4,5,11,5,6,11,7,6,11,`
+`10,4,12,7,10,12,4,11,12,7,11,12,17,18,19,16,17,19,14,15,16,14,16,19,13,14,19,23,24,25,22,23,25,22,25,26,20,21,22,20,22,26,29,30,31,32,33,34,32,34,27,28,29,31,28,31,35,31,32,35,32,27,35,28,27,35,36,37,38,36,38,39,40,41,42,40,42,43,43,44,45,40,43,45,48,49,46,46,47,48,57,50,58,50,51,58,52,51,58,55,54,58,52,53,58,54,53,58,55,56,58,57,56,58,59,60,61,62,63,59,59,61,62,72,73,74,65,66,67,65,67,68,76,71,79,71,72,79,74,72,79,74,75,79,76,75,79,68,69,80,69,70,80,76,71,80,70,71,80,76,77,80,78,77,80,68,65,80,`
+`78,64,80,65,64,80,81,82,83,84,85,86,89,90,91,88,89,91,93,84,86,88,91,92,88,92,94,92,93,94,86,93,94,86,87,94,88,87,94,95,96,97,101,102,98,98,99,100,98,100,101,107,108,103,106,107,103,105,106,103,103,104,105,109,110,111,112,113,114,112,114,115,116,117,118,121,122,119,119,120,121,123,124,125,126,127,128,129,130,131,131,132,133,129,131,133,134,135,136,137,138,134,134,136,137,141,142,139,139,140,141,143,144,145,146,147,148,149,150,151,152,153,154,155,156,152,152,154,155,157,158,159,159,160,161,157,15`
+`9,161,162,163,164,164,165,166,162,164,166,170,167,168,168,169,170,174,171,172,172,173,174,178,175,176,176,177,178],"trislength":118,"triTopoly":[0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,4,4,4,4,4,5,5,6,6,6,6,7,7,8,8,8,8,8,8,8,8,9,9,9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,11,12,12,12,12,12,12,12,12,12,12,13,14,14,14,15,15,15,15,16,17,17,18,19,19,20,21,22,22,22,23,23,23,24,24,25,26,27,28,28,28,29,29,29,30,30,30,31,31,32,32,33,33],"baseVert":[0,4,13,20,27,36,40,46,50,59,64,81,84`
+`,95,98,103,109,112,116,119,123,126,129,134,139,143,146,149,152,157,162,167,171,175],"vertsCount":[4,9,7,7,9,4,6,4,9,5,17,3,11,3,5,6,3,4,3,4,3,3,5,5,4,3,3,3,5,5,5,4,4,4],"baseTri":[0,2,11,16,21,29,31,35,37,45,48,65,66,76,77,80,84,85,87,88,90,91,92,95,98,100,101,102,103,106,109,112,114,116],"triCount":[2,9,5,5,8,2,4,2,8,3,17,1,10,1,3,4,1,2,1,2,1,1,3,3,2,1,1,1,3,3,3,2,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["9_6",{"tileId":"9_6","tx":9,"ty":6,"mesh":{"verts":[1558,547,-`
+`416,1558,347,-416,1638,347,-416,1638,547,-416,1558,555,-256,1558,347,-256,1646,347,-256,1646,555,-256,1646,731,68,1606,731,68,1606,587,68,1646,347,68,1606,587,68,1558,579,68,1558,347,68,1646,347,68,1558,731,148,1558,595,148,1590,595,148,1590,731,148,1558,723,68,1558,603,68,1574,603,68,1574,723,68,1598,747,68,1606,731,68,1646,731,68,1646,859,68,1558,859,68,1558,859,68,1558,747,68,1598,747,68,1678,811,-412,1678,347,-412,2070,347,-408,2070,811,-412,1678,827,-348,2070,827,-348,2070,859,-352,1678,859`
+`,-352],"vertslength":40,"polys":[0,3,4,7,8,11,12,15,16,19,20,23,24,28,29,31,32,35,36,39],"polyslength":10,"regions":[5,2,3,3,6,8,4,4,1,7],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,6],[0],[1,3],[0]],[[0],[0],[0],[1,2]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,2],[0],[0],[1,7]],[[0],[0],[1,6]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1558,547,-416,1558,347,-416,1638,347,-416,1638,547,-416,1558,555,-256,1558,347,-256,1646,347,-256,1646,555,-256,1646,731,68,1606,731,68,`
+`1606,587,68,1646,347,68,1606,587,68,1558,579,68,1558,347,68,1646,347,68,1558,731,148,1558,595,148,1590,595,148,1590,731,148,1558,723,68,1558,603,68,1574,603,68,1574,723,68,1598,747,68,1606,731,68,1646,731,68,1646,859,68,1558,859,68,1558,859,68,1558,747,68,1598,747,68,1678,811,-412,1678,347,-412,2070,347,-408,2070,811,-412,1678,827,-348,2070,827,-348,2070,859,-352,1678,859,-352],"vertslength":40,"tris":[3,0,1,1,2,3,7,4,5,5,6,7,8,9,10,8,10,11,12,13,14,12,14,15,19,16,17,17,18,19,23,20,21,21,22,23,2`
+`4,25,26,24,26,27,24,27,28,29,30,31,35,32,33,33,34,35,39,36,37,37,38,39],"trislength":20,"triTopoly":[0,0,1,1,2,2,3,3,4,4,5,5,6,6,6,7,8,8,9,9],"baseVert":[0,4,8,12,16,20,24,29,32,36],"vertsCount":[4,4,4,4,4,4,5,3,4,4],"baseTri":[0,2,4,6,8,10,12,15,16,18],"triCount":[2,2,2,2,2,2,3,1,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["10_6",{"tileId":"10_6","tx":10,"ty":6,"mesh":{"verts":[2102,363,-408,2070,371,-406,2070,347,-408,2102,491,-408,2070,499,-406,2070,395,-408,2110,395,`
+`-408,2110,515,-408,2102,491,-408,2110,395,-408,2302,347,-408,2278,563,-408,2270,563,-408,2102,363,-408,2070,347,-408,2302,347,-408,2110,395,-408,2102,363,-408,2302,347,-408,2070,659,-406,2070,523,-408,2102,531,-408,2110,659,-408,2070,659,-406,2102,531,-408,2110,515,-408,2254,571,-408,2270,611,-408,2110,515,-408,2270,563,-408,2254,571,-408,2070,811,-412,2070,683,-408,2102,691,-408,2110,659,-408,2270,611,-408,2286,611,-408,2102,691,-408,2294,811,-412,2070,811,-412,2102,691,-408,2286,611,-408,2070,`
+`859,-352,2070,827,-348,2582,827,-348,2582,859,-352,2078,387,-208,2078,347,-208,2102,347,-208,2102,387,-208,2078,571,-208,2078,499,-208,2102,499,-208,2102,571,-208,2078,675,-208,2078,603,-208,2102,603,-208,2102,675,-208,2262,387,-110,2262,347,-110,2278,347,-111,2278,387,-111,2262,571,-110,2262,499,-110,2278,499,-111,2278,571,-111,2278,611,-111,2278,675,-111,2262,675,-110,2262,603,-110,2582,675,-412,2558,675,-408,2534,651,-408,2526,531,-408,2558,523,-408,2582,347,-412,2558,395,-408,2550,363,-408,2`
+`582,347,-412,2550,363,-408,2518,371,-406,2302,347,-408,2518,499,-406,2526,531,-408,2534,651,-408,2294,595,-408,2526,403,-408,2518,499,-406,2294,595,-408,2278,563,-408,2302,347,-408,2518,371,-406,2582,347,-412,2582,675,-412,2558,523,-408,2582,347,-412,2558,523,-408,2550,491,-408,2558,395,-408,2526,403,-408,2558,395,-408,2550,491,-408,2518,499,-406,2550,691,-408,2558,675,-408,2582,675,-412,2582,811,-412,2518,683,-408,2550,691,-408,2582,811,-412,2294,811,-412,2286,611,-408,2294,595,-408,2518,683,-4`
+`08,2582,811,-412,2294,595,-408,2534,651,-408,2518,683,-408,2526,387,-208,2526,347,-208,2550,347,-208,2550,387,-208,2526,571,-208,2526,499,-208,2550,499,-208,2550,571,-208,2526,675,-208,2526,603,-208,2550,603,-208,2550,675,-208],"vertslength":130,"polys":[0,2,3,6,7,12,13,15,16,18,19,21,22,27,28,30,31,33,34,37,38,41,42,45,46,49,50,53,54,57,58,61,62,65,66,69,70,74,75,77,78,81,82,85,86,91,92,94,95,98,99,102,103,106,107,109,110,114,115,117,118,121,122,125,126,129],"polyslength":33,"regions":[3,3,3,3,`
+`3,5,5,5,4,4,4,6,7,10,11,16,17,18,1,1,1,1,1,1,1,1,2,2,2,2,19,22,23],"neighbors":[[[0],[0],[1,3]],[[0],[0],[0],[1,2]],[[0],[1,1],[1,4],[1,22],[0],[1,7]],[[1,0],[0],[1,4]],[[0],[1,3],[1,2]],[[0],[0],[1,6]],[[0],[1,5],[0],[1,7],[0],[1,9]],[[1,2],[0],[1,6]],[[0],[0],[1,10]],[[1,6],[0],[1,10],[0]],[[0],[1,8],[1,9],[1,28]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[1,26],[0],[1,21],[0],[1,23]],[[1,24],[0],[1,20]],[[1,1`
+`9],[0],[1,22],[0]],[[0],[1,18],[1,29],[1,22]],[[1,25],[1,21],[0],[1,2],[1,20],[0]],[[0],[1,18],[1,24]],[[1,23],[0],[1,25],[1,19]],[[0],[1,24],[0],[1,22]],[[0],[1,18],[0],[1,27]],[[0],[1,26],[1,28]],[[1,10],[0],[1,29],[1,27],[0]],[[1,21],[0],[1,28]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2102,363,-408,2070,371,-406,2070,347,-408,2102,491,-408,2070,499,-406,2070,395,-408,2110,395,-412,2110,515,-412,2102,491,-408,2110,395,-412,2302,347,-408,2278,563,-408,2270,563`
+`,-408,2102,363,-408,2070,347,-408,2302,347,-408,2110,395,-412,2102,363,-408,2302,347,-408,2070,659,-406,2070,523,-408,2102,531,-408,2110,659,-412,2070,659,-406,2102,531,-408,2110,515,-412,2254,571,-408,2270,611,-408,2110,515,-412,2270,563,-408,2254,571,-408,2070,811,-412,2070,683,-408,2102,691,-408,2110,659,-412,2270,611,-408,2286,611,-408,2102,691,-408,2294,811,-412,2070,811,-412,2102,691,-408,2286,611,-408,2070,859,-352,2070,827,-348,2582,827,-348,2582,859,-352,2078,387,-209,2078,347,-209,2102`
+`,347,-208,2102,387,-208,2078,571,-209,2078,499,-209,2102,499,-208,2102,571,-208,2078,675,-209,2078,603,-209,2102,603,-208,2102,675,-208,2262,387,-111,2262,347,-111,2278,347,-111,2278,387,-111,2262,571,-111,2262,499,-111,2278,499,-111,2278,571,-111,2278,611,-111,2278,675,-111,2262,675,-111,2262,603,-111,2582,675,-412,2558,675,-412,2534,651,-408,2526,531,-408,2558,523,-412,2582,347,-412,2558,395,-412,2550,363,-408,2582,347,-412,2550,363,-408,2518,371,-408,2302,347,-412,2518,499,-408,2526,531,-408,`
+`2534,651,-408,2294,595,-412,2526,403,-408,2518,499,-408,2294,595,-412,2278,563,-408,2302,347,-412,2518,371,-408,2582,347,-412,2582,675,-412,2558,523,-412,2582,347,-412,2558,523,-412,2550,491,-408,2558,395,-412,2526,403,-408,2558,395,-412,2550,491,-408,2518,499,-408,2550,691,-408,2558,675,-412,2582,675,-412,2582,811,-412,2518,683,-408,2550,691,-408,2582,811,-412,2294,811,-412,2286,611,-408,2294,595,-412,2518,683,-408,2582,811,-412,2294,595,-412,2534,651,-408,2518,683,-408,2526,387,-209,2526,347,-`
+`209,2550,347,-208,2550,387,-208,2526,571,-209,2526,499,-209,2550,499,-208,2550,571,-208,2526,675,-209,2526,603,-209,2550,603,-208,2550,675,-208],"vertslength":130,"tris":[0,1,2,5,6,3,3,4,5,7,8,9,11,12,7,11,7,9,9,10,11,13,14,15,16,17,18,19,20,21,23,24,25,22,23,25,26,27,22,22,25,26,28,29,30,31,32,33,34,35,36,34,36,37,38,39,40,38,40,41,45,42,43,43,44,45,49,46,47,47,48,49,53,50,51,51,52,53,57,54,55,55,56,57,61,58,59,59,60,61,65,62,63,63,64,65,66,67,68,66,68,69,70,71,72,72,73,74,70,72,74,75,76,77,78,`
+`79,80,78,80,81,82,83,84,82,84,85,91,86,87,88,89,90,90,91,87,87,88,90,92,93,94,96,97,98,95,96,98,99,100,101,99,101,102,103,104,105,103,105,106,107,108,109,110,111,112,113,114,110,110,112,113,115,116,117,121,118,119,119,120,121,125,122,123,123,124,125,129,126,127,127,128,129],"trislength":64,"triTopoly":[0,1,1,2,2,2,2,3,4,5,6,6,6,6,7,8,9,9,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,18,19,20,20,21,21,22,22,22,22,23,24,24,25,25,26,26,27,28,28,28,29,30,30,31,31,32,32],"baseVert":[0,3,7,13,`
+`16,19,22,28,31,34,38,42,46,50,54,58,62,66,70,75,78,82,86,92,95,99,103,107,110,115,118,122,126],"vertsCount":[3,4,6,3,3,3,6,3,3,4,4,4,4,4,4,4,4,4,5,3,4,4,6,3,4,4,4,3,5,3,4,4,4],"baseTri":[0,1,3,7,8,9,10,14,15,16,18,20,22,24,26,28,30,32,34,37,38,40,42,46,47,49,51,53,54,57,58,60,62],"triCount":[1,2,4,1,1,1,4,1,1,2,2,2,2,2,2,2,2,2,3,1,2,2,4,1,2,2,2,1,3,1,2,2,2]},"links":{"poly":[13,14,31,32],"cost":[1536,1536],"type":[1,1],"pos":[2102,571,-208,2102,603,-208,2550,571,-208,2550,603,-208],"length":2}}]`
+`,["11_6",{"tileId":"11_6","tx":11,"ty":6,"mesh":{"verts":[3094,611,-412,3006,595,-408,2998,563,-408,3094,347,-412,2694,563,-408,2686,603,-408,2582,619,-412,2582,347,-412,2966,563,-408,2958,595,-408,2734,595,-408,2726,563,-408,3094,347,-412,2998,563,-408,2966,563,-408,2726,563,-408,2694,563,-408,2582,347,-412,2966,611,-408,3006,595,-408,3094,611,-412,2582,619,-412,2686,603,-408,2726,611,-408,2726,611,-408,2734,595,-408,2958,595,-408,2966,611,-408,2966,611,-408,3094,611,-412,3094,811,-412,2582,811`
+`,-412,2582,619,-412,2726,611,-408,2582,859,-352,2582,827,-348,3094,827,-348,3094,859,-352,2694,419,-110,2702,347,-114,2718,347,-114,2718,427,-114,2694,707,-110,2694,467,-110,2726,467,-110,2726,707,-110,2974,387,-110,2974,347,-110,2990,347,-111,2990,387,-111,2974,571,-110,2974,499,-110,2990,499,-111,2990,571,-111,2974,675,-110,2974,611,-110,2990,611,-111,2990,675,-111],"vertslength":58,"polys":[0,3,4,7,8,11,12,17,18,20,21,23,24,27,28,33,34,37,38,41,42,45,46,49,50,53,54,57],"polyslength":14,"regio`
+`ns":[1,1,1,1,2,2,2,2,3,4,5,6,7,8],"neighbors":[[[1,4],[0],[1,3],[0]],[[0],[1,5],[0],[1,3]],[[0],[1,6],[0],[1,3]],[[1,0],[0],[1,2],[0],[1,1],[0]],[[0],[1,0],[1,7]],[[1,1],[0],[1,7]],[[0],[1,2],[0],[1,7]],[[1,4],[0],[0],[0],[1,5],[1,6]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[3094,611,-412,3006,595,-412,2998,563,-408,3094,347,-412,2694,563,-408,2686,603,-412,2582,619,-412,2582,347,-412,2966,563,-408,2958,595,-`
+`408,2734,595,-412,2726,563,-408,3094,347,-412,2998,563,-408,2966,563,-408,2726,563,-408,2694,563,-408,2582,347,-412,2966,611,-408,3006,595,-412,3094,611,-412,2582,619,-412,2686,603,-408,2726,611,-408,2726,611,-408,2734,595,-412,2958,595,-408,2966,611,-408,2966,611,-408,3094,611,-412,3094,811,-412,2582,811,-412,2582,619,-412,2726,611,-408,2582,859,-352,2582,827,-348,3094,827,-348,3094,859,-352,2694,419,-114,2702,347,-114,2718,347,-114,2718,427,-114,2694,707,-114,2694,467,-110,2726,467,-110,2726,7`
+`07,-110,2974,387,-111,2974,347,-111,2990,347,-111,2990,387,-111,2974,571,-111,2974,499,-111,2990,499,-111,2990,571,-111,2974,675,-111,2974,611,-111,2990,611,-111,2990,675,-111],"vertslength":58,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,8,9,10,8,10,11,12,13,14,15,16,17,12,14,15,12,15,17,18,19,20,21,22,23,24,25,26,24,26,27,28,29,30,31,32,33,33,28,30,30,31,33,37,34,35,35,36,37,38,39,40,38,40,41,45,42,43,43,44,45,49,46,47,47,48,49,53,50,51,51,52,53,57,54,55,55,56,57],"trislength":30,"triTopoly":[0,0,1,1,2,2,3`
+`,3,3,3,4,5,6,6,7,7,7,7,8,8,9,9,10,10,11,11,12,12,13,13],"baseVert":[0,4,8,12,18,21,24,28,34,38,42,46,50,54],"vertsCount":[4,4,4,6,3,3,4,6,4,4,4,4,4,4],"baseTri":[0,2,4,6,10,11,12,14,18,20,22,24,26,28],"triCount":[2,2,2,4,1,1,2,4,2,2,2,2,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_6",{"tileId":"12_6","tx":12,"ty":6,"mesh":{"verts":[3222,347,-408,3206,363,-404,3174,371,-408,3094,347,-412,3174,395,-408,3198,403,-408,3198,491,-408,3198,491,-408,3174,515,-408,3094,523,-41`
+`2,3094,347,-412,3174,395,-408,3094,347,-412,3174,371,-408,3174,395,-408,3094,523,-412,3174,515,-408,3198,531,-408,3198,651,-408,3174,675,-408,3094,683,-412,3094,683,-412,3174,675,-408,3182,691,-408,3094,811,-412,3382,675,-408,3414,691,-408,3422,811,-412,3094,811,-412,3182,691,-408,3214,675,-408,3094,859,-352,3094,827,-348,3510,827,-348,3518,851,-352,3550,859,-352,3094,859,-352,3518,851,-352,3550,859,-352,3518,851,-352,3542,843,-352,3542,843,-352,3542,347,-352,3550,347,-352,3550,859,-352,3190,347`
+`,-140,3198,347,-140,3198,363,-140,3190,387,-14,3190,347,-14,3206,347,-13,3206,387,-13,3190,571,-14,3190,499,-14,3206,499,-13,3206,571,-13,3206,675,-13,3190,675,-14,3190,611,-14,3206,603,-13,3214,395,-408,3206,363,-404,3222,347,-408,3430,347,-408,3414,363,-408,3382,371,-404,3222,347,-408,3198,491,-408,3198,403,-408,3214,395,-408,3214,515,-408,3382,395,-408,3406,403,-408,3406,491,-408,3382,515,-408,3382,395,-408,3382,515,-408,3214,515,-408,3214,395,-408,3222,347,-408,3382,371,-404,3198,651,-408,31`
+`98,531,-408,3214,515,-408,3382,515,-408,3406,531,-408,3406,651,-408,3382,675,-408,3214,675,-408,3198,651,-408,3214,515,-408,3382,515,-408,3406,651,-408,3390,387,105,3390,347,105,3406,347,104,3406,387,104,3390,571,105,3390,499,105,3406,499,104,3406,571,104,3406,611,104,3406,675,104,3390,675,105,3390,603,105,3398,347,-205,3406,347,-205,3406,363,-205,3398,347,-48,3406,347,-48,3406,363,-48,3422,395,-408,3414,363,-408,3430,347,-408,3406,491,-408,3406,403,-408,3422,395,-408,3422,395,-408,3430,347,-408`
+`,3510,347,-412,3510,531,-412,3422,515,-408,3406,491,-408,3406,531,-408,3422,515,-408,3510,531,-412,3510,691,-412,3422,675,-408,3406,651,-408,3414,691,-408,3422,675,-408,3510,691,-412,3510,811,-412,3422,811,-412],"vertslength":134,"polys":[0,3,4,6,7,11,12,14,15,20,21,24,25,30,31,34,35,37,38,40,41,44,45,47,48,51,52,55,56,59,60,62,63,66,67,70,71,74,75,80,81,83,84,86,87,92,93,96,97,100,101,104,105,107,108,110,111,113,114,116,117,122,123,128,129,133],"polyslength":33,"regions":[7,7,7,7,5,3,3,9,9,9,9,`
+`12,13,15,18,2,2,2,2,2,1,1,1,19,20,21,22,23,8,8,8,4,6],"neighbors":[[[1,15],[0],[1,3],[0]],[[0],[1,17],[1,2]],[[0],[1,4],[0],[1,3],[1,1]],[[1,0],[0],[1,2]],[[1,2],[0],[1,20],[0],[1,5],[0]],[[1,4],[0],[1,6],[0]],[[0],[1,32],[0],[1,5],[0],[1,22]],[[0],[0],[0],[1,8]],[[0],[1,7],[1,9]],[[1,8],[0],[1,10]],[[0],[0],[0],[1,9]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,0],[1,19]],[[1,28],[0],[1,19],[0]],[[1,1],[0],[1,19],[0]],[[0],[1,29],[0],[1,19]],[[1,18],[1,22],[1,17]`
+`,[1,15],[1,16],[0]],[[1,4],[0],[1,22]],[[0],[1,31],[1,22]],[[1,6],[0],[1,20],[1,19],[1,21],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0]],[[0],[1,16],[1,30]],[[1,18],[0],[1,30]],[[1,28],[0],[0],[1,31],[0],[1,29]],[[0],[1,30],[0],[1,32],[0],[1,21]],[[0],[1,31],[0],[0],[1,6]]]},"detail":{"verts":[3222,347,-408,3206,363,-408,3174,371,-408,3094,347,-412,3174,395,-408,3198,403,-408,3198,491,-408,3198,491,-408,3174,515,-408,3094,523,-412,3094,347,-412,3174,395,-`
+`408,3094,347,-412,3174,371,-408,3174,395,-408,3094,523,-412,3174,515,-408,3198,531,-408,3198,651,-408,3174,675,-408,3094,683,-412,3094,683,-412,3174,675,-408,3182,691,-408,3094,811,-412,3382,675,-408,3414,691,-408,3422,811,-412,3094,811,-412,3182,691,-408,3214,675,-412,3094,859,-352,3094,827,-348,3510,827,-352,3518,851,-352,3550,859,-352,3094,859,-352,3518,851,-352,3550,859,-352,3518,851,-352,3542,843,-352,3542,843,-352,3542,347,-352,3550,347,-352,3550,859,-352,3190,347,-140,3198,347,-140,3198,3`
+`63,-140,3190,387,-13,3190,347,-13,3206,347,-13,3206,387,-13,3190,571,-13,3190,499,-13,3206,499,-13,3206,571,-13,3206,675,-13,3190,675,-13,3190,611,-13,3206,603,-13,3214,395,-412,3206,363,-408,3222,347,-412,3430,347,-408,3414,363,-408,3382,371,-408,3222,347,-412,3198,491,-408,3198,403,-408,3214,395,-412,3214,515,-412,3382,395,-408,3406,403,-408,3406,491,-408,3382,515,-408,3382,395,-408,3382,515,-408,3214,515,-412,3214,395,-412,3222,347,-412,3382,371,-408,3198,651,-408,3198,531,-408,3214,515,-412,`
+`3382,515,-408,3406,531,-408,3406,651,-408,3382,675,-408,3214,675,-412,3198,651,-408,3214,515,-412,3382,515,-408,3406,651,-408,3390,387,104,3390,347,104,3406,347,104,3406,387,104,3390,571,104,3390,499,104,3406,499,104,3406,571,104,3406,611,104,3406,675,104,3390,675,104,3390,603,104,3398,347,-205,3406,347,-205,3406,363,-205,3398,347,-48,3406,347,-48,3406,363,-48,3422,395,-412,3414,363,-408,3430,347,-412,3406,491,-408,3406,403,-408,3422,395,-412,3422,395,-412,3430,347,-412,3510,347,-412,3510,531,-4`
+`12,3422,515,-412,3406,491,-408,3406,531,-408,3422,515,-412,3510,531,-412,3510,691,-412,3422,675,-412,3406,651,-408,3414,691,-408,3422,675,-412,3510,691,-412,3510,811,-412,3422,811,-412],"vertslength":134,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,11,7,9,9,10,11,12,13,14,15,16,17,18,19,20,15,17,18,15,18,20,21,22,23,21,23,24,25,26,27,28,29,30,30,25,27,27,28,30,31,32,33,31,33,34,35,36,37,38,39,40,41,42,43,41,43,44,45,46,47,51,48,49,49,50,51,55,52,53,53,54,55,56,57,58,56,58,59,60,61,62,63,64,65,63,65,66,67,68,`
+`69,67,69,70,71,72,73,71,73,74,77,78,79,79,80,75,75,76,77,75,77,79,81,82,83,84,85,86,88,89,90,91,92,87,91,87,88,88,90,91,96,93,94,94,95,96,100,97,98,98,99,100,101,102,103,101,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,120,122,117,117,119,120,123,124,125,126,127,128,128,123,125,125,126,128,129,130,131,131,132,133,129,131,133],"trislength":68,"triTopoly":[0,0,1,2,2,2,3,4,4,4,4,5,5,6,6,6,6,7,7,8,9,10,10,11,12,12,13,13,14,14,15,16,16,17,17,18,18,19,19,19,19,20,21,`
+`22,22,22,22,23,23,24,24,25,25,26,27,28,29,30,30,30,30,31,31,31,31,32,32,32],"baseVert":[0,4,7,12,15,21,25,31,35,38,41,45,48,52,56,60,63,67,71,75,81,84,87,93,97,101,105,108,111,114,117,123,129],"vertsCount":[4,3,5,3,6,4,6,4,3,3,4,3,4,4,4,3,4,4,4,6,3,3,6,4,4,4,3,3,3,3,6,6,5],"baseTri":[0,2,3,6,7,11,13,17,19,20,21,23,24,26,28,30,31,33,35,37,41,42,43,47,49,51,53,54,55,56,57,61,65],"triCount":[2,1,3,1,4,2,4,2,1,1,2,1,2,2,2,1,2,2,2,4,1,1,4,2,2,2,1,1,1,1,4,4,3]},"links":{"poly":[13,14],"cost":[1536],"t`
+`ype":[1],"pos":[3206,571,-13,3206,603,-13],"length":1}}],["0_7",{"tileId":"0_7","tx":0,"ty":7,"mesh":{"verts":[-2538,859,-416,-2818,899,-408,-2826,875,-408,-2842,907,-408,-2818,899,-408,-2538,859,-416,-2538,1371,-416,-3042,1371,-417,-3010,859,-408,-2538,859,-416,-2826,875,-408,-3010,859,-408,-2826,875,-408,-2850,883,-408,-3010,859,-408,-2850,883,-408,-2842,907,-408,-3010,907,-408,-3042,1371,-417,-3042,907,-409,-3010,907,-408,-2842,907,-408],"vertslength":22,"polys":[0,2,3,7,8,10,11,13,14,17,18,2`
+`1],"polyslength":6,"regions":[1,1,1,1,1,1],"neighbors":[[[1,1],[0],[1,2]],[[0],[1,0],[0],[0],[1,5]],[[0],[1,0],[1,3]],[[1,2],[0],[1,4]],[[1,3],[0],[1,5],[0]],[[0],[0],[1,4],[1,1]]]},"detail":{"verts":[-2538,859,-416,-2818,899,-416,-2826,875,-408,-2803.84619140625,873.7692260742188,-416,-2842,907,-416,-2830,903,-408,-2818,899,-416,-2538,859,-416,-2538,1371,-416,-3042,1371,-416,-3010,859,-408,-2538,859,-416,-2803.84619140625,873.7692260742188,-416,-2826,875,-408,-3010,859,-408,-2826,875,-408,-2850`
+`,883,-408,-3010,859,-408,-2850,883,-408,-2842,907,-416,-3010,907,-416,-3010,891,-408,-3042,1371,-416,-3042,907,-416,-3010,907,-416,-2842,907,-416],"vertslength":26,"tris":[1,2,3,0,1,3,5,6,7,4,5,7,4,7,8,4,8,9,12,13,10,10,11,12,14,15,16,19,20,21,21,17,18,18,19,21,22,23,24,22,24,25],"trislength":14,"triTopoly":[0,0,1,1,1,1,2,2,3,4,4,4,5,5],"baseVert":[0,4,10,14,17,22],"vertsCount":[4,6,4,3,5,4],"baseTri":[0,2,6,8,9,12],"triCount":[2,4,2,1,3,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"lengt`
+`h":0}}],["1_7",{"tileId":"1_7","tx":1,"ty":7,"mesh":{"verts":[-2538,1371,-416,-2538,859,-416,-2026,859,-408,-2026,1371,-416],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[-2538,1371,-416,-2538,859,-416,-2514.727294921875,859,-408,-2026,859,-408,-2026,905.5454711914062,-408,-2026,928.8181762695312,-416,-2026,1371,-416,-2526,871,-408],"vertslength":8,"tris":[2,3,4,2,4,5,5,6,0,0,1,7,5,0,7,1,2,7,5,2,7],"trislength":7,"triTopoly":[0,0,`
+`0,0,0,0,0],"baseVert":[0],"vertsCount":[8],"baseTri":[0],"triCount":[7]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_7",{"tileId":"2_7","tx":2,"ty":7,"mesh":{"verts":[-2026,1371,-416,-2026,859,-408,-1514,859,-408,-1514,1371,-416],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[-2026,1371,-416,-2026,928.8181762695312,-416,-2026,905.5454711914062,-408,-2026,859,-408,-1514,859,-408,-1514,905.5454711914062,-408,-15`
+`14,928.8181762695312,-416,-1514,1371,-416],"vertslength":8,"tris":[2,3,4,1,2,4,1,4,5,1,5,6,7,0,1,1,6,7],"trislength":6,"triTopoly":[0,0,0,0,0,0],"baseVert":[0],"vertsCount":[8],"baseTri":[0],"triCount":[6]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_7",{"tileId":"3_7","tx":3,"ty":7,"mesh":{"verts":[-1514,1371,-416,-1514,859,-408,-1002,859,-408,-1002,1371,-416],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[-1`
+`514,1371,-416,-1514,928.8181762695312,-416,-1514,905.5454711914062,-408,-1514,859,-408,-1002,859,-408,-1002,905.5454711914062,-408,-1002,928.8181762695312,-416,-1002,1371,-416],"vertslength":8,"tris":[2,3,4,1,2,4,1,4,5,1,5,6,7,0,1,1,6,7],"trislength":6,"triTopoly":[0,0,0,0,0,0],"baseVert":[0],"vertsCount":[8],"baseTri":[0],"triCount":[6]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_7",{"tileId":"4_7","tx":4,"ty":7,"mesh":{"verts":[-1002,859,-408,-906,859,-408,-898,883,-408,`
+`-666,859,-416,-682,899,-416,-762,939,-416,-898,883,-408,-874,859,-408,-898,883,-408,-762,939,-416,-730,1019,-416,-698,1371,-416,-1002,1371,-416,-1002,859,-408,-730,1019,-416,-706,1027,-416,-698,1371,-416,-738,939,-340,-674,915,-340,-650,987,-340,-714,1011,-340,-714,995,-414,-730,947,-414,-682,931,-414,-666,979,-414,-706,1027,-416,-634,979,-416,-490,979,-416,-490,1371,-416,-698,1371,-416,-658,915,-416,-682,899,-416,-666,859,-416,-634,979,-416,-658,915,-416,-666,859,-416,-490,859,-416,-490,979,-41`
+`6],"vertslength":38,"polys":[0,2,3,7,8,13,14,16,17,20,21,24,25,29,30,32,33,37],"polyslength":9,"regions":[1,1,1,1,4,5,2,3,3],"neighbors":[[[0],[0],[1,2]],[[1,7],[0],[1,2],[0],[0]],[[1,1],[0],[1,3],[0],[0],[1,0]],[[0],[1,6],[1,2]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[1,8],[0],[0],[1,3]],[[0],[1,1],[1,8]],[[0],[1,7],[0],[0],[1,6]]]},"detail":{"verts":[-1002,859,-408,-906,859,-408,-898,883,-408,-666,859,-416,-682,899,-416,-762,939,-416,-878.5714111328125,891,-416,-898,883,-408,-874,859,-408,-8`
+`27.7777709960938,859,-416,-862,871,-408,-898,883,-408,-878.5714111328125,891,-416,-762,939,-416,-730,1019,-416,-698,1371,-416,-1002,1371,-416,-1002,928.8181762695312,-416,-1002,905.5454711914062,-408,-1002,859,-408,-730,1019,-416,-706,1027,-416,-698,1371,-416,-738,939,-340,-674,915,-340,-650,987,-340,-714,1011,-340,-714,995,-414,-730,947,-414,-682,931,-414,-666,979,-414,-706,1027,-416,-634,979,-416,-490,979,-416,-490,1371,-416,-698,1371,-416,-658,915,-416,-682,899,-416,-666,859,-416,-634,979,-41`
+`6,-658,915,-416,-666,859,-416,-490,859,-416,-490,979,-416],"vertslength":44,"tris":[0,1,2,6,7,8,3,4,5,5,6,9,3,5,9,6,8,10,8,9,10,9,6,10,19,11,12,18,19,12,17,18,12,12,13,14,17,12,14,14,15,16,14,16,17,20,21,22,23,24,25,23,25,26,30,27,28,28,29,30,31,32,33,34,35,31,31,33,34,36,37,38,39,40,41,42,43,39,39,41,42],"trislength":27,"triTopoly":[0,1,1,1,1,1,1,1,2,2,2,2,2,2,2,3,4,4,5,5,6,6,6,7,8,8,8],"baseVert":[0,3,11,20,23,27,31,36,39],"vertsCount":[3,8,9,3,4,4,5,3,5],"baseTri":[0,1,8,15,16,18,20,23,24],"t`
+`riCount":[1,7,7,1,2,2,3,1,3]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_7",{"tileId":"5_7","tx":5,"ty":7,"mesh":{"verts":[-490,859,-416,-394,859,-416,-386,1043,-416,-490,1371,-416,-386,1043,-416,22,1043,-416,22,1371,-416,-490,1371,-416,-362,1011,68,-362,859,68,22,859,68,22,1011,68],"vertslength":12,"polys":[0,3,4,7,8,11],"polyslength":3,"regions":[1,1,2],"neighbors":[[[0],[0],[1,1],[0]],[[0],[0],[0],[1,0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-490,859,-416,-394,859,-416`
+`,-386,1043,-416,-490,1371,-416,-386,1043,-416,22,1043,-416,22,1371,-416,-490,1371,-416,-362,1011,68,-362,859,68,22,859,68,22,1011,68],"vertslength":12,"tris":[0,1,2,0,2,3,4,5,6,4,6,7,11,8,9,9,10,11],"trislength":6,"triTopoly":[0,0,1,1,2,2],"baseVert":[0,4,8],"vertsCount":[4,4,4],"baseTri":[0,2,4],"triCount":[2,2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["6_7",{"tileId":"6_7","tx":6,"ty":7,"mesh":{"verts":[510,971,68,510,939,68,534,931,68,534,1011,68,422,907,68,470,923,68`
+`,454,947,68,382,859,68,422,875,68,422,907,68,422,907,68,454,947,68,470,971,68,22,1011,68,22,859,68,382,859,68,470,971,68,510,971,68,534,1011,68,22,1011,68,22,1371,-416,22,1043,-416,534,1043,-416,534,1371,-416,534,891,68,534,907,68,446,891,68,446,875,68,486,947,-80,486,859,-80,534,859,-80,534,947,-80,494,915,-480,494,859,-480,534,859,-480,534,939,-480,494,939,-184,494,859,-192,534,859,-192,534,939,-184,510,859,68,534,859,68,534,867,68],"vertslength":43,"polys":[0,3,4,6,7,9,10,15,16,19,20,23,24,27`
+`,28,31,32,35,36,39,40,42],"polyslength":11,"regions":[2,2,2,2,2,1,6,3,4,5,7],"neighbors":[[[0],[0],[0],[1,4]],[[0],[0],[1,3]],[[0],[0],[1,3]],[[1,1],[0],[1,4],[0],[0],[1,2]],[[0],[1,0],[0],[1,3]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[510,971,68,510,939,68,534,931,68,534,1011,68,422,907,68,470,923,68,454,947,68,382,859,68,422,875,68,422,907,68,422,907,68,454,947,68,470,971,68,22,1011,68,22,859,68,382,859,68,470`
+`,971,68,510,971,68,534,1011,68,22,1011,68,22,1371,-416,22,1043,-416,534,1043,-416,534,1371,-416,534,891,68,534,907,68,446,891,68,446,875,68,486,947,-80,486,859,-80,534,859,-80,534,947,-80,494,915,-480,494,859,-480,534,859,-480,534,939,-480,494,939,-184,494,859,-192,534,859,-192,534,939,-184,510,859,68,534,859,68,534,867,68],"vertslength":43,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,10,11,12,15,10,12,13,14,15,12,13,15,16,17,18,16,18,19,23,20,21,21,22,23,24,25,26,24,26,27,31,28,29,29,30,31,32,33,34,32,34,35`
+`,39,36,37,37,38,39,40,41,42],"trislength":21,"triTopoly":[0,0,1,2,3,3,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10],"baseVert":[0,4,7,10,16,20,24,28,32,36,40],"vertsCount":[4,3,3,6,4,4,4,4,4,4,3],"baseTri":[0,2,3,4,8,10,12,14,16,18,20],"triCount":[2,1,1,4,2,2,2,2,2,2,1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_7",{"tileId":"7_7","tx":7,"ty":7,"mesh":{"verts":[534,939,-184,534,859,-192,790,859,-192,790,939,-184,534,947,-80,534,859,-80,798,859,-80,798,947,-80,654,883,68,534,867,68,534,`
+`859,68,782,859,68,534,939,-480,534,867,-480,614,859,-480,790,859,-480,766,939,-480,534,891,68,550,891,68,534,907,68,902,859,68,1046,859,68,1046,1011,68,534,1011,68,782,907,68,534,1011,68,534,931,68,782,907,68,534,1371,-416,534,1043,-416,622,1043,-416,622,1371,-416,678,1043,-352,1046,1043,-352,1046,1371,-352,678,1371,-352],"vertslength":36,"polys":[0,3,4,7,8,11,12,16,17,19,20,24,25,27,28,31,32,35],"polyslength":9,"regions":[5,4,7,6,8,2,2,3,1],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],`
+`[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,6],[0]],[[0],[0],[1,5]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[534,939,-184,534,859,-192,790,859,-192,790,939,-184,534,947,-80,534,859,-80,798,859,-80,798,947,-80,654,883,68,534,867,68,534,859,68,782,859,68,534,939,-480,534,867,-480,614,859,-480,790,859,-480,766,939,-480,534,891,68,550,891,68,534,907,68,902,859,68,1046,859,68,1046,1011,68,534,1011,68,782,907,68,534,1011,68,534,931,68,782,907,68,534,1371,-416,534,1`
+`043,-416,622,1043,-416,622,1371,-416,678,1043,-352,1046,1043,-352,1046,1371,-352,678,1371,-352],"vertslength":36,"tris":[3,0,1,1,2,3,7,4,5,5,6,7,8,9,10,8,10,11,12,13,14,14,15,16,12,14,16,17,18,19,20,21,22,24,20,22,22,23,24,25,26,27,31,28,29,29,30,31,35,32,33,33,34,35],"trislength":18,"triTopoly":[0,0,1,1,2,2,3,3,3,4,5,5,5,6,7,7,8,8],"baseVert":[0,4,8,12,17,20,25,28,32],"vertsCount":[4,4,4,5,3,5,3,4,4],"baseTri":[0,2,4,6,9,10,13,14,16],"triCount":[2,2,2,3,1,3,1,2,2]},"links":{"poly":[],"cost":[],`
+`"type":[],"pos":[],"length":0}}],["8_7",{"tileId":"8_7","tx":8,"ty":7,"mesh":{"verts":[1046,1011,68,1046,859,68,1558,859,68,1558,1011,68,1046,1371,-352,1046,1043,-352,1558,1043,-352,1558,1371,-352],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[2,1],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1046,1011,68,1046,859,68,1558,859,68,1558,1011,68,1046,1371,-352,1046,1043,-352,1558,1043,-352,1558,1371,-352],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trisl`
+`ength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["9_7",{"tileId":"9_7","tx":9,"ty":7,"mesh":{"verts":[1558,1011,68,1558,859,68,1646,859,68,1646,1011,68,1558,1371,-352,1558,1043,-352,1678,1035,-352,1678,1035,-352,1678,859,-352,2070,859,-352,2070,1371,-352,1558,1371,-352,1678,1035,-352,2070,859,-352],"vertslength":14,"polys":[0,3,4,6,7,9,10,13],"polyslength":4,"regions":[2,1,1,1],"nei`
+`ghbors":[[[0],[0],[0],[0]],[[0],[0],[1,3]],[[0],[0],[1,3]],[[0],[1,1],[1,2],[0]]]},"detail":{"verts":[1558,1011,68,1558,859,68,1646,859,68,1646,1011,68,1558,1371,-352,1558,1043,-352,1678,1035,-352,1678,1035,-352,1678,859,-352,2070,859,-352,2070,1371,-352,1558,1371,-352,1678,1035,-352,2070,859,-352],"vertslength":14,"tris":[3,0,1,1,2,3,4,5,6,7,8,9,10,11,12,10,12,13],"trislength":6,"triTopoly":[0,0,1,2,3,3],"baseVert":[0,4,7,10],"vertsCount":[4,3,3,4],"baseTri":[0,2,3,4],"triCount":[2,1,1,2]},"lin`
+`ks":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["10_7",{"tileId":"10_7","tx":10,"ty":7,"mesh":{"verts":[2070,1371,-352,2070,859,-352,2582,859,-352,2582,1371,-352],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[2070,1371,-352,2070,859,-352,2582,859,-352,2582,1371,-352],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cos`
+`t":[],"type":[],"pos":[],"length":0}}],["11_7",{"tileId":"11_7","tx":11,"ty":7,"mesh":{"verts":[2582,1371,-352,2582,859,-352,3094,859,-352,3094,1371,-352],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[2582,1371,-352,2582,859,-352,3094,859,-352,3094,1371,-352],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cost":[],"type":[],"po`
+`s":[],"length":0}}],["12_7",{"tileId":"12_7","tx":12,"ty":7,"mesh":{"verts":[3094,1371,-352,3094,859,-352,3550,859,-352,3550,1371,-352],"vertslength":4,"polys":[0,3],"polyslength":1,"regions":[1],"neighbors":[[[0],[0],[0],[0]]]},"detail":{"verts":[3094,1371,-352,3094,859,-352,3550,859,-352,3550,1371,-352],"vertslength":4,"tris":[3,0,1,1,2,3],"trislength":2,"triTopoly":[0,0],"baseVert":[0],"vertsCount":[4],"baseTri":[0],"triCount":[2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}]`
+`,["0_8",{"tileId":"0_8","tx":0,"ty":8,"mesh":{"verts":[-3042,1515,-417,-3042,1371,-417,-2538,1371,-416,-2538,1515,-416,-2538,1531,-320,-2538,1539,-320,-3042,1539,-320],"vertslength":7,"polys":[0,3,4,6],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[-3042,1515,-416,-3042,1371,-416,-2538,1371,-416,-2538,1515,-416,-2538,1531,-320,-2538,1539,-320,-3042,1539,-320],"vertslength":7,"tris":[3,0,1,1,2,3,4,5,6],"trislength":3,"triTopoly":[0,0,1],"baseVert`
+`":[0,4],"vertsCount":[4,3],"baseTri":[0,2],"triCount":[2,1]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_8",{"tileId":"1_8","tx":1,"ty":8,"mesh":{"verts":[-2538,1515,-416,-2538,1371,-416,-2026,1371,-416,-2026,1515,-416,-2538,1539,-320,-2538,1531,-320,-2026,1531,-320,-2026,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2538,1515,-416,-2538,1371,-416,-2026,1371,-416,-2026,1515`
+`,-416,-2538,1539,-320,-2538,1531,-320,-2026,1531,-320,-2026,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_8",{"tileId":"2_8","tx":2,"ty":8,"mesh":{"verts":[-2026,1515,-416,-2026,1371,-416,-1514,1371,-416,-1514,1515,-416,-2026,1539,-320,-2026,1531,-320,-1514,1531,-320,-1514,1539,-320],"vertslength":8,"polys":[0,3`
+`,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-2026,1515,-416,-2026,1371,-416,-1514,1371,-416,-1514,1515,-416,-2026,1539,-320,-2026,1531,-320,-1514,1531,-320,-1514,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_8",{"tileId":"3_8","tx":3,"ty":8,"mesh":{`
+`"verts":[-1514,1515,-416,-1514,1371,-416,-1002,1371,-416,-1002,1515,-416,-1514,1539,-320,-1514,1531,-320,-1002,1531,-320,-1002,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-1514,1515,-416,-1514,1371,-416,-1002,1371,-416,-1002,1515,-416,-1514,1539,-320,-1514,1531,-320,-1002,1531,-320,-1002,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":`
+`[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_8",{"tileId":"4_8","tx":4,"ty":8,"mesh":{"verts":[-810,1371,-416,-826,1411,-416,-842,1435,-416,-1002,1371,-416,-842,1435,-416,-818,1443,-416,-810,1515,-416,-842,1435,-416,-810,1515,-416,-1002,1515,-416,-1002,1371,-416,-1002,1539,-320,-1002,1531,-320,-490,1531,-320,-490,1539,-320,-810,1419,-416,-826,1411,-416,-810,1371,-416,-554,1371,-416,-570,1411,-416,-586,1435,-416,-586,`
+`1435,-416,-562,1443,-416,-554,1515,-416,-810,1515,-416,-818,1443,-416,-810,1419,-416,-554,1419,-416,-570,1411,-416,-554,1371,-416,-490,1371,-416,-554,1515,-416,-562,1443,-416,-554,1419,-416,-490,1371,-416,-490,1515,-416],"vertslength":36,"polys":[0,3,4,6,7,10,11,14,15,20,21,26,27,30,31,35],"polyslength":8,"regions":[1,1,1,5,2,2,3,3],"neighbors":[[[1,4],[0],[1,2],[0]],[[0],[1,5],[1,2]],[[1,1],[0],[0],[1,0]],[[0],[0],[0],[0]],[[0],[1,0],[0],[1,6],[0],[1,5]],[[0],[1,7],[0],[1,1],[0],[1,4]],[[0],[1,`
+`4],[0],[1,7]],[[1,5],[0],[1,6],[0],[0]]]},"detail":{"verts":[-810,1371,-416,-826,1411,-416,-842,1435,-416,-1002,1371,-416,-842,1435,-416,-818,1443,-416,-810,1515,-416,-842,1435,-416,-810,1515,-416,-1002,1515,-416,-1002,1371,-416,-1002,1539,-320,-1002,1531,-320,-490,1531,-320,-490,1539,-320,-810,1419,-416,-826,1411,-416,-810,1371,-416,-554,1371,-416,-570,1411,-416,-586,1435,-416,-586,1435,-416,-562,1443,-416,-554,1515,-416,-810,1515,-416,-818,1443,-416,-810,1419,-416,-554,1419,-416,-570,1411,-416`
+`,-554,1371,-416,-490,1371,-416,-554,1515,-416,-562,1443,-416,-554,1419,-416,-490,1371,-416,-490,1515,-416],"vertslength":36,"tris":[0,1,2,0,2,3,4,5,6,7,8,9,7,9,10,14,11,12,12,13,14,15,16,17,18,19,20,20,15,17,17,18,20,21,22,23,24,25,26,24,26,21,21,23,24,27,28,29,27,29,30,31,32,33,35,31,33,33,34,35],"trislength":20,"triTopoly":[0,0,1,2,2,3,3,4,4,4,4,5,5,5,5,6,6,7,7,7],"baseVert":[0,4,7,11,15,21,27,31],"vertsCount":[4,3,4,4,6,6,4,5],"baseTri":[0,2,3,5,7,11,15,17],"triCount":[2,1,2,2,4,4,2,3]},"link`
+`s":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_8",{"tileId":"5_8","tx":5,"ty":8,"mesh":{"verts":[-490,1515,-416,-490,1371,-416,22,1371,-416,22,1515,-416,-490,1539,-320,-490,1531,-320,22,1531,-320,22,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[-490,1515,-416,-490,1371,-416,22,1371,-416,22,1515,-416,-490,1539,-320,-490,1531,-320,22,1531,-320,22,1539,-320],"vertslength":8,"tris":[3,0`
+`,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["6_8",{"tileId":"6_8","tx":6,"ty":8,"mesh":{"verts":[22,1515,-416,22,1371,-416,534,1371,-416,534,1515,-416,22,1539,-320,22,1531,-320,534,1531,-320,534,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[22,1515,-4`
+`16,22,1371,-416,534,1371,-416,534,1515,-416,22,1539,-320,22,1531,-320,534,1531,-320,534,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_8",{"tileId":"7_8","tx":7,"ty":8,"mesh":{"verts":[534,1515,-416,534,1371,-416,622,1371,-416,622,1515,-416,534,1539,-320,534,1531,-320,1046,1531,-320,1046,1539,-320,678,1515,-352,6`
+`78,1371,-352,1046,1371,-352,1046,1515,-352],"vertslength":12,"polys":[0,3,4,7,8,11],"polyslength":3,"regions":[2,3,1],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[534,1515,-416,534,1371,-416,622,1371,-416,622,1515,-416,534,1539,-320,534,1531,-320,1046,1531,-320,1046,1539,-320,678,1515,-352,678,1371,-352,1046,1371,-352,1046,1515,-352],"vertslength":12,"tris":[3,0,1,1,2,3,7,4,5,5,6,7,11,8,9,9,10,11],"trislength":6,"triTopoly":[0,0,1,1,2,2],"baseVert":[0,4`
+`,8],"vertsCount":[4,4,4],"baseTri":[0,2,4],"triCount":[2,2,2]},"links":{"poly":[1,2],"cost":[1920],"type":[2],"pos":[678,1531,-320,678,1515,-352],"length":1}}],["8_8",{"tileId":"8_8","tx":8,"ty":8,"mesh":{"verts":[1046,1515,-352,1046,1371,-352,1558,1371,-352,1558,1515,-352,1046,1539,-320,1046,1531,-320,1558,1531,-320,1558,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1046,1515,-352,1046,1371,-352`
+`,1558,1371,-352,1558,1515,-352,1046,1539,-320,1046,1531,-320,1558,1531,-320,1558,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[0,1],"cost":[1920],"type":[2],"pos":[1046,1515,-352,1046,1531,-320],"length":1}}],["9_8",{"tileId":"9_8","tx":9,"ty":8,"mesh":{"verts":[1558,1515,-352,1558,1371,-352,2070,1371,-352,2070,1515,-352,1558,1539,-320,1558,1531,-320,2070,1531`
+`,-320,2070,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[1558,1515,-352,1558,1371,-352,2070,1371,-352,2070,1515,-352,1558,1539,-320,1558,1531,-320,2070,1531,-320,2070,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[0,1],"cost":[1920],"type":[2],"pos":[1558,1515,-`
+`352,1558,1531,-320],"length":1}}],["10_8",{"tileId":"10_8","tx":10,"ty":8,"mesh":{"verts":[2070,1515,-352,2070,1371,-352,2582,1371,-352,2582,1515,-352,2070,1539,-320,2070,1531,-320,2582,1531,-320,2582,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0],[0]]]},"detail":{"verts":[2070,1515,-352,2070,1371,-352,2582,1371,-352,2582,1515,-352,2070,1539,-320,2070,1531,-320,2582,1531,-320,2582,1539,-320],"vertslength":8,"tris":[3,0,1,1`
+`,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[0,1],"cost":[1920],"type":[2],"pos":[2070,1515,-352,2070,1531,-320],"length":1}}],["11_8",{"tileId":"11_8","tx":11,"ty":8,"mesh":{"verts":[2582,1515,-352,2582,1371,-352,3094,1371,-352,3094,1515,-352,2582,1539,-320,2582,1531,-320,3094,1531,-320,3094,1539,-320],"vertslength":8,"polys":[0,3,4,7],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[`
+`[0],[0],[0],[0]]]},"detail":{"verts":[2582,1515,-352,2582,1371,-352,3094,1371,-352,3094,1515,-352,2582,1539,-320,2582,1531,-320,3094,1531,-320,3094,1539,-320],"vertslength":8,"tris":[3,0,1,1,2,3,7,4,5,5,6,7],"trislength":4,"triTopoly":[0,0,1,1],"baseVert":[0,4],"vertsCount":[4,4],"baseTri":[0,2],"triCount":[2,2]},"links":{"poly":[0,1],"cost":[1920],"type":[2],"pos":[2582,1515,-352,2582,1531,-320],"length":1}}],["12_8",{"tileId":"12_8","tx":12,"ty":8,"mesh":{"verts":[3094,1515,-352,3094,1371,-352`
+`,3550,1371,-352,3550,1515,-352,3094,1531,-320,3550,1531,-320,3094,1539,-320],"vertslength":7,"polys":[0,3,4,6],"polyslength":2,"regions":[1,2],"neighbors":[[[0],[0],[0],[0]],[[0],[0],[0]]]},"detail":{"verts":[3094,1515,-352,3094,1371,-352,3550,1371,-352,3550,1515,-352,3094,1531,-320,3550,1531,-320,3094,1539,-320],"vertslength":7,"tris":[3,0,1,1,2,3,4,5,6],"trislength":3,"triTopoly":[0,0,1],"baseVert":[0,4],"vertsCount":[4,3],"baseTri":[0,2],"triCount":[2,1]},"links":{"poly":[0,1],"cost":[1920],"`
+`type":[2],"pos":[3094,1515,-352,3094,1531,-320],"length":1}}],["0_9",{"tileId":"0_9","tx":0,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_9",{"tileId":"1_9","tx":1,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"`
+`neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_9",{"tileId":"2_9","tx":2,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost"`
+`:[],"type":[],"pos":[],"length":0}}],["3_9",{"tileId":"3_9","tx":3,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_9",{"tileId":"4_9","tx":4,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{`
+`"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_9",{"tileId":"5_9","tx":5,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"l`
+`ength":0}}],["6_9",{"tileId":"6_9","tx":6,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_9",{"tileId":"7_9","tx":7,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":`
+`0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["8_9",{"tileId":"8_9","tx":8,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["9_9",{"tile`
+`Id":"9_9","tx":9,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["10_9",{"tileId":"10_9","tx":10,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislengt`
+`h":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["11_9",{"tileId":"11_9","tx":11,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_9",{"tileId":"12_9","tx":12`
+`,"ty":9,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["0_10",{"tileId":"0_10","tx":0,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly"`
+`:[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_10",{"tileId":"1_10","tx":1,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_10",{"tileId":"2_10","tx":2,"ty":10,"mesh":{`
+`"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_10",{"tileId":"3_10","tx":3,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[`
+`],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_10",{"tileId":"4_10","tx":4,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_10",{"tileId":"5_10","tx":5,"ty":10,"mesh":{"verts":[],"vert`
+`slength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["6_10",{"tileId":"6_10","tx":6,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[`
+`],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_10",{"tileId":"7_10","tx":7,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["8_10",{"tileId":"8_10","tx":8,"ty":10,"mesh":{"verts":[],"vertslength":0,"poly`
+`s":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["9_10",{"tileId":"9_10","tx":9,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"`
+`triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["10_10",{"tileId":"10_10","tx":10,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["11_10",{"tileId":"11_10","tx":11,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"pol`
+`yslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_10",{"tileId":"12_10","tx":12,"ty":10,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCoun`
+`t":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["0_11",{"tileId":"0_11","tx":0,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_11",{"tileId":"1_11","tx":1,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"`
+`regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["2_11",{"tileId":"2_11","tx":2,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{`
+`"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_11",{"tileId":"3_11","tx":3,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_11",{"tileId":"4_11","tx":4,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"nei`
+`ghbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_11",{"tileId":"5_11","tx":5,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost"`
+`:[],"type":[],"pos":[],"length":0}}],["6_11",{"tileId":"6_11","tx":6,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_11",{"tileId":"7_11","tx":7,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"det`
+`ail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["8_11",{"tileId":"8_11","tx":8,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"p`
+`os":[],"length":0}}],["9_11",{"tileId":"9_11","tx":9,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["10_11",{"tileId":"10_11","tx":10,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts"`
+`:[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["11_11",{"tileId":"11_11","tx":11,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"le`
+`ngth":0}}],["12_11",{"tileId":"12_11","tx":12,"ty":11,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["0_12",{"tileId":"0_12","tx":0,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"verts`
+`length":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["1_12",{"tileId":"1_12","tx":1,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["`
+`2_12",{"tileId":"2_12","tx":2,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["3_12",{"tileId":"3_12","tx":3,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris"`
+`:[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["4_12",{"tileId":"4_12","tx":4,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["5_12",{"tileId":`
+`"5_12","tx":5,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["6_12",{"tileId":"6_12","tx":6,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength"`
+`:0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["7_12",{"tileId":"7_12","tx":7,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["8_12",{"tileId":"8_12","tx":8,"t`
+`y":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["9_12",{"tileId":"9_12","tx":9,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[`
+`],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["10_12",{"tileId":"10_12","tx":10,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["11_12",{"tileId":"11_12","tx":11,"ty":12,"mes`
+`h":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"baseVert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}],["12_12",{"tileId":"12_12","tx":12,"ty":12,"mesh":{"verts":[],"vertslength":0,"polys":[],"polyslength":0,"regions":[],"neighbors":[]},"detail":{"verts":[],"vertslength":0,"tris":[],"trislength":0,"triTopoly":[],"base`
+`Vert":[],"vertsCount":[],"baseTri":[],"triCount":[]},"links":{"poly":[],"cost":[],"type":[],"pos":[],"length":0}}]]}`;

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
                                        contour = this.splitLongEdges(this.simplifyContour(contour));
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
     * @param {Contour[]} counter
     */
    splitLongEdges(counter) {
        const maxEdgeLen = this.getContourMaxEdgeLen();
        if (maxEdgeLen <= 0) return counter;

        let guard = 0;
        while (guard++ < counter.length * 8) {
            let inserted = false;
            for (let i = 0; i < counter.length; i++) {
                const i0 = counter[i];
                const i1 = counter[(i + 1) % counter.length];
                const dx = Math.abs(i1.x - i0.x);
                const dy = Math.abs(i1.y - i0.y);
                if (Math.max(dx, dy) <= maxEdgeLen) continue;
                //这里在counter插入新点，值为两端点的中点
                const newPoint = {
                    x: (i0.x + i1.x) * 0.5,
                    y: (i0.y + i1.y) * 0.5,
                    z: (i0.z + i1.z) * 0.5,
                    regionId: i0.regionId,
                    neighborRegionId: i0.neighborRegionId
                };

                // 如果你的 counter/contour 存的是点对象：
                counter.splice(i + 1, 0, newPoint);
                inserted = true;
                break;
            }
            if (!inserted) break;
        }
        return counter;
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
     * 计算三维欧氏距离平方。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static lengthsq(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
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
     * 计算二维欧氏距离平方（仅 XY）。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static length2Dsq(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return dx * dx + dy * dy;
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

        /** @type {Float32Array} 顶点坐标数组，顺序为[x0,y0,z0,x1,y1,z1,...] */
        this.verts = new Float32Array(MAX_VERTS * 3); // 0:顶点0的x，1:顶点0的y，2:顶点0的z，3:顶点1的x，4:顶点1的y，5:顶点1的z，以此类推
        /** @type {number} 当前已用顶点数 */
        this.vertslength = 0;
        /** @type {Int32Array} 多边形顶点索引区间数组，顺序为[start0,end0,start1,end1,...] */
        this.polys = new Int32Array(MAX_POLYS * 2); // 0:多边形0的第一个顶点索引，1:多边形0的终点索引，2:多边形1的第一个顶点索引，3:多边形1的终点索引，以此类推
        /** @type {number} 当前已用多边形数 */
        this.polyslength = 0;
        /** @type {Int16Array} 多边形所属区域id数组 */
        this.regions = new Int16Array(MAX_POLYS);
        //最多32767个多边形，每个最多POLY_MAX_VERTS_PER_POLY条边，每个边几个邻居？100?
        /**
         * @type {Array<Array<Int16Array>>}
         * 多边形邻接信息：
         *  - neighbors[polyIdx][edgeIdx][0] 表示该边有几个邻居
         *  - neighbors[polyIdx][edgeIdx][1...N] 存储邻居多边形的索引
         * 结构：
         *   - 外层数组长度为最大多边形数
         *   - 每个多边形有 POLY_MAX_VERTS_PER_POLY 条边
         *   - 每条边可有多个邻居（最大100）
         */
        this.neighbors = new Array(MAX_POLYS); // [][][0] 0号位表示有几个邻居
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
            vertslength:this.vertslength,
            polys: this.polys,
            polyslength:this.polyslength,
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
        const pi=this.polyslength*2;
        this.polys[pi]=this.vertslength;
        for (const v of poly) {
            const vi = this.vertslength*3;
            this.verts[vi]=v.x;
            this.verts[vi+1]=v.y;
            this.verts[vi+2]=v.z;
            this.vertslength++;
        }
        this.polys[pi+1]=this.vertslength-1;
        this.regions[this.polyslength]=poly[0].regionId;
        this.polyslength++;
    }

    convertVertsToWorldAfterAdjacency() {
        if (this.worldConverted) return;
        // 只转换实际已用顶点，且每次步进3
        for (let i = 0; i < this.vertslength; i++) {
            const vi = i * 3;
            const v = this.toWorldVertex({
                x: this.verts[vi],
                y: this.verts[vi + 1],
                z: this.verts[vi + 2]
            });
            this.verts[vi] = v.x;
            this.verts[vi + 1] = v.y;
            this.verts[vi + 2] = v.z;
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
        // 先重置所有邻居信息
        for (let pi = 0; pi < this.polyslength; pi++) {
            const startVert = this.polys[pi * 2];
            const endVert = this.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            this.neighbors[pi]=new Array(vertCount);
            for (let ei = 0; ei < vertCount; ei++) {
                if (!this.neighbors[pi][ei]) {
                    this.neighbors[pi][ei] = new Int16Array(100);
                }
                this.neighbors[pi][ei][0] = 0; // 0号位表示邻居数量
            }
        }
        for (let pi = 0; pi < this.polyslength; pi++) {
            const startVert = this.polys[pi * 2];
            const endVert = this.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            for (let ei = 0; ei < vertCount; ei++) {
                const a = startVert + ei;
                const b = startVert + ((ei + 1) % vertCount);
                const ka = `${this.verts[a * 3]},${this.verts[a * 3 + 1]},${this.verts[a * 3 + 2]}`;
                const kb = `${this.verts[b * 3]},${this.verts[b * 3 + 1]},${this.verts[b * 3 + 2]}`;
                const lk = ka + '|' + kb;
                const rk = kb + '|' + ka;
                if (!edgeMap.has(lk)) {
                    edgeMap.set(lk, { poly: pi, edge: ei });
                    edgeMap.set(rk, { poly: pi, edge: ei });
                } else {
                    const other = edgeMap.get(lk);
                    if (!other) continue;
                    // 双向写入邻居
                    let n1 = ++this.neighbors[pi][ei][0];
                    this.neighbors[pi][ei][n1] = other.poly;
                    let n2 = ++this.neighbors[other.poly][other.edge][0];
                    this.neighbors[other.poly][other.edge][n2] = pi;
                }
            }
        }
    }

    debugDrawPolys(duration = 5) {
        // 修正：this.polys为Int32Array，存储为[起始顶点索引, 结束顶点索引]，每个多边形2个元素
        for (let pi = 0; pi < this.polyslength; pi++) {
            const startVert = this.polys[pi * 2];
            const endVert = this.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            if (vertCount < 3) continue;
            const color = { r: 255, g: 255, b: 0 };
            for (let i = 0; i < vertCount; i++) {
                const vi0 = startVert + i;
                const vi1 = startVert + ((i + 1) % vertCount);
                const v0 = {
                    x: this.verts[vi0 * 3],
                    y: this.verts[vi0 * 3 + 1],
                    z: this.verts[vi0 * 3 + 2],
                };
                const v1 = {
                    x: this.verts[vi1 * 3],
                    y: this.verts[vi1 * 3 + 1],
                    z: this.verts[vi1 * 3 + 2],
                };
                const start = vec.Zfly(v0, 0);
                const end = vec.Zfly(v1, 0);
                Instance.DebugLine({ start, end, color, duration });
            }
        }
    }

    debugDrawAdjacency(duration = 15) {
        // 修正：边数应由多边形顶点数决定，不能直接用neighborsOfPoly.length
        for (let pi = 0; pi < this.polyslength; pi++) {
            const start = this.polyCenter(pi);
            const startVert = this.polys[pi * 2];
            const endVert = this.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            for (let ei = 0; ei < vertCount; ei++) {
                for(let ni=1;ni<=this.neighbors[pi][ei][0];ni++){
                    const neighborIndex = this.neighbors[pi][ei][ni];
                    // 只画一次，避免重复
                    if (neighborIndex < 0 || neighborIndex <= pi) continue;
                    const end = this.polyCenter(neighborIndex);
                    Instance.DebugLine({ start, end, color: { r: 255, g: 0, b: 255 }, duration });
                }
            }
        }
    }

    /**
     * @param {number} pi
     * @returns {{x:number, y:number, z:number}}
     */
    polyCenter(pi) {
        // 修正：根据多边形索引区间遍历顶点，累加坐标
        const startVert = this.polys[pi * 2];
        const endVert = this.polys[pi * 2 + 1];
        const vertCount = endVert - startVert + 1;
        if (vertCount <= 0) return { x: 0, y: 0, z: 0 };
        let x = 0, y = 0, z = 0;
        for (let vi = startVert; vi <= endVert; vi++) {
            x += this.verts[vi * 3];
            y += this.verts[vi * 3 + 1];
            z += this.verts[vi * 3 + 2];
        }
        return { x: x / vertCount, y: y / vertCount, z: z / vertCount };
    }

    debugDrawSharedEdges(duration = 15) {
        // 修正：遍历所有多边形和每条边，判断该边是否有邻居，有则高亮
        for (let pi = 0; pi < this.polyslength; pi++) {
            const startVert = this.polys[pi * 2];
            const endVert = this.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            if (vertCount < 3) continue;
            const neighborsOfPoly = this.neighbors[pi];
            if (!neighborsOfPoly) continue;
            for (let ei = 0; ei < vertCount; ei++) {
                const edgeNeighbors = neighborsOfPoly[ei];
                if (!edgeNeighbors) continue;
                const count = edgeNeighbors[0];
                if (count > 0) {
                    const vi0 = startVert + ei;
                    const vi1 = startVert + ((ei + 1) % vertCount);
                    const v0 = {
                        x: this.verts[vi0 * 3],
                        y: this.verts[vi0 * 3 + 1],
                        z: this.verts[vi0 * 3 + 2],
                    };
                    const v1 = {
                        x: this.verts[vi1 * 3],
                        y: this.verts[vi1 * 3 + 1],
                        z: this.verts[vi1 * 3 + 2],
                    };
                    const start = vec.Zfly(v0, 20);
                    const end = vec.Zfly(v1, 20);
                    Instance.DebugLine({ start, end, color: { r: 0, g: 255, b: 0 }, duration });
                }
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
        /** @type {Float32Array} */
        this.verts = new Float32Array(MAX_TRIS*3 * 3);//全局顶点数组，顺序为[x0,y0,z0,x1,y1,z1,...]，每个多边形的顶点在其中占用一个连续区间
        /** @type {number} */
        this.vertslength = 0;//点总数
        /** @type {Uint16Array} */
        this.tris = new Uint16Array(MAX_TRIS * 3);//第i个三角形的三个顶点为tris[3i][3i+1][3i+2],每个坐标为verts[tris[3i]|+1|+2]
        /** @type {number} */
        this.trislength = 0;//三角形总数
        /** @type {Uint16Array} */
        this.triTopoly = new Uint16Array(MAX_TRIS);//[i]:第i个三角形对应的多边形索引
        //每个多边形对应的三角形索引范围，格式为[baseVert=该多边形点索引起点, vertCount=该多边形有几个点, baseTri=该多边形三角索引起点, triCount=该多边形有几个三角形]
        /** @type {Uint16Array} */
        this.baseVert = new Uint16Array(MAX_POLYS);//该多边形点索引起点
        /** @type {Uint16Array} */
        this.vertsCount = new Uint16Array(MAX_POLYS);//该多边形有几个点
        /** @type {Uint16Array} */
        this.baseTri = new Uint16Array(MAX_POLYS);//该多边形三角索引起点
        /** @type {Uint16Array} */
        this.triCount = new Uint16Array(MAX_POLYS);//该多边形有几个三角形

        ///**@type {Vector[]}*/
        //this.verts = [];
        ///**@type {number[][]}*/
        //this.tris = [];
        ///**@type {number[][]}*/
        //this.meshes = [];
        ///**@type {number[]} */
        //this.triTopoly=[];
    }

    init() {
        this.error = false;
        for (let pi = 0; pi < this.mesh.polyslength; pi++) {
            this.buildPoly(pi);
        }

        return {
            verts: this.verts,
            vertslength:this.vertslength,
            tris: this.tris,
            trislength:this.trislength,
            triTopoly:this.triTopoly,
            baseVert:this.baseVert,
            vertsCount:this.vertsCount,
            baseTri:this.baseTri,
            triCount:this.triCount
        };
    }
    debugDrawPolys(duration = 5) {
        // TypedArray结构：tris为Uint16Array，verts为Float32Array
        for (let ti = 0; ti < this.trislength; ti++) {
            const ia = this.tris[ti * 3];
            const ib = this.tris[ti * 3 + 1];
            const ic = this.tris[ti * 3 + 2];
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            const va = {
                x: this.verts[ia * 3],
                y: this.verts[ia * 3 + 1],
                z: this.verts[ia * 3 + 2]
            };
            const vb = {
                x: this.verts[ib * 3],
                y: this.verts[ib * 3 + 1],
                z: this.verts[ib * 3 + 2]
            };
            const vc = {
                x: this.verts[ic * 3],
                y: this.verts[ic * 3 + 1],
                z: this.verts[ic * 3 + 2]
            };
            Instance.DebugLine({ start: va, end: vb, color, duration });
            Instance.DebugLine({ start: vb, end: vc, color, duration });
            Instance.DebugLine({ start: vc, end: va, color, duration });
        }
    }
    /**
     * @param {number} pi
     */
    buildPoly(pi) {
        // TypedArray结构：polys为索引区间数组，regions为Int16Array
        const startVert = this.mesh.polys[pi * 2];
        const endVert = this.mesh.polys[pi * 2 + 1];
        const poly = [startVert, endVert];
        const regionid = this.mesh.regions[pi];
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
        // TypedArray结构填充
        const baseVert = this.vertslength;
        const baseTri = this.trislength;
        const allVerts = trianglesCDT.vertices;
        // 填充verts
        for (let i = 0; i < allVerts.length; i++) {
            const v = allVerts[i];
            this.verts[baseVert * 3 + i * 3] = v.x;
            this.verts[baseVert * 3 + i * 3 + 1] = v.y;
            this.verts[baseVert * 3 + i * 3 + 2] = v.z;
        }
        this.vertslength += allVerts.length;
        triangles = trianglesCDT.getTri();
        if (trianglesCDT.error) this.error = true;
        // 填充tris和triTopoly
        for (let i = 0; i < triangles.length; i++) {
            const tri = triangles[i];
            this.tris[(baseTri + i) * 3] = baseVert + tri.a;
            this.tris[(baseTri + i) * 3 + 1] = baseVert + tri.b;
            this.tris[(baseTri + i) * 3 + 2] = baseVert + tri.c;
            this.triTopoly[baseTri + i] = pi;
        }
        this.trislength += triangles.length;
        // 填充baseVert、vertsCount、baseTri、triCount
        this.baseVert[pi] = baseVert;
        this.vertsCount[pi] = allVerts.length;
        this.baseTri[pi] = baseTri;
        this.triCount[pi] = triangles.length;
        // meshes数组可选，若需要保留
        // this.meshes.push([
        //     baseVert,
        //     allVerts.length,
        //     baseTri,
        //     triangles.length
        // ]);
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
        // poly为[startVert, endVert]区间
        const [start, end] = poly;
        const verts = [];
        for (let i = start; i <= end; i++) {
            const x = mesh.verts[i * 3];
            const y = mesh.verts[i * 3 + 1];
            const z = mesh.verts[i * 3 + 2];
            verts.push({ x, y, z });
        }
        return verts;
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
        /**@type {NavMeshMesh} */
        this.mesh = polyMesh;
        // 待更新：Max Jump Down Dist: 157
        this.jumpDist = 32;
        this.jumpHeight = MAX_JUMP_HEIGHT*MESH_CELL_SIZE_Z;
        this.walkHeight = MAX_WALK_HEIGHT*MESH_CELL_SIZE_Z;
        this.agentHeight = AGENT_HEIGHT * MESH_CELL_SIZE_Z;
        this.linkdist=250;// 可行走区域 A 与可行走区域 B 之间跳点最小间距

        /**@type {Uint16Array} */
        this.poly=new Uint16Array(MAX_LINKS*2);//每个link占2个uint16，this.poly[i*2]=startpoly, this.poly[i*2+1]=endpoly
        /**@type {Float32Array} */
        this.cost=new Float32Array(MAX_LINKS);//每个link的代价

        /**@type {Uint8Array} */
        this.type=new Uint8Array(MAX_LINKS);//每个link的类型

        /**@type {Float32Array} */
        this.pos=new Float32Array(MAX_LINKS*6);//每个link占6个float，this.pos[i*6]~this.pos[i*6+2]为startpos，this.pos[i*6+3]~this.pos[i*6+5]为endpos
        /**@type {number} */
        this.length=0;

        // 存储每个多边形所属的连通区域 ID
        /**@type {Int16Array} */
        this.islandIds=new Int16Array(MAX_POLYS);
    }
    /**
     * 收集所有边界边，返回TypedArray，每3个为一组：polyIndex, p1索引, p2索引
     * p1/p2为顶点索引（不是坐标），便于后续批量处理
     * @returns {{boundarylengh:number,boundaryEdges:Uint16Array}} [polyIndex, p1, p2, ...]
     */
    collectBoundaryEdges() {
        const polyCount = this.mesh.polyslength;
        // 预估最大边界边数量
        const maxEdges = polyCount * 6;
        const result = new Uint16Array(maxEdges * 3);
        let edgeCount = 0;
        for (let i = 0; i < polyCount; i++) {
            const startVert = this.mesh.polys[i * 2];
            const endVert = this.mesh.polys[i * 2 + 1];
            const vertCount = endVert - startVert + 1;
            for (let j = 0; j < vertCount; j++) {
                const neighList = this.mesh.neighbors[i][j];
                if (!neighList[0]) {
                    const vi0 = startVert + j;
                    const vi1 = startVert + ((j + 1) % vertCount);
                    const idx = edgeCount * 3;
                    result[idx] = i;
                    result[idx + 1] = vi0;
                    result[idx + 2] = vi1;
                    edgeCount++;
                }
            }
        }
        // 截取有效部分
        return {boundarylengh:edgeCount,boundaryEdges:result};
    }
    /**
     * 判断两个多边形是否已经是物理邻居
     * @param {number} idxA
     * @param {number} idxB
     */
    areNeighbors(idxA, idxB) {
        const edgeList = this.mesh.neighbors[idxA];
        for (const entry of edgeList) {
            for (let k = 1; k <= entry[0]; k++) {
                if (entry[k] === idxB) return true;
            }
        }
        return false;
    }
    // 1D 区间间距：重叠返回 0，不重叠返回最小间距
    /**
     * @param {number} a0
     * @param {number} a1
     * @param {number} b0
     * @param {number} b1
     */
    intervalGap(a0, a1, b0, b1) {
        const amin = Math.min(a0, a1);
        const amax = Math.max(a0, a1);
        const bmin = Math.min(b0, b1);
        const bmax = Math.max(b0, b1);

        if (amax < bmin) return bmin - amax; // A 在 B 左侧
        if (bmax < amin) return amin - bmax; // B 在 A 左侧
        return 0; // 重叠
    }
    /**
     * @param {number} p1x
     * @param {number} p1y
     * @param {number} p1z
     * @param {number} p2x
     * @param {number} p2y
     * @param {number} p2z
     * @param {number} p3x
     * @param {number} p3y
     * @param {number} p3z
     * @param {number} p4x
     * @param {number} p4y
     * @param {number} p4z
     * @param {number} dist2dsq
     */
    closestPtSegmentSegment(p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z,p4x,p4y,p4z,dist2dsq) {
        const gapZ=this.intervalGap(p1z, p2z, p3z, p4z);
        if (gapZ > this.jumpHeight) return;
        const gapX = this.intervalGap(p1x, p2x, p3x, p4x);
        const gapY = this.intervalGap(p1y, p2y, p3y, p4y);

        if (gapX * gapX + gapY * gapY > dist2dsq)return
        // 算法来源：Real-Time Collision Detection (Graham Walsh)
        // 计算线段 S1(p1,p2) 与 S2(p3,p4) 之间最近点
        
        const d1 = { x: p2x - p1x, y: p2y - p1y}; // 忽略 Z 参与平面距离计算
        const d2 = { x: p4x - p3x, y: p4y - p3y};
        const r = { x: p1x - p3x, y: p1y - p3y};

        const a = d1.x * d1.x + d1.y * d1.y; // Squared length of segment S1
        const e = d2.x * d2.x + d2.y * d2.y; // Squared length of segment S2
        const f = d2.x * r.x + d2.y * r.y;

        const EPSILON = 1;

        // 检查线段是否退化成点
        if (a <= EPSILON && e <= EPSILON) {
            // 两个都是点
            return { dist: (p1x - p3x)*(p1x - p3x) + (p1y - p3y)*(p1y - p3y) + (p1z - p3z)*(p1z - p3z), ptA: {x: p1x, y: p1y, z: p1z}, ptB: {x: p3x, y: p3y, z: p3z} };
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
            x: p1x + (p2x - p1x) * s,
            y: p1y + (p2y - p1y) * s,
            z: p1z + (p2z - p1z) * s
        };

        const ptB = {
            x: p3x + (p4x - p3x) * t,
            y: p3y + (p4y - p3y) * t,
            z: p3z + (p4z - p3z) * t
        };
        const heightDiff = Math.abs(ptA.z - ptB.z);
        if (heightDiff > this.jumpHeight) return;

        let dist=(ptA.x - ptB.x)*(ptA.x - ptB.x) + (ptA.y - ptB.y)*(ptA.y - ptB.y);
        if(dist > dist2dsq)return;
        dist+=heightDiff*heightDiff;
        if (heightDiff < 1 && dist < 1) return;
        return {
            dist,
            ptA,
            ptB
        };
    }
    /**
     * 返回当前构建的 navmeshlink 结构
     * @returns {{poly: Uint16Array;pos: Float32Array;type: Uint8Array;cost: Float32Array;length: number;}}
     * @param {import("./path_manager").NavMeshLink} [Extlink]
     */
    return(Extlink) {
        if(Extlink)
        {
            const a = Extlink.length;
            const b = this.length;

            this.poly.set(
                Extlink.poly.subarray(0, a * 2),
                b*2
            );

            this.cost.set(
                Extlink.cost.subarray(0, a),
                b
            );

            this.type.set(
                Extlink.type.subarray(0, a),
                b
            );

            this.pos.set(
                Extlink.pos.subarray(0, a * 6),
                b * 6
            );
            this.length+=a;
        }
        return {
            poly: this.poly,
            pos: this.pos,
            type: this.type,
            cost: this.cost,
            length: this.length
        };
    }
    //mesh内所有连接，根据constructor给的mesh,用在tile内构建
    init() {
        // 3) 计算 mesh 连通分量（islandIds），后续用于“同岛且高度可走”过滤。
        this.buildConnectivity();
        // 4) 收集边界边（只在边界边之间寻找 jump 候选）。
        const {boundarylengh,boundaryEdges} = this.collectBoundaryEdges();
        // 5) 为边界边建立空间网格索引，加速近邻边查询。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges,boundarylengh);
        // 6) 收集候选并执行首轮筛选，得到每个 poly 对的最优候选。
        const bestJumpPerPoly = this._collectBestJumpCandidates(boundaryEdges,boundarylengh, edgeGrid);
        // 7) 对候选做收尾去重（pair 去重 + 岛对近距去重），并生成最终 links。
        this._finalizeJumpLinks(bestJumpPerPoly);
        // 9) 返回构建完成的 links。
        return this.return();
    }
    /**
     * 仅构建传入tile的 jump link。
     * @param {number} boundarylengh 
     * @param {Uint16Array} boundaryEdges//边界边
     * @param {Uint8Array} tileid
     * @param {NavMeshLink} Extlink//已有的link
     */
    initInterTileIn(boundarylengh,boundaryEdges,tileid,Extlink) {
        // 4) 计算 mesh 连通分量。
        this.buildConnectivity(tileid);
        // 5) 收集边界边。
        // 6) 建立边界边空间索引。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges,boundarylengh);
        // 7) 收集候选并筛选：额外过滤“同 tile”pair，只保留跨 tile 候选。
        const bestJumpPerPoly = this._collectBestJumpCandidates(boundaryEdges,boundarylengh,edgeGrid,tileid);
        // 8) 对候选做收尾去重并生成最终 links。
        this._finalizeJumpLinks(bestJumpPerPoly);
        // 10) 返回构建完成的 links。
        return this.return(Extlink);
    }
    /**
     * @param {Uint16Array} boundaryEdges
     * @param {number} boundaryLength
     * @param {{grid: Map<number, number[]>;metas: Float32Array;cellSize: number;count: number}} edgeGrid
     * @param {Uint8Array} [tileid] //tile内的多边形打上标记，这样link只会从tile=2出发,即中心tile出发
     */
    _collectBestJumpCandidates(boundaryEdges, boundaryLength, edgeGrid, tileid) {
        // Key: "polyA_polyB", Value: { targetPoly, dist, startPos, endPos }
        const verts = this.mesh.verts;
        const islandIds = this.islandIds;
        const jumpDistSq = this.jumpDist * this.jumpDist;
        const bestJumpPerPoly = new Map();
        const candidateIndices=new Uint16Array(boundaryLength);
        for (let i = 0; i < boundaryLength; i++) {
            const idxA = (i<<1)+i;
            const polyIndexA = boundaryEdges[idxA];
            if(!islandIds[polyIndexA])continue;
            if(tileid&&tileid[polyIndexA]!=2)continue;
            const viA0 = boundaryEdges[idxA + 1]* 3;
            const viA1 = boundaryEdges[idxA + 2]* 3;
            candidateIndices[0]=0;
            this.queryNearbyEdges(edgeGrid, i, this.jumpDist,candidateIndices);
            for(let s=1;s<=candidateIndices[0];s++)
            {
                const j=candidateIndices[s];
                const idxB = (j<<1)+j;
                const polyIndexB = boundaryEdges[idxB];
                if(!islandIds[polyIndexB])continue;
                if(islandIds[polyIndexA] === islandIds[polyIndexB])continue;//同岛内的边界边不考虑构建跳跃链接
                if (polyIndexA === polyIndexB) continue;
                if(tileid&&tileid[polyIndexB]==2)continue;
                if(!tileid)
                {
                    //init()调用，判断多边形是否是邻居
                    if (this.areNeighbors(polyIndexA, polyIndexB)) continue;
                }
                const viB0 = boundaryEdges[idxB + 1]* 3;
                const viB1 = boundaryEdges[idxB + 2]* 3;
                const minBoxDist = this.bboxMinDist2D(edgeGrid.metas,i,j);
                if (minBoxDist > jumpDistSq) continue;
                
                const closestResult = this.closestPtSegmentSegment(
                    verts[viA0], verts[viA0+1], verts[viA0+2],
                    verts[viA1], verts[viA1+1], verts[viA1+2],
                    verts[viB0], verts[viB0+1], verts[viB0+2],
                    verts[viB1], verts[viB1+1], verts[viB1+2],
                    jumpDistSq);
                if (!closestResult) continue;

                const { dist, ptA, ptB } = closestResult;
                if (!this.validateJumpPath(ptA, ptB)) continue;
                this.updateBestCandidate(bestJumpPerPoly, polyIndexA, polyIndexB, dist, ptA, ptB);
            }
        }
        return bestJumpPerPoly;
    }

    /**
     * @param {Map<string,any>} bestJumpPerPoly
     * 直接填充TypedArray全局变量（poly、pos、type、cost、length）
     */
    _finalizeJumpLinks(bestJumpPerPoly) {
        const sortedCandidates = Array.from(bestJumpPerPoly.values());
        let linkCount = 0;
        const linkdistsq=this.linkdist*this.linkdist;
        for (const cand of sortedCandidates) {
            // 距离判重，需遍历已写入的link
            let tooClose = false;
            for (let k = 0; k < linkCount; k++) {
                const plIdx = k << 1;
                const exA = this.poly[plIdx];
                const exB = this.poly[plIdx + 1];
                const exIslandA = this.islandIds[exA];
                const exIslandB = this.islandIds[exB];
                const islandA = this.islandIds[cand.startPoly];
                const islandB = this.islandIds[cand.endPoly];
                if ((islandA === exIslandA && islandB === exIslandB) || (islandA === exIslandB && islandB === exIslandA)) {
                    // 距离判重
                    const posIdx = (k << 2) + (k << 1);
                    const exStart = {
                        x: this.pos[posIdx],
                        y: this.pos[posIdx + 1],
                        z: this.pos[posIdx + 2]
                    };
                    const exEnd = {
                        x: this.pos[posIdx + 3],
                        y: this.pos[posIdx + 4],
                        z: this.pos[posIdx + 5]
                    };
                    const dSqStart = vec.lengthsq(cand.startPos, exStart);
                    const dSqEnd = vec.lengthsq(cand.endPos, exEnd);
                    if (dSqStart < linkdistsq || dSqEnd < linkdistsq) {
                        tooClose = true;
                        break;
                    }
                }
            }
            if (tooClose) continue;
            // 写入TypedArray
            const pid=linkCount<<1;
            this.poly[pid] = cand.startPoly;
            this.poly[pid + 1] = cand.endPoly;
            const posIdx = (linkCount << 2) + (linkCount << 1);
            this.pos[posIdx] = cand.startPos.x;
            this.pos[posIdx + 1] = cand.startPos.y;
            this.pos[posIdx + 2] = cand.startPos.z;
            this.pos[posIdx + 3] = cand.endPos.x;
            this.pos[posIdx + 4] = cand.endPos.y;
            this.pos[posIdx + 5] = cand.endPos.z;
            this.cost[linkCount] = cand.dist * 1.5;
            this.type[linkCount] = (Math.abs(cand.startPos.z - cand.endPos.z) <= this.walkHeight ? PathState.WALK : PathState.JUMP);
            linkCount++;
        }
        this.length = linkCount;
    }
    /**
     * 计算多边形网格的连通分量
     * 给互相连通的多边形打上相同标识
     * 计算多边形网格的连通分量，TypedArray健壮兼容
     * 给互相连通的多边形打上相同标识
     * @param {Uint8Array<ArrayBufferLike>} [tileid]
     */
    buildConnectivity(tileid) {
        const numPolys = this.mesh.polyslength;
        this.islandIds = new Int16Array(numPolys);
        let currentId = 1;
        // 用TypedArray实现队列
        const queue = new Uint16Array(numPolys);
        for (let i = 0; i < numPolys; i++) {
            if (this.islandIds[i]) continue;
            if(tileid&&!tileid[i])continue;
            currentId++;
            let head = 0, tail = 0;
            queue[tail++] = i;
            this.islandIds[i] = currentId;
            while (head < tail) {
                let u = queue[head++];
                const neighbors = this.mesh.neighbors[u];
                // 获取该多边形的边数
                u<<=1;
                const startVert = this.mesh.polys[u];
                const endVert = this.mesh.polys[u + 1];
                const edgeCount = endVert - startVert + 1;
                for (let j = 0; j < edgeCount; j++) {
                    const entry = neighbors[j];
                    if (entry[0] == 0) continue;
                    for (let k = 1; k <= entry[0]; k++) {
                        const v = entry[k];
                        if (!this.islandIds[v]) {
                            this.islandIds[v] = currentId;
                            queue[tail++] = v;
                        }
                    }
                }
            }
        }
        //Instance.Msg(`共有${currentId-1}个独立行走区域`);
    }

    /**
     * 构建边界边空间网格索引，适配TypedArray输入
     * @param {Uint16Array} edges - 每3个为一组：polyIndex, p1, p2
     * @param {number} count - 边界边数量
     */
    buildEdgeGrid(edges, count) {
        const cellSize = this.jumpDist;
        const grid = new Map();
        const metas = new Float32Array(count << 2);
        for (let i = 0; i < count; i++) {
            const idx = (i<<1)+i;
            // const polyIndex = edges[idx]; // 未用
            const vi0 = edges[idx + 1]*3;
            const vi1 = edges[idx + 2]*3;
            const x0 = this.mesh.verts[vi0], y0 = this.mesh.verts[vi0 + 1];
            const x1 = this.mesh.verts[vi1], y1 = this.mesh.verts[vi1 + 1];
            const minX = Math.min(x0, x1);
            const maxX = Math.max(x0, x1);
            const minY = Math.min(y0, y1);
            const maxY = Math.max(y0, y1);
            const metaIdx = i << 2;
            metas[metaIdx] = minX;
            metas[metaIdx + 1] = maxX;
            metas[metaIdx + 2] = minY;
            metas[metaIdx + 3] = maxY;
            const gridX0 = Math.floor(minX / cellSize);
            const gridX1 = Math.floor(maxX / cellSize);
            const gridY0 = Math.floor(minY / cellSize);
            const gridY1 = Math.floor(maxY / cellSize);
            for (let x = gridX0; x <= gridX1; x++) {
                for (let y = gridY0; y <= gridY1; y++) {
                    const k = (y << 16) | x;
                    if(!grid.has(k)) grid.set(k, []);
                    grid.get(k).push(i);
                }
            }
        }
        return { grid, metas, cellSize,count};
    }

    /**
     * @param {{grid: Map<number, number[]>;metas: Float32Array;cellSize: number;count: number}} edgeGrid
     * @param {number} edgeIndex
     * @param {number} expand
     * @param {Uint16Array} result
     */
    queryNearbyEdges(edgeGrid, edgeIndex, expand, result) {
        edgeIndex <<=2;
        const x0 = Math.floor((edgeGrid.metas[edgeIndex] - expand) / edgeGrid.cellSize);
        const x1 = Math.floor((edgeGrid.metas[edgeIndex + 1] + expand) / edgeGrid.cellSize);
        const y0 = Math.floor((edgeGrid.metas[edgeIndex + 2] - expand) / edgeGrid.cellSize);
        const y1 = Math.floor((edgeGrid.metas[edgeIndex + 3] + expand) / edgeGrid.cellSize);
        /**@type {Uint8Array} */
        const seen = new Uint8Array(edgeGrid.count);
        for (let x = x0; x <= x1; x++) {
            for (let y = y0; y <= y1; y++) {
                const k = (y << 16) | x;
                const list = edgeGrid.grid.get(k);
                if (!list) continue;
                for (const idx of list) {
                    if (seen[idx]) continue;
                    seen[idx] = 1;
                    result[++result[0]] = idx;
                }
            }
        }
        return;
    }

    /**
     * @param {Float32Array} metas
     * @param {number} idxA
     * @param {number} idxB
     */
    bboxMinDist2D(metas, idxA, idxB) {
        idxA<<=2;
        idxB<<=2;
        return vec.length2Dsq({x:Math.max(0, Math.max(metas[idxA], metas[idxB]) - Math.min(metas[idxA + 1], metas[idxB + 1])),y:Math.max(0, Math.max(metas[idxA + 2], metas[idxB + 2]) - Math.min(metas[idxA + 3], metas[idxB + 3])),z:0});
    }

    /**
     * @param {Vector} a
     * @param {Vector} b
     */
    validateJumpPath(a, b) {
        const z=Math.max(a.z, b.z)+8;

        const start = { x: a.x, y: a.y, z: 8 };
        const end = { x: b.x, y: b.y, z: 8 };

        const boxMins = { x: -1, y: -1, z: 0 };
        const boxMaxs = { x: 1, y: 1, z: 1 };
        const hit = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(start,z),
            end:vec.Zfly(end,z),
            ignorePlayers: true
        });
        if (hit && hit.didHit) return false;
        const hitup = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(start,a.z),
            end:vec.Zfly(start,z),
            ignorePlayers: true
        });
        if (hitup && hitup.didHit) return false;
        const hitdown = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(end,z),
            end:vec.Zfly(end,b.z),
            ignorePlayers: true
        });
        if (hitdown && hitdown.didHit) return false;

        const hitReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start: vec.Zfly(end,z),
            end: vec.Zfly(start,z),
            ignorePlayers: true
        });
        if (hitReverse && hitReverse.didHit) return false;
        const hitupReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(end,b.z),
            end:vec.Zfly(end,z),
            ignorePlayers: true
        });
        if (hitupReverse && hitupReverse.didHit) return false;
        const hitdownReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(start,z),
            end:vec.Zfly(start,a.z),
            ignorePlayers: true
        });
        if (hitdownReverse && hitdownReverse.didHit) return false;
        return true;
    }
    /**
     * @param {Map<number,any>} map
     * @param {number} idxA
     * @param {number} idxB
    * @param {number} dist 两个多边形边界边之间的最短距离
     * @param {Vector} ptA
     * @param {Vector} ptB
     */
    updateBestCandidate(map, idxA, idxB, dist, ptA, ptB) {
        // 检查是否已记录过该多边形对的跳跃目标
        const key = (idxA << 16) | idxB;

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
        // 支持TypedArray结构
        Instance.Msg("debug");
        const { poly, pos, type, length } = this;
        const mesh = this.mesh;
        for (let i = 0; i < length; i++) {
            const polyA = poly[i * 2];
            const polyB = poly[i * 2 + 1];
            const t = type[i];
            const start = {
                x: pos[i * 6],
                y: pos[i * 6 + 1],
                z: pos[i * 6 + 2]
            };
            const end = {
                x: pos[i * 6 + 3],
                y: pos[i * 6 + 4],
                z: pos[i * 6 + 5]
            };
            Instance.DebugLine({
                start,
                end,
                color: { r: 0, g: (t === 1 ? 255 : 0), b: 255 },
                duration
            });
            // 可选：画起点终点球体
            // Instance.DebugSphere({ center: start, radius: 4, color: { r: 0, g: 255, b: 0 }, duration });
            // Instance.DebugSphere({ center: end, radius: 4, color: { r: 255, g: 0, b: 0 }, duration });
            // 绘制PolyB边界
            if (mesh && mesh.polys && mesh.verts) {
                const startVertB = mesh.polys[polyB * 2];
                const endVertB = mesh.polys[polyB * 2 + 1];
                const vertCountB = endVertB - startVertB+1;
                for (let j = 0; j < vertCountB; j++) {
                    const vi0 = startVertB + j;
                    const vi1 = startVertB + ((j + 1) % vertCountB);
                    const v0 = {
                        x: mesh.verts[vi0 * 3],
                        y: mesh.verts[vi0 * 3 + 1],
                        z: mesh.verts[vi0 * 3 + 2]
                    };
                    const v1 = {
                        x: mesh.verts[vi1 * 3],
                        y: mesh.verts[vi1 * 3 + 1],
                        z: mesh.verts[vi1 * 3 + 2]
                    };
                    Instance.DebugLine({ start: v0, end: v1, color: { r: 255, g: 0, b: 255 }, duration });
                }
                // 绘制PolyA边界
                const startVertA = mesh.polys[polyA * 2];
                const endVertA = mesh.polys[polyA * 2 + 1];
                const vertCountA = endVertA - startVertA + 1;
                for (let j = 0; j < vertCountA; j++) {
                    const vi0 = startVertA + j;
                    const vi1 = startVertA + ((j + 1) % vertCountA);
                    const v0 = {
                        x: mesh.verts[vi0 * 3],
                        y: mesh.verts[vi0 * 3 + 1],
                        z: mesh.verts[vi0 * 3 + 2]
                    };
                    const v1 = {
                        x: mesh.verts[vi1 * 3],
                        y: mesh.verts[vi1 * 3 + 1],
                        z: mesh.verts[vi1 * 3 + 2]
                    };
                    Instance.DebugLine({ start: v0, end: v1, color: { r: 255, g: 0, b: 255 }, duration });
                }
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
     * @param {{x:number,y:number,z:number}} pos
     */
    fromPosGetTile(pos) {
        const gx = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.x - origin.x) / MESH_CELL_SIZE_XY)));
        const gy = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.y - origin.y) / MESH_CELL_SIZE_XY)));
        const tx = Math.max(0, Math.min(this.tilesX - 1, Math.floor(gx / this.tileSize)));
        const ty = Math.max(0, Math.min(this.tilesY - 1, Math.floor(gy / this.tileSize)));
        return `${tx}_${ty}`;
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
        //if (POLY_DEBUG) {
        //    this.polyMeshGenerator.debugDrawPolys(tileDebugDuration);
        //    this.polyMeshGenerator.debugDrawAdjacency(tileDebugDuration);
        //}

        phaseStartMs = nowMs();

        this.polidetail = new PolyMeshDetailBuilder(tileMesh, this.hf);
        /** @type {NavMeshDetail} */
        let tileDetail = this.polidetail.init();
        //if(POLY_DETAIL_DEBUG)
        //{
        //    this.polidetail.debugDrawPolys(tileDebugDuration);
        //}
        if (this.polidetail.error) tileHasError = true;
        timing.detail += nowMs() - phaseStartMs;

        phaseStartMs = nowMs();
        this.jumplinkbuilder = new JumpLinkBuilder(tileMesh);
        /**
         * @type {NavMeshLink}
         */
        let tileLinks = this.jumplinkbuilder.init();
        //if(LINK_DEBUG)
        //{
        //    this.jumplinkbuilder.debugDraw(tileDebugDuration);
        //}
        timing.jumpLinks += nowMs() - phaseStartMs;

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
 * NavMesh 调试工具集
 *
 * 说明：
 * - MESH：体素化 / 网格化 步骤
 * - REGION：区域分割步骤
 * - CONTOUR：轮廓构建步骤
 * - POLY：多边形生成步骤
 * - DETAIL：细节层（detail）三角网
 * - LINK：连接（links）构建
 * - PATH：路径生成与输出（poly path / funnel path / final path）
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
     * 绘制 detail 层三角形（用于调试 detail 网格）。
     * 期望 detail 使用 TypedArray 布局：`verts` 为 Float32Array，`tris` 为 Uint16Array，
     * 并存在 `trislength` / `vertslength` 等计数字段。
     * @param {number} [duration]
     */
    debugDrawMeshDetail(duration = 10) {
        const detail = this.nav.meshdetail;
        if (!detail) return;
        // TypedArray 结构：detail.verts 为 Float32Array，detail.tris 为 Uint16Array，并存在 trislength/vertslength
        for (let i = 0; i < detail.trislength; i++) {
            const ia = detail.tris[i * 3];
            const ib = detail.tris[i * 3 + 1];
            const ic = detail.tris[i * 3 + 2];
            const va = {
                x: detail.verts[ia * 3],
                y: detail.verts[ia * 3 + 1],
                z: detail.verts[ia * 3 + 2]
            };
            const vb = {
                x: detail.verts[ib * 3],
                y: detail.verts[ib * 3 + 1],
                z: detail.verts[ib * 3 + 2]
            };
            const vc = {
                x: detail.verts[ic * 3],
                y: detail.verts[ic * 3 + 1],
                z: detail.verts[ic * 3 + 2]
            };
            const color = { r: 0, g: 180, b: 255 };
            Instance.DebugLine({ start: va, end: vb, color, duration });
            Instance.DebugLine({ start: vb, end: vc, color, duration });
            Instance.DebugLine({ start: vc, end: va, color, duration });
        }
        return;
    }
    debugLinks(duration = 30) {
        const links = this.nav.links;
        const mesh = this.nav.mesh;
        if (!links || !mesh || !mesh.polys || !mesh.verts) return;

        for (let li = 0; li < links.length; li++) {
            const type = links.type[li];
            const isJump = type === PathState.JUMP;
            const isLadder = type === PathState.LADDER;
            const lineColor = isLadder
                ? { r: 255, g: 165, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 0, b: 255 });
            const startColor = isLadder
                ? { r: 255, g: 215, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 255, b: 0 });

            const posBase = li * 6;
            const start = {
                x: links.pos[posBase],
                y: links.pos[posBase + 1],
                z: links.pos[posBase + 2]
            };
            const end = {
                x: links.pos[posBase + 3],
                y: links.pos[posBase + 4],
                z: links.pos[posBase + 5]
            };

            Instance.DebugLine({ start, end, color: lineColor, duration });
            Instance.DebugSphere({ center: start, radius: 4, color: startColor, duration });

            const pi = links.poly[(li << 1) + 1];
            if (pi < 0 || pi >= mesh.polyslength) continue;

            const startVert = mesh.polys[pi * 2];
            const endVert = mesh.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            for (let i = 0; i < vertCount; i++) {
                const vi0 = startVert + i;
                const vi1 = startVert + ((i + 1) % vertCount);
                const v0 = { x: mesh.verts[vi0 * 3], y: mesh.verts[vi0 * 3 + 1], z: mesh.verts[vi0 * 3 + 2] };
                const v1 = { x: mesh.verts[vi1 * 3], y: mesh.verts[vi1 * 3 + 1], z: mesh.verts[vi1 * 3 + 2] };
                Instance.DebugLine({ start: v0, end: v1, color: isLadder ? { r: 255, g: 140, b: 0 } : { r: 255, g: 0, b: 255 }, duration });
            }
        }
    }
    /**
     * 绘制所有多边形（不展示 links），用于检查多边形边界。
     * @param {number} duration
     */
    debugDrawMeshPolys(duration = 10) {
        if (!this.nav.mesh) return;
        const mesh = this.nav.mesh;
        for (let pi = 0; pi < mesh.polyslength; pi++) {
            const startVert = mesh.polys[pi * 2];
            const endVert = mesh.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            if (vertCount < 3) continue;
            const color = { r: 255, g: 0, b: 0 };
            for (let i = 0; i < vertCount; i++) {
                const vi0 = startVert + i;
                const vi1 = startVert + ((i + 1) % vertCount);
                const v0 = { x: mesh.verts[vi0 * 3], y: mesh.verts[vi0 * 3 + 1], z: mesh.verts[vi0 * 3 + 2] };
                const v1 = { x: mesh.verts[vi1 * 3], y: mesh.verts[vi1 * 3 + 1], z: mesh.verts[vi1 * 3 + 2] };
                Instance.DebugLine({ start: v0, end: v1, color, duration });
            }
        }
    }

    /**
     * 绘制网格连通关系（多边形邻接），用于调试跨 tile 的边界匹配。
     * 直接读取 `this.nav.mesh.neighbors` 结构并绘制连接线。
     * @param {number} [duration]
     */
    debugDrawMeshConnectivity(duration = 15) {
        if (!this.nav.mesh) return;
        const mesh = this.nav.mesh;
        const drawn = new Set();
        for (let i = 0; i < mesh.polyslength; i++) {
            const start = this._meshPolyCenter(i);
            const pstart=this.nav.mesh.polys[i*2];
            const pend=this.nav.mesh.polys[i*2+1];
            const ecount=pend-pstart+1;
            for (let e = 0; e < ecount; e++) {
                const edgeNei = mesh.neighbors[i][e][0];
                if(edgeNei==0)continue;
                for(let j=1;j<=edgeNei;j++)
                {
                    const ni=mesh.neighbors[i][e][j];
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
     * 计算指定多边形的几何中心（用于调试绘制）。
     * 适配 TypedArray 布局，返回 {x,y,z}。
     * @param {number} polyIndex
     */
    _meshPolyCenter(polyIndex) {
        const mesh = this.nav.mesh;
        const startVert = mesh.polys[polyIndex * 2];
        const endVert = mesh.polys[polyIndex * 2 + 1];
        const vertCount = endVert - startVert + 1;
        if (vertCount <= 0) return { x: 0, y: 0, z: 0 };
        let x = 0, y = 0, z = 0;
        for (let vi = startVert; vi <= endVert; vi++) {
            x += mesh.verts[vi * 3];
            y += mesh.verts[vi * 3 + 1];
            z += mesh.verts[vi * 3 + 2];
        }
        return { x: x / vertCount, y: y / vertCount, z: z / vertCount };
    }

    /**
     * 绘制 Funnel 生成的路径（用于调试 funnel 算法）。
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
     * 绘制路径（包含不同模式的颜色区分，例如行走/跳跃/梯子）。
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
     * @param {{id:number,mode:number}[]} polyPath
     * @param {number} [duration]
     */
    debugDrawPolyPath(polyPath, duration = 10) {
        if (!polyPath || polyPath.length === 0 || !this.nav.mesh) return;
        const mesh = this.nav.mesh;
        let prev = null;
        // 避免重复绘制相同路径段或中心点
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
            // 适配 TypedArray 布局：mesh.polys 存为 start/end 对，mesh.verts 为扁平 Float32Array
            const polyIndex = pi.id;
            const startVert = mesh.polys[polyIndex * 2];
            const endVert = mesh.polys[polyIndex * 2 + 1];
            const vertCount = endVert - startVert + 1;
            if (vertCount < 3) continue;
            let cx = 0, cy = 0, cz = 0;
            for (let vi = startVert; vi <= endVert; vi++) {
                cx += mesh.verts[vi * 3];
                cy += mesh.verts[vi * 3 + 1];
                cz += mesh.verts[vi * 3 + 2];
            }
            cx /= vertCount;
            cy /= vertCount;
            cz /= vertCount;
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
    /**
     * @param {number} duration
     */
    debugDrawALLTiles(duration = 120) {
        const color = { r: 255, g: 255, b: 255 };
        const fullGrid=Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        const tiles=Math.ceil(fullGrid / TILE_SIZE);
        for (let ty = 0; ty < tiles; ty++) {
            for (let tx = 0; tx < tiles; tx++) {
                const coreMinX = tx * TILE_SIZE;
                const coreMinY = ty * TILE_SIZE;
                const coreMaxX = Math.min(fullGrid - 1, coreMinX + TILE_SIZE - 1);
                const coreMaxY = Math.min(fullGrid - 1, coreMinY + TILE_SIZE - 1);

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
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
//不多，可以每次都重新构建
class LadderLinkBuilder {
    /**
     * @param {NavMeshMesh} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        /** @type {boolean} */
        this.error = false;
        /** @type {Uint16Array} */
        this.poly = new Uint16Array(MAX_LINKS * 2);
        /** @type {Float32Array} */
        this.cost = new Float32Array(MAX_LINKS);
        /** @type {Uint8Array} */
        this.type = new Uint8Array(MAX_LINKS);
        /** @type {Float32Array} */
        this.pos = new Float32Array(MAX_LINKS * 6);
        /** @type {number} */
        this.length = 0;
    }

    /**
     * @returns {NavMeshLink}
     */
    return() {
        return {
            poly: this.poly,
            cost: this.cost,
            type: this.type,
            pos: this.pos,
            length: this.length
        };
    }

    /**
     * @param {number} polyA
     * @param {number} polyB
     * @param {Vector} posA
     * @param {Vector} posB
     * @param {number} cost
     */
    pushLink(polyA, polyB, posA, posB, cost) {
        const i = this.length;
        const pi = i << 1;
        const vi = i * 6;
        this.poly[pi] = polyA;
        this.poly[pi + 1] = polyB;
        this.cost[i] = cost;
        this.type[i] = PathState.LADDER;
        this.pos[vi] = posA.x;
        this.pos[vi + 1] = posA.y;
        this.pos[vi + 2] = posA.z;
        this.pos[vi + 3] = posB.x;
        this.pos[vi + 4] = posB.y;
        this.pos[vi + 5] = posB.z;
        this.length++;
    }

    init() {
        this.error = false;
        this.length = 0;
        if (!this.mesh || !this.mesh.polys || this.mesh.polyslength === 0) return this.return();

        /** @type {Map<string, Vector[]>} */
        const groups = new Map();
        const ents = Instance.FindEntitiesByClass("info_target");

        for (const ent of ents) {
            const name = ent.GetEntityName();
            if (!name.startsWith("navmesh_LADDER_")) continue;

            const tag = name.slice("navmesh_LADDER_".length);
            if (!tag) continue;

            const p = ent.GetAbsOrigin();
            if (!p) continue;

            if (!groups.has(tag)) groups.set(tag, []);
            groups.get(tag)?.push({ x: p.x, y: p.y, z: p.z });
        }
        //let start=new Date();
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
            const p0 = points[0], p1 = points[1];
            const aPos = p0.z <= p1.z ? p0 : p1;
            const bPos = p0.z <= p1.z ? p1 : p0;
            //points.sort((a, b) => a.z - b.z);
            //const aPos = points[0];
            //const bPos = points[points.length - 1];
            rawPairs++;
            const aNearest = Tool.findNearestPoly(aPos, this.mesh);//,this.heightfixer);
            const bNearest = Tool.findNearestPoly(bPos, this.mesh);//,this.heightfixer);
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
            const cost = Math.max(1, vec.lengthsq(aNearest.pos, bNearest.pos));
            this.pushLink(aPoly, bPoly, aPos, bPos, cost);
            validPairs++;
        }
        Instance.Msg(`LadderLink统计: group=${groups.size} pair=${rawPairs} link=${this.length} valid=${validPairs}`);
        return this.return();
    }

    /**
     * @param {number} duration
     */
    debugDraw(duration = 30) {
        for (let i = 0; i < this.length; i++) {
            const vi = i * 6;
            const start = {
                x: this.pos[vi],
                y: this.pos[vi + 1],
                z: this.pos[vi + 2]
            };
            const end = {
                x: this.pos[vi + 3],
                y: this.pos[vi + 4],
                z: this.pos[vi + 5]
            };
            Instance.DebugLine({
                start,
                end,
                color: { r: 255, g: 165, b: 0 },
                duration
            });
            Instance.DebugSphere({ center: start, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
            Instance.DebugSphere({ center: end, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
        }
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
//手动跳点
class MapJUMPLinkBuilder {
    /**
     * @param {NavMeshMesh} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        /** @type {boolean} */
        this.error = false;
        /** @type {Uint16Array} */
        this.poly = new Uint16Array(MAX_LINKS * 2);
        /** @type {Float32Array} */
        this.cost = new Float32Array(MAX_LINKS);
        /** @type {Uint8Array} */
        this.type = new Uint8Array(MAX_LINKS);
        /** @type {Float32Array} */
        this.pos = new Float32Array(MAX_LINKS * 6);
        /** @type {number} */
        this.length = 0;
    }

    /**
     * @returns {NavMeshLink}
     */
    return() {
        return {
            poly: this.poly,
            cost: this.cost,
            type: this.type,
            pos: this.pos,
            length: this.length
        };
    }

    /**
     * @param {number} polyA
     * @param {number} polyB
     * @param {Vector} posA
     * @param {Vector} posB
     * @param {number} cost
     */
    pushLink(polyA, polyB, posA, posB, cost) {
        const i = this.length;
        const pi = i << 1;
        const vi = i * 6;
        this.poly[pi] = polyA;
        this.poly[pi + 1] = polyB;
        this.cost[i] = cost;
        this.type[i] = PathState.JUMP;
        this.pos[vi] = posA.x;
        this.pos[vi + 1] = posA.y;
        this.pos[vi + 2] = posA.z;
        this.pos[vi + 3] = posB.x;
        this.pos[vi + 4] = posB.y;
        this.pos[vi + 5] = posB.z;
        this.length++;
    }

    init() {
        this.error = false;
        this.length = 0;
        if (!this.mesh || !this.mesh.polys || this.mesh.polyslength === 0) return this.return();

        /** @type {Map<string, Vector[]>} */
        const groups = new Map();
        const ents = Instance.FindEntitiesByClass("info_target");

        for (const ent of ents) {
            const name = ent.GetEntityName();
            if (!name.startsWith("navmesh_JUMP_")) continue;

            const tag = name.slice("navmesh_JUMP_".length);
            if (!tag) continue;

            const p = ent.GetAbsOrigin();
            if (!p) continue;

            if (!groups.has(tag)) groups.set(tag, []);
            groups.get(tag)?.push({ x: p.x, y: p.y, z: p.z });
        }
        //let start=new Date();
        let rawPairs = 0;
        let validPairs = 0;

        for (const [tag, points] of groups) {
            if (points.length < 2) {
                this.error = true;
                Instance.Msg(`MapJUMPLink: ${tag} 点位不足(=${points.length})，已跳过`);
                continue;
            }
            if (points.length !== 2) {
                this.error = true;
                Instance.Msg(`MapJUMPLink: ${tag} 点位数量过多(${points.length})，已跳过`);
                continue;
            }
            const p0 = points[0], p1 = points[1];
            const aPos = p0.z <= p1.z ? p0 : p1;
            const bPos = p0.z <= p1.z ? p1 : p0;
            //points.sort((a, b) => a.z - b.z);
            //const aPos = points[0];
            //const bPos = points[points.length - 1];
            rawPairs++;
            const aNearest = Tool.findNearestPoly(aPos, this.mesh);//,this.heightfixer);
            const bNearest = Tool.findNearestPoly(bPos, this.mesh);//,this.heightfixer);
            const aPoly = aNearest.poly;
            const bPoly = bNearest.poly;
            if (aPoly < 0 || bPoly < 0) {
                this.error = true;
                Instance.Msg(`MapJUMPLink: ${tag} 找不到最近多边形，已跳过`);
                continue;
            }
            if (aPoly === bPoly) {
                this.error = true;
                Instance.Msg(`MapJUMPLink: ${tag} 两端落在同一 poly(${aPoly})，已跳过`);
                continue;
            }
            const cost = Math.max(1, vec.lengthsq(aNearest.pos, bNearest.pos));
            this.pushLink(aPoly, bPoly, aPos, bPos, cost);
            validPairs++;
        }
        Instance.Msg(`MapJUMPLink统计: group=${groups.size} pair=${rawPairs} link=${this.length} valid=${validPairs}`);
        return this.return();
    }

    /**
     * @param {number} duration
     */
    debugDraw(duration = 30) {
        for (let i = 0; i < this.length; i++) {
            const vi = i * 6;
            const start = {
                x: this.pos[vi],
                y: this.pos[vi + 1],
                z: this.pos[vi + 2]
            };
            const end = {
                x: this.pos[vi + 3],
                y: this.pos[vi + 4],
                z: this.pos[vi + 5]
            };
            Instance.DebugLine({
                start,
                end,
                color: { r: 255, g: 165, b: 0 },
                duration
            });
            Instance.DebugSphere({ center: start, radius: 4, color: { r: 0, g: 215, b: 255 }, duration });
            Instance.DebugSphere({ center: end, radius: 4, color: { r: 0, g: 215, b: 255 }, duration });
        }
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
//手动传送点
class PortalLinkBuilder {
    /**
     * @param {NavMeshMesh} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        /** @type {boolean} */
        this.error = false;
        /** @type {Uint16Array} */
        this.poly = new Uint16Array(MAX_LINKS * 2);
        /** @type {Float32Array} */
        this.cost = new Float32Array(MAX_LINKS);
        /** @type {Uint8Array} */
        this.type = new Uint8Array(MAX_LINKS);
        /** @type {Float32Array} */
        this.pos = new Float32Array(MAX_LINKS * 6);
        /** @type {number} */
        this.length = 0;
    }

    /**
     * @returns {NavMeshLink}
     */
    return() {
        return {
            poly: this.poly,
            cost: this.cost,
            type: this.type,
            pos: this.pos,
            length: this.length
        };
    }

    /**
     * @param {number} polyA
     * @param {number} polyB
     * @param {Vector} posA
     * @param {Vector} posB
     * @param {number} cost
     */
    pushLink(polyA, polyB, posA, posB, cost) {
        const i = this.length;
        const pi = i << 1;
        const vi = i * 6;
        this.poly[pi] = polyA;
        this.poly[pi + 1] = polyB;
        this.cost[i] = cost;
        this.type[i] = PathState.PORTAL;
        this.pos[vi] = posA.x;
        this.pos[vi + 1] = posA.y;
        this.pos[vi + 2] = posA.z;
        this.pos[vi + 3] = posB.x;
        this.pos[vi + 4] = posB.y;
        this.pos[vi + 5] = posB.z;
        this.length++;
    }

    init() {
        this.error = false;
        this.length = 0;
        if (!this.mesh || !this.mesh.polys || this.mesh.polyslength === 0) return this.return();

        /** @type {Map<string, Vector[]>} */
        const groups = new Map();
        const ents = Instance.FindEntitiesByClass("info_target");

        for (const ent of ents) {
            const name = ent.GetEntityName();
            if (!name.startsWith("navmesh_PORTAL_")) continue;

            const tag = name.slice("navmesh_PORTAL_".length);
            if (!tag) continue;

            const p = ent.GetAbsOrigin();
            if (!p) continue;

            if (!groups.has(tag)) groups.set(tag, []);
            groups.get(tag)?.push({ x: p.x, y: p.y, z: p.z });
        }
        //let start=new Date();
        let rawPairs = 0;
        let validPairs = 0;

        for (const [tag, points] of groups) {
            if (points.length < 2) {
                this.error = true;
                Instance.Msg(`PortalLink: ${tag} 点位不足(=${points.length})，已跳过`);
                continue;
            }
            if (points.length !== 2) {
                this.error = true;
                Instance.Msg(`PortalLink: ${tag} 点位数量过多(${points.length})，已跳过`);
                continue;
            }
            const p0 = points[0], p1 = points[1];
            const aPos = p0.z <= p1.z ? p0 : p1;
            const bPos = p0.z <= p1.z ? p1 : p0;
            //points.sort((a, b) => a.z - b.z);
            //const aPos = points[0];
            //const bPos = points[points.length - 1];
            rawPairs++;
            const aNearest = Tool.findNearestPoly(aPos, this.mesh);//,this.heightfixer);
            const bNearest = Tool.findNearestPoly(bPos, this.mesh);//,this.heightfixer);
            const aPoly = aNearest.poly;
            const bPoly = bNearest.poly;
            if (aPoly < 0 || bPoly < 0) {
                this.error = true;
                Instance.Msg(`PortalLink: ${tag} 找不到最近多边形，已跳过`);
                continue;
            }
            if (aPoly === bPoly) {
                this.error = true;
                Instance.Msg(`PortalLink: ${tag} 两端落在同一 poly(${aPoly})，已跳过`);
                continue;
            }
            this.pushLink(aPoly, bPoly, aPos, bPos, 1);
            validPairs++;
        }
        Instance.Msg(`PortalLink统计: group=${groups.size} pair=${rawPairs} link=${this.length} valid=${validPairs}`);
        return this.return();
    }

    /**
     * @param {number} duration
     */
    debugDraw(duration = 30) {
        for (let i = 0; i < this.length; i++) {
            const vi = i * 6;
            const start = {
                x: this.pos[vi],
                y: this.pos[vi + 1],
                z: this.pos[vi + 2]
            };
            const end = {
                x: this.pos[vi + 3],
                y: this.pos[vi + 4],
                z: this.pos[vi + 5]
            };
            Instance.DebugLine({
                start,
                end,
                color: { r: 255, g: 165, b: 0 },
                duration
            });
            Instance.DebugSphere({ center: start, radius: 4, color: { r: 0, g: 215, b: 255 }, duration });
            Instance.DebugSphere({ center: end, radius: 4, color: { r: 0, g: 215, b: 255 }, duration });
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
 *  links:NavMeshLink
 * }} TileData
 */
function newmesh()
{
    return {
        verts: new Float32Array(MAX_VERTS*3),
        vertslength: 0,
        polys: new Int32Array(MAX_POLYS*2),
        polyslength: 0,
        regions: new Int16Array(0),///这里和之后都不会用到，先放个空数组占位
        neighbors: new Array(MAX_POLYS)
    };
}
function newdetailmesh()
{
    return {
        verts: new Float32Array(MAX_TRIS*3*3),
        vertslength: 0,
        tris: new Uint16Array(MAX_TRIS*3),
        trislength: 0,
        triTopoly: new Uint16Array(MAX_TRIS),
        baseVert: new Uint16Array(MAX_POLYS),
        vertsCount: new Uint16Array(MAX_POLYS),
        baseTri: new Uint16Array(MAX_POLYS),
        triCount: new Uint16Array(MAX_POLYS)
    };
}
function newlink()
{
    return {
        poly:new Uint16Array(MAX_LINKS*2),
        cost:new Float32Array(MAX_LINKS),
        type:new Uint8Array(MAX_LINKS),
        pos:new Float32Array(MAX_LINKS*6),
        length:0
    };
}
class TileManager {
    /**
     * @param {NavMesh} nav
     */
    constructor(nav) {
        this.nav=nav;
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();
        /** @type {NavMeshMesh} */
        this.mesh=newmesh();
        /** @type {NavMeshDetail} */
        this.meshdetail=newdetailmesh();
        /** @type {NavMeshLink} */
        this.links= newlink();

        /** @type {NavMeshMesh} */
        this.prunemesh;
        /** @type {NavMeshDetail} */
        this.prunemeshdetail;
        /** @type {NavMeshLink} */
        this.prunelinks;

        /** @type {NavMeshLink} */
        this.supprlink= newlink();//ladder连接
        /** @type {NavMeshLink} */
        this.Extlink = newlink();//tile间连接
        /** @type {NavMeshLink} */
        this.baseLinks =newlink();//tile内连接

        /** @type {Map<string, {vertBase:number,vertCount:number,polyBase:number,polyCount:number,detailVertBase:number,detailVertCount:number,triBase:number,triCount:number,meshRecBase:number,meshRecCount:number}>} */
        this.tileRanges = new Map();
    }

    /**
     * @param {string} key
     * @param {number} tx
     * @param {number} ty
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink} tileLinks
     */
    addtile(key, tx, ty, tileMesh, tileDetail, tileLinks) {
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
        this._rebuildDeferredLinks(true,true,key);
    }

    /**
     * @param {string} key
     */
    removetile(key) {
        if (!this.tiles.has(key)) return;
        this.tiles.delete(key);
        this._removeTileData(key);
        this._rebuildDeferredLinks(false,false);
    }

    /**
     * @param {string} key
     * @param {number} tx
     * @param {number} ty
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink} tileLinks
     */
    updatetile(key, tx, ty, tileMesh, tileDetail, tileLinks) {
        this.addtile(key, tx, ty, tileMesh, tileDetail, tileLinks);//48ms
    }
    /**
     * @param {NavMeshMesh} mesh
     */
    buildSupperLinksForMesh(mesh) {
        let merged = this.copyLinks(new LadderLinkBuilder(mesh).init(), new MapJUMPLinkBuilder(mesh).init());
        return this.copyLinks(merged, new PortalLinkBuilder(mesh).init());
    }
    updatemesh()
    {
        const merged = this.return();
        this.nav.mesh = merged.mesh;
        this.nav.meshdetail = merged.meshdetail;
        this.nav.links = merged.links;
    }
    return() {
        return {
                mesh: this.prunemesh,
                meshdetail: this.prunemeshdetail,
                links: this.prunelinks
            }
    }

    /**
     * 初始化，什么都没有
     * @param {tile} tileBuilder
     */
    rebuildAll(tileBuilder) {
        this.tiles.clear();
        this.tileRanges.clear();
        this.mesh=newmesh();
        this.meshdetail = newdetailmesh();
        this.links = newlink();
        this.supprlink = newlink();
        this.Extlink=newlink();
        this.baseLinks =newlink();
        
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
                const key = tileData.tileId;
                this.tiles.set(key, {
                    tileId: key,
                    tx: tileData.tx,
                    ty: tileData.ty,
                    mesh: tileData.mesh,
                    detail: tileData.detail,
                    links: tileData.links
                });
                this._appendTileData(key, tileData.mesh, tileData.detail, tileData.links);
                this._rebuildDeferredLinks(true,false,key);
            }
        }
        this._rebuildDeferredLinks(false,true);
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
     * @param {NavMeshLink} tileLinks
     */
    _appendTileData(tileId, tileMesh, tileDetail, tileLinks) {
        const mesh = this.mesh;
        const meshdetail = this.meshdetail;
        const baseLinks = this.baseLinks;
        // 记录本次追加前的全局基址（用于后续写入时做偏移）
        const vertBase = mesh.vertslength; // 顶点基址（顶点数，不是浮点数长度）
        const polyBase = mesh.polyslength; // 多边形基址（多边形计数）

        // 记录 detail 层的基址（细节顶点与细节三角）
        const detailVertBase = meshdetail.vertslength;
        const triBase = meshdetail.trislength;
        const meshRecBase = polyBase; // mesh record 基址与 polyBase 对齐（每个 poly 一条 record）
        
        // =========================
        // 1) 追加多边形：把 tile 的每个 poly 的顶点按顺序追加到全局 verts 中，
        //    并在 polys 中记录该 poly 在 verts 中的 start/end 索引区间
        // =========================
        // append polys
        for (let i = 0; i < tileMesh.polyslength; i++) {
            const tstart = tileMesh.polys[i<<1];
            const tend = tileMesh.polys[(i<<1)+1];
            // poly 在全局 verts 中的起始顶点索引
            const start= mesh.vertslength;
            for (let k = tstart; k <= tend; k++) {

                const sx = tileMesh.verts[k * 3];
                const sy = tileMesh.verts[k * 3 + 1];
                const sz = tileMesh.verts[k * 3 + 2];
                const writeIndex = (mesh.vertslength) * 3;
                mesh.verts[writeIndex] = sx;
                mesh.verts[writeIndex + 1] = sy;
                mesh.verts[writeIndex + 2] = sz;

                mesh.vertslength++;
            }
            const end = mesh.vertslength - 1;
            // 将该 poly 的 start/end 写入 polys（每个 poly 占两个 Int32）
            const pi = mesh.polyslength * 2;
            mesh.polys[pi] = start;
            mesh.polys[pi + 1] = end;
            

            // 把 tile 本地的邻接关系（如果有）映射到全局 poly 索引空间
            const vertCount = tend - tstart + 1;
            mesh.neighbors[mesh.polyslength]=new Array(vertCount);
            for (let ei = 0; ei < vertCount; ei++) 
            {
                const nc=tileMesh.neighbors[i][ei][0];
                mesh.neighbors[mesh.polyslength][ei]=new Int16Array(100);
                mesh.neighbors[mesh.polyslength][ei][0]=nc;
                for(let ni=1;ni<=nc;ni++)
                {
                    const nei = tileMesh.neighbors[i][ei][ni];
                    const mappedNei = polyBase + nei;
                    mesh.neighbors[mesh.polyslength][ei][ni] = mappedNei;
                }
            }
            mesh.polyslength++;
        }

        meshdetail.verts.set(tileDetail.verts.subarray(0, tileDetail.vertslength * 3), detailVertBase * 3);
        meshdetail.vertslength+=tileDetail.vertslength;
        // =========================
        // 3) 追加 detail 三角形（tris）和 tri->poly 映射（triTopoly）到 TypedArray
        //    tris 以三元组存储顶点索引（每个值指向 meshdetail.verts 的顶点索引）
        // =========================

        for (let i = 0; i < tileDetail.trislength; i++) {

            let a = detailVertBase + tileDetail.tris[i * 3];
            let b = detailVertBase + tileDetail.tris[i * 3 + 1];
            let c = detailVertBase + tileDetail.tris[i * 3 + 2];

            const writeIdx = meshdetail.trislength * 3;
            meshdetail.tris[writeIdx] = a;
            meshdetail.tris[writeIdx + 1] = b;
            meshdetail.tris[writeIdx + 2] = c;

            meshdetail.triTopoly[meshdetail.trislength] = polyBase + tileDetail.triTopoly[i];
            meshdetail.trislength++;
        }

        // =========================
        // 4) 追加每个 poly 对应的 mesh record（baseVert, vertsCount, baseTri, triCount）
        //    这些数组以 poly 索引为下标，存储该 poly 的细节数据在全局数组中的起点与计数
        // =========================
        for (let i = 0; i < tileMesh.polyslength; i++) {

            const gi = meshRecBase + i;

            meshdetail.baseVert[gi] = detailVertBase + tileDetail.baseVert[i];
            meshdetail.vertsCount[gi] = tileDetail.vertsCount[i];
            meshdetail.baseTri[gi] = triBase + tileDetail.baseTri[i];
            meshdetail.triCount[gi] = tileDetail.triCount[i];
        }
        // 追加link
        const blid=baseLinks.length;
        baseLinks.cost.set(tileLinks.cost.subarray(0, tileLinks.length), blid);
        baseLinks.type.set(tileLinks.type.subarray(0, tileLinks.length), blid);
        baseLinks.pos.set(tileLinks.pos.subarray(0, tileLinks.length * 6), blid * 6);

        for (let i=0;i<tileLinks.length;i++)
        {
            baseLinks.poly[(blid+i)<<1]=polyBase+tileLinks.poly[i<<1];
            baseLinks.poly[((blid+i)<<1)+1]=polyBase+tileLinks.poly[(i<<1)+1];
        }
        baseLinks.length+=tileLinks.length;
        //记录 tile 在全局 mesh/detail 中的范围
        this.tileRanges.set(tileId, {
            vertBase,
            vertCount: mesh.vertslength-vertBase,
            polyBase,
            polyCount: tileMesh.polyslength,
            detailVertBase,
            detailVertCount: tileDetail.vertslength,
            triBase,
            triCount: tileDetail.trislength,
            meshRecBase,
            meshRecCount: tileMesh.polyslength
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

        const neighborTiles = this._collectNeighborTiles(tileData.tx, tileData.ty);
        if (neighborTiles.length === 0) return;
        //邻居 tile 的“开放边”
        const openEdgeStorebuckets = new Map();
        // =========================
        // 1️⃣ 收集邻居 tile 的开放边
        // =========================
        //收集所有邻居中的多边形的开放边(无邻居边)
        for (const nei of neighborTiles) {
            const neiRange = this.tileRanges.get(nei);
            if (!neiRange || neiRange.polyCount <= 0) continue;

            const end = neiRange.polyBase + neiRange.polyCount;
            for (let poly = neiRange.polyBase; poly < end; poly++) {
                const polyStart = this.mesh.polys[poly << 1];
                const polyEnd   = this.mesh.polys[(poly << 1) + 1];
                const vertCount = polyEnd - polyStart + 1;
                for (let edge = 0; edge < vertCount; edge++) 
                {
                    if (this.mesh.neighbors[poly][edge][0] > 0) continue; // 有邻居
                    const va = polyStart + edge;
                    const vb = polyStart + ((edge + 1) % vertCount);
                    const edgeRec = this.buildOpenEdgeRecord(this.mesh, poly, edge, va, vb);

                    const bucketKey = `${edgeRec.major}|${edgeRec.bucketId}`;
                    const bucket = Tool.getOrCreateArray(openEdgeStorebuckets, bucketKey);
                    bucket.push(edgeRec);
                }
            }
        }
        // =========================
        // 2️⃣ 当前 tile 尝试匹配
        // =========================
        const dedup = new Set();
        /**
         * @type {any[]}
         */
        const candidates=[];
        const curEnd = curRange.polyBase + curRange.polyCount;
        for (let poly = curRange.polyBase; poly < curEnd; poly++) {
            const polyStart = this.mesh.polys[poly << 1];
            const polyEnd   = this.mesh.polys[(poly << 1) + 1];
            const vertCount = polyEnd - polyStart + 1;
            for (let edge = 0; edge < vertCount; edge++) 
            {
                if (this.mesh.neighbors[poly][edge][0] > 0) continue;
                dedup.clear();
                candidates.length = 0;
                // ===== 2️⃣ 模糊匹配 =====
                this.findOpenEdgesByOverlap(
                    this.mesh,
                    openEdgeStorebuckets,
                    poly,
                    edge,
                    curRange.polyBase,
                    candidates,
                    dedup
                );

                for (const cand of candidates) {
                    this.addNeighborLink(this.mesh, poly, edge, cand.poly, cand.edge);
                }
                //可以维护一个所有tile的边界边
            }
        }
    }

    /**
     * @param {number} tx
     * @param {number} ty
     * @param {boolean} [includeDiagonal] 是否包含对角线邻居和自己
     * @returns {string[]}
     */
    _collectNeighborTiles(tx, ty, includeDiagonal = false) {
        /** @type {string[]} */
        const out = [];
        // 4/8邻居偏移
        const offsets = includeDiagonal
            ? [
                [-1, -1], [0, -1], [1, -1],
                [-1,  0], [0,  0], [1,  0],
                [-1,  1], [0,  1], [1,  1]
            ]
            : [
                [0, -1], [-1, 0], [1, 0], [0, 1]
            ];
        for (const [dx, dy] of offsets) {
            const ntx = tx + dx;
            const nty = ty + dy;
            // 构造 tileId，需与 addtile 时一致
            const tileId = `${ntx}_${nty}`;
            if (this.tiles.has(tileId)) out.push(tileId);
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
        const mesh = this.mesh;
        const detail = this.meshdetail;

        // 2) 预先计算被删除区间的右边界，用于后续索引重映射判断。
        const vertEnd = range.vertBase + range.vertCount;
        const polyEnd = range.polyBase + range.polyCount;
        const dVertEnd = range.detailVertBase + range.detailVertCount;
        const triEnd = range.triBase + range.triCount;

        // 3) 从主 mesh 中删除该 tile 占用的顶点/多边形/邻接记录。
        // =========================
        // 1️⃣ 删除 mesh verts（float x3）
        // =========================
        const vertMoveCount = mesh.vertslength - vertEnd;
        if (vertMoveCount > 0) {
            mesh.verts.copyWithin(
                range.vertBase * 3,
                vertEnd * 3,
                mesh.vertslength * 3
            );
        }
        mesh.vertslength -= range.vertCount;
        // =========================
        // 2️⃣ 删除 polys
        // =========================
        const polyMoveCount = mesh.polyslength - polyEnd;
        const oldpolylen=mesh.polyslength;
        if (polyMoveCount > 0) {
            mesh.polys.copyWithin(
                range.polyBase * 2,
                polyEnd * 2,
                mesh.polyslength * 2
            );
        }
        mesh.polyslength -= range.polyCount;

        // neighbors 也要左移
        mesh.neighbors.splice(range.polyBase, range.polyCount);

        // =========================
        // 3️⃣ 重映射 poly 顶点索引
        // =========================
        for (let i = range.polyBase; i < mesh.polyslength; i++) {

            const pi = i << 1;

            let start = mesh.polys[pi];
            let end   = mesh.polys[pi + 1];

            if (start >= vertEnd) {
                start -= range.vertCount;
                end   -= range.vertCount;
                mesh.polys[pi] = start;
                mesh.polys[pi + 1] = end;
            }
        }
        // =========================
        // 4️⃣ 重映射 neighbors poly index
        // =========================
        for (let p = 0; p < mesh.polyslength; p++) {

            const ppolyStart = mesh.polys[p << 1];
            const ppolyEnd   = mesh.polys[(p << 1) + 1];
            const vertCount = ppolyEnd - ppolyStart + 1;

            for (let e = 0; e < vertCount; e++) {

                const list = mesh.neighbors[p][e];
                const count = list[0];

                let write = 1;

                for (let i = 1; i <= count; i++) {

                    const n = list[i];

                    if (n >= range.polyBase && n < polyEnd) {
                        continue; // 删除
                    }

                    list[write++] = n >= polyEnd
                        ? n - range.polyCount
                        : n;
                }

                list[0] = write - 1;
            }
        }

        // =========================
        // 5️⃣ 删除 detail verts
        // =========================
        const dMove = detail.vertslength - dVertEnd;
        if (dMove > 0) {
            detail.verts.copyWithin(
                range.detailVertBase * 3,
                dVertEnd * 3,
                detail.vertslength * 3
            );
        }
        detail.vertslength -= range.detailVertCount;
        // =========================
        // 6️⃣ 删除 detail tris
        // =========================
        const triMove = detail.trislength - triEnd;
        if (triMove > 0) {
            detail.tris.copyWithin(
                range.triBase * 3,
                triEnd * 3,
                detail.trislength * 3
            );

            detail.triTopoly.copyWithin(
                range.triBase,
                triEnd,
                detail.trislength
            );
        }
        detail.trislength -= range.triCount;

        // =========================
        // 7️⃣ 重映射 detail tris 顶点
        // =========================
        for (let i = range.triBase*3; i < detail.trislength * 3; i++) {
            const v = detail.tris[i];
            if (v >= dVertEnd) {
                detail.tris[i] = v - range.detailVertCount;
            }
        }

        // =========================
        // 8️⃣ 重映射 triTopoly
        // =========================
        for (let i = range.triBase; i < detail.trislength; i++) {
            const p = detail.triTopoly[i];
            if (p >= polyEnd) {
                detail.triTopoly[i] = p - range.polyCount;
            }
        }

        detail.baseVert.copyWithin(range.polyBase, polyEnd, oldpolylen);
        detail.vertsCount.copyWithin(range.polyBase, polyEnd, oldpolylen);
        detail.baseTri.copyWithin(range.polyBase, polyEnd, oldpolylen);
        detail.triCount.copyWithin(range.polyBase, polyEnd, oldpolylen);
        for (let i = range.polyBase; i < mesh.polyslength; i++) {
            if (detail.baseVert[i] >= dVertEnd) detail.baseVert[i] -= range.detailVertCount;
            if (detail.baseTri[i]  >= triEnd)   detail.baseTri[i]  -= range.triCount;
        }

        // =========================
        // 9️⃣ 重映射 Links（TypedArray 版本）
        // =========================
        const remapLinks = (/** @type {NavMeshLink} */ linkSet) => {

            let write = 0;

            for (let i = 0; i < linkSet.length; i++) {

                const a = linkSet.poly[i << 1];
                const b = linkSet.poly[(i << 1) + 1];

                if (
                    (a >= range.polyBase && a < polyEnd) ||
                    (b >= range.polyBase && b < polyEnd)
                ) {
                    continue;
                }

                linkSet.poly[write << 1] =
                    a >= polyEnd ? a - range.polyCount : a;

                linkSet.poly[(write << 1) + 1] =
                    b >= polyEnd ? b - range.polyCount : b;

                linkSet.cost[write] = linkSet.cost[i];
                linkSet.type[write] = linkSet.type[i];

                for (let k = 0; k < 6; k++) {
                    linkSet.pos[write * 6 + k] =
                        linkSet.pos[i * 6 + k];
                }

                write++;
            }

            linkSet.length = write;
        };

        remapLinks(this.baseLinks);
        remapLinks(this.Extlink);
        remapLinks(this.supprlink);

        // =========================
        // 🔟 更新 tileRanges
        // =========================
        this.tileRanges.delete(tileId);

        for (const [k, r] of this.tileRanges.entries()) {

            if (r.vertBase > range.vertBase)
                r.vertBase -= range.vertCount;

            if (r.polyBase > range.polyBase)
                r.polyBase -= range.polyCount;

            if (r.detailVertBase > range.detailVertBase)
                r.detailVertBase -= range.detailVertCount;

            if (r.triBase > range.triBase)
                r.triBase -= range.triCount;

            if (r.meshRecBase > range.meshRecBase)
                r.meshRecBase -= range.meshRecCount;

            this.tileRanges.set(k, r);
        }
    }
    /**
     * @param {string} targettileId
     */
    getedgebytileid(targettileId)
    {
        /**
         * @type {string[]}
         */
        let neitileid = [];
        const tileData = this.tiles.get(targettileId);
        if (tileData) neitileid=this._collectNeighborTiles(tileData.tx, tileData.ty, true);
        const tilemark=new Uint8Array(4096*3);
        const result = new Uint16Array(4096 * 3);
        let edgeCount = 0;
        for (const tileId of neitileid) {
            const range=this.tileRanges.get(tileId);
            if(!range)continue;
            const end = range.polyBase + range.polyCount;
            for (let p = range.polyBase; p < end; p++) {
                const polyStart = this.mesh.polys[p << 1];
                const polyEnd   = this.mesh.polys[(p << 1) + 1];
                const vertCount = polyEnd - polyStart + 1;
                if(targettileId===tileId)tilemark[p]=2;
                else tilemark[p]=1;
                for (let j = 0; j < vertCount; j++) {
                    // 如果没有邻居，就是边界边
                    if (this.mesh.neighbors[p][j][0] === 0) {
                        const vi1 = polyStart + j;
                        const vi2 = polyStart + ((j + 1) % vertCount);
                        const idx =  edgeCount*3;
                        result[idx] = p;
                        result[idx+1] = vi1;
                        result[idx + 2] = vi2;
                        edgeCount++;
                   }
                }
            }
        }
        return { edgeCount, result, tilemark };
    }
    /**
     * @param {boolean} Extjump
     * @param {boolean} Supperlink
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     */
    _rebuildDeferredLinks(Extjump,Supperlink,targettileId) {
        if(Extjump&&targettileId)
        {
            const { edgeCount, result, tilemark } = this.getedgebytileid(targettileId);
            if(Extjump)this.Extlink = new JumpLinkBuilder(this.mesh).initInterTileIn(edgeCount,result,tilemark,this.Extlink);//15ms
        }
        if(Supperlink)
        {
            Tool.buildSpatialIndex(this.mesh);//ladder最后才会运行，弄完后才会裁剪，裁剪也会使用这个
            this.supprlink= this.buildSupperLinksForMesh(this.mesh);
        }
        let merged = this.copyLinks(this.baseLinks, this.Extlink);
        merged = this.copyLinks(merged, this.supprlink);
        this.links = merged;
    }
    /**
     * 把 b 追加到 a 后面，返回新的 link
     * @param {NavMeshLink} a
     * @param {NavMeshLink} b
     * @returns {NavMeshLink}
     */
    copyLinks(a, b) {
        const total = a.length + b.length;
        /** @type {NavMeshLink} */
        const merged = {
            poly: new Uint16Array(total * 2),
            cost: new Float32Array(total),
            type: new Uint8Array(total),
            pos:  new Float32Array(total * 6),
            length: total
        };

        let linkOff = 0;
        let polyOff = 0;
        let posOff  = 0;

        const append = (/** @type {NavMeshLink} */ src) => {
            if (!src || src.length === 0) return;

            merged.poly.set(src.poly.subarray(0, src.length * 2), polyOff);
            merged.cost.set(src.cost.subarray(0, src.length), linkOff);
            merged.type.set(src.type.subarray(0, src.length), linkOff);
            merged.pos.set(src.pos.subarray(0, src.length * 6), posOff);

            polyOff += src.length * 2;
            linkOff += src.length;
            posOff  += src.length * 6;
        };

        append(a); // 先 a
        append(b); // 再 b（追加到后面）
        return merged;
    }
    /**
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     * @returns {(string)[]}
     */
    _buildPolyTileKeys(targettileId) {
        /**
         * @type {string[]}
         */
        let neitileid = [];
        const polyTileKeys = new Array(this.mesh.polyslength);
        
        if (targettileId) {
            const tileData = this.tiles.get(targettileId);
            if (tileData) neitileid=this._collectNeighborTiles(tileData.tx, tileData.ty, true);
            for (const tileId of neitileid) {
                const range=this.tileRanges.get(tileId);
                if(!range)continue;
                const end = range.polyBase + range.polyCount;
                for (let p = range.polyBase; p < end; p++) {
                    polyTileKeys[p] = tileId;
                }
            }
        }
        else {
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
        const tileId = tileBuilder.fromPosGetTile(pos);
        if (this.tiles.has(tileId)) {
            this.removetile(tileId);
            this.pruneUnreachablePolys();
            return false;
        }
        const tileData = tileBuilder.buildTileNavMeshAtPos(pos);
        this.addtile(tileId, tileData.tx, tileData.ty, tileData.mesh, tileData.detail, tileData.links || []);
        this.pruneUnreachablePolys();
        return true;
    }

    pruneUnreachablePolys() {//15ms
        const mesh = this.mesh;
        const detail = this.meshdetail;
        const polyCount = mesh.polyslength;

        if (polyCount === 0) return;
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
        const reachable = new Uint8Array(polyCount);
        const queue = new Int32Array(polyCount);
        let keepCount = 0;
        let qh = 0, qt = 0;
        // 入队 seed
        for (const s of seedPolys) {
            if (reachable[s]) continue;
            reachable[s] = 1;
            keepCount++;
            queue[qt++] = s;
        }

        // 先把 links 建成按 poly 的邻接（一次性）
        const linkAdj = new Array(polyCount);
        for (let i = 0; i < polyCount; i++) linkAdj[i] = [];
        for (let i = 0; i < this.links.length; i++) 
        {
            const a = this.links.poly[i << 1];
            const b = this.links.poly[(i << 1) + 1];
            if (a >= 0 && a < polyCount && b >= 0 && b < polyCount)
            {
                linkAdj[a].push(b);
                linkAdj[b].push(a);
            }
        }

        // BFS
        while (qh < qt) 
        {
            const p = queue[qh++];

            // 走 neighbors
            const ps = mesh.polys[p << 1];
            const pe = mesh.polys[(p << 1) + 1];
            const edgeCount = pe - ps + 1;
            const edges = mesh.neighbors[p];
            for (let e = 0; e < edgeCount; e++) 
            {
                const list = edges[e];
                const count = list[0] | 0;
                for (let k = 1; k <= count; k++) {
                const n = list[k];
                if (n < 0 || n >= polyCount || reachable[n]) continue;
                reachable[n] = 1;
                keepCount++;
                queue[qt++] = n;
                }
            }

            // 走 links
            const la = linkAdj[p];
            for (let i = 0; i < la.length; i++) 
            {
                const n = la[i];
                if (reachable[n]) continue;
                reachable[n] = 1;
                keepCount++;
                queue[qt++] = n;
            }
        }

        const oldToNewPoly = new Int32Array(polyCount).fill(-1);

        let newPolyCount = 0;
        for (let i = 0; i < polyCount; i++) {
            if (reachable[i]) oldToNewPoly[i] = newPolyCount++;
        }
        // =========================
        // 5️⃣ 统计新 verts 数量
        // =========================

        const vertUsed = new Uint8Array(mesh.vertslength);
        let newVertCount = 0;

        for (let p = 0; p < polyCount; p++) {

            if (!reachable[p]) continue;

            const start = mesh.polys[p<<1];
            const end   = mesh.polys[(p<<1)+1];

            for (let v = start; v <= end; v++) {
                if (!vertUsed[v]) {
                    vertUsed[v] = 1;
                    newVertCount++;
                }
            }
        }

        const vertRemap = new Int32Array(mesh.vertslength).fill(-1);

        let writeV = 0;
        for (let i = 0; i < mesh.vertslength; i++) {
            if (vertUsed[i])
                vertRemap[i] = writeV++;
        }
        // =========================
        // 6️⃣ 构建 prunemesh
        // =========================
        /** @type {NavMeshMesh} */
        const newMesh = {
            verts: new Float32Array(newVertCount * 3),
            polys: new Int32Array(newPolyCount * 2),
            neighbors: new Array(newPolyCount),
            regions: new Int16Array(0),//无用
            polyslength: newPolyCount,
            vertslength: newVertCount
        };
        // verts copy
        for (let i = 0; i < mesh.vertslength; i++) {

            if (!vertUsed[i]) continue;

            const nv = vertRemap[i];

            newMesh.verts[nv*3]     = mesh.verts[i*3];
            newMesh.verts[nv*3 + 1] = mesh.verts[i*3 + 1];
            newMesh.verts[nv*3 + 2] = mesh.verts[i*3 + 2];
        }
        // polys copy
        for (let p = 0; p < polyCount; p++) {

            if (!reachable[p]) continue;

            const np = oldToNewPoly[p];

            const start = mesh.polys[p<<1];
            const end   = mesh.polys[(p<<1)+1];

            newMesh.polys[np<<1]     = vertRemap[start];
            newMesh.polys[(np<<1)+1] = vertRemap[end];

            // neighbors
            //////////////////////
            const edgeList = mesh.neighbors[p];
            const vertCount = end - start + 1;
            const newEdges = new Array(vertCount);

            for (let e = 0; e < vertCount; e++) {

                const list = edgeList[e];
                const count = list[0];

                const newList = new Int16Array(count + 1);

                let w = 1;

                for (let i = 1; i <= count; i++) {

                    const newIdx = oldToNewPoly[list[i]];
                    if (newIdx !== -1)newList[w++] = newIdx;
                }

                newList[0] = w - 1;
                newEdges[e] = newList;
            }

            newMesh.neighbors[np] = newEdges;
        }
        // =========================
        // 7️⃣ 统计 tri 数量
        // =========================

        let newTriCount = 0;

        for (let p = 0; p < polyCount; p++) {

            if (!reachable[p]) continue;
            newTriCount += detail.triCount[p];
        }
        let newDetailVertCount = 0;

        const detailVertRemap = new Int32Array(detail.vertslength);
        detailVertRemap.fill(-1);
        for (let t = 0; t < detail.trislength; t++) {
            if (!reachable[detail.triTopoly[t]]) continue;
            const base = t * 3;
            detailVertRemap[detail.tris[base]]     = newDetailVertCount++;
            detailVertRemap[detail.tris[base + 1]] = newDetailVertCount++;
            detailVertRemap[detail.tris[base + 2]] = newDetailVertCount++;
        }

        /**@type {NavMeshDetail} */
        const newDetail = {
            verts: new Float32Array(newDetailVertCount * 3),
            vertslength: newDetailVertCount,
            tris: new Uint16Array(newTriCount * 3),
            triTopoly: new Uint16Array(newTriCount),
            trislength: newTriCount,
            baseVert: new Uint16Array(newPolyCount),
            vertsCount: new Uint16Array(newPolyCount),
            baseTri: new Uint16Array(newPolyCount),
            triCount: new Uint16Array(newPolyCount)
        };
        for (let i = 0; i < detail.vertslength; i++) {

            const newIdx = detailVertRemap[i];
            if (newIdx === -1) continue;

            newDetail.verts[newIdx*3]     = detail.verts[i*3];
            newDetail.verts[newIdx*3 + 1] = detail.verts[i*3 + 1];
            newDetail.verts[newIdx*3 + 2] = detail.verts[i*3 + 2];
        }
        let writeTri = 0;

        for (let oldP = 0; oldP < polyCount; oldP++) {
            if (!reachable[oldP]) continue;
            const newP = oldToNewPoly[oldP];

            const triBase  = detail.baseTri[oldP];
            const triCount = detail.triCount[oldP];

            newDetail.baseVert[newP] = detail.baseVert[oldP];
            newDetail.vertsCount[newP] = detail.vertsCount[oldP];
            newDetail.baseTri[newP] = writeTri;
            newDetail.triCount[newP] = triCount;

            for (let t = 0; t < triCount; t++) {

                const oldTriIdx = triBase + t;

                const baseOld = oldTriIdx * 3;
                const baseNew = writeTri * 3;

                newDetail.tris[baseNew] =
                    detailVertRemap[detail.tris[baseOld]];

                newDetail.tris[baseNew + 1] =
                    detailVertRemap[detail.tris[baseOld + 1]];

                newDetail.tris[baseNew + 2] =
                    detailVertRemap[detail.tris[baseOld + 2]];

                newDetail.triTopoly[writeTri] = newP;

                writeTri++;
            }
        }
        this.prunemesh = newMesh;
        this.prunemeshdetail = newDetail;
        // =========================
        // 8️⃣ link copy
        // =========================

        const linkSet = this.links;

        let newLinkCount = 0;

        for (let i = 0; i < linkSet.length; i++) {
            const a = oldToNewPoly[linkSet.poly[i<<1]];
            const b = oldToNewPoly[linkSet.poly[(i<<1)+1]];
            if (a !== -1 && b !== -1)
                newLinkCount++;
        }
        /**@type {NavMeshLink} */
        const newLinks = {
            poly: new Uint16Array(newLinkCount * 2),
            cost: new Float32Array(newLinkCount),
            type: new Uint8Array(newLinkCount),
            pos:  new Float32Array(newLinkCount * 6),
            length: newLinkCount
        };

        let w = 0;

        for (let i = 0; i < linkSet.length; i++) {

            const na = oldToNewPoly[linkSet.poly[i<<1]];
            const nb = oldToNewPoly[linkSet.poly[(i<<1)+1]];

            if (na === -1 || nb === -1) continue;

            newLinks.poly[w<<1]     = na;
            newLinks.poly[(w<<1)+1] = nb;
            newLinks.cost[w] = linkSet.cost[i];
            newLinks.type[w] = linkSet.type[i];

            for (let k=0;k<6;k++)
                newLinks.pos[w*6+k] = linkSet.pos[i*6+k];

            w++;
        }
        this.prunelinks = newLinks;
        Instance.Msg(`可达性筛选完成: ${polyCount} -> ${keepCount}`);
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {number} poly
     * @param {number} edge
     * @param {number} va
     * @param {number} vb
     */
    buildOpenEdgeRecord(mesh, poly, edge, va, vb) {
        const ax = mesh.verts[va * 3];
        const ay = mesh.verts[va * 3 + 1];
        const az = mesh.verts[va * 3 + 2];

        const bx = mesh.verts[vb * 3];
        const by = mesh.verts[vb * 3 + 1];
        const bz = mesh.verts[vb * 3 + 2];

        const dx = bx - ax;
        const dy = by - ay;

        const len = Math.hypot(dx, dy);
        const major = Math.abs(dx) >= Math.abs(dy) ? 0 : 1;
        const lineCoord = major === 0
        ? (ay + by) * 0.5
        : (ax + bx) * 0.5;

        const pa = major === 0 ? ax : ay;
        const pb = major === 0 ? bx : by;

        const projMin = Math.min(pa, pb);
        const projMax = Math.max(pa, pb);
        const invLen = len > 1e-6 ? 1 / len : 0;

        const dirX = dx * invLen;
        const dirY = dy * invLen;

        const centerZ = (az + bz) * 0.5;

        const bucketScale = Math.max(1e-4, MESH_CELL_SIZE_XY * 0.6);
        const bucketId = Math.round(lineCoord / bucketScale);

        return { poly, edge, va, vb, exactKey: `${va}|${vb}`, major, lineCoord, projMin, projMax, dirX, dirY, centerZ, bucketId, };
    }

    /**
     * @param {NavMeshMesh} mesh
     * @param {Map<string,any[]>} buckets
     * @param {number} poly
     * @param {number} edge
     * @param {number} tilePolyStart
     * @param {any[]} candidates
     * @param {Set<string>} dedup
     */
    findOpenEdgesByOverlap(mesh, buckets, poly, edge, tilePolyStart,candidates,dedup) {

        const polys = mesh.polys;
        const verts = mesh.verts;

        const polyStart = polys[poly << 1];
        const polyEnd   = polys[(poly << 1) + 1];
        const vertCount = polyEnd - polyStart + 1;

        const va = polyStart + edge;
        const vb = polyStart + ((edge + 1) % vertCount);

        const ax = verts[va * 3];
        const ay = verts[va * 3 + 1];
        const az = verts[va * 3 + 2];

        const bx = verts[vb * 3];
        const by = verts[vb * 3 + 1];
        const bz = verts[vb * 3 + 2];

        const dx = bx - ax;
        const dy = by - ay;

        const len = Math.hypot(dx, dy);
        const invLen = len > 1e-6 ? 1 / len : 0;

        const dirX = dx * invLen;
        const dirY = dy * invLen;

        const major = Math.abs(dx) >= Math.abs(dy) ? 0 : 1;

        const lineCoord = major === 0
            ? (ay + by) * 0.5
            : (ax + bx) * 0.5;

        const pa = major === 0 ? ax : ay;
        const pb = major === 0 ? bx : by;

        const projMin = pa < pb ? pa : pb;
        const projMax = pa > pb ? pa : pb;

        const bucketScale = Math.max(1e-4, MESH_CELL_SIZE_XY * 0.6);
        const bucketId = Math.round(lineCoord / bucketScale);

        const lineTol = MESH_CELL_SIZE_XY * 0.6;
        const maxProjGapXY = MESH_CELL_SIZE_XY;
        const minXYOverlap = 0.1;
        const maxZDiff = MAX_WALK_HEIGHT * MESH_CELL_SIZE_Z;

        for (let b = bucketId - 1; b <= bucketId + 1; b++) {

            const bucketKey = `${major}|${b}`;
            const bucket = buckets.get(bucketKey);
            if (!bucket) continue;

            for (let i = 0; i < bucket.length; i++) {

                const candidate = bucket[i];

                if (candidate.poly === poly) continue;
                if (candidate.poly >= tilePolyStart) continue;
                if (Math.abs(candidate.lineCoord - lineCoord) > lineTol) continue;

                const dot = dirX * candidate.dirX + dirY * candidate.dirY;
                if (dot > -0.8) continue;

                // ===== XY 投影 gap =====

                const cva = candidate.va;
                const cvb = candidate.vb;

                const cax = verts[cva * 3];
                const cay = verts[cva * 3 + 1];
                const caz = verts[cva * 3 + 2];

                const cbx = verts[cvb * 3];
                const cby = verts[cvb * 3 + 1];
                const cbz = verts[cvb * 3 + 2];

                const curXMin = ax < bx ? ax : bx;
                const curXMax = ax > bx ? ax : bx;
                const curYMin = ay < by ? ay : by;
                const curYMax = ay > by ? ay : by;

                const candXMin = cax < cbx ? cax : cbx;
                const candXMax = cax > cbx ? cax : cbx;
                const candYMin = cay < cby ? cay : cby;
                const candYMax = cay > cby ? cay : cby;

                const gapX = Math.max(0, Math.max(curXMin, candXMin) - Math.min(curXMax, candXMax));
                const gapY = Math.max(0, Math.max(curYMin, candYMin) - Math.min(curYMax, candYMax));

                if (Math.hypot(gapX, gapY) >= maxProjGapXY) continue;

                // ===== 主轴 overlap =====

                const overlapMin = projMin > candidate.projMin ? projMin : candidate.projMin;
                const overlapMax = projMax < candidate.projMax ? projMax : candidate.projMax;

                if (overlapMax <= overlapMin) continue;
                if ((overlapMax - overlapMin) < minXYOverlap) continue;

                // ===== Z overlap =====

                const ca = major === 0 ? ax : ay;
                const cb = major === 0 ? bx : by;
                const cdc = cb - ca;

                let zMinA, zMaxA;

                if (Math.abs(cdc) <= 1e-6) {
                    zMinA = az < bz ? az : bz;
                    zMaxA = az > bz ? az : bz;
                } else {
                    const inv = 1 / cdc;
                    const t0 = (overlapMin - ca) * inv;
                    const t1 = (overlapMax - ca) * inv;

                    const z0 = az + (bz - az) * t0;
                    const z1 = az + (bz - az) * t1;

                    zMinA = z0 < z1 ? z0 : z1;
                    zMaxA = z0 > z1 ? z0 : z1;
                }

                const cca = major === 0 ? cax : cay;
                const ccb = major === 0 ? cbx : cby;
                const cdc2 = ccb - cca;

                let zMinB, zMaxB;

                if (Math.abs(cdc2) <= 1e-6) {
                    zMinB = caz < cbz ? caz : cbz;
                    zMaxB = caz > cbz ? caz : cbz;
                } else {
                    const inv2 = 1 / cdc2;
                    const t0 = (overlapMin - cca) * inv2;
                    const t1 = (overlapMax - cca) * inv2;

                    const z0 = caz + (cbz - caz) * t0;
                    const z1 = caz + (cbz - caz) * t1;

                    zMinB = z0 < z1 ? z0 : z1;
                    zMaxB = z0 > z1 ? z0 : z1;
                }

                const gapZ = Math.max(0, Math.max(zMinA, zMinB) - Math.min(zMaxA, zMaxB));
                if (gapZ >= maxZDiff) continue;

                const key = candidate.poly + "|" + candidate.edge;
                if (dedup.has(key)) continue;

                dedup.add(key);
                candidates.push(candidate);
            }
        }

        return ;
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
        // list[0] 存数量
        const countA = listA[0];
        let exists = false;

        for (let i = 1; i <= countA; i++) {
            if (listA[i] === polyB) {
                exists = true;
                break;
            }
        }

        if (!exists) {
            listA[0]++;
            listA[listA[0]] = polyB;
        }

        const countB = listB[0];
        exists = false;

        for (let i = 1; i <= countB; i++) {
            if (listB[i] === polyA) {
                exists = true;
                break;
            }
        }

        if (!exists) {
            listB[0]++;
            listB[listB[0]] = polyA;
        }
    }
}

//储存静态备选数据
class NVpluginStaticData
{
    constructor()
    {
        this.Data = ``+`{"tiles":[["kkkk_6_2",{"tileId":"6_2","tx":6,"ty":2,"mesh":{"verts":[246,-1573,-416,246,-1541,-416,174,-1533,-416,278,-1701,-416,278,-1589,-416,246,-1573,-416,174,-1533,-416,174,-1477,-416,22,-1477,-416,278,-1701,-416,246,-1573,-416,174,-1533,-416,22,-1477,-416,22,-1701,-416,246,-1437,-55,190,-1437,-55,182,-1461,-47,246,-1541,-42,246,-1541,-42,182,-1461,-47,134,-1445,-46,22,-1453,-45,22,-1701,-19,294,-1701,-46,294,-1701,-46,278,-1549,-47,246,-1541,-42,166,-1621,-608,190,-1621,-608,190,-1605,-608`
+`,158,-1597,-608,86,-1573,-608,86,-1549,-608,62,-1533,-608,54,-1573,-608,166,-1685,-608,182,-1669,-608,166,-1621,-608,166,-1685,-608,166,-1621,-608,158,-1597,-608,86,-1573,-608,54,-1573,-608,22,-1685,-608,158,-1597,-608,174,-1573,-608,86,-1573,-608,54,-1573,-608,22,-1549,-608,22,-1685,-608,254,-1549,-768,246,-1517,-767,150,-1517,-767,142,-1549,-768,22,-1565,-768,22,-1661,-768,94,-1669,-768,142,-1549,-768,326,-1669,-759,318,-1565,-764,254,-1549,-768,142,-1549,-768,94,-1669,-768,102,-1685,-768,246,`
+`-1421,-90,246,-1405,-90,22,-1405,-99,22,-1429,-101,118,-1429,-247,174,-1445,-248,182,-1421,-246,38,-1389,-248,30,-1437,-248,118,-1429,-247,182,-1421,-246,110,-1229,-256,142,-1269,-256,246,-1189,-256,22,-1381,-248,38,-1389,-248,182,-1421,-246,246,-1421,-248,110,-1325,-256,62,-1325,-256,78,-1221,-256,110,-1229,-256,246,-1189,-256,22,-1189,-256,246,-1189,-256,142,-1269,-256,110,-1325,-256,246,-1421,-248,30,-1277,-256,78,-1221,-256,22,-1189,-256,30,-1277,-256,22,-1189,-256,22,-1381,-248,62,-1325,-25`
+`6,30,-1277,-256,22,-1381,-248,262,-1293,-415,262,-1261,-415,246,-1253,-416,190,-1301,-416,230,-1357,-416,246,-1317,-416,198,-1317,-416,46,-1357,-416,190,-1301,-416,246,-1253,-416,246,-1189,-416,22,-1189,-416,22,-1341,-416,46,-1357,-416,46,-1357,-416,198,-1317,-416,190,-1301,-416,302,-1317,-640,302,-1277,-640,22,-1277,-640,22,-1325,-640,22,-1325,-488,166,-1317,-488,22,-1293,-488,22,-1189,-768,22,-1245,-768,246,-1245,-768,246,-1189,-768,54,-1397,-416,30,-1413,-416,30,-1437,-416,118,-1429,-416,174,`
+`-1397,-416,118,-1429,-416,174,-1445,-416,174,-1397,-416,78,-1245,-160,54,-1277,-160,62,-1301,-160,110,-1301,-160,118,-1269,-160,534,-1365,-642,534,-1341,-642,502,-1357,-642,486,-1341,-642,438,-1341,-642,430,-1357,-642,502,-1357,-642,534,-1365,-642,502,-1357,-642,430,-1357,-642,310,-1357,-641,134,-1357,-641,86,-1357,-641,86,-1509,-641,142,-1341,-640,134,-1357,-641,86,-1509,-641,302,-1509,-641,310,-1357,-641,310,-1357,-641,430,-1357,-642,414,-1341,-642,142,-1341,-640,118,-1373,-776,94,-1381,-776,1`
+`18,-1405,-776,118,-1469,-776,94,-1493,-776,126,-1493,-770,118,-1469,-776,126,-1493,-770,150,-1517,-767,246,-1517,-767,294,-1493,-776,262,-1477,-776,246,-1517,-767,262,-1477,-776,262,-1405,-776,118,-1405,-776,118,-1469,-776,150,-1517,-767,246,-1517,-767,262,-1405,-776,118,-1373,-776,262,-1405,-776,294,-1397,-776,294,-1285,-769,262,-1277,-776,118,-1309,-776,118,-1373,-776,262,-1277,-776,94,-1285,-776,118,-1309,-776,198,-1517,-416,222,-1509,-416,198,-1493,-416,246,-1445,-416,230,-1445,-416,222,-147`
+`7,-416,246,-1493,-416,222,-1397,-416,222,-1421,-416,246,-1421,-416,246,-1397,-416,254,-1605,-608,230,-1597,-608,230,-1613,-608,246,-1629,-608,230,-1453,-144,230,-1477,-146,246,-1477,-140,246,-1453,-139,238,-1661,-608,254,-1645,-608,238,-1637,-608,302,-1189,-625,262,-1189,-625,262,-1229,-625,302,-1245,-625,262,-1229,-625,238,-1253,-628,302,-1245,-625,254,-1685,-608,294,-1677,-608,262,-1669,-608,270,-1621,-608,254,-1645,-608,278,-1653,-608,294,-1629,-608,262,-1541,-608,310,-1549,-608,318,-1533,-60`
+`8,262,-1261,-415,262,-1293,-415,278,-1301,-416,350,-1445,-416,350,-1469,-416,406,-1469,-416,406,-1397,-416,502,-1397,-416,502,-1453,-416,534,-1453,-416,534,-1205,-416,278,-1301,-416,278,-1341,-416,318,-1365,-416,278,-1221,-416,262,-1261,-415,278,-1301,-416,318,-1365,-416,326,-1221,-416,318,-1365,-416,326,-1437,-416,350,-1445,-416,406,-1397,-416,422,-1381,-416,334,-1205,-416,326,-1221,-416,318,-1365,-416,422,-1381,-416,486,-1381,-416,534,-1205,-416,486,-1381,-416,502,-1397,-416,534,-1205,-416,534`
+`,-1517,74,534,-1461,77,414,-1453,75,278,-1517,74,398,-1253,74,390,-1189,74,278,-1189,74,422,-1341,75,534,-1341,75,534,-1253,74,398,-1253,74,414,-1453,75,422,-1341,75,398,-1253,74,278,-1189,74,278,-1517,74,278,-1413,-743,278,-1469,-736,294,-1469,-735,294,-1413,-743,286,-1189,-128,286,-1245,-128,302,-1253,-128,318,-1189,-128,286,-1317,-128,302,-1501,-128,318,-1501,-128,318,-1189,-128,302,-1253,-128,302,-1237,-760,302,-1189,-760,286,-1189,-760,286,-1237,-760,294,-1581,-608,294,-1605,-608,310,-1613,`
+`-608,334,-1565,-593,310,-1613,-608,302,-1669,-608,318,-1677,-608,422,-1581,-540,390,-1573,-556,334,-1565,-593,390,-1541,-561,334,-1541,-593,334,-1565,-593,390,-1573,-556,294,-1549,-368,294,-1565,-368,318,-1565,-368,318,-1549,-368,374,-1581,-416,358,-1549,-416,302,-1581,-416,374,-1581,-416,302,-1581,-416,310,-1701,-416,470,-1701,-416,470,-1581,-416,342,-1541,-748,318,-1565,-764,326,-1669,-759,534,-1669,-640,534,-1557,-640,494,-1541,-652,534,-1357,-768,534,-1189,-770,342,-1189,-768,334,-1357,-768,`
+`342,-1685,-588,462,-1685,-513,462,-1589,-513,534,-1493,-177,534,-1477,-177,342,-1477,-176,342,-1501,-177,534,-1493,-128,534,-1469,-128,342,-1469,-128,342,-1501,-128,382,-1253,-633,382,-1189,-636,342,-1189,-636,342,-1317,-640,406,-1277,-640,382,-1253,-633,342,-1317,-640,534,-1317,-640,534,-1277,-640,406,-1277,-640,342,-1317,-640,342,-1317,-488,534,-1317,-488,534,-1277,-488,342,-1277,-488,374,-1549,-368,374,-1565,-368,470,-1565,-368,470,-1549,-368,414,-1549,-540,446,-1573,-524,462,-1541,-513,454,-`
+`1581,-768,438,-1541,-768,422,-1541,-768,422,-1677,-768,486,-1541,-768,462,-1541,-768,454,-1581,-768,486,-1677,-768,486,-1541,-768,454,-1581,-768,422,-1677,-768,438,-1221,74,438,-1189,74,422,-1189,74,422,-1221,74,438,-1453,-608,534,-1453,-608,534,-1413,-608,438,-1413,-608,438,-1437,92,534,-1437,92,534,-1365,92,438,-1365,92,462,-1237,149,534,-1237,132,534,-1189,131,454,-1189,149,486,-1213,74,486,-1189,74,462,-1189,74,462,-1221,74,486,-1701,-224,526,-1701,-235,534,-1637,-239,534,-1549,-239,486,-154`
+`9,-224,494,-1453,-526,534,-1453,-526,534,-1413,-526,494,-1413,-526,534,-1549,-416,518,-1549,-416,510,-1597,-416,510,-1701,-416,534,-1701,-416],"vertslength":414,"polys":[0,2,3,5,6,8,9,13,14,17,18,23,24,26,27,30,31,34,35,37,38,43,44,46,47,49,50,53,54,57,58,63,64,67,68,70,71,74,75,77,78,83,84,87,88,91,92,94,95,97,98,100,101,104,105,108,109,114,115,117,118,121,122,124,125,128,129,133,134,136,137,141,142,144,145,148,149,152,153,155,156,160,161,164,165,167,168,170,171,173,174,176,177,179,180,185,186,`
+`191,192,194,195,197,198,201,202,205,206,209,210,213,214,216,217,220,221,223,224,226,227,230,231,233,234,236,237,240,241,244,245,247,248,252,253,257,258,263,264,266,267,270,271,273,274,277,278,282,283,286,287,290,291,295,296,299,300,303,304,309,310,313,314,317,318,320,321,325,326,331,332,335,336,338,339,342,343,346,347,350,351,353,354,357,358,361,362,365,366,368,369,372,373,375,376,379,380,383,384,387,388,391,392,395,396,399,400,404,405,408,409,413],"polyslength":105,"regions":[2,2,2,2,1,1,1,13,1`
+`3,13,13,13,13,6,6,6,33,10,10,10,10,10,10,10,10,10,5,5,5,5,24,34,19,25,25,21,4,4,4,4,4,4,7,7,7,7,7,7,7,7,39,45,46,48,49,50,26,26,53,30,54,3,3,3,3,3,3,3,3,9,9,9,9,62,31,31,64,14,14,14,65,12,12,11,8,17,67,32,16,16,16,27,68,69,18,18,18,71,28,15,22,74,23,29,75],"neighbors":[[[0],[0],[1,3]],[[0],[0],[1,3]],[[0],[0],[1,3]],[[1,1],[1,0],[1,2],[0],[0]],[[0],[0],[1,5],[0]],[[1,4],[0],[0],[0],[0],[1,6]],[[0],[0],[1,5]],[[0],[0],[0],[1,10]],[[0],[0],[0],[1,10]],[[0],[0],[1,10]],[[1,9],[1,7],[1,11],[1,8],[1,`
+`12],[0]],[[0],[0],[1,10]],[[0],[0],[1,10]],[[0],[1,47],[0],[1,15]],[[0],[0],[1,15],[0]],[[1,83],[0],[1,13],[1,14],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[1,18]],[[0],[0],[1,17],[1,20]],[[0],[1,22],[1,21]],[[0],[1,18],[0],[1,22],[0],[1,25]],[[0],[1,19],[0],[1,23]],[[1,19],[0],[1,20],[0]],[[0],[1,21],[1,24]],[[1,23],[0],[1,25]],[[0],[1,24],[1,20]],[[1,61],[0],[1,28],[0]],[[0],[0],[1,29],[0]],[[1,26],[0],[0],[0],[0],[1,29]],[[1,27],[0],[1,28]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0]`
+`,[0],[1,34],[0]],[[0],[0],[1,33]],[[0],[0],[0],[0],[0]],[[0],[0],[1,38]],[[0],[0],[1,38],[0]],[[1,36],[1,37],[1,41],[0]],[[0],[0],[1,40]],[[0],[1,39],[0],[0],[1,41]],[[1,38],[0],[0],[1,40]],[[0],[0],[1,47]],[[0],[0],[1,44]],[[1,43],[0],[1,47]],[[0],[0],[1,46]],[[1,45],[0],[1,47]],[[0],[1,44],[1,13],[1,46],[1,48],[1,42]],[[0],[0],[0],[1,49],[0],[1,47]],[[0],[0],[1,48]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[1,57],[0]],[[0],[0]`
+`,[1,56]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[1,26],[0],[1,65]],[[0],[0],[0],[1,66]],[[0],[0],[0],[1,68]],[[0],[0],[1,65]],[[0],[1,61],[1,64],[1,67],[0]],[[0],[0],[1,62],[0],[1,67]],[[0],[1,65],[1,66],[0],[1,68],[0]],[[0],[1,63],[1,67]],[[0],[0],[1,72],[0]],[[0],[0],[1,72]],[[0],[0],[0],[1,72]],[[0],[1,71],[1,70],[0],[1,69]],[[0],[0],[0],[0]],[[0],[0],[1,75],[0]],[[0],[0],[0],[1,74],[0]],[[0],[0],[0],[0]],[[0],[0],[1,78],[0]],[[0],[0],[0],[0],[1,79],[1,77]],[[0],[0],[1,78],[0]],[[0],[`
+`0],[0],[0]],[[0],[0],[1,82]],[[1,81],[0],[0],[0],[0]],[[0],[1,15],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[1,89]],[[0],[1,88],[1,90]],[[0],[0],[1,89],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0]],[[0],[0],[0],[1,96]],[[0],[0],[1,96]],[[0],[1,95],[1,94],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]],[[0],[0],[0],[0]],[[0],[0],[0],[0],[0]]]},"detail":{"verts":[24`
+`6,-1573,-416,246,-1541,-416,174,-1533,-416,278,-1701,-416,278,-1589,-416,246,-1573,-416,174,-1533,-416,174,-1477,-416,22,-1477,-416,278,-1701,-416,246,-1573,-416,174,-1533,-416,22,-1477,-416,22,-1701,-416,246,-1437,-55,190,-1437,-55,182,-1461,-55,246,-1541,-44,246,-1541,-44,182,-1461,-55,134,-1445,-46,22,-1453,-46,22,-1565.727294921875,-21,22,-1701,-19,180.6666717529297,-1701,-21,294,-1701,-46,106,-1569,-19,178,-1617,-22,294,-1701,-46,278,-1549,-47,246,-1541,-44,166,-1621,-608,190,-1621,-608,190`
+`,-1605,-608,158,-1597,-608,86,-1573,-608,86,-1549,-608,62,-1533,-608,54,-1573,-608,166,-1685,-608,182,-1669,-608,166,-1621,-608,166,-1685,-608,166,-1621,-608,158,-1597,-608,86,-1573,-608,54,-1573,-608,22,-1685,-608,158,-1597,-608,174,-1573,-608,86,-1573,-608,54,-1573,-608,22,-1549,-608,22,-1685,-608,254,-1549,-768,246,-1517,-767,150,-1517,-767,142,-1549,-768,22,-1565,-768,22,-1661,-768,94,-1669,-768,142,-1549,-768,326,-1669,-764,318,-1565,-768,254,-1549,-768,142,-1549,-768,94,-1669,-768,102,-168`
+`5,-768,246,-1421,-90,246,-1405,-90,66.80000305175781,-1405,-90,22,-1405,-97,22,-1429,-95,118,-1429,-246,174,-1445,-246,182,-1421,-246,38,-1389,-248,30,-1437,-247,118,-1429,-246,182,-1421,-246,110,-1229,-256,142,-1269,-256,246,-1189,-256,22,-1381,-248,38,-1389,-248,182,-1421,-246,246,-1421,-248,110,-1325,-256,62,-1325,-256,78,-1221,-256,110,-1229,-256,246,-1189,-256,22,-1189,-256,246,-1189,-256,142,-1269,-256,110,-1325,-256,246,-1421,-248,246,-1374.5999755859375,-248,246,-1351.4000244140625,-256,`
+`30,-1277,-256,78,-1221,-256,22,-1189,-256,30,-1277,-256,22,-1189,-256,22,-1338.3333740234375,-256,22,-1381,-248,23.600000381469727,-1360.199951171875,-255,62,-1325,-256,30,-1277,-256,23.600000381469727,-1360.199951171875,-255,22,-1381,-248,262,-1293,-415,262,-1261,-415,246,-1253,-416,190,-1301,-416,230,-1357,-416,246,-1317,-415,198,-1317,-416,46,-1357,-416,190,-1301,-416,246,-1253,-416,246,-1189,-416,22,-1189,-416,22,-1341,-416,46,-1357,-416,46,-1357,-416,198,-1317,-416,190,-1301,-416,302,-1317,`
+`-640,302,-1277,-640,22,-1277,-640,22,-1325,-640,22,-1325,-488,166,-1317,-488,22,-1293,-488,22,-1189,-768,22,-1245,-768,246,-1245,-768,246,-1189,-768,54,-1397,-416,30,-1413,-416,30,-1437,-416,118,-1429,-416,174,-1397,-416,118,-1429,-416,174,-1445,-416,174,-1397,-416,78,-1245,-160,54,-1277,-160,62,-1301,-160,110,-1301,-160,118,-1269,-160,534,-1365,-642,534,-1341,-642,502,-1357,-642,486,-1341,-642,438,-1341,-642,430,-1357,-642,502,-1357,-642,534,-1365,-642,502,-1357,-642,430,-1357,-642,310,-1357,-6`
+`46,134,-1357,-640,86,-1357,-641,86,-1509,-641,142,-1341,-640,134,-1357,-640,86,-1509,-641,302,-1509,-641,310,-1357,-646,310,-1357,-646,430,-1357,-642,414,-1341,-642,142,-1341,-640,118,-1373,-776,94,-1381,-776,118,-1405,-776,118,-1469,-776,94,-1493,-776,126,-1493,-776,118,-1469,-776,126,-1493,-776,150,-1517,-768,246,-1517,-770,294,-1493,-776,262,-1477,-776,246,-1517,-770,262,-1477,-776,262,-1405,-776,118,-1405,-776,118,-1469,-776,150,-1517,-768,246,-1517,-770,262,-1405,-776,118,-1373,-776,154,-14`
+`81,-776,262,-1405,-776,294,-1397,-776,294,-1285,-769,262,-1277,-776,118,-1309,-776,118,-1373,-776,262,-1277,-776,94,-1285,-776,118,-1309,-776,198,-1517,-416,222,-1509,-416,198,-1493,-416,246,-1445,-416,230,-1445,-416,222,-1477,-416,246,-1493,-416,222,-1397,-416,222,-1421,-416,246,-1421,-416,246,-1397,-416,254,-1605,-608,230,-1597,-608,230,-1613,-608,246,-1629,-608,230,-1453,-139,230,-1477,-137,246,-1477,-137,246,-1453,-139,238,-1661,-608,254,-1645,-608,238,-1637,-608,302,-1189,-625,262,-1189,-62`
+`5,262,-1229,-625,302,-1245,-625,262,-1229,-625,238,-1253,-628,302,-1245,-625,254,-1685,-608,294,-1677,-608,262,-1669,-608,270,-1621,-608,254,-1645,-608,278,-1653,-608,294,-1629,-608,262,-1541,-608,310,-1549,-608,318,-1533,-608,262,-1261,-416,262,-1293,-416,278,-1301,-416,350,-1445,-416,350,-1469,-416,406,-1469,-416,406,-1397,-416,502,-1397,-416,502,-1453,-416,534,-1453,-416,534,-1205,-416,278,-1301,-416,278,-1341,-416,318,-1365,-416,278,-1221,-416,262,-1261,-416,278,-1301,-416,318,-1365,-416,326`
+`,-1221,-416,318,-1365,-416,326,-1437,-416,350,-1445,-416,406,-1397,-416,422,-1381,-416,334,-1205,-416,326,-1221,-416,318,-1365,-416,422,-1381,-416,486,-1381,-416,534,-1205,-416,486,-1381,-416,502,-1397,-416,534,-1205,-416,534,-1517,74,534,-1461,77,414,-1453,74,278,-1517,74,398,-1253,74,390,-1189,74,278,-1189,74,422,-1341,74,534,-1341,74,534,-1253,74,398,-1253,74,414,-1453,74,422,-1341,74,398,-1253,74,278,-1189,74,278,-1517,74,278,-1413,-743,278,-1450.3333740234375,-743,278,-1469,-734,294,-1469,-`
+`734,294,-1450.3333740234375,-743,294,-1413,-743,286,-1189,-128,286,-1245,-128,302,-1253,-128,318,-1189,-128,286,-1317,-128,302,-1501,-128,318,-1501,-128,318,-1189,-128,302,-1253,-128,302,-1237,-760,302,-1189,-760,286,-1189,-760,286,-1237,-760,294,-1581,-608,294,-1605,-608,310,-1613,-604,334,-1565,-588,310,-1613,-604,302,-1669,-608,318,-1677,-599,422,-1581,-540,406,-1577,-540,390,-1573,-561,334,-1565,-588,390,-1541,-561,334,-1541,-588,334,-1565,-588,390,-1573,-561,294,-1549,-368,294,-1565,-368,31`
+`8,-1565,-368,318,-1549,-368,374,-1581,-416,358,-1549,-416,302,-1581,-416,374,-1581,-416,302,-1581,-416,310,-1701,-416,470,-1701,-416,470,-1581,-416,342,-1541,-743,318,-1565,-753,319.6000061035156,-1585.800048828125,-759,326,-1669,-753,349.1111145019531,-1669,-743,487.77777099609375,-1669,-647,534,-1669,-640,534,-1557,-640,514,-1549,-640,494,-1541,-652,472.28570556640625,-1541,-657,498,-1561,-641,534,-1357,-768,534,-1189,-770,342,-1189,-768,334,-1357,-768,342,-1685,-583,422,-1685,-529,462,-1685,-`
+`513,462,-1589,-513,450,-1601,-513,534,-1493,-177,534,-1477,-177,342,-1477,-176,342,-1501,-177,534,-1493,-128,534,-1469,-128,342,-1469,-128,342,-1501,-128,382,-1253,-636,382,-1189,-636,342,-1189,-636,342,-1274.3333740234375,-633,342,-1317,-640,406,-1277,-640,394,-1265,-628,382,-1253,-636,342,-1317,-640,534,-1317,-640,534,-1277,-640,406,-1277,-640,342,-1317,-640,342,-1317,-488,534,-1317,-488,534,-1277,-488,342,-1277,-488,374,-1549,-368,374,-1565,-368,470,-1565,-368,470,-1549,-368,414,-1549,-535,44`
+`6,-1573,-513,462,-1541,-513,446,-1543.6666259765625,-513,454,-1581,-768,438,-1541,-768,422,-1541,-768,422,-1677,-768,486,-1541,-768,462,-1541,-768,454,-1581,-768,486,-1677,-768,486,-1541,-768,454,-1581,-768,422,-1677,-768,438,-1221,74,438,-1189,74,422,-1189,74,422,-1221,74,438,-1453,-608,534,-1453,-608,534,-1413,-608,438,-1413,-608,438,-1437,92,534,-1437,92,534,-1365,92,438,-1365,92,462,-1237,145,534,-1237,131,534,-1189,131,454,-1189,147,486,-1213,74,486,-1189,74,462,-1189,74,462,-1221,74,486,-1`
+`701,-224,526,-1701,-235,534,-1637,-239,534,-1549,-239,486,-1549,-224,494,-1453,-526,534,-1453,-526,534,-1413,-526,494,-1413,-526,534,-1549,-416,518,-1549,-416,510,-1597,-416,510,-1701,-416,534,-1701,-416],"vertslength":439,"tris":[0,1,2,3,4,5,6,7,8,9,10,11,11,12,13,9,11,13,14,15,16,14,16,17,18,19,26,20,19,26,22,21,26,20,21,26,22,23,26,18,26,27,24,25,27,18,25,27,26,23,27,24,23,27,28,29,30,31,32,33,31,33,34,38,35,36,36,37,38,39,40,41,42,43,44,44,45,46,42,44,46,42,46,47,48,49,50,51,52,53,54,55,56,5`
+`4,56,57,58,59,60,58,60,61,65,66,67,62,63,64,64,65,67,62,64,67,70,71,72,68,69,70,68,70,72,73,74,75,76,77,78,76,78,79,80,81,82,88,83,84,87,88,84,85,86,87,84,85,87,92,89,90,90,91,92,95,96,97,95,97,98,94,95,98,93,94,98,99,100,101,104,105,106,104,106,102,102,103,104,109,110,107,107,108,109,111,112,113,111,113,114,115,116,117,115,117,118,119,120,121,122,123,124,122,124,119,119,121,122,125,126,127,128,129,130,128,130,131,132,133,134,138,135,136,136,137,138,139,140,141,139,141,142,139,142,143,144,145,14`
+`6,147,148,149,150,151,147,147,149,150,152,153,154,155,156,157,155,157,158,159,160,161,159,161,162,163,164,165,166,167,168,169,170,166,166,168,169,171,172,173,171,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,189,194,195,196,192,193,196,194,193,196,195,190,196,192,191,196,190,191,196,197,198,199,197,199,200,200,201,202,197,200,202,203,204,205,206,207,208,209,210,211,209,211,212,216,213,214,214,215,216,217,218,219,217,219,220,224,221,222,222,223,224,225,226,227,228,229,230,228,23`
+`0,231,232,233,234,235,236,237,238,239,240,238,240,241,242,243,244,245,246,247,248,249,250,248,250,251,252,253,254,252,254,255,256,257,258,259,260,261,263,259,261,261,262,263,264,265,266,266,267,268,264,266,268,269,270,271,271,272,273,269,271,273,269,273,274,275,276,277,278,279,280,278,280,281,282,283,284,285,286,287,285,287,288,289,290,291,289,291,292,289,292,293,295,296,297,295,297,298,299,294,295,295,298,299,300,301,302,300,302,303,304,305,306,308,304,306,306,307,308,312,309,310,310,311,312,31`
+`3,314,315,313,315,316,317,318,319,321,322,323,320,321,323,323,317,319,319,320,323,324,325,326,324,326,327,331,328,329,329,330,331,332,333,334,335,336,337,338,339,335,335,337,338,340,341,342,342,343,344,340,342,344,350,340,344,344,345,350,350,345,351,345,346,351,347,346,351,347,348,351,350,349,351,348,349,351,352,353,354,352,354,355,356,357,360,359,356,360,357,358,360,359,358,360,361,362,363,361,363,364,365,366,367,365,367,368,372,373,369,369,370,371,369,371,372,377,374,375,375,376,377,378,379,38`
+`0,378,380,381,385,382,383,383,384,385,389,386,387,387,388,389,391,392,393,390,391,393,394,395,396,394,396,397,398,399,400,403,404,401,401,402,403,408,405,406,406,407,408,412,409,410,410,411,412,416,413,414,414,415,416,417,418,419,417,419,420,421,422,423,421,423,424,425,426,427,427,428,429,425,427,429,433,430,431,431,432,433,434,435,436,436,437,438,434,436,438],"trislength":234,"triTopoly":[0,1,2,3,3,3,4,4,5,5,5,5,5,5,5,5,5,5,6,7,7,8,8,9,10,10,10,10,11,12,13,13,14,14,15,15,15,15,16,16,16,17,18,18`
+`,19,20,20,20,20,21,21,22,22,22,22,23,24,24,24,25,25,26,26,27,27,28,28,28,28,29,30,30,31,32,32,33,33,33,34,35,35,35,36,37,37,38,38,39,40,40,40,41,41,42,43,44,45,46,47,47,47,47,47,47,48,48,48,48,49,50,51,51,52,52,53,53,54,54,55,56,56,57,58,59,59,60,61,62,62,63,63,64,65,65,65,66,66,66,67,67,67,67,68,69,69,70,71,71,72,72,72,73,73,73,73,74,74,75,75,75,76,76,77,77,78,78,78,78,78,79,79,80,80,81,82,82,82,83,83,83,83,83,83,83,83,83,83,83,84,84,85,85,85,85,86,86,87,87,88,88,88,89,89,90,90,91,91,92,92,93,9`
+`3,94,94,95,96,96,97,97,98,98,99,99,100,100,101,101,102,102,102,103,103,104,104,104],"baseVert":[0,3,6,9,14,18,28,31,35,39,42,48,51,54,58,62,68,73,76,80,83,89,93,99,102,107,111,115,119,125,128,132,135,139,144,147,152,155,159,163,166,171,175,178,181,184,187,190,197,203,206,209,213,217,221,225,228,232,235,238,242,245,248,252,256,259,264,269,275,278,282,285,289,294,300,304,309,313,317,324,328,332,335,340,352,356,361,365,369,374,378,382,386,390,394,398,401,405,409,413,417,421,425,430,434],"vertsCount`
+`":[3,3,3,5,4,10,3,4,4,3,6,3,3,4,4,6,5,3,4,3,6,4,6,3,5,4,4,4,6,3,4,3,4,5,3,5,3,4,4,3,5,4,3,3,3,3,3,7,6,3,3,4,4,4,4,3,4,3,3,4,3,3,4,4,3,5,5,6,3,4,3,4,5,6,4,5,4,4,7,4,4,3,5,12,4,5,4,4,5,4,4,4,4,4,4,3,4,4,4,4,4,4,5,4,5],"baseTri":[0,1,2,3,6,8,18,19,21,23,24,28,29,30,32,34,38,41,42,44,45,49,51,55,56,59,61,63,65,69,70,72,73,75,78,79,82,83,85,87,88,91,93,94,95,96,97,98,104,108,109,110,112,114,116,118,119,121,122,123,125,126,127,129,131,132,135,138,142,143,145,146,148,151,155,157,160,162,164,169,171,173`
+`,174,177,188,190,194,196,198,201,203,205,207,209,211,213,214,216,218,220,222,224,226,229,231],"triCount":[1,1,1,3,2,10,1,2,2,1,4,1,1,2,2,4,3,1,2,1,4,2,4,1,3,2,2,2,4,1,2,1,2,3,1,3,1,2,2,1,3,2,1,1,1,1,1,6,4,1,1,2,2,2,2,1,2,1,1,2,1,1,2,2,1,3,3,4,1,2,1,2,3,4,2,3,2,2,5,2,2,1,3,11,2,4,2,2,3,2,2,2,2,2,2,1,2,2,2,2,2,2,3,2,3]},"links":{"poly":[4,16,30,57,40,60,45,73,53,59,53,55,55,58,55,59,58,59,58,78,59,77,60,77,71,100,72,99,78,85,78,93,80,81,81,92,85,93,97,101],"cost":[2221.5,1080,2649.179931640625,270`
+`7.199951171875,361.8461608886719,172.8000030517578,768,384,564.7058715820312,192,691.2000122070312,977.4720458984375,5430,901.8045654296875,1036.07177734375,806.2463989257812,3509.169189453125,3532.800048828125,902.1210327148438,864],"type":[2,1,2,2,1,1,1,1,1,1,1,1,2,2,2,1,2,2,1,1],"pos":[246,-1437,-55,246,-1421,-90,238,-1277,-640,238,-1253,-628,302,-1509,-641,305.67999267578125,-1534.760009765625,-608,271.6000061035156,-1481.800048828125,-776,278,-1469,-736,246,-1629,-608,258.9230651855469,-163`
+`7.6153564453125,-608,246,-1629,-608,241.1999969482422,-1638.5999755859375,-608,246,-1653,-608,262,-1669,-608,238,-1645,-608,254,-1645,-608,273.29412841796875,-1671.823486328125,-608,278,-1653,-608,294,-1677,-608,302,-1669,-608,294,-1629,-608,303.6000061035156,-1609.800048828125,-608,310,-1549,-608,318.82757568359375,-1571.0689697265625,-598.6896362304688,534,-1253,74,534,-1237,132,420.3756408691406,-1363.7410888671875,75,438,-1365,92,326.97125244140625,-1668.7188720703125,-602.1341552734375,342,`
+`-1685,-588,422,-1581,-540,434.4800109863281,-1564.3599853515625,-529.760009765625,318,-1565,-368,320.953857421875,-1570.169189453125,-416,367.6000061035156,-1568.199951171875,-416,374,-1565,-368,460.04876708984375,-1590.56103515625,-514.219482421875,446,-1573,-524,438,-1221,74,462,-1221,74],"length":20}}]]}`
;
    }
}

/** @typedef {import("../path_tilemanager").TileData} TileData */
class NVplugin {
    /**
     * @param {NavMesh} nav
     */
    constructor(nav) {
        this.nav=nav;
        //aaabbccc_1_2,tiledata
        //aaabbccc_2_2,tiledata
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();
        /** @type {Map<string, TileData>} */
        this.deftiles=new Map();
        this.up=0;//下一tick更新什么
        /**
         * @type {{name:string, td: TileData; }[]}
         */
        this.updata=[];
        /** @type {TileManager} */
        this.tileManager;
        this.importNavData(new NVpluginStaticData().Data,new StaticData().Data);
    }
    /**
     * @param {TileManager} tilemanager
     * @param {tile} tile
     */
    init(tilemanager,tile)
    {
        /** @type {TileManager} */
        this.tileManager = tilemanager;

        /** @type {tile} */
        this.tile=tile;
        
        Instance.OnScriptInput("addtile",(e)=>{
            Instance.Msg("addtile");
            if(!e.caller)return;
            let name=e.caller?.GetEntityName();
            if (!name.startsWith("navmesh_")) return;
            const sp=name.split("_");
            if(sp.length<3)return;
            this.addTile(sp[1],sp.slice(2));
        });
        Instance.OnScriptInput("settile",(e)=>{
            if(!e.caller)return;
            let name=e.caller?.GetEntityName();
            if (!name.startsWith("navmesh_")) return;
            const sp=name.split("_");
            if(sp.length<3)return;
            if(sp[1]=="default")this.setTile(sp[1],sp.slice(2),true);
            else this.setTile(sp[1],sp.slice(2));
        });
    }
    /**
     * 导出导航网格数据为文本字符串
     */
    exportNavData() {
        const charsPerLine = 500;
        const data = {
            tiles: Array.from(this.tiles, ([key, td]) => [key, Tool._compactTileData(td)])
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
     * @param {string} defaultJsonStr
     */
    importNavData(jsonStr, defaultJsonStr) {
        try {
            const cleanJson = jsonStr.replace(/\s/g, "");
            if(cleanJson.length==0)throw new Error("空数据");
            const data = JSON.parse(cleanJson);
            for (const [key,value] of data.tiles) {
                const mesh = Tool.toTypedMesh(value.mesh);
                const detail = Tool.toTypedDetail(value.detail);
                const links = Tool.toTypedLinks(value.links);
                value.mesh=mesh;
                value.detail=detail;
                value.links=links;
                this.tiles.set(key, value);
            }

            const dfcleanJson = defaultJsonStr.replace(/\s/g, "");
            if(dfcleanJson.length==0)throw new Error("空数据");

            const dfdata = JSON.parse(dfcleanJson);

            for (const [key,value] of dfdata.tiles) {
                const mesh = Tool.toTypedMesh(value.mesh);
                const detail = Tool.toTypedDetail(value.detail);
                const links = Tool.toTypedLinks(value.links);
                value.mesh=mesh;
                value.detail=detail;
                value.links=links;
                this.deftiles.set(value.tileId, value);
            }
            Instance.Msg(`加载备选数据成功！`);
            return true;
        } catch (e) {
            Instance.Msg(`加载备选数据失败: ${e}`);
            return false;
        }
    }
    /**
     * @param {string} name
     * @param {string[]} key
     */
    addTile(name,key)
    {
        if(!this.tile)return;
        for(let k of key)
        {
            k=k.replace("-","_");
            const tx=parseInt(k.split("_")[0]);
            const ty=parseInt(k.split("_")[1]);
            //这里获取数据
            this.tiles.set(name+"_"+k,this.tile.buildTile(tx, ty));
        }
        this.exportNavData();
    }
    tick()
    {
        let start=new Date();
        switch(this.up)
        {
            case 0:
                if(this.updata.length!=0)this.up=1;
                break;
            case 1:
                //5ms;
                this.tileManager.removetile(this.updata[0].name);
                const td=this.updata[0].td;
                this.tileManager.tiles.set(td.tileId, {
                    tileId: td.tileId,
                    tx: td.tx,
                    ty: td.ty,
                    mesh: td.mesh,
                    detail: td.detail,
                    links: td.links
                });
                this.tileManager._appendTileData(td.tileId, td.mesh, td.detail, td.links);
                this.up++;
                break;
            case 2:
                //1ms
                const { edgeCount, result, tilemark } = this.tileManager.getedgebytileid(this.updata[0].name);
                this.edgeCount=edgeCount;
                this.result=result;
                this.tilemark=tilemark;
                this.up++;
                break;
            case 3:
                //7ms
                if(this.edgeCount&&this.result&&this.tilemark)this.tileManager.Extlink = new JumpLinkBuilder(this.tileManager.mesh).initInterTileIn(this.edgeCount,this.result,this.tilemark,this.tileManager.Extlink);
                this.up++;
                break;
            case 4:
                //3ms
                Tool.buildSpatialIndex(this.tileManager.mesh);
                this.up++;
                break;
            case 5:
                //2ms
                this.tileManager.supprlink= this.tileManager.buildSupperLinksForMesh(this.tileManager.mesh);
                let merged = this.tileManager.copyLinks(this.tileManager.baseLinks, this.tileManager.Extlink);
                merged = this.tileManager.copyLinks(merged, this.tileManager.supprlink);
                this.tileManager.links = merged;
                this.up++;
                break;
            case 6:
                //7ms
                this.tileManager.pruneUnreachablePolys();
                this.tileManager.updatemesh();
                this.nav._refreshRuntime();
                this.updata.shift();
                this.up++;
                break;
            default:
                this.up=0;
                break;
        }
        let end=new Date();
        if(this.up>0)Instance.Msg(`${this.up} ${end.getTime() - start.getTime()} ms`);
    }
    /**
     * @param {string} name
     * @param {string[]} key
     * @param {boolean} [pre] //是否恢复默认设置
     */
    setTile(name,key,pre=false)
    {
        if(!this.tileManager)return;
        if(pre)
        {   //2-3->2_3
            for(let k of key)
            {
                k=k.replace("-","_");
                const td=this.deftiles.get(k);
                if(!td)continue;
                //这里替换数据
                this.updata.push({name:k,td:td});
            }
        }
        else
        {
            for(let k of key)
            {
                k=k.replace("-","_");
                let td=this.tiles.get(name+"_"+k);
                if(!td)
                {//没有数据，必定是开发环境
                    this.addTile(name,[k]);
                    td=this.tiles.get(name+"_"+k);
                }
                if(!td)continue;
                //这里替换数据
                this.updata.push({name:k,td:td});
            }
        }
    }
}

/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_tilemanager").TileData} TileData */
/**
 * @typedef {{
 *  verts: Float32Array<ArrayBufferLike>,
 *  vertslength: number,
 *  polys: Int32Array<ArrayBufferLike>,
 *  polyslength: number,
 *  regions: Int16Array<ArrayBufferLike>,
 *  neighbors: Int16Array<ArrayBufferLike>[][]
 * }} NavMeshMesh
 */

/**
 * @typedef {{
 *  verts: Float32Array<ArrayBufferLike>,
 *  vertslength: number,
 *  tris: Uint16Array<ArrayBufferLike>,
 *  trislength: number,
 *  triTopoly: Uint16Array<ArrayBufferLike>,
 *  baseVert: Uint16Array<ArrayBufferLike>,
 *  vertsCount: Uint16Array<ArrayBufferLike>,
 *  baseTri: Uint16Array<ArrayBufferLike>,
 *  triCount: Uint16Array<ArrayBufferLike>
 * }} NavMeshDetail
 */

/**
 * @typedef {{
 *  poly: Uint16Array,
 *  cost: Float32Array,
 *  type: Uint8Array,
 *  pos: Float32Array,
 *  length: number
 * }} NavMeshLink
 */

/**
 * @typedef {{
 *  PolyA:number,
 *  PolyB:number,
 *  PosA:Vector,
 *  PosB:Vector,
 *  cost:number,
 *  type:number
 * }} NavMeshLinkARRAY
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
        /**@type {NavMeshLink} */
        this.links;
        /** @type {TileManager} */
        this.tileManager = new TileManager(this);
        /** @type {tile} */
        this.tile = new tile();
        this.debugTools = new NavMeshDebugTools(this);
        this.plugin=new NVplugin(this);
        //删除prop_door_rotating实体？也许应该弄一个目录，让作者把门一类的实体名字放里面
    }
    /**
     * 导出导航网格数据为文本字符串
     */
    exportNavData() {
        const charsPerLine = 500;
        const data = {
            tiles: Array.from(this.tileManager.tiles, ([key, td]) => [key, Tool._compactTileData(td)])
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
            for (const tile of data.tiles) {
                const tiledata=tile[1];
                const key = tiledata.tileId;
                const mesh = Tool.toTypedMesh(tiledata.mesh);
                const detail = Tool.toTypedDetail(tiledata.detail);
                const links = Tool.toTypedLinks(tiledata.links);
                this.tileManager.tiles.set(key, {
                    tileId: key,
                    tx: tiledata.tx,
                    ty: tiledata.ty,
                    mesh: mesh,
                    detail: detail,
                    links: links
                });
                this.tileManager._appendTileData(key, mesh, detail, links);
                this.tileManager._rebuildDeferredLinks(true,false,key);
            }
            this.tileManager._rebuildDeferredLinks(false,true);
            if (TILE_OPTIMIZATION_1)this.tileManager.pruneUnreachablePolys();
            this.tileManager.updatemesh();
            Instance.Msg(`导航数据加载成功！多边形数量: ${this.mesh.polyslength-1}`);
            return true;
        } catch (e) {
            Instance.Msg(`加载导航数据失败: ${e}`);
            return false;
        }
    }
    //初始化网格
    init() {
        this.tileManager = new TileManager(this);
        {
            this.importNavData(new StaticData().Data);
        }
        this._refreshRuntime();
        this.plugin?.init(this.tileManager,this.tile);
    }
    /**
     * 仅更新 pos 所在 tile 的导航网格。不存在就创建
     * @param {Vector} pos
     */
    update(pos)
    {
        this.tileManager.rebuildAtPos(this.tile, pos);
        this.tileManager.updatemesh();
        this._refreshRuntime();
    }
    _refreshRuntime() {
        Tool.buildSpatialIndex(this.mesh);
        
//        /** @type {Map<number, number>} */
//        const degree = new Map();
//        const globalLinks = this.links;
//        const globalLen = globalLinks?.length ?? 0;
//
//        // 1) 先统计每个 poly 需要多少条 link（双向展开）
//        for (let i = 0; i < globalLen; i++) {
//            const a = globalLinks.poly[i * 2];
//            const b = globalLinks.poly[i * 2 + 1];
//            if (a < 0 || b < 0) continue;
//
//            degree.set(a, (degree.get(a) ?? 0) + 1);
//            degree.set(b, (degree.get(b) ?? 0) + 1);
//        }
//
//        /**@type {NavMeshLink[]} */
//        const links=new Array();
//        // 2) 按统计容量分配，避免固定 32 溢出
//        for (const [poly, cnt] of degree.entries()) {
//            links[poly] = {
//                poly: new Uint16Array(cnt * 2),
//                cost: new Float32Array(cnt),
//                type: new Uint8Array(cnt),
//                pos: new Float32Array(cnt * 6),
//                length: 0
//            };
//        }
//
//        // 3) 写入双向 link（reverse 方向要交换 start/end）
//        for (let i = 0; i < globalLen; i++) {
//            const polyA = globalLinks.poly[i * 2];
//            const polyB = globalLinks.poly[i * 2 + 1];
//            if (polyA < 0 || polyB < 0) continue;
//
//            const cost = globalLinks.cost[i] * OFF_MESH_LINK_COST_SCALE;
//            const type = globalLinks.type[i];
//            const srcPosBase = i * 6;
//
//            const la = links[polyA];
//            const lb = links[polyB];
//            if (!la || !lb) continue;
//
//            // A -> B
//            let wa = la.length;
//            la.poly[wa * 2] = polyA;
//            la.poly[wa * 2 + 1] = polyB;
//            la.cost[wa] = cost;
//            la.type[wa] = type;
//            la.pos[wa * 6] = globalLinks.pos[srcPosBase];
//            la.pos[wa * 6 + 1] = globalLinks.pos[srcPosBase + 1];
//            la.pos[wa * 6 + 2] = globalLinks.pos[srcPosBase + 2];
//            la.pos[wa * 6 + 3] = globalLinks.pos[srcPosBase + 3];
//            la.pos[wa * 6 + 4] = globalLinks.pos[srcPosBase + 4];
//            la.pos[wa * 6 + 5] = globalLinks.pos[srcPosBase + 5];
//            la.length = wa + 1;
//
//            // B -> A（交换端点）
//            let wb = lb.length;
//            lb.poly[wb * 2] = polyB;
//            lb.poly[wb * 2 + 1] = polyA;
//            lb.cost[wb] = cost;
//            lb.type[wb] = type;
//            lb.pos[wb * 6] = globalLinks.pos[srcPosBase + 3];
//            lb.pos[wb * 6 + 1] = globalLinks.pos[srcPosBase + 4];
//            lb.pos[wb * 6 + 2] = globalLinks.pos[srcPosBase + 5];
//            lb.pos[wb * 6 + 3] = globalLinks.pos[srcPosBase];
//            lb.pos[wb * 6 + 4] = globalLinks.pos[srcPosBase + 1];
//            lb.pos[wb * 6 + 5] = globalLinks.pos[srcPosBase + 2];
//            lb.length = wb + 1;
//        }
        /**@type {Map<number,NavMeshLinkARRAY[]>} */
        const links = new Map();
        for (let i = 0; i < this.links.length; i++) {
            const polyA = this.links.poly[i * 2];
            const polyB = this.links.poly[i * 2 + 1];
            if (polyA < 0 || polyB < 0) continue;
            const cost = this.links.cost[i] * OFF_MESH_LINK_COST_SCALE;
            const type = this.links.type[i];
            const srcPosBase = i * 6;
            if (!links.has(polyA)) links.set(polyA, []);
            if (!links.has(polyB)) links.set(polyB, []);
            const link={
                PolyA: polyA,
                PolyB: polyB,
                PosA: {
                    x: this.links.pos[srcPosBase],
                    y: this.links.pos[srcPosBase + 1],
                    z: this.links.pos[srcPosBase + 2]
                },
                PosB: {
                    x: this.links.pos[srcPosBase + 3],
                    y: this.links.pos[srcPosBase + 4],
                    z: this.links.pos[srcPosBase + 5]
                },
                cost: cost,
                type: type
            };
            links.get(polyA)?.push(link);
            links.get(polyB)?.push(link);
        }
        this.heightfixer = new FunnelHeightFixer(this.mesh, this.meshdetail, ADJUST_HEIGHT_DISTANCE);
        this.astar = new PolyGraphAStar(this.mesh, links, this.heightfixer);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers, links);
    }
    /**
     * @param {Vector} [pos] //玩家所在位置
     */
    tick(pos)
    {
        this.plugin?.tick();
    }
    /**
     * 只调试“可持久保存”的数据（mesh/detail/links）。
     * @param {number} duration
     */
    debug(duration = 60) {
        {
            try{
                Instance.Msg("debug");
                Instance.Msg(`多边形总数: ${this.mesh.polyslength-1}  跳点总数: ${this.links.length-1}`);
                this.debugTools.debugDrawMeshPolys(duration);
                this.debugTools.debugDrawMeshConnectivity(duration);
                this.debugTools.debugLinks(duration);
            }
            catch(e)
            {
            }
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
        //this.debugTools.debugDrawPolyPath(polyPath.path,1);
        //if (!polyPath || polyPath.path.length === 0) return [];
        const funnelPath = this.funnel.build(polyPath.path, polyPath.start, polyPath.end);
        //this.debugTools.debugDrawfunnelPath(funnelPath,1);
        {
            const ans=this.heightfixer.fixHeight(funnelPath,polyPath.path);
            //this.debugTools.debugDrawPath(ans,1);
            return ans;
        }
        //if (!ans || ans.length === 0) return [];
        //多边形总数：649跳点数：82
        //100次A*           30ms
        //100次funnelPath   46ms-30=16ms
        //100次200fixHeight    100ms-46=54ms
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
let start={x:3457,y:-984,z:-352};
let end={x:-2960,y:-625,z:-416};
let pd=false;
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
        //            pathfinder.tick(pos);
        //            //end={x:pos.x,y:pos.y,z:pos.z};
        //            return;
        //        }
        //    }
        //})
        //pathfinder.tick();
        //ttt++;
        //if(ttt%64==0)pathfinder.debug(1);
        for(let i=0;i<100;i++)pathfinder.findPath(start,end);
    }
    Instance.SetNextThink(Instance.GetGameTime()+1/1);
});
Instance.SetNextThink(Instance.GetGameTime()+1/1);
Instance.OnBulletImpact((event)=>{
    let start = new Date();
    pathfinder.update(event.position);
    let end = new Date();
    Instance.Msg(`导航更新完成,耗时${end.getTime()-start.getTime()}ms`);
    //pathfinder.debug(30);
    //if(sss)end=event.position;
    //else start=event.position;
    //sss=!sss;
    //pathfinder.findPath(start,end);
    //pathfinder.findPath(start,end);
});
//init();
Instance.OnPlayerChat((event) => {
    const text = (event.text || "").trim().toLowerCase().split(' ')[0];
    if (text === "debug" || text === "!debug")
    {
        //每秒200000次tracebox,每tick 3125次，每个怪物平均10次，总共312只怪物
        //10v10 1000次tracebox
        init();
        //多边形总数: 651  跳点总数: 116_>91???
        //多边形总数: 651  跳点总数: 91
        //多边形总数: 635  跳点总数: 78
        Instance.Msg("开始调试");
        for(let i=0;i<1;i++)pathfinder.findPath(start,end);
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
