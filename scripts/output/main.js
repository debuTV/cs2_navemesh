import { Instance } from 'cs_script/point_script';

//==============================世界相关设置=====================================
const origin = { x: -2250, y: -1200, z: -200 };
//==============================Recast设置======================================
//体素化参数
const MESH_CELL_SIZE_XY = 5;                               // 体素大小
const MESH_CELL_SIZE_Z = 1;                                // 体素高度
const MESH_WORLD_SIZE_XY = 4500;                           // 世界大小
const MESH_WORLD_SIZE_Z = 480;                             // 世界高度
const MESH_ERODE_RADIUS = 0;                               // 腐蚀半径，如果一个区域应该不能被走到，但优化1未去除，可以增加此值
const MESH_HOLE_FILLING = 0;                               // 有些时候莫名其妙有空洞，这时候可以自动填洞,数值越高，越有可能填中
//区域生成参数
const REGION_MERGE_AREA = 1024;                            // 合并区域阈值（体素单位）
const REGION_MIN_AREA = 16;                                // 最小区域面积（体素单位）
//轮廓生成参数
const CONT_MAX_ERROR = MESH_CELL_SIZE_XY * 1.2;            // 原始点到简化后边的最大允许偏离距离（长度）
const POLY_MAX_VERTS_PER_POLY = 6;                         // 每个多边形的最大顶点数
const POLY_MERGE_LONGEST_EDGE_FIRST = true;                // 优先合并最长边
const POLY_DETAIL_SAMPLE_DIST = 3;                         // 构建细节网格时，相邻采样点之间的“间距”,选3耗时比较合适
const POLY_DETAIL_HEIGHT_ERROR = 5;                        // 构建细节网格时，采样点和计算点之差如果小于这个高度阈值就跳过
//其他参数
const MAX_WALK_HEIGHT = 13 / MESH_CELL_SIZE_Z;             // 怪物最大可行走高度（体素单位）
const MAX_JUMP_HEIGHT = 64 / MESH_CELL_SIZE_Z;             // 怪物最大可跳跃高度（体素单位）
const AGENT_HEIGHT = 72 / MESH_CELL_SIZE_Z;                // 人物高度（体素单位）
const ASTAR_HEURISTIC_SCALE = 1.2;                         //A*推荐数值
//Funnel参数
const FUNNEL_DISTANCE = 25;                                //拉直的路径距离边缘多远(0-100，百分比，100%意味着只能走边的中点)
//高度修正参数
const ADJUST_HEIGHT_DISTANCE = 200;                        //路径中每隔这个距离增加一个点，用于修正高度
/**
 * 计算空间两点之间的距离的平方
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @returns {number}
 */
function posDistance3Dsqr(a, b) {
    const dx = a.x - b.x; const dy = a.y - b.y; const dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

/**
 * 计算xy平面两点之间的距离的平方
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @returns {number}
 */
function posDistance2Dsqr(a, b) {
    const dx = a.x - b.x; const dy = a.y - b.y;
    return dx * dx + dy * dy;
}
/**
 * 返回pos上方height高度的点
 * @param {import("cs_script/point_script").Vector} pos
 * @param {number} height
 * @returns {import("cs_script/point_script").Vector}
 */
function posZfly(pos, height) {
    return { x: pos.x, y: pos.y, z: pos.z + height };
}
/**
 * 点p到线段ab距离的平方
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
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
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @param {import("cs_script/point_script").Vector} c
 */
function area(a, b, c) {
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
function isConvex(prev, cur, next) {
    return area(prev, cur, next) > 0;
}
/**
 * xy平面上点p是否在abc构成的三角形内（不包括边上）
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
 * @param {import("cs_script/point_script").Vector} c
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
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector} a
 * @param {import("cs_script/point_script").Vector} b
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
 * @param {import("cs_script/point_script").Vector} p
 * @param {import("cs_script/point_script").Vector[]} verts
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
 * @param {import("cs_script/point_script").Vector} pos
 * @param {import("cs_script/point_script").Vector[]} verts
 * @param {number[]} poly
 */
function closestPointOnPoly(pos, verts, poly) {
    // 1. 如果在多边形内部（XY），直接投影到平面
    if (pointInConvexPolyXY(pos, verts, poly)) {
        // 用平均高度（你也可以用平面方程）
        let z = 0;
        for (const vi of poly) z += verts[vi].z;
        z /= poly.length;

        return { x: pos.x, y: pos.y, z };
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
            best = c;
        }
    }

    return best;
}

class PolyGraphAStar {
    /**
     * @param {{verts: {x: number;y: number;z: number;}[];polys: number[][];regions: number[];neighbors: number[][];}} polys
     * @param {{ PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number; type: number; }[]} links
     */
    constructor(polys, links) {
        this.mesh = polys;
        this.polyCount = polys.polys.length;
        /**@type {Map<number,{PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number; type: number;}[]>} */
        this.links = new Map();
        for (const link of links) {
            const polyA = link.PolyA;
            const polyB = link.PolyB;
            if (!this.links.has(polyA)) this.links.set(polyA, []);
            if (!this.links.has(polyB)) this.links.set(polyB, []);
            this.links.get(polyA)?.push(link);
            this.links.get(polyB)?.push(link);
        }
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
        Instance.Msg("多边形总数：" + this.polyCount + "跳点数：" + links.length);
        this.open = new MinHeap(this.polyCount);

        //查询所在多边形优化
        this.spatialCellSize = 256;
        this.spatialGrid = new Map();
        this.buildSpatialIndex();
    }

    /**
     * @param {import("cs_script/point_script").Vector} start
     * @param {import("cs_script/point_script").Vector} end
     */
    findPath(start, end) {
        const startPoly = this.findContainingPoly(start);
        const endPoly = this.findContainingPoly(end);

        //Instance.Msg(startPoly+"   "+endPoly);
        if (startPoly < 0 || endPoly < 0) {
            Instance.Msg(`跑那里去了?`);
            return [];
        }

        if (startPoly == endPoly) {
            return [{ id: endPoly, mode: 1 }];
        }
        return this.findPolyPath(startPoly, endPoly);
    }
    buildSpatialIndex() {
        for (let i = 0; i < this.mesh.polys.length; i++) {
            const poly = this.mesh.polys[i];

            let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
            for (const vi of poly) {
                const v = this.mesh.verts[vi];
                if (v.x < minX) minX = v.x;
                if (v.y < minY) minY = v.y;
                if (v.x > maxX) maxX = v.x;
                if (v.y > maxY) maxY = v.y;
            }

            const x0 = Math.floor(minX / this.spatialCellSize);
            const x1 = Math.ceil(maxX / this.spatialCellSize);
            const y0 = Math.floor(minY / this.spatialCellSize);
            const y1 = Math.ceil(maxY / this.spatialCellSize);

            for (let x = x0; x <= x1; x++) {
                for (let y = y0; y <= y1; y++) {
                    const key = `${x}_${y}`;
                    if (!this.spatialGrid.has(key)) this.spatialGrid.set(key, []);
                    this.spatialGrid.get(key).push(i);
                }
            }
        }
    }
    /**
     * 返回包含点的 poly index，找不到返回 -1
     * @param {{x:number,y:number,z:number}} p
     */
    findContainingPoly(p) {
        let bestPoly = -1;
        let bestDist = Infinity;
        const x = Math.floor(p.x / this.spatialCellSize);
        const y = Math.floor(p.y / this.spatialCellSize);
        const key = `${x}_${y}`;
        const candidates = this.spatialGrid.get(key);
        if (!candidates) return -1;
        for (const polyIdx of candidates) {
            const poly = this.mesh.polys[polyIdx];
            const cp = closestPointOnPoly(p, this.mesh.verts, poly);
            if (!cp) continue;

            const dx = cp.x - p.x;
            const dy = cp.y - p.y;
            const dz = cp.z - p.z;
            const d = dx * dx + dy * dy + dz * dz;

            if (d < bestDist) {
                bestDist = d;
                bestPoly = polyIdx;
            }
        }
        return bestPoly;
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
            
            const hToTarget=this.distsqr(current, end);
            if (hToTarget < minH) {
                minH = hToTarget;
                closestNode = current;
            }

            const neighbors = this.mesh.neighbors[current];
            for (let i = 0; i < neighbors.length; i++) {
                const n = neighbors[i];
                if (n < 0 || state[n] == 2) continue;
                // @ts-ignore
                const tentative = g[current] + this.distsqr(current, n);
                if (tentative < g[n]) {
                    parent[n] = current;
                    walkMode[n] = 1;
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
                    walkMode[v] = 2;
                    if (state[v] != 1) {
                        open.push(v, f);
                        state[v] = 1;
                    }
                    else open.update(v, f);
                }
            }
        }
        return this.reconstruct(parent,walkMode, closestNode);
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
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
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
        if (i == null) return;
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

class FunnelPath {
    /**
     * @param {{verts:import("cs_script/point_script").Vector[],polys:number[][],regions:number[],neighbors:number[][]}} mesh
     * @param {import("cs_script/point_script").Vector[]} centers
     * @param {{ PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number; type: number; }[]} links
     */
    constructor(mesh, centers, links) {
        this.mesh = mesh;
        this.centers = centers;
        /**@type {Map<number,{PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number; type: number;}[]>} */
        this.links = new Map();
        for (const link of links) {
            const polyA = link.PolyA;
            const polyB = link.PolyB;
            if (!this.links.has(polyA)) this.links.set(polyA, []);
            if (!this.links.has(polyB)) this.links.set(polyB, []);
            this.links.get(polyA)?.push(link);
            this.links.get(polyB)?.push(link);
        }
        //Instance.Msg(this.links.size);
    }
    //返回pA到pB的跳点
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
     * @param {import("cs_script/point_script").Vector} startPos
     * @param {import("cs_script/point_script").Vector} endPos
     */
    build(polyPath, startPos, endPos) {
        if (!polyPath || polyPath.length === 0) return [];
        if (polyPath.length === 1) return [{pos:startPos,mode:1}, {pos:endPos,mode:1}];
        const ans = [];
        // 当前这一段行走路径的【起点坐标】
        let currentSegmentStartPos = startPos;
        // 当前这一段行走路径的【多边形起始索引】（在 polyPath 中的 index）
        let segmentStartIndex = 0;
        for (let i = 1; i < polyPath.length; i++) {
            const prevPoly = polyPath[i - 1];
            const currPoly = polyPath[i];
            if (polyPath[i].mode == 2)//到第i个多边形需要跳跃，那就拉直最开始到i-1的路径
            {
                // 1. 获取跳点坐标信息
                const linkInfo = this.getlink(currPoly.id,prevPoly.id);
                if (!linkInfo)continue;
                const walkPathSegment = polyPath.slice(segmentStartIndex, i);
                const portals = this.buildPortals(walkPathSegment, currentSegmentStartPos, linkInfo.start, FUNNEL_DISTANCE);
                const smoothedWalk = this.stringPull(portals);
                for (const p of smoothedWalk) ans.push({pos:p,mode:1});
                ans.push({pos:linkInfo.end,mode:2});
                currentSegmentStartPos = linkInfo.end; // 下一段从落地点开始走
                segmentStartIndex = i; // 下一段多边形从 currPoly 开始
            }
        }
        const lastWalkSegment = polyPath.slice(segmentStartIndex, polyPath.length);
        const lastPortals = this.buildPortals(lastWalkSegment, currentSegmentStartPos, endPos, FUNNEL_DISTANCE);
        const lastSmoothed = this.stringPull(lastPortals);

        for (const p of lastSmoothed) ans.push({pos:p,mode:1});
        return this.removeDuplicates(ans);
    }
    /**
     * 简单的去重，防止相邻点坐标完全一样
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     */
    removeDuplicates(path) {
        if (path.length < 2) return path;
        const res = [path[0]];
        for (let i = 1; i < path.length; i++) {
            const last = res[res.length - 1];
            const curr = path[i];
            const d = (last.pos.x - curr.pos.x) ** 2 + (last.pos.y - curr.pos.y) ** 2 + (last.pos.z - curr.pos.z) ** 2;
            // 容差极小值
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
     * @param {import("cs_script/point_script").Vector} startPos
     * @param {import("cs_script/point_script").Vector} endPos
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
            if (neigh[ei] !== pb) continue;

            const v0 = this.mesh.verts[poly[ei]];
            const v1 = this.mesh.verts[poly[(ei + 1) % poly.length]];

            // 统一左右（从 pa 看向 pb）
            const ca = this.centers[pa];
            const cb = this.centers[pb];

            if (this.triArea2(ca, cb, v0) < 0) {
                return this._applyFunnelDistance(v0, v1, funnelDistance);
            } else {
                return this._applyFunnelDistance(v1, v0, funnelDistance);
            }
        }
    }
    /**
     * 根据参数收缩门户宽度
     * @param {import("cs_script/point_script").Vector} left 
     * @param {import("cs_script/point_script").Vector} right 
     * @param {number} distance 0-100
     */
    _applyFunnelDistance(left, right, distance) {
        // 限制在 0-100
        const t = Math.max(0, Math.min(100, distance)) / 100.0;

        // 如果 t 是 0，保持原样（虽然前面判断过了，这里做个安全兜底）
        if (t === 0) return { left, right };

        // 计算中点
        const midX = (left.x + right.x) * 0.5;
        const midY = (left.y + right.y) * 0.5;
        const midZ = (left.z + right.z) * 0.5;
        const mid = { x: midX, y: midY, z: midZ };

        // 使用线性插值将端点向中点移动
        // t=0 -> 保持端点, t=1 -> 变成中点
        const newLeft = this._lerp(left, mid, t);
        const newRight = this._lerp(right, mid, t);

        return { left: newLeft, right: newRight };
    }

    /**
     * 向量线性插值
     * @param {import("cs_script/point_script").Vector} a 
     * @param {import("cs_script/point_script").Vector} b 
     * @param {number} t 
     */
    _lerp(a, b, t) {
        return {
            x: a.x + (b.x - a.x) * t,
            y: a.y + (b.y - a.y) * t,
            z: a.z + (b.z - a.z) * t
        };
    }
    /* ===============================
       Funnel (String Pull)
    =============================== */

    /**
       * @param {{left:import("cs_script/point_script").Vector,right:import("cs_script/point_script").Vector}[]} portals
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
            if (this.triArea2(apex, right, pRight) <= 0) {
                if (apex === right || this.triArea2(apex, left, pRight) > 0) {
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
            if (this.triArea2(apex, left, pLeft) >= 0) {
                if (apex === left || this.triArea2(apex, right, pLeft) < 0) {
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
    /**
     * 返回值 > 0 表示 c 在 ab 线的左侧
     * 返回值 < 0 表示 c 在 ab 线的右侧
     * 返回值 = 0 表示三点共线
     * @param {{ x: number;y:number, z: number; }} a
     * @param {{ x: number;y:number, z: number; }} b
     * @param {{ x: number;y:number, z: number;}} c
     */
    triArea2(a, b, c) {
        return (b.x - a.x) * (c.y - a.y)
            - (b.y - a.y) * (c.x - a.x);
    }

}

class OpenSpan {
    /**
     * @param {number} floor
     * @param {number} ceiling
     * @param {number} id
     */
    constructor(floor, ceiling, id) {
        /**@type {number} */
        this.floor = floor;
        /**@type {number} */
        this.ceiling = ceiling;
        /**@type {OpenSpan|null} */
        this.next = null;
        /**@type {number} */
        this.id = id;

        /**@type {any[]} */
        this.neighbors = [null, null, null, null];
        /**@type {number} */
        this.distance = 0;
        /**@type {number} */
        this.regionId = 0;

        //区域距离场优化
        this.newDist = 0;
        //是否在使用
        this.use=true;
    }

    /**
     * @param {OpenSpan} other
     * @param {number} maxStep
     * @param {number} agentHeight
     * @returns {boolean}
     */
    canTraverseTo(other, maxStep = MAX_WALK_HEIGHT, agentHeight = AGENT_HEIGHT) {
        if(!other.use)return false;
        if (Math.abs(other.floor - this.floor) > maxStep) {
            return false;
        }

        const floor = Math.max(this.floor, other.floor);
        const ceil = Math.min(this.ceiling, other.ceiling);

        if (ceil - floor < agentHeight) {
            return false;
        }

        return true;
    }
}

class OpenHeightfield {
    constructor() {

        /**@type {(OpenSpan | null)[][]}*/
        this.cells = [];
        /**@type {boolean[][][]}*/
        this.precells = [];
        this.SPAN_ID = 1;
        /**@type {number} */
        this.gridX = Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        /**@type {number} */
        this.gridY = Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        /**@type {number} */
        this.gridZ = Math.floor(MESH_WORLD_SIZE_Z / MESH_CELL_SIZE_Z) + 1;
    }
    init() {
        const minZ = origin.z;
        const maxZ = origin.z + MESH_WORLD_SIZE_Z;

        for (let x = 0; x < this.gridX; x++) {
            this.cells[x] = [];
            for (let y = 0; y < this.gridY; y++) {
                const worldX = origin.x + x * MESH_CELL_SIZE_XY;
                const worldY = origin.y + y * MESH_CELL_SIZE_XY;

                this.cells[x][y] = this.voxelizeColumn(worldX, worldY, minZ, maxZ);
            }
        }
        this.erode(MESH_ERODE_RADIUS);
    }

    /**
     * @param {number} wx
     * @param {number} wy
     * @param {number} minZ
     * @param {number} maxZ
     */
    voxelizeColumn(wx, wy, minZ, maxZ) {
        let head = null;
        let currentZ = maxZ;
        const radius = MESH_CELL_SIZE_XY / 2+ MESH_HOLE_FILLING;

        while (currentZ >= minZ + radius) {
            //寻找地板 (floor)
            const downStart = { x: wx, y: wy, z: currentZ };
            const downEnd = { x: wx, y: wy, z: minZ };
            const downTr = Instance.TraceSphere({ radius, start: downStart, end: downEnd, ignorePlayers: true });

            if (!downTr || !downTr.didHit) break; // 下面没东西了，结束

            const floorZ = downTr.end.z - radius;

            //从地板向上寻找天花板 (ceiling)
            const upStart = { x: wx, y: wy, z: downTr.end.z + 1 };
            const upEnd = { x: wx, y: wy, z: maxZ };
            const upTr = Instance.TraceSphere({ radius, start: upStart, end: upEnd, ignorePlayers: true });

            let ceilingZ = maxZ;
            if (upTr.didHit) ceilingZ = upTr.end.z + radius;

            const floor = Math.round((floorZ - origin.z) / MESH_CELL_SIZE_Z);
            const ceiling = Math.round((ceilingZ - origin.z) / MESH_CELL_SIZE_Z);

            if ((ceiling - floor) >= AGENT_HEIGHT) {
                const newSpan = new OpenSpan(floor, ceiling, this.SPAN_ID++);

                if (!head || floor < head.floor) {
                    newSpan.next = head;
                    head = newSpan;
                } else {
                    let curr = head;
                    while (curr.next && curr.next.floor < floor) {
                        curr = curr.next;
                    }
                    newSpan.next = curr.next;
                    curr.next = newSpan;
                }
            }

            currentZ = floorZ - radius - 1;
        }

        return head;
    }
    //筛选不能去的区域
    findcanwalk() {
        const slist = Instance.FindEntitiesByClass("info_target");
        /**@type {Entity|undefined} */
        let s;
        slist.forEach((i) => {
            if (i.GetEntityName() == "navmesh") {
                s = i;
                return;
            }
        });
        if (!s) return;
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];
        let vis = Array(this.SPAN_ID + 5).fill(false);
        const centerPos = s.GetAbsOrigin();
        const cx = Math.ceil((centerPos.x - origin.x) / MESH_CELL_SIZE_XY);
        const cy = Math.ceil((centerPos.y - origin.y) / MESH_CELL_SIZE_XY);
        const cz = Math.ceil((centerPos.z - origin.z) / MESH_CELL_SIZE_Z) + 2;
        /**@type {OpenSpan|null} */
        let startSpan = this.cells[cx][cy];
        while (startSpan) {
            if (startSpan.use&&cz <= startSpan.ceiling && cz >= startSpan.floor) break;
            startSpan = startSpan.next;
        }
        if (!startSpan) return;
        let queue = [{ span: startSpan, i: cx, j: cy }];
        vis[startSpan.id] = true;
        while (queue.length > 0) {
            let currentSpan = queue.shift();
            if (!currentSpan) break;
            for (let dir = 0; dir < 4; dir++) {
                const nx = currentSpan.i + dirs[dir].dx;
                const ny = currentSpan.j + dirs[dir].dy;
                if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;
                /**@type {OpenSpan|null} */
                let neighbor = this.cells[nx][ny];
                while (neighbor) {
                    if(neighbor.use)
                    {
                        if (!vis[neighbor.id]) {
                            // 检查是否可以通过
                            if (currentSpan.span.canTraverseTo(neighbor, MAX_JUMP_HEIGHT, AGENT_HEIGHT)) {
                                vis[neighbor.id] = true;
                                queue.push({ span: neighbor, i: nx, j: ny });
                            }
                        }
                    }
                    neighbor = neighbor.next;
                }
            }
        }
        // 遍历所有的cell
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                //let prevSpan = null;
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        if (!vis[currentSpan.id]) {
                            // 如果当前span不可达，则删除它
                            currentSpan.use=false;
                            //if (prevSpan) {
                            //    // 如果有前驱节点，跳过当前节点
                            //    prevSpan.next = currentSpan.next;
                            //} else {
                            //    // 如果没有前驱节点，说明是链表的第一个节点，直接修改
                            //    this.cells[i][j] = currentSpan.next;
                            //}
                        } //else {
                            // 如果当前span是可达的，更新prevSpan
                           // prevSpan = currentSpan;
                        //}
                        // 继续遍历下一个span
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
    }
    /**
     * 根据半径腐蚀可行走区域
     * @param {number} radius
     */
    erode(radius) {
        if (radius <= 0) return;

        // 1. 初始化距离场，默认给一个很大的值
        // 使用 Uint16Array 节省内存，索引为 span.id
        const distances = new Uint16Array(this.SPAN_ID + 1).fill(65535);
        const dirs = [{ dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 1, dy: 0 }, { dx: 0, dy: -1 }];

        // 2. 标记边界点（距离为 0）
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let span = this.cells[i][j];
                while (span) {
                    if(span.use)
                    {
                        let isBoundary = false;
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;

                            // 触碰地图边界或没有邻居，即为边界
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                                isBoundary = true;
                                break;
                            }

                            let hasNeighbor = false;
                            let nspan = this.cells[nx][ny];
                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan,MAX_JUMP_HEIGHT,AGENT_HEIGHT)) {
                                        hasNeighbor = true;
                                        break;
                                    }
                                }
                                nspan = nspan.next;
                            }

                            if (!hasNeighbor) {
                                isBoundary = true;
                                break;
                            }
                        }

                        if (isBoundary) distances[span.id] = 0;
                    }
                    span = span.next;
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
                //let prevSpan = null;
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        // 如果距离边界太近，则剔除
                        if (distances[currentSpan.id] < radius) {
                            currentSpan.use=false;
                            //if (prevSpan) prevSpan.next = currentSpan.next;
                            //else this.cells[i][j] = currentSpan.next;
                        } //else {
                        //  prevSpan = currentSpan;
                        //}
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
    }

    /**
     * 内部辅助：距离场传递
     * @param {Uint16Array<ArrayBuffer>} distances
     * @param {boolean} forward
     */
    _passDist(distances, forward) {
        const dirs = [{ dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 1, dy: 0 }, { dx: 0, dy: -1 }];
        const startX = forward ? 0 : this.gridX - 1;
        const endX = forward ? this.gridX : -1;
        const step = forward ? 1 : -1;

        for (let i = startX; i !== endX; i += step) {
            for (let j = forward ? 0 : this.gridY - 1; j !== (forward ? this.gridY : -1); j += step) {
                let span = this.cells[i][j];
                while (span) {
                    if(span.use)
                    {
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;

                            let nspan = this.cells[nx][ny];
                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan,MAX_JUMP_HEIGHT,AGENT_HEIGHT)) {
                                        // 核心公式：当前点距离 = min(当前距离, 邻居距离 + 1)
                                        distances[span.id] = Math.min(distances[span.id], distances[nspan.id] + 1);
                                    }
                                }
                                nspan = nspan.next;
                            }
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    deleteboundary() {
        let boundary = Array(this.SPAN_ID + 5).fill(false);
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];
        // 遍历所有的cell
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let neighbors = [false, false, false, false];
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    if(span.use)
                    {
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;
                            /**@type {OpenSpan|null} */
                            let nspan = this.cells[nx][ny];
                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan)) {
                                        neighbors[d] = true;
                                    }
                                }
                                nspan = nspan.next;
                            }
                        }
                        if (!(neighbors[0] && neighbors[1] && neighbors[2] && neighbors[3])) {
                            boundary[span.id] = true;
                        }
                    }
                    span = span.next;
                }
            }
        }
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                //let prevSpan = null;
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        if (boundary[currentSpan.id]) {
                            currentSpan.use=false;
                            //// 如果当前span不可达，则删除它
                            //if (prevSpan) {
                            //    // 如果有前驱节点，跳过当前节点
                            //    prevSpan.next = currentSpan.next;
                            //} else {
                            //    // 如果没有前驱节点，说明是链表的第一个节点，直接修改
                            //    this.cells[i][j] = currentSpan.next;
                            //}
                        }// else {
                            // 如果当前span是可达的，更新prevSpan
                         //   prevSpan = currentSpan;
                        //}
                        // 继续遍历下一个span
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
    }
    deletealone() {
        let del = Array(this.SPAN_ID + 5).fill(false);
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];
        // 遍历所有的cell
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let neighbors = [false, false, false, false];
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    if(span.use)
                    {
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;
                            /**@type {OpenSpan|null} */
                            let nspan = this.cells[nx][ny];
                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan)) {
                                        neighbors[d] = true;
                                    }
                                }
                                nspan = nspan.next;
                            }
                        }
                        if ((neighbors[0] ? 1 : 0) + (neighbors[1] ? 1 : 0) + (neighbors[2] ? 1 : 0) + (neighbors[3] ? 1 : 0) <= 1) {
                            del[span.id] = true;
                        }
                    }
                    span = span.next;
                }
            }
        }
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        if (del[currentSpan.id]) {
                            currentSpan.use=false;
                            // 如果当前span不可达，则删除它
                            //if (prevSpan) {
                            //    // 如果有前驱节点，跳过当前节点
                            //    prevSpan.next = currentSpan.next;
                            //} else {
                            //    // 如果没有前驱节点，说明是链表的第一个节点，直接修改
                            //    this.cells[i][j] = currentSpan.next;
                            //}
                        } //else {
                           // prevSpan = currentSpan;
                        //}
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
    }
    debug(duration = 30) {
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    if(span.use==true)
                    {
                        const c = {
                            r: 255,
                            g: 255,
                            b: 0
                        };
                        Instance.DebugSphere({
                            center: {
                                x: origin.x + i * MESH_CELL_SIZE_XY,
                                y: origin.y + j * MESH_CELL_SIZE_XY,
                                z: origin.z + span.floor * MESH_CELL_SIZE_Z
                            },
                            radius: 3,
                            duration,
                            color: c
                        });
                    }
                    //Instance.DebugBox({ 
                    //    mins: { 
                    //        x: origin.x+i*MESH_CELL_SIZE_XY - MESH_CELL_SIZE_XY/2, 
                    //        y: origin.y+j*MESH_CELL_SIZE_XY - MESH_CELL_SIZE_XY/2, 
                    //        z: origin.z +span.floor*MESH_CELL_SIZE_Z
                    //    },
                    //    maxs: { 
                    //        x: origin.x+i*MESH_CELL_SIZE_XY + MESH_CELL_SIZE_XY/2, 
                    //        y: origin.y+j*MESH_CELL_SIZE_XY + MESH_CELL_SIZE_XY/2, 
                    //        z: origin.z +span.floor*MESH_CELL_SIZE_Z+5
                    //    }, 
                    //    duration: duration, 
                    //    color: c
                    //});
                    span = span.next;
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
        /**@type {(OpenSpan|null)[][]} */
        this.hf = openHeightfield.cells;
        /**@type {number} */
        this.gridX = openHeightfield.gridX;
        /**@type {number} */
        this.gridY = openHeightfield.gridY;
        this.regions = [];
        /**@type {number} */
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
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        span.neighbors = [null, null, null, null];

                        for (let d = 0; d < 4; d++) {
                            const nx = x + dirs[d].dx;
                            const ny = y + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;

                            let best = null;
                            let bestDiff = Infinity;
                            /**@type {OpenSpan|null} */
                            let nspan = this.hf[nx][ny];

                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan)) {
                                        const diff = Math.abs(span.floor - nspan.floor);
                                        if (diff < bestDiff) {
                                            best = nspan;
                                            bestDiff = diff;
                                        }
                                    }
                                }
                                nspan = nspan.next;
                            }

                            span.neighbors[d] = best;
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    /**
     * 获取邻居。
     * @param {OpenSpan} span 
     * @param {number} dir 方向 (0:W, 1:N, 2:E, 3:S)
     * @returns {OpenSpan|null}
     */
    getNeighbor(span, dir) {
        return span.neighbors[dir];
    }

    /**
     * 获取对角线邻居。
     * 例如：西北 (NW) = 先向西(0)再向北(1)
     * @param {OpenSpan} span 
     * @param {number} dir1 
     * @param {number} dir2 
     */
    getDiagonalNeighbor(span, dir1, dir2) {
        const n = span.neighbors[dir1];
        if (n) {
            return n.neighbors[dir2];
        }
        return null;
    }
    //构建距离场
    buildDistanceField() {
        // 1. 初始化：边界设为0，内部设为无穷大
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        // 如果任意一个邻居缺失，说明是边界
                        span.distance = this.isBorderSpan(span) ? 0 : Infinity;
                    }
                    span = span.next;
                }
            }
        }

        // 第一遍扫描：从左下到右上
        // 西(0)、西南(0+3)、南(3)、东南(3+2)
        for (let y = 0; y < this.gridY; y++) {
            for (let x = 0; x < this.gridX; x++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.distance > 0) {
                            // 西
                            let n = this.getNeighbor(span, 0);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 西南
                            let nd = this.getDiagonalNeighbor(span, 0, 3);
                            if (nd) span.distance = Math.min(span.distance, nd.distance + 3);
                            // 南
                            n = this.getNeighbor(span, 3);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 东南
                            nd = this.getDiagonalNeighbor(span, 3, 2);
                            if (nd) span.distance = Math.min(span.distance, nd.distance + 3);
                        }
                    }
                    span = span.next;
                }
            }
        }

        // 第二遍扫描：从右上到左下
        // 东(2)、东北(2+1)、北(1)、西北(1+0)
        for (let y = this.gridY - 1; y >= 0; y--) {
            for (let x = this.gridX - 1; x >= 0; x--) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.distance > 0) {
                            // 东
                            let n = this.getNeighbor(span, 2);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 东北
                            let nd = this.getDiagonalNeighbor(span, 2, 1);
                            if (nd) span.distance = Math.min(span.distance, nd.distance + 3);
                            // 北
                            n = this.getNeighbor(span, 1);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 西北
                            let nd2 = this.getDiagonalNeighbor(span, 1, 0);
                            if (nd2) span.distance = Math.min(span.distance, nd2.distance + 3);
                        }
                    }
                    span = span.next;
                }
            }
        }
        this.blurDistanceField();
    }
    //对距离场进行平滑处理
    blurDistanceField() {
        const threshold = 2; //距离阈值，小于的不进行模糊

        //计算模糊后的值
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        //只有远离边界的体素才参与模糊
                        if (span.distance <= threshold) span.newDist = span.distance;
                        else {
                            let d = span.distance;
                            //计算平均距离
                            for (let i = 0; i < 4; i++) {
                                const n = span.neighbors[i];
                                if (n) d += n.distance;
                                else d += span.distance;
                            }
                            span.newDist = Math.floor((d + 2) / 5);
                        }
                    }
                    span = span.next;
                }
            }
        }
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.newDist !== undefined) {
                            span.distance = span.newDist;
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    /**
     * 计算当前span从dir方向过来的距离
     * @param {OpenSpan} span
     * @param {number} dir
     */
    sample(span, dir) {
        const n = span.neighbors[dir];
        if (!n) return Infinity;

        return n.distance + 2;
    }

    /**
     * 是否是边界span
     * @param {OpenSpan} span
     */
    isBorderSpan(span) {
        for (let d = 0; d < 4; d++) {
            if (!span.neighbors[d]) return true;
        }
        return false;
    }

    //洪水扩张
    buildRegionsWatershed() {
        const spans = [];
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        span.regionId = 0;
                        if (span.distance >= 0) {
                            spans.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        //从大到小排序
        spans.sort((a, b) => b.distance - a.distance);

        for (const span of spans) {

            let bestRegion = 0;
            let maxNeighborDist = -1;

            for (let d = 0; d < 4; d++) {
                const n = span.neighbors[d];
                if (!n) continue;

                //如果邻居已经有Region了，说明这个邻居比当前span更靠近“中心”

                if (n.regionId > 0) {
                    if (n.distance > maxNeighborDist) {
                        maxNeighborDist = n.distance;
                        bestRegion = n.regionId;
                    }
                }
            }

            if (bestRegion !== 0) span.regionId = bestRegion;
            else span.regionId = this.nextRegionId++;
        }
        //this.floodRemaining();
        //this.mergeAndFilterRegions();
    }
    //abandon
    floodRemaining() {
        //填补距离为0的边界缝隙
        let changed = true;
        let iterCount = 0;
        while (changed && iterCount < 5) {
            changed = false;
            iterCount++;
            for (let x = 0; x < this.gridX; x++) {
                for (let y = 0; y < this.gridY; y++) {
                    let span = this.hf[x][y];
                    while (span) {
                        if(span.use)
                        {
                            //如果当前没有区域
                            if (span.regionId === 0) {
                                let bestRegion = 0;
                                let bestDist = -1;
                                for (let d = 0; d < 4; d++) {
                                    const n = span.neighbors[d];
                                    if (n && n.regionId > 0) {
                                        if (n.distance > bestDist) {
                                            bestDist = n.distance;
                                            bestRegion = n.regionId;
                                        }
                                    }
                                }
                                if (bestRegion > 0) {
                                    span.regionId = bestRegion;
                                    changed = true;
                                }
                            }
                        }
                        span = span.next;
                    }
                }
            }
        }
    }

    //合并过滤小region
    mergeAndFilterRegions() {
        /**@type {Map<number,OpenSpan[]>} */
        const regionSpans = new Map();

        //统计每个region包含的span
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            if (!regionSpans.has(span.regionId)) regionSpans.set(span.regionId, []);
                            regionSpans.get(span.regionId)?.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        //合并过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MERGE_AREA) continue;
            const neighbors = new Map();
            for (const span of spans) {
                for (let d = 0; d < 4; d++) {
                    const n = span.neighbors[d];
                    if (n && n.regionId !== id) {
                        neighbors.set(
                            n.regionId,
                            (neighbors.get(n.regionId) ?? 0) + 1
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
                for (const span of spans) {
                    span.regionId = best;
                    regionSpans.get(span.regionId)?.push(span);
                }
                regionSpans.set(id, []);
            }
        }
        //统计每个region包含的span
        regionSpans.clear();
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            if (!regionSpans.has(span.regionId)) regionSpans.set(span.regionId, []);
                            regionSpans.get(span.regionId)?.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        //忽略过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MIN_AREA) continue;
            for (const span of spans) {
                if (span.regionId == id) span.regionId = 0;
            }
        }
    }
    //abandon
    smooth() {
        /**@type {Map<number,OpenSpan[]>} */
        const regionSpans = new Map();
        //统计每个region包含的span
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            if (!regionSpans.has(span.regionId)) regionSpans.set(span.regionId, []);
                            regionSpans.get(span.regionId)?.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        for (const [id, spans] of regionSpans) {
            let smoth = true;
            while (smoth) {
                smoth = false;
                let diff = 0;
                let df = [];
                outer:
                for (const span of spans) {
                    if (span.regionId != id) continue;
                    for (let d = 0; d < 4; d++) {
                        const n = span.neighbors[d];
                        if (n && n.regionId !== id) {
                            df[diff] = n.regionId;
                            diff++;
                        }
                    }
                    if (diff == 3) {
                        //这个span周围有三个不同于自身的区域
                        if (df[0] == df[1]) {
                            span.regionId = df[0];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        else if (df[1] == df[2]) {
                            span.regionId = df[1];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        else if (df[0] == df[2]) {
                            span.regionId = df[0];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        else {
                            //四个区域不相同加入旁边
                            span.regionId = df[0];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        smoth = true;
                        break outer;
                    }
                }
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
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            const c = randomColor(span.regionId);

                            const center = {
                                x: origin.x + (x + 0.5) * MESH_CELL_SIZE_XY,
                                y: origin.y + (y + 0.5) * MESH_CELL_SIZE_XY,
                                z: origin.z + span.floor * MESH_CELL_SIZE_Z
                            };

                            Instance.DebugSphere({
                                center,
                                radius: MESH_CELL_SIZE_XY * 0.3,
                                color: c,
                                duration
                            });
                        }
                    }
                    span = span.next;
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
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        maxDist = Math.max(maxDist, span.distance);
                    }
                    span = span.next;
                }
            }
        }

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.distance < Infinity) {
                            const t = span.distance / maxDist;
                            const c = {
                                r: Math.floor(255 * t),
                                g: Math.floor(255 * (1 - t)),
                                b: 0
                            };

                            Instance.DebugSphere({
                                center: {
                                    x: origin.x + x * MESH_CELL_SIZE_XY,
                                    y: origin.y + y * MESH_CELL_SIZE_XY,
                                    z: origin.z + span.floor * MESH_CELL_SIZE_Z
                                },
                                radius: MESH_CELL_SIZE_XY * 0.25,
                                color: c,
                                duration
                            });
                        }
                    }
                    span = span.next;
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
        /** @type {(OpenSpan|null)[][]} */
        this.hf = hf.cells;
        this.gridX = hf.gridX;
        this.gridY = hf.gridY;

        /** @type {Contour[][]} */
        this.contours = [];
        this.cornerHeightCache = new Map();
    }

    /**
     * 边界边：没有邻居，或邻居 region 不同
     * @param {OpenSpan} span
     * @param {number} dir
     */
    isBoundaryEdge(span, dir) {
        const n = span.neighbors[dir];
        return !n || n.regionId !== span.regionId;
    }
    /**
     * @param {OpenSpan} span
     * @param {number} dir
     */
    getNeighborregionid(span, dir) {
        const n = span.neighbors[dir];
        if (n) return n.regionId;
        else return 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {OpenSpan} span
     * @param {number} dir
     */
    edgeKey(x, y, span, dir) {
        return `${x},${y},${span.id},${dir}`;
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
    /**
     * 判断线段 (p1, p2) 和 (p3, p4) 是否相交
     * @param {import("cs_script/point_script").Vector} p1
     * @param {import("cs_script/point_script").Vector} p2
     * @param {import("cs_script/point_script").Vector} p3
     * @param {import("cs_script/point_script").Vector} p4
     * @param {boolean} checkpoint //是否包含端点
     */
    segmentsIntersect(p1, p2, p3, p4, checkpoint) {
        const crossProduct = (/** @type {{ x: any; y: any; z?: number; }} */ a, /** @type {{ x: any; y: any; z?: number; }} */ b, /** @type {{ x: any; y: any; z?: number; }} */ c) => (c.y - a.y) * (b.x - a.x) - (b.y - a.y) * (c.x - a.x);
        // 快速排斥实验 + 跨立实验
        const d1 = crossProduct(p1, p2, p3);
        const d2 = crossProduct(p1, p2, p4);
        const d3 = crossProduct(p3, p4, p1);
        const d4 = crossProduct(p3, p4, p2);
        if (checkpoint) return (d1 * d2 <= 0 && d3 * d4 <= 0);
        return (d1 * d2 < 0 && d3 * d4 < 0);
        //return (ccw(p1, p3, p4) !== ccw(p2, p3, p4)) && (ccw(p1, p2, p3) !== ccw(p1, p2, p4));
    }
    /**
     * @param {Contour} holePt
     * @param {Contour[]} outer
     * @param {Contour[][]} holos
     * @param {number} innerid
     */
    findBridgeOuterIndex(holePt, outer, holos, innerid) {
        const inner = holos[innerid];
        let bestDistsq = Infinity;
        let bestIdx = -1;

        for (let i = 0; i < outer.length; i++) {
            const a = outer[i];
            //1.计算距离
            const dx = holePt.x - a.x;
            const dy = holePt.y - a.y;
            const distSq = dx * dx + dy * dy;

            //2.如果比当前最好的还远，直接跳过
            if (distSq >= bestDistsq) continue;
            //3.桥连线(holePt -> outerPt)是否与外圈的任何一条边相交
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
            //4.是否与内圈自身的边相交（处理洞的凹陷部分）
            for (let j = 0; j < inner.length; j++) {
                const p1 = inner[j];
                const p2 = inner[(j + 1) % inner.length];
                // 忽略包含起点 holePt 的两条边
                if (p1 === holePt || p2 === holePt) continue;
                if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                    intersects = true;
                    break;
                }
            }
            if (intersects) continue;
            //5.是否与剩下几个未合并的内轮廓边相交
            for (let k = innerid + 1; k < holos.length; k++) {
                for (let j = 0; j < holos[k].length; j++) {
                    const p1 = holos[k][j];
                    const p2 = holos[k][(j + 1) % holos[k].length];
                    if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                        intersects = true;
                        break;
                    }
                }
            }
            if (!intersects) {
                bestDistsq = distSq;
                bestIdx = i;
            }
        }

        return bestIdx;
    }
    /**
     * @param {Contour[]} outer
     * @param {Contour[][]} holes
     * @param {number} holeid
     */
    mergeHoleIntoOuter(outer, holes, holeid) {
        const hole = holes[holeid];
        let oi = -1;
        let holePt = hole[0];
        let hi = 0;
        for (hi = 0; hi < hole.length; hi++) {
            holePt = hole[hi];
            oi = this.findBridgeOuterIndex(holePt, outer, holes, holeid);
            if (oi >= 0) break;
        }
        if (oi < 0) {
            Instance.Msg("没有找到桥连接内外轮廓");
            return outer;
        }
        /**@type {Contour[]} */
        const merged = [];

        // 1. outer → bridge 点
        for (let i = 0; i <= oi; i++) {
            merged.push(outer[i]);
        }

        // 2. bridge → hole 起点
        merged.push(holePt);

        // 3. 绕 hole 一圈（从 hi 开始到hole 起点及hi）
        for (let i = 1; i <= hole.length; i++) {
            merged.push(hole[(hi + i) % hole.length]);
        }

        // 4. 回到 outer bridge 点
        merged.push(outer[oi]);

        // 5. outer 剩余部分
        for (let i = oi + 1; i < outer.length; i++) {
            merged.push(outer[i]);
        }

        return merged;
    }
    mergeRegionContours() {
        /**@type {Map<number,Contour[][]>} */
        const byRegion = new Map();

        //按region分组
        for (const c of this.contours) {
            const rid = c[0].regionId;
            if (!byRegion.has(rid)) byRegion.set(rid, []);
            byRegion.get(rid)?.push(c);
        }

        const mergedContours = [];

        for (const [rid, contours] of byRegion) {
            /**@type {Contour[]} */
            let outer = [];
            const holes = [];

            for (const c of contours) {
                if (this.computeSignedArea(c) > 0) {
                    outer = c;
                } else {
                    holes.push(c);
                }
            }

            if (!outer) continue;
            //输出区域有几个内轮廓，和多边形生成时报错比较
            //if(holes.length>0)
            //{
            //    Instance.Msg(rid+"=="+holes.length);
            //}

            //待更新：给hole排序，按某一坐标从小到大

            //逐个把hole融进outer
            for (let i = 0; i < holes.length; i++) {
                outer = this.mergeHoleIntoOuter(outer, holes, i);
            }
            //outer=this.fixWinding(outer);
            mergedContours.push(outer);
        }

        // 用融合后的结果替换
        this.contours = mergedContours;
    }

    init() {
        /** @type {Set<string>} */
        const visited = new Set();

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            for (let dir = 0; dir < 4; dir++) {
                                if (this.isBoundaryEdge(span, dir)) {

                                    const key = this.edgeKey(x, y, span, dir);
                                    if (visited.has(key)) continue;

                                    let contour = this.traceContour(x, y, span, dir, visited);
                                    if (contour && contour.length >= 3) {
                                        //外轮廓：逆时针（CCW）
                                        //洞轮廓：顺时针（CW）
                                        contour.length;
                                        contour = this.simplifyContour(contour);
                                        if (contour && contour.length >= 3) {
                                            this.contours.push(contour);
                                            //Instance.Msg(`{${l}}=>{${contour.length}}`);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    span = span.next;
                }
            }
        }
        this.mergeRegionContours();
    }
    /**
     * @param {Contour[]} contour
     */
    simplifyContour(contour) {
        const n = contour.length;
        if (n < 4) return contour.slice();
        const pts = contour.slice();
        const locked = new Array(n).fill(0);
        let locknum = 0;
        for (let i = 0; i < n; i++) {
            contour[(i + n - 1) % n];
            const cur = contour[i];
            const next = contour[(i + 1) % n];
            // portal 端点,相邻的1相连
            if (next.neighborRegionId != cur.neighborRegionId) {
                locked[i] += 1;
                locknum++;
                //Instance.DebugSphere({center:cur,radius:8,duration:120,color:{r:255,g:0,b:0}});
                //区域切换端点
            }
            //方向发生变化，相邻的2相连
            //if (cur.neighborRegionId==0&&!isCollinear(cur,prev,next)) {
            //    //Instance.DebugSphere({center:{x:cur.x,y:cur.y,z:cur.z+10},radius:8,duration:120,color:{r:0,g:255,b:0}});
            //    //转弯端点
            //    locked[i] += 2;
            //}
        }
        if (locknum == 0) {
            let minPt = pts[0];
            let maxPt = pts[0];
            let minid = 0;
            let maxid = 0;
            for (let i = 0; i < n; i++) {
                const pt = pts[i];
                if (pt.x < minPt.x || pt.y < minPt.y) {
                    minPt = pt;
                    minid = i;
                }
                if (pt.x > maxPt.x || pt.y > maxPt.y) {
                    maxPt = pt;
                    maxid = i;
                }
            }
            //没有强制顶点，判断为四周都是边境，或者四周都是某一区域，手动指定两个强制顶点
            locked[minid] = 1;
            locked[maxid] = 1;
        }
        /**@type {Contour[]}*/
        const out = [];
        let i = 0;
        let minI = -1;
        let maxJ = n;
        while (i < n - 1) {
            if (locked[i] == 0) {
                i++;
                continue;
            }
            if (minI == -1) minI = i;
            let j = (i + 1);
            while (j < n - 1 && locked[j] == 0) {
                j = j + 1;
            }
            if (locked[j]) maxJ = j;
            /**
             * @type {Contour[]}
             */
            if (locked[i] && locked[j]) this.simplifySegmentbymaxErrorSq(pts, locked, i, j, out);
            i = j;
        }
        this.simplifySegmentbymaxErrorSqFinally(pts, locked, maxJ, minI, out, n);
        return out;
    }
    /**
     * @param {Contour[]} pts
     * @param {number[]} locked
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     * @param {number}n
     */
    simplifySegmentbymaxErrorSqFinally(pts, locked, i0, i1, out, n) {
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
        const maxErrorSq = CONT_MAX_ERROR * CONT_MAX_ERROR;

        if (index !== -1 && maxDistSq > maxErrorSq) {
            if (index < i0) this.simplifySegmentbymaxErrorSqFinally(pts, locked, i0, index, out, n);
            else this.simplifySegmentbymaxErrorSq(pts, locked, i0, index, out);
            if (index < i1) this.simplifySegmentbymaxErrorSq(pts, locked, index, i1, out);
            else this.simplifySegmentbymaxErrorSqFinally(pts, locked, index, i1, out, n);
        } else {
            //只输出起点
            out.push(a);
        }

    }
    /**
     * @param {Contour[]} pts
     * @param {number[]} locked
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     */
    simplifySegmentbymaxErrorSq(pts, locked, i0, i1, out) {
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
        const maxErrorSq = CONT_MAX_ERROR * CONT_MAX_ERROR;
        if (index !== -1 && maxDistSq > maxErrorSq) {
            this.simplifySegmentbymaxErrorSq(pts, locked, i0, index, out);
            this.simplifySegmentbymaxErrorSq(pts, locked, index, i1, out);
        } else {
            //只输出起点
            out.push(a);
        }
    }
    /**
     * @param {Contour[]} contour
     */
    computeSignedArea(contour) {
        let area = 0;
        const n = contour.length;
        for (let i = 0; i < n; i++) {
            const p = contour[i];
            const q = contour[(i + 1) % n];
            area += (p.x * q.y - q.x * p.y);
        }
        return area * 0.5;
    }
    /**
     * abandon
     * @param {Contour[]} contour
     */
    fixWinding(contour) {
        const area = this.computeSignedArea(contour);
        //外轮廓CCW（area>0）
        if (area < 0) {
            contour.reverse();
        }
        return contour;
    }

    /**
     * @param {number} sx 起始 cell x
     * @param {number} sy 起始 cell y
     * @param {OpenSpan} startSpan
     * @param {number} startDir 起始边方向
     * @returns {Contour[] | null}
     * @param {Set<string>} visited
     */
    traceContour(sx, sy, startSpan, startDir, visited) {
        let x = sx;
        let y = sy;
        let span = startSpan;
        let dir = startDir;

        const verts = [];

        let iter = 0;
        const MAX_ITER = this.gridX * this.gridY * 4;
        if (!this.isBoundaryEdge(startSpan, startDir)) return null;
        const startKey = this.edgeKey(x, y, span, dir);
        while (iter++ < MAX_ITER) {
            const key = this.edgeKey(x, y, span, dir);
            //回到起点
            if (key === startKey && verts.length > 0) break;
            if (visited.has(key)) {
                Instance.Msg("奇怪的轮廓边,找了一遍现在又找一遍");
                return null;
            }
            visited.add(key);

            //只有在boundary边才输出顶点
            if (this.isBoundaryEdge(span, dir)) {
                const c = this.corner(x, y, dir);

                const h = this.getCornerHeightFromEdge(x, y, span, dir);
                const nid = this.getNeighborregionid(span, dir);
                //Instance.Msg(nid);
                if (h !== null) {
                    verts.push({
                        x: origin.x + c.x * MESH_CELL_SIZE_XY-MESH_CELL_SIZE_XY/2,
                        y: origin.y + c.y * MESH_CELL_SIZE_XY-MESH_CELL_SIZE_XY/2,
                        z: origin.z + h * MESH_CELL_SIZE_Z,
                        regionId: span.regionId,      //当前span的region
                        neighborRegionId: nid   //对面span的region（或 0）
                    });
                }
            }

            //顺序：右转 → 直行 → 左转 → 后转
            let advanced = false;
            for (let i = 0; i < 4; i++) {
                const ndir = (dir + 3 - i + 4) % 4;
                const nspan = span.neighbors[ndir];
                //如果这条边是boundary，就沿边走
                if (!nspan || nspan.regionId !== span.regionId) {
                    dir = ndir;
                    advanced = true;
                    break;
                }
                //否则穿过这条边
                const p = this.move(x, y, ndir);
                x = p.x;
                y = p.y;
                span = nspan;
                dir = (ndir + 2) % 4;
                advanced = true;
                break;
            }

            if (!advanced) {
                Instance.Msg("轮廓断啦");
                return null;
            }
        }
        if (verts.length < 3) return null;
        return verts;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {OpenSpan} span
     * @param {number} dir
     */
    getCornerHeightFromEdge(x, y, span, dir) {
        let maxFloor = span.floor;
        const leftDir = (dir + 3) & 3;
        //左侧
        const p1 = this.move(x, y, leftDir);
        //前方
        const p2 = this.move(x, y, dir);
        //左前方
        const p3 = this.move(p1.x, p1.y, dir);

        //左侧能到的span
        if (this.inBounds(p1.x, p1.y)) {
            /**@type {OpenSpan|null} */
            let s = this.hf[p1.x][p1.y];
            while (s) {
                if(s.use)
                {
                    if (span.canTraverseTo(s)) {
                        if (maxFloor < s.floor) {
                            maxFloor = s.floor;
                        }
                    }
                }
                s = s.next;
            }
        }
        //前方能到的span
        if (this.inBounds(p2.x, p2.y)) {
            /**@type {OpenSpan|null} */
            let s = this.hf[p2.x][p2.y];
            while (s) {
                if(s.use)
                {
                    if (span.canTraverseTo(s)) {
                        if (maxFloor < s.floor) {
                            maxFloor = s.floor;
                        }
                    }
                }
                s = s.next;
            }
        }
        //对角能到的span
        if (this.inBounds(p3.x, p3.y)) {
            /**@type {OpenSpan|null} */
            let s = this.hf[p3.x][p3.y];
            while (s) {
                if(s.use)
                {
                    if (span.canTraverseTo(s)) {
                        if (maxFloor < s.floor) {
                            maxFloor = s.floor;
                        }
                    }
                }
                s = s.next;
            }
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

    debugDrawContours(duration = 5) {
        Instance.Msg(`一共${this.contours.length}个轮廓`);
        for (const contour of this.contours) {
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            const z = Math.random() * 20;
            for (let i = 0; i < contour.length; i++) {
                const a = contour[i];
                const b = contour[(i + 1) % contour.length];
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
                //if(a.neighborRegionId!=b.neighborRegionId)
                //{
                //Instance.DebugSphere({center:start,radius:8,duration,color});
                //}
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
 * @property {number} regionId
 * @property {number} neighborRegionId
 */

class PolyMeshBuilder {

    /**
     * @param {import("./path_contourbuilder").Contour[][]} contours
     */
    constructor(contours) {
        /** @type {import("./path_contourbuilder").Contour[][]} */
        this.contours = contours;

        /** @type {import("cs_script/point_script").Vector[]} */
        this.verts = [];
        /** @type {number[][]} */
        this.polys = [];
        /** @type {number[]} */
        this.regions = [];
        /** @type {number[][]} */
        this.neighbors = [];
    }

    init() {
        const unmerged=[];
        for (const contour of this.contours) {
            const pl=this.mergeTriangles(this.triangulate(contour),POLY_MERGE_LONGEST_EDGE_FIRST);
            for (const p of pl) {
                unmerged.push(p);
            }
        }
        //const merged = this.mergeTriangles(unmerged,false);
        //不能区域间merge，detail需要使用区域id
        for (const p of unmerged) {
            this.addPolygon(p);
        }
        this.buildAdjacency();
    }
    return() {
        return {
            verts: this.verts,
            polys: this.polys,
            regions: this.regions,
            neighbors: this.neighbors
        }
    }
    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    triangulate(poly) {
        //
        const debugid = -1;

        const verts = poly.slice();
        const result = [];

        let guard = 0;
        {
            while (verts.length > 3 && guard++ < 5000) {
                let bestEar=null;
                let minPerimeter=Infinity;
                let bestIndex=-1;

                for (let i = 0; i < verts.length; i++) {
                    const prev = verts[(i - 1 + verts.length) % verts.length];
                    const cur = verts[i];
                    const next = verts[(i + 1) % verts.length];
                    //cur对应的角度是否<180度
                    if (!isConvex(prev, cur, next)) continue;
                    //这三个点构成的三角形是否把剩下几个点包含进去了，也就是和已有边相交了
                    let contains = false;
                    for (let j = 0; j < verts.length; j++) {
                        if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                        if (pointInTri(verts[j], prev, cur, next)) {
                            contains = true;
                            break;
                        }
                    }
                    if (contains) continue;
                    // 其他端点不能在新生成的边上,如果在边上，判断那个点与这边上两点是否在同一位置
                    for (let j = 0; j < verts.length; j++) {
                        if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                        if (distPtSegSq(verts[j], prev, next) == 0) //判断点p是否在ab线段上
                        {
                            if (posDistance2Dsqr(prev, verts[j]) == 0 || posDistance2Dsqr(next, verts[j]) == 0) continue;
                            contains = true;
                            break;
                        }
                    }
                    if (contains) continue;
                    const perimeter = 
                    Math.sqrt(posDistance2Dsqr(prev, cur)) +
                    Math.sqrt(posDistance2Dsqr(cur, next)) +
                    Math.sqrt(posDistance2Dsqr(next, prev));
                
                    // 找到周长最短的耳朵
                    if (perimeter < minPerimeter) {
                        minPerimeter = perimeter;
                        bestEar = {prev, cur, next};
                        bestIndex = i;
                    }
                }
                // 如果找到了最佳耳朵，割掉它
                if (bestEar && bestIndex !== -1) {
                    result.push([bestEar.prev, bestEar.cur, bestEar.next]);
                    verts.splice(bestIndex, 1);
                } else {
                    // 找不到耳朵，退出循环
                    break;
                }
            }
        }

        if (verts.length == 3) {
            result.push([verts[0], verts[1], verts[2]]);
        }
        if (verts.length != 3) {
            //debug
            if (verts[0].regionId == debugid) {
                Instance.Msg(poly.length);
                for (let i = 0; i < poly.length; i++) {
                    const a = poly[i];
                    const b = poly[(i + 1) % poly.length];
                    Instance.DebugLine({ start: a, end: b, color: { r: 125, g: 125, b: 0 }, duration: 30 });
                }
                for (let i = 0; i < verts.length; i++) {
                    const prev = verts[(i - 1 + verts.length) % verts.length];
                    const cur = verts[i];
                    const next = verts[(i + 1) % verts.length];
                    //cur对应的角度是否<180度
                    if (!isConvex(prev, cur, next)) {
                        Instance.DebugSphere({ center: cur, radius: 2, duration: 30, color: { r: 255, g: 0, b: 0 } });
                        continue;
                    }
                    //这三个点构成的三角形是否把剩下几个点包含进去了，也就是和已有边相交了
                    let contains = false;
                    for (let j = 0; j < verts.length; j++) {
                        if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                        if (pointInTri(verts[j], prev, cur, next)) {
                            contains = true;
                            break;
                        }
                    }
                    if (contains) {
                        Instance.DebugSphere({ center: cur, radius: 5, duration: 30, color: { r: 0, g: 255, b: 0 } });
                        continue;
                    }
                    // 其他端点不能在新生成的边上,如果在边上，判断那个点与这边上两点是否在同一位置
                    for (let j = 0; j < verts.length; j++) {
                        if (j == i || j == ((i - 1 + verts.length) % verts.length) || j == ((i + 1) % verts.length)) continue;
                        if (distPtSegSq(verts[j], prev, next) == 0) {
                            if (posDistance2Dsqr(prev, verts[j]) == 0 || posDistance2Dsqr(next, verts[j]) == 0) continue;
                            contains = true;
                            break;
                        }
                    }
                    if (contains) {
                        Instance.DebugSphere({ center: cur, radius: 5, duration: 30, color: { r: 0, g: 0, b: 255 } });
                        continue;
                    }
                }
                //verts.forEach((e)=>{
                //    Instance.DebugSphere({center:e,radius:5,duration:30,color:{r:255,g:0,b:0}});
                //});
            }
            Instance.Msg("区域：" + poly[0].regionId + "：出现奇怪小错误,耳割法无法分割多边形,猜测,简化轮廓中的两个点,拥有同一个x,y值");
        }
        return result;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[][]} tris
     * @param {boolean}config//是否合并最长边
     */
    mergeTriangles(tris,config) {
        const polys = tris.map(t => t.slice());
        let merged=true;
        if(config)
        {
            while (merged) {
                merged = false;
                let bdist=-Infinity;
                let bi=-1;
                let bj=-1;
                let binfo=null;
                for (let i = 0; i < polys.length; i++) {
                    for (let j = i + 1; j < polys.length; j++) {
                        const info = this.getMergeInfo(polys[i], polys[j]);
                        if (info&&info.dist>bdist) {
                            bdist=info.dist;
                            bi=i;
                            bj=j;
                            binfo=info.info;
                        }
                    }
                }
                if(binfo)
                {
                    polys[bi] = binfo;
                    polys.splice(bj, 1);
                    merged = true;
                }
            }
        }
        else
        {
            while (merged) {
                merged = false;
                outer:
                for (let i = 0; i < polys.length; i++) {
                    for (let j = i + 1; j < polys.length; j++) {
                        const info = this.getMergeInfo(polys[i], polys[j]);
                        if (info) {
                            polys[i] = info.info;
                            polys.splice(j, 1);
                            merged = true;
                            break outer;
                        }
                    }
                }
            }
        }
        return polys;
    }
    /**
     * [新增] 获取合并信息，包含合并后的多边形和公共边长度
     * @param {{x:number,y:number,z:number,regionId:number}[]} a
     * @param {{x:number,y:number,z:number,regionId:number}[]} b
     */
    getMergeInfo(a, b) {
        let ai = -1, bi = -1;

        // 寻找公共边
        for (let i = 0; i < a.length; i++) {
            const aNext = (i + 1) % a.length;
            for (let j = 0; j < b.length; j++) {
                const bNext = (j + 1) % b.length;
                // 判断边是否重合 (a[i]->a[i+1] == b[j+1]->b[j])
                if (posDistance3Dsqr(a[i],b[bNext])<=1&&posDistance3Dsqr(a[aNext],b[j])<=1) {
                    ai = i;
                    bi = j;
                    break;
                }
            }
            if (ai != -1) break;
        }
        //面积都是>0的，都是逆时针
        if (ai < 0) return null;
        //Instance.DebugLine({start:a[ai],end:b[bi],duration:60,color:{r:255,g:0,b:0}});
        // 构建合并后的数组
        const merged = [];
        const nA = a.length;
        const nB = b.length;
        for (let i = 0; i < nA - 1; i++)
            merged.push(a[(ai + 1 + i) % nA]);
        for (let i = 0; i < nB - 1; i++)
            merged.push(b[(bi + 1 + i) % nB]);
        // [关键步骤] 移除共线点，加入后，这个点对应的角是180度，就可以去除这个点
        //this.removeCollinearPoints(merged);

        // 检查顶点数量限制
        if (merged.length > POLY_MAX_VERTS_PER_POLY) return null;

        // 检查凸性
        if (!this.isPolyConvex(merged)) return null;

        // 计算公共边长度 (用于优先权排序)
        const v1 = a[ai];
        const v2 = a[(ai + 1) % nA];
        const distSq = (v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2; // 只计算XY平面距离

        return {info:merged,dist:distSq};
    }
    /**
     * [新增] 移除多边形中三点共线的冗余顶点
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly 
     */
    removeCollinearPoints(poly) {
        for (let i = 0; i < poly.length; i++) {
            const n = poly.length;
            if (n <= 3) break; // 三角形不能再减了

            const prev = poly[(i - 1 + n) % n];
            const cur = poly[i];
            const next = poly[(i + 1) % n];

            // 使用面积法判断三点共线 (Area接近0)
            // area() 已经在你的 path_const 中引入了
            // 注意：这里需要一个极小的容差，防止浮点数误差
            if (Math.abs(area(prev, cur, next))<=1) { 
                poly.splice(i, 1);
                i--; // 索引回退，检查新的组合
                //Instance.DebugSphere({center:cur,radius:2,duration:60,color:{r:255,g:0,b:0}});
            }
        }
    }
    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    isPolyConvex(poly) {
        const n = poly.length;
        for (let i = 0; i < n; i++) {
            if (area(poly[i],poly[(i + 1) % n],poly[(i + 2) % n]) < -1) return false;
        }
        return true;
    }

    /**
     * @param {{x:number,y:number,z:number,regionId:number}[]} poly
     */
    addPolygon(poly) {
        const idx = [];
        for (const v of poly) {
            let i = this.verts.findIndex(
                //p=>Math.abs(p.x-v.x)<=0.5&&Math.abs(p.y-v.y)<=0.5&&Math.abs(p.z-v.z)<=0.5
                p => p.x === v.x && p.y === v.y && p.z === v.z
            );
            if (i < 0) {
                i = this.verts.length;
                this.verts.push({ x: v.x, y: v.y, z: v.z });
                //Instance.DebugSphere({center:{x:v.x,y:v.y,z:v.z+Math.random()*30},radius:8,duration:60,color:{r:0,g:0,b:0}});
            }
            idx.push(i);
        }
        this.polys.push(idx);
        this.regions.push(poly[0].regionId);
        this.neighbors.push(new Array(idx.length).fill(-1));
    }

    buildAdjacency() {
        /** edgeKey → {poly, edge} */
        const edgeMap = new Map();

        for (let pi = 0; pi < this.polys.length; pi++) {
            const poly = this.polys[pi];
            for (let ei = 0; ei < poly.length; ei++) {
                const a = poly[ei];
                const b = poly[(ei + 1) % poly.length];

                // 无向边：小索引在前
                const k = a < b ? `${a},${b}` : `${b},${a}`;

                if (!edgeMap.has(k)) {
                    edgeMap.set(k, { poly: pi, edge: ei });
                } else {
                    const other = edgeMap.get(k);
                    this.neighbors[pi][ei] = other.poly;
                    this.neighbors[other.poly][other.edge] = pi;
                }
            }
        }
    }
    debugDrawPolys(duration = 5) {
        for (let pi = 0; pi < this.polys.length; pi++) {
            const poly = this.polys[pi];
            //const color = getRandomColor();
            const color={r:255,g:255,b:0};
            const z = Math.random() * 40*0;
            for (let i = 0; i < poly.length; i++) {
                const start = posZfly(this.verts[poly[i]], z);
                const end = posZfly(this.verts[poly[(i + 1) % poly.length]], z);
                Instance.DebugLine({ start, end, color, duration });
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
        }
    }
    debugDrawAdjacency(duration = 15) {
        for (let i = 0; i < this.polys.length; i++) {
            const start = this.polyCenter(i);
            for (let e = 0; e < this.neighbors[i].length; e++) {
                const ni = this.neighbors[i][e];
                if (ni < 0) continue;
                // 只画一次，避免双向重复
                if (ni < i) continue;
                const end = this.polyCenter(ni);
                Instance.DebugLine({ start, end, color: { r: 255, g: 0, b: 255 }, duration });
            }
        }
    }
    /**
     * @param {number} pi
     */
    polyCenter(pi) {
        const poly = this.polys[pi];
        let x = 0, y = 0, z = 0;

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
                if (ni < 0) continue;
                // 只画一次
                if (ni < i) continue;
                const start = posZfly(this.verts[polyA[ei]], 20);
                const end = posZfly(this.verts[polyA[(ei + 1) % polyA.length]], 20);
                Instance.DebugLine({ start, end, color: { r: 0, g: 255, b: 0 }, duration });
            }
        }
    }
}

class PolyMeshDetailBuilder {
    /**
     * @param {{verts:import("cs_script/point_script").Vector[],polys:number[][],regions:number[],neighbors:number[][]}} mesh
     * @param {OpenHeightfield} hf
     */
    constructor(mesh, hf) {
        this.mesh = mesh;
        /**@type {OpenHeightfield} */
        this.hf = hf;
        /**@type {import("cs_script/point_script").Vector[]}*/
        this.verts = [];
        /**@type {number[][]}*/
        this.tris = [];
        /**@type {number[][]}*/
        this.meshes = [];
        /**@type {number[]} */
        this.triTopoly=[];
    }

    init() {
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
        //待更新：生成内部采样点时高度用三角剖分后的高度

        // 1. 为多边形边界顶点采样高度
        const borderVerts = this.applyHeights(polyVerts, this.hf,regionid);
        // 2. 计算边界平均高度和高度范围
        const borderHeightInfo = this.calculateBorderHeightInfo(borderVerts);
        // 3. 获取初始三角剖分（用于高度差异检查）
        const initialVertices = [...borderVerts];
        const initialConstraints = [];
        for (let i = 0; i < borderVerts.length; i++) {
            const j = (i + 1) % borderVerts.length;
            initialConstraints.push([i, j]);
        }
        // 4. 执行初始剖分（基于边界点）
        const trianglesCDT = new SimplifiedCDT(initialVertices, initialConstraints);
        let triangles = trianglesCDT.getTri();
        // 5. 生成内部包括边界采样点
        let rawSamples = this.buildDetailSamples(polyVerts, borderHeightInfo, this.hf,triangles,trianglesCDT.vertices,regionid);
        // 6. 过滤内部采样点：只保留高度差异大的点
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
                // 只有当高度差异超过阈值时才保留
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
     * 计算边界顶点的高度信息
     * @param {import("cs_script/point_script").Vector[]} borderVerts
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
     * @param {{ verts: import("cs_script/point_script").Vector[]; polys?: number[][]; regions?: number[]; neighbors?: number[][]; }} mesh
     * @param {number[]} poly
     */
    getPolyVerts(mesh, poly) {
        return poly.map(vi => mesh.verts[vi]);
    }
    /**
     * 生成内部采样点（带高度误差检查）
     * @param {import("cs_script/point_script").Vector[]} polyVerts
     * @param {{avgHeight: number;minHeight: number;maxHeight: number;heightRange: number;}} heightInfo
     * @param {OpenHeightfield} hf
     * @returns {import("cs_script/point_script").Vector[]}
     * @param {Triangle[]} initialTriangles
     * @param {import("cs_script/point_script").Vector[]} initialVertices
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
     * @param {import("cs_script/point_script").Vector} sample
     * @param {Triangle} tri
     * @param {import("cs_script/point_script").Vector[]} verts
     */
    isNearTriangleEdge(sample, tri, verts) {

        const dis = Math.min(distPtSegSq(sample,verts[tri.a],verts[tri.b]),distPtSegSq(sample,verts[tri.b],verts[tri.c]),distPtSegSq(sample,verts[tri.c],verts[tri.a]));
        if (dis < POLY_DETAIL_SAMPLE_DIST * 0.5) return true;
        return false;
    }
    /**
     * @param {import("cs_script/point_script").Vector[]} polyVerts
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
     * 在 [start, end] 之间递归插入高度偏差最大的点
     * @param {import("cs_script/point_script").Vector} start
     * @param {import("cs_script/point_script").Vector} end
     * @param {import("cs_script/point_script").Vector[]} samples // 该边上的细分点（不含 start/end）
     * @param {OpenHeightfield} hf
     * @param {number} regionid
     * @param {import("cs_script/point_script").Vector[]} outVerts
     */
    subdivideEdgeByHeight(start, end,samples,hf,regionid,outVerts) {
        let maxError = 0;
        let maxIndex = -1;
        let maxVert = null;

        const total = samples.length;

        for (let i = 0; i < total; i++) {
            const s = samples[i];
            const t = (i + 1) / (total + 1);

            // 如果不加该点时的插值高度
            const interpZ = start.z * (1 - t) + end.z * t;

            const h = this.sampleHeight(hf, s.x, s.y, interpZ, regionid);
            const err = Math.abs(h - interpZ);

            if (err > maxError) {
                maxError = err;
                maxIndex = i;
                maxVert = { x: s.x, y: s.y, z: h };
            }
        }

        // 没有需要加的点
        if (maxError <= POLY_DETAIL_HEIGHT_ERROR || maxIndex === -1||!maxVert) {
            return;
        }

        // 递归左半
        this.subdivideEdgeByHeight(
            start,
            maxVert,
            samples.slice(0, maxIndex),
            hf,
            regionid,
            outVerts
        );

        // 插入当前最大误差点（顺序保证）
        outVerts.push(maxVert);

        // 递归右半
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
     * @param {import("cs_script/point_script").Vector} start
     * @param {import("cs_script/point_script").Vector} end
     * @param {OpenHeightfield} hf
     * @param {number} sampleDist
     * @returns {import("cs_script/point_script").Vector[]}
     */
    sampleEdgeWithHeightCheck(start, end, hf, sampleDist) {
        const samples = [];
        
        // 计算边向量和长度
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const length = Math.sqrt(dx * dx + dy * dy);
        
        if (length <= 1e-6) {
            return []; // 边长度为0，不采样
        }
        
        // 计算方向向量
        const dirX = dx / length;
        const dirY = dy / length;
        // 计算采样点数（不包括起点和终点）
        const numSamples = Math.floor(length / sampleDist);
        
        // 记录上一个采样点的高度

        for (let i = 1; i <= numSamples; i++) {
            const t = i / (numSamples + 1); // 确保不采样到端点
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
        const ix = Math.floor((wx - origin.x) / MESH_CELL_SIZE_XY);
        const iy = Math.floor((wy - origin.y) / MESH_CELL_SIZE_XY);

        if (ix < 0 || iy < 0 || ix >= hf.gridX || iy >= hf.gridY) return fallbackZ;

        let best = null;
        let bestDiff = Infinity;
        /**@type {OpenSpan|null} */
        let span = hf.cells[ix][iy];
        while (span) {
            if(span.regionId==regionid)
            {
                const z = origin.z + span.floor * MESH_CELL_SIZE_Z;
                const d = Math.abs(z - fallbackZ);
                if (d < bestDiff) {
                    bestDiff = d;
                    best = z;
                }
            }
            span = span.next;
        }
        // 如果没有找到合适的span，开始螺旋式搜索
        if (best === null) {
            const maxRadius = Math.max(hf.gridX, hf.gridY); // 搜索的最大半径
            let radius = 1; // 初始半径
            out:
            while (radius <= maxRadius) {
                // 螺旋式外扩，检查四个方向
                for (let offset = 0; offset <= radius; offset++) {
                    // 检查 (ix + offset, iy + radius) 或 (ix + radius, iy + offset) 等位置
                    let candidates = [
                        [ix + offset, iy + radius], // 上
                        [ix + radius, iy + offset], // 右
                        [ix - offset, iy - radius], // 下
                        [ix - radius, iy - offset]  // 左
                    ];

                    for (const [nx, ny] of candidates) {
                        if (nx >= 0 && ny >= 0 && nx < hf.gridX && ny < hf.gridY) {
                            // 在有效范围内，查找对应的span
                            span = hf.cells[nx][ny];
                            while (span) {
                                if(span.regionId==regionid)
                                {
                                    const z = origin.z + span.floor * MESH_CELL_SIZE_Z;
                                    const d = Math.abs(z - fallbackZ);
                                    if (d < bestDiff) {
                                        bestDiff = d;
                                        best = z;
                                        break out;
                                    }
                                }
                                span = span.next;
                            }
                        }
                    }
                }
                // 增大半径，继续螺旋扩展
                radius++;
            }
        }

        // 如果最终没有找到合适的span，返回默认的fallbackZ
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

            // ===== 点在边上（直接算 outside）=====
            if (this.pointOnSegment2D(px, py, xi, yi, xj, yj)) {
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

    /**
     * 点是否在线段上（含端点）
     * @param {number} px
     * @param {number} py
     * @param {number} x1
     * @param {number} y1
     * @param {number} x2
     * @param {number} y2
     */
    pointOnSegment2D(px, py, x1, y1, x2, y2) {
        // 共线
        const cross = (px - x1) * (y2 - y1) - (py - y1) * (x2 - x1);
        if (Math.abs(cross) > 1e-6) return false;

        // 在线段范围内
        const dot =
            (px - x1) * (px - x2) +
            (py - y1) * (py - y2);

        return dot <= 0;
    }
}

/**
 * 简化的约束Delaunay三角剖分器（针对凸多边形优化）
 */
class SimplifiedCDT {
    /**
     * @param {import("cs_script/point_script").Vector[]} vertices 顶点列表
     * @param {number[][]} constraints 约束边列表
     */
    constructor(vertices, constraints) {
        this.vertices = vertices;
        this.constraints = constraints;
        /** @type {Triangle[]} */
        this.triangles = [];
        
        // 构建约束边的查找集
        this.constraintEdges = new Set();
        for (const [a, b] of constraints) {
            // 确保边是规范化的（小索引在前）
            const key = `${Math.min(a, b)}-${Math.max(a, b)}`;
            this.constraintEdges.add(key);
        }
        //初始剖分：耳割法
        this.earClipping(vertices);
    }

    /**
     * @returns {Triangle[]} 三角形顶点索引列表
     */
    getTri() {
        return this.triangles;
    }
    /**
     * @param {{x:number,y:number,z:number}[]} poly
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
                //cur对应的角度是否<180度
                if (!isConvex(prev, cur, next)) continue;
                //这三个点构成的三角形是否把剩下几个点包含进去了，也就是和已有边相交了
                let contains = false;
                for (let j = 0; j < verts.length; j++) {
                    if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                    if (pointInTri(poly[verts[j]], prev, cur, next)) {
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                // 其他端点不能在新生成的边上,如果在边上，判断那个点与这边上两点是否在同一位置
                for (let j = 0; j < verts.length; j++) {
                    if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                    if (distPtSegSq(poly[verts[j]], prev, next) == 0) //判断点p是否在ab线段上
                    {
                        if (posDistance2Dsqr(prev, poly[verts[j]]) == 0 || posDistance2Dsqr(next, poly[verts[j]]) == 0) continue;
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                const perimeter = 
                Math.sqrt(posDistance2Dsqr(prev, cur)) +
                Math.sqrt(posDistance2Dsqr(cur, next)) +
                Math.sqrt(posDistance2Dsqr(next, prev));
            
                // 找到周长最短的耳朵
                if (perimeter < minPerimeter) {
                    minPerimeter = perimeter;
                    bestEar = {p:verts[(i - 1 + verts.length) % verts.length], c:verts[i], n:verts[(i + 1) % verts.length]};
                    bestIndex = i;
                }
            }
            // 如果找到了最佳耳朵，割掉它
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
        }else Instance.Msg("细节多边形耳割法错误!");
    }
    /**
     * 简化的点插入方法（你的版本，稍作优化）
     * @param {import("cs_script/point_script").Vector} point
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
            // 点不在任何三角形内（可能在边上），尝试找到包含点的边
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

        // 只对这三条边进行局部优化，而不是全图扫描
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
        // 首先检查是否在约束边上
        for (const [a, b] of this.constraints) {
            if (this.pointOnSegment(p, this.vertices[a], this.vertices[b])) {
                Instance.Msg("点在约束边上");
                return;
            }
        }
        // 查找包含该点的边
        for (let i = 0; i < this.triangles.length; i++) {
            const tri = this.triangles[i];
            const edges = tri.edges();
            
            for (const [a, b] of edges) {
                if (this.isConstraintEdge(a, b)) continue;
                if (this.pointOnSegment(p, this.vertices[a], this.vertices[b])) {
                    // 找到共享这条边的另一个三角形
                    const otherTri = this.findAdjacentTriangleByEdge([a, b], tri);
                    
                    if (otherTri) {

                        // 移除两个共享这条边的三角形
                        this.triangles.splice(this.triangles.indexOf(tri), 1);
                        this.triangles.splice(this.triangles.indexOf(otherTri), 1);
                        
                        // 获取两个三角形中不在这条边上的顶点
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
     * 判断点是否在线段上
     * @param {{ x: any; y: any;}} p
     * @param {{ x: any; y: any;}} a
     * @param {{ x: any; y: any;}} b
     */
    pointOnSegment(p, a, b) {
        const cross = (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
        if (Math.abs(cross) > 1e-6) return false;
        
        const dot = (p.x - a.x) * (p.x - b.x) + (p.y - a.y) * (p.y - b.y);
        return dot <= 1e-6;
    }

    /**
     * 局部递归优化 (Standard Delaunay Legalization)
     * @param {number} pIdx 新插入的点
     * @param {number} v1 边的一个端点
     * @param {number} v2 边的另一个端点
     */
    legalizeEdge(pIdx, v1, v2) {
        // 检查是否是约束边，约束边不可翻转
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
     * 检查是否是约束边
     * @param {number} a
     * @param {number} b
     */
    isConstraintEdge(a, b) {
        const key = `${Math.min(a, b)}-${Math.max(a, b)}`;
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
     * 检查点是否在三角形的外接圆内
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
     * @param {import("cs_script/point_script").Vector} point
     * @param {import("cs_script/point_script").Vector[]} vertices
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
     * @param {number} x 点的x坐标
     * @param {number} y 点的y坐标
     * @param {import("cs_script/point_script").Vector[]} vertices
     * @returns {number} 插值高度
     */
    interpolateHeight(x, y, vertices) {
        const va = vertices[this.a];
        const vb = vertices[this.b];
        const vc = vertices[this.c];
        
        // 使用重心坐标插值
        const denom = (vb.y - vc.y) * (va.x - vc.x) + (vc.x - vb.x) * (va.y - vc.y);
        
        if (Math.abs(denom) < 1e-6) {
            // 三角形退化，返回三个顶点高度的平均值
            return (va.z + vb.z + vc.z) / 3;
        }
        
        const u = ((vb.y - vc.y) * (x - vc.x) + (vc.x - vb.x) * (y - vc.y)) / denom;
        const v = ((vc.y - va.y) * (x - vc.x) + (va.x - vc.x) * (y - vc.y)) / denom;
        const w = 1 - u - v;
        
        // 插值高度
        return u * va.z + v * vb.z + w * vc.z;
    }
}

class JumpLinkBuilder
{
    /**
     * @param {{ verts: import("cs_script/point_script").Vector[]; polys: number[][]; regions: number[]; neighbors: number[][]; }} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        this.jumpDist = MESH_CELL_SIZE_XY*6;
        this.jumpHeight = MAX_JUMP_HEIGHT*MESH_CELL_SIZE_Z;
        this.walkHeight = MAX_WALK_HEIGHT*MESH_CELL_SIZE_Z;
        this.linkdist=250;//可行走区域A和可行走区域B之间的每个跳点的最小距离
        /**@type {{ PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number;type: number; }[]}*/
        this.links = [];
        //存储每个多边形所属的连通区域 ID
        /**@type {number[] | Int32Array<ArrayBuffer>}*/
        this.islandIds=[];
    }
    /**
     * 收集所有的边界边
     */
    collectBoundaryEdges() {
        const edges = [];
        const { polys, verts, neighbors } = this.mesh;

        for (let i = 0; i < polys.length; i++) {
            const poly = polys[i];
            for (let j = 0; j < poly.length; j++) {
                // 如果没有邻居，就是边界边
                if (neighbors[i][j] < 0) {
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
        const nList = this.mesh.neighbors[idxA];
        return nList.includes(idxB);
    }
    /**
     * @param {import("cs_script/point_script").Vector} p1
     * @param {import("cs_script/point_script").Vector} p2
     * @param {import("cs_script/point_script").Vector} p3
     * @param {import("cs_script/point_script").Vector} p4
     */
    closestPtSegmentSegment(p1, p2, p3, p4) {
        // 算法来源：Real-Time Collision Detection (Graham Walsh)
        // 计算线段 S1(p1, p2) 和 S2(p3, p4) 之间的最近点
        
        const d1 = { x: p2.x - p1.x, y: p2.y - p1.y}; // 忽略 Z 轴分量参与距离计算
        const d2 = { x: p4.x - p3.x, y: p4.y - p3.y};
        const r = { x: p1.x - p3.x, y: p1.y - p3.y};

        const a = d1.x * d1.x + d1.y * d1.y; // Squared length of segment S1
        const e = d2.x * d2.x + d2.y * d2.y; // Squared length of segment S2
        const f = d2.x * r.x + d2.y * r.y;

        const EPSILON = 1e-6;

        // 检查线段是否退化成点
        if (a <= EPSILON && e <= EPSILON) {
            // 两个都是点
            return { distSq: posDistance2Dsqr(p1, p3), ptA: p1, ptB: p3 };
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

        // 计算最近点坐标 (包含 Z)
        // 注意：这里的 t 和 s 是在 XY 平面上算出来的比例
        // 我们将其应用到 3D 坐标上，得到线段上的实际 3D 点
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
            distSq: posDistance2Dsqr(ptA, ptB),
            ptA,
            ptB
        };
    }

    init() {
        this.buildConnectivity();
        const boundaryEdges = this.collectBoundaryEdges();
        // Key: "polyA_polyB", Value: { targetPoly, dist, startPos, endPos }
        const bestJumpPerPoly = new Map();

        for (let i = 0; i < boundaryEdges.length; i++) {
            for (let j = i + 1; j < boundaryEdges.length; j++) {
                const edgeA = boundaryEdges[i];
                const edgeB = boundaryEdges[j];

                // 1. 排除同一个多边形的边
                if (edgeA.polyIndex === edgeB.polyIndex) continue;
                // B. [核心需求] 如果已经在同一个连通区域（能走过去），排除
                //if (this.islandIds[edgeA.polyIndex] === this.islandIds[edgeB.polyIndex])continue; 
                // 2. 排除已经是邻居的多边形 (可选，看你是否想要捷径)
                //if (this.areNeighbors(edgeA.polyIndex, edgeB.polyIndex)) continue;

                // 3. 计算两条线段在 XY 平面上的最近距离
                // 我们主要关心水平距离是否足够近
                const closestResult = this.closestPtSegmentSegment(
                    edgeA.p1, edgeA.p2, 
                    edgeB.p1, edgeB.p2
                );

                // 如果计算失败（平行重叠等极端情况），跳过
                if (!closestResult) continue;

                const { distSq, ptA, ptB } = closestResult;
                // 5. 距离判断
                if (distSq > this.jumpDist * this.jumpDist) continue;

                //如果a和b在同一个可行走区域，并且没有跳跃的捷径，就跳过
                if(this.islandIds[edgeA.polyIndex] === this.islandIds[edgeB.polyIndex]&&Math.abs(ptA.z-ptB.z)<=this.walkHeight)continue;
                // 4. 高度判断 (Z轴)
                const heightDiff = Math.abs(ptA.z - ptB.z);
                if (heightDiff > this.jumpHeight) continue;
                //同一点跳过
                if (heightDiff <1&&distSq < 1) continue;
                //Instance.DebugLine({start:ptA,end:ptB,duration:60,color:{r:255,g:0,b:0}});
                // 6. 记录候选
                this.updateBestCandidate(bestJumpPerPoly, edgeA.polyIndex, edgeB.polyIndex, distSq, ptA, ptB);
            }
        }
        // 3. 根据 linkdist 过滤掉靠得太近的跳点
        // 我们需要按距离从短到长排序，优先保留质量最高的跳点
        const sortedCandidates = Array.from(bestJumpPerPoly.values()).sort((a, b) => a.distSq - b.distSq);
        
        const finalLinks = [];

        for (const cand of sortedCandidates) {
            const islandA = this.islandIds[cand.startPoly];
            const islandB = this.islandIds[cand.endPoly];

            // 检查在这两个区域(Island)之间，是否已经存在位置太近的跳点
            let tooClose = false;
            for (const existing of finalLinks) {
                const exIslandA = this.islandIds[existing.PolyA];
                const exIslandB = this.islandIds[existing.PolyB];

                // 如果这两个跳点连接的是相同的两个岛屿
                if ((islandA === exIslandA && islandB === exIslandB) || 
                    (islandA === exIslandB && islandB === exIslandA)) {
                    
                    // 检查起点或终点的欧几里得距离是否小于 linkdist
                    const dSqStart = posDistance3Dsqr(cand.startPos, existing.PosA);
                    const dSqEnd = posDistance3Dsqr(cand.endPos, existing.PosB);

                    if (dSqStart < this.linkdist * this.linkdist || dSqEnd < this.linkdist * this.linkdist) {
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
                    cost: Math.sqrt(cand.distSq) * 1.5,
                    type: (Math.abs(cand.startPos.z-cand.endPos.z)<=this.walkHeight?0:1)
                });
            }
        }

        this.links = finalLinks;
        return this.links;
    }
    /**
     * 计算多边形网格的连通分量
     * 给互相连接的多边形打上相同的标识符码
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
                //遍历该多边形的所有邻居
                if(!u)break;
                const neighbors = this.mesh.neighbors[u];
                for (let v of neighbors) {
                    //v是邻居多边形的索引。如果是负数表示边界，跳过
                    if (v >= 0 && this.islandIds[v] === -1) {
                        this.islandIds[v] = currentId;
                        stack.push(v);
                    }
                }
            }
        }
        Instance.Msg(`共有${currentId}个独立行走区域`);
    }
    /**
     * @param {Map<string,any>} map
     * @param {number} idxA
     * @param {number} idxB
     * @param {number} distSq 两个多边形边界边之间的最短平方距离
     * @param {import("cs_script/point_script").Vector} ptA
     * @param {import("cs_script/point_script").Vector} ptB
     */
    updateBestCandidate(map, idxA, idxB, distSq, ptA, ptB) {
        //检查是否已经记录过这个多边形的跳跃目标
        const id1 = Math.min(idxA, idxB);
        const id2 = Math.max(idxA, idxB);
        const key = `${id1}_${id2}`;

        const existing = map.get(key);
        //如果还没有记录，或者发现了一个更近的目标（distSq 更小）
        if (!existing || distSq < existing.distSq) {
            map.set(key, {
                startPoly: idxA,
                endPoly: idxB,
                distSq: distSq,
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

class FunnelHeightFixer {
    /**
     * @param {{verts:import("cs_script/point_script").Vector[],polys:number[][]}} navMesh
     * @param {{verts:{x:number,y:number,z:number}[], tris:number[][],meshes:number[][], triTopoly:number[]}} detailMesh
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
        while (polyid < polyPath.length &&!this._pointInPolyXY(pos, polyPath[polyid].id))polyid++;

        if (polyid >= polyPath.length) return;
        const h = this._getHeightOnDetail(polyPath[polyid].id, pos);
        out.push({
            pos: { x: pos.x, y: pos.y, z: h },
            mode: 1
        });
    }
    /**
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} funnelPath
     * @param {{id:number,mode:number}[]} polyPath
     */
    fixHeight(funnelPath,polyPath) {
        if (funnelPath.length === 0) return [];

        const result = [];
        let polyIndex = 0;

        for (let i = 0; i < funnelPath.length - 1; i++) {
            const curr = funnelPath[i];
            const next = funnelPath[i + 1];

            // 跳点：直接输出，不插值
            if (next.mode == 2) {
                result.push(curr);
                continue;
            }
            if(curr.mode == 2)result.push(curr);
            // 分段采样
            const samples = this._subdivide(curr.pos, next.pos);
            let preh=curr.pos.z;
            let prep=curr;
            for (let j = (curr.mode == 2)?1:0; j < samples.length; j++) {
                const p = samples[j];
                // 跳过重复首点
                //if (result.length > 0) {
                //    const last = result[result.length - 1].pos;
                //    if (posDistance2Dsqr(last, p) < 1e-4) continue;
                //}
                const preid=polyIndex;
                // 推进 poly corridor
                while (polyIndex < polyPath.length &&!this._pointInPolyXY(p, polyPath[polyIndex].id))polyIndex++;

                if (polyIndex >= polyPath.length) break;
                //如果这个样本点比前一个点高度发生足够变化，就在中间加入一个样本点
                const h = this._getHeightOnDetail(polyPath[polyIndex].id, p);
                if(j>0&&Math.abs(preh-h)>5)
                {
                    const mid={x:(p.x+prep.pos.x)/2,y:(p.y+prep.pos.y)/2,z:p.z};
                    this.addpoint(mid,preid,polyPath,result);
                }
                result.push({
                    pos: { x: p.x, y: p.y, z: h },
                    mode: 1
                });
                preh=h;
                prep=result[result.length - 1];
            }
        }

        // 最后一个点
        result.push(funnelPath[funnelPath.length - 1]);
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
     * @param {{ y: number; x: number; }} p
     * @param {number} polyId
     */
    _pointInPolyXY(p, polyId) {
        const poly = this.navMesh.polys[polyId];
        let inside = false;

        for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
            const vi = this.navMesh.verts[poly[i]];
            const vj = this.navMesh.verts[poly[j]];

            if (
                ((vi.y > p.y) !== (vj.y > p.y)) &&
                (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x)
            ) {
                inside = !inside;
            }
        }
        return inside;
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
}

class NavMesh {
    constructor() {
        /**@type {OpenHeightfield} */
        this.hf;
        /**@type {RegionGenerator} */
        this.regionGen;
        /**@type {ContourBuilder} */
        this.contourBuilder;
        /**@type {PolyMeshBuilder} */
        this.polyMeshGenerator;
        /**@type {PolyMeshDetailBuilder} */
        this.polidetail;
        /**@type {JumpLinkBuilder} */
        this.jumplinkbuilder;
        /**@type {PolyGraphAStar} */
        this.astar;
        /**@type {{verts: {x: number;y: number;z: number;}[];polys: number[][];regions: number[];neighbors: number[][];}} */
        this.mesh;
        /**@type {{verts: {x: number;y: number;z: number;}[];tris:number[][];triTopoly:number[];meshes: number[][];}} */
        this.meshdetail;
        /**@type {FunnelPath} */
        this.funnel;
        /**@type {FunnelHeightFixer} */
        this.heightfixer;
        /**@type {{ PolyA: number; PolyB: number; PosA: { x: number; y: number; z: number; }; PosB: { x: number; y: number; z: number; }; cost: number;type: number; }[]} */
        this.links;
        /** @type {number[]}*/
        this._polyAreas = [];
        /**@type {number[]}*/
        this._polyPrefix = [];
        /**@type {number}*/
        this._totalPolyArea;
    }
    /**
     * @param {number} [duration]
     */
    debug(duration = 5) {
    }
    debugLoad(duration = 5) {
        for (let pi = 0; pi < this.mesh.polys.length; pi++) {
            const poly = this.mesh.polys[pi];
            const color = {r:255,g:0,b:0};
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({ start, end, color, duration });
            }
        }
        for (const link of this.links) {
            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: { r: 0, g: (link.type==1?255:0), b: 255 },
                duration
            });
            Instance.DebugSphere({ center: link.PosA, radius: 4, color: { r: 0, g: 255, b: 0 }, duration });
            //Instance.DebugSphere({ center: link.endPos, radius: 4, color: { r: 255, g: 0, b: 0 }, duration });
            const poly = this.mesh.polys[link.PolyB];
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({start,end,color:{ r: 255, g: 0, b: 255 },duration});
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
        }
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
            links: this.jumplinkbuilder ? this.jumplinkbuilder.links : [],
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
            let start = new Date();
            //创建世界网格
            this.hf = new OpenHeightfield();
            this.hf.init();
            this.hf.findcanwalk();
            this.hf.deleteboundary();
            let end = new Date();
            Instance.Msg(`网格生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;
            //分割区域
            start = new Date();
            this.regionGen = new RegionGenerator(this.hf);
            this.regionGen.init();
            end = new Date();
            Instance.Msg(`区域生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;
            //轮廓生成
            start = new Date();
            this.contourBuilder = new ContourBuilder(this.hf);
            this.contourBuilder.init();
            end = new Date();
            Instance.Msg(`轮廓生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;
            //生成多边形网格
            start = new Date();
            this.polyMeshGenerator = new PolyMeshBuilder(this.contourBuilder.contours);
            this.polyMeshGenerator.init();
            this.mesh = this.polyMeshGenerator.return();
            end = new Date();
            Instance.Msg(`多边形生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;

            //构建细节网格
            start = new Date();
            this.polidetail = new PolyMeshDetailBuilder(this.mesh, this.hf);
            this.meshdetail = this.polidetail.init();
            end = new Date();
            Instance.Msg(`细节多边形生成完成,耗时${end.getTime() - start.getTime()}ms`);

            //return;
            start = new Date();
            this.jumplinkbuilder=new JumpLinkBuilder(this.mesh);
            this.links=this.jumplinkbuilder.init();
            end = new Date();
            Instance.Msg(`跳点生成完成,耗时${end.getTime() - start.getTime()}ms`);
        }

        //构建A*寻路
        this.astar = new PolyGraphAStar(this.mesh,this.links);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers,this.links);
        this.heightfixer=new FunnelHeightFixer(this.mesh,this.meshdetail,ADJUST_HEIGHT_DISTANCE);
    }
    /**
     * 输入起点终点，返回世界坐标路径点
     * @param {{x:number,y:number,z:number}} start
     * @param {{x:number,y:number,z:number}} end
     * @returns {{pos:{x:number,y:number,z:number},mode:number}[]}
     */
    findPath(start, end) {
        const polyPath=this.astar.findPath(start,end);

        if (!polyPath || polyPath.length === 0) return [];
        const funnelPath = this.funnel.build(polyPath, start, end);
        const ans=this.heightfixer.fixHeight(funnelPath,polyPath);
        //this.debugDrawPolyPath(polyPath, 1 / 2);
        this.debugDrawfunnelPath(funnelPath,1/2);
        this.debugDrawPath(ans,1/2);

        //无后台
        //多边形总数：1025跳点数：162
        //100次A*           68ms
        //100次funnelPath   74ms-68=6ms
        //100次fixHeight    82ms-74=8ms
        return ans;
    }
    /**
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     */
    debugDrawfunnelPath(path, duration = 10) {
        if (!path || path.length < 2) {
            Instance.Msg("No path to draw");
            return;
        }
        const color = {
            r: Math.floor(0),
            g: Math.floor(255),
            b: Math.floor(0),
        };
        const colorJ = {
            r: Math.floor(0),
            g: Math.floor(255),
            b: Math.floor(255),
        };

        const last = path[0].pos;
        Instance.DebugSphere({
            center: { x: last.x, y: last.y, z: last.z },
            radius: 3,
            color: { r: 255, g: 0, b: 0 },
            duration
        });
        for (let i = 1; i < path.length; i++) {
            const a = path[i-1].pos;
            const b = path[i].pos;

            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color:path[i].mode==2?colorJ:color,
                duration
            });

            Instance.DebugSphere({
                center: { x: b.x, y: b.y, z: b.z },
                radius: 3,
                color:path[i].mode==2?colorJ:color,
                duration
            });
        }
    }
    /**
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     */
    debugDrawPath(path, duration = 10) {
        if (!path || path.length < 2) {
            Instance.Msg("No path to draw");
            return;
        }
        const color = {
            r: Math.floor(255),
            g: Math.floor(0),
            b: Math.floor(0),
        };
        const colorJ = {
            r: Math.floor(255),
            g: Math.floor(255),
            b: Math.floor(0),
        };

        const last = path[0].pos;
        Instance.DebugSphere({
            center: { x: last.x, y: last.y, z: last.z },
            radius: 3,
            color: { r: 255, g: 0, b: 0 },
            duration
        });
        for (let i = 1; i < path.length; i++) {
            const a = path[i-1].pos;
            const b = path[i].pos;

            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color:path[i].mode==2?colorJ:color,
                duration
            });

            Instance.DebugSphere({
                center: { x: b.x, y: b.y, z: b.z },
                radius: 3,
                color:path[i].mode==2?colorJ:color,
                duration
            });
        }
    }
    /**
     * @param {{id:number,mode:number}[]} polyPath
     */
    debugDrawPolyPath(polyPath, duration = 10) {
        if (!polyPath || polyPath.length === 0) return;

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
            const poly = this.mesh.polys[pi.id];

            // poly 中心
            let cx = 0, cy = 0, cz = 0;
            for (const vi of poly) {
                const v = this.mesh.verts[vi];
                cx += v.x; cy += v.y; cz += v.z;
            }
            cx /= poly.length;
            cy /= poly.length;
            cz /= poly.length;

            const center = { x: cx, y: cy, z: cz };
            if(pi.mode==2)
            {
                Instance.DebugSphere({
                    center,
                    radius: 10,
                    color:colorJ,
                    duration
                });

                if (prev) {
                    Instance.DebugLine({
                        start: prev,
                        end: center,
                        color:colorJ,
                        duration
                    });
                }
            }
            else
            {
                Instance.DebugSphere({
                    center,
                    radius: 10,
                    color,
                    duration
                });

                if (prev) {
                    Instance.DebugLine({
                        start: prev,
                        end: center,
                        color,
                        duration
                    });
                }
                }
            prev = center;
        }
    }

    testinit() {
        const polys = this.mesh.polys;
        const verts = this.mesh.verts;

        this._polyAreas = [];
        this._polyPrefix = [];

        let total = 0;

        for (let i = 0; i < polys.length; i++) {
            const poly = polys[i];
            let area = 0;

            for (let j = 0; j < poly.length; j++) {
                const a = verts[poly[j]];
                const b = verts[poly[(j + 1) % poly.length]];
                area += (a.x * b.y - b.x * a.y);
            }

            area = Math.abs(area) * 0.5;
            this._polyAreas.push(area);

            total += area;
            this._polyPrefix.push(total);
        }

        this._totalPolyArea = total;
    }
    _randomPoint() {
        // 1. 按面积随机选 poly
        const r = Math.random() * this._totalPolyArea;
        let lo = 0, hi = this._polyPrefix.length - 1;

        while (lo < hi) {
            const mid = (lo + hi) >> 1;
            if (r <= this._polyPrefix[mid]) hi = mid;
            else lo = mid + 1;
        }

        const pi = lo;
        const poly = this.mesh.polys[pi];
        const verts = this.mesh.verts;

        // 2. 扇形三角化采样
        const v0 = verts[poly[0]];
        let total = 0;
        const tris = [];

        for (let i = 1; i < poly.length - 1; i++) {
            const v1 = verts[poly[i]];
            const v2 = verts[poly[i + 1]];

            const area = Math.abs(
                (v1.x - v0.x) * (v2.y - v0.y) -
                (v1.y - v0.y) * (v2.x - v0.x)
            ) * 0.5;

            tris.push({ v0, v1, v2, area });
            total += area;
        }

        let t = Math.random() * total;
        for (const tri of tris) {
            if (t <= tri.area) {
                return this._randomPointInTri(tri.v0, tri.v1, tri.v2);
            }
            t -= tri.area;
        }

        return null;
    }
    /**
     * @param {{ x: number; y: number; z: number; }} a
     * @param {{ x: number; y: number; z: number; }} b
     * @param {{ x: number; y: number; z: number; }} c
     */
    _randomPointInTri(a, b, c) {
        let r1 = Math.random();
        let r2 = Math.random();

        if (r1 + r2 > 1) {
            r1 = 1 - r1;
            r2 = 1 - r2;
        }

        return {
            x: a.x + r1 * (b.x - a.x) + r2 * (c.x - a.x),
            y: a.y + r1 * (b.y - a.y) + r2 * (c.y - a.y),
            z: a.z + r1 * (b.z - a.z) + r2 * (c.z - a.z)
        };
    }
    /**
     * NavMesh 压力测试
     * @param {number} count  随机寻路次数
     */
    randomTest(count = 1000) {
        for (let i = 0; i < count; i++) {
            const start = this._randomPoint();
            const end = this._randomPoint();

            if (!start || !end) {
                continue;
            }

            const path = this.findPath(start, end);
            //this.debugDrawPolyPath(path,30);
            if (path && path.length > 0) ;
        }
    }

}

Instance.ServerCommand("mp_warmup_offline_enabled 1");
Instance.ServerCommand("mp_warmup_pausetimer 1");
Instance.ServerCommand("mp_roundtime 60");
Instance.ServerCommand("mp_freezetime 1");
Instance.ServerCommand("mp_ignore_round_win_conditions 1");
Instance.ServerCommand("weapon_accuracy_nospread 1");
let pathfinder = new NavMesh();
Instance.OnScriptReload({
    before: () => {
    },
    after: () => {
        //let start = new Date();
        //Instance.Msg("导航初始化中");
        //pathfinder.init();
        //let end = new Date();
        //Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
    }
});
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
let start={x:-896,y:-783,z:117};
let end={x:351,y:2352,z:-110};
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
        //            end={x:pos.x,y:pos.y,z:pos.z};
        //            return;
        //        }
        //    }
        //})
        for(let i=0;i<1;i++)pathfinder.findPath(start,end);
    }
    Instance.SetNextThink(Instance.GetGameTime()+1/2);
});
Instance.SetNextThink(Instance.GetGameTime()+1/2);
Instance.OnBulletImpact((event)=>{
    end=event.position;
    //pathfinder.findPath(start,end);
    //pathfinder.findPath(start,end);
});
Instance.OnPlayerChat((event) => {
    const text = (event.text || "").trim().toLowerCase().split(' ')[0];
    Number((event.text || "").trim().toUpperCase().split(' ')[1]);
    if (text === "debug" || text === "!debug")
    {
        init();
        pathfinder.debug(60);
        //pathfinder.testinit();
        pd=true;
        //pathfinder.testinit();
        //pathfinder.randomTest(num);
    }
    if (text === "c" || text === "!c")
    {
        const p=event.player?.GetPlayerPawn();
        if(p)
        {
            const pos=p.GetAbsOrigin();
            start={x:pos.x,y:pos.y,z:pos.z};
            //Instance.Msg(`${Math.floor(pos.x)}  ${Math.floor(pos.y)}  ${Math.floor(pos.z)}`);
        }
    }
    if (text === "v" || text === "!v")
    {
        const p=event.player?.GetPlayerPawn();
        if(p)
        {
            const pos=p.GetAbsOrigin();
            end={x:pos.x,y:pos.y,z:pos.z};
            //const path = pathfinder.findPath(start,end);
            //Instance.Msg(`${Math.floor(end.x)}  ${Math.floor(end.y)}  ${Math.floor(end.z)}`);
            //pathfinder.debugDrawPath(path,30);
        }
    }
});
