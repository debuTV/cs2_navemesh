import { Instance } from 'cs_script/point_script';

const ASTAR_HEURISTIC_SCALE = 1.2;                         //A*推荐数值
const FUNNEL_DISTANCE = 25;                                //拉直的路径距离边缘多远(0-100，百分比，100%意味着只能走边的中点)
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
        Instance.Msg(this.links.size);
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

class StaticData
{
    constructor()
    {
        this.Data = ""+`{"mesh":{"verts":[{"x":-1845,"y":-1050,"z":128},{"x":-1820,"y":-1040,"z":128},{"x":-1820,"y":-980,"z":129},{"x":-1860,"y":-945,"z":120},{"x":-2035,"y":-945,"z":120},{"x":-2120,"y":-1040,"z":128},{"x":-2040,"y":-860,"z":116},{"x":-2020,"y":-855,"z":115},{"x":-2020,"y":-680,"z":127},{"x":-2210,"y":-680,"z":128},{"x":-2220,"y":-825,"z":120},{"x":-2210,"y":-1040,"z":128},{"x":-2160,"y":-1040,"z":128},{"x":-2155,"y":-1050,"z":128},{"x":-2125,"y":-1050,"z":128},{"x":-2210,"y":-640,"z":128},{"x":-1780,`
+`"y":-650,"z":129},{"x":-1800,"y":-635,"z":128},{"x":-1865,"y":-850,"z":115},{"x":-1845,"y":-870,"z":116},{"x":-1785,"y":-815,"z":116},{"x":-1805,"y":-440,"z":87},{"x":-2065,"y":-505,"z":104},{"x":-2065,"y":-435,"z":86},{"x":-1800,"y":-125,"z":9},{"x":-2065,"y":-120,"z":7},{"x":-1990,"y":1055,"z":35},{"x":-1970,"y":1055,"z":34},{"x":-1975,"y":1195,"z":32},{"x":-2050,"y":1270,"z":33},{"x":-2170,"y":1280,"z":40},{"x":-2170,"y":1265,"z":40},{"x":-2195,"y":1250,"z":38},{"x":-2195,"y":1055,"z":39},{"x`
+`":-2170,"y":1040,"z":40},{"x":-2000,"y":1040,"z":35},{"x":-2140,"y":2095,"z":3},{"x":-2130,"y":2080,"z":2},{"x":-2035,"y":2175,"z":-1},{"x":-1970,"y":2185,"z":-2},{"x":-1850,"y":2310,"z":0},{"x":-1870,"y":2325,"z":0},{"x":-1865,"y":2460,"z":22},{"x":-1975,"y":2425,"z":15},{"x":-2130,"y":2305,"z":3},{"x":-1970,"y":2460,"z":22},{"x":-2160,"y":2420,"z":25},{"x":-2160,"y":2350,"z":11},{"x":-2130,"y":2340,"z":6},{"x":-2140,"y":2295,"z":3},{"x":-2185,"y":2295,"z":3},{"x":-2185,"y":2095,"z":4},{"x":-20`
+`40,"y":1810,"z":32},{"x":-2030,"y":1795,"z":32},{"x":-1935,"y":1795,"z":32},{"x":-1925,"y":1810,"z":32},{"x":-1875,"y":1805,"z":0},{"x":-1875,"y":1865,"z":0},{"x":-1855,"y":1880,"z":0},{"x":-1975,"y":2005,"z":-2},{"x":-2160,"y":2005,"z":4},{"x":-2160,"y":1805,"z":3},{"x":-2160,"y":2040,"z":4},{"x":-2130,"y":2050,"z":3},{"x":-1780,"y":1880,"z":0},{"x":-1770,"y":2310,"z":1},{"x":-1770,"y":1865,"z":0},{"x":-2025,"y":2460,"z":65},{"x":-2020,"y":2500,"z":67},{"x":-2095,"y":2500,"z":67},{"x":-2090,"y"`
+`:2460,"z":67},{"x":-2115,"y":2455,"z":62},{"x":-2115,"y":2440,"z":63},{"x":-2075,"y":2430,"z":65},{"x":-1980,"y":2435,"z":63},{"x":-1980,"y":2460,"z":65},{"x":-1975,"y":2470,"z":24},{"x":-1860,"y":2470,"z":24},{"x":-1750,"y":2465,"z":26},{"x":-1750,"y":2680,"z":32},{"x":-1795,"y":2655,"z":32},{"x":-1830,"y":2620,"z":33},{"x":-1795,"y":2675,"z":32},{"x":-1820,"y":2645,"z":33},{"x":-1850,"y":2620,"z":34},{"x":-1865,"y":2645,"z":33},{"x":-2110,"y":2645,"z":37},{"x":-2020,"y":2510,"z":31},{"x":-2110`
+`,"y":2465,"z":33},{"x":-2095,"y":2510,"z":33},{"x":-2010,"y":2465,"z":28},{"x":-2115,"y":2695,"z":38},{"x":-1850,"y":2660,"z":34},{"x":-1935,"y":2750,"z":31},{"x":-1970,"y":2750,"z":31},{"x":-2035,"y":2820,"z":32},{"x":-2095,"y":2820,"z":33},{"x":-2095,"y":2705,"z":35},{"x":-2030,"y":2900,"z":33},{"x":-2020,"y":2885,"z":32},{"x":-1900,"y":2890,"z":32},{"x":-1905,"y":2925,"z":36},{"x":-1895,"y":2935,"z":36},{"x":-1855,"y":2925,"z":36},{"x":-1870,"y":2895,"z":35},{"x":-1740,"y":2900,"z":36},{"x":-`
+`1800,"y":3000,"z":32},{"x":-1855,"y":2965,"z":33},{"x":-1875,"y":3000,"z":33},{"x":-1890,"y":2955,"z":38},{"x":-1905,"y":2970,"z":36},{"x":-1900,"y":3010,"z":34},{"x":-2005,"y":3010,"z":32},{"x":-2010,"y":3055,"z":33},{"x":-2105,"y":3125,"z":35},{"x":-2100,"y":2895,"z":33},{"x":-1975,"y":3065,"z":32},{"x":-1975,"y":3125,"z":33},{"x":-2090,"y":2890,"z":89},{"x":-2090,"y":2830,"z":89},{"x":-2030,"y":2830,"z":89},{"x":-2030,"y":2890,"z":89},{"x":-1780,"y":-95,"z":8},{"x":-2035,"y":-110,"z":8},{"x":`
+`-1875,"y":0,"z":-2},{"x":-1875,"y":15,"z":-3},{"x":-2025,"y":15,"z":8},{"x":-2035,"y":10,"z":9},{"x":-2055,"y":405,"z":8},{"x":-2050,"y":210,"z":9},{"x":-1730,"y":210,"z":0},{"x":-1715,"y":425,"z":1},{"x":-1790,"y":500,"z":18},{"x":-2040,"y":460,"z":9},{"x":-1730,"y":-105,"z":20},{"x":-1710,"y":-105,"z":8},{"x":-2045,"y":505,"z":12},{"x":-2050,"y":465,"z":25},{"x":-2050,"y":460,"z":22},{"x":-2055,"y":540,"z":61},{"x":-2055,"y":515,"z":62},{"x":-1790,"y":515,"z":60},{"x":-1790,"y":540,"z":59},{"x`
+`":-2035,"y":100,"z":8},{"x":-2025,"y":120,"z":8},{"x":-2050,"y":160,"z":9},{"x":-1790,"y":550,"z":32},{"x":-1780,"y":530,"z":32},{"x":-1545,"y":530,"z":33},{"x":-1530,"y":550,"z":32},{"x":-1320,"y":550,"z":35},{"x":-1320,"y":600,"z":38},{"x":-1405,"y":600,"z":34},{"x":-1415,"y":610,"z":35},{"x":-1420,"y":690,"z":36},{"x":-1450,"y":700,"z":33},{"x":-1605,"y":695,"z":32},{"x":-1615,"y":735,"z":32},{"x":-1720,"y":695,"z":32},{"x":-1710,"y":735,"z":31},{"x":-1795,"y":690,"z":32},{"x":-1825,"y":705,"`
+`z":35},{"x":-1825,"y":600,"z":32},{"x":-1810,"y":555,"z":32},{"x":-1905,"y":690,"z":34},{"x":-1905,"y":610,"z":33},{"x":-1875,"y":580,"z":32},{"x":-1915,"y":600,"z":32},{"x":-2050,"y":600,"z":37},{"x":-2050,"y":550,"z":35},{"x":-2040,"y":545,"z":35},{"x":-1870,"y":550,"z":32},{"x":-1950,"y":1055,"z":34},{"x":-1940,"y":1040,"z":34},{"x":-1845,"y":1035,"z":37},{"x":-1845,"y":1140,"z":32},{"x":-1830,"y":1140,"z":32},{"x":-1830,"y":1220,"z":32},{"x":-1815,"y":1235,"z":32},{"x":-1845,"y":1255,"z":32}`
+`,{"x":-1830,"y":1280,"z":33},{"x":-2035,"y":1285,"z":32},{"x":-2030,"y":1435,"z":32},{"x":-1805,"y":1325,"z":37},{"x":-1775,"y":1310,"z":37},{"x":-1730,"y":1315,"z":38},{"x":-1925,"y":1395,"z":32},{"x":-1715,"y":1295,"z":37},{"x":-1675,"y":1300,"z":36},{"x":-1675,"y":1400,"z":42},{"x":-1935,"y":1435,"z":32},{"x":-1835,"y":2665,"z":33},{"x":-1825,"y":2690,"z":33},{"x":-1805,"y":2690,"z":32},{"x":-1785,"y":2765,"z":33},{"x":-1795,"y":2780,"z":32},{"x":-1750,"y":2775,"z":34},{"x":-1775,"y":2810,"z"`
+`:32},{"x":-1750,"y":2870,"z":34},{"x":-1890,"y":2870,"z":31},{"x":-1750,"y":2795,"z":33},{"x":-2020,"y":2830,"z":32},{"x":-1855,"y":-865,"z":150},{"x":-1870,"y":-860,"z":148},{"x":-2030,"y":-865,"z":158},{"x":-2030,"y":-930,"z":159},{"x":-1855,"y":-935,"z":151},{"x":-2000,"y":3055,"z":71},{"x":-2000,"y":3020,"z":70},{"x":-1930,"y":3020,"z":70},{"x":-1930,"y":3050,"z":71},{"x":-1965,"y":3055,"z":70},{"x":-1920,"y":3050,"z":106},{"x":-1920,"y":3030,"z":106},{"x":-1900,"y":3030,"z":106},{"x":-1900,`
+`"y":3050,"z":106},{"x":-1795,"y":-1050,"z":129},{"x":-1600,"y":-1070,"z":128},{"x":-1600,"y":-815,"z":116},{"x":-1795,"y":-1070,"z":129},{"x":-1770,"y":-650,"z":129},{"x":-1845,"y":-935,"z":120},{"x":-1825,"y":550,"z":106},{"x":-1830,"y":575,"z":106},{"x":-1855,"y":565,"z":106},{"x":-1850,"y":545,"z":106},{"x":-1835,"y":545,"z":106},{"x":-1715,"y":2430,"z":65},{"x":-1720,"y":2870,"z":62},{"x":-1740,"y":2870,"z":62},{"x":-1745,"y":2835,"z":65},{"x":-1740,"y":2465,"z":65},{"x":-1750,"y":2455,"z":6`
+`3},{"x":-1855,"y":2460,"z":64},{"x":-1855,"y":2440,"z":63},{"x":-1835,"y":2655,"z":78},{"x":-1855,"y":2640,"z":78},{"x":-1830,"y":2630,"z":78},{"x":-1850,"y":2425,"z":102},{"x":-1850,"y":2330,"z":103},{"x":-1760,"y":2330,"z":102},{"x":-1765,"y":2425,"z":102},{"x":-1840,"y":975,"z":40},{"x":-1725,"y":980,"z":34},{"x":-1730,"y":1000,"z":34},{"x":-1775,"y":1010,"z":38},{"x":-1775,"y":1050,"z":36},{"x":-1725,"y":1060,"z":32},{"x":-1715,"y":1045,"z":31},{"x":-1705,"y":1050,"z":31},{"x":-1665,"y":1090`
+`,"z":31},{"x":-1660,"y":1270,"z":35},{"x":-1555,"y":1250,"z":36},{"x":-1570,"y":1270,"z":38},{"x":-1675,"y":1280,"z":35},{"x":-1725,"y":1250,"z":33},{"x":-1765,"y":1250,"z":33},{"x":-1780,"y":1270,"z":34},{"x":-1795,"y":1235,"z":32},{"x":-1775,"y":1305,"z":70},{"x":-1790,"y":1315,"z":70},{"x":-1805,"y":1310,"z":71},{"x":-1835,"y":1260,"z":70},{"x":-1810,"y":1240,"z":70},{"x":-1795,"y":1250,"z":70},{"x":-1775,"y":1285,"z":70},{"x":-1810,"y":2655,"z":77},{"x":-1805,"y":2680,"z":77},{"x":-1825,"y":`
+`2680,"z":77},{"x":-1825,"y":2665,"z":77},{"x":-1420,"y":-150,"z":157},{"x":-1420,"y":-125,"z":157},{"x":-1785,"y":-130,"z":158},{"x":-1610,"y":-150,"z":157},{"x":-1790,"y":-630,"z":157},{"x":-1765,"y":-640,"z":158},{"x":-1765,"y":-315,"z":157},{"x":-1685,"y":-315,"z":164},{"x":-1685,"y":-245,"z":164},{"x":-1750,"y":-245,"z":164},{"x":-1760,"y":-260,"z":162},{"x":-1765,"y":-230,"z":164},{"x":-1695,"y":-225,"z":164},{"x":-1690,"y":-170,"z":164},{"x":-1695,"y":-160,"z":164},{"x":-1680,"y":-175,"z":`
+`164},{"x":-1675,"y":-230,"z":164},{"x":-1610,"y":-225,"z":164},{"x":-1665,"y":-165,"z":164},{"x":-1660,"y":-155,"z":164},{"x":-1690,"y":-150,"z":158},{"x":-1690,"y":-115,"z":8},{"x":-1495,"y":-115,"z":8},{"x":-1490,"y":-105,"z":8},{"x":-1465,"y":-115,"z":26},{"x":-1465,"y":-105,"z":25},{"x":-1480,"y":-110,"z":30},{"x":-1470,"y":-90,"z":8},{"x":-1485,"y":-65,"z":8},{"x":-1460,"y":-45,"z":8},{"x":-1470,"y":-15,"z":8},{"x":-1460,"y":30,"z":8},{"x":-1535,"y":500,"z":25},{"x":-1440,"y":55,"z":8},{"x"`
+`:-1420,"y":55,"z":8},{"x":-1380,"y":95,"z":3},{"x":-1365,"y":95,"z":8},{"x":-1325,"y":45,"z":8},{"x":-1275,"y":45,"z":8},{"x":-1255,"y":65,"z":8},{"x":-1260,"y":185,"z":10},{"x":-1320,"y":505,"z":8},{"x":-1760,"y":2795,"z":77},{"x":-1785,"y":2785,"z":77},{"x":-1760,"y":2775,"z":77},{"x":-1035,"y":-1070,"z":128},{"x":-1010,"y":-1055,"z":128},{"x":-1010,"y":-1000,"z":128},{"x":-1030,"y":-985,"z":128},{"x":-1030,"y":-970,"z":128},{"x":-1010,"y":-960,"z":128},{"x":-930,"y":-920,"z":120},{"x":-930,"y`
+`":-715,"z":121},{"x":-970,"y":-705,"z":128},{"x":-995,"y":-980,"z":128},{"x":-970,"y":-620,"z":128},{"x":-1050,"y":-620,"z":127},{"x":-1055,"y":-425,"z":129},{"x":-1040,"y":-410,"z":129},{"x":-1080,"y":-370,"z":129},{"x":-1080,"y":-330,"z":128},{"x":-1150,"y":-265,"z":128},{"x":-1185,"y":-265,"z":128},{"x":-1190,"y":-255,"z":128},{"x":-1255,"y":-270,"z":128},{"x":-1250,"y":-255,"z":128},{"x":-1260,"y":-255,"z":155},{"x":-1295,"y":-270,"z":136},{"x":-1295,"y":-255,"z":154},{"x":-1345,"y":-265,"z"`
+`:130},{"x":-1415,"y":-195,"z":129},{"x":-1415,"y":-160,"z":130},{"x":-1525,"y":-190,"z":130},{"x":-1540,"y":-220,"z":129},{"x":-1565,"y":-160,"z":129},{"x":-1560,"y":-175,"z":130},{"x":-1535,"y":-175,"z":130},{"x":-1555,"y":-220,"z":130},{"x":-1570,"y":-190,"z":130},{"x":-1590,"y":-195,"z":130},{"x":-1600,"y":-230,"z":129},{"x":-1605,"y":-180,"z":130},{"x":-1675,"y":-240,"z":128},{"x":-1680,"y":-325,"z":128},{"x":-1755,"y":-320,"z":129},{"x":-1755,"y":-640,"z":128},{"x":-1725,"y":1805,"z":3},{"x`
+`":-1715,"y":1790,"z":3},{"x":-1695,"y":1805,"z":2},{"x":-1685,"y":1860,"z":0},{"x":-1660,"y":1805,"z":0},{"x":-1500,"y":1920,"z":0},{"x":-1490,"y":1885,"z":0},{"x":-1410,"y":1875,"z":4},{"x":-1525,"y":1945,"z":0},{"x":-1385,"y":1895,"z":6},{"x":-1385,"y":2085,"z":3},{"x":-1545,"y":1945,"z":-1},{"x":-1355,"y":2115,"z":2},{"x":-1295,"y":2200,"z":1},{"x":-1295,"y":2235,"z":1},{"x":-1355,"y":2095,"z":3},{"x":-1325,"y":2130,"z":3},{"x":-1280,"y":2210,"z":3},{"x":-1330,"y":2235,"z":1},{"x":-1350,"y":2`
+`205,"z":1},{"x":-1375,"y":2215,"z":0},{"x":-1340,"y":2280,"z":1},{"x":-1355,"y":2320,"z":1},{"x":-1380,"y":2330,"z":0},{"x":-1370,"y":2360,"z":0},{"x":-1380,"y":2390,"z":2},{"x":-1355,"y":2400,"z":2},{"x":-1355,"y":2575,"z":4},{"x":-1480,"y":2580,"z":2},{"x":-1490,"y":2605,"z":3},{"x":-1555,"y":2605,"z":4},{"x":-1580,"y":2510,"z":2},{"x":-1580,"y":2580,"z":3},{"x":-1590,"y":2500,"z":2},{"x":-1670,"y":2500,"z":7},{"x":-1680,"y":2510,"z":8},{"x":-1705,"y":2430,"z":10},{"x":-1675,"y":2585,"z":6},{"`
+`x":-1705,"y":2590,"z":7},{"x":-1745,"y":2420,"z":12},{"x":-1740,"y":2325,"z":1},{"x":-1765,"y":1805,"z":3},{"x":-1710,"y":970,"z":33},{"x":-1615,"y":970,"z":32},{"x":-1605,"y":980,"z":33},{"x":-1545,"y":975,"z":35},{"x":-1545,"y":995,"z":35},{"x":-1570,"y":1010,"z":35},{"x":-1565,"y":1050,"z":35},{"x":-1715,"y":1010,"z":32},{"x":-1560,"y":1720,"z":1},{"x":-1625,"y":1800,"z":0},{"x":-1605,"y":1760,"z":2},{"x":-1655,"y":1795,"z":0},{"x":-1710,"y":1615,"z":3},{"x":-1650,"y":1715,"z":2},{"x":-1615,"`
+`y":1615,"z":2},{"x":-1595,"y":1655,"z":2},{"x":-1650,"y":1700,"z":2},{"x":-1710,"y":2870,"z":19},{"x":-1665,"y":2595,"z":5},{"x":-1590,"y":2595,"z":4},{"x":-1630,"y":2735,"z":9},{"x":-1490,"y":2660,"z":5},{"x":-1420,"y":2670,"z":12},{"x":-1425,"y":2725,"z":14},{"x":-1405,"y":2745,"z":16},{"x":-1500,"y":2870,"z":5},{"x":-1535,"y":2780,"z":7},{"x":-1355,"y":2745,"z":17},{"x":-1355,"y":2780,"z":15},{"x":-1560,"y":2710,"z":6},{"x":-1600,"y":2805,"z":11},{"x":-1525,"y":1055,"z":36},{"x":-1525,"y":125`
+`0,"z":37},{"x":-1630,"y":1850,"z":34},{"x":-1635,"y":1880,"z":41},{"x":-1675,"y":1850,"z":37},{"x":-1650,"y":1805,"z":36},{"x":-1610,"y":1820,"z":39},{"x":-1605,"y":1830,"z":50},{"x":-1565,"y":1715,"z":60},{"x":-1600,"y":1750,"z":60},{"x":-1640,"y":1705,"z":60},{"x":-1605,"y":1670,"z":60},{"x":-1565,"y":1700,"z":60},{"x":-1560,"y":1860,"z":52},{"x":-1580,"y":1900,"z":52},{"x":-1630,"y":1885,"z":45},{"x":-1630,"y":1875,"z":53},{"x":-1585,"y":1850,"z":54},{"x":-1570,"y":1845,"z":45},{"x":-1610,"y"`
+`:1835,"z":53},{"x":-1595,"y":1830,"z":45},{"x":-1545,"y":2775,"z":64},{"x":-1585,"y":2795,"z":64},{"x":-1600,"y":2795,"z":64},{"x":-1620,"y":2745,"z":64},{"x":-1580,"y":2720,"z":64},{"x":-1560,"y":2725,"z":64},{"x":-1575,"y":-185,"z":175},{"x":-1575,"y":-160,"z":174},{"x":-1590,"y":-160,"z":174},{"x":-1595,"y":-175,"z":174},{"x":-1500,"y":1885,"z":25},{"x":-1525,"y":1935,"z":26},{"x":-1545,"y":1935,"z":28},{"x":-1580,"y":1910,"z":44},{"x":-1550,"y":1855,"z":41},{"x":-1540,"y":-210,"z":175},{"x":`
+`-1540,"y":-185,"z":175},{"x":-1555,"y":-185,"z":175},{"x":-1560,"y":-200,"z":175},{"x":-1530,"y":540,"z":61},{"x":-1535,"y":515,"z":62},{"x":-1320,"y":515,"z":58},{"x":-1320,"y":540,"z":57},{"x":-1505,"y":1040,"z":37},{"x":-1335,"y":1040,"z":38},{"x":-1325,"y":1055,"z":37},{"x":-1285,"y":1055,"z":32},{"x":-1275,"y":1040,"z":32},{"x":-1175,"y":1045,"z":-8},{"x":-1330,"y":1175,"z":33},{"x":-1375,"y":1205,"z":34},{"x":-1185,"y":1150,"z":-16},{"x":-1245,"y":1145,"z":24},{"x":-1245,"y":1155,"z":32},{`
+`"x":-1190,"y":1155,"z":32},{"x":-1160,"y":1180,"z":32},{"x":-1205,"y":1190,"z":32},{"x":-1280,"y":1195,"z":32},{"x":-1155,"y":1275,"z":32},{"x":-1190,"y":1275,"z":32},{"x":-1195,"y":1200,"z":32},{"x":-1290,"y":1170,"z":32},{"x":-1335,"y":1200,"z":33},{"x":-1375,"y":1270,"z":36},{"x":-1505,"y":1270,"z":38},{"x":-1480,"y":2655,"z":60},{"x":-1480,"y":2625,"z":60},{"x":-1465,"y":2625,"z":60},{"x":-1460,"y":2655,"z":60},{"x":-1420,"y":2660,"z":60},{"x":-1400,"y":15,"z":64},{"x":-1405,"y":30,"z":61},{`
+`"x":-1450,"y":30,"z":69},{"x":-1460,"y":-35,"z":69},{"x":-1440,"y":-45,"z":64},{"x":-1460,"y":-50,"z":68},{"x":-1460,"y":-80,"z":68},{"x":-1455,"y":-105,"z":69},{"x":-1420,"y":-30,"z":65},{"x":-1420,"y":-105,"z":86},{"x":-1455,"y":2645,"z":107},{"x":-1455,"y":2595,"z":107},{"x":-1355,"y":2595,"z":119},{"x":-1355,"y":2645,"z":119},{"x":-1405,"y":2650,"z":122},{"x":-1280,"y":2645,"z":119},{"x":-1280,"y":2600,"z":102},{"x":-1230,"y":2600,"z":101},{"x":-1235,"y":2710,"z":114},{"x":-1305,"y":2665,"z"`
+`:128},{"x":-1330,"y":2665,"z":128},{"x":-1330,"y":2695,"z":128},{"x":-1355,"y":2710,"z":124},{"x":-1355,"y":2725,"z":124},{"x":-1400,"y":2720,"z":125},{"x":-1355,"y":55,"z":20},{"x":-1370,"y":75,"z":20},{"x":-1405,"y":45,"z":20},{"x":-1395,"y":25,"z":20},{"x":-1370,"y":40,"z":20},{"x":-1330,"y":1230,"z":70},{"x":-1335,"y":1270,"z":71},{"x":-1365,"y":1270,"z":71},{"x":-1365,"y":1215,"z":71},{"x":-1330,"y":1210,"z":70},{"x":-1325,"y":1185,"z":69},{"x":-1290,"y":1190,"z":70},{"x":-1285,"y":1250,"z"`
+`:69},{"x":-1320,"y":1250,"z":70},{"x":-1310,"y":2285,"z":3},{"x":-1260,"y":2190,"z":3},{"x":-1295,"y":2125,"z":5},{"x":-1280,"y":2115,"z":7},{"x":-1280,"y":2095,"z":9},{"x":-1225,"y":2095,"z":11},{"x":-1215,"y":2065,"z":12},{"x":-1180,"y":2065,"z":8},{"x":-1170,"y":2130,"z":4},{"x":-1120,"y":2130,"z":-1},{"x":-1105,"y":2165,"z":-6},{"x":-1115,"y":2175,"z":-4},{"x":-1115,"y":2330,"z":-1},{"x":-1280,"y":2300,"z":6},{"x":-1280,"y":2330,"z":9},{"x":-1280,"y":2590,"z":68},{"x":-1070,"y":2165,"z":-16}`
+`,{"x":-1065,"y":2155,"z":-17},{"x":-960,"y":2255,"z":-64},{"x":-955,"y":2370,"z":-52},{"x":-910,"y":2415,"z":-60},{"x":-915,"y":2425,"z":-43},{"x":-900,"y":2420,"z":-55},{"x":-875,"y":2420,"z":-56},{"x":-865,"y":2420,"z":-66},{"x":-830,"y":2420,"z":-67},{"x":-820,"y":2460,"z":-66},{"x":-820,"y":2475,"z":-66},{"x":-825,"y":2485,"z":-64},{"x":-840,"y":2495,"z":-57},{"x":-840,"y":2480,"z":-54},{"x":-850,"y":2480,"z":-41},{"x":-815,"y":2485,"z":-77},{"x":-815,"y":2525,"z":-68},{"x":-850,"y":2495,"z"`
+`:-42},{"x":-820,"y":2555,"z":-65},{"x":-840,"y":2555,"z":-64},{"x":-840,"y":2565,"z":-77},{"x":-850,"y":2580,"z":-74},{"x":-820,"y":2565,"z":-82},{"x":-815,"y":2580,"z":-82},{"x":-865,"y":2560,"z":-64},{"x":-855,"y":2525,"z":-43},{"x":-870,"y":2530,"z":-41},{"x":-885,"y":2550,"z":-48},{"x":-905,"y":2540,"z":-43},{"x":-905,"y":2470,"z":-41},{"x":-870,"y":2550,"z":-48},{"x":-915,"y":2470,"z":-25},{"x":-920,"y":2535,"z":-25},{"x":-920,"y":2545,"z":-39},{"x":-1085,"y":2570,"z":32},{"x":-885,"y":2560`
+`,"z":-58},{"x":-1110,"y":2585,"z":46},{"x":-1230,"y":2590,"z":65},{"x":-1110,"y":2710,"z":87},{"x":-1225,"y":2710,"z":88},{"x":-1220,"y":2600,"z":65},{"x":-1210,"y":1270,"z":104},{"x":-1225,"y":1275,"z":104},{"x":-1265,"y":1270,"z":104},{"x":-1275,"y":1240,"z":104},{"x":-1270,"y":1215,"z":104},{"x":-1215,"y":1210,"z":104},{"x":-1160,"y":1160,"z":-40},{"x":-1115,"y":1065,"z":-32},{"x":-1060,"y":1120,"z":-56},{"x":-1035,"y":1190,"z":-80},{"x":-1040,"y":1320,"z":-111},{"x":-1135,"y":1195,"z":-72},{`
+`"x":-1010,"y":1330,"z":-111},{"x":-1010,"y":1405,"z":-113},{"x":-1030,"y":1405,"z":-113},{"x":-1065,"y":1445,"z":-112},{"x":-1200,"y":1450,"z":-110},{"x":-1200,"y":1340,"z":-107},{"x":-1130,"y":1320,"z":-111},{"x":-1065,"y":1455,"z":-76},{"x":-1060,"y":1525,"z":-76},{"x":-1125,"y":1525,"z":-76},{"x":-1130,"y":1495,"z":-76},{"x":-1140,"y":1525,"z":-77},{"x":-1185,"y":1525,"z":-77},{"x":-1200,"y":1515,"z":-77},{"x":-1200,"y":1465,"z":-77},{"x":-1160,"y":2125,"z":39},{"x":-1170,"y":2065,"z":40},{"x`
+`":-1135,"y":2065,"z":39},{"x":-1135,"y":2125,"z":39},{"x":-970,"y":-410,"z":129},{"x":-970,"y":-285,"z":130},{"x":-995,"y":-280,"z":129},{"x":-995,"y":-235,"z":129},{"x":-970,"y":-170,"z":150},{"x":-990,"y":-170,"z":150},{"x":-985,"y":-180,"z":150},{"x":-975,"y":-195,"z":145},{"x":-970,"y":-230,"z":129},{"x":-980,"y":-210,"z":129},{"x":-1005,"y":-205,"z":128},{"x":-1005,"y":-185,"z":135},{"x":-1010,"y":-160,"z":135},{"x":-995,"y":-185,"z":145},{"x":-995,"y":-200,"z":145},{"x":-1090,"y":-170,"z":`
+`130},{"x":-1090,"y":-210,"z":129},{"x":-1140,"y":-215,"z":128},{"x":-1135,"y":-255,"z":128},{"x":-1125,"y":2120,"z":65},{"x":-1125,"y":2065,"z":64},{"x":-1060,"y":2065,"z":64},{"x":-1060,"y":2115,"z":64},{"x":-1070,"y":2120,"z":65},{"x":-1105,"y":2155,"z":43},{"x":-1105,"y":2130,"z":43},{"x":-1075,"y":2130,"z":43},{"x":-1075,"y":2155,"z":43},{"x":-1060,"y":2590,"z":138},{"x":-1060,"y":2710,"z":139},{"x":-1095,"y":2710,"z":139},{"x":-1095,"y":2595,"z":138},{"x":-1090,"y":2585,"z":139},{"x":-905,"`
+`y":1325,"z":-108},{"x":-900,"y":1335,"z":-109},{"x":-865,"y":1335,"z":-110},{"x":-865,"y":1390,"z":-112},{"x":-855,"y":1400,"z":-112},{"x":-780,"y":1400,"z":-112},{"x":-780,"y":1525,"z":-109},{"x":-855,"y":1515,"z":-110},{"x":-850,"y":1525,"z":-109},{"x":-1050,"y":1525,"z":-110},{"x":-1050,"y":1475,"z":-112},{"x":-875,"y":2065,"z":-84},{"x":-830,"y":2065,"z":-99},{"x":-825,"y":2405,"z":-82},{"x":-825,"y":2415,"z":-81},{"x":-1065,"y":2135,"z":-13},{"x":-1040,"y":2125,"z":-16},{"x":-1040,"y":2095,`
+`"z":-12},{"x":-885,"y":2100,"z":-81},{"x":-975,"y":-420,"z":166},{"x":-990,"y":-415,"z":166},{"x":-1040,"y":-420,"z":165},{"x":-1045,"y":-595,"z":168},{"x":-1040,"y":-615,"z":166},{"x":-975,"y":-610,"z":168},{"x":-970,"y":-470,"z":169},{"x":-985,"y":-1055,"z":128},{"x":-975,"y":-1070,"z":128},{"x":-810,"y":-1070,"z":128},{"x":-785,"y":-1055,"z":128},{"x":-790,"y":-820,"z":116},{"x":-895,"y":-715,"z":121},{"x":-745,"y":-650,"z":130},{"x":-745,"y":-1070,"z":128},{"x":-590,"y":-1070,"z":128},{"x":-`
+`580,"y":-1055,"z":128},{"x":-880,"y":-650,"z":130},{"x":-880,"y":-705,"z":128},{"x":-835,"y":2625,"z":-70},{"x":-835,"y":2605,"z":-76},{"x":-815,"y":2605,"z":-79},{"x":-815,"y":2625,"z":-74},{"x":-755,"y":2065,"z":-112},{"x":-740,"y":2100,"z":-114},{"x":-710,"y":2100,"z":-116},{"x":-715,"y":2300,"z":-114},{"x":-515,"y":2140,"z":-124},{"x":-515,"y":2455,"z":-89},{"x":-590,"y":2460,"z":-88},{"x":-585,"y":2095,"z":-119},{"x":-575,"y":2080,"z":-119},{"x":-605,"y":2475,"z":-86},{"x":-595,"y":2545,"z"`
+`:-84},{"x":-605,"y":2570,"z":-84},{"x":-815,"y":2425,"z":-83},{"x":-810,"y":2555,"z":-83},{"x":-785,"y":2625,"z":-74},{"x":-790,"y":2605,"z":-79},{"x":-760,"y":2605,"z":-80},{"x":-760,"y":2625,"z":-73},{"x":-545,"y":1370,"z":-113},{"x":-505,"y":1380,"z":-112},{"x":-505,"y":1475,"z":-112},{"x":-545,"y":1485,"z":-112},{"x":-545,"y":1525,"z":-111},{"x":-770,"y":1390,"z":-112},{"x":-770,"y":1325,"z":-112},{"x":-545,"y":1325,"z":-110},{"x":-430,"y":-1065,"z":128},{"x":-430,"y":-1030,"z":128},{"x":-55`
+`5,"y":-1025,"z":128},{"x":-550,"y":-1015,"z":127},{"x":-510,"y":-650,"z":123},{"x":-505,"y":-1015,"z":121},{"x":-410,"y":-1015,"z":104},{"x":-640,"y":500,"z":9},{"x":-655,"y":245,"z":8},{"x":-650,"y":205,"z":8},{"x":-520,"y":205,"z":6},{"x":-520,"y":385,"z":3},{"x":-625,"y":510,"z":8},{"x":-735,"y":500,"z":10},{"x":-725,"y":470,"z":24},{"x":-745,"y":455,"z":24},{"x":-745,"y":440,"z":24},{"x":-710,"y":425,"z":24},{"x":-720,"y":360,"z":9},{"x":-700,"y":265,"z":8},{"x":-685,"y":245,"z":8},{"x":-660`
+`,"y":205,"z":56},{"x":-665,"y":240,"z":56},{"x":-680,"y":235,"z":57},{"x":-680,"y":220,"z":57},{"x":-495,"y":195,"z":-1},{"x":-495,"y":-45,"z":1},{"x":-435,"y":-45,"z":-1},{"x":-425,"y":-60,"z":-1},{"x":-395,"y":-55,"z":-1},{"x":-395,"y":230,"z":0},{"x":-320,"y":300,"z":0},{"x":-260,"y":305,"z":-1},{"x":-250,"y":295,"z":6},{"x":-255,"y":265,"z":0},{"x":-245,"y":265,"z":0},{"x":-230,"y":270,"z":6},{"x":-220,"y":305,"z":0},{"x":-190,"y":305,"z":2},{"x":-195,"y":540,"z":-1},{"x":-210,"y":550,"z":-1`
+`},{"x":-235,"y":540,"z":-2},{"x":-250,"y":555,"z":-1},{"x":-250,"y":570,"z":0},{"x":-225,"y":585,"z":0},{"x":-230,"y":725,"z":1},{"x":-245,"y":740,"z":0},{"x":-260,"y":750,"z":0},{"x":-265,"y":775,"z":-1},{"x":-265,"y":885,"z":-24},{"x":-495,"y":720,"z":1},{"x":-495,"y":880,"z":-25},{"x":-525,"y":710,"z":8},{"x":-525,"y":645,"z":8},{"x":-535,"y":635,"z":8},{"x":-625,"y":640,"z":8},{"x":-635,"y":625,"z":8},{"x":-585,"y":1800,"z":-118},{"x":-570,"y":1780,"z":-118},{"x":-545,"y":1805,"z":-120},{"x"`
+`:-330,"y":1805,"z":-122},{"x":-330,"y":1985,"z":-127},{"x":-310,"y":2000,"z":-127},{"x":-370,"y":2065,"z":-129},{"x":-570,"y":1930,"z":-119},{"x":-575,"y":2065,"z":-118},{"x":-610,"y":1920,"z":-118},{"x":-610,"y":1800,"z":-118},{"x":-520,"y":2545,"z":8},{"x":-545,"y":2550,"z":8},{"x":-585,"y":2550,"z":-22},{"x":-590,"y":2480,"z":-17},{"x":-525,"y":2475,"z":9},{"x":-515,"y":2530,"z":6},{"x":-330,"y":2625,"z":-84},{"x":-520,"y":2630,"z":-73},{"x":-590,"y":2630,"z":-75},{"x":-590,"y":2590,"z":-84},`
+`{"x":-580,"y":2570,"z":-85},{"x":-530,"y":2565,"z":-87},{"x":-515,"y":2550,"z":-87},{"x":-390,"y":2545,"z":-86},{"x":-360,"y":2530,"z":-89},{"x":-325,"y":2515,"z":-92},{"x":-320,"y":2565,"z":-93},{"x":-365,"y":2515,"z":-88},{"x":-305,"y":2530,"z":-93},{"x":-305,"y":2555,"z":-93},{"x":-310,"y":2610,"z":-89},{"x":-425,"y":1605,"z":-126},{"x":-410,"y":1585,"z":-127},{"x":-395,"y":1595,"z":-127},{"x":-385,"y":1640,"z":-128},{"x":-420,"y":1660,"z":-126},{"x":-405,"y":1685,"z":-128},{"x":-335,"y":1650`
+`,"z":-128},{"x":-330,"y":1730,"z":-122},{"x":-320,"y":1740,"z":-120},{"x":-265,"y":1740,"z":-103},{"x":-265,"y":1785,"z":-105},{"x":-310,"y":1780,"z":-116},{"x":-505,"y":1665,"z":-122},{"x":-575,"y":1665,"z":-117},{"x":-495,"y":1635,"z":-126},{"x":-160,"y":2000,"z":-126},{"x":-160,"y":2060,"z":-125},{"x":-150,"y":2070,"z":-125},{"x":-85,"y":2070,"z":-125},{"x":-90,"y":2210,"z":-128},{"x":-145,"y":2200,"z":-126},{"x":-105,"y":2225,"z":-128},{"x":-135,"y":2220,"z":-127},{"x":-220,"y":2200,"z":-126`
+`},{"x":-230,"y":2210,"z":-126},{"x":-225,"y":2290,"z":-121},{"x":-410,"y":2195,"z":-125},{"x":-315,"y":2295,"z":-118},{"x":-465,"y":2190,"z":-125},{"x":-505,"y":895,"z":-27},{"x":-270,"y":1120,"z":-80},{"x":-500,"y":1125,"z":-81},{"x":-510,"y":1110,"z":-75},{"x":-530,"y":1110,"z":-73},{"x":-530,"y":895,"z":-26},{"x":-330,"y":2305,"z":-116},{"x":-330,"y":2435,"z":-90},{"x":-405,"y":-1055,"z":101},{"x":-340,"y":-1055,"z":86},{"x":-340,"y":-650,"z":86},{"x":-315,"y":-1055,"z":80},{"x":-490,"y":1365`
+`,"z":-120},{"x":-475,"y":1380,"z":-126},{"x":-365,"y":1380,"z":-126},{"x":-365,"y":1480,"z":-126},{"x":-340,"y":1495,"z":-126},{"x":-265,"y":1490,"z":-125},{"x":-265,"y":1525,"z":-125},{"x":-320,"y":1525,"z":-126},{"x":-330,"y":1535,"z":-126},{"x":-490,"y":1485,"z":-120},{"x":-335,"y":1615,"z":-128},{"x":-430,"y":1570,"z":-126},{"x":-495,"y":1605,"z":-126},{"x":-500,"y":2525,"z":-20},{"x":-505,"y":2485,"z":-31},{"x":-490,"y":2485,"z":-20},{"x":-485,"y":2525,"z":-20},{"x":-335,"y":1360,"z":-126},`
+`{"x":-265,"y":1360,"z":-123},{"x":-340,"y":-640,"z":138},{"x":-340,"y":-620,"z":140},{"x":-495,"y":-620,"z":158},{"x":-495,"y":-630,"z":158},{"x":-495,"y":-600,"z":2},{"x":-395,"y":-600,"z":1},{"x":-395,"y":-500,"z":1},{"x":-335,"y":-500,"z":0},{"x":-300,"y":-460,"z":8},{"x":-380,"y":-385,"z":1},{"x":-370,"y":-265,"z":1},{"x":-380,"y":-260,"z":1},{"x":-380,"y":-200,"z":0},{"x":-395,"y":-190,"z":0},{"x":-425,"y":-135,"z":-1},{"x":-445,"y":-155,"z":-1},{"x":-460,"y":-210,"z":0},{"x":-495,"y":-220,`
+`"z":0},{"x":-485,"y":-155,"z":69},{"x":-495,"y":-175,"z":69},{"x":-470,"y":-175,"z":69},{"x":-455,"y":-160,"z":69},{"x":-460,"y":-150,"z":69},{"x":-455,"y":-80,"z":107},{"x":-480,"y":-70,"z":107},{"x":-495,"y":-135,"z":108},{"x":-470,"y":-145,"z":107},{"x":-455,"y":-100,"z":107},{"x":-415,"y":2465,"z":-27},{"x":-480,"y":2475,"z":-49},{"x":-430,"y":2455,"z":-33},{"x":-355,"y":2465,"z":-4},{"x":-330,"y":2455,"z":1},{"x":-330,"y":2465,"z":3},{"x":-355,"y":2475,"z":-21},{"x":-390,"y":2470,"z":-18},{`
+`"x":-330,"y":2475,"z":-20},{"x":-330,"y":2500,"z":-18},{"x":-375,"y":2505,"z":-21},{"x":-375,"y":2475,"z":-17},{"x":-410,"y":2485,"z":1},{"x":-390,"y":2515,"z":-21},{"x":-435,"y":2475,"z":-10},{"x":-475,"y":2525,"z":1},{"x":-480,"y":2485,"z":1},{"x":-265,"y":2615,"z":25},{"x":-390,"y":2625,"z":19},{"x":-470,"y":2625,"z":19},{"x":-475,"y":2610,"z":19},{"x":-355,"y":2605,"z":27},{"x":-355,"y":2515,"z":26},{"x":-320,"y":2505,"z":26},{"x":-265,"y":2510,"z":26},{"x":5,"y":-605,"z":2},{"x":5,"y":-490,`
+`"z":1},{"x":-65,"y":-490,"z":1},{"x":40,"y":-490,"z":2},{"x":-75,"y":-460,"z":8},{"x":-320,"y":275,"z":3},{"x":-345,"y":245,"z":1},{"x":-275,"y":260,"z":0},{"x":-285,"y":275,"z":0},{"x":-265,"y":1470,"z":-29},{"x":-275,"y":1475,"z":-29},{"x":-345,"y":1470,"z":-28},{"x":-345,"y":1390,"z":-28},{"x":-335,"y":1380,"z":-29},{"x":-265,"y":1380,"z":-29},{"x":-305,"y":-1070,"z":76},{"x":-150,"y":-920,"z":31},{"x":-145,"y":-645,"z":37},{"x":-330,"y":-620,"z":102},{"x":-330,"y":-635,"z":101},{"x":-190,"y"`
+`:-635,"z":102},{"x":-190,"y":-620,"z":103},{"x":165,"y":-790,"z":-11},{"x":145,"y":-790,"z":-11},{"x":160,"y":-1020,"z":1},{"x":170,"y":-1015,"z":1},{"x":190,"y":-1005,"z":1},{"x":5,"y":-645,"z":5},{"x":-305,"y":-1170,"z":74},{"x":155,"y":-1155,"z":0},{"x":180,"y":-1170,"z":0},{"x":180,"y":-1160,"z":0},{"x":-230,"y":770,"z":30},{"x":-230,"y":1525,"z":30},{"x":-250,"y":1525,"z":30},{"x":-245,"y":1280,"z":32},{"x":-255,"y":1480,"z":29},{"x":-255,"y":780,"z":29},{"x":-185,"y":585,"z":0},{"x":-175,"`
+`y":570,"z":0},{"x":-140,"y":575,"z":0},{"x":-220,"y":765,"z":0},{"x":-40,"y":1460,"z":-1},{"x":-65,"y":1340,"z":0},{"x":-40,"y":1340,"z":0},{"x":100,"y":1345,"z":0},{"x":105,"y":1525,"z":1},{"x":60,"y":1535,"z":0},{"x":-140,"y":1270,"z":0},{"x":-70,"y":1460,"z":-1},{"x":-130,"y":1525,"z":0},{"x":-220,"y":1525,"z":1},{"x":-190,"y":550,"z":44},{"x":-185,"y":575,"z":44},{"x":-235,"y":575,"z":44},{"x":-215,"y":560,"z":44},{"x":-240,"y":560,"z":44},{"x":-235,"y":550,"z":44},{"x":40,"y":305,"z":0},{"x`
+`":75,"y":295,"z":0},{"x":-65,"y":470,"z":-1},{"x":-140,"y":545,"z":0},{"x":85,"y":465,"z":0},{"x":-175,"y":555,"z":0},{"x":-180,"y":-615,"z":61},{"x":-180,"y":-635,"z":61},{"x":5,"y":-635,"z":61},{"x":5,"y":-615,"z":61},{"x":60,"y":1590,"z":1},{"x":-115,"y":1590,"z":2},{"x":-115,"y":1535,"z":1},{"x":-60,"y":2060,"z":-125},{"x":-60,"y":2000,"z":-122},{"x":25,"y":2005,"z":-123},{"x":90,"y":2050,"z":-123},{"x":145,"y":2045,"z":-123},{"x":95,"y":2165,"z":-124},{"x":45,"y":2220,"z":-131},{"x":-40,"y"`
+`:2295,"z":-127},{"x":145,"y":2165,"z":-124},{"x":-100,"y":2295,"z":-128},{"x":-90,"y":2235,"z":-128},{"x":105,"y":2310,"z":-31},{"x":55,"y":2365,"z":-32},{"x":40,"y":2365,"z":-30},{"x":-10,"y":2315,"z":-31},{"x":-10,"y":2290,"z":-31},{"x":35,"y":2240,"z":-31},{"x":50,"y":2240,"z":-31},{"x":105,"y":2295,"z":-31},{"x":450,"y":-1005,"z":0},{"x":445,"y":-650,"z":2},{"x":365,"y":-650,"z":3},{"x":340,"y":-620,"z":2},{"x":15,"y":-620,"z":4},{"x":355,"y":-595,"z":0},{"x":375,"y":-555,"z":1},{"x":375,"y"`
+`:-595,"z":0},{"x":385,"y":-550,"z":0},{"x":375,"y":-380,"z":2},{"x":390,"y":-370,"z":8},{"x":360,"y":-340,"z":2},{"x":110,"y":-420,"z":2},{"x":105,"y":-335,"z":0},{"x":320,"y":-300,"z":0},{"x":110,"y":-265,"z":2},{"x":340,"y":2050,"z":-125},{"x":340,"y":2235,"z":-126},{"x":470,"y":2235,"z":-130},{"x":515,"y":2285,"z":-124},{"x":500,"y":2295,"z":-126},{"x":495,"y":2330,"z":-117},{"x":125,"y":2295,"z":-126},{"x":115,"y":2325,"z":-117},{"x":60,"y":2380,"z":-116},{"x":500,"y":2550,"z":-107},{"x":395`
+`,"y":2540,"z":-118},{"x":385,"y":2555,"z":-117},{"x":85,"y":2400,"z":-118},{"x":210,"y":2555,"z":-118},{"x":200,"y":2540,"z":-118},{"x":180,"y":2550,"z":-117},{"x":80,"y":2550,"z":-117},{"x":145,"y":225,"z":8},{"x":205,"y":280,"z":0},{"x":220,"y":320,"z":-1},{"x":225,"y":445,"z":0},{"x":205,"y":470,"z":1},{"x":100,"y":475,"z":0},{"x":255,"y":1525,"z":2},{"x":290,"y":1360,"z":0},{"x":315,"y":1360,"z":0},{"x":310,"y":1470,"z":0},{"x":270,"y":1535,"z":2},{"x":110,"y":1330,"z":1},{"x":225,"y":1345,"`
+`z":0},{"x":215,"y":1330,"z":1},{"x":280,"y":1345,"z":0},{"x":320,"y":-255,"z":-1},{"x":295,"y":-255,"z":0},{"x":145,"y":-255,"z":8},{"x":210,"y":-165,"z":8},{"x":130,"y":-165,"z":9},{"x":130,"y":-190,"z":9},{"x":145,"y":-200,"z":8},{"x":195,"y":15,"z":7},{"x":225,"y":-50,"z":3},{"x":615,"y":-45,"z":-2},{"x":600,"y":30,"z":-3},{"x":635,"y":50,"z":-2},{"x":680,"y":55,"z":0},{"x":755,"y":245,"z":8},{"x":660,"y":245,"z":-1},{"x":735,"y":-145,"z":1},{"x":760,"y":-140,"z":8},{"x":710,"y":275,"z":0},{"`
+`x":640,"y":255,"z":0},{"x":660,"y":285,"z":1},{"x":720,"y":325,"z":1},{"x":345,"y":110,"z":-4},{"x":650,"y":335,"z":1},{"x":590,"y":300,"z":1},{"x":580,"y":275,"z":1},{"x":540,"y":245,"z":1},{"x":475,"y":245,"z":0},{"x":195,"y":110,"z":7},{"x":170,"y":70,"z":8},{"x":165,"y":80,"z":8},{"x":145,"y":50,"z":8},{"x":145,"y":15,"z":8},{"x":130,"y":-5,"z":9},{"x":150,"y":95,"z":8},{"x":460,"y":255,"z":0},{"x":460,"y":280,"z":1},{"x":145,"y":145,"z":8},{"x":130,"y":135,"z":9},{"x":130,"y":85,"z":9},{"x"`
+`:135,"y":80,"z":53},{"x":135,"y":60,"z":53},{"x":155,"y":60,"z":53},{"x":150,"y":85,"z":53},{"x":145,"y":-95,"z":44},{"x":140,"y":-160,"z":44},{"x":210,"y":-155,"z":44},{"x":205,"y":-90,"z":44},{"x":205,"y":-65,"z":79},{"x":190,"y":-5,"z":79},{"x":145,"y":-20,"z":79},{"x":145,"y":-60,"z":79},{"x":155,"y":-85,"z":79},{"x":165,"y":-1150,"z":40},{"x":175,"y":-1150,"z":40},{"x":180,"y":-1025,"z":40},{"x":470,"y":310,"z":6},{"x":470,"y":470,"z":2},{"x":250,"y":455,"z":0},{"x":250,"y":470,"z":0},{"x":`
+`555,"y":-370,"z":8},{"x":595,"y":-360,"z":7},{"x":605,"y":-230,"z":-2},{"x":565,"y":-385,"z":8},{"x":665,"y":-170,"z":-2},{"x":645,"y":-145,"z":-2},{"x":220,"y":-150,"z":8},{"x":210,"y":-80,"z":7},{"x":230,"y":-70,"z":5},{"x":420,"y":1360,"z":0},{"x":420,"y":1430,"z":0},{"x":500,"y":1445,"z":0},{"x":505,"y":1635,"z":1},{"x":495,"y":1760,"z":5},{"x":420,"y":1650,"z":3},{"x":425,"y":1755,"z":4},{"x":410,"y":1650,"z":19},{"x":405,"y":1650,"z":21},{"x":405,"y":1640,"z":3},{"x":390,"y":1640,"z":3},{"`
+`x":390,"y":1650,"z":20},{"x":270,"y":1655,"z":10},{"x":265,"y":1895,"z":98},{"x":405,"y":1765,"z":99},{"x":415,"y":1775,"z":102},{"x":500,"y":1775,"z":102},{"x":505,"y":1850,"z":97},{"x":500,"y":1960,"z":96},{"x":305,"y":1910,"z":97},{"x":425,"y":2040,"z":96},{"x":435,"y":2045,"z":96},{"x":405,"y":2080,"z":96},{"x":425,"y":1965,"z":96},{"x":300,"y":2080,"z":97},{"x":290,"y":2105,"z":98},{"x":500,"y":2045,"z":96},{"x":520,"y":2065,"z":96},{"x":515,"y":2430,"z":96},{"x":505,"y":2460,"z":96},{"x":2`
+`65,"y":2460,"z":98},{"x":525,"y":2435,"z":96},{"x":265,"y":2105,"z":98},{"x":265,"y":2535,"z":98},{"x":595,"y":2435,"z":97},{"x":595,"y":2495,"z":96},{"x":290,"y":2535,"z":98},{"x":715,"y":2435,"z":97},{"x":560,"y":2535,"z":96},{"x":545,"y":2535,"z":96},{"x":530,"y":2550,"z":96},{"x":530,"y":2575,"z":96},{"x":505,"y":2590,"z":96},{"x":540,"y":2630,"z":95},{"x":555,"y":2610,"z":95},{"x":580,"y":2630,"z":95},{"x":590,"y":2700,"z":97},{"x":520,"y":2630,"z":95},{"x":575,"y":2710,"z":97},{"x":505,"y"`
+`:2615,"z":96},{"x":575,"y":2775,"z":97},{"x":405,"y":2755,"z":97},{"x":380,"y":2775,"z":98},{"x":410,"y":2725,"z":97},{"x":395,"y":2710,"z":96},{"x":370,"y":2715,"z":104},{"x":360,"y":2700,"z":104},{"x":305,"y":2695,"z":97},{"x":305,"y":2560,"z":97},{"x":370,"y":2755,"z":138},{"x":370,"y":2775,"z":138},{"x":310,"y":2765,"z":138},{"x":310,"y":2755,"z":138},{"x":270,"y":2775,"z":138},{"x":270,"y":2755,"z":138},{"x":300,"y":2705,"z":138},{"x":360,"y":2710,"z":130},{"x":365,"y":2725,"z":131},{"x":39`
+`0,"y":2720,"z":138},{"x":400,"y":2730,"z":138},{"x":395,"y":2750,"z":138},{"x":520,"y":2070,"z":-120},{"x":800,"y":2065,"z":-48},{"x":800,"y":2200,"z":-48},{"x":715,"y":2290,"z":-62},{"x":500,"y":2045,"z":-124},{"x":370,"y":-640,"z":48},{"x":375,"y":-605,"z":49},{"x":355,"y":-605,"z":48},{"x":350,"y":-625,"z":48},{"x":435,"y":2035,"z":132},{"x":435,"y":1970,"z":132},{"x":500,"y":1970,"z":132},{"x":500,"y":2035,"z":132},{"x":650,"y":785,"z":1},{"x":630,"y":770,"z":1},{"x":660,"y":730,"z":1},{"x":`
+`725,"y":780,"z":4},{"x":710,"y":800,"z":0},{"x":690,"y":800,"z":0},{"x":690,"y":935,"z":0},{"x":725,"y":1080,"z":0},{"x":720,"y":935,"z":0},{"x":885,"y":1080,"z":1},{"x":710,"y":1090,"z":1},{"x":705,"y":1210,"z":2},{"x":525,"y":1170,"z":3},{"x":530,"y":1070,"z":2},{"x":525,"y":1040,"z":2},{"x":510,"y":1030,"z":1},{"x":635,"y":805,"z":0},{"x":510,"y":795,"z":2},{"x":540,"y":795,"z":3},{"x":575,"y":770,"z":1},{"x":545,"y":2590,"z":131},{"x":540,"y":2620,"z":131},{"x":520,"y":2620,"z":131},{"x":515`
+`,"y":2595,"z":131},{"x":525,"y":2585,"z":131},{"x":1275,"y":2310,"z":125},{"x":1285,"y":2720,"z":124},{"x":1260,"y":2720,"z":125},{"x":1265,"y":2480,"z":126},{"x":1255,"y":2330,"z":125},{"x":1055,"y":2330,"z":125},{"x":1030,"y":2310,"z":125},{"x":795,"y":2425,"z":125},{"x":790,"y":2460,"z":138},{"x":760,"y":2450,"z":137},{"x":770,"y":2435,"z":138},{"x":750,"y":2430,"z":136},{"x":735,"y":2425,"z":127},{"x":750,"y":2440,"z":138},{"x":730,"y":2460,"z":138},{"x":725,"y":2435,"z":138},{"x":525,"y":24`
+`25,"z":125},{"x":520,"y":2065,"z":124},{"x":545,"y":2065,"z":125},{"x":540,"y":2400,"z":126},{"x":550,"y":2410,"z":126},{"x":575,"y":2410,"z":126},{"x":1050,"y":2425,"z":125},{"x":560,"y":2400,"z":124},{"x":1020,"y":2410,"z":127},{"x":1030,"y":2400,"z":125},{"x":635,"y":355,"z":1},{"x":760,"y":340,"z":0},{"x":760,"y":435,"z":0},{"x":750,"y":440,"z":0},{"x":760,"y":465,"z":0},{"x":760,"y":575,"z":0},{"x":750,"y":585,"z":0},{"x":690,"y":575,"z":1},{"x":680,"y":605,"z":1},{"x":645,"y":615,"z":1},{"`
+`x":645,"y":700,"z":1},{"x":635,"y":715,"z":1},{"x":585,"y":745,"z":1},{"x":570,"y":715,"z":1},{"x":525,"y":705,"z":1},{"x":535,"y":605,"z":1},{"x":525,"y":610,"z":1},{"x":525,"y":575,"z":1},{"x":535,"y":460,"z":1},{"x":525,"y":465,"z":1},{"x":525,"y":340,"z":2},{"x":575,"y":320,"z":1},{"x":570,"y":2555,"z":138},{"x":555,"y":2580,"z":137},{"x":540,"y":2575,"z":137},{"x":540,"y":2555,"z":138},{"x":550,"y":2545,"z":137},{"x":625,"y":2365,"z":65},{"x":625,"y":2395,"z":65},{"x":560,"y":2390,"z":65},{`
+`"x":555,"y":2370,"z":65},{"x":565,"y":2355,"z":65},{"x":580,"y":2550,"z":96},{"x":710,"y":2450,"z":97},{"x":725,"y":2470,"z":96},{"x":760,"y":2460,"z":96},{"x":785,"y":2475,"z":96},{"x":800,"y":2460,"z":96},{"x":835,"y":2490,"z":95},{"x":840,"y":2690,"z":96},{"x":815,"y":2710,"z":96},{"x":850,"y":2750,"z":96},{"x":840,"y":2775,"z":100},{"x":740,"y":2700,"z":96},{"x":750,"y":2775,"z":98},{"x":750,"y":2710,"z":96},{"x":580,"y":2575,"z":96},{"x":555,"y":2590,"z":96},{"x":770,"y":-355,"z":8},{"x":74`
+`5,"y":-330,"z":8},{"x":565,"y":-400,"z":8},{"x":585,"y":-410,"z":8},{"x":700,"y":-410,"z":8},{"x":720,"y":-400,"z":8},{"x":710,"y":-345,"z":7},{"x":735,"y":-355,"z":8},{"x":725,"y":-360,"z":8},{"x":725,"y":-315,"z":7},{"x":565,"y":2325,"z":-28},{"x":565,"y":2305,"z":-28},{"x":635,"y":2305,"z":-12},{"x":635,"y":2325,"z":-12},{"x":605,"y":2330,"z":-20},{"x":605,"y":2355,"z":28},{"x":605,"y":2335,"z":28},{"x":635,"y":2335,"z":28},{"x":635,"y":2360,"z":28},{"x":695,"y":-10,"z":37},{"x":675,"y":45,"z`
+`":33},{"x":610,"y":25,"z":33},{"x":660,"y":-160,"z":35},{"x":690,"y":-35,"z":59},{"x":715,"y":-150,"z":35},{"x":725,"y":-145,"z":35},{"x":720,"y":-105,"z":37},{"x":710,"y":-70,"z":41},{"x":700,"y":-70,"z":58},{"x":705,"y":-45,"z":37},{"x":645,"y":2325,"z":-43},{"x":645,"y":2300,"z":-43},{"x":710,"y":2300,"z":-29},{"x":710,"y":2325,"z":-29},{"x":645,"y":2390,"z":-1},{"x":645,"y":2335,"z":-1},{"x":710,"y":2335,"z":5},{"x":705,"y":2390,"z":4},{"x":745,"y":795,"z":3},{"x":810,"y":795,"z":0},{"x":805`
+`,"y":810,"z":0},{"x":820,"y":830,"z":0},{"x":840,"y":830,"z":0},{"x":850,"y":815,"z":0},{"x":880,"y":840,"z":0},{"x":730,"y":2710,"z":170},{"x":730,"y":2775,"z":170},{"x":705,"y":2775,"z":170},{"x":705,"y":2765,"z":170},{"x":710,"y":2710,"z":170},{"x":900,"y":2065,"z":-31},{"x":910,"y":2030,"z":-23},{"x":960,"y":2030,"z":-12},{"x":960,"y":2180,"z":-8},{"x":975,"y":2180,"z":-5},{"x":975,"y":2240,"z":2},{"x":1030,"y":2295,"z":19},{"x":1015,"y":2305,"z":19},{"x":1015,"y":2390,"z":23},{"x":760,"y":2`
+`390,"z":-42},{"x":765,"y":2365,"z":-43},{"x":725,"y":2350,"z":-54},{"x":725,"y":2295,"z":-58},{"x":715,"y":2390,"z":22},{"x":715,"y":2365,"z":22},{"x":745,"y":2370,"z":25},{"x":745,"y":2390,"z":25},{"x":730,"y":1205,"z":115},{"x":725,"y":1130,"z":123},{"x":730,"y":1100,"z":116},{"x":745,"y":1110,"z":118},{"x":750,"y":1190,"z":117},{"x":890,"y":1100,"z":115},{"x":835,"y":1145,"z":132},{"x":830,"y":1105,"z":118},{"x":890,"y":1205,"z":115},{"x":825,"y":1190,"z":123},{"x":825,"y":1180,"z":137},{"x":`
+`825,"y":1145,"z":140},{"x":755,"y":1180,"z":145},{"x":755,"y":1115,"z":150},{"x":825,"y":1115,"z":142},{"x":1015,"y":2580,"z":96},{"x":1080,"y":2570,"z":96},{"x":1080,"y":2720,"z":95},{"x":1025,"y":2775,"z":100},{"x":885,"y":2775,"z":101},{"x":880,"y":2750,"z":97},{"x":885,"y":2725,"z":97},{"x":905,"y":2710,"z":97},{"x":900,"y":2690,"z":96},{"x":870,"y":2685,"z":96},{"x":860,"y":2705,"z":96},{"x":805,"y":2435,"z":97},{"x":995,"y":2500,"z":96},{"x":1005,"y":2505,"z":96},{"x":995,"y":2435,"z":96},`
+`{"x":840,"y":795,"z":46},{"x":840,"y":820,"z":46},{"x":815,"y":810,"z":46},{"x":820,"y":795,"z":46},{"x":855,"y":2710,"z":118},{"x":845,"y":2730,"z":120},{"x":825,"y":2715,"z":120},{"x":845,"y":2700,"z":118},{"x":900,"y":785,"z":1},{"x":975,"y":705,"z":8},{"x":990,"y":720,"z":7},{"x":1240,"y":720,"z":8},{"x":1240,"y":775,"z":8},{"x":1260,"y":800,"z":0},{"x":1250,"y":1205,"z":0},{"x":1210,"y":1205,"z":0},{"x":1205,"y":1215,"z":0},{"x":1165,"y":1205,"z":0},{"x":980,"y":1140,"z":1},{"x":910,"y":109`
+`0,"z":1},{"x":1170,"y":1215,"z":0},{"x":990,"y":1215,"z":0},{"x":980,"y":1205,"z":0},{"x":910,"y":1135,"z":1},{"x":840,"y":785,"z":0},{"x":875,"y":2775,"z":142},{"x":850,"y":2775,"z":141},{"x":860,"y":2755,"z":142},{"x":875,"y":2725,"z":141},{"x":850,"y":2735,"z":142},{"x":870,"y":2720,"z":140},{"x":870,"y":2695,"z":141},{"x":895,"y":2700,"z":141},{"x":910,"y":1205,"z":36},{"x":910,"y":1145,"z":36},{"x":970,"y":1145,"z":36},{"x":970,"y":1205,"z":36},{"x":1240,"y":205,"z":10},{"x":975,"y":610,"z"`
+`:9},{"x":960,"y":600,"z":10},{"x":960,"y":205,"z":12},{"x":1255,"y":2030,"z":2},{"x":1255,"y":2295,"z":13},{"x":1085,"y":2560,"z":152},{"x":1045,"y":2570,"z":152},{"x":1025,"y":2570,"z":152},{"x":1010,"y":2520,"z":152},{"x":1070,"y":2505,"z":152},{"x":1015,"y":2490,"z":168},{"x":1015,"y":2435,"z":168},{"x":1065,"y":2440,"z":169},{"x":1065,"y":2495,"z":168},{"x":1095,"y":2560,"z":96},{"x":1190,"y":2560,"z":96},{"x":1250,"y":2560,"z":99},{"x":1250,"y":2795,"z":116},{"x":1260,"y":2790,"z":125},{"x"`
+`:1300,"y":2820,"z":122},{"x":1280,"y":2835,"z":125},{"x":1280,"y":2850,"z":126},{"x":1300,"y":2865,"z":126},{"x":1270,"y":2900,"z":128},{"x":1040,"y":2905,"z":128},{"x":1040,"y":2785,"z":108},{"x":1045,"y":2945,"z":129},{"x":1330,"y":2855,"z":116},{"x":1355,"y":2870,"z":117},{"x":1370,"y":2850,"z":115},{"x":1385,"y":2865,"z":116},{"x":1575,"y":2865,"z":124},{"x":1570,"y":2975,"z":128},{"x":1595,"y":2985,"z":133},{"x":1595,"y":3010,"z":133},{"x":1570,"y":3020,"z":127},{"x":1575,"y":3070,"z":127},`
+`{"x":1045,"y":3020,"z":129},{"x":1030,"y":3005,"z":129},{"x":1030,"y":2960,"z":129},{"x":1040,"y":3070,"z":130},{"x":1180,"y":2550,"z":96},{"x":1080,"y":2515,"z":96},{"x":1085,"y":2430,"z":96},{"x":1060,"y":2420,"z":97},{"x":1060,"y":2340,"z":99},{"x":1180,"y":2480,"z":95},{"x":1250,"y":2340,"z":100},{"x":1250,"y":2470,"z":97},{"x":1190,"y":2470,"z":96},{"x":1200,"y":2540,"z":170},{"x":1200,"y":2490,"z":170},{"x":1255,"y":2490,"z":170},{"x":1255,"y":2540,"z":170},{"x":1245,"y":1550,"z":0},{"x":1`
+`265,"y":1535,"z":1},{"x":1605,"y":1540,"z":0},{"x":1610,"y":1675,"z":0},{"x":1250,"y":1595,"z":1},{"x":1250,"y":1675,"z":0},{"x":1250,"y":1635,"z":1},{"x":1235,"y":1625,"z":1},{"x":1235,"y":1605,"z":1},{"x":1235,"y":1690,"z":0},{"x":1640,"y":1705,"z":1},{"x":1665,"y":1755,"z":1},{"x":1660,"y":1780,"z":0},{"x":1685,"y":1805,"z":0},{"x":1655,"y":1840,"z":0},{"x":1265,"y":1795,"z":3},{"x":1235,"y":1765,"z":0},{"x":1265,"y":1840,"z":0},{"x":1695,"y":1920,"z":1},{"x":1680,"y":1930,"z":1},{"x":1265,"y`
+`":1925,"z":2},{"x":1250,"y":765,"z":30},{"x":1250,"y":335,"z":29},{"x":1270,"y":335,"z":30},{"x":1275,"y":770,"z":30},{"x":1280,"y":780,"z":-7},{"x":1425,"y":925,"z":-9},{"x":1505,"y":925,"z":-9},{"x":1605,"y":1030,"z":1},{"x":1590,"y":1040,"z":1},{"x":1600,"y":1065,"z":1},{"x":1600,"y":1240,"z":1},{"x":1585,"y":1265,"z":1},{"x":1265,"y":1215,"z":0},{"x":1280,"y":1260,"z":0},{"x":1265,"y":1245,"z":0},{"x":1250,"y":1260,"z":1},{"x":1585,"y":1295,"z":1},{"x":1595,"y":1300,"z":1},{"x":1590,"y":1415`
+`,"z":1},{"x":1265,"y":1375,"z":0},{"x":1250,"y":1365,"z":1},{"x":1265,"y":1415,"z":0},{"x":1610,"y":1435,"z":1},{"x":1265,"y":2015,"z":1},{"x":1300,"y":2050,"z":0},{"x":1640,"y":2050,"z":0},{"x":1630,"y":2125,"z":0},{"x":1665,"y":2145,"z":1},{"x":1785,"y":2280,"z":4},{"x":1720,"y":2125,"z":2},{"x":1755,"y":1990,"z":3},{"x":1800,"y":1995,"z":3},{"x":1705,"y":2145,"z":2},{"x":1730,"y":2270,"z":2},{"x":1735,"y":2280,"z":3},{"x":1715,"y":2280,"z":10},{"x":1675,"y":2270,"z":1},{"x":1680,"y":2280,"z":`
+`10},{"x":1630,"y":2280,"z":0},{"x":1620,"y":2290,"z":1},{"x":1620,"y":2435,"z":33},{"x":1320,"y":2360,"z":24},{"x":1295,"y":2305,"z":15},{"x":1320,"y":2430,"z":43},{"x":1295,"y":2355,"z":23},{"x":1295,"y":2720,"z":108},{"x":1295,"y":2650,"z":95},{"x":1615,"y":2645,"z":86},{"x":1620,"y":2760,"z":110},{"x":1575,"y":2770,"z":113},{"x":1360,"y":2825,"z":114},{"x":1325,"y":2835,"z":115},{"x":1750,"y":1935,"z":2},{"x":1800,"y":1805,"z":2},{"x":1760,"y":1945,"z":2},{"x":1285,"y":765,"z":-10},{"x":1585,`
+`"y":770,"z":-10},{"x":1590,"y":780,"z":-9},{"x":1625,"y":780,"z":-5},{"x":1630,"y":770,"z":1},{"x":1780,"y":775,"z":1},{"x":1790,"y":805,"z":0},{"x":1780,"y":995,"z":1},{"x":1725,"y":995,"z":0},{"x":1720,"y":1025,"z":1},{"x":1580,"y":200,"z":-181},{"x":1580,"y":670,"z":-37},{"x":1290,"y":665,"z":-39},{"x":1285,"y":200,"z":-183},{"x":1315,"y":185,"z":-187},{"x":1535,"y":185,"z":-187},{"x":1290,"y":2425,"z":87},{"x":1290,"y":2365,"z":75},{"x":1310,"y":2365,"z":75},{"x":1310,"y":2425,"z":87},{"x":1`
+`295,"y":2440,"z":49},{"x":1290,"y":2465,"z":60},{"x":1315,"y":2830,"z":165},{"x":1315,"y":2855,"z":165},{"x":1295,"y":2855,"z":166},{"x":1290,"y":2840,"z":166},{"x":1335,"y":2855,"z":159},{"x":1335,"y":2835,"z":159},{"x":1360,"y":2835,"z":159},{"x":1355,"y":2860,"z":160},{"x":1620,"y":720,"z":53},{"x":1620,"y":770,"z":53},{"x":1595,"y":770,"z":53},{"x":1595,"y":760,"z":53},{"x":1600,"y":720,"z":55},{"x":1600,"y":710,"z":78},{"x":1600,"y":335,"z":78},{"x":1620,"y":335,"z":76},{"x":1620,"y":710,"z`
+`":76},{"x":1780,"y":305,"z":58},{"x":1630,"y":305,"z":58},{"x":1750,"y":1945,"z":35},{"x":1705,"y":2135,"z":34},{"x":1665,"y":2135,"z":35},{"x":1640,"y":2120,"z":34},{"x":1640,"y":2105,"z":35},{"x":1685,"y":1935,"z":36},{"x":1730,"y":1020,"z":64},{"x":1730,"y":1005,"z":64},{"x":1785,"y":1005,"z":64},{"x":1780,"y":1020,"z":64}],"polys":[[0,1,2,3,4,5],[6,7,8,9],[4,6,9,10,11,12],[4,12,13,5],[13,14,5],[15,9,8,16,17],[8,7,18],[8,18,19,20,16],[15,17,21,22],[23,22,21,24,25],[26,27,28,29,30,31],[26,31,3`
+`2,33,34,35],[36,37,38],[38,39,40,41],[36,38,41,42,43,44],[42,45,43],[43,46,47,48],[43,48,44],[36,44,49],[36,49,50,51],[52,53,54,55],[55,56,57],[52,55,57],[52,57,58,59,60,61],[62,60,59,63],[59,58,64,65,40,39],[64,66,65],[59,39,38,37,63],[67,68,69,70],[67,70,71,72,73,74],[74,75,67],[76,45,42,77],[77,78,79],[76,77,79,80,81],[79,82,80],[80,83,81],[76,81,84],[76,84,85,86,87],[86,88,89],[86,89,87],[87,90,76],[91,86,85,92,93,94],[94,95,96,97],[94,97,91],[98,99,100,101],[98,101,102],[103,104,105],[102,1`
+`03,105,106,107],[106,108,107],[98,102,107,109],[98,109,110],[98,110,111,112],[98,112,113,114,115],[113,116,117,114],[118,119,120,121],[25,24,122,123],[124,125,126],[122,124,126,127,123],[128,129,130,131,132,133],[130,134,135,131],[132,136,137,133],[133,138,128],[139,140,141,142],[143,126,125,130,144],[125,124,122,134,130],[130,129,145,144],[146,147,148,149],[146,149,150,151,152],[146,152,153],[146,153,154,155,156],[146,156,157,158],[157,159,158],[146,158,160],[146,160,161,162,163],[161,164,165,1`
+`66,162],[167,168,169,170,171],[165,167,171,166],[28,27,172],[28,172,173,174,175],[28,175,176,177],[28,177,178,179],[28,179,180,181,29],[182,181,180,183],[183,184,185],[182,183,185,186],[185,187,188,189],[185,189,186],[186,190,182],[53,182,190,54],[93,92,191,192],[93,192,193],[193,82,79],[93,193,79,194,195],[79,196,194],[93,195,197,198,199],[197,200,198],[93,199,100,99,201],[93,201,95,94],[202,203,204,205,206],[207,208,209,210,211],[212,213,214,215],[2,1,216,217,218,20],[216,219,217],[218,220,16,`
+`20],[2,20,19,221],[221,3,2],[222,223,224,225,226],[227,228,229,230,231],[227,231,232],[227,232,233,234],[235,236,237],[238,239,240,241],[242,243,244,245],[242,245,246],[247,248,249,250],[251,246,247,250,252,253],[242,246,251,254],[242,254,188,255],[188,187,255],[242,255,256],[242,256,257,258],[242,258,178,176],[178,177,176],[242,176,175,174],[259,260,261,262,263,264],[264,265,259],[266,267,268,269],[270,271,272,273],[272,274,275,276],[276,277,278,279,280],[272,276,280,281],[272,281,282,283,284],`
+`[285,286,287,273,288],[273,289,288],[283,285,288,284],[272,284,290],[272,290,289,273],[131,135,291,292,293],[131,293,294],[294,295,296],[296,297,298],[298,299,300],[298,300,301,302,148,131],[303,304,305],[306,307,308,309,310],[305,306,310],[303,305,310,311],[301,303,311,302],[148,147,132,131],[296,298,131],[294,296,131],[312,313,314],[218,217,315,316,317,318],[218,318,319],[218,319,320,321,322,323],[320,324,321],[218,323,325,326],[218,326,327],[218,327,328,329],[218,329,330,331,332],[218,332,333`
+`,334],[333,335,334],[218,334,336,337],[336,338,337],[218,337,339],[218,339,340],[218,340,341,342,343],[341,344,345,346],[341,346,342],[218,343,347],[218,347,348,349,350],[349,351,350],[218,350,352,353],[218,353,354,355],[355,220,218],[356,357,358,359],[358,360,359],[361,362,363],[364,361,363,365,366],[367,364,366],[359,367,366,368,369,370],[366,371,368],[368,372,369],[369,373,370],[370,374,375],[359,370,375],[359,375,376],[359,376,377,378],[356,359,378,379],[356,379,380,381],[381,382,383],[356,3`
+`81,383,384],[356,384,385,386,387],[386,388,387],[356,387,389],[356,389,390],[356,390,391,392],[391,393,394,392],[392,395,396],[356,392,396],[356,396,65,66,397],[244,243,398,399,400],[398,159,157,399],[244,400,401,402,403],[244,403,404,405],[404,249,248,405],[406,363,362,407,408],[409,360,358,357,410],[407,409,410,411],[410,412,413,414],[410,414,411],[407,411,408],[415,394,393,416],[415,416,417],[415,417,388,386,418],[386,385,419],[419,420,421],[386,419,421,422,423,424],[422,425,426,423],[386,424`
+`,427],[386,427,418],[415,418,428],[415,428,424,423],[250,249,404,429],[250,429,430,252],[431,432,433,434],[431,434,435,436],[437,438,439,440,441],[442,443,444,445,446,447],[445,448,449,446],[450,451,452,453,454,455],[456,457,458,459],[460,461,462,463,464],[465,466,467,468],[469,470,471,472],[430,429,473,474,475],[430,475,476],[476,477,478],[430,476,478,479,480],[478,481,482],[483,484,485,486,487],[485,488,489,490],[485,490,486],[482,483,487,491],[478,482,491,479],[479,492,480],[430,480,493,494],`
+`[495,496,497,498],[498,499,495],[500,501,502,503,504],[504,505,506],[500,504,506,507,508],[507,509,508],[510,511,512,513,514],[515,516,517,518],[519,515,518],[520,519,518,521],[513,520,521,522],[513,522,523,524,514],[525,526,527,528,529],[530,531,532,533,534],[530,534,535,536,537,538],[539,374,370],[539,370,373,540],[540,541,542],[540,542,543,544],[544,545,546],[540,544,546],[539,540,546,547],[539,547,548,549,550],[539,550,551,552],[553,552,551,554],[551,550,549,555],[551,555,556,557],[554,551,5`
+`57,558],[554,558,559,560],[554,560,561,562,563,564],[564,565,566,567,568,569],[564,569,570],[568,567,571,572],[573,568,572,574,575],[573,575,576,577],[576,578,579,577],[577,580,581],[573,577,581,570],[570,581,582],[564,570,582,583,584,585],[582,586,583],[564,585,587],[554,564,587],[554,587,588],[554,588,589,590],[589,584,583,591],[589,591,590],[554,590,592,593],[592,594,595,596],[592,596,593],[597,598,599,600,601,602],[603,481,478,604,605,606],[603,606,607,608],[607,609,610,611],[607,611,612,613`
+`,614,615],[607,615,608],[616,617,618,619],[619,620,621],[616,619,621,622,623],[624,625,626,627],[329,328,628,629,630],[329,630,631],[632,633,634,635],[636,632,635],[636,635,637],[631,636,637],[329,631,637,638],[329,638,639,640,330],[641,642,635,634],[639,641,634,640],[640,643,644],[644,645,646],[640,644,646,330],[646,331,330],[647,648,649,650,651],[652,653,654,655],[656,657,658,659,660],[610,609,661,662],[610,662,663,664],[610,664,665],[610,665,666,667,668],[667,669,668],[610,668,670,671],[610,6`
+`71,612,611],[672,673,674,675,564,563],[672,563,562,561,559,558],[557,556,676,677],[557,677,678,679],[558,557,679],[558,679,672],[680,681,682,683,684,685],[685,686,680],[317,316,687],[317,687,688,689,690,691],[691,692,322,321],[317,691,321,324],[693,691,690,694,695,696],[691,693,697,698],[698,692,691],[699,700,701,702],[674,673,703,704],[674,704,705,706],[675,674,706,707,708,709],[706,705,710],[706,710,711,707],[675,709,712],[675,712,713,714,715],[714,579,578,716],[714,716,572,571],[714,571,566,5`
+`65,715],[717,718,719,720],[721,722,723,724],[721,724,725,667,666,726],[721,726,727,728],[696,729,730,731],[693,696,731,732],[733,693,732,734,735],[736,737,738,739,740,741],[736,742,743],[736,743,744,745,746],[736,746,747,748,749,737],[750,751,752,753],[740,739,754],[754,755,756],[740,754,756,757,758,759],[740,759,760],[740,760,761],[762,763,764,765,766],[761,762,766],[740,761,766,767,768,769],[740,769,770],[740,770,771],[740,771,772],[772,773,774],[740,772,774,775,776],[740,776,777],[740,777,778`
+`,779],[778,780,779],[740,779,781,782],[740,782,783],[740,783,784,785,741],[786,787,788],[788,789,790],[786,788,790],[786,790,791,792,793],[794,793,792,711],[786,793,795,796],[797,798,799,800,801,802],[803,804,805,806,807,808],[803,808,809,810],[803,810,811,812,813],[811,814,812],[812,815,816,813],[813,817,803],[818,819,820,821,822],[823,824,825],[823,825,826],[826,827,828],[823,826,828,829],[823,829,789,788],[822,823,788],[818,822,788,787,830],[787,831,830],[830,832,818],[792,791,833,834],[711,7`
+`92,834,835],[711,835,836,837,838],[837,839,840,838],[711,838,841],[711,841,842],[711,842,843,844],[843,845,844],[844,846,707,711],[847,780,778,848,849,850],[847,850,851,852],[708,707,846],[708,846,844,853,854],[844,845,853],[735,855,856],[857,733,735,856,858],[723,722,859,860],[723,860,861,862],[723,862,863],[863,864,865],[723,863,865,866],[723,866,867,868],[867,869,821,820],[867,820,819,870,868],[870,871,868],[872,873,874,875],[859,849,848,876,861,860],[848,877,876],[878,879,880,881],[882,883,8`
+`84],[884,885,886,887],[882,884,887,888,889],[882,889,890,891],[882,891,758,892],[758,757,892],[882,892,893,894],[894,895,882],[896,897,898,899,900],[901,902,903,904,905],[906,907,908],[909,906,908,910,911],[906,909,912,913],[912,914,915,916,917],[912,917,913],[918,906,913,919],[920,918,919,921,922],[923,924,925,926,927],[923,927,928,929,930],[884,883,931,885],[932,933,931,934],[933,935,886],[931,933,886,885],[936,937,938,939],[940,941,942,943,944,945],[857,858,946,947,948],[949,950,951,952],[953`
+`,954,955,956,957],[955,954,958,948,947],[955,947,946,959,960],[959,961,962,960],[963,964,965,966],[965,967,966],[966,968,963],[774,773,969],[774,969,970,971,972],[973,974,975,976,977,978],[979,974,973,980],[979,980,981,982],[971,979,982,972],[972,775,774],[983,984,985,986],[985,987,988,986],[768,767,989],[768,989,990,991,992],[990,993,991],[768,992,971,994],[971,970,994],[995,996,997,998],[973,978,999,1000,1001],[973,1001,981,980],[837,836,1002],[1002,1003,1004],[837,1002,1004,1005],[837,1005,10`
+`06,1007,1008,1009],[1006,1010,1007],[1009,1011,1012],[837,1009,1012],[1012,839,837],[1013,1014,1015,1016,1017,1018],[1013,1018,1019,1020],[953,957,1021,1022,1023],[953,1023,1024,1025,958,954],[931,1025,1024,1026,1027],[1026,1028,1027],[931,1027,1029,1030],[931,1030,1031,1032,1033,934],[1034,1033,1032,1035,1036],[1010,1006,1037,1038],[1038,1039,1040,1041],[1010,1038,1041,1042,1043,1008],[1042,1044,1043],[1008,1007,1010],[1045,1044,1042,1046,1047],[1045,1047,1048,1049],[1048,1050,1051],[1051,1052,`
+`1053,1049],[1048,1051,1049],[990,1054,1055,1056,1057,1058],[990,1058,1059,993],[1060,1061,1062,1063,1064],[1061,1060,977,976,1065,1066],[1065,1067,1066],[1066,1068,1061],[1036,1035,1069,1070,1071],[1070,1072,1073,1074,1075],[1070,1075,1071],[1076,1077,1078,1079],[1076,1079,1080],[1076,1080,1081,1082,1083],[1081,1084,1085,1082],[1082,1086,1083],[1076,1083,1087],[1076,1087,1088],[1076,1088,1089,1090],[1089,1091,1092,1093],[1089,1093,1094],[1090,1094,1095,1096,1097,1076],[1089,1094,1090],[1096,1098`
+`,1097],[1076,1097,1099,1100],[1100,1101,1076],[1102,1098,1096],[1102,1096,1095,1103],[1102,1103,1104,1055,1054,1105],[1102,1105,1106,1107],[1108,1109,1110,1111],[1112,1113,1114,1115],[1116,1117,1118,1119,1120],[1121,1122,1123],[1056,1055,1104,1124],[1056,1124,1125,1126,1057],[1125,1127,1126],[1035,1032,1031,1128,1129,1130],[1128,1131,1129],[1035,1130,1132,1133,1069],[1133,1134,1072,1070,1069],[1135,1134,1133,1078,1136],[1078,1077,1136],[1063,1062,1137,1138],[1063,1138,1139,1140,1141,1142],[1141,`
+`1143,1142],[1144,1145,1146,1142],[1063,1142,1146],[1063,1146,1147],[1063,1147,1148,1149,1064],[1150,1149,1148,1145,1144,1151],[1150,1151,1152],[1150,1152,1153,1154,1155,1156],[1157,1158,1159],[1160,1157,1159,1161,1156],[1155,1160,1156],[1162,1161,1159],[1159,1158,1163,1164],[1162,1159,1164,1165,1166,1167],[1165,1168,1166],[1167,1169,1162],[1170,1167,1166],[1166,1168,1171],[1172,1173,1170,1166,1171,1174],[1172,1175,1176],[1177,1178,1179],[1180,1181,1182,1183],[1184,1180,1183,1185],[1186,1184,1185`
+`,1187,1188],[1187,1189,1188],[1179,1186,1188,1190],[1177,1179,1190,1191],[1177,1191,1192,1193],[1176,1177,1193,1194],[1172,1176,1194,1195],[1172,1195,1173],[1196,1197,1198,1199],[1198,1200,1201,1199],[1196,1199,1202,1203,1204],[1196,1204,1205,1206,1207],[1208,1209,1210,1211,1040,1039],[1208,1039,1038,1037,1212],[1213,1214,1215,1216],[1217,1218,1219,1220],[1221,1222,1223,1224,1225,1226],[1221,1226,1227],[1228,1227,1229,1230],[1221,1227,1228,1231],[1221,1231,1232,1233,1234],[1221,1234,1235],[1221,`
+`1235,1236,1237],[1236,1238,1239],[1236,1239,1240,1237],[1241,1242,1243,1244,1245],[1246,1247,1248,1249],[1246,1249,1250],[1246,1250,1251,1252],[1253,1254,1255,1256],[1253,1256,1257,1258],[1259,1256,1255,1260],[1257,1259,1260,1261,1258],[1262,1263,1264,1265],[1262,1265,1266],[1253,1258,1262,1266,1267,1268],[1266,1269,1267],[1268,1267,1270],[1268,1270,1271],[1251,1268,1271,1252],[1272,1091,1089,1273,1274,1275],[1272,1275,1276,1277,1278,1279],[1272,1279,1280,1281],[1272,1281,1282,1283],[1283,1223,1`
+`222],[1272,1283,1222,1284,1285],[1272,1285,1286,1287],[1286,1288,1287],[1272,1287,1289,1290],[1289,1291,1290],[1272,1290,1292,1293],[1294,1295,1296,1297,1298],[1299,1300,1301,1302,1303],[1304,1175,1172,1174,1305],[1304,1305,1306],[1304,1306,1307,1308],[1308,1309,1310],[1304,1308,1310,1311,1312],[1304,1312,1313,1314,1315],[1314,1316,1317],[1314,1317,1315],[1318,1304,1315,1183,1182],[1182,1181,1319,1318],[1320,1085,1084,1132,1321],[1132,1130,1129],[1129,1131,1322,1323],[1132,1129,1323,1324,1325,13`
+`26],[1320,1321,1327],[1328,1320,1327],[1328,1327,1326],[1325,1328,1326],[1132,1326,1329],[1132,1329,1321],[1330,1331,1332,1333,1334],[1335,1336,1337,1338],[1339,1340,1341,1342,1343],[1342,1344,1345,1346,1347,1348],[1342,1348,1349,1343],[1350,1351,1352,1353],[1354,1355,1356,1357],[1225,1224,1358],[1225,1358,1359,1360],[1225,1360,1361,1230,1229],[1362,1363,1364],[1361,1362,1364,1230],[1225,1229,1227,1226],[1365,1366,1367,1368,1369],[1210,1209,1370],[1370,1371,1372],[1210,1370,1372,1373],[1210,1373`
+`,1374,1375],[1210,1375,1376,1377],[1210,1377,1378,1379,1380],[1210,1380,1381,1382],[1382,1211,1210],[1383,1384,1385,1386],[1387,1388,1389,1390,1391],[1392,1393,1394],[1389,1392,1394,1390],[1395,1387,1391,1396],[1392,1395,1396,1397,1393],[1398,1397,1399,1400,1401],[1397,1398,1393],[1402,1403,1404,1405,1406,1407],[1407,1408,1409],[1402,1407,1409],[1402,1409,1410],[1402,1410,1411],[1411,1412,1311],[1402,1411,1311,1310],[1402,1310,1309,1413,1414,1415],[1413,1416,1414],[1417,1418,1419,1420],[1421,142`
+`2,1423,1424],[1425,1426,1427],[1425,1427,1428,1429],[1425,1429,1430,1431,1432],[1425,1432,1433,1434,1435,1436],[1433,1437,1434],[1434,1438,1439,1435],[1435,1440,1436],[1425,1436,1230,1364],[1425,1364,1363,1441],[1442,1443,1444,1445],[1445,1444,1446,1447],[1445,1447,1448,1449],[1450,1451,1452,1453],[1454,1428,1427,1426,1455],[1454,1455,1456,1457],[1373,1372,1458,1374],[1458,1459,1376,1375,1374],[1460,1461,1462,1463,1464],[1465,1466,1467,1468],[1404,1403,1469,1470,1471,1472],[1472,1473,1474],[1404`
+`,1472,1474,1475],[1404,1475,1476],[1404,1476,1477,1478,1479,1480],[1480,1405,1404],[1481,1479,1478],[1478,1477,1482,1483],[1483,1484,1485],[1478,1483,1485,1486],[1481,1478,1486,1487],[1481,1487,1488,1489,1490],[1481,1490,1491,1492,1493,1494],[1491,1495,1492],[1496,1470,1469,1497],[1496,1497,1498],[1498,1499,1500],[1496,1498,1500,1501],[1500,1502,1503,1504],[1500,1504,1501],[1505,1506,1507,1508],[1509,1510,1511,1512,1513],[1512,1514,1515],[1512,1515,1516,1517,1513],[1518,1514,1512,1519,1520,1521]`
+`,[1518,1521,1522,1523,1524,1525],[1526,1524,1523,1527,1528,1529],[1530,1531,1532,1533],[1431,1430,1534,1535],[1431,1535,1536,1537,1538],[1431,1538,1539,1540,1541,1542],[1541,1543,1544,1542],[1545,1544,1543],[1545,1543,1541,1546],[1545,1546,1547,1548,1549,1550],[1551,1549,1548,1552,1511,1510],[1458,1553,1554],[1554,1555,1556],[1554,1556,1557,1558],[1559,1560,1561,1558],[1562,1559,1558],[1557,1562,1558],[1458,1554,1558,1563],[1558,1564,1563],[1458,1563,1565,1566],[1565,1567,1566],[1458,1566,1568],`
+`[1458,1568,1569],[1458,1569,1570,1571,1572],[1570,1573,1571],[1571,1574,1572],[1572,1459,1458],[1553,1529,1528,1555,1554],[1473,1248,1247,1575],[1575,1576,1577],[1473,1575,1577,1578,1579],[1473,1579,1486,1580],[1486,1485,1484,1580],[1473,1580,1581,1474],[1582,1523,1522,1583,1584],[1583,1561,1560,1584],[1523,1582,1527],[1534,1585,1586,1587],[1588,1589,1590,1591],[1534,1587,1588,1591,1592,1593],[1534,1593,1594,1536],[1594,1537,1536],[1536,1535,1534],[1595,1596,1597,1598,1599,1600],[1585,1597,1596,`
+`1586],[1601,1602,1603,1604],[1605,1573,1570,1577,1576,1606],[1607,1608,1609,1610],[1611,1612,1613,1614],[1615,1616,1617,1618,1619],[1620,1621,1622,1623],[1624,1590,1589,1625],[1626,1627,1628,1629,1630,1631],[1632,1633,1634,1635]],"regions":[588,588,588,588,588,158,158,158,158,158,471,471,131,131,131,131,131,131,131,131,234,234,234,234,33,33,33,33,1153,1153,1153,386,386,386,386,386,386,386,386,386,386,352,352,352,636,636,636,636,636,636,636,636,636,636,947,354,354,354,10,10,10,10,1075,12,12,12,31`
+`5,315,315,315,315,315,315,315,315,315,315,387,387,387,387,387,532,532,532,532,532,532,765,368,368,368,368,368,368,368,368,368,866,1033,1155,27,27,27,27,27,1162,1086,1086,1086,1161,805,520,520,520,520,520,520,520,520,520,520,520,520,1034,1034,1165,1087,1087,1087,1087,1087,1087,1087,1087,1087,1087,7,7,7,7,7,7,7,7,7,7,7,7,7,7,1169,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,772,772,772,772,772,783,783,783,783,783,783,65`
+`5,655,655,655,655,655,655,655,655,655,655,389,389,1005,1005,920,1364,1364,921,1180,1183,1181,1091,472,472,472,472,472,472,472,472,472,472,472,472,1187,1187,1039,1039,1039,1039,1196,1196,1196,1196,1196,1196,1093,1041,1041,235,235,235,235,235,235,235,235,235,235,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,134,1199,758,758,758,758,758,899,899,899,1059,544,544,544,544,544,544,544,544,544,544,544,544,544,544,959,1096,1410,545,545,545,545,545,545,545`
+`,227,227,227,227,227,227,877,877,160,160,160,160,80,80,80,1212,186,186,99,99,99,99,99,99,99,99,1213,546,546,546,73,73,73,213,213,213,213,1218,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,326,326,326,326,326,326,902,933,933,933,933,933,933,424,424,424,424,424,424,424,424,424,424,138,138,138,138,138,138,138,138,138,374,374,108,108,108,177,177,674,674,674,674,674,674,674,674,674,1226,461,461,1237,548,548,548,548,548,548,548,548,1233,1044,1105,1105,1105,1105,1105,1105,1105,1460,1460,549,`
+`549,549,549,1107,846,61,1243,19,19,19,19,1463,1463,1463,605,605,605,605,605,605,605,1113,1113,445,445,445,445,445,1250,426,426,495,495,495,495,495,495,495,495,792,792,37,37,205,205,205,205,205,84,84,84,84,84,123,123,123,123,123,448,448,561,561,561,561,293,293,293,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,165,165,165,165,1254,906,1256,1474,214,214,214,69,69,69,69,68,68,483,483,483,483,483,483,483,512,512,512,512,512,512,394,394,394,394,394,328,328,328,328,328,328,328,328,328,328,328,328,328,32`
+`8,328,1266,1266,1266,1266,171,171,1263,936,266,266,266,266,266,266,266,266,266,1123,1310,1310,1310,1310,1310,1310,1310,1310,1310,1310,1310,1310,1310,1310,465,465,465,465,465,465,465,465,465,465,465,1126,1063,450,450,450,450,450,450,450,450,450,450,399,399,399,399,399,399,399,399,399,399,1274,1277,884,884,884,1128,969,268,268,268,268,268,268,1282,257,257,257,257,257,257,257,257,1283,943,943,943,943,943,943,943,403,403,403,403,403,403,403,403,403,1293,1135,29,29,29,29,29,29,29,29,29,1302,1302,1302`
+`,973,321,321,298,298,974,995,363,363,363,363,363,363,331,331,331,331,331,331,331,331,634,634,634,634,634,634,997,192,192,192,104,104,104,1140,26,26,26,26,217,217,217,217,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,86,261,261,261,261,261,261,52,52,52,51,51,51,51,51,51,299,288,1316,253,1317,1319,1322,1321,635,1324,1325],"neighbors":[[-1,101,105,-1,3,-1],[-1,6,5,2],[-1,1,-1,-1,-1,3],[2,-1,4,0],[-1,-1,3],[-1,1,7,-1,8],[1,-1,7],[6,-1,104,103,5],[5,-1,9,-1],[-1,8,-1,55,-1],[-1,77,81,-1,-1,11],[10,`
+`-1,-1,-1,-1,-1],[-1,27,14],[27,25,-1,14],[12,13,-1,15,17,18],[31,-1,14],[-1,-1,-1,17],[16,-1,14],[14,-1,19],[18,-1,-1,-1],[-1,88,-1,22],[-1,-1,22],[20,21,23],[22,-1,25,24,-1,-1],[-1,23,27,-1],[23,-1,26,-1,13,27],[-1,200,25],[25,13,12,-1,24],[-1,-1,-1,29],[28,-1,-1,-1,-1,30],[-1,-1,29],[-1,15,-1,33],[-1,-1,33],[31,32,34,35,36],[91,-1,33],[-1,-1,33],[33,-1,37],[36,-1,41,39,40],[-1,-1,39],[38,-1,37],[-1,-1,37],[-1,37,-1,89,97,43],[97,-1,-1,43],[42,-1,41],[-1,96,-1,45],[44,-1,49],[-1,-1,47],[-1,46,-`
+`1,48,49],[-1,-1,47],[45,47,-1,50],[49,-1,51],[50,-1,-1,52],[51,-1,53,-1,-1],[-1,-1,-1,52],[-1,-1,-1,-1],[9,-1,57,-1],[64,63,57],[64,56,-1,-1,55],[-1,65,59,148,60,61],[64,-1,137,58],[-1,-1,-1,58],[-1,-1,58],[-1,-1,-1,-1],[-1,56,64,65,-1],[56,57,-1,59,63],[58,-1,-1,63],[-1,148,-1,67],[66,-1,-1,-1,68],[67,-1,69],[68,-1,-1,-1,70],[69,-1,71,72],[202,-1,70],[70,-1,73],[72,-1,74,-1,-1],[-1,-1,76,-1,73],[-1,-1,-1,-1,76],[-1,75,-1,74],[10,-1,78],[77,-1,-1,123,79],[78,123,122,80],[79,122,-1,81],[80,-1,82,`
+`-1,10],[-1,81,-1,84],[-1,-1,84],[82,83,86,87],[-1,118,-1,86],[85,-1,84],[-1,88,84],[-1,87,-1,20],[41,-1,-1,90],[89,-1,92],[-1,34,92],[90,91,93,-1,94],[-1,-1,92],[92,-1,95,-1,96],[-1,-1,94],[94,-1,44,-1,97],[96,-1,42,41],[-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,-1,-1],[0,-1,102,152,103,104],[-1,-1,101],[174,-1,7,101],[101,7,-1,105],[-1,0,104],[-1,-1,-1,-1,-1],[-1,-1,-1,-1,108],[107,-1,109],[108,-1,-1,-1],[-1,-1,-1],[-1,-1,-1,-1],[-1,201,-1,113],[112,-1,116],[-1,205,223,115],[116,-1,114,224,-1,-1]`
+`,[113,115,-1,117],[116,-1,118,119],[85,-1,117],[117,-1,120],[119,-1,-1,121],[120,-1,122,123],[80,79,121],[121,79,78,-1],[-1,-1,-1,-1,-1,125],[-1,-1,124],[-1,-1,-1,-1],[-1,-1,136,-1],[-1,-1,-1,130],[-1,-1,-1,-1,130],[128,129,-1,131],[130,-1,-1,134,135],[-1,-1,-1,133,134],[136,-1,132],[-1,132,-1,131],[131,-1,136],[135,-1,133,127],[59,-1,-1,-1,138],[137,-1,150],[-1,-1,150],[-1,-1,149],[-1,-1,142],[141,-1,147,-1,148,149],[-1,-1,146],[-1,-1,-1,-1,145],[-1,144,146],[143,145,-1,147],[-1,146,-1,142],[66`
+`,-1,58,142],[140,142,150],[139,149,138],[-1,-1,-1],[101,-1,-1,339,-1,153],[152,-1,154],[153,-1,155,341,-1,156],[-1,342,154],[154,-1,-1,157],[156,-1,158],[157,-1,307,159],[158,314,320,-1,160],[159,-1,161,162],[-1,-1,160],[160,-1,163,164],[-1,-1,162],[162,-1,165],[164,-1,166],[165,-1,168,-1,169],[-1,-1,-1,168],[167,-1,166],[166,-1,170],[169,-1,-1,171,172],[-1,-1,170],[170,-1,-1,173],[172,-1,-1,174],[-1,103,173],[-1,207,176,188],[207,-1,175],[-1,206,178],[-1,177,-1,-1,179],[-1,178,180],[-1,179,181,`
+`182,183,185],[-1,-1,180],[-1,-1,180],[-1,263,180],[262,-1,185],[180,184,186],[185,-1,187],[186,-1,-1,188],[175,187,-1,189],[188,-1,-1,191],[-1,-1,191],[189,190,-1,192],[191,-1,215,193,194],[214,-1,192],[192,-1,195],[194,-1,196],[195,-1,197,199],[-1,212,-1,196],[-1,-1,199],[196,198,200],[199,-1,26,-1,-1],[112,-1,202,-1,203],[-1,71,-1,201],[201,-1,-1,-1,204],[203,-1,205,-1],[223,114,-1,204],[-1,177,-1,211,-1],[-1,176,175,-1,208],[-1,207,210,211],[-1,-1,-1,210],[209,-1,208],[208,-1,206],[-1,197,-1,`
+`213],[212,-1,214],[213,-1,193,220,221],[192,-1,217],[-1,-1,217],[215,216,-1,218,222,219],[-1,-1,-1,217],[217,-1,220],[219,-1,214],[214,-1,222],[221,-1,217,-1],[114,205,-1,224],[223,235,-1,115],[-1,-1,-1,226],[225,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,-1,229,-1,-1],[-1,-1,-1,228],[-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1],[224,-1,-1,-1,236],[235,-1,238],[-1,-1,238],[236,237,244,245,246],[298,-1,244],[-1,-1,242,-1,243],[-1,-1,-1,242],[241,-1,240],[-1,240,-1,244],[23`
+`9,243,-1,238],[-1,-1,238],[238,-1,-1,-1],[-1,-1,-1,248],[-1,-1,247],[-1,-1,-1,-1,251],[-1,-1,251],[249,250,-1,252,-1],[-1,-1,251],[-1,-1,-1,258,-1],[-1,-1,-1,255],[-1,254,256],[-1,255,-1,257],[-1,256,-1,258],[257,-1,-1,-1,253],[-1,-1,-1,-1,-1],[-1,-1,-1,-1,261],[260,-1,-1,-1,-1,-1],[-1,184,263],[262,183,-1,268],[-1,-1,265],[264,-1,-1,267],[-1,-1,267],[265,266,268],[263,267,-1,269],[268,-1,-1,272,270],[269,272,271,-1],[-1,270,274,-1],[270,269,-1,273],[272,-1,333,274],[271,273,335,275],[274,332,-1`
+`,276],[275,-1,332,332,331,289],[-1,356,-1,279,-1,278],[277,-1,286],[277,-1,355,280],[-1,279,-1,-1,281],[280,-1,282,284],[-1,354,-1,281],[-1,-1,284],[281,283,285,-1],[284,-1,286],[278,285,287,292,-1,288],[-1,-1,286],[286,-1,289],[276,288,290],[289,-1,291],[290,-1,293,294],[-1,286,-1,293],[292,-1,291],[291,-1,296,-1],[-1,-1,-1,296],[295,-1,294],[-1,-1,-1,-1,-1,-1],[-1,239,-1,-1,-1,299],[298,-1,302,-1],[-1,324,330,301],[300,330,-1,-1,-1,302],[301,-1,299],[-1,-1,-1,305],[-1,-1,305],[303,304,-1,-1,-1`
+`],[-1,-1,-1,-1],[158,-1,-1,-1,308],[307,-1,313],[-1,-1,315,310],[-1,309,311],[310,-1,312],[-1,311,313],[308,312,-1,314],[313,-1,316,319,159],[-1,-1,309,316],[-1,315,-1,314],[-1,-1,319],[-1,-1,319],[317,318,320,314],[-1,159,319],[-1,-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1,-1],[300,-1,-1,325],[324,-1,-1,326],[325,-1,327],[326,-1,359,328,329],[-1,-1,327],[327,-1,-1,330],[329,-1,301,300],[-1,347,349,-1,276,332],[331,276,276,-1,275,336],[273,-1,-1,334],[333,-1,-1,335],[274,334,336],[335,-1,332],[-1,-`
+`1,-1,-1,-1,338],[-1,-1,337],[152,-1,340],[339,-1,-1,-1,343,342],[345,-1,154,342],[340,341,155,-1],[344,340,-1,-1,-1,362],[343,-1,-1,345],[-1,341,344],[-1,-1,-1,-1],[331,-1,-1,348],[347,-1,350,349],[331,348,351,422,-1,352],[348,-1,351],[350,-1,419,349],[349,-1,353],[352,-1,-1,356,-1],[-1,282,-1,355],[354,-1,279,356],[355,-1,277,-1,353],[-1,-1,-1,-1],[-1,427,-1,359],[358,-1,-1,327,-1,360],[359,-1,-1,-1],[-1,-1,-1,362],[343,361,-1,363],[-1,362,-1,-1,426],[367,-1,-1,369,387,-1],[-1,-1,366],[365,-1,-`
+`1,-1,367],[366,-1,-1,-1,-1,364],[-1,-1,-1,-1],[364,-1,371],[-1,-1,371],[369,370,-1,445,-1,372],[371,-1,373],[372,-1,376],[-1,-1,-1,-1,375],[-1,374,376],[373,375,-1,483,-1,377],[376,-1,378],[377,-1,379],[378,-1,381],[-1,474,381],[379,380,480,-1,382],[381,-1,383],[382,-1,384,385],[420,-1,383],[383,-1,-1,386],[385,-1,387],[386,-1,-1,-1,364],[-1,408,390],[406,-1,390],[388,389,391],[390,-1,411,392,393],[-1,391,412,-1],[391,-1,-1,-1],[-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1,396],[395,-1,-1,397],[396,-1,398`
+`,399,400],[-1,-1,397],[-1,-1,-1,397],[-1,-1,397],[-1,434,433,-1,408],[-1,-1,403],[402,-1,405],[-1,-1,405],[403,404,-1,406],[405,-1,389,407],[-1,406,408],[401,407,388,409,410],[-1,-1,408],[-1,-1,408],[391,-1,-1,412],[392,411,-1,413],[412,-1,491,414,415],[498,-1,-1,413],[413,-1,416],[415,-1,417],[416,-1,418,419],[-1,424,417],[423,422,351,417],[-1,384,-1,437,-1,421],[420,-1,-1,-1],[349,419,423],[422,419,424,-1,-1],[418,-1,423],[-1,-1,426],[-1,363,425,-1,465],[358,-1,437,428],[427,437,-1,429],[428,-`
+`1,431],[-1,-1,431],[429,430,-1,432],[431,-1,434,-1],[-1,-1,401,434],[433,401,-1,435,432],[-1,-1,434],[-1,-1,-1,-1],[-1,420,438,-1,428,427],[-1,-1,437],[-1,-1,-1,-1],[-1,459,442],[459,462,-1,442],[440,441,-1,-1,443],[442,-1,-1,444],[443,-1,445,446],[371,-1,444],[444,-1,-1,447],[-1,-1,446],[-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,451],[452,450,-1,-1,-1],[451,-1,454,455],[-1,-1,-1,-1,454],[453,-1,452],[-1,452,-1,456],[-1,455,-1,-1,-1],[-1,-1,-1,-1,458],[457,-1,-1,-1,-1],[440,-1,462,441],[-1,462,506`
+`,-1],[-1,-1,462],[460,461,441,459],[-1,-1,-1,-1],[-1,-1,-1,-1,-1,-1],[426,-1,469,468,-1],[-1,-1,-1,-1],[502,468,-1,-1,501],[467,502,-1,465,469],[468,465,-1,470,-1],[-1,-1,-1,469],[-1,-1,472,473],[-1,-1,471],[-1,-1,471],[380,-1,475],[474,-1,487,479,480],[477,-1,-1,521,-1,489],[-1,476,490,478],[477,490,-1,479],[-1,478,-1,475],[-1,381,475],[-1,-1,482,-1],[-1,-1,-1,481],[376,-1,484],[483,-1,485,-1,486],[519,-1,484],[484,-1,487,-1],[475,-1,486],[-1,-1,-1,-1],[476,-1,-1,-1,490],[489,-1,478,477],[413,-`
+`1,493],[-1,-1,493],[491,492,-1,494],[493,-1,495,512,-1,497],[508,512,494],[-1,-1,497],[494,496,498],[-1,414,497],[-1,-1,-1,-1,-1,500],[499,-1,-1,-1],[467,-1,-1,-1,502],[501,-1,503,-1,468,467],[-1,502,-1,504,505],[-1,-1,503],[503,-1,-1,506],[505,-1,553,507,-1,460],[-1,506,553,524,-1],[495,-1,597,510],[597,596,-1,510],[508,509,-1,511,-1,512],[513,-1,510],[494,495,510],[-1,511,-1,-1,514],[513,-1,517,-1],[-1,-1,517],[-1,-1,-1,517],[515,516,514],[-1,544,550,551,-1,519],[518,-1,-1,485],[521,-1,559,565`
+`,-1],[520,-1,476,-1,522,523],[-1,-1,521],[-1,-1,521],[507,555,556,526,-1],[556,-1,-1,-1,526],[525,-1,524],[-1,558,-1,528],[527,-1,529],[528,-1,530,531,532],[-1,647,-1,529],[-1,-1,529],[529,-1,533],[532,-1,534],[533,-1,538,537],[624,-1,-1,536],[535,-1,538],[538,-1,543,539,540,534],[536,537,534],[542,-1,537],[537,-1,-1,541],[-1,-1,540],[-1,539,543],[542,537,-1,544],[543,-1,550,518,-1,545],[544,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,-1],[518,544,-1,551],[550,-1,552,-1,518],[-`
+`1,-1,551],[507,506,-1,554,648,555],[-1,649,553],[553,648,-1,556,524],[557,-1,525,524,555],[-1,556,-1,558,-1],[527,-1,557],[520,-1,-1,560],[559,-1,-1,-1,561,563],[-1,-1,560],[566,-1,563,-1],[560,562,564],[563,-1,565],[564,-1,566,-1,520],[-1,565,-1,562,-1,567],[566,-1,568],[567,-1,-1,-1,571,-1],[-1,573,570],[-1,569,572,-1,571],[-1,570,568],[-1,570,574],[569,-1,-1,574],[572,573,-1,575,577,576],[-1,578,574],[-1,-1,574],[-1,574,579],[575,-1,579],[591,-1,577,578,-1,637],[637,-1,590],[-1,-1,587],[-1,64`
+`6,645,583],[-1,582,-1,584],[-1,583,-1,585,586],[-1,-1,584],[-1,584,-1,587],[581,586,-1,588],[587,-1,-1,589],[-1,588,-1,590],[580,589,-1,591],[590,-1,579],[-1,-1,593,594],[-1,-1,-1,592],[592,-1,-1,-1,595],[594,-1,-1,-1,-1],[-1,671,678,-1,509,597],[596,509,508,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,628,-1,664,669,601],[600,669,603],[603,669,666,-1],[601,602,-1,604],[603,-1,-1,-1,605],[604,-1,606],[605,-1,608,-1],[-1,-1,608],[607,-1,-1,606],[-1,-1,-1,-1,-1],[-1,770,-1,611],[610,-1,612],[611,-1,623,`
+`-1],[-1,-1,615,614],[613,-1,616,619],[-1,613,-1,616],[-1,615,-1,-1,614],[-1,-1,-1,618],[617,-1,619],[614,-1,618,620,621,-1],[-1,-1,619],[619,-1,622],[621,-1,623],[-1,622,-1,612],[-1,535,-1,-1,-1,625],[624,-1,-1,-1,-1,626],[625,-1,-1,627],[626,-1,-1,629],[-1,600,629],[627,628,-1,-1,630],[629,-1,631,632],[-1,-1,630],[630,-1,633,634],[-1,-1,632],[632,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,580,579,-1,638],[637,-1,639],[638,-1,-1,641],[-1,694,641],[639,640,693,-1,642],[641,-1,-1,644,645],[-1`
+`,-1,644],[643,-1,642],[-1,642,-1,582,646],[582,-1,-1,645],[-1,530,-1,656,651],[555,553,650],[554,-1,-1,650],[648,649,-1,-1,654,655],[647,-1,652],[-1,651,653],[652,-1,654],[-1,653,650],[650,-1,656],[655,-1,647],[-1,-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,661,-1],[-1,-1,-1,-1,-1,661],[660,-1,-1,659],[-1,-1,-1,-1],[-1,-1,-1,-1],[600,-1,665],[664,-1,-1,666],[665,-1,668,602,669],[-1,706,668],[-1,667,705,666],[666,602,601,600],[-1,-1,-1,-1,-1],[596,-1,673],[-1,-1,673],[671,672,713,674],[673,713,714,675],`
+`[674,714,-1,676],[675,-1,-1,-1,677],[676,-1,-1,678],[-1,596,677],[-1,-1,-1,-1],[-1,-1,682,-1,683],[684,-1,682],[-1,681,-1,680],[-1,680,-1,684],[-1,683,-1,686,681],[686,-1,-1,-1,-1],[685,-1,684],[-1,717,722,-1,-1,689],[-1,-1,689],[687,688,690],[689,-1,691],[690,-1,693],[-1,-1,693],[691,692,641,694],[693,640,-1,695,-1,-1],[-1,-1,694],[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,711,699],[698,711,-1,700],[699,-1,745,-1,701],[700,-1,702,703,704,705],[-1,-1,701],[-1,-1,-1,701],[-1,-1,701],[701,-1,668,706],[705,66`
+`7,-1,-1],[-1,-1,708,-1],[707,-1,-1,709],[708,-1,-1,-1],[-1,-1,-1,-1],[-1,699,698,-1,712],[711,-1,-1,-1],[673,-1,714,674],[768,-1,675,674,713],[-1,-1,-1,-1,-1],[-1,-1,-1,-1],[687,-1,731,-1,-1,719],[-1,775,719],[717,718,-1,720],[719,-1,721],[720,-1,724,723,-1,722],[-1,687,721],[-1,721,727],[721,-1,-1,726],[-1,774,726],[724,725,774,727],[723,726,-1,728],[727,-1,-1,-1,729],[728,-1,730,-1,-1,-1],[-1,-1,729],[-1,717,-1,732],[731,-1,734],[-1,-1,734],[732,733,736,-1],[-1,-1,-1,736],[735,-1,734],[-1,-1,-`
+`1,-1],[-1,752,-1,740,-1],[741,-1,740],[739,-1,-1,-1,738],[-1,739,-1,-1,-1,742],[741,-1,776,743,-1,-1],[-1,742,778,-1,769,-1],[-1,-1,-1,-1],[700,-1,784,746],[745,784,783,-1,747],[746,-1,-1,-1,748,-1],[750,749,-1,747],[-1,748,750],[749,748,-1,751],[750,-1,-1,752,-1,-1],[-1,751,-1,-1,738,-1],[-1,769,759],[769,-1,755],[754,-1,758,759],[-1,777,-1,757],[-1,756,758],[-1,757,755],[753,755,760,761],[-1,-1,759],[759,-1,762,763],[-1,-1,761],[761,-1,764],[763,-1,765],[764,-1,766,767,768],[788,-1,765],[-1,-1`
+`,765],[-1,714,765],[-1,743,-1,754,753],[-1,610,-1,772],[-1,788,772],[770,771,-1,-1,773],[772,-1,774,775],[726,725,-1,773],[773,-1,-1,718],[778,742,-1,777,-1],[-1,756,-1,776],[776,-1,743],[-1,786,-1,781],[-1,793,-1,781],[779,-1,780,-1,-1,782],[781,-1,783,784],[-1,746,782],[746,745,782],[-1,786,-1,-1,-1,-1],[-1,785,-1,779],[-1,-1,-1,-1],[-1,766,-1,771,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1],[-1,-1,-1,-1,-1],[-1,-1,-1,-1],[-1,780,-1,-1],[-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1]]},"links":[{"PolyA":540,"PolyB":5`
+`46,"PosA":{"x":155.97560975609755,"y":58.78048780487805,"z":8},"PosB":{"x":155,"y":60,"z":53},"cost":2.3426064283290846,"type":1},{"PolyA":647,"PolyB":660,"PosA":{"x":715.6787330316743,"y":-151.9004524886878,"z":0.171945701357466},"PosB":{"x":715,"y":-150,"z":35},"cost":3.02702757298344,"type":1},{"PolyA":28,"PolyB":40,"PosA":{"x":-2020,"y":2500,"z":67},"PosB":{"x":-2017.8823529411766,"y":2500.470588235294,"z":30.36470588235294},"cost":3.253956867279695,"type":1},{"PolyA":83,"PolyB":124,"PosA":{`
+`"x":-1789,"y":1317,"z":37},"PosB":{"x":-1790,"y":1315,"z":70},"cost":3.3541019662496847,"type":1},{"PolyA":107,"PolyB":198,"PosA":{"x":-1715,"y":2430,"z":65},"PosB":{"x":-1714.4117647058824,"y":2427.6470588235293,"z":10.470588235294118},"cost":3.63803437554516,"type":1},{"PolyA":724,"PolyB":790,"PosA":{"x":1333.6764705882354,"y":2857.205882352941,"z":116.1470588235294},"PosB":{"x":1335,"y":2855,"z":159},"cost":3.858718165706157,"type":1},{"PolyA":220,"PolyB":230,"PosA":{"x":-1580.9049773755655,"`
+`y":2717.4660633484164,"z":6.895927601809955},"PosB":{"x":-1580,"y":2720,"z":64},"cost":4.036036763977737,"type":1},{"PolyA":53,"PolyB":99,"PosA":{"x":-2000.754716981132,"y":3057.6415094339623,"z":32.735849056603776},"PosB":{"x":-2000,"y":3055,"z":71},"cost":4.120816918460671,"type":1},{"PolyA":93,"PolyB":151,"PosA":{"x":-1759.245283018868,"y":2772.3584905660377,"z":33.735849056603776},"PosB":{"x":-1760,"y":2775,"z":77},"cost":4.120816918460671,"type":1},{"PolyA":694,"PolyB":715,"PosA":{"x":1007.`
+`0524017467249,"y":2520.3930131004367,"z":96},"PosB":{"x":1010,"y":2520,"z":152},"cost":4.460525553071923,"type":1},{"PolyA":141,"PolyB":249,"PosA":{"x":-1463,"y":-36,"z":8},"PosB":{"x":-1460,"y":-35,"z":69},"cost":4.743416490252569,"type":1},{"PolyA":226,"PolyB":229,"PosA":{"x":-1605,"y":1830,"z":50},"PosB":{"x":-1604,"y":1833,"z":49.8},"cost":4.743416490252569,"type":0},{"PolyA":696,"PolyB":706,"PosA":{"x":840,"y":795,"z":46},"PosB":{"x":843,"y":794,"z":0},"cost":4.743416490252569,"type":1},{"P`
+`olyA":380,"PolyB":481,"PosA":{"x":-236.76470588235293,"y":577.9411764705883,"z":0},"PosB":{"x":-235,"y":575,"z":44},"cost":5.144957554275324,"type":1},{"PolyA":7,"PolyB":98,"PosA":{"x":-1852.5,"y":-862.5,"z":115.625},"PosB":{"x":-1855,"y":-865,"z":150},"cost":5.303300858899107,"type":1},{"PolyA":488,"PolyB":502,"PosA":{"x":5,"y":-635,"z":61},"PosB":{"x":8.448275862068966,"y":-636.3793103448276,"z":4.655172413793103},"cost":5.570860145311542,"type":1},{"PolyA":35,"PolyB":110,"PosA":{"x":-1826.551`
+`724137931,"y":2628.6206896551726,"z":33},"PosB":{"x":-1830,"y":2630,"z":78},"cost":5.570860145311619,"type":1},{"PolyA":89,"PolyB":126,"PosA":{"x":-1828.448275862069,"y":2681.3793103448274,"z":33},"PosB":{"x":-1825,"y":2680,"z":77},"cost":5.570860145311619,"type":1},{"PolyA":269,"PolyB":322,"PosA":{"x":-1108.6206896551723,"y":2156.551724137931,"z":-4.793103448275862},"PosB":{"x":-1105,"y":2155,"z":43},"cost":5.908789478687471,"type":1},{"PolyA":206,"PolyB":227,"PosA":{"x":-1597.2413793103449,"y"`
+`:1753.103448275862,"z":1.8275862068965516},"PosB":{"x":-1600,"y":1750,"z":60},"cost":6.228410989030468,"type":1},{"PolyA":642,"PolyB":697,"PosA":{"x":821.8141592920354,"y":2717.787610619469,"z":96},"PosB":{"x":825,"y":2715,"z":120},"cost":6.349865861589281,"type":1},{"PolyA":62,"PolyB":66,"PosA":{"x":-1790,"y":540,"z":59},"PosB":{"x":-1786,"y":542,"z":32},"cost":6.708203932499369,"type":1},{"PolyA":166,"PolyB":233,"PosA":{"x":-1536,"y":-212,"z":129.26666666666668},"PosB":{"x":-1540,"y":-210,"z":`
+`175},"cost":6.708203932499369,"type":1},{"PolyA":303,"PolyB":330,"PosA":{"x":-1065,"y":1455,"z":-76},"PosB":{"x":-1061,"y":1453,"z":-112},"cost":6.708203932499369,"type":1},{"PolyA":692,"PolyB":709,"PosA":{"x":866,"y":2693,"z":96},"PosB":{"x":870,"y":2695,"z":141},"cost":6.708203932499369,"type":1},{"PolyA":502,"PolyB":598,"PosA":{"x":346.55737704918033,"y":-627.8688524590164,"z":2.262295081967213},"PosB":{"x":350,"y":-625,"z":48},"cost":6.721936196476996,"type":1},{"PolyA":142,"PolyB":234,"PosA`
+`":{"x":-1539.5,"y":513.5,"z":28.6},"PosB":{"x":-1535,"y":515,"z":62},"cost":7.115124735378854,"type":1},{"PolyA":724,"PolyB":789,"PosA":{"x":1316.5,"y":2859.5,"z":120.5},"PosB":{"x":1315,"y":2855,"z":165},"cost":7.115124735378854,"type":1},{"PolyA":228,"PolyB":232,"PosA":{"x":-1580,"y":1900,"z":52},"PosB":{"x":-1575.7961783439491,"y":1902.2929936305732,"z":43.5796178343949},"cost":7.182781960208436,"type":0},{"PolyA":176,"PolyB":225,"PosA":{"x":-1679.5205479452054,"y":1847.945205479452,"z":0},"P`
+`osB":{"x":-1675,"y":1850,"z":37},"cost":7.448452997421284,"type":1},{"PolyA":62,"PolyB":75,"PosA":{"x":-2040,"y":540,"z":60.886792452830186},"PosB":{"x":-2040,"y":545,"z":35},"cost":7.5,"type":1},{"PolyA":62,"PolyB":106,"PosA":{"x":-1850,"y":540,"z":59.45283018867924},"PosB":{"x":-1850,"y":545,"z":106},"cost":7.5,"type":1},{"PolyA":95,"PolyB":107,"PosA":{"x":-1750,"y":2835,"z":33.53333333333333},"PosB":{"x":-1745,"y":2835,"z":65},"cost":7.5,"type":1},{"PolyA":129,"PolyB":173,"PosA":{"x":-1755,"y`
+`":-315,"z":157.875},"PosB":{"x":-1755,"y":-320,"z":129},"cost":7.5,"type":1},{"PolyA":156,"PolyB":337,"PosA":{"x":-1040,"y":-620,"z":127.125},"PosB":{"x":-1040,"y":-615,"z":166},"cost":7.5,"type":1},{"PolyA":269,"PolyB":306,"PosA":{"x":-1160,"y":2130,"z":3},"PosB":{"x":-1160,"y":2125,"z":39},"cost":7.5,"type":1},{"PolyA":367,"PolyB":368,"PosA":{"x":-665,"y":245,"z":8},"PosB":{"x":-665,"y":240,"z":56},"cost":7.5,"type":1},{"PolyA":525,"PolyB":547,"PosA":{"x":140,"y":-165,"z":8.875},"PosB":{"x":14`
+`0,"y":-160,"z":44},"cost":7.5,"type":1},{"PolyA":657,"PolyB":658,"PosA":{"x":605,"y":2330,"z":-20},"PosB":{"x":605,"y":2335,"z":28},"cost":7.5,"type":1},{"PolyA":206,"PolyB":232,"PosA":{"x":-1497.1611001964636,"y":1880.4911591355599,"z":0},"PosB":{"x":-1500,"y":1885,"z":25},"cost":7.992199045292164,"type":1},{"PolyA":239,"PolyB":240,"PosA":{"x":-1189.551724137931,"y":1149.6206896551723,"z":-12.96551724137931},"PosB":{"x":-1190,"y":1155,"z":32},"cost":8.096934285739788,"type":1},{"PolyA":623,"Pol`
+`yB":733,"PosA":{"x":1054.488950276243,"y":2339.709944751381,"z":125},"PosB":{"x":1060,"y":2340,"z":99},"cost":8.278016216835386,"type":1},{"PolyA":44,"PolyB":54,"PosA":{"x":-2025.3846153846155,"y":2893.076923076923,"z":32.53846153846154},"PosB":{"x":-2030,"y":2890,"z":89},"cost":8.320502943378152,"type":1},{"PolyA":588,"PolyB":594,"PosA":{"x":364.61538461538464,"y":2706.923076923077,"z":104},"PosB":{"x":360,"y":2710,"z":130},"cost":8.320502943378296,"type":1},{"PolyA":128,"PolyB":174,"PosA":{"x"`
+`:-1765,"y":-640,"z":158},"PosB":{"x":-1761.923076923077,"y":-644.6153846153846,"z":128.46153846153845},"cost":8.320502943378484,"type":1},{"PolyA":682,"PolyB":685,"PosA":{"x":754.6724137931035,"y":1109.4310344827586,"z":118},"PosB":{"x":755,"y":1115,"z":150},"cost":8.367888128084545,"type":1},{"PolyA":571,"PolyB":599,"PosA":{"x":434.62389380530976,"y":1964.358407079646,"z":96},"PosB":{"x":435,"y":1970,"z":132},"cost":8.48117384178122,"type":1},{"PolyA":704,"PolyB":710,"PosA":{"x":970.40609137055`
+`83,"y":1139.3147208121827,"z":1},"PosB":{"x":970,"y":1145,"z":36},"cost":8.549645998549193,"type":1},{"PolyA":582,"PolyB":609,"PosA":{"x":544.8,"y":2623.6,"z":95},"PosB":{"x":540,"y":2620,"z":131},"cost":8.999999999999863,"type":1},{"PolyA":757,"PolyB":794,"PosA":{"x":1709.8,"y":2138.6,"z":2},"PosB":{"x":1705,"y":2135,"z":34},"cost":8.999999999999863,"type":1},{"PolyA":170,"PolyB":231,"PosA":{"x":-1573.5294117647059,"y":-190.88235294117646,"z":130},"PosB":{"x":-1575,"y":-185,"z":175},"cost":9.09`
+`5085938862486,"type":1},{"PolyA":216,"PolyB":247,"PosA":{"x":-1480.9,"y":2661.3,"z":5.91},"PosB":{"x":-1480,"y":2655,"z":60},"cost":9.545941546018682,"type":1},{"PolyA":782,"PolyB":795,"PosA":{"x":1723.5135135135135,"y":1003.918918918919,"z":0.2972972972972973},"PosB":{"x":1730,"y":1005,"z":64},"cost":9.863939238321382,"type":1},{"PolyA":247,"PolyB":253,"PosA":{"x":-1461.4864864864865,"y":2646.0810810810813,"z":60},"PosB":{"x":-1455,"y":2645,"z":107},"cost":9.863939238321437,"type":1},{"PolyA":6`
+`36,"PolyB":658,"PosA":{"x":625,"y":2365,"z":65},"PosB":{"x":626.081081081081,"y":2358.5135135135133,"z":28},"cost":9.863939238321716,"type":1},{"PolyA":249,"PolyB":259,"PosA":{"x":-1405,"y":30,"z":61},"PosB":{"x":-1399,"y":33,"z":20},"cost":10.062305898749054,"type":1},{"PolyA":450,"PolyB":456,"PosA":{"x":-436.04046242774564,"y":2468.2369942196533,"z":-34.12138728323699},"PosB":{"x":-435,"y":2475,"z":-10},"cost":10.263859937140843,"type":1},{"PolyA":767,"PolyB":787,"PosA":{"x":1311.3461538461538`
+`,"y":2358.269230769231,"z":23.653846153846153},"PosB":{"x":1310,"y":2365,"z":75},"cost":10.296097094754394,"type":1},{"PolyA":313,"PolyB":315,"PosA":{"x":-996.3461538461538,"y":-206.73076923076923,"z":128.34615384615384},"PosB":{"x":-995,"y":-200,"z":145},"cost":10.296097094754645,"type":1},{"PolyA":448,"PolyB":449,"PosA":{"x":-468.65384615384613,"y":-151.73076923076923,"z":69},"PosB":{"x":-470,"y":-145,"z":107},"cost":10.29609709475466,"type":1},{"PolyA":245,"PolyB":261,"PosA":{"x":-1331.730769`
+`2307693,"y":1183.6538461538462,"z":33},"PosB":{"x":-1325,"y":1185,"z":69},"cost":10.296097094754728,"type":1},{"PolyA":470,"PolyB":549,"PosA":{"x":163.65384615384616,"y":-1156.7307692307693,"z":0},"PosB":{"x":165,"y":-1150,"z":40},"cost":10.296097094754735,"type":1},{"PolyA":254,"PolyB":296,"PosA":{"x":-1230,"y":2600,"z":101},"PosB":{"x":-1225,"y":2595,"z":65},"cost":10.606601717798213,"type":1},{"PolyA":277,"PolyB":353,"PosA":{"x":-830,"y":2420,"z":-67},"PosB":{"x":-825,"y":2415,"z":-81},"cost"`
+`:10.606601717798213,"type":1},{"PolyA":613,"PolyB":640,"PosA":{"x":790,"y":2460,"z":138},"PosB":{"x":795,"y":2465,"z":96},"cost":10.606601717798213,"type":1},{"PolyA":697,"PolyB":708,"PosA":{"x":845,"y":2730,"z":120},"PosB":{"x":850,"y":2735,"z":142},"cost":10.606601717798213,"type":1},{"PolyA":180,"PolyB":228,"PosA":{"x":-1633.7232059645853,"y":1891.1323392357874,"z":-0.3662628145386766},"PosB":{"x":-1630,"y":1885,"z":45},"cost":10.761164254130591,"type":1},{"PolyA":453,"PolyB":458,"PosA":{"x":`
+`-330,"y":2500,"z":-18},"PosB":{"x":-327.92452830188677,"y":2507.264150943396,"z":26},"cost":11.332246525766656,"type":1},{"PolyA":663,"PolyB":679,"PosA":{"x":707.3360655737705,"y":2364.3032786885246,"z":4.467213114754099},"PosB":{"x":715,"y":2365,"z":22},"cost":11.543307620421096,"type":1},{"PolyA":744,"PolyB":779,"PosA":{"x":1275,"y":770,"z":30},"PosB":{"x":1282.5,"y":772.5,"z":-8.5},"cost":11.858541225631422,"type":1},{"PolyA":109,"PolyB":111,"PosA":{"x":-1764.3908629441623,"y":2433.5279187817`
+`26,"z":64.29441624365482},"PosB":{"x":-1765,"y":2425,"z":102},"cost":12.824468997823809,"type":1},{"PolyA":635,"PolyB":646,"PosA":{"x":555,"y":2580,"z":137},"PosB":{"x":559.4117647058823,"y":2587.3529411764707,"z":96},"cost":12.86239388568831,"type":1},{"PolyA":610,"PolyB":737,"PosA":{"x":1263.7462039045554,"y":2540.182212581345,"z":125.74924078091107},"PosB":{"x":1255,"y":2540,"z":170},"cost":13.122152619556584,"type":1},{"PolyA":575,"PolyB":617,"PosA":{"x":521,"y":2433,"z":96},"PosB":{"x":525,`
+`"y":2425,"z":125},"cost":13.416407864998739,"type":1},{"PolyA":779,"PolyB":791,"PosA":{"x":1587,"y":774,"z":-9.6},"PosB":{"x":1595,"y":770,"z":53},"cost":13.416407864998739,"type":1},{"PolyA":547,"PolyB":548,"PosA":{"x":155.75862068965517,"y":-94.10344827586206,"z":44},"PosB":{"x":155,"y":-85,"z":79},"cost":13.702504175867087,"type":1},{"PolyA":423,"PolyB":450,"PosA":{"x":-430.98194945848377,"y":2445.916967509025,"z":-89.45415162454874},"PosB":{"x":-430,"y":2455,"z":-33},"cost":13.70393497996181`
+`5,"type":1},{"PolyA":161,"PolyB":162,"PosA":{"x":-1250,"y":-255,"z":128},"PosB":{"x":-1259,"y":-258,"z":149.6},"cost":14.230249470757707,"type":1},{"PolyA":353,"PolyB":394,"PosA":{"x":-595,"y":2545,"z":-84},"PosB":{"x":-585.4060913705583,"y":2544.3147208121827,"z":-21.593908629441625},"cost":14.427527622551775,"type":1},{"PolyA":660,"PolyB":661,"PosA":{"x":710,"y":-70,"z":41},"PosB":{"x":700.3846153846154,"y":-68.07692307692308,"z":56.38461538461539},"cost":14.708710135363841,"type":1},{"PolyA":`
+`436,"PolyB":456,"PosA":{"x":-489.84615384615387,"y":2486.230769230769,"z":-20},"PosB":{"x":-480,"y":2485,"z":1},"cost":14.884168150705012,"type":1},{"PolyA":620,"PolyB":636,"PosA":{"x":560,"y":2400,"z":124},"PosB":{"x":560.7647058823529,"y":2390.0588235294117,"z":65},"cost":14.955817282523803,"type":1},{"PolyA":439,"PolyB":465,"PosA":{"x":-340,"y":-640,"z":138},"PosB":{"x":-339.74375821287776,"y":-649.9934296977661,"z":85.93561103810777},"cost":14.995071463642294,"type":1},{"PolyA":99,"PolyB":10`
+`0,"PosA":{"x":-1930,"y":3030,"z":70.33333333333333},"PosB":{"x":-1920,"y":3030,"z":106},"cost":15,"type":1},{"PolyA":127,"PolyB":167,"PosA":{"x":-1420,"y":-150,"z":157},"PosB":{"x":-1420,"y":-160,"z":129.96666666666667},"cost":15,"type":1},{"PolyA":127,"PolyB":231,"PosA":{"x":-1575,"y":-150,"z":157},"PosB":{"x":-1575,"y":-160,"z":174},"cost":15,"type":1},{"PolyA":306,"PolyB":321,"PosA":{"x":-1135,"y":2065,"z":39},"PosB":{"x":-1125,"y":2065,"z":64},"cost":15,"type":1},{"PolyA":321,"PolyB":322,"Po`
+`sA":{"x":-1105,"y":2120,"z":65},"PosB":{"x":-1105,"y":2130,"z":43},"cost":15,"type":1},{"PolyA":383,"PolyB":473,"PosA":{"x":-265,"y":780,"z":-2.0454545454545454},"PosB":{"x":-255,"y":780,"z":29},"cost":15,"type":1},{"PolyA":439,"PolyB":466,"PosA":{"x":-340,"y":-635,"z":138.5},"PosB":{"x":-330,"y":-635,"z":101},"cost":15,"type":1},{"PolyA":466,"PolyB":488,"PosA":{"x":-190,"y":-635,"z":102},"PosB":{"x":-180,"y":-635,"z":61},"cost":15,"type":1},{"PolyA":471,"PolyB":478,"PosA":{"x":-230,"y":1525,"z"`
+`:30},"PosB":{"x":-220,"y":1525,"z":1},"cost":15,"type":1},{"PolyA":561,"PolyB":566,"PosA":{"x":420,"y":1650,"z":3},"PosB":{"x":410,"y":1650,"z":19},"cost":15,"type":1},{"PolyA":610,"PolyB":771,"PosA":{"x":1285,"y":2720,"z":124},"PosB":{"x":1295,"y":2720,"z":108},"cost":15,"type":1},{"PolyA":619,"PolyB":716,"PosA":{"x":1015,"y":2425,"z":125},"PosB":{"x":1015,"y":2435,"z":168},"cost":15,"type":1},{"PolyA":657,"PolyB":662,"PosA":{"x":635,"y":2305,"z":-12},"PosB":{"x":645,"y":2305,"z":-43},"cost":15`
+`,"type":1},{"PolyA":658,"PolyB":663,"PosA":{"x":635,"y":2335,"z":28},"PosB":{"x":645,"y":2335,"z":-1},"cost":15,"type":1},{"PolyA":662,"PolyB":663,"PosA":{"x":645,"y":2325,"z":-43},"PosB":{"x":645,"y":2335,"z":-1},"cost":15,"type":1},{"PolyA":711,"PolyB":744,"PosA":{"x":1240,"y":335,"z":9.495145631067961},"PosB":{"x":1250,"y":335,"z":29},"cost":15,"type":1},{"PolyA":791,"PolyB":792,"PosA":{"x":1620,"y":720,"z":53},"PosB":{"x":1620,"y":710,"z":76},"cost":15,"type":1},{"PolyA":792,"PolyB":793,"Pos`
+`A":{"x":1620,"y":335,"z":76},"PosB":{"x":1630,"y":335,"z":54.32258064516129},"cost":15,"type":1},{"PolyA":596,"PolyB":662,"PosA":{"x":710.2529668956902,"y":2289.8813241723924,"z":-63.47158026233604},"PosB":{"x":710,"y":2300,"z":-29},"cost":15.18275612982315,"type":1},{"PolyA":0,"PolyB":98,"PosA":{"x":-1860,"y":-945,"z":120},"PosB":{"x":-1859.71044045677,"y":-934.8654159869494,"z":151.2153344208809},"cost":15.208079601152022,"type":1},{"PolyA":464,"PolyB":472,"PosA":{"x":-265,"y":1470,"z":-29},"P`
+`osB":{"x":-254.52618453865335,"y":1470.5236907730673,"z":29.14214463840399},"cost":15.730349337326071,"type":1},{"PolyA":261,"PolyB":297,"PosA":{"x":-1285.7586206896551,"y":1240.896551724138,"z":69.15172413793104},"PosB":{"x":-1275,"y":1240,"z":104},"cost":16.19386857147921,"type":1},{"PolyA":715,"PolyB":716,"PosA":{"x":1067.6470588235295,"y":2505.5882352941176,"z":152},"PosB":{"x":1065,"y":2495,"z":168},"cost":16.371154689952395,"type":1},{"PolyA":465,"PolyB":466,"PosA":{"x":-189.71419185282522`
+`,"y":-646.146517739816,"z":48.23587385019711},"PosB":{"x":-190,"y":-635,"z":102},"cost":16.725272017139314,"type":1},{"PolyA":0,"PolyB":4,"PosA":{"x":-2120,"y":-1040,"z":128},"PosB":{"x":-2125,"y":-1050,"z":128},"cost":16.770509831248425,"type":0},{"PolyA":137,"PolyB":139,"PosA":{"x":-1490,"y":-105,"z":8},"PosB":{"x":-1480,"y":-110,"z":30},"cost":16.770509831248425,"type":1},{"PolyA":610,"PolyB":788,"PosA":{"x":1278.7871581450654,"y":2465.273483947681,"z":124.62128418549347},"PosB":{"x":1290,"y"`
+`:2465,"z":60},"cost":16.824264793748874,"type":1},{"PolyA":609,"PolyB":635,"PosA":{"x":545,"y":2590,"z":131},"PosB":{"x":549,"y":2578,"z":137},"cost":18.973665961010276,"type":0},{"PolyA":110,"PolyB":126,"PosA":{"x":-1835,"y":2655,"z":78},"PosB":{"x":-1825,"y":2665,"z":77},"cost":21.213203435596427,"type":0},{"PolyA":657,"PolyB":663,"PosA":{"x":635,"y":2325,"z":-12},"PosB":{"x":645,"y":2335,"z":-1},"cost":21.213203435596427,"type":0},{"PolyA":295,"PolyB":323,"PosA":{"x":-1110,"y":2710,"z":87},"P`
+`osB":{"x":-1095,"y":2710,"z":139},"cost":22.5,"type":1},{"PolyA":361,"PolyB":363,"PosA":{"x":-430,"y":-1030,"z":128},"PosB":{"x":-430,"y":-1015,"z":107.57894736842105},"cost":22.5,"type":1},{"PolyA":663,"PolyB":677,"PosA":{"x":710,"y":2335,"z":5},"PosB":{"x":725,"y":2335,"z":-55.09090909090909},"cost":22.5,"type":1},{"PolyA":107,"PolyB":151,"PosA":{"x":-1744.4622968778529,"y":2795.20996896111,"z":65},"PosB":{"x":-1760,"y":2795,"z":77},"cost":23.308682649870512,"type":0},{"PolyA":394,"PolyB":436,`
+`"PosA":{"x":-515.4,"y":2527.8,"z":6.12},"PosB":{"x":-500,"y":2525,"z":-20},"cost":23.478713763747805,"type":1},{"PolyA":144,"PolyB":259,"PosA":{"x":-1357.1951219512196,"y":85.2439024390244,"z":8},"PosB":{"x":-1370,"y":75,"z":20},"cost":24.5973674974554,"type":0},{"PolyA":0,"PolyB":104,"PosA":{"x":-1860,"y":-945,"z":120},"PosB":{"x":-1845,"y":-935,"z":120},"cost":27.04163456597992,"type":0},{"PolyA":372,"PolyB":463,"PosA":{"x":-332.4703087885986,"y":288.36104513064134,"z":0},"PosB":{"x":-320,"y":`
+`275,"z":3},"cost":27.414572559107604,"type":0},{"PolyA":231,"PolyB":233,"PosA":{"x":-1575,"y":-185,"z":175},"PosB":{"x":-1557,"y":-191,"z":175},"cost":28.460498941515414,"type":0},{"PolyA":789,"PolyB":790,"PosA":{"x":1315,"y":2835,"z":165},"PosB":{"x":1335,"y":2835,"z":159},"cost":30,"type":0},{"PolyA":394,"PolyB":395,"PosA":{"x":-585,"y":2550,"z":-22},"PosB":{"x":-580,"y":2570,"z":-85},"cost":30.923292192132458,"type":1},{"PolyA":353,"PolyB":395,"PosA":{"x":-605,"y":2570,"z":-84},"PosB":{"x":-5`
+`85,"y":2580,"z":-84.5},"cost":33.54101966249685,"type":0},{"PolyA":573,"PolyB":617,"PosA":{"x":520,"y":2065,"z":96},"PosB":{"x":544.9944320712694,"y":2065.3730512249444,"z":125.0011135857461},"cost":37.49582382091243,"type":1},{"PolyA":282,"PolyB":346,"PosA":{"x":-835,"y":2580,"z":-77.42857142857143},"PosB":{"x":-835,"y":2605,"z":-76},"cost":37.5,"type":0},{"PolyA":346,"PolyB":357,"PosA":{"x":-815,"y":2605,"z":-79},"PosB":{"x":-790,"y":2605,"z":-79},"cost":37.5,"type":0},{"PolyA":354,"PolyB":357`
+`,"PosA":{"x":-791.2443438914027,"y":2578.868778280543,"z":-82.22624434389141},"PosB":{"x":-790,"y":2605,"z":-79},"cost":39.24124826774542,"type":0},{"PolyA":293,"PolyB":295,"PosA":{"x":-1085,"y":2570,"z":32},"PosB":{"x":-1110,"y":2585,"z":46},"cost":43.73213921133976,"type":1},{"PolyA":365,"PolyB":366,"PosA":{"x":-725,"y":500,"z":9.894736842105264},"PosB":{"x":-725,"y":470,"z":24},"cost":45,"type":1},{"PolyA":396,"PolyB":455,"PosA":{"x":-390,"y":2545,"z":-86},"PosB":{"x":-390,"y":2515,"z":-21},"`
+`cost":45,"type":1}]}`;
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
        /**@type {{verts: {x: number;y: number;z: number;}[];tris:number[][];meshes: number[][];}} */
        this.meshdetail;
        /**@type {FunnelPath} */
        this.funnel;
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
        this.debugLoad(30);
    }
    debugLoad(duration = 5) {
        for (let pi = 0; pi < this.mesh.polys.length; pi++) {
            const poly = this.mesh.polys[pi];
            const color = {r:255,g:255,b:0};
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
        const data = {
            mesh: this.mesh,           // 包含 verts, polys, regions, neighbors
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
            Instance.Msg(`导航数据加载成功！多边形数量: ${this.mesh.polys.length}`);
            return true;
        } catch (e) {
            Instance.Msg(`加载导航数据失败: ${e}`);
            return false;
        }
    }
    init() {
        this.importNavData(new StaticData().Data);

        //构建A*寻路
        this.astar = new PolyGraphAStar(this.mesh,this.links);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers,this.links);
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

        const ans = this.funnel.build(polyPath, start, end);
        // 用 detail mesh 修正高度
        //for (const p of ans) {
        //    const poly = this.findContainingPoly(p);
        //    if (poly >= 0) {
        //        p.z = this.getHeightOnPolyDetail(poly, p.x, p.y);
        //    }
        //}
        this.debugDrawPolyPath(polyPath, 1 / 2);
        this.debugDrawPath(ans,1/2);
        return ans;
    }
    /**
     * @param {number} polyIndex
     * @param {number} x
     * @param {number} y
     */
    getHeightOnPolyDetail(polyIndex, x, y) {
        const [vbase, vcount, tbase, tcount] = this.meshdetail.meshes[polyIndex];

        for (let i = 0; i < tcount; i++) {
            const tri = this.meshdetail.tris[tbase + i];

            const a = this.meshdetail.verts[tri[0]];
            const b = this.meshdetail.verts[tri[1]];
            const c = this.meshdetail.verts[tri[2]];

            if (this.pointInTri2D(x, y, a, b, c)) {
                return this.baryZ(x, y, a, b, c);
            }
        }

        const center = this.polyCenter(polyIndex);
        return center.z;
    }
    /**
     * @param {number} pi
     */
    polyCenter(pi) {
        const poly = this.mesh.polys[pi];
        let x = 0, y = 0, z = 0;

        for (const vi of poly) {
            const v = this.mesh.verts[vi];
            x += v.x;
            y += v.y;
            z += v.z;
        }

        const n = poly.length;
        return { x: x / n, y: y / n, z: z / n };
    }
    /**
     * @param {number} px
     * @param {number} py
     * @param {{ x: number; y: number; z: number; }} a
     * @param {{ x: number; y: number; z: number; }} b
     * @param {{ x: number; y: number; z: number; }} c
     */
    pointInTri2D(px, py, a, b, c) {
        const area = (/** @type {{ x: number; y: number; z: number; }} */ p, /** @type {{ x: number; y: number; z: number; }} */ q, /** @type {{ x: number; y: number; }} */ r) =>
            (q.x - p.x) * (r.y - p.y) -
            (q.y - p.y) * (r.x - p.x);

        const p = { x: px, y: py };
        const ab = area(a, b, p);
        const bc = area(b, c, p);
        const ca = area(c, a, p);

        return ab >= 0 && bc >= 0 && ca >= 0;
    }
    /**
     * @param {number} px
     * @param {number} py
     * @param {{ x: number; y: number; z: number; }} a
     * @param {{ x: number; y: number; z: number; }} b
     * @param {{ x: number; y: number; z: number; }} c
     */
    baryZ(px, py, a, b, c) {
        const v0x = b.x - a.x, v0y = b.y - a.y;
        const v1x = c.x - a.x, v1y = c.y - a.y;
        const v2x = px - a.x, v2y = py - a.y;

        const d00 = v0x * v0x + v0y * v0y;
        const d01 = v0x * v1x + v0y * v1y;
        const d11 = v1x * v1x + v1y * v1y;
        const d20 = v2x * v0x + v2y * v0y;
        const d21 = v2x * v1x + v2y * v1y;

        const denom = d00 * d11 - d01 * d01;
        if (denom === 0) return a.z;

        const v = (d11 * d20 - d01 * d21) / denom;
        const w = (d00 * d21 - d01 * d20) / denom;
        const u = 1 - v - w;

        return u * a.z + v * b.z + w * c.z;
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
    /**
     * @param {{left:{x:number,y:number,z:number},right:{x:number,y:number,z:number}}[]} portals
     */
    debugDrawPortals(portals, duration = 10) {
        if (!portals) return;

        for (let i = 0; i < portals.length; i++) {
            const { left, right } = portals[i];

            Instance.DebugLine({
                start: { x: left.x, y: left.y, z: left.z },
                end: { x: right.x, y: right.y, z: right.z },
                color: { r: 0, g: 180, b: 255 },
                duration
            });

            Instance.DebugSphere({
                center: { x: left.x, y: left.y, z: left.z },
                radius: 5,
                color: { r: 0, g: 255, b: 255 },
                duration
            });
            Instance.DebugSphere({
                center: { x: right.x, y: right.y, z: right.z },
                radius: 5,
                color: { r: 255, g: 0, b: 255 },
                duration
            });
        }
    }
    /**
     * @param {{left:{x:number,y:number,z:number},right:{x:number,y:number,z:number}}[]} portals
     * @param {import("cs_script/point_script").Vector} start
     * @param {import("cs_script/point_script").Vector} end
     */
    debugDrawFunnel(portals, start, end, duration = 10) {
        let prevL = start;
        let prevR = start;

        for (const p of portals) {
            const l = p.left;
            const r = p.right;

            Instance.DebugLine({
                start: { x: prevL.x, y: prevL.y, z: prevL.z },
                end: { x: l.x, y: l.y, z: l.z },
                color: { r: 0, g: 255, b: 0 },
                duration
            });

            Instance.DebugLine({
                start: { x: prevR.x, y: prevR.y, z: prevR.z },
                end: { x: r.x, y: r.y, z: r.z },
                color: { r: 255, g: 0, b: 0 },
                duration
            });

            prevL = l;
            prevR = r;
        }

        Instance.DebugLine({
            start: { x: prevL.x, y: prevL.y, z: prevL.z },
            end: { x: end.x, y: end.y, z: end.z },
            color: { r: 0, g: 255, b: 0 },
            duration
        });

        Instance.DebugLine({
            start: { x: prevR.x, y: prevR.y, z: prevR.z },
            end: { x: end.x, y: end.y, z: end.z },
            color: { r: 255, g: 0, b: 0 },
            duration
        });
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
    pathfinder.findPath(start,end);
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
