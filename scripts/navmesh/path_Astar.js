import { Instance } from "cs_script/point_script";
import { ASTAR_HEURISTIC_SCALE, ASTAR_OPTIMIZATION_1, closestPointOnPoly, MAX_JUMP_HEIGHT, MESH_CELL_SIZE_Z } from "./path_const";
import { FunnelHeightFixer } from "./path_funnelheightfixer";

export class PolyGraphAStar {
    /**
     * @param {{verts: {x: number;y: number;z: number;}[];polys: number[][];regions: number[];neighbors: number[][];}} polys
     * @param {{PolyA: number;PolyB: number;PosA: import("cs_script/point_script").Vector;PosB: import("cs_script/point_script").Vector;cost: number;type: number;}[]} links
     * @param {FunnelHeightFixer} heightfixer
     */
    constructor(polys, links, heightfixer) {
        this.mesh = polys;
        this.polyCount = polys.polys.length;
        /**@type {Map<number,{PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number; type: number;}[]>} */
        this.links = new Map();
        this.heightfixer = heightfixer;
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
        //预计算距离
        if (ASTAR_OPTIMIZATION_1) {
            this.edgeWeights = new Array(this.polyCount);
            for (let i = 0; i < this.polyCount; i++) {
                this.edgeWeights[i] = new Float32Array(this.polyCount);
            }
            for (let i = 0; i < this.polyCount; i++) {
                for (let j = i + 1; j < this.polyCount; j++) {
                    this.edgeWeights[j][i] = this.edgeWeights[i][j] = this.distsqr(i, j);
                }
            }
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
        const startPoly = this.findNearestPoly(start);
        const endPoly = this.findNearestPoly(end);
        //Instance.Msg(startPoly.poly+"   "+endPoly.poly);
        if (startPoly.poly < 0 || endPoly.poly < 0) {
            Instance.Msg(`跑那里去了?`);
            return { start: startPoly.pos, end: endPoly.pos, path: [] };
        }

        if (startPoly == endPoly) {
            return { start: startPoly.pos, end: endPoly.pos, path: [{ id: endPoly.poly, mode: 1 }] };
        }
        return { start: startPoly.pos, end: endPoly.pos, path: this.findPolyPath(startPoly.poly, endPoly.poly) };
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
    findNearestPoly(p) {
        const extents = MAX_JUMP_HEIGHT * MESH_CELL_SIZE_Z;//高度误差
        let bestPoly = -1;
        let bestDist = Infinity;
        let bestPos = p;
        const x = Math.floor(p.x / this.spatialCellSize);
        const y = Math.floor(p.y / this.spatialCellSize);
        for (let i = -1; i <= 1; i++) {
            for (let j = -1; j <= 1; j++) {
                const key = `${x + i}_${y + j}`;
                //const key = `${x}_${y}`;
                const candidates = this.spatialGrid.get(key);
                if (!candidates) continue;
                //if (!candidates) return{pos:bestPos,poly:bestPoly};
                for (const polyIdx of candidates) {
                    const poly = this.mesh.polys[polyIdx];
                    const cp = closestPointOnPoly(p, this.mesh.verts, poly);
                    if (!cp) continue;

                    if (cp.in == true) {
                        const h = this.heightfixer._getHeightOnDetail(polyIdx, p);
                        cp.z = h;
                    }
                    //Instance.DebugSphere({center:{x:cp.x,y:cp.y,z:cp.z},radius:2,duration:1,color:{r:255,g:0,b:0}});
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

        return { pos: bestPos, poly: bestPoly };
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
        // @ts-ignore
        if (ASTAR_OPTIMIZATION_1) return this.edgeWeights[a][b];
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