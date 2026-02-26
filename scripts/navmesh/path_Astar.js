import { Instance } from "cs_script/point_script";
import { ASTAR_HEURISTIC_SCALE, PathState } from "./path_const";
import { FunnelHeightFixer } from "./path_funnelheightfixer";
import { Tool } from "./util/tool";

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("cs_script/point_script").Vector} Vector */

export class PolyGraphAStar {
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
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
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
