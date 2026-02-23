import { Instance } from "cs_script/point_script";
import { closestPointOnPoly, MAX_JUMP_HEIGHT, MESH_CELL_SIZE_Z } from "../path_const";
import { FunnelHeightFixer } from "../path_funnelheightfixer";
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
export class Tool {
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
        //Instance.DebugSphere({center:{x:p.x,y:p.y,z:p.z},radius:2,duration:30,color:{r:255,g:255,b:255}});
        const extents = MAX_JUMP_HEIGHT * MESH_CELL_SIZE_Z;//高度误差
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
export class UnionFind {
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

