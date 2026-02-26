import { Instance } from "cs_script/point_script";
import { closestPointOnPoly, MAX_JUMP_HEIGHT, MESH_CELL_SIZE_Z } from "../path_const";
import { FunnelHeightFixer } from "../path_funnelheightfixer";
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
     * @param {boolean} [findall=false] 
     */
    static findNearestPoly(p, mesh, heightfixer,findall=false) {
        //Instance.DebugSphere({center:{x:p.x,y:p.y,z:p.z},radius:2,duration:30,color:{r:255,g:255,b:255}});
        if (gridW <= 0 || gridH <= 0 || cellStart.length === 0) {
            return { pos: p, poly: -1 };
        }
        const extents = MAX_JUMP_HEIGHT * MESH_CELL_SIZE_Z;
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
            if(inpoly&& !findall)break;
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

