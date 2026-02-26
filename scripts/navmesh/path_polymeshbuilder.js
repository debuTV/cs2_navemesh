import { Instance } from "cs_script/point_script";
import {area,distPtSegSq,isConvex,POLY_MAX_VERTS_PER_POLY,pointInTri,POLY_MERGE_LONGEST_EDGE_FIRST,POLY_BIG_TRI,origin,MESH_CELL_SIZE_XY,MESH_CELL_SIZE_Z, MAX_POLYS, MAX_VERTS} from "./path_const";
import { vec } from "./util/vector";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_contourbuilder").Contour} Contour */
export class PolyMeshBuilder {
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
                if (POLY_BIG_TRI) {
                    if (perimeter < bestPerimeter) {
                        bestPerimeter = perimeter;
                        bestIndex = i;
                    }
                } else {
                    bestIndex = i;
                    break;
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

