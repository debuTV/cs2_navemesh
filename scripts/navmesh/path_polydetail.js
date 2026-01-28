import { area, POLY_DETAIL_SAMPLE_DIST, isConvex, MESH_CELL_SIZE_XY,MESH_CELL_SIZE_Z,origin, pointInTri, posDistance2Dsqr } from "./path_const";
import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";

export class PolyMeshDetailBuilder {
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
    }

    init() {
        for (let pi = 0; pi < this.mesh.polys.length; pi++) {
            this.buildPoly(pi);
        }

        return {
            verts: this.verts,
            tris: this.tris,
            meshes: this.meshes
        };
    }

    /**
     * @param {number} pi
     */
    buildPoly(pi) {
        const poly = this.mesh.polys[pi];
        const polyVerts = this.getPolyVerts(this.mesh, poly);

        const samples = this.buildDetailSamples(polyVerts);
        this.applyHeights(samples, this.hf);

        const baseVert = this.verts.length;
        for (const p of samples) {
            this.verts.push(p);
        }

        const tris = this.triangulate(samples);
        const baseTri = this.tris.length;

        for (const t of tris) {
            this.tris.push([
            baseVert + samples.indexOf(t[0]),
            baseVert + samples.indexOf(t[1]),
            baseVert + samples.indexOf(t[2]),
            ]);
        }

        this.meshes.push([
            baseVert,
            samples.length,
            baseTri,
            tris.length
        ]);
    }
    /**
     * @param {{ verts: import("cs_script/point_script").Vector[]; polys?: number[][]; regions?: number[]; neighbors?: number[][]; }} mesh
     * @param {number[]} poly
     */
    getPolyVerts(mesh, poly) {
        return poly.map(vi => mesh.verts[vi]);
    }
    /**
     * @param {import("cs_script/point_script").Vector[]} polyVerts
     */
    buildDetailSamples(polyVerts) {
        const samples = [];

        // 1. 边界点必加
        for (const v of polyVerts) {
            samples.push({ x: v.x, y: v.y, z: v.z });
        }

        // 2. AABB
        let minx=Infinity, miny=Infinity, maxx=-Infinity, maxy=-Infinity;
        for (const v of polyVerts) {
            minx = Math.min(minx, v.x);
            miny = Math.min(miny, v.y);
            maxx = Math.max(maxx, v.x);
            maxy = Math.max(maxy, v.y);
        }

        const step = POLY_DETAIL_SAMPLE_DIST * MESH_CELL_SIZE_XY;

        for (let x = minx; x <= maxx; x += step) {
            for (let y = miny; y <= maxy; y += step) {
            if (this.pointInPoly2D(x, y, polyVerts)) {
                samples.push({ x, y, z: 0 });
            }
            }
        }

        return samples;
    }

    /**
     * @param {import("cs_script/point_script").Vector[]} samples
     * @param {OpenHeightfield} hf
     */
    applyHeights(samples, hf) {
        for (const p of samples) {
            p.z = this.sampleHeight(hf, p.x, p.y, p.z);
        }
    }
    /**
     * @param {OpenHeightfield} hf
     * @param {number} wx
     * @param {number} wy
     * @param {number} fallbackZ
     */
    sampleHeight(hf, wx, wy, fallbackZ) {
        const ix = Math.floor((wx - origin.x) / MESH_CELL_SIZE_XY);
        const iy = Math.floor((wy - origin.y) / MESH_CELL_SIZE_XY);

        if (
            ix < 0 || iy < 0 ||
            ix >= hf.gridX || iy >= hf.gridY
        ) return fallbackZ;
        /**@type {OpenSpan|null} */
        let span = hf.cells[ix][iy];
        let best = null;
        let bestDiff = Infinity;

        while (span) {
            const z = origin.z + span.floor * MESH_CELL_SIZE_Z;
            const d = Math.abs(z - fallbackZ);
            if (d < bestDiff) {
                bestDiff = d;
                best = z;
            }
            span = span.next;
        }

        return best ?? fallbackZ;
    }
    /**
     * 判断点是否在多边形内（含边界）
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

            // ===== 点在边上（直接算 inside）=====
            if (this.pointOnSegment2D(px, py, xi, yi, xj, yj)) {
                return true;
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
    /**
     * @param {{x:number,y:number,z:number}[]} poly
     */
    triangulate(poly){
        const verts = poly.slice();
        const result = [];

        let guard = 0;
        while (verts.length > 3 && guard++ < 10000) {
            let earFound = false;

            for (let i = 0; i < verts.length; i++) {
                const prev = verts[(i-1+verts.length)%verts.length];
                const cur  = verts[i];
                const next = verts[(i+1)%verts.length];
                //cur对应的角度是否<180度
                if (!isConvex(prev,cur,next)) continue;

                let contains = false;
                for (let j = 0; j < verts.length; j++) {
                    if (j==i || j==(i-1+verts.length)%verts.length || j==(i+1)%verts.length) continue;
                    if (pointInTri(verts[j],prev,cur,next)) {
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                // 其他端点不能在新生成的边上,如果在边上，判断那个点与这边上两点是否在同一位置
                for (let j = 0; j < verts.length; j++) {
                    if (j==i || j==(i-1+verts.length)%verts.length || j==(i+1)%verts.length) continue;
                    if (area(prev,verts[j],next)==0) 
                    {
                        if(posDistance2Dsqr(prev,verts[j])==0||posDistance2Dsqr(next,verts[j])==0)continue;
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                result.push([prev,cur,next]);
                verts.splice(i,1);
                earFound = true;
                break;
            }
            if (!earFound) break;
        }

        if (verts.length === 3) {
            result.push([verts[0],verts[1],verts[2]]);
        }
        //Instance.Msg(verts.length);
        return result;
    }
}
