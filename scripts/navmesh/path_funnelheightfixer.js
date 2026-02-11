import { pointInConvexPolyXY } from "./path_const";

export class FunnelHeightFixer {
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
        //Instance.DebugSphere({center:{ x: pos.x, y: pos.y, z: h },radius:1,duration:1/32,color:{r:0,g:255,b:0}});
                
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
            //Instance.Msg(samples.length);
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
                //Instance.DebugSphere({center:{ x: p.x, y: p.y, z: h },radius:1,duration:1/32,color:{r:255,g:0,b:0}});
                preh=h;
                prep=result[result.length - 1];
            }
        }
        // 最后一个点
        ////result.push(funnelPath[funnelPath.length - 1]);
        //删除起点和终点
        return result.slice(1,result.length);
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
     * @param {{ y: number; x: number; z:0}} p
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
}