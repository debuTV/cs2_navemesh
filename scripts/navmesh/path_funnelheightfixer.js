import { Instance } from "cs_script/point_script";
import { PathState, pointInConvexPolyXY, closestPointOnPoly } from "./path_const";

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */

export class FunnelHeightFixer {
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
        let bestDistSq = Infinity;

        for (let i = startIndex; i <= polyPath.length-1; i++) {
            const polyId2 = polyPath[i].id<<1;
            const start = this.navMesh.polys[polyId2];
            const end = this.navMesh.polys[polyId2 + 1];
            const cp = closestPointOnPoly(p, this.navMesh.verts, start, end);
            if (!cp||!cp.in) continue;
            return i;
            //cp.z = cp.z;
            //const dx = cp.x - p.x;
            //const dy = cp.y - p.y;
            //const dz = cp.z - p.z;
            //const d = dx * dx + dy * dy + dz * dz;
            //if (d < bestDistSq) {
            //    bestDistSq = d;
            //    bestIndex = i;
            //    return i; // 直接返回第一个找到的点，因为点一定在多边形投影内，不需要继续找了
            //}
        }
        return bestIndex;
    }
}
