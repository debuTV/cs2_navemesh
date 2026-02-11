import { Instance } from "cs_script/point_script";
import { area, distPtSegSq, isConvex, POLY_MAX_VERTS_PER_POLY, pointInTri, POLY_MERGE_LONGEST_EDGE_FIRST, POLY_BIG_TRI } from "./path_const";
import { vec } from "../util/vector";

export class PolyMeshBuilder {

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
        if(POLY_BIG_TRI)
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
                            if (vec.length2D(prev, verts[j]) == 0 || vec.length2D(next, verts[j]) == 0) continue;
                            contains = true;
                            break;
                        }
                    }
                    if (contains) continue;
                    const perimeter = 
                    vec.length2D(prev, cur) +
                    vec.length2D(cur, next) +
                    vec.length2D(next, prev);
                
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
        else
        {
            while (verts.length > 3 && guard++ < 5000) {
                let earFound = false;
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
                            if (vec.length2D(prev, verts[j]) == 0 || vec.length2D(next, verts[j]) == 0) continue;
                            contains = true;
                            break;
                        }
                    }
                    if (contains) continue;
                    result.push([prev, cur, next]);
                    verts.splice(i, 1);
                    earFound = true;
                    break;
                }
                if (!earFound) break;
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
                            if (vec.length2D(prev, verts[j]) == 0 || vec.length2D(next, verts[j]) == 0) continue;
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
                if (vec.length(a[i],b[bNext])<=1&&vec.length(a[aNext],b[j])<=1) {
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
                const start = vec.Zfly(this.verts[poly[i]], z);
                const end = vec.Zfly(this.verts[poly[(i + 1) % poly.length]], z);
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
                const start = vec.Zfly(this.verts[polyA[ei]], 20);
                const end = vec.Zfly(this.verts[polyA[(ei + 1) % polyA.length]], 20);
                Instance.DebugLine({ start, end, color: { r: 0, g: 255, b: 0 }, duration });
            }
        }
    }
}   