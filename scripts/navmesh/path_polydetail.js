import { Instance } from "cs_script/point_script";
import { POLY_DETAIL_SAMPLE_DIST, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, origin, pointInTri, POLY_DETAIL_HEIGHT_ERROR, isConvex, distPtSegSq } from "./path_const";
import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";
import { Tool } from "./util/tool";
import { vec } from "./util/vector";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */

export class PolyMeshDetailBuilder {
    /**
     * @param {NavMeshMesh} mesh
     * @param {OpenHeightfield} hf
     */
    constructor(mesh, hf) {
        /** @type {boolean} */
        this.error = false;
        this.mesh = mesh;
        /**@type {OpenHeightfield} */
        this.hf = hf;
        /**@type {Vector[]}*/
        this.verts = [];
        /**@type {number[][]}*/
        this.tris = [];
        /**@type {number[][]}*/
        this.meshes = [];
        /**@type {number[]} */
        this.triTopoly=[];
    }

    init() {
        this.error = false;
        for (let pi = 0; pi < this.mesh.polys.length; pi++) {
            this.buildPoly(pi);
        }

        return {
            verts: this.verts,
            tris: this.tris,
            meshes: this.meshes,
            triTopoly:this.triTopoly
        };
    }
    debugDrawPolys(duration = 5) {
        for (let pi = 0; pi < this.tris.length; pi++) {
            const tri = this.tris[pi];
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            Instance.DebugLine({ start:this.verts[tri[0]], end:this.verts[tri[1]], color, duration });
            Instance.DebugLine({ start:this.verts[tri[1]], end:this.verts[tri[2]], color, duration });
            Instance.DebugLine({ start:this.verts[tri[2]], end:this.verts[tri[0]], color, duration });
        }
    }
    /**
     * @param {number} pi
     */
    buildPoly(pi) {
        const poly = this.mesh.polys[pi];
        const regionid=this.mesh.regions[pi];
        const polyVerts = this.getPolyVerts(this.mesh, poly);
        // 待优化：内部采样点高度可改为基于细分后三角形插值

        // 1. 为多边形边界顶点采样高度
        const borderVerts = this.applyHeights(polyVerts, this.hf,regionid);
        // 2. 计算边界平均高度和高度范围
        const borderHeightInfo = this.calculateBorderHeightInfo(borderVerts);
        // 3. 获取初始三角划分（用于高度误差检查）
        const initialVertices = [...borderVerts];
        const initialConstraints = [];
        for (let i = 0; i < borderVerts.length; i++) {
            const j = (i + 1) % borderVerts.length;
            initialConstraints.push([i, j]);
        }
        // 4. 执行初始划分（基于边界点）
        const trianglesCDT = new SimplifiedCDT(initialVertices, initialConstraints, () => {
            this.error = true;
        });
        let triangles = trianglesCDT.getTri();
        // 5. 生成内部采样点
        let rawSamples = this.buildDetailSamples(polyVerts, borderHeightInfo, this.hf,triangles,trianglesCDT.vertices,regionid);
        // 6. 过滤内部采样点：仅保留高度误差较大的点
        while(rawSamples.length>0)
        {
            let insert=false;
            let heightDiff = 0;
            let heightid = -1;
            triangles = trianglesCDT.getTri();
            let toRemoveIndices = [];
            for (let i=0;i<rawSamples.length;i++) {
                const sample=rawSamples[i];
                let diff=0;
                // 找到包含采样点的三角形
                for (const tri of triangles) {
                    if (tri.containsPoint(sample, trianglesCDT.vertices)) {
                        const interpolatedHeight = tri.interpolateHeight(sample.x, sample.y, trianglesCDT.vertices);
                        diff = Math.abs(sample.z - interpolatedHeight);
                        if(this.isNearTriangleEdge(sample,tri,trianglesCDT.vertices)) diff = 0;
                        break;
                    }
                }
                // 仅当高度误差超过阈值时保留
                if(diff<=POLY_DETAIL_HEIGHT_ERROR)toRemoveIndices.push(i);
                else if (diff > heightDiff) {
                    heightDiff=diff;
                    heightid=i;
                    insert=true;
                }
            }
            if(insert)trianglesCDT.insertPointSimplified(rawSamples[heightid]);
            else break;
            for (let i = toRemoveIndices.length - 1; i >= 0; i--) {
                rawSamples.splice(toRemoveIndices[i], 1);
            }
        }
        // 7. 添加到全局列表
        const baseVert = this.verts.length;
        const baseTri = this.tris.length;
        const allVerts=trianglesCDT.vertices;
        for (const v of allVerts) {
            this.verts.push(v);
        }
        triangles = trianglesCDT.getTri();
        if (trianglesCDT.error) this.error = true;

        for (const tri of triangles) {
            this.tris.push([
                baseVert + tri.a,
                baseVert + tri.b,
                baseVert + tri.c
            ]);
            this.triTopoly.push(pi);
        }

        this.meshes.push([
            baseVert,
            allVerts.length,
            baseTri,
            triangles.length
        ]);
    }
    /**
    * 计算边界顶点高度信息
     * @param {Vector[]} borderVerts
     * @returns {{avgHeight: number, minHeight: number, maxHeight: number, heightRange: number}}
     */
    calculateBorderHeightInfo(borderVerts) {
        let sumHeight = 0;
        let minHeight = Infinity;
        let maxHeight = -Infinity;

        for (const v of borderVerts) {
            sumHeight += v.z;
            minHeight = Math.min(minHeight, v.z);
            maxHeight = Math.max(maxHeight, v.z);
        }

        const avgHeight = sumHeight / borderVerts.length;
        const heightRange = maxHeight - minHeight;

        return {
            avgHeight,
            minHeight,
            maxHeight,
            heightRange
        };
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {number[]} poly
     */
    getPolyVerts(mesh, poly) {
        return poly.map(vi => mesh.verts[vi]);
    }
    /**
    * 生成内部采样点（带高度误差检查）
     * @param {Vector[]} polyVerts
     * @param {{avgHeight: number;minHeight: number;maxHeight: number;heightRange: number;}} heightInfo
     * @param {OpenHeightfield} hf
     * @returns {Vector[]}
     * @param {Triangle[]} initialTriangles
     * @param {Vector[]} initialVertices
     * @param {number} regionid
     */
    buildDetailSamples(polyVerts, heightInfo, hf,initialTriangles,initialVertices,regionid) {
        const samples = [];
        // 2. AABB
        let minx = Infinity, miny = Infinity, maxx = -Infinity, maxy = -Infinity;
        for (const v of polyVerts) {
            minx = Math.min(minx, v.x);
            miny = Math.min(miny, v.y);
            maxx = Math.max(maxx, v.x);
            maxy = Math.max(maxy, v.y);
        }

        const step = POLY_DETAIL_SAMPLE_DIST * MESH_CELL_SIZE_XY;
        for (let x = minx + step / 2; x <= maxx; x += step) {
            for (let y = miny + step / 2; y <= maxy; y += step) {
                if (this.pointInPoly2D(x, y, polyVerts)) {
                    // 采样高度
                    let triheight=heightInfo.avgHeight;

                    // 计算与边界平均高度的差值
                    //const heightDiff = Math.abs(height - heightInfo.avgHeight);
                    for (const tri of initialTriangles) {
                        if (tri.containsPoint({x, y,z:heightInfo.avgHeight},initialVertices)) {
                            // 使用三角形插值计算高度
                            triheight = tri.interpolateHeight(x, y, initialVertices);
                            break;
                        }
                    }
                    const height=this.sampleHeight(hf, x, y, triheight??heightInfo.avgHeight,regionid);
                    // 检查是否超过阈值
                    if(Math.abs(height - triheight)>POLY_DETAIL_HEIGHT_ERROR) {
                        samples.push({ x: x, y: y, z: height });
                    }
                }
            }
        }
        return samples;
    }
    /**
     * @param {Vector} sample
     * @param {Triangle} tri
     * @param {Vector[]} verts
     */
    isNearTriangleEdge(sample, tri, verts) {

        const dis = Math.min(distPtSegSq(sample,verts[tri.a],verts[tri.b]),distPtSegSq(sample,verts[tri.b],verts[tri.c]),distPtSegSq(sample,verts[tri.c],verts[tri.a]));
        if (dis < POLY_DETAIL_SAMPLE_DIST * 0.5) return true;
        return false;
    }
    /**
     * @param {Vector[]} polyVerts
     * @param {OpenHeightfield} hf
     * @param {number} regionid
     */
    applyHeights(polyVerts, hf,regionid) {
        const resultVerts = [];
        const n = polyVerts.length;
        const step = POLY_DETAIL_SAMPLE_DIST * MESH_CELL_SIZE_XY;
        for (let i = 0; i < n; i++) {
            const a = polyVerts[i];
            const b = polyVerts[(i + 1) % n];
            // 对当前顶点采样高度
            const az = this.sampleHeight(hf, a.x, a.y, a.z,regionid);
            const bz = this.sampleHeight(hf, b.x, b.y, b.z, regionid);
            const A = { x: a.x, y: a.y, z: az };
            const B = { x: b.x, y: b.y, z: bz };
            // 添加当前顶点（起始点）
            resultVerts.push(A);

            // 细分当前边
            const samples = this.sampleEdgeWithHeightCheck(
                A, 
                B, 
                hf,
                step
            );
            // 递归插点
            this.subdivideEdgeByHeight(
                A,
                B,
                samples,
                hf,
                regionid,
                resultVerts
            );
        }
        
        return resultVerts;
    }
    /**
    * 在 [start, end] 之间递归插入高度误差最大的点
     * @param {Vector} start
     * @param {Vector} end
    * @param {Vector[]} samples // 该边上的细分点（不含 start/end）
     * @param {OpenHeightfield} hf
     * @param {number} regionid
     * @param {Vector[]} outVerts
     */
    subdivideEdgeByHeight(start, end,samples,hf,regionid,outVerts) {
        let maxError = 0;
        let maxIndex = -1;
        let maxVert = null;

        const total = samples.length;

        for (let i = 0; i < total; i++) {
            const s = samples[i];
            const t = (i + 1) / (total + 1);

            // 不加入该点时的插值高度
            const interpZ = start.z * (1 - t) + end.z * t;

            const h = this.sampleHeight(hf, s.x, s.y, interpZ, regionid);
            const err = Math.abs(h - interpZ);

            if (err > maxError) {
                maxError = err;
                maxIndex = i;
                maxVert = { x: s.x, y: s.y, z: h };
            }
        }

        // 没有需要加入的点
        if (maxError <= POLY_DETAIL_HEIGHT_ERROR || maxIndex === -1||!maxVert) {
            return;
        }

        // 递归左半段
        this.subdivideEdgeByHeight(
            start,
            maxVert,
            samples.slice(0, maxIndex),
            hf,
            regionid,
            outVerts
        );

        // 插入当前最大误差点（保持顺序）
        outVerts.push(maxVert);

        // 递归右半段
        this.subdivideEdgeByHeight(
            maxVert,
            end,
            samples.slice(maxIndex + 1),
            hf,
            regionid,
            outVerts
        );
    }
    /**
    * 在边上采样并检查高度误差
     * @param {Vector} start
     * @param {Vector} end
     * @param {OpenHeightfield} hf
     * @param {number} sampleDist
     * @returns {Vector[]}
     */
    sampleEdgeWithHeightCheck(start, end, hf, sampleDist) {
        const samples = [];
        
        // 计算边向量和长度
        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const length = Math.sqrt(dx * dx + dy * dy);
        
        if (length <= 1e-6) {
            return []; // 边长度为 0，不采样
        }
        
        // 计算方向向量
        const dirX = dx / length;
        const dirY = dy / length;
        // 计算采样点数（不包含起点和终点）
        const numSamples = Math.floor(length / sampleDist);
        
        // 记录采样点高度

        for (let i = 1; i <= numSamples; i++) {
            const t = i / (numSamples + 1); // 确保不会采样到端点
            const x = start.x + dirX * length * t;
            const y = start.y + dirY * length * t;
            const z = start.z * (1 - t) + end.z * t;
            samples.push({ x, y, z });
        }
        
        return samples;
    }
    /**
     * @param {OpenHeightfield} hf
     * @param {number} wx
     * @param {number} wy
     * @param {number} fallbackZ
     * @param {number} regionid
     */
    sampleHeight(hf, wx, wy, fallbackZ,regionid) {
        const globalIx = Math.round((wx - origin.x+ MESH_CELL_SIZE_XY / 2) / MESH_CELL_SIZE_XY);
        const globalIy = Math.round((wy - origin.y+ MESH_CELL_SIZE_XY / 2) / MESH_CELL_SIZE_XY);
        const ix = globalIx - (hf.baseX);
        const iy = globalIy - (hf.baseY);

        if (ix < 0 || iy < 0 || ix >= hf.gridX || iy >= hf.gridY) return fallbackZ;

        let best = null;
        let bestDiff = Infinity;
        let spanId = hf.cells[ix][iy];
        while (spanId !== 0) {
            if(OpenSpan.getRegionId(spanId)===regionid)
            {
                const z = origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z;
                const d = Math.abs(z - fallbackZ);
                if (d < bestDiff) {
                    bestDiff = d;
                    best = z;
                }
            }
            spanId = OpenSpan.getNext(spanId);
        }
        // 如果没有找到合适的 span，开始螺旋式搜索
        if (best === null) {
            const maxRadius = Math.max(hf.gridX, hf.gridY); // 搜索最大半径
            let radius = 1; // 初始半径
            out:
            while (radius <= maxRadius) {
                // 螺旋式外扩，检查四个方向
                for (let offset = 0; offset <= radius; offset++) {
                    // 检查 (ix + offset, iy + radius) 等候选位置
                    let candidates = [
                        [ix + offset, iy + radius], // 上
                        [ix + radius, iy + offset], // 右
                        [ix - offset, iy - radius], // 下
                        [ix - radius, iy - offset]  // 左
                    ];

                    for (const [nx, ny] of candidates) {
                        if (nx >= 0 && ny >= 0 && nx < hf.gridX && ny < hf.gridY) {
                            // 在有效范围内查找对应 span
                            spanId = hf.cells[nx][ny];
                            while (spanId !== 0) {
                                if(OpenSpan.getRegionId(spanId)===regionid)
                                {
                                    const z = origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z;
                                    const d = Math.abs(z - fallbackZ);
                                    if (d < bestDiff) {
                                        bestDiff = d;
                                        best = z;
                                        break out;
                                    }
                                }
                                spanId = OpenSpan.getNext(spanId);
                            }
                        }
                    }
                }
                // 增大半径，继续搜索
                radius++;
            }
        }

        // 如果最终未找到合适 span，返回 fallbackZ
        return best ?? fallbackZ;
    }
    /**
    * 判断点是否在多边形内（不含边界）
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

            // ===== 点在边上，按 outside 处理 =====
            if (Tool.pointOnSegment2D(px, py, xi, yi, xj, yj, { includeEndpoints: true })) {
                return false;
            }

            // ===== 射线法 =====
            const intersect =
                ((yi > py) !== (yj > py)) &&
                (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi);

            if (intersect) inside = !inside;
        }

        return inside;
    }

}

/**
 * 简化的约束 Delaunay 三角剖分器
 */
class SimplifiedCDT {
    /**
    * @param {Vector[]} vertices 顶点列表
    * @param {number[][]} constraints 约束边列表
     * @param {(() => void)} onError
     */
    constructor(vertices, constraints, onError) {
        /** @type {boolean} */
        this.error = false;
        /** @type {(() => void) | undefined} */
        this.onError = onError;
        this.vertices = vertices;
        this.constraints = constraints;
        /** @type {Triangle[]} */
        this.triangles = [];
        
        // 构建约束边查找集合
        this.constraintEdges = new Set();
        for (const [a, b] of constraints) {
            // 规范化边键（小索引在前）
            const key = Tool.orderedPairKey(a, b);
            this.constraintEdges.add(key);
        }
        // 初始剖分：耳切法
        this.earClipping(vertices);
    }

    /**
    * @returns {Triangle[]} 三角形顶点索引列表
     */
    getTri() {
        return this.triangles;
    }
    /**
     * @param {Vector[]} poly
     */
    earClipping(poly) {
        const verts = Array.from({ length: poly.length }, (_, i) => i);
        let guard = 0;
        while (verts.length > 3 && guard++ < 5000) {
            let bestEar=null;
            let minPerimeter=Infinity;
            let bestIndex=-1;

            for (let i = 0; i < verts.length; i++) {
                const prev = poly[verts[(i - 1 + verts.length) % verts.length]];
                const cur = poly[verts[i]];
                const next = poly[verts[(i + 1) % verts.length]];
                // cur 对应角度是否小于 180 度
                if (!isConvex(prev, cur, next)) continue;
                // 检查三角形是否包含其他点
                let contains = false;
                for (let j = 0; j < verts.length; j++) {
                    if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                    if (pointInTri(poly[verts[j]], prev, cur, next)) {
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                // 其他点不能在线段 prev-next 上
                for (let j = 0; j < verts.length; j++) {
                    if (j == i || j == (i - 1 + verts.length) % verts.length || j == (i + 1) % verts.length) continue;
                    if (distPtSegSq(poly[verts[j]], prev, next) == 0) // 判断点是否在线段上
                    {
                        if (vec.length2D(prev, poly[verts[j]]) == 0 || vec.length2D(next, poly[verts[j]]) == 0) continue;
                        contains = true;
                        break;
                    }
                }
                if (contains) continue;
                const perimeter = 
                vec.length2D(prev, cur) +
                vec.length2D(cur, next) +
                vec.length2D(next, prev);
            
                // 找到周长最小的耳朵
                if (perimeter < minPerimeter) {
                    minPerimeter = perimeter;
                    bestEar = {p:verts[(i - 1 + verts.length) % verts.length], c:verts[i], n:verts[(i + 1) % verts.length]};
                    bestIndex = i;
                }
            }
            // 找到最佳耳朵则切除
            if (bestEar && bestIndex !== -1) {
                this.triangles.push(new Triangle(bestEar.p, bestEar.c, bestEar.n));
                verts.splice(bestIndex, 1);
            } else {
                // 找不到耳朵，退出循环
                break;
            }
        }
        if (verts.length == 3) {
            this.triangles.push(new Triangle(verts[0], verts[1], verts[2]));
        }else {
            this.error = true;
            if (this.onError) this.onError();
            Instance.Msg("细节多边形耳切失败");
        }
    }
    /**
     * 简化的点插入方法
     * @param {Vector} point
     */
    insertPointSimplified(point) {

        const pointIndex = this.vertices.length;
        this.vertices.push(point);
        const p=this.vertices[pointIndex];
        let targetIdx = -1;

        // 找到包含点的三角形
        for (let i = 0; i < this.triangles.length; i++) {
            if (this.triangles[i].containsPoint(p, this.vertices)) {
                targetIdx = i;
                break;
            }
        }
        
        if (targetIdx === -1) {
            // 点不在任何三角形内（可能在边上），尝试处理边上点
            this.handlePointOnEdge(pointIndex);
            //Instance.Msg("点在边上");
            return;
        }

        const t = this.triangles[targetIdx];

        this.triangles.splice(targetIdx, 1);

        // 分裂为三个新三角形
        const t1 = new Triangle(t.a, t.b, pointIndex);
        const t2 = new Triangle(t.b, t.c, pointIndex);
        const t3 = new Triangle(t.c, t.a, pointIndex);
        
        this.triangles.push(t1, t2, t3);

        // 只对这三条边进行局部优化
        this.legalizeEdge(pointIndex, t.a, t.b);
        this.legalizeEdge(pointIndex, t.b, t.c);
        this.legalizeEdge(pointIndex, t.c, t.a);
    }
    /**
     * 处理点在边上的情况
     * @param {number} pointIndex 
     */
    handlePointOnEdge(pointIndex) {
        const p = this.vertices[pointIndex];
        // 先检查是否在约束边上
        for (const [a, b] of this.constraints) {
            if (Tool.pointOnSegment2D(p.x, p.y, this.vertices[a].x, this.vertices[a].y, this.vertices[b].x, this.vertices[b].y, { includeEndpoints: true })) {
                return;
            }
        }
        // 查找包含该点的边
        for (let i = 0; i < this.triangles.length; i++) {
            const tri = this.triangles[i];
            const edges = tri.edges();
            
            for (const [a, b] of edges) {
                if (this.isConstraintEdge(a, b)) continue;
                if (Tool.pointOnSegment2D(p.x, p.y, this.vertices[a].x, this.vertices[a].y, this.vertices[b].x, this.vertices[b].y, { includeEndpoints: true })) {
                    // 找到共享该边的另一个三角形
                    const otherTri = this.findAdjacentTriangleByEdge([a, b], tri);
                    
                    if (otherTri) {

                        // 移除两个共享该边的三角形
                        this.triangles.splice(this.triangles.indexOf(tri), 1);
                        this.triangles.splice(this.triangles.indexOf(otherTri), 1);
                        
                        // 获取两个三角形中不在该边上的顶点
                        const c = tri.oppositeVertex(a, b);
                        const d = otherTri.oppositeVertex(a, b);
                        
                        // 创建四个新三角形
                        const t1=new Triangle(a, pointIndex, c);
                        const t2=new Triangle(pointIndex, b, c);
                        const t3=new Triangle(a, d, pointIndex);
                        const t4=new Triangle(pointIndex, d, b);

                        this.triangles.push(t1,t2,t3,t4);

                        // 优化新产生的边
                        this.legalizeEdge(pointIndex, a, c);
                        this.legalizeEdge(pointIndex, b, c);
                        this.legalizeEdge(pointIndex, a, d);
                        this.legalizeEdge(pointIndex, b, d);
                        
                        return;
                    }
                }
            }
        }
    }
    /**
    * 局部递归优化 (Standard Delaunay Legalization)
    * @param {number} pIdx 新插入点
    * @param {number} v1 边的一端
    * @param {number} v2 边的另一端
     */
    legalizeEdge(pIdx, v1, v2) {
        // 约束边不可翻转
        if (this.isConstraintEdge(v1, v2)) {
            return;
        }
        
        const edge = [v1, v2];
        const triangleWithP = this.findTriangleByVerts(v1, v2, pIdx);
        if (!triangleWithP) return;
        
        const t2 = this.findAdjacentTriangleByEdge(edge, triangleWithP);
        if (!t2) return;

        const otherVert = t2.oppositeVertex(v1, v2);
        
        // 检查 Delaunay 条件
        if (this.inCircumcircle(
            this.vertices[v1], 
            this.vertices[v2], 
            this.vertices[pIdx], 
            this.vertices[otherVert]
        )) {
            // 翻转边
            this.removeTriangle(t2);
            this.removeTriangle(triangleWithP);

            // 创建两个新三角形
            const tt1=new Triangle(v1, otherVert, pIdx);
            const tt2=new Triangle(v2, otherVert, pIdx);

            this.triangles.push(tt1,tt2);

            // 递归优化新产生的两条外边
            this.legalizeEdge(pIdx, v1, otherVert);
            this.legalizeEdge(pIdx, v2, otherVert);
        }
    }
    
    /**
    * 检查是否为约束边
     * @param {number} a
     * @param {number} b
     */
    isConstraintEdge(a, b) {
        const key = Tool.orderedPairKey(a, b);
        return this.constraintEdges.has(key);
    }

    /**
    * 通过三个顶点找到三角形
     * @param {number} a
     * @param {number} b
     * @param {number} c
     */
    findTriangleByVerts(a, b, c) {
        for (const tri of this.triangles) {
            if ((tri.a === a && tri.b === b && tri.c === c) ||
                (tri.a === a && tri.b === c && tri.c === b) ||
                (tri.a === b && tri.b === a && tri.c === c) ||
                (tri.a === b && tri.b === c && tri.c === a) ||
                (tri.a === c && tri.b === a && tri.c === b) ||
                (tri.a === c && tri.b === b && tri.c === a)) {
                return tri;
            }
        }
        return null;
    }
    
    /**
    * 通过共享边找到相邻三角形
     * @param {number[]} edge
     * @param {Triangle} excludeTriangle
     */
    findAdjacentTriangleByEdge(edge, excludeTriangle) {
        const [a, b] = edge;
        
        for (const tri of this.triangles) {
            if (tri === excludeTriangle) continue;
            
            if ((tri.a === a && tri.b === b) ||
                (tri.a === b && tri.b === a) ||
                (tri.a === a && tri.c === b) ||
                (tri.a === b && tri.c === a) ||
                (tri.b === a && tri.c === b) ||
                (tri.b === b && tri.c === a)) {
                return tri;
            }
        }
        
        return null;
    }
    
    /**
    * 移除三角形
     * @param {Triangle} triangle
     */
    removeTriangle(triangle) {
        const index = this.triangles.indexOf(triangle);
        if (index !== -1) {
            this.triangles.splice(index, 1);
        }
    }

    /**
    * 检查点是否在三角形外接圆内
     * @param {{ x: any; y: any;}} a
     * @param {{ x: any; y: any;}} b
     * @param {{ x: any; y: any;}} c
     * @param {{ x: any; y: any;}} d
     */
    inCircumcircle(a, b, c, d) {
        const orient =
        (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
        const ax = a.x, ay = a.y;
        const bx = b.x, by = b.y;
        const cx = c.x, cy = c.y;
        const dx = d.x, dy = d.y;
        
        const adx = ax - dx;
        const ady = ay - dy;
        const bdx = bx - dx;
        const bdy = by - dy;
        const cdx = cx - dx;
        const cdy = cy - dy;
        
        const abdet = adx * bdy - bdx * ady;
        const bcdet = bdx * cdy - cdx * bdy;
        const cadet = cdx * ady - adx * cdy;
        const alift = adx * adx + ady * ady;
        const blift = bdx * bdx + bdy * bdy;
        const clift = cdx * cdx + cdy * cdy;
        
        const det = alift * bcdet + blift * cadet + clift * abdet;
        
        return orient > 0 ? det > 0 : det < 0;
    }
}
/**
 * 三角形类
 */
class Triangle {
    /**
     * @param {number} a
     * @param {number} b
     * @param {number} c
     */
    constructor(a, b, c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /**
    * 返回三角形的三条边
     * @returns {number[][]}
     */
    edges() {
        return [
            [this.a, this.b],
            [this.b, this.c],
            [this.c, this.a]
        ];
    }

    /**
    * 检查是否包含某条边
     * @param {number[]} edge
     * @returns {boolean}
     */
    hasEdge(edge) {
        const [e1, e2] = edge;
        return (this.a === e1 && this.b === e2) ||
            (this.b === e1 && this.c === e2) ||
            (this.c === e1 && this.a === e2) ||
            (this.a === e2 && this.b === e1) ||
            (this.b === e2 && this.c === e1) ||
            (this.c === e2 && this.a === e1);
    }

    /**
    * 检查点是否在三角形内
     * @param {Vector} point
     * @param {Vector[]} vertices
     * @returns {boolean}
     */
    containsPoint(point, vertices) {
        const va = vertices[this.a];
        const vb = vertices[this.b];
        const vc = vertices[this.c];

        return pointInTri(point, va, vb, vc);
    }

    /**
    * 找到边对面的顶点
     * @param {number} v1
     * @param {number} v2
     * @returns {number}
     */
    oppositeVertex(v1, v2) {
        if (this.a !== v1 && this.a !== v2) return this.a;
        if (this.b !== v1 && this.b !== v2) return this.b;
        if (this.c !== v1 && this.c !== v2) return this.c;
        return -1;
    }
    /**
    * 计算点在三角形平面上的插值高度
    * @param {number} x 点的 x 坐标
    * @param {number} y 点的 y 坐标
     * @param {Vector[]} vertices
    * @returns {number} 插值高度
     */
    interpolateHeight(x, y, vertices) {
        const va = vertices[this.a];
        const vb = vertices[this.b];
        const vc = vertices[this.c];
        
        // 使用重心坐标插值
        const denom = (vb.y - vc.y) * (va.x - vc.x) + (vc.x - vb.x) * (va.y - vc.y);
        
        if (Math.abs(denom) < 1e-6) {
            // 三角形退化时，返回三个顶点高度平均值
            return (va.z + vb.z + vc.z) / 3;
        }
        
        const u = ((vb.y - vc.y) * (x - vc.x) + (vc.x - vb.x) * (y - vc.y)) / denom;
        const v = ((vc.y - va.y) * (x - vc.x) + (va.x - vc.x) * (y - vc.y)) / denom;
        const w = 1 - u - v;
        
        // 插值高度
        return u * va.z + v * vb.z + w * vc.z;
    }
}
