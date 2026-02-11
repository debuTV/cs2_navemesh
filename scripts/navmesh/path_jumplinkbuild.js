import { Instance } from "cs_script/point_script";
import { MAX_JUMP_HEIGHT, MAX_WALK_HEIGHT, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z } from "./path_const";
import { vec } from "../util/vector";

export class JumpLinkBuilder
{
    /**
     * @param {{ verts: import("cs_script/point_script").Vector[]; polys: number[][]; regions: number[]; neighbors: number[][]; }} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        this.jumpDist = MESH_CELL_SIZE_XY*6;
        this.jumpHeight = MAX_JUMP_HEIGHT*MESH_CELL_SIZE_Z;
        this.walkHeight = MAX_WALK_HEIGHT*MESH_CELL_SIZE_Z;
        this.linkdist=250;//可行走区域A和可行走区域B之间的每个跳点的最小距离
        /**@type {{ PolyA: number; PolyB: number; PosA: import("cs_script/point_script").Vector; PosB: import("cs_script/point_script").Vector; cost: number;type: number; }[]}*/
        this.links = [];
        //存储每个多边形所属的连通区域 ID
        /**@type {number[] | Int32Array<ArrayBuffer>}*/
        this.islandIds=[];
    }
    /**
     * 收集所有的边界边
     */
    collectBoundaryEdges() {
        const edges = [];
        const { polys, verts, neighbors } = this.mesh;

        for (let i = 0; i < polys.length; i++) {
            const poly = polys[i];
            for (let j = 0; j < poly.length; j++) {
                // 如果没有邻居，就是边界边
                if (neighbors[i][j] < 0) {
                    const v1 = verts[poly[j]];
                    const v2 = verts[poly[(j + 1) % poly.length]];
                    edges.push({
                        polyIndex: i,
                        p1: v1,
                        p2: v2
                    });
                    //Instance.DebugLine({start:v1,end:v2,duration:60,color:{r:255,g:0,b:0}});
                }
            }
        }
        return edges;
    }
    /**
     * 判断两个多边形是否已经是物理邻居
     * @param {number} idxA
     * @param {number} idxB
     */
    areNeighbors(idxA, idxB) {
        const nList = this.mesh.neighbors[idxA];
        return nList.includes(idxB);
    }
    /**
     * @param {import("cs_script/point_script").Vector} p1
     * @param {import("cs_script/point_script").Vector} p2
     * @param {import("cs_script/point_script").Vector} p3
     * @param {import("cs_script/point_script").Vector} p4
     */
    closestPtSegmentSegment(p1, p2, p3, p4) {
        // 算法来源：Real-Time Collision Detection (Graham Walsh)
        // 计算线段 S1(p1, p2) 和 S2(p3, p4) 之间的最近点
        
        const d1 = { x: p2.x - p1.x, y: p2.y - p1.y, z: 0 }; // 忽略 Z 轴分量参与距离计算
        const d2 = { x: p4.x - p3.x, y: p4.y - p3.y, z: 0 };
        const r = { x: p1.x - p3.x, y: p1.y - p3.y, z: 0 };

        const a = d1.x * d1.x + d1.y * d1.y; // Squared length of segment S1
        const e = d2.x * d2.x + d2.y * d2.y; // Squared length of segment S2
        const f = d2.x * r.x + d2.y * r.y;

        const EPSILON = 1e-6;

        // 检查线段是否退化成点
        if (a <= EPSILON && e <= EPSILON) {
            // 两个都是点
            return { dist: vec.length2D(p1, p3), ptA: p1, ptB: p3 };
        }
        
        let s, t;
        if (a <= EPSILON) {
            // S1 是点
            s = 0.0;
            t = f / e;
            t = Math.max(0.0, Math.min(1.0, t));
        } else {
            const c = d1.x * r.x + d1.y * r.y;
            if (e <= EPSILON) {
                // S2 是点
                t = 0.0;
                s = Math.max(0.0, Math.min(1.0, -c / a));
            } else {
                // 常规情况：两条线段
                const b = d1.x * d2.x + d1.y * d2.y;
                const denom = a * e - b * b;

                if (denom !== 0.0) {
                    s = Math.max(0.0, Math.min(1.0, (b * f - c * e) / denom));
                } else {
                    // 平行
                    s = 0.0;
                }

                t = (b * s + f) / e;

                if (t < 0.0) {
                    t = 0.0;
                    s = Math.max(0.0, Math.min(1.0, -c / a));
                } else if (t > 1.0) {
                    t = 1.0;
                    s = Math.max(0.0, Math.min(1.0, (b - c) / a));
                }
            }
        }

        // 计算最近点坐标 (包含 Z)
        // 注意：这里的 t 和 s 是在 XY 平面上算出来的比例
        // 我们将其应用到 3D 坐标上，得到线段上的实际 3D 点
        const ptA = {
            x: p1.x + (p2.x - p1.x) * s,
            y: p1.y + (p2.y - p1.y) * s,
            z: p1.z + (p2.z - p1.z) * s
        };

        const ptB = {
            x: p3.x + (p4.x - p3.x) * t,
            y: p3.y + (p4.y - p3.y) * t,
            z: p3.z + (p4.z - p3.z) * t
        };

        return {
            dist: vec.length2D(ptA, ptB),
            ptA,
            ptB
        };
    }

    init() {
        this.buildConnectivity();
        const boundaryEdges = this.collectBoundaryEdges();
        // Key: "polyA_polyB", Value: { targetPoly, dist, startPos, endPos }
        const bestJumpPerPoly = new Map();

        for (let i = 0; i < boundaryEdges.length; i++) {
            for (let j = i + 1; j < boundaryEdges.length; j++) {
                const edgeA = boundaryEdges[i];
                const edgeB = boundaryEdges[j];

                // 1. 排除同一个多边形的边
                if (edgeA.polyIndex === edgeB.polyIndex) continue;
                // B. [核心需求] 如果已经在同一个连通区域（能走过去），排除
                //if (this.islandIds[edgeA.polyIndex] === this.islandIds[edgeB.polyIndex])continue; 
                // 2. 排除已经是邻居的多边形 (可选，看你是否想要捷径)
                //if (this.areNeighbors(edgeA.polyIndex, edgeB.polyIndex)) continue;

                // 3. 计算两条线段在 XY 平面上的最近距离
                // 我们主要关心水平距离是否足够近
                const closestResult = this.closestPtSegmentSegment(
                    edgeA.p1, edgeA.p2, 
                    edgeB.p1, edgeB.p2
                );

                // 如果计算失败（平行重叠等极端情况），跳过
                if (!closestResult) continue;

                const { dist, ptA, ptB } = closestResult;
                // 5. 距离判断
                if (dist > this.jumpDist) continue;

                //如果a和b在同一个可行走区域，并且没有跳跃的捷径，就跳过
                if(this.islandIds[edgeA.polyIndex] === this.islandIds[edgeB.polyIndex]&&Math.abs(ptA.z-ptB.z)<=this.walkHeight)continue;
                // 4. 高度判断 (Z轴)
                const heightDiff = Math.abs(ptA.z - ptB.z);
                if (heightDiff > this.jumpHeight) continue;
                //同一点跳过
                if (heightDiff <1&&dist < 1) continue;
                //Instance.DebugLine({start:ptA,end:ptB,duration:60,color:{r:255,g:0,b:0}});
                // 6. 记录候选
                this.updateBestCandidate(bestJumpPerPoly, edgeA.polyIndex, edgeB.polyIndex, dist, ptA, ptB);
            }
        }
        // 3. 根据 linkdist 过滤掉靠得太近的跳点
        // 我们需要按距离从短到长排序，优先保留质量最高的跳点
        const sortedCandidates = Array.from(bestJumpPerPoly.values()).sort((a, b) => a.distSq - b.distSq);
        
        const finalLinks = [];

        for (const cand of sortedCandidates) {
            const islandA = this.islandIds[cand.startPoly];
            const islandB = this.islandIds[cand.endPoly];

            // 检查在这两个区域(Island)之间，是否已经存在位置太近的跳点
            let tooClose = false;
            for (const existing of finalLinks) {
                const exIslandA = this.islandIds[existing.PolyA];
                const exIslandB = this.islandIds[existing.PolyB];

                // 如果这两个跳点连接的是相同的两个岛屿
                if ((islandA === exIslandA && islandB === exIslandB) || 
                    (islandA === exIslandB && islandB === exIslandA)) {
                    
                    // 检查起点或终点的欧几里得距离是否小于 linkdist
                    const dSqStart = vec.length(cand.startPos, existing.PosA);
                    const dSqEnd = vec.length(cand.endPos, existing.PosB);

                    if (dSqStart < this.linkdist || dSqEnd < this.linkdist) {
                        tooClose = true;
                        break;
                    }
                }
            }

            if (!tooClose) {
                finalLinks.push({
                    PolyA: cand.startPoly,
                    PolyB: cand.endPoly,
                    PosA: cand.startPos,
                    PosB: cand.endPos,
                    cost: Math.sqrt(cand.distSq) * 1.5,
                    type: (Math.abs(cand.startPos.z-cand.endPos.z)<=this.walkHeight?0:1)
                });
            }
        }

        this.links = finalLinks;
        return this.links;
    }
    /**
     * 计算多边形网格的连通分量
     * 给互相连接的多边形打上相同的标识符码
     */
    buildConnectivity() {
        const numPolys = this.mesh.polys.length;
        this.islandIds = new Int32Array(numPolys).fill(-1);
        let currentId = 0;

        for (let i = 0; i < numPolys; i++) {
            if (this.islandIds[i] !== -1) continue;

            currentId++;
            const stack = [i];
            this.islandIds[i] = currentId;

            while (stack.length > 0) {
                const u = stack.pop();
                //遍历该多边形的所有邻居
                if(!u)break;
                const neighbors = this.mesh.neighbors[u];
                for (let v of neighbors) {
                    //v是邻居多边形的索引。如果是负数表示边界，跳过
                    if (v >= 0 && this.islandIds[v] === -1) {
                        this.islandIds[v] = currentId;
                        stack.push(v);
                    }
                }
            }
        }
        Instance.Msg(`共有${currentId}个独立行走区域`);
    }
    /**
     * @param {Map<string,any>} map
     * @param {number} idxA
     * @param {number} idxB
     * @param {number} dist 两个多边形边界边之间的最短平方距离
     * @param {import("cs_script/point_script").Vector} ptA
     * @param {import("cs_script/point_script").Vector} ptB
     */
    updateBestCandidate(map, idxA, idxB, dist, ptA, ptB) {
        //检查是否已经记录过这个多边形的跳跃目标
        const id1 = Math.min(idxA, idxB);
        const id2 = Math.max(idxA, idxB);
        const key = `${id1}_${id2}`;

        const existing = map.get(key);
        //如果还没有记录，或者发现了一个更近的目标（distSq 更小）
        if (!existing || dist < existing.dist) {
            map.set(key, {
                startPoly: idxA,
                endPoly: idxB,
                dist: dist,
                startPos: { ...ptA },
                endPos: { ...ptB }
            });
        }
    }
    debugDraw(duration = 10) {
        for (const link of this.links) {
            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: { r: 0, g: (link.type==1?255:0), b: 255 },
                duration
            });
            //Instance.DebugSphere({ center: link.PosA, radius: 4, color: { r: 0, g: 255, b: 0 }, duration });
            //Instance.DebugSphere({ center: link.endPos, radius: 4, color: { r: 255, g: 0, b: 0 }, duration });
            let poly = this.mesh.polys[link.PolyB];
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({start,end,color:{ r: 255, g: 0, b: 255 },duration});
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
            poly = this.mesh.polys[link.PolyA];
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({start,end,color:{ r: 255, g: 0, b: 255 },duration});
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
        }
    }
}