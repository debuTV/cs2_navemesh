import { Instance } from "cs_script/point_script";
import { AGENT_HEIGHT, AGENT_RADIUS, MAX_JUMP_HEIGHT, MAX_WALK_HEIGHT, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, PathState } from "./path_const";
import { Tool } from "./util/tool";
import { vec } from "./util/vector";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
export class JumpLinkBuilder
{
    /**
    * @param {NavMeshMesh} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        // 待更新：Max Jump Down Dist: 157
        this.jumpDist = 64;
        this.jumpHeight = MAX_JUMP_HEIGHT*MESH_CELL_SIZE_Z;
        this.walkHeight = MAX_WALK_HEIGHT*MESH_CELL_SIZE_Z;
        this.agentHeight = AGENT_HEIGHT * MESH_CELL_SIZE_Z;
        this.linkdist=250;// 可行走区域 A 与可行走区域 B 之间跳点最小间距
        /**@type {NavMeshLink[]}*/
        this.links = [];
        // 存储每个多边形所属的连通区域 ID
        /**@type {number[] | Int32Array<ArrayBuffer>}*/
        this.islandIds=[];
        /**
         * @type {string[]}
         */
        this.tileKeys = [];
    }
    /**
     * 收集所有边界边
     * @param {(string)[]} polyTileKeys 
     */
    collectBoundaryEdges(polyTileKeys=[]) {
        const edges = [];
        const { polys, verts, neighbors } = this.mesh;
        const skip=polyTileKeys.length!=0;
        for (let i = 0; i < polys.length; i++) {
            if(skip&&!polyTileKeys[i])continue;
            const poly = polys[i];
            for (let j = 0; j < poly.length; j++) {
                // 如果没有邻居，就是边界边
                const neighList = neighbors[i][j];
                if (neighList.length === 0) {
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
        const edgeList = this.mesh.neighbors[idxA] || [];
        for (const entry of edgeList) {
            if (entry.includes(idxB)) return true;
        }
        return false;
    }
    /**
     * @param {Vector} p1
     * @param {Vector} p2
     * @param {Vector} p3
     * @param {Vector} p4
     */
    closestPtSegmentSegment(p1, p2, p3, p4) {
        // 算法来源：Real-Time Collision Detection (Graham Walsh)
        // 计算线段 S1(p1,p2) 与 S2(p3,p4) 之间最近点
        
        const d1 = { x: p2.x - p1.x, y: p2.y - p1.y, z: 0 }; // 忽略 Z 参与平面距离计算
        const d2 = { x: p4.x - p3.x, y: p4.y - p3.y, z: 0 };
        const r = { x: p1.x - p3.x, y: p1.y - p3.y, z: 0 };

        const a = d1.x * d1.x + d1.y * d1.y; // Squared length of segment S1
        const e = d2.x * d2.x + d2.y * d2.y; // Squared length of segment S2
        const f = d2.x * r.x + d2.y * r.y;

        const EPSILON = 1e-6;

        // 检查线段是否退化成点
        if (a <= EPSILON && e <= EPSILON) {
            // 两个都是点
            return { dist: vec.lengthsq(p1, p3), ptA: p1, ptB: p3 };
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

        // 计算最近点坐标（包含 Z）
        // 注意：t 和 s 在 XY 平面求得，再应用到 3D 坐标
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
            dist: vec.lengthsq(ptA, ptB),
            ptA,
            ptB
        };
    }
    //mesh内所有连接，根据constructor给的mesh,用在tile内构建
    init() {
        // 1) 设置日志标签（用于区分构建类型）。
        const logTag = "JumpLink";
        // 2) 构建“已存在 poly 对”集合；普通模式下传空数组，表示不做外部去重。
        const existingPairSet = this._buildExistingPairSet([]);

        // 3) 计算 mesh 连通分量（islandIds），后续用于“同岛且高度可走”过滤。
        this.buildConnectivity();
        // 4) 收集边界边（只在边界边之间寻找 jump 候选）。
        const boundaryEdges = this.collectBoundaryEdges();
        // 5) 为边界边建立空间网格索引，加速近邻边查询。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges);
        // 6) 收集候选并执行首轮筛选，得到每个 poly 对的最优候选。
        const { bestJumpPerPoly, pairChecks, nearChecks } = this._collectBestJumpCandidates(boundaryEdges, edgeGrid);
        // 7) 对候选做收尾去重（pair 去重 + 岛对近距去重），并生成最终 links。
        const finalLinks = this._finalizeJumpLinks(bestJumpPerPoly, existingPairSet);

        // 8) 回写结果并输出统计日志。
        this.links = finalLinks;
        Instance.Msg(`${logTag}统计: 边=${boundaryEdges.length} pair=${pairChecks} near=${nearChecks} link=${this.links.length}`);
        // 9) 返回构建完成的 links。
        return this.links;
    }

    /**
     * 仅构建跨 tile 的 jump link。
     * @param {(string)[]} polyTileKeys
     * @param {NavMeshLink[]} existingLinks
     * @param {string} [midTile]
     */
    initInterTile(polyTileKeys, existingLinks,midTile) {
        // 1) 读取/兜底 tile 键映射（polyIndex -> tileId），用于“仅跨 tile”筛选。不存在则直接跳过
        this.tileKeys = polyTileKeys;
        // 2) 设置日志标签。
        const logTag = "InterTileJumpLink";
        // 3) 构建“已存在 poly 对”集合，避免与外部已有 links 重复。
        const existingPairSet = this._buildExistingPairSet(existingLinks);
        // 4) 计算 mesh 连通分量。
        this.buildConnectivity();
        // 5) 收集边界边。
        const boundaryEdges = this.collectBoundaryEdges(polyTileKeys);
        // 6) 建立边界边空间索引。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges);
        // 7) 收集候选并筛选：额外过滤“同 tile”pair，只保留跨 tile 候选。
        const { bestJumpPerPoly, pairChecks, nearChecks } = this._collectBestJumpCandidates(boundaryEdges,edgeGrid,true,midTile);
        // 8) 对候选做收尾去重并生成最终 links。
        const finalLinks = this._finalizeJumpLinks(bestJumpPerPoly, existingPairSet);
        // 9) 回写结果并输出统计日志。
        this.links = finalLinks;
        Instance.Msg(`${logTag}统计: 边=${boundaryEdges.length} pair=${pairChecks} near=${nearChecks} link=${this.links.length}`);
        // 10) 返回构建完成的 links。
        return this.links;
    }

    /**
     * @param {NavMeshLink[]} existingLinks
     */
    _buildExistingPairSet(existingLinks) {
        const pairSet = new Set();
        for (const link of existingLinks || []) {
            if (link.PolyA < 0 || link.PolyB < 0) continue;
            pairSet.add(Tool.orderedPairKey(link.PolyA, link.PolyB, "_"));
        }
        return pairSet;
    }
    /**
     * @param {number} polyIndexA
     * @param {number} [polyIndexB]
     */
    _shouldSkipPair(polyIndexA, polyIndexB)
    {
        const tileA = this.tileKeys[polyIndexA];
        if(polyIndexB)
        {
            const tileB = this.tileKeys[polyIndexB];
            return !tileA || !tileB || tileA === tileB;
        }
        return !tileA;
    }
    /**
     * @param {{polyIndex:number,p1:Vector,p2:Vector}[]} boundaryEdges
     * @param {{grid:Map<string, number[]>, metas:{minX:number,maxX:number,minY:number,maxY:number}[], cellSize:number}} edgeGrid
     * @param {boolean} [shouldSkipPair]
     * @param {string} [midTile]
     */
    _collectBestJumpCandidates(boundaryEdges, edgeGrid, shouldSkipPair=false,midTile) {
        // Key: "polyA_polyB", Value: { targetPoly, dist, startPos, endPos }
        const bestJumpPerPoly = new Map();
        let pairChecks = 0;
        let nearChecks = 0;
        const jdsq=this.jumpDist*this.jumpDist;
        for (let i = 0; i < boundaryEdges.length; i++) {
            const edgeA = boundaryEdges[i];
            if(shouldSkipPair&&this._shouldSkipPair(edgeA.polyIndex))continue;
            
            const candidateIndices = this.queryNearbyEdges(edgeGrid, i, this.jumpDist);

            for (const j of candidateIndices) {
                if (j <= i) continue;
                const edgeB = boundaryEdges[j];
                pairChecks++;
                if (edgeA.polyIndex === edgeB.polyIndex) continue;
                if (shouldSkipPair && this._shouldSkipPair(edgeA.polyIndex, edgeB.polyIndex)) continue;
                if(midTile&&this.tileKeys[edgeA.polyIndex]!=midTile&&this.tileKeys[edgeB.polyIndex]!=midTile)continue;

                const minBoxDist = this.bboxMinDist2D(edgeGrid.metas[i], edgeGrid.metas[j]);
                if (minBoxDist > jdsq) continue;
                nearChecks++;

                const closestResult = this.closestPtSegmentSegment(edgeA.p1, edgeA.p2, edgeB.p1, edgeB.p2);

                if (!closestResult) continue;
                const { dist, ptA, ptB } = closestResult;
                if (!ptA || !ptB) continue;
                if (vec.length2Dsq(ptA, ptB) > jdsq) continue;
                if (this.islandIds[edgeA.polyIndex] === this.islandIds[edgeB.polyIndex] && Math.abs(ptA.z - ptB.z) <= this.walkHeight) continue;
                const heightDiff = Math.abs(ptA.z - ptB.z);
                if (heightDiff > this.jumpHeight) continue;
                if (heightDiff < 1 && dist < 1) continue;

                if (!this.validateJumpPath(ptA, ptB)) continue;
                this.updateBestCandidate(bestJumpPerPoly, edgeA.polyIndex, edgeB.polyIndex, dist, ptA, ptB);
            }
        }
        return { bestJumpPerPoly, pairChecks, nearChecks };
    }

    /**
     * @param {Map<string,any>} bestJumpPerPoly
     * @param {Set<string>} existingPairSet
     */
    _finalizeJumpLinks(bestJumpPerPoly, existingPairSet) {
        const sortedCandidates = Array.from(bestJumpPerPoly.values()).sort((a, b) => a.dist - b.dist);

        const finalLinks = [];
        for (const cand of sortedCandidates) {
            const pairKey = Tool.orderedPairKey(cand.startPoly, cand.endPoly, "_");
            if (existingPairSet.has(pairKey)) continue;

            const islandA = this.islandIds[cand.startPoly];
            const islandB = this.islandIds[cand.endPoly];

            let tooClose = false;
            for (const existing of finalLinks) {
                const exIslandA = this.islandIds[existing.PolyA];
                const exIslandB = this.islandIds[existing.PolyB];

                if ((islandA === exIslandA && islandB === exIslandB)
                    || (islandA === exIslandB && islandB === exIslandA)) {
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
                    cost: cand.dist * 1.5,
                    type: (Math.abs(cand.startPos.z - cand.endPos.z) <= this.walkHeight ? PathState.WALK : PathState.JUMP)
                });
                existingPairSet.add(pairKey);
            }
        }

        return finalLinks;
    }
    /**
     * 计算多边形网格的连通分量
     * 给互相连通的多边形打上相同标识
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
                // 遍历该多边形所有邻居
                if (u === undefined) break;
                const neighbors = this.mesh.neighbors[u] || [];
                for (const entry of neighbors) {
                    for (const v of entry) {
                        if (v >= 0 && this.islandIds[v] === -1) {
                            this.islandIds[v] = currentId;
                            stack.push(v);
                        }
                    }
                }
            }
        }
        Instance.Msg(`共有${currentId}个独立行走区域`);
    }

    /**
     * @param {{polyIndex:number,p1:Vector,p2:Vector}[]} edges
     */
    buildEdgeGrid(edges) {
        const cellSize = this.jumpDist*2;
        /** @type {Map<string, number[]>} */
        const grid = new Map();
        /** @type {{minX:number,maxX:number,minY:number,maxY:number}[]} */
        const metas = new Array(edges.length);

        for (let i = 0; i < edges.length; i++) {
            const e = edges[i];
            const minX = Math.min(e.p1.x, e.p2.x);
            const maxX = Math.max(e.p1.x, e.p2.x);
            const minY = Math.min(e.p1.y, e.p2.y);
            const maxY = Math.max(e.p1.y, e.p2.y);
            metas[i] = { minX, maxX, minY, maxY };

            const x0 = Math.floor(minX / cellSize);
            const x1 = Math.floor(maxX / cellSize);
            const y0 = Math.floor(minY / cellSize);
            const y1 = Math.floor(maxY / cellSize);

            for (let x = x0; x <= x1; x++) {
                for (let y = y0; y <= y1; y++) {
                    const k = Tool.gridKey2(x, y);
                    Tool.getOrCreateArray(grid, k).push(i);
                }
            }
        }

        return { grid, metas, cellSize };
    }

    /**
     * @param {{grid: Map<string, number[]>;metas: {minX: number;maxX: number;minY: number;maxY: number;}[];cellSize: number;}} edgeGrid
     * @param {number} edgeIndex
     * @param {number} expand
     */
    queryNearbyEdges(edgeGrid, edgeIndex, expand) {
        const m = edgeGrid.metas[edgeIndex];
        const x0 = Math.floor((m.minX - expand) / edgeGrid.cellSize);
        const x1 = Math.floor((m.maxX + expand) / edgeGrid.cellSize);
        const y0 = Math.floor((m.minY - expand) / edgeGrid.cellSize);
        const y1 = Math.floor((m.maxY + expand) / edgeGrid.cellSize);
        const ans = [];
        /**@type {Map<number,boolean>} */
        const seen = new Map();
        for (let x = x0; x <= x1; x++) {
            for (let y = y0; y <= y1; y++) {
                const k = `${x}_${y}`;
                const list = edgeGrid.grid.get(k);
                if (!list) continue;
                for (const idx of list) {
                    if (seen.has(idx)) continue;
                    seen.set(idx,true);
                    ans.push(idx);
                }
            }
        }
        return ans;
    }

    /**
     * @param {{minX:number,maxX:number,minY:number,maxY:number}} a
     * @param {{minX:number,maxX:number,minY:number,maxY:number}} b
     */
    bboxMinDist2D(a, b) {
        const dx = Math.max(0, Math.max(a.minX, b.minX) - Math.min(a.maxX, b.maxX));
        const dy = Math.max(0, Math.max(a.minY, b.minY) - Math.min(a.maxY, b.maxY));
        return vec.length2Dsq({x:dx,y:dy,z:0});
    }

    /**
     * @param {Vector} a
     * @param {Vector} b
     */
    validateJumpPath(a, b) {
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        if (vec.length2Dsq({x:dx,y:dy,z:0}) < 1e-4) return false;
        const z=Math.max(a.z, b.z)+8;

        const start = { x: a.x, y: a.y, z: z };
        const end = { x: b.x, y: b.y, z: z };

        const boxMins = { x: -1, y: -1, z: 0 };
        const boxMaxs = { x: 1, y: 1, z: 52 };

        const hit = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start,
            end,
            ignorePlayers: true
        });
        if (hit && hit.didHit) return false;

        const hitReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start: end,
            end: start,
            ignorePlayers: true
        });
        if (hitReverse && hitReverse.didHit) return false;

        return true;
    }

    /**
     * @param {Vector} p
     */
    hasStandClearance(p) {
        const floorCheck = Instance.TraceLine({
            start: { x: p.x, y: p.y, z: p.z + 8 },
            end: { x: p.x, y: p.y, z: p.z - 32 },
            ignorePlayers: true,
        });
        if (!floorCheck || !floorCheck.didHit) return false;

        const upCheck = Instance.TraceLine({
            start: { x: p.x, y: p.y, z: p.z + 4 },
            end: { x: p.x, y: p.y, z: p.z + Math.max(8, this.agentHeight - 4) },
            ignorePlayers: true,
        });
        if (upCheck && upCheck.didHit) return false;

        return true;
    }
    /**
     * @param {Map<string,any>} map
     * @param {number} idxA
     * @param {number} idxB
    * @param {number} dist 两个多边形边界边之间的最短距离
     * @param {Vector} ptA
     * @param {Vector} ptB
     */
    updateBestCandidate(map, idxA, idxB, dist, ptA, ptB) {
        // 检查是否已记录过该多边形对的跳跃目标
        const key = Tool.orderedPairKey(idxA, idxB, "_");

        const existing = map.get(key);
        // 若未记录或发现更近目标，则更新
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
