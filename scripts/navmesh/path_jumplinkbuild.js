import { Instance } from "cs_script/point_script";
import { AGENT_HEIGHT, AGENT_RADIUS, MAX_JUMP_HEIGHT, MAX_LINKS, MAX_POLYS, MAX_WALK_HEIGHT, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, PathState } from "./path_const";
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
        /**@type {NavMeshMesh} */
        this.mesh = polyMesh;
        // 待更新：Max Jump Down Dist: 157
        this.jumpDist = 32;
        this.jumpHeight = MAX_JUMP_HEIGHT*MESH_CELL_SIZE_Z;
        this.walkHeight = MAX_WALK_HEIGHT*MESH_CELL_SIZE_Z;
        this.agentHeight = AGENT_HEIGHT * MESH_CELL_SIZE_Z;
        this.linkdist=250;// 可行走区域 A 与可行走区域 B 之间跳点最小间距

        /**@type {Uint16Array} */
        this.poly=new Uint16Array(MAX_LINKS*2);//每个link占2个uint16，this.poly[i*2]=startpoly, this.poly[i*2+1]=endpoly
        /**@type {Float32Array} */
        this.cost=new Float32Array(MAX_LINKS);//每个link的代价

        /**@type {Uint8Array} */
        this.type=new Uint8Array(MAX_LINKS);//每个link的类型

        /**@type {Float32Array} */
        this.pos=new Float32Array(MAX_LINKS*6);//每个link占6个float，this.pos[i*6]~this.pos[i*6+2]为startpos，this.pos[i*6+3]~this.pos[i*6+5]为endpos
        /**@type {number} */
        this.length=0;

        // 存储每个多边形所属的连通区域 ID
        /**@type {Int16Array} */
        this.islandIds=new Int16Array(MAX_POLYS);
    }
    /**
     * 收集所有边界边，返回TypedArray，每3个为一组：polyIndex, p1索引, p2索引
     * p1/p2为顶点索引（不是坐标），便于后续批量处理
     * @returns {{boundarylengh:number,boundaryEdges:Uint16Array}} [polyIndex, p1, p2, ...]
     */
    collectBoundaryEdges() {
        const polyCount = this.mesh.polyslength;
        // 预估最大边界边数量
        const maxEdges = polyCount * 6;
        const result = new Uint16Array(maxEdges * 3);
        let edgeCount = 0;
        for (let i = 0; i < polyCount; i++) {
            const startVert = this.mesh.polys[i * 2];
            const endVert = this.mesh.polys[i * 2 + 1];
            const vertCount = endVert - startVert + 1;
            for (let j = 0; j < vertCount; j++) {
                const neighList = this.mesh.neighbors[i][j];
                if (!neighList[0]) {
                    const vi0 = startVert + j;
                    const vi1 = startVert + ((j + 1) % vertCount);
                    const idx = edgeCount * 3;
                    result[idx] = i;
                    result[idx + 1] = vi0;
                    result[idx + 2] = vi1;
                    edgeCount++;
                }
            }
        }
        // 截取有效部分
        return {boundarylengh:edgeCount,boundaryEdges:result};
    }
    /**
     * 判断两个多边形是否已经是物理邻居
     * @param {number} idxA
     * @param {number} idxB
     */
    areNeighbors(idxA, idxB) {
        const edgeList = this.mesh.neighbors[idxA];
        for (const entry of edgeList) {
            for (let k = 1; k <= entry[0]; k++) {
                if (entry[k] === idxB) return true;
            }
        }
        return false;
    }
    // 1D 区间间距：重叠返回 0，不重叠返回最小间距
    /**
     * @param {number} a0
     * @param {number} a1
     * @param {number} b0
     * @param {number} b1
     */
    intervalGap(a0, a1, b0, b1) {
        const amin = Math.min(a0, a1);
        const amax = Math.max(a0, a1);
        const bmin = Math.min(b0, b1);
        const bmax = Math.max(b0, b1);

        if (amax < bmin) return bmin - amax; // A 在 B 左侧
        if (bmax < amin) return amin - bmax; // B 在 A 左侧
        return 0; // 重叠
    }
    /**
     * @param {number} p1x
     * @param {number} p1y
     * @param {number} p1z
     * @param {number} p2x
     * @param {number} p2y
     * @param {number} p2z
     * @param {number} p3x
     * @param {number} p3y
     * @param {number} p3z
     * @param {number} p4x
     * @param {number} p4y
     * @param {number} p4z
     * @param {number} dist2dsq
     */
    closestPtSegmentSegment(p1x,p1y,p1z,p2x,p2y,p2z,p3x,p3y,p3z,p4x,p4y,p4z,dist2dsq) {
        const gapZ=this.intervalGap(p1z, p2z, p3z, p4z);
        if (gapZ > this.jumpHeight) return;
        const gapX = this.intervalGap(p1x, p2x, p3x, p4x);
        const gapY = this.intervalGap(p1y, p2y, p3y, p4y);

        if (gapX * gapX + gapY * gapY > dist2dsq)return
        // 算法来源：Real-Time Collision Detection (Graham Walsh)
        // 计算线段 S1(p1,p2) 与 S2(p3,p4) 之间最近点
        
        const d1 = { x: p2x - p1x, y: p2y - p1y, z: 0 }; // 忽略 Z 参与平面距离计算
        const d2 = { x: p4x - p3x, y: p4y - p3y, z: 0 };
        const r = { x: p1x - p3x, y: p1y - p3y, z: 0 };

        const a = d1.x * d1.x + d1.y * d1.y; // Squared length of segment S1
        const e = d2.x * d2.x + d2.y * d2.y; // Squared length of segment S2
        const f = d2.x * r.x + d2.y * r.y;

        const EPSILON = 1;

        // 检查线段是否退化成点
        if (a <= EPSILON && e <= EPSILON) {
            // 两个都是点
            return { dist: (p1x - p3x)*(p1x - p3x) + (p1y - p3y)*(p1y - p3y) + (p1z - p3z)*(p1z - p3z), ptA: {x: p1x, y: p1y, z: p1z}, ptB: {x: p3x, y: p3y, z: p3z} };
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
            x: p1x + (p2x - p1x) * s,
            y: p1y + (p2y - p1y) * s,
            z: p1z + (p2z - p1z) * s
        };

        const ptB = {
            x: p3x + (p4x - p3x) * t,
            y: p3y + (p4y - p3y) * t,
            z: p3z + (p4z - p3z) * t
        };
        const heightDiff = Math.abs(ptA.z - ptB.z);
        if (heightDiff > this.jumpHeight) return;

        let dist=(ptA.x - ptB.x)*(ptA.x - ptB.x) + (ptA.y - ptB.y)*(ptA.y - ptB.y);
        if(dist > dist2dsq)return;
        dist+=heightDiff*heightDiff;
        if (heightDiff < 1 && dist < 1) return;
        return {
            dist,
            ptA,
            ptB
        };
    }
    /**
     * 返回当前构建的 navmeshlink 结构
     * @returns {{poly: Uint16Array;pos: Float32Array;type: Uint8Array;cost: Float32Array;length: number;}}
     * @param {import("./path_manager").NavMeshLink} [Extlink]
     */
    return(Extlink) {
        if(Extlink)
        {
            const a = Extlink.length;
            const b = this.length;

            this.poly.set(
                Extlink.poly.subarray(0, a * 2),
                b*2
            );

            this.cost.set(
                Extlink.cost.subarray(0, a),
                b
            );

            this.type.set(
                Extlink.type.subarray(0, a),
                b
            );

            this.pos.set(
                Extlink.pos.subarray(0, a * 6),
                b * 6
            );
            this.length+=a;
        }
        return {
            poly: this.poly,
            pos: this.pos,
            type: this.type,
            cost: this.cost,
            length: this.length
        };
    }
    //mesh内所有连接，根据constructor给的mesh,用在tile内构建
    init() {
        // 3) 计算 mesh 连通分量（islandIds），后续用于“同岛且高度可走”过滤。
        this.buildConnectivity();
        // 4) 收集边界边（只在边界边之间寻找 jump 候选）。
        const {boundarylengh,boundaryEdges} = this.collectBoundaryEdges();
        // 5) 为边界边建立空间网格索引，加速近邻边查询。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges,boundarylengh);
        // 6) 收集候选并执行首轮筛选，得到每个 poly 对的最优候选。
        const bestJumpPerPoly = this._collectBestJumpCandidates(boundaryEdges,boundarylengh, edgeGrid);
        // 7) 对候选做收尾去重（pair 去重 + 岛对近距去重），并生成最终 links。
        this._finalizeJumpLinks(bestJumpPerPoly);
        // 9) 返回构建完成的 links。
        return this.return();
    }
    /**
     * 仅构建传入tile的 jump link。
     * @param {number} boundarylengh 
     * @param {Uint16Array} boundaryEdges//边界边
     * @param {Uint8Array} tileid
     * @param {NavMeshLink} Extlink//已有的link
     */
    initInterTileIn(boundarylengh,boundaryEdges,tileid,Extlink) {
        // 4) 计算 mesh 连通分量。
        this.buildConnectivity(tileid);
        // 5) 收集边界边。
        // 6) 建立边界边空间索引。
        const edgeGrid = this.buildEdgeGrid(boundaryEdges,boundarylengh);
        // 7) 收集候选并筛选：额外过滤“同 tile”pair，只保留跨 tile 候选。
        const bestJumpPerPoly = this._collectBestJumpCandidates(boundaryEdges,boundarylengh,edgeGrid,tileid);
        // 8) 对候选做收尾去重并生成最终 links。
        this._finalizeJumpLinks(bestJumpPerPoly);
        // 10) 返回构建完成的 links。
        return this.return(Extlink);
    }
    /**
     * @param {Uint16Array} boundaryEdges
     * @param {number} boundaryLength
     * @param {{grid: Map<number, number[]>;metas: Float32Array;cellSize: number;count: number}} edgeGrid
     * @param {Uint8Array} [tileid] //tile内的多边形打上标记，这样link只会从tile=2出发,即中心tile出发
     */
    _collectBestJumpCandidates(boundaryEdges, boundaryLength, edgeGrid, tileid) {
        // Key: "polyA_polyB", Value: { targetPoly, dist, startPos, endPos }
        const verts = this.mesh.verts;
        const islandIds = this.islandIds;
        const jumpDistSq = this.jumpDist * this.jumpDist;
        const bestJumpPerPoly = new Map();
        const candidateIndices=new Uint16Array(boundaryLength);
        for (let i = 0; i < boundaryLength; i++) {
            const idxA = (i<<1)+i;
            const polyIndexA = boundaryEdges[idxA];
            if(!islandIds[polyIndexA])continue;
            if(tileid&&tileid[polyIndexA]!=2)continue;
            const viA0 = boundaryEdges[idxA + 1]* 3;
            const viA1 = boundaryEdges[idxA + 2]* 3;
            candidateIndices[0]=0;
            this.queryNearbyEdges(edgeGrid, i, this.jumpDist,candidateIndices);
            for(let s=1;s<=candidateIndices[0];s++)
            {
                const j=candidateIndices[s];
                const idxB = (j<<1)+j;
                const polyIndexB = boundaryEdges[idxB];
                if(!islandIds[polyIndexB])continue;
                if(islandIds[polyIndexA] === islandIds[polyIndexB])continue;//同岛内的边界边不考虑构建跳跃链接
                if (polyIndexA === polyIndexB) continue;
                if(tileid&&tileid[polyIndexB]==2)continue;
                if(!tileid)
                {
                    //init()调用，判断多边形是否是邻居
                    if (this.areNeighbors(polyIndexA, polyIndexB)) continue;
                }
                const viB0 = boundaryEdges[idxB + 1]* 3;
                const viB1 = boundaryEdges[idxB + 2]* 3;
                const minBoxDist = this.bboxMinDist2D(edgeGrid.metas,i,j);
                if (minBoxDist > jumpDistSq) continue;
                
                const closestResult = this.closestPtSegmentSegment(
                    verts[viA0], verts[viA0+1], verts[viA0+2],
                    verts[viA1], verts[viA1+1], verts[viA1+2],
                    verts[viB0], verts[viB0+1], verts[viB0+2],
                    verts[viB1], verts[viB1+1], verts[viB1+2],
                    jumpDistSq);
                if (!closestResult) continue;

                const { dist, ptA, ptB } = closestResult;
                if (!this.validateJumpPath(ptA, ptB)) continue;
                this.updateBestCandidate(bestJumpPerPoly, polyIndexA, polyIndexB, dist, ptA, ptB);
            }
        }
        return bestJumpPerPoly;
    }

    /**
     * @param {Map<string,any>} bestJumpPerPoly
     * 直接填充TypedArray全局变量（poly、pos、type、cost、length）
     */
    _finalizeJumpLinks(bestJumpPerPoly) {
        const sortedCandidates = Array.from(bestJumpPerPoly.values());
        let linkCount = 0;
        const linkdistsq=this.linkdist*this.linkdist;
        for (const cand of sortedCandidates) {
            // 距离判重，需遍历已写入的link
            let tooClose = false;
            for (let k = 0; k < linkCount; k++) {
                const plIdx = k << 1;
                const exA = this.poly[plIdx];
                const exB = this.poly[plIdx + 1];
                const exIslandA = this.islandIds[exA];
                const exIslandB = this.islandIds[exB];
                const islandA = this.islandIds[cand.startPoly];
                const islandB = this.islandIds[cand.endPoly];
                if ((islandA === exIslandA && islandB === exIslandB) || (islandA === exIslandB && islandB === exIslandA)) {
                    // 距离判重
                    const posIdx = (k << 2) + (k << 1);
                    const exStart = {
                        x: this.pos[posIdx],
                        y: this.pos[posIdx + 1],
                        z: this.pos[posIdx + 2]
                    };
                    const exEnd = {
                        x: this.pos[posIdx + 3],
                        y: this.pos[posIdx + 4],
                        z: this.pos[posIdx + 5]
                    };
                    const dSqStart = vec.lengthsq(cand.startPos, exStart);
                    const dSqEnd = vec.lengthsq(cand.endPos, exEnd);
                    if (dSqStart < linkdistsq || dSqEnd < linkdistsq) {
                        tooClose = true;
                        break;
                    }
                }
            }
            if (tooClose) continue;
            // 写入TypedArray
            const pid=linkCount<<1;
            this.poly[pid] = cand.startPoly;
            this.poly[pid + 1] = cand.endPoly;
            const posIdx = (linkCount << 2) + (linkCount << 1);
            this.pos[posIdx] = cand.startPos.x;
            this.pos[posIdx + 1] = cand.startPos.y;
            this.pos[posIdx + 2] = cand.startPos.z;
            this.pos[posIdx + 3] = cand.endPos.x;
            this.pos[posIdx + 4] = cand.endPos.y;
            this.pos[posIdx + 5] = cand.endPos.z;
            this.cost[linkCount] = cand.dist * 1.5;
            this.type[linkCount] = (Math.abs(cand.startPos.z - cand.endPos.z) <= this.walkHeight ? PathState.WALK : PathState.JUMP);
            linkCount++;
        }
        this.length = linkCount;
    }
    /**
     * 计算多边形网格的连通分量
     * 给互相连通的多边形打上相同标识
     * 计算多边形网格的连通分量，TypedArray健壮兼容
     * 给互相连通的多边形打上相同标识
     * @param {Uint8Array<ArrayBufferLike>} [tileid]
     */
    buildConnectivity(tileid) {
        const numPolys = this.mesh.polyslength;
        this.islandIds = new Int16Array(numPolys);
        let currentId = 1;
        // 用TypedArray实现队列
        const queue = new Uint16Array(numPolys);
        for (let i = 0; i < numPolys; i++) {
            if (this.islandIds[i]) continue;
            if(tileid&&!tileid[i])continue;
            currentId++;
            let head = 0, tail = 0;
            queue[tail++] = i;
            this.islandIds[i] = currentId;
            while (head < tail) {
                let u = queue[head++];
                const neighbors = this.mesh.neighbors[u];
                // 获取该多边形的边数
                u<<=1;
                const startVert = this.mesh.polys[u];
                const endVert = this.mesh.polys[u + 1];
                const edgeCount = endVert - startVert + 1;
                for (let j = 0; j < edgeCount; j++) {
                    const entry = neighbors[j];
                    if (entry[0] == 0) continue;
                    for (let k = 1; k <= entry[0]; k++) {
                        const v = entry[k];
                        if (!this.islandIds[v]) {
                            this.islandIds[v] = currentId;
                            queue[tail++] = v;
                        }
                    }
                }
            }
        }
        //Instance.Msg(`共有${currentId-1}个独立行走区域`);
    }

    /**
     * 构建边界边空间网格索引，适配TypedArray输入
     * @param {Uint16Array} edges - 每3个为一组：polyIndex, p1, p2
     * @param {number} count - 边界边数量
     */
    buildEdgeGrid(edges, count) {
        const cellSize = this.jumpDist;
        const grid = new Map();
        const metas = new Float32Array(count << 2);
        for (let i = 0; i < count; i++) {
            const idx = (i<<1)+i;
            // const polyIndex = edges[idx]; // 未用
            const vi0 = edges[idx + 1]*3;
            const vi1 = edges[idx + 2]*3;
            const x0 = this.mesh.verts[vi0], y0 = this.mesh.verts[vi0 + 1];
            const x1 = this.mesh.verts[vi1], y1 = this.mesh.verts[vi1 + 1];
            const minX = Math.min(x0, x1);
            const maxX = Math.max(x0, x1);
            const minY = Math.min(y0, y1);
            const maxY = Math.max(y0, y1);
            const metaIdx = i << 2;
            metas[metaIdx] = minX;
            metas[metaIdx + 1] = maxX;
            metas[metaIdx + 2] = minY;
            metas[metaIdx + 3] = maxY;
            const gridX0 = Math.floor(minX / cellSize);
            const gridX1 = Math.floor(maxX / cellSize);
            const gridY0 = Math.floor(minY / cellSize);
            const gridY1 = Math.floor(maxY / cellSize);
            for (let x = gridX0; x <= gridX1; x++) {
                for (let y = gridY0; y <= gridY1; y++) {
                    const k = (y << 16) | x;
                    if(!grid.has(k)) grid.set(k, []);
                    grid.get(k).push(i);
                }
            }
        }
        return { grid, metas, cellSize,count};
    }

    /**
     * @param {{grid: Map<number, number[]>;metas: Float32Array;cellSize: number;count: number}} edgeGrid
     * @param {number} edgeIndex
     * @param {number} expand
     * @param {Uint16Array} result
     */
    queryNearbyEdges(edgeGrid, edgeIndex, expand, result) {
        edgeIndex <<=2;
        const x0 = Math.floor((edgeGrid.metas[edgeIndex] - expand) / edgeGrid.cellSize);
        const x1 = Math.floor((edgeGrid.metas[edgeIndex + 1] + expand) / edgeGrid.cellSize);
        const y0 = Math.floor((edgeGrid.metas[edgeIndex + 2] - expand) / edgeGrid.cellSize);
        const y1 = Math.floor((edgeGrid.metas[edgeIndex + 3] + expand) / edgeGrid.cellSize);
        /**@type {Uint8Array} */
        const seen = new Uint8Array(edgeGrid.count);
        for (let x = x0; x <= x1; x++) {
            for (let y = y0; y <= y1; y++) {
                const k = (y << 16) | x;
                const list = edgeGrid.grid.get(k);
                if (!list) continue;
                for (const idx of list) {
                    if (seen[idx]) continue;
                    seen[idx] = 1;
                    result[++result[0]] = idx;
                }
            }
        }
        return;
    }

    /**
     * @param {Float32Array} metas
     * @param {number} idxA
     * @param {number} idxB
     */
    bboxMinDist2D(metas, idxA, idxB) {
        idxA<<=2;
        idxB<<=2;
        return vec.length2Dsq({x:Math.max(0, Math.max(metas[idxA], metas[idxB]) - Math.min(metas[idxA + 1], metas[idxB + 1])),y:Math.max(0, Math.max(metas[idxA + 2], metas[idxB + 2]) - Math.min(metas[idxA + 3], metas[idxB + 3])),z:0});
    }

    /**
     * @param {Vector} a
     * @param {Vector} b
     */
    validateJumpPath(a, b) {
        const z=Math.max(a.z, b.z)+8;

        const start = { x: a.x, y: a.y, z: 8 };
        const end = { x: b.x, y: b.y, z: 8 };

        const boxMins = { x: -1, y: -1, z: 0 };
        const boxMaxs = { x: 1, y: 1, z: 1 };
        const hit = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(start,z),
            end:vec.Zfly(end,z),
            ignorePlayers: true
        });
        if (hit && hit.didHit) return false;
        const hitup = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(start,a.z),
            end:vec.Zfly(start,z),
            ignorePlayers: true
        });
        if (hitup && hitup.didHit) return false;
        const hitdown = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(end,z),
            end:vec.Zfly(end,b.z),
            ignorePlayers: true
        });
        if (hitdown && hitdown.didHit) return false;

        const hitReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start: vec.Zfly(end,z),
            end: vec.Zfly(start,z),
            ignorePlayers: true
        });
        if (hitReverse && hitReverse.didHit) return false;
        const hitupReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(end,b.z),
            end:vec.Zfly(end,z),
            ignorePlayers: true
        });
        if (hitupReverse && hitupReverse.didHit) return false;
        const hitdownReverse = Instance.TraceBox({
            mins: boxMins,
            maxs: boxMaxs,
            start:vec.Zfly(start,z),
            end:vec.Zfly(start,a.z),
            ignorePlayers: true
        });
        if (hitdownReverse && hitdownReverse.didHit) return false;
        return true;
    }
    /**
     * @param {Map<number,any>} map
     * @param {number} idxA
     * @param {number} idxB
    * @param {number} dist 两个多边形边界边之间的最短距离
     * @param {Vector} ptA
     * @param {Vector} ptB
     */
    updateBestCandidate(map, idxA, idxB, dist, ptA, ptB) {
        // 检查是否已记录过该多边形对的跳跃目标
        const key = (idxA << 16) | idxB;

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
        // 支持TypedArray结构
        Instance.Msg("debug");
        const { poly, pos, type, length } = this;
        const mesh = this.mesh;
        for (let i = 0; i < length; i++) {
            const polyA = poly[i * 2];
            const polyB = poly[i * 2 + 1];
            const t = type[i];
            const start = {
                x: pos[i * 6],
                y: pos[i * 6 + 1],
                z: pos[i * 6 + 2]
            };
            const end = {
                x: pos[i * 6 + 3],
                y: pos[i * 6 + 4],
                z: pos[i * 6 + 5]
            };
            Instance.DebugLine({
                start,
                end,
                color: { r: 0, g: (t === 1 ? 255 : 0), b: 255 },
                duration
            });
            // 可选：画起点终点球体
            // Instance.DebugSphere({ center: start, radius: 4, color: { r: 0, g: 255, b: 0 }, duration });
            // Instance.DebugSphere({ center: end, radius: 4, color: { r: 255, g: 0, b: 0 }, duration });
            // 绘制PolyB边界
            if (mesh && mesh.polys && mesh.verts) {
                const startVertB = mesh.polys[polyB * 2];
                const endVertB = mesh.polys[polyB * 2 + 1];
                const vertCountB = endVertB - startVertB+1;
                for (let j = 0; j < vertCountB; j++) {
                    const vi0 = startVertB + j;
                    const vi1 = startVertB + ((j + 1) % vertCountB);
                    const v0 = {
                        x: mesh.verts[vi0 * 3],
                        y: mesh.verts[vi0 * 3 + 1],
                        z: mesh.verts[vi0 * 3 + 2]
                    };
                    const v1 = {
                        x: mesh.verts[vi1 * 3],
                        y: mesh.verts[vi1 * 3 + 1],
                        z: mesh.verts[vi1 * 3 + 2]
                    };
                    Instance.DebugLine({ start: v0, end: v1, color: { r: 255, g: 0, b: 255 }, duration });
                }
                // 绘制PolyA边界
                const startVertA = mesh.polys[polyA * 2];
                const endVertA = mesh.polys[polyA * 2 + 1];
                const vertCountA = endVertA - startVertA + 1;
                for (let j = 0; j < vertCountA; j++) {
                    const vi0 = startVertA + j;
                    const vi1 = startVertA + ((j + 1) % vertCountA);
                    const v0 = {
                        x: mesh.verts[vi0 * 3],
                        y: mesh.verts[vi0 * 3 + 1],
                        z: mesh.verts[vi0 * 3 + 2]
                    };
                    const v1 = {
                        x: mesh.verts[vi1 * 3],
                        y: mesh.verts[vi1 * 3 + 1],
                        z: mesh.verts[vi1 * 3 + 2]
                    };
                    Instance.DebugLine({ start: v0, end: v1, color: { r: 255, g: 0, b: 255 }, duration });
                }
            }
        }
    }
}
