import { Instance } from "cs_script/point_script";
import { area, FUNNEL_DISTANCE, PathState } from "./path_const";
import { Tool } from "./util/tool";
import { vec } from "./util/vector";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
export class FunnelPath {
    /**
    * @param {NavMeshMesh} mesh
     * @param {Vector[]} centers
     * @param {Map<number,import("./path_manager").NavMeshLinkARRAY[]>} links 每个poly映射到typed link容器
     */
    constructor(mesh, centers, links) {
        this.mesh = mesh;
        this.centers = centers;
        /**@type {Map<number,import("./path_manager").NavMeshLinkARRAY[]>} */
        this.links = links;
        //Instance.Msg(this.links.size);
    }
    // 返回 pA 到 pB 的跳点
    /**
    * @param {number} polyA
    * @param {number} polyB
    */
    getlink(polyA, polyB) {
        const linkSet = this.links.get(polyA);
        if (!linkSet) return;
        for (const link of linkSet) {
            if (link.PolyB == polyB) return { start: link.PosB, end: link.PosA };
            if(link.PolyA == polyB)return { start: link.PosA, end: link.PosB };
        }
        //for (let i = 0; i < linkSet.length; i++) {
        //    const a = linkSet.poly[i<<1];
        //    const b = linkSet.poly[(i<<1) + 1];
        //    const posBase = i * 6;
        //    if (a === polyA && b === polyB) {
        //        return {
        //            start: {
        //                x: linkSet.pos[posBase + 3],
        //                y: linkSet.pos[posBase + 4],
        //                z: linkSet.pos[posBase + 5]
        //            },
        //            end: {
        //                x: linkSet.pos[posBase],
        //                y: linkSet.pos[posBase + 1],
        //                z: linkSet.pos[posBase + 2]
        //            }
        //        };
        //    }
        //    if (a === polyB && b === polyA) {
        //        return {
        //            start: {
        //                x: linkSet.pos[posBase],
        //                y: linkSet.pos[posBase + 1],
        //                z: linkSet.pos[posBase + 2]
        //            },
        //            end: {
        //                x: linkSet.pos[posBase + 3],
        //                y: linkSet.pos[posBase + 4],
        //                z: linkSet.pos[posBase + 5]
        //            }
        //        };
        //    }
        //}
    }
    /**
     * @param {{id:number,mode:number}[]} polyPath
     * @param {Vector} startPos
     * @param {Vector} endPos
     */
    build(polyPath, startPos, endPos) {
        if (!polyPath || polyPath.length === 0) return [];
        if (polyPath.length === 1) return [{pos:startPos,mode:PathState.WALK}, {pos:endPos,mode:PathState.WALK}];
        const ans = [];
        // 当前这一段行走路径的起点坐标
        let currentSegmentStartPos = startPos;
        // 当前这一段行走路径在 polyPath 中的起始索引
        let segmentStartIndex = 0;
        for (let i = 1; i < polyPath.length; i++) {
            const prevPoly = polyPath[i - 1];
            const currPoly = polyPath[i];
            if (currPoly.mode !=PathState.WALK)// 到第 i 个多边形需要特殊过渡（跳跃/梯子/传送）
            {
                // 1. 获取跳点坐标信息
                const linkInfo = this.getlink(currPoly.id,prevPoly.id);
                if (!linkInfo)continue;
                const portals = this.buildPortals(polyPath,segmentStartIndex,i-1, currentSegmentStartPos, linkInfo.start, FUNNEL_DISTANCE);
                const smoothedWalk = this.stringPull(portals);
                for (const p of smoothedWalk) ans.push({pos:p,mode:PathState.WALK});
                ans.push({pos:linkInfo.end,mode:currPoly.mode});
                currentSegmentStartPos = linkInfo.end; // 下一段从落地点开始
                segmentStartIndex = i; // 下一段多边形从 currPoly 开始
            }
        }
        const lastPortals = this.buildPortals(polyPath, segmentStartIndex, polyPath.length-1, currentSegmentStartPos, endPos, FUNNEL_DISTANCE);
        const lastSmoothed = this.stringPull(lastPortals);

        for (const p of lastSmoothed) ans.push({pos:p,mode:PathState.WALK});
        return this.removeDuplicates(ans);
    }
    /**
    * 简单去重，防止相邻点坐标完全一致
     * @param {{pos:Vector,mode:number}[]} path
     */
    removeDuplicates(path) {
        if (path.length < 2) return path;
        const res = [path[0]];
        for (let i = 1; i < path.length; i++) {
            const last = res[res.length - 1];
            const curr = path[i];
            const d = (last.pos.x - curr.pos.x) ** 2 + (last.pos.y - curr.pos.y) ** 2 + (last.pos.z - curr.pos.z) ** 2;
            // 容差阈值
            if (d > 1) {
                res.push(curr);
            }
        }
        return res;
    }
    /* ===============================
       Portal Construction
    =============================== */

    /**
     * @param {{id:number,mode:number}[]} polyPath
     * @param {number}start
     * @param {number}end
     * @param {Vector} startPos
     * @param {Vector} endPos
     * @param {number} funnelDistance
     */
    buildPortals(polyPath, start, end, startPos, endPos, funnelDistance) {
        const portals = [];

        // 起点
        portals.push({ left: startPos, right: startPos });
        for (let i = start; i < end; i++) {
            const a = polyPath[i].id;
            const b = polyPath[i + 1].id;
            const por = this.findPortal(a, b, funnelDistance);
            if (!por) continue;
            portals.push(por);
        }
        // 终点
        portals.push({ left: endPos, right: endPos });
        return portals;
    }

    /**
    * 寻找两个多边形的公共边
    * 再把这条边投影到 pb 的各条边上，取真正共线且重叠的片段
     * @param {number} pa
    * @param {number} pb
     * @param {number} funnelDistance
     */
    findPortal(pa, pb, funnelDistance) {
        const startA = this.mesh.polys[pa * 2];
        const endA = this.mesh.polys[pa * 2 + 1];
        const countA = endA - startA + 1;
        if (countA <= 0) return;

        const startB = this.mesh.polys[pb * 2];
        const endB = this.mesh.polys[pb * 2 + 1];
        const countB = endB - startB + 1;
        if (countB <= 0) return;

        const neighA = this.mesh.neighbors[pa];
        const neighB = this.mesh.neighbors[pb];
        if (!neighA || !neighB) return;

        // 1) 在 pa 找到通向 pb 的边（找到即用）
        let a0, a1;
        for (let ea = 0; ea < countA; ea++) {
            const entry = neighA[ea];
            if (!entry) continue;
            const n = entry[0] | 0;
            let hit = false;
            for (let k = 1; k <= n; k++) {
                if (entry[k] === pb) { hit = true; break; }
            }
            if (!hit) continue;

            const va0 = startA + ea;
            const va1 = startA + ((ea + 1) % countA);
            a0 = { x: this.mesh.verts[va0 * 3], y: this.mesh.verts[va0 * 3 + 1], z: this.mesh.verts[va0 * 3 + 2] };
            a1 = { x: this.mesh.verts[va1 * 3], y: this.mesh.verts[va1 * 3 + 1], z: this.mesh.verts[va1 * 3 + 2] };
            break;
        }
        if (!a0 || !a1) return;

        // 2) 只从 pb 里“通向 pa”的边里找共线重叠段
        const abx = a1.x - a0.x;
        const aby = a1.y - a0.y;
        const abLen2 = abx * abx + aby * aby;
        if (abLen2 < 1e-6) return;

        let best = null;
        //Instance.DebugLine({start:vec.Zfly(a0,5),end:vec.Zfly(a1,15),color:{r:255,g:255,b:0},duration:1/32});
        
        for (let eb = 0; eb < countB; eb++) {
            const entryB = neighB[eb];
            if (!entryB) continue;
            const nb = entryB[0] | 0;

            let bConnectedToA = false;
            for (let k = 1; k <= nb; k++) {
                if (entryB[k] === pa) { bConnectedToA = true; break; }
            }
            if (!bConnectedToA) continue;

            const vb0 = startB + eb;
            const vb1 = startB + ((eb + 1) % countB);
            const b0 = { x: this.mesh.verts[vb0 * 3], y: this.mesh.verts[vb0 * 3 + 1], z: this.mesh.verts[vb0 * 3 + 2] };
            const b1 = { x: this.mesh.verts[vb1 * 3], y: this.mesh.verts[vb1 * 3 + 1], z: this.mesh.verts[vb1 * 3 + 2] };
            //Instance.DebugLine({start:vec.Zfly(b0,5),end:vec.Zfly(b1,15),color:{r:255,g:255,b:0},duration:1/32});
        

            const tb0 = ((b0.x - a0.x) * abx + (b0.y - a0.y) * aby) / abLen2;
            const tb1 = ((b1.x - a0.x) * abx + (b1.y - a0.y) * aby) / abLen2;

            const tMin = Math.max(0, Math.min(tb0, tb1));
            const tMax = Math.min(1, Math.max(tb0, tb1));
            if (tMax - tMin <= 1e-4) continue;

            const p0 = {
                x: a0.x + abx * tMin,
                y: a0.y + aby * tMin,
                z: a0.z + (a1.z - a0.z) * tMin
            };
            const p1 = {
                x: a0.x + abx * tMax,
                y: a0.y + aby * tMax,
                z: a0.z + (a1.z - a0.z) * tMax
            };

            const dx = p1.x - p0.x;
            const dy = p1.y - p0.y;
            const len2 = dx * dx + dy * dy;
            if (!best || len2 > best.len2) best = { p0, p1, len2 };
        }
        
        // 没找到重叠段就退化
        const v0 = best ? best.p0 : a0;
        const v1 = best ? best.p1 : a1;

        // 左右稳定排序（不要只看一个点）
        const ca = this.centers[pa];
        const cb = this.centers[pb];
        const s0 = area(ca, cb, v0);
        const s1 = area(ca, cb, v1);
        const left = s0 >= s1 ? v0 : v1;
        const right = s0 >= s1 ? v1 : v0;
        //这里反了？？？？？
        return this._applyFunnelDistance(right, left, funnelDistance);
        
    }
    /**
     * 点到直线（ab）在 XY 上距离平方
     * @param {Vector} p
     * @param {Vector} a
     * @param {Vector} b
     */
    _pointLineDistSq2D(p, a, b) {
        const abx = b.x - a.x;
        const aby = b.y - a.y;
        const apx = p.x - a.x;
        const apy = p.y - a.y;
        const den = abx * abx + aby * aby;
        if (den < 1e-6) return Infinity;
        const cross = abx * apy - aby * apx;
        return (cross * cross) / den;
    }
    /**
    * 根据参数收缩门户宽度
     * @param {Vector} left 
     * @param {Vector} right 
     * @param {number} distance 0-100
     */
    _applyFunnelDistance(left, right, distance) {
        // 限制在 0-100
        const t = Tool.clamp(distance, 0, 100) / 100.0;

        // 若 t 为 0，保持原样
        if (t === 0) return { left, right };

        // 计算中点
        const midX = (left.x + right.x) * 0.5;
        const midY = (left.y + right.y) * 0.5;
        const midZ = (left.z + right.z) * 0.5;
        const mid = { x: midX, y: midY, z: midZ };

        // 使用线性插值将端点向中点移动
        // t=0 保持端点, t=1 变成中点
        const newLeft = Tool.lerpVector(left, mid, t);
        const newRight = Tool.lerpVector(right, mid, t);

        return { left: newLeft, right: newRight };
    }
    /* ===============================
       Funnel (String Pull)
    =============================== */

    /**
       * @param {{left:Vector,right:Vector}[]} portals
       */
    stringPull(portals) {
        const path = [];

        let apex = portals[0].left;
        let left = portals[0].left;
        let right = portals[0].right;

        let apexIndex = 0;
        let leftIndex = 0;
        let rightIndex = 0;

        path.push(apex);

        for (let i = 1; i < portals.length; i++) {
            const pLeft = portals[i].left;
            const pRight = portals[i].right;

            // 更新右边
            if (area(apex, right, pRight) <= 0) {
                if (apex === right || area(apex, left, pRight) > 0) {
                    right = pRight;
                    rightIndex = i;
                } else {
                    path.push(left);
                    apex = left;
                    apexIndex = leftIndex;
                    left = apex;
                    right = apex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    i = apexIndex;
                    continue;
                }
            }

            // 更新左边
            if (area(apex, left, pLeft) >= 0) {
                if (apex === left || area(apex, right, pLeft) < 0) {
                    left = pLeft;
                    leftIndex = i;
                } else {
                    path.push(right);
                    apex = right;
                    apexIndex = rightIndex;
                    left = apex;
                    right = apex;
                    leftIndex = apexIndex;
                    rightIndex = apexIndex;
                    i = apexIndex;
                    continue;
                }
            }
        }

        path.push(portals[portals.length - 1].left);
        return path;
    }
}

