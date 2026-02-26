import { area, FUNNEL_DISTANCE, PathState } from "./path_const";
import { Tool } from "./util/tool";
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
     * @param {number} pa
    * @param {number} pb
     * @param {number} funnelDistance
     */
    findPortal(pa, pb, funnelDistance) {
        const startVert = this.mesh.polys[pa * 2];
        const endVert = this.mesh.polys[pa * 2 + 1];
        const vertCount = endVert - startVert + 1;
        if (vertCount <= 0) return;
        const neigh = this.mesh.neighbors[pa];
        if (!neigh) return;

        for (let ei = 0; ei < vertCount; ei++) {
            const entry = neigh[ei];
            if (!entry) continue;
            const count = entry[0] | 0;
            if (count <= 0) continue;
            let connected = false;
            for (let k = 1; k <= count; k++) {
                if (entry[k] === pb) {
                    connected = true;
                    break;
                }
            }
            if (!connected) continue;

            const vi0 = startVert + ei;
            const vi1 = startVert + ((ei + 1) % vertCount);
            const v0 = {
                x: this.mesh.verts[vi0 * 3],
                y: this.mesh.verts[vi0 * 3 + 1],
                z: this.mesh.verts[vi0 * 3 + 2]
            };
            const v1 = {
                x: this.mesh.verts[vi1 * 3],
                y: this.mesh.verts[vi1 * 3 + 1],
                z: this.mesh.verts[vi1 * 3 + 2]
            };

            // 统一左右（从 pa 看向 pb）
            const ca = this.centers[pa];
            const cb = this.centers[pb];

            if (area(ca, cb, v0) < 0) {
                return this._applyFunnelDistance(v0, v1, funnelDistance);
            } else {
                return this._applyFunnelDistance(v1, v0, funnelDistance);
            }
        }
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

