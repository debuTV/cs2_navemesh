import { Instance } from "cs_script/point_script";
import { origin, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, CONT_MAX_ERROR, CONT_MAX_EDGE_LEN, distPtSegSq } from "./path_const";
import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";

export class ContourBuilder {
    /**
     * @param {OpenHeightfield} hf
     */
    constructor(hf) {
        /** @type {boolean} */
        this.error = false;
        /** @type {number[][]} */
        this.hf = hf.cells;
        this.gridX = hf.gridX;
        this.gridY = hf.gridY;
        this.baseX = hf.baseX;
        this.baseY = hf.baseY;
        this.tileCoreMinX = hf.tileCoreMinX;
        this.tileCoreMaxX = hf.tileCoreMaxX;
        this.tileCoreMinY = hf.tileCoreMinY;
        this.tileCoreMaxY = hf.tileCoreMaxY;

        /** @type {Contour[][]} */
        this.contours = [];
    }

    buildCompactNeighbors() {
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if (OpenSpan.getUse(spanId)) {
                        for (let d = 0; d < 4; d++) {
                            const nx = x + dirs[d].dx;
                            const ny = y + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                                OpenSpan.setNeighbor(spanId, d, 0);
                                continue;
                            }

                            let best = 0;
                            let bestDiff = Infinity;
                            let nspanId = this.hf[nx][ny];
                            while (nspanId !== 0) {
                                if (OpenSpan.getUse(nspanId) && OpenSpan.canTraverseTo(spanId, nspanId)) {
                                    const diff = Math.abs(OpenSpan.getFloor(spanId) - OpenSpan.getFloor(nspanId));
                                    if (diff < bestDiff) {
                                        best = nspanId;
                                        bestDiff = diff;
                                    }
                                }
                                nspanId = OpenSpan.getNext(nspanId);
                            }
                            OpenSpan.setNeighbor(spanId, d, best);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 边界边：没有邻居，或邻居 region 不同
     * @param {number} spanId
     * @param {number} dir
     */
    isBoundaryEdge(spanId, dir) {
        const n = OpenSpan.getNeighbor(spanId, dir);
        if (n === 0) return true;
        return OpenSpan.getRegionId(n) !== OpenSpan.getRegionId(spanId);
    }
    /**
     * @param {number} spanId
     * @param {number} dir
     */
    getNeighborregionid(spanId, dir) {
        const n = OpenSpan.getNeighbor(spanId, dir);
        if (n !== 0) return OpenSpan.getRegionId(n);
        else return 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {number} spanId
     * @param {number} dir
     */
    edgeKey(x, y, spanId, dir) {
        return `${x},${y},${spanId},${dir}`;
    }

    /**
     * @param {number} x
     * @param {number} y
     * @param {number} dir
     */
    move(x, y, dir) {
        switch (dir) {
            case 0: return { x: x - 1, y };
            case 1: return { x, y: y + 1 };
            case 2: return { x: x + 1, y };
            case 3: return { x, y: y - 1 };
        }
        return { x, y };
    }

    /**
     * dir对应的cell角点
     * @param {number} x
     * @param {number} y
     * @param {number} dir
     */
    corner(x, y, dir) {
        switch (dir) {
            case 0: return { x, y };
            case 1: return { x, y: y + 1 };
            case 2: return { x: x + 1, y: y + 1 };
            case 3: return { x: x + 1, y };
        }
        return { x, y };
    }

    init() {
        /** @type {Set<string>} */
        const visited = new Set();
        this.buildCompactNeighbors();

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            for (let dir = 0; dir < 4; dir++) {
                                if (this.isBoundaryEdge(spanId, dir)) {

                                    const key = this.edgeKey(x, y, spanId, dir);
                                    if (visited.has(key)) continue;

                                    let contour = this.traceContour(x, y, spanId, dir, visited);
                                    if (contour && contour.length >= 3) {
                                        //外轮廓：逆时针（CCW）
                                        //洞轮廓：顺时针（CW）
                                        contour = this.splitLongEdges(this.simplifyContour(contour));
                                        if (!contour || contour.length < 2) continue;

                                        if (!this.isDegenerateContour(contour) && contour.length >= 3) {
                                            this.contours.push(contour);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }
    /**
     * @param {Contour[]} contour
     */
    simplifyContour(contour) {
        const n = contour.length;
        if (n < 4) return contour.slice();
        const pts = contour.slice();

        const locked = new Array(n).fill(0);
        let lockCount = 0;
        for (let i = 0; i < n; i++) {
            const cur = pts[i];
            const next = pts[(i + 1) % n];
            const prev = pts[(i - 1 + n) % n];
            const isPortalChange = next.neighborRegionId !== cur.neighborRegionId;
            const keepBorderPoint = this.isPointOnTileBorder(cur) && !this.isBorderCollinearPoint(prev, cur, next);

            if (isPortalChange || keepBorderPoint) {
                locked[i] = 1;
                //Instance.DebugSphere({center: vec.Zfly(this.contourPointToWorld(cur),20*Math.random()), radius: 2, color:{r: 255, g: next.neighborRegionId!=0?255:0, b: 0},duration: 30});
                lockCount++;
            }
        }

        if (lockCount === 0) {
            let minId = 0;
            let maxId = 0;
            for (let i = 1; i < n; i++) {
                const p = pts[i];
                if (p.x < pts[minId].x || (p.x === pts[minId].x && p.y < pts[minId].y)) minId = i;
                if (p.x > pts[maxId].x || (p.x === pts[maxId].x && p.y > pts[maxId].y)) maxId = i;
            }
            locked[minId] = 1;
            locked[maxId] = 1;
        }

        /** @type {Contour[]} */
        const out = [];

        let i = 0;
        let firstLocked = -1;
        let lastLocked = -1;
        while (i < n - 1) {
            if (locked[i] === 0) {
                i++;
                continue;
            }

            if (firstLocked === -1) firstLocked = i;
            let j = i + 1;
            while (j < n - 1 && locked[j] === 0) j++;
            if (locked[j]) lastLocked = j;

            if (locked[i] && locked[j]) {
                // 锁点就是切换点：只看锁点后的第一条边类型
                const portalRegionId = pts[(i + 1) % n]?.neighborRegionId ?? 0;
                if (portalRegionId > 0) {
                    out.push(pts[i]);
                } else {
                    this.simplifySegmentByMaxError(pts, i, j, out);
                }
            }
            i = j;
        }

        // wrap 段同样只看锁点后的第一条边类型
        const wrapPortalRegionId = pts[(lastLocked + 1) % n]?.neighborRegionId ?? 0;
        if (wrapPortalRegionId > 0) {
            out.push(pts[lastLocked]);
        } else {
            this.simplifySegmentByMaxErrorWrap(pts, lastLocked, firstLocked, out);
        }

        if (out.length >= 3) {
            const indexByPoint = new Map();
            for (let k = 0; k < n; k++) {
                indexByPoint.set(pts[k], k);
            }

            /** @type {number[]} */
            const outIndices = [];
            for (const p of out) {
                const idx = indexByPoint.get(p);
                if (idx !== undefined) outIndices.push(idx);
            }
            return outIndices.map((idx) => pts[idx]);
        }

        return out;
    }
    /**
     * @param {Contour[]} pts
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     */
    simplifySegmentByMaxError(pts, i0, i1, out) {
        const a = pts[i0];
        const b = pts[i1];
        let maxDistSq = 0;
        let index = -1;

        for (let i = i0 + 1; i < i1; i++) {
            const d = distPtSegSq(pts[i], a, b);
            if (d > maxDistSq) {
                maxDistSq = d;
                index = i;
            }
        }

        const maxErrorSq = this.getContourMaxErrorSq();
        if (index !== -1 && maxDistSq > maxErrorSq) {
            this.simplifySegmentByMaxError(pts, i0, index, out);
            this.simplifySegmentByMaxError(pts, index, i1, out);
        } else {
            out.push(a);
        }
    }

    /**
     * @param {Contour[]} pts
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     */
    simplifySegmentByMaxErrorWrap(pts, i0, i1, out) {
        if (i0 < 0 || i1 < 0) return;

        const n = pts.length;
        const a = pts[i0];
        const b = pts[i1];
        let maxDistSq = 0;
        let index = -1;

        for (let i = i0 + 1; i < n; i++) {
            const d = distPtSegSq(pts[i], a, b);
            if (d > maxDistSq) {
                maxDistSq = d;
                index = i;
            }
        }
        for (let i = 0; i < i1; i++) {
            const d = distPtSegSq(pts[i], a, b);
            if (d > maxDistSq) {
                maxDistSq = d;
                index = i;
            }
        }

        const maxErrorSq = this.getContourMaxErrorSq();
        if (index !== -1 && maxDistSq > maxErrorSq) {
            if (index < i0) this.simplifySegmentByMaxErrorWrap(pts, i0, index, out);
            else this.simplifySegmentByMaxError(pts, i0, index, out);

            if (index < i1) this.simplifySegmentByMaxError(pts, index, i1, out);
            else this.simplifySegmentByMaxErrorWrap(pts, index, i1, out);
        } else {
            out.push(a);
        }
    }

    /**
     * 线段是否位于当前 tile 的边界上。
     * @param {Contour} a
     * @param {Contour} b
     */
    isSegmentOnTileBorder(a, b) {
        if (this.isPointOnTileBorder(a) || this.isPointOnTileBorder(b)) return true;

        const minX = this.tileCoreMinX;
        const maxX = this.tileCoreMaxX;
        const minY = this.tileCoreMinY;
        const maxY = this.tileCoreMaxY;

        if (a.x === minX && b.x === minX) return true;
        if (a.x === maxX && b.x === maxX) return true;
        if (a.y === minY && b.y === minY) return true;
        if (a.y === maxY && b.y === maxY) return true;

        return false;
    }

    /**
     * 点是否落在当前 tile 的外边界上。
     * @param {Contour} p
     */
    isPointOnTileBorder(p) {
        const minX = this.tileCoreMinX;
        const maxX = this.tileCoreMaxX;
        const minY = this.tileCoreMinY;
        const maxY = this.tileCoreMaxY;

        if (p.x === minX || p.x === maxX) return true;
        if (p.y === minY || p.y === maxY) return true;

        return false;
    }

    /**
     * tile 边界上的“纯共线中间点”判定。
     * 仅当 prev-cur-next 同在同一条 tile 外边界线上时返回 true。
     * @param {Contour} prev
     * @param {Contour} cur
     * @param {Contour} next
     */
    isBorderCollinearPoint(prev, cur, next) {
        const minX = this.tileCoreMinX;
        const maxX = this.tileCoreMaxX;
        const minY = this.tileCoreMinY;
        const maxY = this.tileCoreMaxY;

        if (prev.x === minX && cur.x === minX && next.x === minX) return true;
        if (prev.x === maxX && cur.x === maxX && next.x === maxX) return true;
        if (prev.y === minY && cur.y === minY && next.y === minY) return true;
        if (prev.y === maxY && cur.y === maxY && next.y === maxY) return true;

        return false;
    }

    /**
     * @param {Contour[]} counter
     */
    splitLongEdges(counter) {
        const maxEdgeLen = this.getContourMaxEdgeLen();
        if (maxEdgeLen <= 0) return counter;

        let guard = 0;
        while (guard++ < counter.length * 8) {
            let inserted = false;
            for (let i = 0; i < counter.length; i++) {
                const i0 = counter[i];
                const i1 = counter[(i + 1) % counter.length];
                const dx = Math.abs(i1.x - i0.x);
                const dy = Math.abs(i1.y - i0.y);
                if (Math.max(dx, dy) <= maxEdgeLen) continue;
                //这里在counter插入新点，值为两端点的中点
                const newPoint = {
                    x: (i0.x + i1.x) * 0.5,
                    y: (i0.y + i1.y) * 0.5,
                    z: (i0.z + i1.z) * 0.5,
                    regionId: i0.regionId,
                    neighborRegionId: i0.neighborRegionId
                };

                // 如果你的 counter/contour 存的是点对象：
                counter.splice(i + 1, 0, newPoint);
                inserted = true;
                break;
            }
            if (!inserted) break;
        }
        return counter;
    }
    /**
     * @param {Contour[]} contour
     */
    countUniqueXY(contour) {
        const set = new Set();
        for (const p of contour) set.add(`${p.x}|${p.y}`);
        return set.size;
    }

    /**
     * @param {Contour[]} contour
     */
    isDegenerateContour(contour) {
        if (!contour || contour.length < 3) return true;
        if (this.countUniqueXY(contour) < 3) return true;
        return Math.abs(this.computeSignedArea2D(contour)) <= 1e-6;
    }

    /**
     * @param {Contour[]} contour
     */
    computeSignedArea2D(contour) {
        let area = 0;
        const n = contour.length;
        for (let i = 0; i < n; i++) {
            const a = contour[i];
            const b = contour[(i + 1) % n];
            area += a.x * b.y - b.x * a.y;
        }
        return area * 0.5;
    }

    /**
     * @param {number} sx 起始 cell x
     * @param {number} sy 起始 cell y
     * @param {number} startSpanId
     * @param {number} startDir 起始边方向
     * @returns {Contour[] | null}
     * @param {Set<string>} visited
     */
    traceContour(sx, sy, startSpanId, startDir, visited) {
        let x = sx;
        let y = sy;
        let spanId = startSpanId;
        let dir = startDir;

        const verts = [];

        let iter = 0;
        const MAX_ITER = this.gridX * this.gridY * 4;
        if (!this.isBoundaryEdge(startSpanId, startDir)) return null;
        const startKey = this.edgeKey(x, y, spanId, dir);
        while (iter++ < MAX_ITER) {
            const key = this.edgeKey(x, y, spanId, dir);
            //回到起点
            if (key === startKey && verts.length > 0) break;

            if (visited.has(key)) {
                Instance.Msg("奇怪的轮廓边,找了一遍现在又找一遍");
                this.error=true;
                return null;
            }
            visited.add(key);

            // 只有在边界边才输出顶点
            if (this.isBoundaryEdge(spanId, dir)) {
                const c = this.corner(x, y, dir);

                const h = this.getCornerHeightFromEdge(x, y, spanId, dir);
                const nid = this.getNeighborregionid(spanId, dir);
                //Instance.Msg(nid);
                if (h !== null) {
                    verts.push({
                        x: this.baseX + c.x,
                        y: this.baseY + c.y,
                        z: h,
                        regionId: OpenSpan.getRegionId(spanId),      //当前span的region
                        neighborRegionId: nid   //对面span的region（或 0）
                    });
                }

            }

            // 顺序：右转 → 直行 → 左转 → 后转
            let advanced = false;
            for (let i = 0; i < 4; i++) {
                const ndir = (dir + 3 - i + 4) % 4;
                const nspanId = OpenSpan.getNeighbor(spanId, ndir);

                // 这条边是boundary，就沿边走
                if (nspanId === 0 || OpenSpan.getRegionId(nspanId) !== OpenSpan.getRegionId(spanId)) {
                    dir = ndir;
                    advanced = true;
                    break;
                }

                // 否则穿过这条边
                const p = this.move(x, y, ndir);
                x = p.x;
                y = p.y;
                spanId = nspanId;
                dir = (ndir + 2) % 4;
                advanced = true;
                break;
            }

            if (!advanced) {
                Instance.Msg("轮廓断啦");
                this.error=true;
                return null;
            }
        }
        if (verts.length < 3) {
            this.error=true;
            return null;
        }
        return verts;
    }

    /**
     * @param {number} x
     * @param {number} y
     * @param {number} spanId
     * @param {number} dir
     */
    getCornerHeightFromEdge(x, y, spanId, dir) {
        let maxFloor = OpenSpan.getFloor(spanId);
        const leftDir = (dir + 3) & 3;
        // 只使用 buildCompactNeighbors 建好的 walkable 邻接，
        // 避免在相邻 cell 的整列 span 中误取到“非当前可走链路”的高度层。
        const left = OpenSpan.getNeighbor(spanId, leftDir);
        if (left !== 0) {
            const h = OpenSpan.getFloor(left);
            if (h > maxFloor) maxFloor = h;
        }

        const front = OpenSpan.getNeighbor(spanId, dir);
        if (front !== 0) {
            const h = OpenSpan.getFloor(front);
            if (h > maxFloor) maxFloor = h;
        }

        // 对角采用“先左再前”与“先前再左”两条可走链路择优。
        let diag = 0;
        if (left !== 0) diag = OpenSpan.getNeighbor(left, dir);
        if (diag === 0 && front !== 0) diag = OpenSpan.getNeighbor(front, leftDir);
        if (diag !== 0) {
            const h = OpenSpan.getFloor(diag);
            if (h > maxFloor) maxFloor = h;
        }

        return maxFloor;
    }
    /**
     * @param {number} x
     * @param {number} y
     */
    inBounds(x, y) {
        return x >= 0 && y >= 0 && x < this.gridX && y < this.gridY;
    }

    getContourMaxErrorSq() {
        const e = CONT_MAX_ERROR;
        return e * e;
    }

    getContourMaxEdgeLen() {
        if (CONT_MAX_EDGE_LEN <= 0) return 0;
        return CONT_MAX_EDGE_LEN;
    }

    /**
     * @param {Contour} v
     */
    contourPointToWorld(v) {
        return {
            x: origin.x + v.x * MESH_CELL_SIZE_XY ,//- MESH_CELL_SIZE_XY / 2,
            y: origin.y + v.y * MESH_CELL_SIZE_XY ,//- MESH_CELL_SIZE_XY / 2,
            z: origin.z + v.z * MESH_CELL_SIZE_Z,
        };
    }

    debugDrawContours(duration = 5) {
        Instance.Msg(`一共${this.contours.length}个轮廓`)
        for (const contour of this.contours) {
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            const z = Math.random() * 20;
            for (let i = 0; i < contour.length; i++) {
                const a = this.contourPointToWorld(contour[i]);
                const b = this.contourPointToWorld(contour[(i + 1) % contour.length]);
                const start = {
                    x: a.x,
                    y: a.y,
                    z: a.z + z
                };
                const end = {
                    x: b.x,
                    y: b.y,
                    z: b.z + z
                };
                Instance.DebugLine({
                    start,
                    end,
                    color,
                    duration
                });
            }
        }
    }
}
/**
 * @typedef {Object} Contour
 * @property {number} x
 * @property {number} y
 * @property {number} z
 * x,y 为离散格点坐标；z 为离散高度层
 * @property {number} regionId
 * @property {number} neighborRegionId
 */

