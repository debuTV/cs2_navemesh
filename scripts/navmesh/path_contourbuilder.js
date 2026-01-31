import { Instance } from "cs_script/point_script";
import { origin, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, CONT_MAX_ERROR, distPtSegSq } from "./path_const";
import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";

export class ContourBuilder {
    /**
     * @param {OpenHeightfield} hf
     */
    constructor(hf) {
        /** @type {(OpenSpan|null)[][]} */
        this.hf = hf.cells;
        this.gridX = hf.gridX;
        this.gridY = hf.gridY;

        /** @type {Contour[][]} */
        this.contours = [];
        this.cornerHeightCache = new Map();
    }

    /**
     * 边界边：没有邻居，或邻居 region 不同
     * @param {OpenSpan} span
     * @param {number} dir
     */
    isBoundaryEdge(span, dir) {
        const n = span.neighbors[dir];
        return !n || n.regionId !== span.regionId;
    }
    /**
     * @param {OpenSpan} span
     * @param {number} dir
     */
    getNeighborregionid(span, dir) {
        const n = span.neighbors[dir];
        if (n) return n.regionId;
        else return 0;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {OpenSpan} span
     * @param {number} dir
     */
    edgeKey(x, y, span, dir) {
        return `${x},${y},${span.id},${dir}`;
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
    /**
     * 判断线段 (p1, p2) 和 (p3, p4) 是否相交
     * @param {import("cs_script/point_script").Vector} p1
     * @param {import("cs_script/point_script").Vector} p2
     * @param {import("cs_script/point_script").Vector} p3
     * @param {import("cs_script/point_script").Vector} p4
     * @param {boolean} checkpoint //是否包含端点
     */
    segmentsIntersect(p1, p2, p3, p4, checkpoint) {
        const crossProduct = (/** @type {{ x: any; y: any; z?: number; }} */ a, /** @type {{ x: any; y: any; z?: number; }} */ b, /** @type {{ x: any; y: any; z?: number; }} */ c) => (c.y - a.y) * (b.x - a.x) - (b.y - a.y) * (c.x - a.x);
        // 快速排斥实验 + 跨立实验
        const d1 = crossProduct(p1, p2, p3);
        const d2 = crossProduct(p1, p2, p4);
        const d3 = crossProduct(p3, p4, p1);
        const d4 = crossProduct(p3, p4, p2);
        if (checkpoint) return (d1 * d2 <= 0 && d3 * d4 <= 0);
        return (d1 * d2 < 0 && d3 * d4 < 0);
        //return (ccw(p1, p3, p4) !== ccw(p2, p3, p4)) && (ccw(p1, p2, p3) !== ccw(p1, p2, p4));
    }
    /**
     * @param {Contour} holePt
     * @param {Contour[]} outer
     * @param {Contour[][]} holos
     * @param {number} innerid
     */
    findBridgeOuterIndex(holePt, outer, holos, innerid) {
        const inner = holos[innerid];
        let bestDistsq = Infinity;
        let bestIdx = -1;

        for (let i = 0; i < outer.length; i++) {
            const a = outer[i];
            //1.计算距离
            const dx = holePt.x - a.x;
            const dy = holePt.y - a.y;
            const distSq = dx * dx + dy * dy;

            //2.如果比当前最好的还远，直接跳过
            if (distSq >= bestDistsq) continue;
            //3.桥连线(holePt -> outerPt)是否与外圈的任何一条边相交
            let intersects = false;
            for (let j = 0; j < outer.length; j++) {
                const p1 = outer[j];
                const p2 = outer[(j + 1) % outer.length];

                if (j === i || (j + 1) % outer.length === i) continue;
                if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                    intersects = true;
                    break;
                }
            }
            if (intersects) continue;
            //4.是否与内圈自身的边相交（处理洞的凹陷部分）
            for (let j = 0; j < inner.length; j++) {
                const p1 = inner[j];
                const p2 = inner[(j + 1) % inner.length];
                // 忽略包含起点 holePt 的两条边
                if (p1 === holePt || p2 === holePt) continue;
                if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                    intersects = true;
                    break;
                }
            }
            if (intersects) continue;
            //5.是否与剩下几个未合并的内轮廓边相交
            for (let k = innerid + 1; k < holos.length; k++) {
                for (let j = 0; j < holos[k].length; j++) {
                    const p1 = holos[k][j];
                    const p2 = holos[k][(j + 1) % holos[k].length];
                    if (this.segmentsIntersect(holePt, a, p1, p2, true)) {
                        intersects = true;
                        break;
                    }
                }
            }
            if (!intersects) {
                bestDistsq = distSq;
                bestIdx = i;
            }
        }

        return bestIdx;
    }
    /**
     * @param {Contour[]} outer
     * @param {Contour[][]} holes
     * @param {number} holeid
     */
    mergeHoleIntoOuter(outer, holes, holeid) {
        const hole = holes[holeid];
        let oi = -1;
        let holePt = hole[0];
        let hi = 0;
        for (hi = 0; hi < hole.length; hi++) {
            holePt = hole[hi];
            oi = this.findBridgeOuterIndex(holePt, outer, holes, holeid);
            if (oi >= 0) break;
        }
        if (oi < 0) {
            Instance.Msg("没有找到桥连接内外轮廓");
            return outer;
        }
        /**@type {Contour[]} */
        const merged = [];

        // 1. outer → bridge 点
        for (let i = 0; i <= oi; i++) {
            merged.push(outer[i]);
        }

        // 2. bridge → hole 起点
        merged.push(holePt);

        // 3. 绕 hole 一圈（从 hi 开始到hole 起点及hi）
        for (let i = 1; i <= hole.length; i++) {
            merged.push(hole[(hi + i) % hole.length]);
        }

        // 4. 回到 outer bridge 点
        merged.push(outer[oi]);

        // 5. outer 剩余部分
        for (let i = oi + 1; i < outer.length; i++) {
            merged.push(outer[i]);
        }

        return merged;
    }
    mergeRegionContours() {
        /**@type {Map<number,Contour[][]>} */
        const byRegion = new Map();

        //按region分组
        for (const c of this.contours) {
            const rid = c[0].regionId;
            if (!byRegion.has(rid)) byRegion.set(rid, []);
            byRegion.get(rid)?.push(c);
        }

        const mergedContours = [];

        for (const [rid, contours] of byRegion) {
            /**@type {Contour[]} */
            let outer = [];
            const holes = [];

            for (const c of contours) {
                if (this.computeSignedArea(c) > 0) {
                    outer = c;
                } else {
                    holes.push(c);
                }
            }

            if (!outer) continue;
            //输出区域有几个内轮廓，和多边形生成时报错比较
            //if(holes.length>0)
            //{
            //    Instance.Msg(rid+"=="+holes.length);
            //}

            //待更新：给hole排序，按某一坐标从小到大

            //逐个把hole融进outer
            for (let i = 0; i < holes.length; i++) {
                outer = this.mergeHoleIntoOuter(outer, holes, i);
            }
            //outer=this.fixWinding(outer);
            mergedContours.push(outer);
        }

        // 用融合后的结果替换
        this.contours = mergedContours;
    }

    init() {
        /** @type {Set<string>} */
        const visited = new Set();

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            for (let dir = 0; dir < 4; dir++) {
                                if (this.isBoundaryEdge(span, dir)) {

                                    const key = this.edgeKey(x, y, span, dir);
                                    if (visited.has(key)) continue;

                                    let contour = this.traceContour(x, y, span, dir, visited);
                                    if (contour && contour.length >= 3) {
                                        //外轮廓：逆时针（CCW）
                                        //洞轮廓：顺时针（CW）
                                        const l = contour.length;
                                        contour = this.simplifyContour(contour);
                                        if (contour && contour.length >= 3) {
                                            this.contours.push(contour);
                                            //Instance.Msg(`{${l}}=>{${contour.length}}`);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    span = span.next;
                }
            }
        }
        this.mergeRegionContours();
    }
    /**
     * @param {Contour[]} contour
     */
    simplifyContour(contour) {
        const n = contour.length;
        if (n < 4) return contour.slice();
        const pts = contour.slice();
        const locked = new Array(n).fill(0);
        let locknum = 0;
        for (let i = 0; i < n; i++) {
            const prev = contour[(i + n - 1) % n];
            const cur = contour[i];
            const next = contour[(i + 1) % n];
            // portal 端点,相邻的1相连
            if (next.neighborRegionId != cur.neighborRegionId) {
                locked[i] += 1;
                locknum++;
                //Instance.DebugSphere({center:cur,radius:8,duration:120,color:{r:255,g:0,b:0}});
                //区域切换端点
            }
            //方向发生变化，相邻的2相连
            //if (cur.neighborRegionId==0&&!isCollinear(cur,prev,next)) {
            //    //Instance.DebugSphere({center:{x:cur.x,y:cur.y,z:cur.z+10},radius:8,duration:120,color:{r:0,g:255,b:0}});
            //    //转弯端点
            //    locked[i] += 2;
            //}
        }
        if (locknum == 0) {
            let minPt = pts[0];
            let maxPt = pts[0];
            let minid = 0;
            let maxid = 0;
            for (let i = 0; i < n; i++) {
                const pt = pts[i];
                if (pt.x < minPt.x || pt.y < minPt.y) {
                    minPt = pt;
                    minid = i;
                }
                if (pt.x > maxPt.x || pt.y > maxPt.y) {
                    maxPt = pt;
                    maxid = i;
                }
            }
            //没有强制顶点，判断为四周都是边境，或者四周都是某一区域，手动指定两个强制顶点
            locked[minid] = 1;
            locked[maxid] = 1;
        }
        /**@type {Contour[]}*/
        const out = [];
        let i = 0;
        let minI = -1;
        let maxJ = n;
        while (i < n - 1) {
            if (locked[i] == 0) {
                i++;
                continue;
            }
            if (minI == -1) minI = i;
            let j = (i + 1);
            while (j < n - 1 && locked[j] == 0) {
                j = j + 1;
            }
            if (locked[j]) maxJ = j;
            /**
             * @type {Contour[]}
             */
            if (locked[i] && locked[j]) this.simplifySegmentbymaxErrorSq(pts, locked, i, j, out);
            i = j;
        }
        this.simplifySegmentbymaxErrorSqFinally(pts, locked, maxJ, minI, out, n);
        return out;
    }
    /**
     * @param {Contour[]} pts
     * @param {number[]} locked
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     * @param {number}n
     */
    simplifySegmentbymaxErrorSqFinally(pts, locked, i0, i1, out, n) {
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
        const maxErrorSq = CONT_MAX_ERROR * CONT_MAX_ERROR;

        if (index !== -1 && maxDistSq > maxErrorSq) {
            if (index < i0) this.simplifySegmentbymaxErrorSqFinally(pts, locked, i0, index, out, n);
            else this.simplifySegmentbymaxErrorSq(pts, locked, i0, index, out);
            if (index < i1) this.simplifySegmentbymaxErrorSq(pts, locked, index, i1, out);
            else this.simplifySegmentbymaxErrorSqFinally(pts, locked, index, i1, out, n);
        } else {
            //只输出起点
            out.push(a);
        }

    }
    /**
     * @param {Contour[]} pts
     * @param {number[]} locked
     * @param {number} i0
     * @param {number} i1
     * @param {Contour[]} out
     */
    simplifySegmentbymaxErrorSq(pts, locked, i0, i1, out) {
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
        const maxErrorSq = CONT_MAX_ERROR * CONT_MAX_ERROR;
        if (index !== -1 && maxDistSq > maxErrorSq) {
            this.simplifySegmentbymaxErrorSq(pts, locked, i0, index, out);
            this.simplifySegmentbymaxErrorSq(pts, locked, index, i1, out);
        } else {
            //只输出起点
            out.push(a);
        }
    }
    /**
     * @param {Contour[]} contour
     */
    computeSignedArea(contour) {
        let area = 0;
        const n = contour.length;
        for (let i = 0; i < n; i++) {
            const p = contour[i];
            const q = contour[(i + 1) % n];
            area += (p.x * q.y - q.x * p.y);
        }
        return area * 0.5;
    }
    /**
     * abandon
     * @param {Contour[]} contour
     */
    fixWinding(contour) {
        const area = this.computeSignedArea(contour);
        //外轮廓CCW（area>0）
        if (area < 0) {
            contour.reverse();
        }
        return contour;
    }

    /**
     * @param {number} sx 起始 cell x
     * @param {number} sy 起始 cell y
     * @param {OpenSpan} startSpan
     * @param {number} startDir 起始边方向
     * @returns {Contour[] | null}
     * @param {Set<string>} visited
     */
    traceContour(sx, sy, startSpan, startDir, visited) {
        let x = sx;
        let y = sy;
        let span = startSpan;
        let dir = startDir;

        const verts = [];

        let iter = 0;
        const MAX_ITER = this.gridX * this.gridY * 4;
        if (!this.isBoundaryEdge(startSpan, startDir)) return null;
        const startKey = this.edgeKey(x, y, span, dir);
        while (iter++ < MAX_ITER) {
            const key = this.edgeKey(x, y, span, dir);
            //回到起点
            if (key === startKey && verts.length > 0) break;
            if (visited.has(key)) {
                Instance.Msg("奇怪的轮廓边,找了一遍现在又找一遍");
                return null;
            }
            visited.add(key);

            //只有在boundary边才输出顶点
            if (this.isBoundaryEdge(span, dir)) {
                const c = this.corner(x, y, dir);

                const h = this.getCornerHeightFromEdge(x, y, span, dir);
                const nid = this.getNeighborregionid(span, dir);
                //Instance.Msg(nid);
                if (h !== null) {
                    verts.push({
                        x: origin.x + c.x * MESH_CELL_SIZE_XY-MESH_CELL_SIZE_XY/2,
                        y: origin.y + c.y * MESH_CELL_SIZE_XY-MESH_CELL_SIZE_XY/2,
                        z: origin.z + h * MESH_CELL_SIZE_Z,
                        regionId: span.regionId,      //当前span的region
                        neighborRegionId: nid   //对面span的region（或 0）
                    });
                }
            }

            //顺序：右转 → 直行 → 左转 → 后转
            let advanced = false;
            for (let i = 0; i < 4; i++) {
                const ndir = (dir + 3 - i + 4) % 4;
                const nspan = span.neighbors[ndir];
                //如果这条边是boundary，就沿边走
                if (!nspan || nspan.regionId !== span.regionId) {
                    dir = ndir;
                    advanced = true;
                    break;
                }
                //否则穿过这条边
                const p = this.move(x, y, ndir);
                x = p.x;
                y = p.y;
                span = nspan;
                dir = (ndir + 2) % 4;
                advanced = true;
                break;
            }

            if (!advanced) {
                Instance.Msg("轮廓断啦");
                return null;
            }
        }
        if (verts.length < 3) return null;
        return verts;
    }
    /**
     * @param {number} x
     * @param {number} y
     * @param {OpenSpan} span
     * @param {number} dir
     */
    getCornerHeightFromEdge(x, y, span, dir) {
        let maxFloor = span.floor;
        let maxspan = span;
        const leftDir = (dir + 3) & 3;
        //左侧
        const p1 = this.move(x, y, leftDir);
        //前方
        const p2 = this.move(x, y, dir);
        //左前方
        const p3 = this.move(p1.x, p1.y, dir);

        //左侧能到的span
        if (this.inBounds(p1.x, p1.y)) {
            /**@type {OpenSpan|null} */
            let s = this.hf[p1.x][p1.y];
            while (s) {
                if(s.use)
                {
                    if (span.canTraverseTo(s)) {
                        if (maxFloor < s.floor) {
                            maxFloor = s.floor;
                            maxspan = s;
                        }
                    }
                }
                s = s.next;
            }
        }
        //前方能到的span
        if (this.inBounds(p2.x, p2.y)) {
            /**@type {OpenSpan|null} */
            let s = this.hf[p2.x][p2.y];
            while (s) {
                if(s.use)
                {
                    if (span.canTraverseTo(s)) {
                        if (maxFloor < s.floor) {
                            maxFloor = s.floor;
                            maxspan = s;
                        }
                    }
                }
                s = s.next;
            }
        }
        //对角能到的span
        if (this.inBounds(p3.x, p3.y)) {
            /**@type {OpenSpan|null} */
            let s = this.hf[p3.x][p3.y];
            while (s) {
                if(s.use)
                {
                    if (span.canTraverseTo(s)) {
                        if (maxFloor < s.floor) {
                            maxFloor = s.floor;
                            maxspan = s;
                        }
                    }
                }
                s = s.next;
            }
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

    debugDrawContours(duration = 5) {
        Instance.Msg(`一共${this.contours.length}个轮廓`)
        let k = 0;
        for (const contour of this.contours) {
            k++;
            const color = { r: 255 * Math.random(), g: 255 * Math.random(), b: 255 * Math.random() };
            const z = Math.random() * 20;
            for (let i = 0; i < contour.length; i++) {
                const a = contour[i];
                const b = contour[(i + 1) % contour.length];
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
                //if(a.neighborRegionId!=b.neighborRegionId)
                //{
                //Instance.DebugSphere({center:start,radius:8,duration,color});
                //}
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
 * @property {number} regionId
 * @property {number} neighborRegionId
 */