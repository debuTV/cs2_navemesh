
import { Entity, Instance } from "cs_script/point_script";
import { origin, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, MESH_WORLD_SIZE_XY, MESH_WORLD_SIZE_Z, traceGroundpd, AGENT_HEIGHT, MESH_HOLE_FILLING, MAX_JUMP_HEIGHT, AGENT_RADIUS, MESH_ERODE_RADIUS } from "./path_const";
import { OpenSpan } from "./path_openspan";

export class OpenHeightfield {
    constructor() {

        /**@type {(OpenSpan | null)[][]}*/
        this.cells = [];
        /**@type {boolean[][][]}*/
        this.precells = [];
        this.SPAN_ID = 1;
        /**@type {number} */
        this.gridX = Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        /**@type {number} */
        this.gridY = Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        /**@type {number} */
        this.gridZ = Math.floor(MESH_WORLD_SIZE_Z / MESH_CELL_SIZE_Z) + 1;
    }
    init() {
        const minZ = origin.z;
        const maxZ = origin.z + MESH_WORLD_SIZE_Z;

        for (let x = 0; x < this.gridX; x++) {
            this.cells[x] = [];
            for (let y = 0; y < this.gridY; y++) {
                const worldX = origin.x + x * MESH_CELL_SIZE_XY;
                const worldY = origin.y + y * MESH_CELL_SIZE_XY;

                this.cells[x][y] = this.voxelizeColumn(worldX, worldY, minZ, maxZ);
            }
        }
        this.erode(MESH_ERODE_RADIUS);
    }

    /**
     * @param {number} wx
     * @param {number} wy
     * @param {number} minZ
     * @param {number} maxZ
     */
    voxelizeColumn(wx, wy, minZ, maxZ) {
        let head = null;
        let lastSpan = null;
        let currentZ = maxZ;
        const radius = MESH_CELL_SIZE_XY / 2+ MESH_HOLE_FILLING;

        while (currentZ >= minZ + radius) {
            //寻找地板 (floor)
            const downStart = { x: wx, y: wy, z: currentZ };
            const downEnd = { x: wx, y: wy, z: minZ };
            const downTr = Instance.TraceSphere({ radius, start: downStart, end: downEnd, ignorePlayers: true });

            if (!downTr || !downTr.didHit) break; // 下面没东西了，结束

            const floorZ = downTr.end.z - radius;

            //从地板向上寻找天花板 (ceiling)
            const upStart = { x: wx, y: wy, z: downTr.end.z + 1 };
            const upEnd = { x: wx, y: wy, z: maxZ };
            const upTr = Instance.TraceSphere({ radius, start: upStart, end: upEnd, ignorePlayers: true });

            let ceilingZ = maxZ;
            if (upTr.didHit) ceilingZ = upTr.end.z + radius;

            const floor = Math.round((floorZ - origin.z) / MESH_CELL_SIZE_Z);
            const ceiling = Math.round((ceilingZ - origin.z) / MESH_CELL_SIZE_Z);

            if ((ceiling - floor) >= AGENT_HEIGHT) {
                const newSpan = new OpenSpan(floor, ceiling, this.SPAN_ID++);

                if (!head || floor < head.floor) {
                    newSpan.next = head;
                    head = newSpan;
                } else {
                    let curr = head;
                    while (curr.next && curr.next.floor < floor) {
                        curr = curr.next;
                    }
                    newSpan.next = curr.next;
                    curr.next = newSpan;
                }
            }

            currentZ = floorZ - radius - 1;
        }

        return head;
    }

    abandoninit() {
        const radius = MESH_CELL_SIZE_XY / 2 + MESH_HOLE_FILLING;
        for (let i = 0; i < this.gridX; i++) {
            this.precells[i] = new Array(this.gridY);
            const wx = origin.x + i * MESH_CELL_SIZE_XY;

            for (let j = 0; j < this.gridY; j++) {
                const wy = origin.y + j * MESH_CELL_SIZE_XY;
                this.precells[i][j] = new Array(this.gridZ).fill(true);
                // 检查该列是否有地面
                const h = traceGroundpd({ x: wx, y: wy, z: origin.z })
                if (!h || !h.didHit) continue;
                for (let k = 0; k < this.gridZ; k++) {
                    const wz = origin.z + k * MESH_CELL_SIZE_Z;
                    //let ll=Instance.TraceBox({mins:getmins({ x: wx, y: wy, z: wz}),maxs:getmaxs({ x: wx, y: wy, z: wz}),start:{ x: wx, y: wy, z: wz},end:{ x: wx, y: wy, z: wz},ignorePlayers:true});
                    let ll = Instance.TraceSphere({ radius, start: { x: wx, y: wy, z: wz + MESH_CELL_SIZE_Z / 2 }, end: { x: wx, y: wy, z: wz - MESH_CELL_SIZE_Z / 2 }, ignorePlayers: true });
                    if (!ll || !ll.didHit) this.precells[i][j][k] = false;//是空气
                    else this.precells[i][j][k] = true;//是实体
                }
            }
        }
        for (let i = 0; i < this.gridX; i++) {
            this.cells[i] = new Array(this.gridY);
            const wx = origin.x + i * MESH_CELL_SIZE_XY;

            for (let j = 0; j < this.gridY; j++) {
                const wy = origin.y + j * MESH_CELL_SIZE_XY;
                // 检查该列是否有地面
                const h = traceGroundpd({ x: wx, y: wy, z: origin.z })
                if (!h || !h.didHit) continue;
                let currentSpan = null;
                let spanStart = -1;
                let inair = false;
                for (let k = 0; k < this.gridZ; k++) {
                    if (this.precells[i][j][k]) {
                        if (spanStart != -1 && inair == true) {
                            if (k - 1 - spanStart >= AGENT_HEIGHT) {
                                const span = new OpenSpan(spanStart, k - 1, this.SPAN_ID++);

                                if (currentSpan === null) {
                                    this.cells[i][j] = span;
                                } else {
                                    currentSpan.next = span;
                                }
                                currentSpan = span;
                            }
                        }
                        inair = false;
                    } else {
                        if (inair == false && k != 0) spanStart = k;
                        inair = true;
                    }
                }

                // 处理最后一个跨度
                if (inair == true && spanStart != -1 && this.gridZ - 1 - spanStart >= AGENT_HEIGHT) {
                    const span = new OpenSpan(spanStart, this.gridZ - 1, this.SPAN_ID++);
                    if (currentSpan === null) {
                        this.cells[i][j] = span;
                    } else {
                        currentSpan.next = span;
                    }
                }
            }
        }
    }
    //筛选不能去的区域
    findcanwalk() {
        const slist = Instance.FindEntitiesByClass("info_target");
        /**@type {Entity|undefined} */
        let s;
        slist.forEach((i) => {
            if (i.GetEntityName() == "navmesh") {
                s = i;
                return;
            }
        });
        if (!s) return;
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];
        let vis = Array(this.SPAN_ID + 5).fill(false);
        const centerPos = s.GetAbsOrigin();
        const cx = Math.ceil((centerPos.x - origin.x) / MESH_CELL_SIZE_XY);
        const cy = Math.ceil((centerPos.y - origin.y) / MESH_CELL_SIZE_XY);
        const cz = Math.ceil((centerPos.z - origin.z) / MESH_CELL_SIZE_Z) + 2;
        /**@type {OpenSpan|null} */
        let startSpan = this.cells[cx][cy];
        while (startSpan) {
            if (cz <= startSpan.ceiling && cz >= startSpan.floor) break;
            startSpan = startSpan.next;
        }
        if (!startSpan) return;
        let queue = [{ span: startSpan, i: cx, j: cy }];
        vis[startSpan.id] = true;
        while (queue.length > 0) {
            let currentSpan = queue.shift();
            if (!currentSpan) break;
            for (let dir = 0; dir < 4; dir++) {
                const nx = currentSpan.i + dirs[dir].dx;
                const ny = currentSpan.j + dirs[dir].dy;
                if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;
                /**@type {OpenSpan|null} */
                let neighbor = this.cells[nx][ny];
                while (neighbor) {
                    if (!vis[neighbor.id]) {
                        // 检查是否可以通过
                        if (currentSpan.span.canTraverseTo(neighbor, MAX_JUMP_HEIGHT, AGENT_HEIGHT)) {
                            vis[neighbor.id] = true;
                            queue.push({ span: neighbor, i: nx, j: ny });
                        }
                    }
                    neighbor = neighbor.next;
                }
            }
        }
        // 遍历所有的cell
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let prevSpan = null;
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if (!vis[currentSpan.id]) {
                        // 如果当前span不可达，则删除它
                        if (prevSpan) {
                            // 如果有前驱节点，跳过当前节点
                            prevSpan.next = currentSpan.next;
                        } else {
                            // 如果没有前驱节点，说明是链表的第一个节点，直接修改
                            this.cells[i][j] = currentSpan.next;
                        }
                    } else {
                        // 如果当前span是可达的，更新prevSpan
                        prevSpan = currentSpan;
                    }
                    // 继续遍历下一个span
                    currentSpan = currentSpan.next;
                }
            }
        }
    }
    /**
     * 根据半径腐蚀可行走区域
     * @param {number} radius
     */
    erode(radius) {
        if (radius <= 0) return;

        // 1. 初始化距离场，默认给一个很大的值
        // 使用 Uint16Array 节省内存，索引为 span.id
        const distances = new Uint16Array(this.SPAN_ID + 1).fill(65535);
        const dirs = [{ dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 1, dy: 0 }, { dx: 0, dy: -1 }];

        // 2. 标记边界点（距离为 0）
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let span = this.cells[i][j];
                while (span) {
                    let isBoundary = false;
                    for (let d = 0; d < 4; d++) {
                        const nx = i + dirs[d].dx;
                        const ny = j + dirs[d].dy;

                        // 触碰地图边界或没有邻居，即为边界
                        if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                            isBoundary = true;
                            break;
                        }

                        let hasNeighbor = false;
                        let nspan = this.cells[nx][ny];
                        while (nspan) {
                            if (span.canTraverseTo(nspan,MAX_JUMP_HEIGHT,AGENT_HEIGHT)) {
                                hasNeighbor = true;
                                break;
                            }
                            nspan = nspan.next;
                        }

                        if (!hasNeighbor) {
                            isBoundary = true;
                            break;
                        }
                    }

                    if (isBoundary) distances[span.id] = 0;
                    span = span.next;
                }
            }
        }

        // 3. 两次遍历计算精确距离场 (Pass 1: Top-Left to Bottom-Right)
        this._passDist(distances, true);
        // (Pass 2: Bottom-Right to Top-Left)
        this._passDist(distances, false);

        // 4. 根据 AGENT_RADIUS 删除不合格的 Span
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let prevSpan = null;
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    // 如果距离边界太近，则剔除
                    if (distances[currentSpan.id] < radius) {
                        if (prevSpan) prevSpan.next = currentSpan.next;
                        else this.cells[i][j] = currentSpan.next;
                    } else {
                        prevSpan = currentSpan;
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
    }

    /**
     * 内部辅助：距离场传递
     * @param {Uint16Array<ArrayBuffer>} distances
     * @param {boolean} forward
     */
    _passDist(distances, forward) {
        const dirs = [{ dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 1, dy: 0 }, { dx: 0, dy: -1 }];
        const startX = forward ? 0 : this.gridX - 1;
        const endX = forward ? this.gridX : -1;
        const step = forward ? 1 : -1;

        for (let i = startX; i !== endX; i += step) {
            for (let j = forward ? 0 : this.gridY - 1; j !== (forward ? this.gridY : -1); j += step) {
                let span = this.cells[i][j];
                while (span) {
                    for (let d = 0; d < 4; d++) {
                        const nx = i + dirs[d].dx;
                        const ny = j + dirs[d].dy;
                        if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;

                        let nspan = this.cells[nx][ny];
                        while (nspan) {
                            if (span.canTraverseTo(nspan,MAX_JUMP_HEIGHT,AGENT_HEIGHT)) {
                                // 核心公式：当前点距离 = min(当前距离, 邻居距离 + 1)
                                distances[span.id] = Math.min(distances[span.id], distances[nspan.id] + 1);
                            }
                            nspan = nspan.next;
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    deleteboundary() {
        let boundary = Array(this.SPAN_ID + 5).fill(false);
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];
        // 遍历所有的cell
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let neighbors = [false, false, false, false];
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    for (let d = 0; d < 4; d++) {
                        const nx = i + dirs[d].dx;
                        const ny = j + dirs[d].dy;
                        if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;
                        /**@type {OpenSpan|null} */
                        let nspan = this.cells[nx][ny];
                        while (nspan) {
                            if (span.canTraverseTo(nspan)) {
                                neighbors[d] = true;
                            }
                            nspan = nspan.next;
                        }
                    }
                    if (!(neighbors[0] && neighbors[1] && neighbors[2] && neighbors[3])) {
                        boundary[span.id] = true;
                    }
                    span = span.next;
                }
            }
        }
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let prevSpan = null;
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if (boundary[currentSpan.id]) {
                        // 如果当前span不可达，则删除它
                        if (prevSpan) {
                            // 如果有前驱节点，跳过当前节点
                            prevSpan.next = currentSpan.next;
                        } else {
                            // 如果没有前驱节点，说明是链表的第一个节点，直接修改
                            this.cells[i][j] = currentSpan.next;
                        }
                    } else {
                        // 如果当前span是可达的，更新prevSpan
                        prevSpan = currentSpan;
                    }
                    // 继续遍历下一个span
                    currentSpan = currentSpan.next;
                }
            }
        }
    }
    deletealone() {
        let del = Array(this.SPAN_ID + 5).fill(false);
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];
        // 遍历所有的cell
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let neighbors = [false, false, false, false];
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    for (let d = 0; d < 4; d++) {
                        const nx = i + dirs[d].dx;
                        const ny = j + dirs[d].dy;
                        if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;
                        /**@type {OpenSpan|null} */
                        let nspan = this.cells[nx][ny];
                        while (nspan) {
                            if (span.canTraverseTo(nspan)) {
                                neighbors[d] = true;
                            }
                            nspan = nspan.next;
                        }
                    }
                    if ((neighbors[0] ? 1 : 0) + (neighbors[1] ? 1 : 0) + (neighbors[2] ? 1 : 0) + (neighbors[3] ? 1 : 0) <= 1) {
                        del[span.id] = true;
                    }
                    span = span.next;
                }
            }
        }
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                let prevSpan = null;
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if (del[currentSpan.id]) {
                        // 如果当前span不可达，则删除它
                        if (prevSpan) {
                            // 如果有前驱节点，跳过当前节点
                            prevSpan.next = currentSpan.next;
                        } else {
                            // 如果没有前驱节点，说明是链表的第一个节点，直接修改
                            this.cells[i][j] = currentSpan.next;
                        }
                    } else {
                        prevSpan = currentSpan;
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
    }
    debug(duration = 30) {
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    const c = {
                        r: 255,
                        g: 255,
                        b: 0
                    };
                    Instance.DebugSphere({
                        center: {
                            x: origin.x + i * MESH_CELL_SIZE_XY - MESH_CELL_SIZE_XY / 2,
                            y: origin.y + j * MESH_CELL_SIZE_XY - MESH_CELL_SIZE_XY / 2,
                            z: origin.z + span.floor * MESH_CELL_SIZE_Z
                        },
                        radius: 3,
                        duration,
                        color: c
                    });
                    //Instance.DebugBox({ 
                    //    mins: { 
                    //        x: origin.x+i*MESH_CELL_SIZE_XY - MESH_CELL_SIZE_XY/2, 
                    //        y: origin.y+j*MESH_CELL_SIZE_XY - MESH_CELL_SIZE_XY/2, 
                    //        z: origin.z +span.floor*MESH_CELL_SIZE_Z
                    //    },
                    //    maxs: { 
                    //        x: origin.x+i*MESH_CELL_SIZE_XY + MESH_CELL_SIZE_XY/2, 
                    //        y: origin.y+j*MESH_CELL_SIZE_XY + MESH_CELL_SIZE_XY/2, 
                    //        z: origin.z +span.floor*MESH_CELL_SIZE_Z+5
                    //    }, 
                    //    duration: duration, 
                    //    color: c
                    //});
                    span = span.next;
                }
            }
        }
    }
}