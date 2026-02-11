
import { Entity, Instance } from "cs_script/point_script";
import { origin, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, MESH_WORLD_SIZE_XY, MESH_WORLD_SIZE_Z, AGENT_HEIGHT, MAX_JUMP_HEIGHT, MESH_ERODE_RADIUS, AGENT_RADIUS, MAX_WALK_HEIGHT, MESH_TRACE_SIZE_Z } from "./path_const";
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

        this.mins={x:-MESH_CELL_SIZE_XY/2,y:-MESH_CELL_SIZE_XY/2,z:-MESH_TRACE_SIZE_Z/2};
        this.maxs={x:MESH_CELL_SIZE_XY/2,y:MESH_CELL_SIZE_XY/2,z:MESH_TRACE_SIZE_Z/2};
    }
    init() {
        const minZ = origin.z;
        const maxZ = origin.z + MESH_WORLD_SIZE_Z;
        let start=new Date();
        Instance.Msg(`完成百分比`)
        for (let x = 0; x < this.gridX; x++) {
            this.cells[x] = [];
            for (let y = 0; y < this.gridY; y++) {
                const worldX = origin.x + x * MESH_CELL_SIZE_XY;
                const worldY = origin.y + y * MESH_CELL_SIZE_XY;

                this.cells[x][y] = this.voxelizeColumn(worldX, worldY, minZ, maxZ);
            }
            let end = new Date();
            if(end.getTime()-start.getTime()>1000)
            {
                Instance.Msg(`地图扫描百分比${Math.round(100*((x+1)/this.gridX))}%`);
                start=end;
            }
        }
        Instance.Msg(`地图扫描完成，开始筛选可达区域`);
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
        let currentZ = maxZ;
        const radius = MESH_TRACE_SIZE_Z/2;

        while (currentZ >= minZ + radius) {
            //寻找地板 (floor)
            const downStart = { x: wx, y: wy, z: currentZ };
            const downEnd = { x: wx, y: wy, z: minZ };
            const downTr = Instance.TraceBox({ mins:this.mins,maxs:this.maxs, start: downStart, end: downEnd, ignorePlayers: true });
            if (!downTr || !downTr.didHit) break; // 下面没东西了，结束

            const floorZ = downTr.end.z - radius;

            //从地板向上寻找天花板 (ceiling)
            const upStart = { x: wx, y: wy, z: downTr.end.z + 1 };
            const upEnd = { x: wx, y: wy, z: maxZ };
            const upTr = Instance.TraceBox({ mins:this.mins,maxs:this.maxs, start: upStart, end: upEnd, ignorePlayers: true });
            
            let ceilingZ = maxZ;
            if (upTr.didHit) ceilingZ = upTr.end.z + radius;

            const floor = Math.round(floorZ - origin.z);
            const ceiling = Math.round(ceilingZ - origin.z);

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
    /**
     * 计算某个方向能延伸的距离
     * @param {number} startI 起点x
     * @param {number} startJ 起点y
     * @param {number} dirI 方向x
     * @param {number} dirJ 方向y
     * @param {OpenSpan} span 当前span
     * @returns {number} 能延伸的dist
     */
    calcDirectionDist(startI, startJ, dirI, dirJ, span) {
        let dist = 1; // 至少包括自己
        let currentI = startI;
        let currentJ = startJ;
        let sspan=new OpenSpan(span.floor,span.ceiling,-1);
        while (true) {
            const nextI = currentI + dirI;
            const nextJ = currentJ + dirJ;
            
            // 检查边界
            if (nextI < 0 || nextJ < 0 || nextI >= this.gridX || nextJ >= this.gridY) break;
            
            const neighbors = this.cells[nextI][nextJ];
            if (!neighbors) break;
            
            let hasPassable = false;
            /**@type {OpenSpan|null} */
            let neighborSpan = neighbors;
            
            // 找到能通行的span
            while (neighborSpan) {
                if (neighborSpan.use) {
                    // 检查高度差：能上升MAX_WALK_HEIGHT，能下降MAX_JUMP_HEIGHT
                    if (sspan.canTo(neighborSpan)) {
                        sspan.floor = Math.max(sspan.floor, neighborSpan.floor);
                        sspan.ceiling = Math.min(sspan.ceiling, neighborSpan.ceiling);
                        hasPassable = true;
                        break;
                    }
                }
                neighborSpan = neighborSpan.next;
            }
            
            if (!hasPassable) break;
            
            dist++;
            currentI = nextI;
            currentJ = nextJ;
        }
        
        return dist;
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
        // 检查每个可达的span在四个方向上的宽度是否足够
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    // 计算四个方向的可通行距离
                    const distLeft = this.calcDirectionDist(i, j, -1, 0, currentSpan);
                    const distRight = this.calcDirectionDist(i, j, 1, 0, currentSpan);
                    const distTop = this.calcDirectionDist(i, j, 0, 1, currentSpan);
                    const distBottom = this.calcDirectionDist(i, j, 0, -1, currentSpan);
                    
                    // 检查X方向和Y方向的总宽度是否满足AGENT_RADIUS要求
                    const totalX = distLeft + distRight - 1; // -1避免重复计算自己
                    const totalY = distTop + distBottom - 1; // -1避免重复计算自己
                    
                    if (totalX < AGENT_RADIUS || totalY < AGENT_RADIUS) {
                        currentSpan.use = false;
                    }
                    currentSpan = currentSpan.next;
                }
            }
        }
        // 基于可达性进行BFS标记
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
            if (startSpan.use&&cz <= startSpan.ceiling && cz >= startSpan.floor) break;
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
                    if(neighbor.use)
                    {
                        if (!vis[neighbor.id]) {
                            // 检查是否可以通过（基于高度差）
                            if (currentSpan.span.canTraverseTo(neighbor, MAX_JUMP_HEIGHT, AGENT_HEIGHT)) {
                                vis[neighbor.id] = true;
                                queue.push({ span: neighbor, i: nx, j: ny });
                            }
                        }
                    }
                    neighbor = neighbor.next;
                }
            }
        }
        
        // 删除不可达的span
        for (let i = 0; i < this.gridX; i++) {
            for (let j = 0; j < this.gridY; j++) {
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        if (!vis[currentSpan.id]) {
                            // 如果当前span不可达，则删除它
                            currentSpan.use=false;
                        }
                    }
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
                    if(span.use)
                    {
                        let isBoundary = false;
                        let hasNeighbor = false;
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;

                            // 触碰地图边界或没有邻居，即为边界
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) {
                                isBoundary = true;
                                break;
                            }

                            let nspan = this.cells[nx][ny];
                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan,MAX_JUMP_HEIGHT,AGENT_HEIGHT)) {
                                        hasNeighbor = true;
                                        break;
                                    }
                                }
                                nspan = nspan.next;
                            }
                            if (hasNeighbor) {
                                break;
                            }
                        }
                        if (!hasNeighbor)isBoundary = true;
                        if (isBoundary) distances[span.id] = 0;
                    }
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
                //let prevSpan = null;
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        // 如果距离边界太近，则剔除
                        if (distances[currentSpan.id] < radius) {
                            currentSpan.use=false;
                        }
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
                    if(span.use)
                    {
                        for (let d = 0; d < 4; d++) {
                            const nx = i + dirs[d].dx;
                            const ny = j + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;

                            let nspan = this.cells[nx][ny];
                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan,MAX_JUMP_HEIGHT,AGENT_HEIGHT)) {
                                        // 核心公式：当前点距离 = min(当前距离, 邻居距离 + 1)
                                        distances[span.id] = Math.min(distances[span.id], distances[nspan.id] + 1);
                                    }
                                }
                                nspan = nspan.next;
                            }
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
                /**@type {OpenSpan|null} */
                let span = this.cells[i][j];
                while (span) {
                    let neighbors = [false, false, false, false];
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
                //let prevSpan = null;
                /**@type {OpenSpan|null} */
                let currentSpan = this.cells[i][j];
                while (currentSpan) {
                    if(currentSpan.use)
                    {
                        if (boundary[currentSpan.id]) {
                            currentSpan.use=false;
                        }
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
                    if(span.use==true)
                    {
                        const c = {
                            r: 255,
                            g: 255,
                            b: 0
                        };
                        Instance.DebugSphere({
                            center: {
                                x: origin.x + i * MESH_CELL_SIZE_XY,
                                y: origin.y + j * MESH_CELL_SIZE_XY,
                                z: origin.z + span.floor * MESH_CELL_SIZE_Z
                            },
                            radius: 3,
                            duration,
                            color: c
                        });
                    }
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