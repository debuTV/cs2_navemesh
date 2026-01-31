import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";
import { origin, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, REGION_MERGE_AREA, REGION_MIN_AREA } from "./path_const";
import { Instance } from "cs_script/point_script";

export class RegionGenerator {
    /**
     * @param {OpenHeightfield} openHeightfield
     */
    constructor(openHeightfield) {
        /**@type {(OpenSpan|null)[][]} */
        this.hf = openHeightfield.cells;
        /**@type {number} */
        this.gridX = openHeightfield.gridX;
        /**@type {number} */
        this.gridY = openHeightfield.gridY;
        this.regions = [];
        /**@type {number} */
        this.nextRegionId = 1;
    }

    init() {
        this.buildCompactNeighbors();
        this.buildDistanceField();
        this.buildRegionsWatershed();
        this.mergeAndFilterRegions();
    }
    //为span建立邻居关系
    buildCompactNeighbors() {
        const dirs = [
            { dx: -1, dy: 0 },
            { dx: 0, dy: 1 },
            { dx: 1, dy: 0 },
            { dx: 0, dy: -1 }
        ];

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        span.neighbors = [null, null, null, null];

                        for (let d = 0; d < 4; d++) {
                            const nx = x + dirs[d].dx;
                            const ny = y + dirs[d].dy;
                            if (nx < 0 || ny < 0 || nx >= this.gridX || ny >= this.gridY) continue;

                            let best = null;
                            let bestDiff = Infinity;
                            /**@type {OpenSpan|null} */
                            let nspan = this.hf[nx][ny];

                            while (nspan) {
                                if(nspan.use)
                                {
                                    if (span.canTraverseTo(nspan)) {
                                        const diff = Math.abs(span.floor - nspan.floor);
                                        if (diff < bestDiff) {
                                            best = nspan;
                                            bestDiff = diff;
                                        }
                                    }
                                }
                                nspan = nspan.next;
                            }

                            span.neighbors[d] = best;
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    /**
     * 获取邻居。
     * @param {OpenSpan} span 
     * @param {number} dir 方向 (0:W, 1:N, 2:E, 3:S)
     * @returns {OpenSpan|null}
     */
    getNeighbor(span, dir) {
        return span.neighbors[dir];
    }

    /**
     * 获取对角线邻居。
     * 例如：西北 (NW) = 先向西(0)再向北(1)
     * @param {OpenSpan} span 
     * @param {number} dir1 
     * @param {number} dir2 
     */
    getDiagonalNeighbor(span, dir1, dir2) {
        const n = span.neighbors[dir1];
        if (n) {
            return n.neighbors[dir2];
        }
        return null;
    }
    //构建距离场
    buildDistanceField() {
        // 1. 初始化：边界设为0，内部设为无穷大
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        // 如果任意一个邻居缺失，说明是边界
                        span.distance = this.isBorderSpan(span) ? 0 : Infinity;
                    }
                    span = span.next;
                }
            }
        }

        // 第一遍扫描：从左下到右上
        // 西(0)、西南(0+3)、南(3)、东南(3+2)
        for (let y = 0; y < this.gridY; y++) {
            for (let x = 0; x < this.gridX; x++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.distance > 0) {
                            // 西
                            let n = this.getNeighbor(span, 0);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 西南
                            let nd = this.getDiagonalNeighbor(span, 0, 3);
                            if (nd) span.distance = Math.min(span.distance, nd.distance + 3);
                            // 南
                            n = this.getNeighbor(span, 3);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 东南
                            nd = this.getDiagonalNeighbor(span, 3, 2);
                            if (nd) span.distance = Math.min(span.distance, nd.distance + 3);
                        }
                    }
                    span = span.next;
                }
            }
        }

        // 第二遍扫描：从右上到左下
        // 东(2)、东北(2+1)、北(1)、西北(1+0)
        for (let y = this.gridY - 1; y >= 0; y--) {
            for (let x = this.gridX - 1; x >= 0; x--) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.distance > 0) {
                            // 东
                            let n = this.getNeighbor(span, 2);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 东北
                            let nd = this.getDiagonalNeighbor(span, 2, 1);
                            if (nd) span.distance = Math.min(span.distance, nd.distance + 3);
                            // 北
                            n = this.getNeighbor(span, 1);
                            if (n) span.distance = Math.min(span.distance, n.distance + 2);
                            // 西北
                            let nd2 = this.getDiagonalNeighbor(span, 1, 0);
                            if (nd2) span.distance = Math.min(span.distance, nd2.distance + 3);
                        }
                    }
                    span = span.next;
                }
            }
        }
        this.blurDistanceField();
    }
    //对距离场进行平滑处理
    blurDistanceField() {
        const threshold = 2; //距离阈值，小于的不进行模糊

        //计算模糊后的值
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        //只有远离边界的体素才参与模糊
                        if (span.distance <= threshold) span.newDist = span.distance;
                        else {
                            let d = span.distance;
                            //计算平均距离
                            for (let i = 0; i < 4; i++) {
                                const n = span.neighbors[i];
                                if (n) d += n.distance;
                                else d += span.distance;
                            }
                            span.newDist = Math.floor((d + 2) / 5);
                        }
                    }
                    span = span.next;
                }
            }
        }
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.newDist !== undefined) {
                            span.distance = span.newDist;
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    /**
     * 计算当前span从dir方向过来的距离
     * @param {OpenSpan} span
     * @param {number} dir
     */
    sample(span, dir) {
        const n = span.neighbors[dir];
        if (!n) return Infinity;

        return n.distance + 2;
    }

    /**
     * 是否是边界span
     * @param {OpenSpan} span
     */
    isBorderSpan(span) {
        for (let d = 0; d < 4; d++) {
            if (!span.neighbors[d]) return true;
        }
        return false;
    }

    //洪水扩张
    buildRegionsWatershed() {
        const spans = [];
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        span.regionId = 0;
                        if (span.distance >= 0) {
                            spans.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        //从大到小排序
        spans.sort((a, b) => b.distance - a.distance);

        for (const span of spans) {

            let bestRegion = 0;
            let maxNeighborDist = -1;

            for (let d = 0; d < 4; d++) {
                const n = span.neighbors[d];
                if (!n) continue;

                //如果邻居已经有Region了，说明这个邻居比当前span更靠近“中心”

                if (n.regionId > 0) {
                    if (n.distance > maxNeighborDist) {
                        maxNeighborDist = n.distance;
                        bestRegion = n.regionId;
                    }
                }
            }

            if (bestRegion !== 0) span.regionId = bestRegion;
            else span.regionId = this.nextRegionId++;
        }
        //this.floodRemaining();
        //this.mergeAndFilterRegions();
    }
    //abandon
    floodRemaining() {
        //填补距离为0的边界缝隙
        let changed = true;
        let iterCount = 0;
        while (changed && iterCount < 5) {
            changed = false;
            iterCount++;
            for (let x = 0; x < this.gridX; x++) {
                for (let y = 0; y < this.gridY; y++) {
                    let span = this.hf[x][y];
                    while (span) {
                        if(span.use)
                        {
                            //如果当前没有区域
                            if (span.regionId === 0) {
                                let bestRegion = 0;
                                let bestDist = -1;
                                for (let d = 0; d < 4; d++) {
                                    const n = span.neighbors[d];
                                    if (n && n.regionId > 0) {
                                        if (n.distance > bestDist) {
                                            bestDist = n.distance;
                                            bestRegion = n.regionId;
                                        }
                                    }
                                }
                                if (bestRegion > 0) {
                                    span.regionId = bestRegion;
                                    changed = true;
                                }
                            }
                        }
                        span = span.next;
                    }
                }
            }
        }
    }

    //合并过滤小region
    mergeAndFilterRegions() {
        /**@type {Map<number,OpenSpan[]>} */
        const regionSpans = new Map();

        //统计每个region包含的span
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            if (!regionSpans.has(span.regionId)) regionSpans.set(span.regionId, []);
                            regionSpans.get(span.regionId)?.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        //合并过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MERGE_AREA) continue;
            const neighbors = new Map();
            for (const span of spans) {
                for (let d = 0; d < 4; d++) {
                    const n = span.neighbors[d];
                    if (n && n.regionId !== id) {
                        neighbors.set(
                            n.regionId,
                            (neighbors.get(n.regionId) ?? 0) + 1
                        );
                    }
                }
            }

            let best = 0;
            let bestCount = 0;
            for (const [nid, count] of neighbors) {
                if (count > bestCount) {
                    best = nid;
                    bestCount = count;
                }
            }

            if (best > 0) {
                for (const span of spans) {
                    span.regionId = best;
                    regionSpans.get(span.regionId)?.push(span);
                }
                regionSpans.set(id, []);
            }
        }
        //统计每个region包含的span
        regionSpans.clear();
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            if (!regionSpans.has(span.regionId)) regionSpans.set(span.regionId, []);
                            regionSpans.get(span.regionId)?.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        //忽略过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MIN_AREA) continue;
            for (const span of spans) {
                if (span.regionId == id) span.regionId = 0;
            }
        }
    }
    //abandon
    smooth() {
        /**@type {Map<number,OpenSpan[]>} */
        const regionSpans = new Map();
        //统计每个region包含的span
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            if (!regionSpans.has(span.regionId)) regionSpans.set(span.regionId, []);
                            regionSpans.get(span.regionId)?.push(span);
                        }
                    }
                    span = span.next;
                }
            }
        }
        for (const [id, spans] of regionSpans) {
            const neighbors = new Map();
            let smoth = true;
            while (smoth) {
                smoth = false;
                let diff = 0;
                let df = [];
                outer:
                for (const span of spans) {
                    if (span.regionId != id) continue;
                    for (let d = 0; d < 4; d++) {
                        const n = span.neighbors[d];
                        if (n && n.regionId !== id) {
                            df[diff] = n.regionId;
                            diff++;
                        }
                    }
                    if (diff == 3) {
                        //这个span周围有三个不同于自身的区域
                        if (df[0] == df[1]) {
                            span.regionId = df[0];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        else if (df[1] == df[2]) {
                            span.regionId = df[1];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        else if (df[0] == df[2]) {
                            span.regionId = df[0];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        else {
                            //四个区域不相同加入旁边
                            span.regionId = df[0];
                            regionSpans.get(span.regionId)?.push(span);
                        }
                        smoth = true;
                        break outer;
                    }
                }
            }
        }
    }
    /**
     * Debug: 绘制 Region（按 regionId 上色）
     * @param {number} duration
     */
    debugDrawRegions(duration = 5) {
        const colorCache = new Map();

        const randomColor = (/** @type {number} */ id) => {
            if (!colorCache.has(id)) {
                colorCache.set(id, {
                    r: (id * 97) % 255,
                    g: (id * 57) % 255,
                    b: (id * 17) % 255
                });
            }
            return colorCache.get(id);
        };

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.regionId > 0) {
                            const c = randomColor(span.regionId);

                            const center = {
                                x: origin.x + (x + 0.5) * MESH_CELL_SIZE_XY,
                                y: origin.y + (y + 0.5) * MESH_CELL_SIZE_XY,
                                z: origin.z + span.floor * MESH_CELL_SIZE_Z
                            };

                            Instance.DebugSphere({
                                center,
                                radius: MESH_CELL_SIZE_XY * 0.3,
                                color: c,
                                duration
                            });
                        }
                    }
                    span = span.next;
                }
            }
        }
    }
    /**
     * Debug: 绘制 Distance Field（亮度 = 距离）
     */
    debugDrawDistance(duration = 5) {
        let maxDist = 0;

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        maxDist = Math.max(maxDist, span.distance);
                    }
                    span = span.next;
                }
            }
        }

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                /**@type {OpenSpan|null} */
                let span = this.hf[x][y];
                while (span) {
                    if(span.use)
                    {
                        if (span.distance < Infinity) {
                            const t = span.distance / maxDist;
                            const c = {
                                r: Math.floor(255 * t),
                                g: Math.floor(255 * (1 - t)),
                                b: 0
                            };

                            Instance.DebugSphere({
                                center: {
                                    x: origin.x + x * MESH_CELL_SIZE_XY,
                                    y: origin.y + y * MESH_CELL_SIZE_XY,
                                    z: origin.z + span.floor * MESH_CELL_SIZE_Z
                                },
                                radius: MESH_CELL_SIZE_XY * 0.25,
                                color: c,
                                duration
                            });
                        }
                    }
                    span = span.next;
                }
            }
        }
    }

}
