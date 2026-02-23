import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";
import { origin, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, REGION_MIN_AREA, REGION_MERGE_AREA } from "./path_const";
import { Instance } from "cs_script/point_script";

export class RegionGenerator {
    /**
     * @param {OpenHeightfield} openHeightfield
     */
    constructor(openHeightfield) {
        this.hf = openHeightfield.cells;
        this.baseX = openHeightfield.baseX;
        this.baseY = openHeightfield.baseY;

        this.gridX = openHeightfield.gridX;
        this.gridY = openHeightfield.gridY;

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
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
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
                                if(OpenSpan.getUse(nspanId))
                                {
                                    if (OpenSpan.canTraverseTo(spanId, nspanId)) {
                                        const diff = Math.abs(OpenSpan.getFloor(spanId) - OpenSpan.getFloor(nspanId));
                                        if (diff < bestDiff) {
                                            best = nspanId;
                                            bestDiff = diff;
                                        }
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
     * 获取对角线邻居。
     * 例如：西北 (NW) = 先向西(0)再向北(1)
     * @param {number} spanId 
     * @param {number} dir1 
     * @param {number} dir2 
     * @returns {number} 邻居spanId，0表示无邻居
     */
    getDiagonalNeighbor(spanId, dir1, dir2) {
        const first = OpenSpan.getNeighbor(spanId, dir1);
        if (first !== 0) {
            const diagonal = OpenSpan.getNeighbor(first, dir2);
            if (diagonal !== 0) return diagonal;
        }

        const second = OpenSpan.getNeighbor(spanId, dir2);
        if (second !== 0) {
            return OpenSpan.getNeighbor(second, dir1);
        }

        return 0;
    }
    //构建距离场
    buildDistanceField() {
        // 1. 初始化：边界设为0，内部设为无穷大
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        // 如果任意一个邻居缺失，说明是边界
                        OpenSpan.setDistance(spanId, this.isBorderSpan(spanId) ? 0 : Infinity);
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 第一遍扫描：从左下到右上
        // 西(0)、西南(0+3)、南(3)、东南(3+2)
        for (let y = 0; y < this.gridY; y++) {
            for (let x = 0; x < this.gridX; x++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getDistance(spanId) > 0) {
                            // 西
                            let n = OpenSpan.getNeighbor(spanId, 0);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 西南
                            let nd = this.getDiagonalNeighbor(spanId, 0, 3);
                            if (nd !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd) + 3));
                            // 南
                            n = OpenSpan.getNeighbor(spanId, 3);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 东南
                            nd = this.getDiagonalNeighbor(spanId, 3, 2);
                            if (nd !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd) + 3));
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 第二遍扫描：从右上到左下
        // 东(2)、东北(2+1)、北(1)、西北(1+0)
        for (let y = this.gridY - 1; y >= 0; y--) {
            for (let x = this.gridX - 1; x >= 0; x--) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getDistance(spanId) > 0) {
                            // 东
                            let n = OpenSpan.getNeighbor(spanId, 2);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 东北
                            let nd = this.getDiagonalNeighbor(spanId, 2, 1);
                            if (nd !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd) + 3));
                            // 北
                            n = OpenSpan.getNeighbor(spanId, 1);
                            if (n !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(n) + 2));
                            // 西北
                            let nd2 = this.getDiagonalNeighbor(spanId, 1, 0);
                            if (nd2 !== 0) OpenSpan.setDistance(spanId, Math.min(OpenSpan.getDistance(spanId), OpenSpan.getDistance(nd2) + 3));
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
        // 第二遍扫描后，distance 场已经稳定了，可以用来做降噪了
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        let all=OpenSpan.getDistance(spanId);
                        let n = OpenSpan.getNeighbor(spanId, 0);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = OpenSpan.getNeighbor(spanId, 1);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = OpenSpan.getNeighbor(spanId, 2);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = OpenSpan.getNeighbor(spanId, 3);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);

                        n = this.getDiagonalNeighbor(spanId, 0,3);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = this.getDiagonalNeighbor(spanId, 0,1);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);

                        n = this.getDiagonalNeighbor(spanId, 2,3);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);
                        n = this.getDiagonalNeighbor(spanId, 2,1);
                        if (n !== 0)all+=OpenSpan.getDistance(n);
                        else all+=OpenSpan.getDistance(spanId);

                        // 如果任意一个邻居缺失，说明是边界
                        OpenSpan.setDenoiseDistance(spanId, all/9);
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

    /**
     * 是否是边界span
     * @param {number} spanId
     */
    isBorderSpan(spanId) {
        for (let d = 0; d < 4; d++) {
            if (OpenSpan.getNeighbor(spanId, d) === 0) return true;
        }
        return false;
    }

    //洪水扩张
    buildRegionsWatershed() {
        // 1) 按 denoiseDistance 收集所有可用 span，并重置 regionId
        //    distBuckets: 下标=距离值，value=该距离上的 span 列表
        /** @type {number[][]} */
        const distBuckets = [];
        let maxDist = 0;

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        OpenSpan.setRegionId(spanId, 0);
                        const dist = OpenSpan.getDenoiseDistance(spanId);
                        if (Number.isFinite(dist) && dist >= 0) {
                            const d = Math.floor(dist);
                            if (!distBuckets[d]) distBuckets[d] = [];
                            distBuckets[d].push(spanId);
                            if (d > maxDist) maxDist = d;
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        // 2) 生成“每隔2个距离一个批次”的批次列表（从大到小）
        //    这里的阈值计算会自然形成：当 maxDist 为偶数时，首批包含 d-2/d-1/d
        /** @type {number[][]} */
        const batches = [];
        let coveredMin = maxDist + 1;
        let level = (maxDist + 1) & ~1;

        while (coveredMin > 0) {
            const threshold = Math.max(level - 2, 0);
            const batch = [];

            for (let dist = coveredMin - 1; dist >= threshold; dist--) {
                const list = distBuckets[dist];
                if (list && list.length > 0) batch.push(...list);
            }

            if (batch.length > 0) batches.push(batch);

            coveredMin = threshold;
            level = Math.max(level - 2, 0);
        }

        // 3) 逐批处理（从高距离到低距离）
        for (const batch of batches) {
            // batchSet 用于 O(1) 判断邻居是否仍在当前批次内
            const batchSet = new Set(batch);

            // queue 是“旧水位”的广度扩张队列（BFS）
            // 只装入已经被赋予 region 的节点，向同批次未赋值节点扩散
            const queue = [];

            // 3.1 先尝试让本批次节点接入已有 region（来自历史批次或已处理节点）
            for (const spanId of batch) {
                if (OpenSpan.getRegionId(spanId) !== 0) {
                    queue.push(spanId);
                    continue;
                }

                let bestRegion = 0;
                let maxNeighborDist = -1;

                // 从4邻域中挑一个“最靠内”（距离更大）的已有 region 作为接入目标
                for (let d = 0; d < 4; d++) {
                    const n = OpenSpan.getNeighbor(spanId, d);
                    if (n === 0) continue;

                    const neighborRegion = OpenSpan.getRegionId(n);
                    if (neighborRegion === 0) continue;

                    const neighborDist = OpenSpan.getDenoiseDistance(n);
                    if (neighborDist > maxNeighborDist) {
                        maxNeighborDist = neighborDist;
                        bestRegion = neighborRegion;
                    }
                }

                if (bestRegion !== 0) {
                    OpenSpan.setRegionId(spanId, bestRegion);
                    queue.push(spanId);
                }
            }

            // 3.2 旧水位 BFS：在当前批次内，把已接入的 region 尽量向外扩散
            for (let q = 0; q < queue.length; q++) {
                const current = queue[q];
                const rid = OpenSpan.getRegionId(current);

                for (let d = 0; d < 4; d++) {
                    const n = OpenSpan.getNeighbor(current, d);
                    if (n === 0) continue;
                    if (!batchSet.has(n)) continue;
                    if (OpenSpan.getRegionId(n) !== 0) continue;

                    OpenSpan.setRegionId(n, rid);
                    queue.push(n);
                }
            }

            // 3.3 对仍未覆盖的节点创建新水位（新 region），并立即 DFS 泛洪
            for (const spanId of batch) {
                if (OpenSpan.getRegionId(spanId) !== 0) continue;

                const rid = this.nextRegionId++;
                OpenSpan.setRegionId(spanId, rid);

                // stack 是“新水位”深度扩张栈（DFS）
                const stack = [spanId];
                while (stack.length > 0) {
                    const current = stack.pop();
                    if (current === undefined) break;

                    for (let d = 0; d < 4; d++) {
                        const n = OpenSpan.getNeighbor(current, d);
                        if (n === 0) continue;
                        if (!batchSet.has(n)) continue;
                        if (OpenSpan.getRegionId(n) !== 0) continue;

                        OpenSpan.setRegionId(n, rid);
                        stack.push(n);
                    }
                }
            }
        }
    }
    //合并过滤小region
    mergeAndFilterRegions() {
        /**@type {Map<number,number[]>} */
        const regionSpans = new Map();

        //统计每个region包含的span
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            if (!regionSpans.has(OpenSpan.getRegionId(spanId))) regionSpans.set(OpenSpan.getRegionId(spanId), []);
                            regionSpans.get(OpenSpan.getRegionId(spanId))?.push(spanId);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
        //合并过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MERGE_AREA) continue;
            const neighbors = new Map();
            for (const spanId of spans) {
                for (let d = 0; d < 4; d++) {
                    const n = OpenSpan.getNeighbor(spanId, d);
                    if (n !== 0 && OpenSpan.getRegionId(n) !== id) {
                        neighbors.set(
                            OpenSpan.getRegionId(n),
                            (neighbors.get(OpenSpan.getRegionId(n)) ?? 0) + 1
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
                for (const spanId of spans) {
                    OpenSpan.setRegionId(spanId, best);
                    regionSpans.get(OpenSpan.getRegionId(spanId))?.push(spanId);
                }
                regionSpans.set(id, []);
            }
        }
        //统计每个region包含的span
        regionSpans.clear();
        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            if (!regionSpans.has(OpenSpan.getRegionId(spanId))) regionSpans.set(OpenSpan.getRegionId(spanId), []);
                            regionSpans.get(OpenSpan.getRegionId(spanId))?.push(spanId);
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
        //忽略过小的region
        for (const [id, spans] of regionSpans) {
            if (spans.length >= REGION_MIN_AREA) continue;
            for (const spanId of spans) {
                if (OpenSpan.getRegionId(spanId) == id) OpenSpan.setRegionId(spanId, 0);
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
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getRegionId(spanId) > 0) {
                            const c = randomColor(OpenSpan.getRegionId(spanId));

                            const center = {
                                x: origin.x + (this.baseX + x + 0.5) * MESH_CELL_SIZE_XY,
                                y: origin.y + (this.baseY + y + 0.5) * MESH_CELL_SIZE_XY,
                                z: origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z
                            };

                            Instance.DebugSphere({
                                center,
                                radius: 3,
                                color: c,
                                duration
                            });
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
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
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        maxDist = Math.max(maxDist, OpenSpan.getDistance(spanId));
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }

        for (let x = 0; x < this.gridX; x++) {
            for (let y = 0; y < this.gridY; y++) {
                let spanId = this.hf[x][y];
                while (spanId !== 0) {
                    if(OpenSpan.getUse(spanId))
                    {
                        if (OpenSpan.getDistance(spanId) < Infinity) {
                            const t = OpenSpan.getDistance(spanId) / maxDist;
                            const c = {
                                r: Math.floor(255 * t),
                                g: Math.floor(255 * (1 - t)),
                                b: 0
                            };

                            Instance.DebugSphere({
                                center: {
                                    x: origin.x + (this.baseX + x) * MESH_CELL_SIZE_XY,
                                    y: origin.y + (this.baseY + y) * MESH_CELL_SIZE_XY,
                                    z: origin.z + OpenSpan.getFloor(spanId) * MESH_CELL_SIZE_Z
                                },
                                radius: 3,
                                color: c,
                                duration
                            });
                        }
                    }
                    spanId = OpenSpan.getNext(spanId);
                }
            }
        }
    }

}
