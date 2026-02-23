import { AGENT_HEIGHT, MAX_WALK_HEIGHT, MESH_CELL_SIZE_XY, MESH_TRACE_SIZE_Z, MESH_WORLD_SIZE_XY, MESH_WORLD_SIZE_Z } from "./path_const";

// 计算最大可能的 span 数量
export const totalspan = ((MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1) * ((MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1) * ((MESH_WORLD_SIZE_Z / MESH_TRACE_SIZE_Z) + 1);

// SOA 结构 (Structure of Arrays) 内存布局优化
// 按属性分离存储，提高缓存局部性，减少内存碎片
export const floor = new Int16Array(totalspan);           // ±32k，足够存储高度差
export const ceiling = new Int16Array(totalspan);         // ±32k
export const next = new Uint32Array(totalspan);            // 指向下一个 span，最多 4G 个
export const regionId = new Uint16Array(totalspan);       // ±65k 个区域，足够
export const distance = new Uint16Array(totalspan);       // 距离场，最多 65k（腐蚀半径通常 <256）
export const denoisedistance = new Uint16Array(totalspan);       // 降噪距离场，最多 65k（腐蚀半径通常 <256）
export const neighbor = new Uint32Array(totalspan * 4);   // 每个 span 4 邻居（W,N,E,S），0 表示无邻居
export const use = new Uint8Array(Math.ceil(totalspan / 8)); // 位图：1 bit = 1 span，1 byte = 8 spans
export const DISTANCE_INF = 0xFFFF;

// 内存占用计算：
// Int16Array: 2 bytes * totalspan
// Int16Array: 2 bytes * totalspan
// Uint32Array: 4 bytes * totalspan
// Uint16Array: 2 bytes * totalspan
// Uint16Array: 2 bytes * totalspan
// Uint8Array: 1 byte * (totalspan/8)
// ≈ 13 bytes per span (vs 40+ bytes with object fields)

// ============ 纯函数式 API ============
// 所有操作都直接基于 ID，无需创建对象，内存占用为 0
export class OpenSpan{
    /**
     * 初始化一个 span
     * @param {number} id 
     * @param {number} m_floor 
     * @param {number} m_ceiling 
     */
    static initSpan(id, m_floor, m_ceiling) {
        floor[id] = m_floor;
        ceiling[id] = m_ceiling;
        next[id] = 0;
        regionId[id] = 0;
        distance[id] = 0;
        denoisedistance[id]=0;
        const base = id << 2;
        neighbor[base] = 0;
        neighbor[base + 1] = 0;
        neighbor[base + 2] = 0;
        neighbor[base + 3] = 0;
        use[id >> 3] |= (1 << (id & 7));  // 设置 use 位
    }

    /**
     * 获取 floor 值
     * @param {number} id 
     * @returns {number}
     */
    static getFloor(id) {
        return floor[id];
    }

    /**
     * 设置 floor 值
     * @param {number} id 
     * @param {number} value 
     */
    static setFloor(id, value) {
        floor[id] = value;
    }

    /**
     * 获取 ceiling 值
     * @param {number} id 
     * @returns {number}
     */
    static getCeiling(id) {
        return ceiling[id];
    }

    /**
     * 设置 ceiling 值
     * @param {number} id 
     * @param {number} value 
     */
    static setCeiling(id, value) {
        ceiling[id] = value;
    }

    /**
     * 获取下一个 span 的 ID
     * @param {number} id 
     * @returns {number} 0 表示没有下一个
     */
    static getNext(id) {
        return next[id];
    }

    /**
     * 设置下一个 span 的 ID
     * @param {number} id 
     * @param {number} nextId 
     */
    static setNext(id, nextId) {
        next[id] = nextId;
    }

    /**
     * 获取 use 状态
     * @param {number} id 
     * @returns {boolean}
     */
    static getUse(id) {
        return (use[id >> 3] & (1 << (id & 7))) !== 0;
    }

    /**
     * 设置 use 状态
     * @param {number} id 
     * @param {boolean} flag 
     */
    static setUse(id, flag) {
        if (flag) {
            use[id >> 3] |= (1 << (id & 7));
        } else {
            use[id >> 3] &= ~(1 << (id & 7));
        }
    }

    /**
     * 获取 region ID
     * @param {number} id 
     * @returns {number}
     */
    static getRegionId(id) {
        return regionId[id];
    }

    /**
     * 设置 region ID
     * @param {number} id 
     * @param {number} rid 
     */
    static setRegionId(id, rid) {
        regionId[id] = rid;
    }

    /**
     * 获取距离值
     * @param {number} id 
     * @returns {number}
     */
    static getDistance(id) {
        const d = distance[id];
        return d === DISTANCE_INF ? Infinity : d;
    }

    /**
     * 设置距离值
     * @param {number} id 
     * @param {number} dist 
     */
    static setDistance(id, dist) {
        if (!Number.isFinite(dist)) {
            distance[id] = DISTANCE_INF;
            return;
        }

        if (dist <= 0) {
            distance[id] = 0;
            return;
        }

        const clamped = Math.min(DISTANCE_INF - 1, Math.floor(dist));
        distance[id] = clamped;
    }

    /**
     * 获取距离值
     * @param {number} id 
     * @returns {number}
     */
    static getDenoiseDistance(id) {
        const d = denoisedistance[id];
        return d === DISTANCE_INF ? Infinity : d;
    }

    /**
     * 设置距离值
     * @param {number} id 
     * @param {number} dist 
     */
    static setDenoiseDistance(id, dist) {
        if (!Number.isFinite(dist)) {
            denoisedistance[id] = DISTANCE_INF;
            return;
        }

        if (dist <= 0) {
            denoisedistance[id] = 0;
            return;
        }

        const clamped = Math.min(DISTANCE_INF - 1, Math.floor(dist));
        denoisedistance[id] = clamped;
    }
    /**
     * 获取指定方向邻居 spanId
     * @param {number} id
     * @param {number} dir 0:W, 1:N, 2:E, 3:S
     * @returns {number}
     */
    static getNeighbor(id, dir) {
        return neighbor[(id << 2) + dir];
    }

    /**
     * 设置指定方向邻居 spanId
     * @param {number} id
     * @param {number} dir 0:W, 1:N, 2:E, 3:S
     * @param {number} neighborId
     */
    static setNeighbor(id, dir, neighborId) {
        neighbor[(id << 2) + dir] = neighborId;
    }

    /**
     * 清空 [startId, endId] 范围内的 span 数据
     * @param {number} startId
     * @param {number} endId
     */
    static clearRange(startId, endId) {
        const s = Math.max(1, startId | 0);
        const e = Math.max(s, endId | 0);
        for (let id = s; id <= e; id++) {
            floor[id] = 0;
            ceiling[id] = 0;
            next[id] = 0;
            regionId[id] = 0;
            distance[id] = 0;
            denoisedistance[id]=0;
            const base = id << 2;
            neighbor[base] = 0;
            neighbor[base + 1] = 0;
            neighbor[base + 2] = 0;
            neighbor[base + 3] = 0;
            use[id >> 3] &= ~(1 << (id & 7));
        }
    }

    /**
     * 双向通行检查（id1 和 id2 之间能否通行）
     * @param {number} id1 
     * @param {number} id2 
     * @param {number} maxStep 
     * @param {number} agentHeight 
     * @returns {boolean}
     */
    static canTraverseTo(id1, id2, maxStep = MAX_WALK_HEIGHT, agentHeight = AGENT_HEIGHT) {
        // 检查 id2 是否在使用
        if (!this.getUse(id2)) return false;
        
        // 高度差检查
        if (Math.abs(floor[id2] - floor[id1]) > maxStep) {
            return false;
        }

        // 检查两个 span 之间能否通行
        const floorLevel = Math.max(floor[id1], floor[id2]);
        const ceilLevel = Math.min(ceiling[id1], ceiling[id2]);

        if (ceilLevel - floorLevel < agentHeight) {
            return false;
        }

        return true;
    }

    /**
     * 单向通行检查（从 id1 只能往上到 id2）
     * @param {number} id1 
     * @param {number} id2 
     * @param {number} maxStep 
     * @param {number} agentHeight 
     * @returns {boolean}
     */
    static canTo(id1, id2, maxStep = MAX_WALK_HEIGHT, agentHeight = AGENT_HEIGHT) {
        // 检查 id2 是否在使用
        //if (!this.getUse(id2)) return false;
        
        // 只允许上升 maxStep 高度
        if (floor[id2] - floor[id1] > maxStep) {
            return false;
        }

        // 检查高度空间
        const floorLevel = floor[id1];
        const ceilLevel = ceiling[id2];

        if (ceilLevel - floorLevel < agentHeight) {
            return false;
        }

        return true;
    }
}