import { Instance } from "cs_script/point_script";
/** @typedef {import("cs_script/point_script").Vector} Vector */

/**
 * vec: 轻量向量工具类（无状态静态方法）。
 *
 * 约定：
 * - 不修改传入参数，所有方法返回新对象或标量
 * - `2D` 后缀表示仅计算 XY 分量（通常保留原 z 或返回 z=0）
 */
export class vec {
    /**
     * 三维向量加法。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static add(a, b) {
        return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
    }

    /**
     * 二维向量加法（仅累加 XY，z 保留 a.z）。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static add2D(a, b) {
        return { x: a.x + b.x, y: a.y + b.y, z: a.z };
    }

    /**
     * 三维向量减法。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static sub(a, b) {
        return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
    }

    /**
     * 三维向量按标量缩放。
     *
     * @param {Vector} a
     * @param {number} s
     * @returns {Vector}
     */
    static scale(a, s) {
        return { x: a.x * s, y: a.y * s, z: a.z * s };
    }

    /**
     * 二维向量按标量缩放（仅缩放 XY，z 保留 a.z）。
     *
     * @param {Vector} a
     * @param {number} s
     * @returns {Vector}
     */
    static scale2D(a, s) {
        return {
            x: a.x * s,
            y: a.y * s,
            z: a.z
        };
    }

    /**
     * 构造一个向量对象。
     *
     * @param {number} [x]
     * @param {number} [y]
     * @param {number} [z]
     * @returns {Vector}
     */
    static get(x = 0, y = 0, z = 0) {
        return { x, y, z };
    }

    /**
     * 克隆向量。
     *
     * @param {Vector} a
     * @returns {Vector}
     */
    static clone(a) {
        return { x: a.x, y: a.y, z: a.z };
    }

    /**
     * 计算三维欧氏距离。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static length(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dz = a.z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    /**
     * 计算三维欧氏距离平方。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static lengthsq(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        const dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }
    /**
     * 计算二维欧氏距离（仅 XY）。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static length2D(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    /**
     * 计算二维欧氏距离平方（仅 XY）。
     * b 缺省时按原点处理。
     *
     * @param {Vector} a
     * @param {Vector} [b]
     * @returns {number}
     */
    static length2Dsq(a, b = { x: 0, y: 0, z: 0 }) {
        const dx = a.x - b.x;
        const dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
    /**
     * 返回点在 Z 轴上偏移后的新坐标。
     *
     * @param {Vector} pos
     * @param {number} height
     * @returns {Vector}
     */
    static Zfly(pos, height) {
        return { x: pos.x, y: pos.y, z: pos.z + height };
    }

    /**
     * 输出向量坐标到游戏消息。
     *
     * @param {Vector} pos
     */
    static msg(pos) {
        Instance.Msg(`{${pos.x} ${pos.y} ${pos.z}}`);
    }

    /**
     * 三维点积。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {number}
     */
    static dot(a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * 二维点积（仅 XY）。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {number}
     */
    static dot2D(a, b) {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * 三维叉积。
     *
     * @param {Vector} a
     * @param {Vector} b
     * @returns {Vector}
     */
    static cross(a, b) {
        return {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x
        };
    }

    /**
     * 三维单位化。
     * 当长度过小（<1e-6）时返回零向量，避免除零。
     *
     * @param {Vector} a
     * @returns {Vector}
     */
    static normalize(a) {
        const len = this.length(a);
        if (len < 1e-6) {
            return { x: 0, y: 0, z: 0 };
        }
        return this.scale(a, 1 / len);
    }

    /**
     * 二维单位化（仅 XY，返回 z=0）。
     * 当长度过小（<1e-6）时返回零向量。
     *
     * @param {Vector} a
     * @returns {Vector}
     */
    static normalize2D(a) {
        const len = this.length2D(a);
        if (len < 1e-6) {
            return { x: 0, y: 0, z: 0 };
        }
        return {
            x: a.x / len,
            y: a.y / len,
            z: 0
        };
    }

    /**
     * 判断是否为近似零向量。
     *
     * @param {Vector} a
     * @returns {boolean}
     */
    static isZero(a) {
        return (
            Math.abs(a.x) < 1e-6 &&
            Math.abs(a.y) < 1e-6 &&
            Math.abs(a.z) < 1e-6
        );
    }
}
