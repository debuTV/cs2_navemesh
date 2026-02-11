import { Instance } from "cs_script/point_script";
export class vec{
    /**
     * 返回向量vec1+vec2
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} b
     * @returns {import("cs_script/point_script").Vector}
     */
    static add(a, b) {
        return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
    }
    /**
     * 添加 2D 分量
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} b
     * @returns {import("cs_script/point_script").Vector}
     */
    static add2D(a, b) {
        return { x: a.x + b.x, y: a.y + b.y, z: a.z};
    }
    /**
     * 返回向量vec1-vec2
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} b
     * @returns {import("cs_script/point_script").Vector}
     */
    static sub(a, b) {
        return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
    }
    /**
     * 返回向量vec1*s
     * @param {import("cs_script/point_script").Vector} a
     * @param {number} s
     * @returns {import("cs_script/point_script").Vector}
     */
    static scale(a,s)
    {
        return {x:a.x*s,y:a.y*s,z:a.z*s}
    }
    /**
     * 返回向量vec1*s
     * @param {import("cs_script/point_script").Vector} a
     * @param {number} s
     * @returns {import("cs_script/point_script").Vector}
     */
    static scale2D(a,s) {
        return {
            x:a.x * s,
            y:a.y * s,
            z:a.z
        };
    }
    /**
     * 得到vector
     * @param {number} [x]
     * @param {number} [y]
     * @param {number} [z]
     * @returns {import("cs_script/point_script").Vector}
     */
    static get(x=0,y=0,z=0)
    {
        return {x,y,z};
    }
    /**
     * 深复制
     * @param {import("cs_script/point_script").Vector} a
     * @returns {import("cs_script/point_script").Vector}
     */
    static clone(a)
    {
        return {x:a.x,y:a.y,z:a.z};
    }
    /**
     * 计算空间两点之间的距离
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} [b]
     * @returns {number}
     */
    static length(a, b={x:0,y:0,z:0}) {
        const dx = a.x - b.x; const dy = a.y - b.y; const dz = a.z - b.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }
    /**
     * 计算xy平面两点之间的距离
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} [b]
     * @returns {number}
     */
    static length2D(a, b={x:0,y:0,z:0}) {
        const dx = a.x - b.x; const dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
    /**
     * 返回pos上方height高度的点
     * @param {import("cs_script/point_script").Vector} pos
     * @param {number} height
     * @returns {import("cs_script/point_script").Vector}
     */
    static Zfly(pos, height) {
        return { x: pos.x, y: pos.y, z: pos.z + height };
    }
    /**
     * 输出点pos的坐标
     * @param {import("cs_script/point_script").Vector} pos
     */
    static msg(pos) {
        Instance.Msg(`{${pos.x} ${pos.y} ${pos.z}}`);
    }
    /**
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} b
     */
    static dot(a,b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} b
     */
    static dot2D(a,b) {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * @param {import("cs_script/point_script").Vector} a
     * @param {import("cs_script/point_script").Vector} b
     */
    static cross(a,b) {
        return {
            x:a.y * b.z - a.z * b.y,
            y:a.z * b.x - a.x * b.z,
            z:a.x * b.y - a.y * b.x
        };
    }
    /**
     * @param {import("cs_script/point_script").Vector} a
     */
    static normalize(a) {
        const len = this.length(a);
        if (len < 1e-6) {
            return {x:0,y:0,z:0};
        }
        return this.scale(a,1 / len);
    }
    /**
     * @param {import("cs_script/point_script").Vector} a
     */
    static normalize2D(a) {
        const len = this.length2D(a);
        if (len < 1e-6) {
            return {x:0,y:0,z:0};
        }
        return {
            x:a.x / len,
            y:a.y / len,
            z:0
        };
    }
    /**
     * @param {import("cs_script/point_script").Vector} a
     */
    static isZero(a) {
        return (
            Math.abs(a.x) < 1e-6 &&
            Math.abs(a.y) < 1e-6 &&
            Math.abs(a.z) < 1e-6
        );
    }
}