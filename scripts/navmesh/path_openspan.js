import { AGENT_HEIGHT, MAX_WALK_HEIGHT } from "./path_const";

export class OpenSpan {
    /**
     * @param {number} floor
     * @param {number} ceiling
     * @param {number} id
     */
    constructor(floor, ceiling, id) {
        /**@type {number} */
        this.floor = floor;
        /**@type {number} */
        this.ceiling = ceiling;
        /**@type {OpenSpan|null} */
        this.next = null;
        /**@type {number} */
        this.id = id;

        /**@type {any[]} */
        this.neighbors = [null, null, null, null];
        /**@type {number} */
        this.distance = 0;
        /**@type {number} */
        this.regionId = 0;

        //区域距离场优化
        this.newDist = 0;
        //是否在使用
        this.use=true;
    }

    /**
     * @param {OpenSpan} other
     * @param {number} maxStep
     * @param {number} agentHeight
     * @returns {boolean}
     */
    canTraverseTo(other, maxStep = MAX_WALK_HEIGHT, agentHeight = AGENT_HEIGHT) {
        if(!other.use)return false;
        if (Math.abs(other.floor - this.floor) > maxStep) {
            return false;
        }

        const floor = Math.max(this.floor, other.floor);
        const ceil = Math.min(this.ceiling, other.ceiling);

        if (ceil - floor < agentHeight) {
            return false;
        }

        return true;
    }
}