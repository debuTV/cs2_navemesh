import { Instance } from "cs_script/point_script";
import { MAX_LINKS, PathState } from "./path_const";
import { Tool } from "./util/tool";
import { vec } from "./util/vector";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
//不多，可以每次都重新构建
export class LadderLinkBuilder {
    /**
     * @param {NavMeshMesh} polyMesh
     */
    constructor(polyMesh) {
        this.mesh = polyMesh;
        /** @type {boolean} */
        this.error = false;
        /** @type {Uint16Array} */
        this.poly = new Uint16Array(MAX_LINKS * 2);
        /** @type {Float32Array} */
        this.cost = new Float32Array(MAX_LINKS);
        /** @type {Uint8Array} */
        this.type = new Uint8Array(MAX_LINKS);
        /** @type {Float32Array} */
        this.pos = new Float32Array(MAX_LINKS * 6);
        /** @type {number} */
        this.length = 0;
    }

    /**
     * @returns {NavMeshLink}
     */
    return() {
        return {
            poly: this.poly,
            cost: this.cost,
            type: this.type,
            pos: this.pos,
            length: this.length
        };
    }

    /**
     * @param {number} polyA
     * @param {number} polyB
     * @param {Vector} posA
     * @param {Vector} posB
     * @param {number} cost
     */
    pushLink(polyA, polyB, posA, posB, cost) {
        const i = this.length;
        const pi = i << 1;
        const vi = i * 6;
        this.poly[pi] = polyA;
        this.poly[pi + 1] = polyB;
        this.cost[i] = cost;
        this.type[i] = PathState.LADDER;
        this.pos[vi] = posA.x;
        this.pos[vi + 1] = posA.y;
        this.pos[vi + 2] = posA.z;
        this.pos[vi + 3] = posB.x;
        this.pos[vi + 4] = posB.y;
        this.pos[vi + 5] = posB.z;
        this.length++;
    }

    init() {
        this.error = false;
        this.length = 0;
        if (!this.mesh || !this.mesh.polys || this.mesh.polyslength === 0) return this.return();

        /** @type {Map<string, Vector[]>} */
        const groups = new Map();
        const ents = Instance.FindEntitiesByClass("info_target");

        for (const ent of ents) {
            const name = ent.GetEntityName();
            if (!name.startsWith("navmesh_LADDER_")) continue;

            const tag = name.slice("navmesh_LADDER_".length);
            if (!tag) continue;

            const p = ent.GetAbsOrigin();
            if (!p) continue;

            if (!groups.has(tag)) groups.set(tag, []);
            groups.get(tag)?.push({ x: p.x, y: p.y, z: p.z });
        }
        //let start=new Date();
        let rawPairs = 0;
        let validPairs = 0;

        for (const [tag, points] of groups) {
            if (points.length < 2) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 点位不足(=${points.length})，已跳过`);
                continue;
            }
            if (points.length !== 2) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 点位数量过多(${points.length})，已跳过`);
                continue;
            }
            const p0 = points[0], p1 = points[1];
            const aPos = p0.z <= p1.z ? p0 : p1;
            const bPos = p0.z <= p1.z ? p1 : p0;
            //points.sort((a, b) => a.z - b.z);
            //const aPos = points[0];
            //const bPos = points[points.length - 1];
            rawPairs++;
            const aNearest = Tool.findNearestPoly(aPos, this.mesh);//,this.heightfixer);
            const bNearest = Tool.findNearestPoly(bPos, this.mesh);//,this.heightfixer);
            const aPoly = aNearest.poly;
            const bPoly = bNearest.poly;
            if (aPoly < 0 || bPoly < 0) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 找不到最近多边形，已跳过`);
                continue;
            }
            if (aPoly === bPoly) {
                this.error = true;
                Instance.Msg(`LadderLink: ${tag} 两端落在同一 poly(${aPoly})，已跳过`);
                continue;
            }
            const cost = Math.max(1, vec.lengthsq(aNearest.pos, bNearest.pos));
            this.pushLink(aPoly, bPoly, aPos, bPos, cost);
            validPairs++;
        }
        Instance.Msg(`LadderLink统计: group=${groups.size} pair=${rawPairs} link=${this.length} valid=${validPairs}`);
        return this.return();
    }

    /**
     * @param {number} duration
     */
    debugDraw(duration = 30) {
        for (let i = 0; i < this.length; i++) {
            const vi = i * 6;
            const start = {
                x: this.pos[vi],
                y: this.pos[vi + 1],
                z: this.pos[vi + 2]
            };
            const end = {
                x: this.pos[vi + 3],
                y: this.pos[vi + 4],
                z: this.pos[vi + 5]
            };
            Instance.DebugLine({
                start,
                end,
                color: { r: 255, g: 165, b: 0 },
                duration
            });
            Instance.DebugSphere({ center: start, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
            Instance.DebugSphere({ center: end, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
        }
    }
}

