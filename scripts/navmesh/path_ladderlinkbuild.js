import { Instance } from "cs_script/point_script";
import { ADJUST_HEIGHT_DISTANCE, PathState } from "./path_const";
import { Tool } from "./util/tool";
import { FunnelHeightFixer } from "./path_funnelheightfixer";
import { vec } from "./util/vector";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
//不多，可以每次都重新构建
export class LadderLinkBuilder {
    /**
     * @param {NavMeshMesh} polyMesh
     * @param {NavMeshDetail} detailMesh
     */
    constructor(polyMesh,detailMesh) {
        this.mesh = polyMesh;
        /** @type {boolean} */
        this.error = false;
        /** @type {NavMeshLink[]} */
        this.links = [];
        this.heightfixer=new FunnelHeightFixer(this.mesh,detailMesh,ADJUST_HEIGHT_DISTANCE);
    }

    init() {
        this.error = false;
        this.links = [];
        Tool.buildSpatialIndex(this.mesh);
        if (!this.mesh || !this.mesh.polys || this.mesh.polys.length === 0) return this.links;

        /** @type {Map<string, Vector[]>} */
        const groups = new Map();
        const ents = Instance.FindEntitiesByClass("info_target");

        for (const ent of ents) {
            const name = ent.GetEntityName();
            if (!name.startsWith("LADDER_")) continue;

            const tag = name.slice("LADDER_".length);
            if (!tag) continue;

            const p = ent.GetAbsOrigin();
            if (!p) continue;

            if (!groups.has(tag)) groups.set(tag, []);
            groups.get(tag)?.push({ x: p.x, y: p.y, z: p.z });
        }

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

            points.sort((a, b) => a.z - b.z);
            const aPos = points[0];
            const bPos = points[points.length - 1];
            rawPairs++;

            const aNearest = Tool.findNearestPoly(aPos, this.mesh,this.heightfixer);
            const bNearest = Tool.findNearestPoly(bPos, this.mesh,this.heightfixer);
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
            const cost = Math.max(1, vec.length(aNearest.pos, bNearest.pos));
            this.links.push({
                PolyA: aPoly,
                PolyB: bPoly,
                PosA: aPos,
                PosB: bPos,
                cost,
                type: PathState.LADDER
            });
            validPairs++;
        }
        Instance.Msg(`LadderLink统计: group=${groups.size} pair=${rawPairs} link=${this.links.length} valid=${validPairs}`);
        return this.links;
    }

    /**
     * @param {number} duration
     */
    debugDraw(duration = 30) {
        for (const link of this.links) {
            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: { r: 255, g: 165, b: 0 },
                duration
            });
            Instance.DebugSphere({ center: link.PosA, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
            Instance.DebugSphere({ center: link.PosB, radius: 4, color: { r: 255, g: 215, b: 0 }, duration });
        }
    }
}

