import { Instance } from "cs_script/point_script";
import { MESH_CELL_SIZE_XY, MESH_WORLD_SIZE_XY, origin, MESH_DEBUG, REGION_DEBUG, CONTOUR_DEBUG, TILE_PADDING, TILE_SIZE, LADDER, POLY_DEBUG, POLY_DETAIL_DEBUG, LINK_DEBUG } from "./path_const";
import { OpenHeightfield } from "./path_openheightfield";
import { OpenSpan } from "./path_openspan";
import { RegionGenerator } from "./path_regiongenerator";
import { ContourBuilder } from "./path_contourbuilder";
import { PolyMeshBuilder } from "./path_polymeshbuilder";
import { PolyMeshDetailBuilder } from "./path_polydetail";
import { JumpLinkBuilder } from "./path_jumplinkbuild";

/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */

/**
 * 单 tile 构建器：
 * - 只负责当前 tile 内构建（体素化/区域/轮廓/多边形/细节/link）
 * - 返回 tile 数据，由 TileManager 负责 tile 间聚合与裁剪
 */
export class tile {
    constructor() {
        /** @type {OpenHeightfield | undefined} */
        this.hf = undefined;
        /** @type {RegionGenerator | undefined} */
        this.regionGen = undefined;
        /** @type {ContourBuilder | undefined} */
        this.contourBuilder = undefined;
        /** @type {PolyMeshBuilder | undefined} */
        this.polyMeshGenerator = undefined;
        /** @type {PolyMeshDetailBuilder | undefined} */
        this.polidetail = undefined;
        /** @type {JumpLinkBuilder | undefined} */
        this.jumplinkbuilder = undefined;
        //边界体素
        this.tilePadding = Math.max(0, TILE_PADDING | 0);
        //tile大小,不含padding
        this.tileSize = Math.max(1, TILE_SIZE | 0);
        //一边有几个体素
        this.fullGrid = Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        this.tilesX = Math.ceil(this.fullGrid / this.tileSize);
        this.tilesY = Math.ceil(this.fullGrid / this.tileSize);
    }
    /**
     * @param {{x:number,y:number,z:number}} pos
     */
    fromPosGetTile(pos) {
        const gx = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.x - origin.x) / MESH_CELL_SIZE_XY)));
        const gy = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.y - origin.y) / MESH_CELL_SIZE_XY)));
        const tx = Math.max(0, Math.min(this.tilesX - 1, Math.floor(gx / this.tileSize)));
        const ty = Math.max(0, Math.min(this.tilesY - 1, Math.floor(gy / this.tileSize)));
        return `${tx}_${ty}`;
    }
    /**
     * 仅构建给定世界坐标所在的 tile。
     * @param {{x:number,y:number,z:number}} pos
     */
    buildTileNavMeshAtPos(pos) {
        const gx = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.x - origin.x) / MESH_CELL_SIZE_XY)));
        const gy = Math.max(0, Math.min(this.fullGrid - 1, Math.floor((pos.y - origin.y) / MESH_CELL_SIZE_XY)));
        const tx = Math.max(0, Math.min(this.tilesX - 1, Math.floor(gx / this.tileSize)));
        const ty = Math.max(0, Math.min(this.tilesY - 1, Math.floor(gy / this.tileSize)));
        return this.buildTile(tx, ty);
    }

    /**
     * @param {number} tx
     * @param {number} ty
     */
    buildTile(tx, ty) {
        const tileDebugDuration=60;
        const nowMs = () => new Date().getTime();
        const timing = {hfInit: 0,region: 0,contour: 0,poly: 0,detail: 0,merge: 0,jumpLinks: 0,};

        let tileHasError = false;
        const tileStartMs = nowMs();
        Instance.Msg(`开始构建 Tile (${tx+1}/${this.tilesX},${ty+1}/${this.tilesY})`);
        let phaseStartMs = nowMs();

        this.hf = new OpenHeightfield(tx, ty, this.tileSize, this.fullGrid, this.tilePadding);
        this.hf.init();

        if (MESH_DEBUG) this.hf.debug(tileDebugDuration);
        timing.hfInit += nowMs() - phaseStartMs;
        phaseStartMs = nowMs();

        this.regionGen = new RegionGenerator(this.hf);
        this.regionGen.init();

        if (REGION_DEBUG) this.regionGen.debugDrawRegions(tileDebugDuration);
        timing.region += nowMs() - phaseStartMs;
        phaseStartMs = nowMs();

        this.contourBuilder = new ContourBuilder(this.hf);
        this.contourBuilder.init();

        if (this.contourBuilder.error) tileHasError = true;
        if (CONTOUR_DEBUG) this.contourBuilder.debugDrawContours(tileDebugDuration);
        timing.contour += nowMs() - phaseStartMs;
        phaseStartMs = nowMs();

        this.polyMeshGenerator = new PolyMeshBuilder(this.contourBuilder.contours);
        this.polyMeshGenerator.init();

        const tileMesh = this.polyMeshGenerator.return();
        if (this.polyMeshGenerator.error) tileHasError = true;
        timing.poly += nowMs() - phaseStartMs;
        //if (POLY_DEBUG) {
        //    this.polyMeshGenerator.debugDrawPolys(tileDebugDuration);
        //    this.polyMeshGenerator.debugDrawAdjacency(tileDebugDuration);
        //}

        phaseStartMs = nowMs();

        this.polidetail = new PolyMeshDetailBuilder(tileMesh, this.hf);
        /** @type {NavMeshDetail} */
        let tileDetail = this.polidetail.init();
        //if(POLY_DETAIL_DEBUG)
        //{
        //    this.polidetail.debugDrawPolys(tileDebugDuration);
        //}
        if (this.polidetail.error) tileHasError = true;
        timing.detail += nowMs() - phaseStartMs;

        phaseStartMs = nowMs();
        this.jumplinkbuilder = new JumpLinkBuilder(tileMesh);
        /**
         * @type {NavMeshLink}
         */
        let tileLinks = this.jumplinkbuilder.init();
        //if(LINK_DEBUG)
        //{
        //    this.jumplinkbuilder.debugDraw(tileDebugDuration);
        //}
        timing.jumpLinks += nowMs() - phaseStartMs;

        OpenSpan.clearRange(1, this.hf.SPAN_ID + 2);
        const tileCostMs = nowMs() - tileStartMs;
        Instance.Msg(`完成 Tile (${tx+1}/${this.tilesX},${ty+1}/${this.tilesY}),耗时${tileCostMs}ms`);

        return {tileId: `${tx}_${ty}`,tx,ty,mesh: tileMesh,detail: tileDetail,links: tileLinks,hasError: tileHasError,timing};
    }

    /**
     * @param {{tx:number,ty:number}[]} tiles
     * @param {number} duration
     */
    debugDrawErrorTiles(tiles, duration = 120) {
        if (!tiles || tiles.length === 0) return;
        const color = { r: 255, g: 255, b: 255 };

        for (const tile of tiles) {
            const coreMinX = tile.tx * this.tileSize;
            const coreMinY = tile.ty * this.tileSize;
            const coreMaxX = Math.min(this.fullGrid - 1, coreMinX + this.tileSize - 1);
            const coreMaxY = Math.min(this.fullGrid - 1, coreMinY + this.tileSize - 1);

            const minX = origin.x + coreMinX * MESH_CELL_SIZE_XY;
            const minY = origin.y + coreMinY * MESH_CELL_SIZE_XY;
            const maxX = origin.x + (coreMaxX + 1) * MESH_CELL_SIZE_XY;
            const maxY = origin.y + (coreMaxY + 1) * MESH_CELL_SIZE_XY;

            const z0 = origin.z + 8;
            const z1 = origin.z + 500;

            const a0 = { x: minX, y: minY, z: z0 };
            const b0 = { x: maxX, y: minY, z: z0 };
            const c0 = { x: maxX, y: maxY, z: z0 };
            const d0 = { x: minX, y: maxY, z: z0 };
            const a1 = { x: minX, y: minY, z: z1 };
            const b1 = { x: maxX, y: minY, z: z1 };
            const c1 = { x: maxX, y: maxY, z: z1 };
            const d1 = { x: minX, y: maxY, z: z1 };

            Instance.DebugLine({ start: a0, end: b0, color, duration });
            Instance.DebugLine({ start: b0, end: c0, color, duration });
            Instance.DebugLine({ start: c0, end: d0, color, duration });
            Instance.DebugLine({ start: d0, end: a0, color, duration });

            Instance.DebugLine({ start: a1, end: b1, color, duration });
            Instance.DebugLine({ start: b1, end: c1, color, duration });
            Instance.DebugLine({ start: c1, end: d1, color, duration });
            Instance.DebugLine({ start: d1, end: a1, color, duration });

            Instance.DebugLine({ start: a0, end: a1, color, duration });
            Instance.DebugLine({ start: b0, end: b1, color, duration });
            Instance.DebugLine({ start: c0, end: c1, color, duration });
            Instance.DebugLine({ start: d0, end: d1, color, duration });

            Instance.DebugLine({ start: a1, end: c1, color, duration });
            Instance.DebugLine({ start: b1, end: d1, color, duration });
        }
    }

}
