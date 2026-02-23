import { Instance } from "cs_script/point_script";
import { MESH_CELL_SIZE_XY, MESH_WORLD_SIZE_XY, PathState, TILE_SIZE,origin} from "./path_const";

/**
 * NavMesh 调试工具集合。
 *
 * 分层说明：
 * - MESH：体素/高度场层
 * - REGION：区域分割层
 * - CONTOUR：轮廓层
 * - POLY：主多边形层
 * - DETAIL：细节三角网层
 * - LINK：跳点连接层
 * - PATH：寻路结果层（poly path / funnel path / final path）
 */
export class NavMeshDebugTools {
    /**
     * @param {import("./path_manager").NavMesh} nav
     */
    constructor(nav) {
        /** @type {import("./path_manager").NavMesh} */
        this.nav = nav;
        /** @type {number[]} */
        this._polyAreas = [];
        /** @type {number[]} */
        this._polyPrefix = [];
        /** @type {number} */
        this._totalPolyArea = 0;
    }
    /**
        * 绘制最终细节网格（三角形）。
        * 对应调试部分：DETAIL。
        * 优先使用最终 meshdetail（支持 tile merge 后），无数据时回退到 polidetail 临时数据。
        *
     * 基于最终 this.nav.meshdetail 画细节三角形（兼容 tile merge 后结果）
     * @param {number} [duration]
     */
    debugDrawMeshDetail(duration = 10) {
        const detail = this.nav.meshdetail;
        if (detail?.verts && detail?.tris && detail.tris.length > 0) {
            for (let i = 0; i < detail.tris.length; i++) {
                const tri = detail.tris[i];
                if (!tri || tri.length < 3) continue;
                const a = detail.verts[tri[0]];
                const b = detail.verts[tri[1]];
                const c = detail.verts[tri[2]];
                if (!a || !b || !c) continue;
                const color = { r: 0, g: 180, b: 255 };
                Instance.DebugLine({ start: a, end: b, color, duration });
                Instance.DebugLine({ start: b, end: c, color, duration });
                Instance.DebugLine({ start: c, end: a, color, duration });
            }
            return;
        }
    }
    debugLinks(duration = 30) {
        for (const link of this.nav.links) {
            const isJump = link.type === PathState.JUMP;
            const isLadder = link.type === PathState.LADDER;
            const lineColor = isLadder
                ? { r: 255, g: 165, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 0, b: 255 });
            const startColor = isLadder
                ? { r: 255, g: 215, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 255, b: 0 });

            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: lineColor,
                duration
            });
            Instance.DebugSphere({ center: link.PosA, radius: 4, color: startColor, duration });
            const poly = this.nav.mesh.polys[link.PolyB];
            for (let i = 0; i < poly.length; i++) {
                const start = this.nav.mesh.verts[poly[i]];
                const end = this.nav.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({ start, end, color: isLadder ? { r: 255, g: 140, b: 0 } : { r: 255, g: 0, b: 255 }, duration });
            }
        }
    }
    /**
     * 绘制最终主多边形网格边线（不含 links）。
     * @param {number} duration
     */
    debugDrawMeshPolys(duration = 10) {
        if (!this.nav.mesh) return;
        for (let pi = 0; pi < this.nav.mesh.polys.length; pi++) {
            const poly = this.nav.mesh.polys[pi];
            const color = { r: 255, g: 0, b: 0 };
            for (let i = 0; i < poly.length; i++) {
                const start = this.nav.mesh.verts[poly[i]];
                const end = this.nav.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({ start, end, color, duration });
            }
        }
    }

    /**
        * 绘制最终多边形邻接关系（中心点连线）。
        * 对应调试部分：POLY 邻接/跨 tile 连接检查。
        *
     * 直接基于最终 this.nav.mesh.neighbors 画多边形连接关系
     * @param {number} [duration]
     */
    debugDrawMeshConnectivity(duration = 15) {
        if (!this.nav.mesh) return;
        const mesh = this.nav.mesh;
        const drawn = new Set();
        for (let i = 0; i < mesh.polys.length; i++) {
            const start = this._meshPolyCenter(i);
            const neighbors = mesh.neighbors?.[i] || [];
            for (let e = 0; e < neighbors.length; e++) {
                const edgeNei = neighbors[e];
                const neiList = Array.isArray(edgeNei)
                    ? edgeNei
                    : (typeof edgeNei === "number" && edgeNei >= 0 ? [edgeNei] : []);

                for (const ni of neiList) {
                    if (ni < 0) continue;
                    const a = Math.min(i, ni);
                    const b = Math.max(i, ni);
                    const k = `${a}|${b}`;
                    if (drawn.has(k)) continue;
                    drawn.add(k);

                    const end = this._meshPolyCenter(ni);
                    Instance.DebugLine({
                        start,
                        end,
                        color: { r: 255, g: 0, b: 255 },
                        duration
                    });
                }
            }
        }
    }

    /**
        * 计算指定多边形中心点。
        * 对应调试部分：内部工具（给邻接线、路径可视化复用）。
        *
     * @param {number} polyIndex
     */
    _meshPolyCenter(polyIndex) {
        const poly = this.nav.mesh.polys[polyIndex];
        let x = 0;
        let y = 0;
        let z = 0;
        for (const vi of poly) {
            const v = this.nav.mesh.verts[vi];
            x += v.x;
            y += v.y;
            z += v.z;
        }
        const n = poly.length || 1;
        return { x: x / n, y: y / n, z: z / n };
    }

    /**
        * 绘制 Funnel 拉直后的路径点序列。
        * 对应调试部分：PATH（funnel 阶段）。
        *
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     * @param {number} [duration]
     */
    debugDrawfunnelPath(path, duration = 10) {
        if (!path || path.length < 2) {
            Instance.Msg("No path to draw");
            return;
        }
        const color = { r: 0, g: 255, b: 0 };
        const colorJ = { r: 0, g: 255, b: 255 };

        const last = path[0].pos;
        Instance.DebugSphere({ center: { x: last.x, y: last.y, z: last.z }, radius: 3, color: { r: 255, g: 0, b: 0 }, duration });
        for (let i = 1; i < path.length; i++) {
            const a = path[i - 1].pos;
            const b = path[i].pos;
            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color: path[i].mode == PathState.WALK ? color:colorJ,
                duration
            });
            Instance.DebugSphere({ center: { x: b.x, y: b.y, z: b.z }, radius: 3, color: path[i].mode == 2 ? colorJ : color, duration });
        }
    }

    /**
        * 绘制最终输出路径（含跳跃段颜色区分）。
        * 对应调试部分：PATH（最终路径）。
        *
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     * @param {number} [duration]
     */
    debugDrawPath(path, duration = 10) {
        const color = { r: 0, g: 0, b: 255 };
        const colorJ = { r: 255, g: 255, b: 0 };
        if (!path || path.length == 2) {
            if (path && path.length == 2) {
                Instance.DebugSphere({ center: { x: path[0].pos.x, y: path[0].pos.y, z: path[0].pos.z }, radius: 3, color: { r: 0, g: 0, b: 255 }, duration });
                Instance.DebugLine({
                    start: { x: path[0].pos.x, y: path[0].pos.y, z: path[0].pos.z },
                    end: { x: path[1].pos.x, y: path[1].pos.y, z: path[1].pos.z },
                    color: path[1].mode == PathState.WALK ? color:colorJ,
                    duration
                });
                Instance.DebugSphere({ center: { x: path[1].pos.x, y: path[1].pos.y, z: path[1].pos.z }, radius: 3, color: path[1].mode == PathState.WALK ? color:colorJ, duration });
            } else Instance.Msg("No path to draw");
            return;
        }

        const last = path[0].pos;
        Instance.DebugSphere({ center: { x: last.x, y: last.y, z: last.z }, radius: 3, color: { r: 0, g: 0, b: 255 }, duration });
        for (let i = 1; i < path.length; i++) {
            const a = path[i - 1].pos;
            const b = path[i].pos;
            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color: path[i].mode == PathState.WALK ? color:colorJ,
                duration
            });
            Instance.DebugSphere({ center: { x: b.x, y: b.y, z: b.z }, radius: 3, color: path[i].mode == PathState.WALK ? color:colorJ, duration });
        }
    }

    /**
        * 绘制 A* 返回的多边形路径（按 poly 中心点串线）。
        * 对应调试部分：PATH（poly path 阶段）。
        *
     * @param {{id:number,mode:number}[]} polyPath
     * @param {number} [duration]
     */
    debugDrawPolyPath(polyPath, duration = 10) {
        if (!polyPath || polyPath.length === 0 || !this.nav.mesh) return;

        let prev = null;
        const color = {
            r: Math.floor(100 + Math.random() * 155),
            g: Math.floor(100 + Math.random() * 155),
            b: Math.floor(100 + Math.random() * 155),
        };
        const colorJ = {
            r: Math.floor(100 + Math.random() * 155),
            g: Math.floor(100 + Math.random() * 155),
            b: Math.floor(100 + Math.random() * 155),
        };
        for (const pi of polyPath) {
            Instance.Msg(pi.id);
            const poly = this.nav.mesh.polys[pi.id];
            let cx = 0, cy = 0, cz = 0;
            for (const vi of poly) {
                const v = this.nav.mesh.verts[vi];
                cx += v.x; cy += v.y; cz += v.z;
            }
            cx /= poly.length;
            cy /= poly.length;
            cz /= poly.length;

            const center = { x: cx, y: cy, z: cz };
            if (pi.mode == 2) {
                Instance.DebugSphere({ center, radius: 10, color: colorJ, duration });
                if (prev) Instance.DebugLine({ start: prev, end: center, color: colorJ, duration });
            } else {
                Instance.DebugSphere({ center, radius: 10, color, duration });
                if (prev) Instance.DebugLine({ start: prev, end: center, color, duration });
            }
            prev = center;
        }
    }
    /**
     * @param {number} duration
     */
    debugDrawALLTiles(duration = 120) {
        const color = { r: 255, g: 255, b: 255 };
        const fullGrid=Math.floor(MESH_WORLD_SIZE_XY / MESH_CELL_SIZE_XY) + 1;
        const tiles=Math.ceil(fullGrid / TILE_SIZE);
        for (let ty = 0; ty < tiles; ty++) {
            for (let tx = 0; tx < tiles; tx++) {
                const coreMinX = tx * TILE_SIZE;
                const coreMinY = ty * TILE_SIZE;
                const coreMaxX = Math.min(fullGrid - 1, coreMinX + TILE_SIZE - 1);
                const coreMaxY = Math.min(fullGrid - 1, coreMinY + TILE_SIZE - 1);

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
}
