import { Instance } from "cs_script/point_script";
import { MESH_CELL_SIZE_XY, MESH_WORLD_SIZE_XY, PathState, TILE_SIZE,origin} from "./path_const";

/**
 * NavMesh 调试工具集
 *
 * 说明：
 * - MESH：体素化 / 网格化 步骤
 * - REGION：区域分割步骤
 * - CONTOUR：轮廓构建步骤
 * - POLY：多边形生成步骤
 * - DETAIL：细节层（detail）三角网
 * - LINK：连接（links）构建
 * - PATH：路径生成与输出（poly path / funnel path / final path）
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
     * 绘制 detail 层三角形（用于调试 detail 网格）。
     * 期望 detail 使用 TypedArray 布局：`verts` 为 Float32Array，`tris` 为 Uint16Array，
     * 并存在 `trislength` / `vertslength` 等计数字段。
     * @param {number} [duration]
     */
    debugDrawMeshDetail(duration = 10) {
        const detail = this.nav.meshdetail;
        if (!detail) return;
        // TypedArray 结构：detail.verts 为 Float32Array，detail.tris 为 Uint16Array，并存在 trislength/vertslength
        for (let i = 0; i < detail.trislength; i++) {
            const ia = detail.tris[i * 3];
            const ib = detail.tris[i * 3 + 1];
            const ic = detail.tris[i * 3 + 2];
            const va = {
                x: detail.verts[ia * 3],
                y: detail.verts[ia * 3 + 1],
                z: detail.verts[ia * 3 + 2]
            };
            const vb = {
                x: detail.verts[ib * 3],
                y: detail.verts[ib * 3 + 1],
                z: detail.verts[ib * 3 + 2]
            };
            const vc = {
                x: detail.verts[ic * 3],
                y: detail.verts[ic * 3 + 1],
                z: detail.verts[ic * 3 + 2]
            };
            const color = { r: 0, g: 180, b: 255 };
            Instance.DebugLine({ start: va, end: vb, color, duration });
            Instance.DebugLine({ start: vb, end: vc, color, duration });
            Instance.DebugLine({ start: vc, end: va, color, duration });
        }
        return;
    }
    debugLinks(duration = 30) {
        const links = this.nav.links;
        const mesh = this.nav.mesh;
        if (!links || !mesh || !mesh.polys || !mesh.verts) return;

        for (let li = 0; li < links.length; li++) {
            const type = links.type[li];
            const isJump = type === PathState.JUMP;
            const isLadder = type === PathState.LADDER;
            const lineColor = isLadder
                ? { r: 255, g: 165, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 0, b: 255 });
            const startColor = isLadder
                ? { r: 255, g: 215, b: 0 }
                : (isJump ? { r: 0, g: 255, b: 255 } : { r: 0, g: 255, b: 0 });

            const posBase = li * 6;
            const start = {
                x: links.pos[posBase],
                y: links.pos[posBase + 1],
                z: links.pos[posBase + 2]
            };
            const end = {
                x: links.pos[posBase + 3],
                y: links.pos[posBase + 4],
                z: links.pos[posBase + 5]
            };

            Instance.DebugLine({ start, end, color: lineColor, duration });
            Instance.DebugSphere({ center: start, radius: 4, color: startColor, duration });

            const pi = links.poly[(li << 1) + 1];
            if (pi < 0 || pi >= mesh.polyslength) continue;

            const startVert = mesh.polys[pi * 2];
            const endVert = mesh.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            for (let i = 0; i < vertCount; i++) {
                const vi0 = startVert + i;
                const vi1 = startVert + ((i + 1) % vertCount);
                const v0 = { x: mesh.verts[vi0 * 3], y: mesh.verts[vi0 * 3 + 1], z: mesh.verts[vi0 * 3 + 2] };
                const v1 = { x: mesh.verts[vi1 * 3], y: mesh.verts[vi1 * 3 + 1], z: mesh.verts[vi1 * 3 + 2] };
                Instance.DebugLine({ start: v0, end: v1, color: isLadder ? { r: 255, g: 140, b: 0 } : { r: 255, g: 0, b: 255 }, duration });
            }
        }
    }
    /**
     * 绘制所有多边形（不展示 links），用于检查多边形边界。
     * @param {number} duration
     */
    debugDrawMeshPolys(duration = 10) {
        if (!this.nav.mesh) return;
        const mesh = this.nav.mesh;
        for (let pi = 0; pi < mesh.polyslength; pi++) {
            const startVert = mesh.polys[pi * 2];
            const endVert = mesh.polys[pi * 2 + 1];
            const vertCount = endVert - startVert + 1;
            if (vertCount < 3) continue;
            const color = { r: 255, g: 0, b: 0 };
            for (let i = 0; i < vertCount; i++) {
                const vi0 = startVert + i;
                const vi1 = startVert + ((i + 1) % vertCount);
                const v0 = { x: mesh.verts[vi0 * 3], y: mesh.verts[vi0 * 3 + 1], z: mesh.verts[vi0 * 3 + 2] };
                const v1 = { x: mesh.verts[vi1 * 3], y: mesh.verts[vi1 * 3 + 1], z: mesh.verts[vi1 * 3 + 2] };
                Instance.DebugLine({ start: v0, end: v1, color, duration });
            }
        }
    }

    /**
     * 绘制网格连通关系（多边形邻接），用于调试跨 tile 的边界匹配。
     * 直接读取 `this.nav.mesh.neighbors` 结构并绘制连接线。
     * @param {number} [duration]
     */
    debugDrawMeshConnectivity(duration = 15) {
        if (!this.nav.mesh) return;
        const mesh = this.nav.mesh;
        const drawn = new Set();
        for (let i = 0; i < mesh.polyslength; i++) {
            const start = this._meshPolyCenter(i);
            const pstart=this.nav.mesh.polys[i*2];
            const pend=this.nav.mesh.polys[i*2+1];
            const ecount=pend-pstart+1;
            for (let e = 0; e < ecount; e++) {
                const edgeNei = mesh.neighbors[i][e][0];
                if(edgeNei==0)continue;
                for(let j=1;j<=edgeNei;j++)
                {
                    const ni=mesh.neighbors[i][e][j];
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
     * 计算指定多边形的几何中心（用于调试绘制）。
     * 适配 TypedArray 布局，返回 {x,y,z}。
     * @param {number} polyIndex
     */
    _meshPolyCenter(polyIndex) {
        const mesh = this.nav.mesh;
        const startVert = mesh.polys[polyIndex * 2];
        const endVert = mesh.polys[polyIndex * 2 + 1];
        const vertCount = endVert - startVert + 1;
        if (vertCount <= 0) return { x: 0, y: 0, z: 0 };
        let x = 0, y = 0, z = 0;
        for (let vi = startVert; vi <= endVert; vi++) {
            x += mesh.verts[vi * 3];
            y += mesh.verts[vi * 3 + 1];
            z += mesh.verts[vi * 3 + 2];
        }
        return { x: x / vertCount, y: y / vertCount, z: z / vertCount };
    }

    /**
     * 绘制 Funnel 生成的路径（用于调试 funnel 算法）。
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
     * 绘制路径（包含不同模式的颜色区分，例如行走/跳跃/梯子）。
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
     * @param {{id:number,mode:number}[]} polyPath
     * @param {number} [duration]
     */
    debugDrawPolyPath(polyPath, duration = 10) {
        if (!polyPath || polyPath.length === 0 || !this.nav.mesh) return;
        const mesh = this.nav.mesh;
        let prev = null;
        // 避免重复绘制相同路径段或中心点
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
            // 适配 TypedArray 布局：mesh.polys 存为 start/end 对，mesh.verts 为扁平 Float32Array
            const polyIndex = pi.id;
            const startVert = mesh.polys[polyIndex * 2];
            const endVert = mesh.polys[polyIndex * 2 + 1];
            const vertCount = endVert - startVert + 1;
            if (vertCount < 3) continue;
            let cx = 0, cy = 0, cz = 0;
            for (let vi = startVert; vi <= endVert; vi++) {
                cx += mesh.verts[vi * 3];
                cy += mesh.verts[vi * 3 + 1];
                cz += mesh.verts[vi * 3 + 2];
            }
            cx /= vertCount;
            cy /= vertCount;
            cz /= vertCount;
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
