/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("./path_tile").tile} tile */
import { Instance } from "cs_script/point_script";
import { LADDER, MAX_WALK_HEIGHT, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, PathState, TILE_OPTIMIZATION_1 } from "./path_const";
import { Tool, UnionFind } from "./util/tool";
import { LadderLinkBuilder } from "./path_ladderlinkbuild";
import { JumpLinkBuilder } from "./path_jumplinkbuild";
import { NavMesh } from "./path_manager";

/**
 * @typedef {{
 *  tileId:string,
 *  tx:number,
 *  ty:number,
 *  mesh:NavMeshMesh,
 *  detail:NavMeshDetail,
 *  links:NavMeshLink[]
 * }} TileData
 */

export class TileManager {
    /**
     * @param {NavMesh} nav
     */
    constructor(nav) {
        this.nav=nav;
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();
        /** @type {NavMeshMesh} */
        this.mesh = { verts: [], polys: [], regions: [], neighbors: [] };
        /** @type {NavMeshDetail} */
        this.meshdetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        /** @type {NavMeshLink[]} */
        this.links = [];

        /** @type {NavMeshMesh} */
        this.prunemesh = { verts: [], polys: [], regions: [], neighbors: [] };
        /** @type {NavMeshDetail} */
        this.prunemeshdetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        /** @type {NavMeshLink[]} */
        this.prunelinks = [];

        /** @type {NavMeshLink[]} */
        this.supprlink = [];//ladder连接
        /** @type {NavMeshLink[]} */
        this.Extlink=[];//tile间连接
        /** @type {NavMeshLink[]} */
        this.baseLinks = [];//tile内连接
        /** @type {Map<string, {vertBase:number,vertCount:number,polyBase:number,polyCount:number,detailVertBase:number,detailVertCount:number,triBase:number,triCount:number,meshRecBase:number,meshRecCount:number}>} */
        this.tileRanges = new Map();
    }

    /**
     * @param {string} key
     * @param {number} tx
     * @param {number} ty
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink[]} [tileLinks]
     */
    addtile(key, tx, ty, tileMesh, tileDetail, tileLinks = []) {
        if (this.tiles.has(key)) {
            this.removetile(key);
        }
        this.tiles.set(key, {
            tileId: key,
            tx,
            ty,
            mesh: tileMesh,
            detail: tileDetail,
            links: tileLinks
        });
        this._appendTileData(key, tileMesh, tileDetail, tileLinks);
        this._rebuildDeferredLinks(true,true,key);
    }

    /**
     * @param {string} key
     */
    removetile(key) {
        if (!this.tiles.has(key)) return;
        this.tiles.delete(key);
        this._removeTileData(key);
        this._rebuildDeferredLinks(false,false);
    }

    /**
     * @param {string} key
     * @param {number} tx
     * @param {number} ty
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink[]} tileLinks
     */
    updatetile(key, tx, ty, tileMesh, tileDetail, tileLinks) {
        this.removetile(key);
        this.addtile(key, tx, ty, tileMesh, tileDetail, tileLinks);
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {NavMeshDetail} meshdetail
     */
    buildLadderLinksForMesh(mesh, meshdetail) {
        if (!LADDER) return [];
        this.ladderlinkbuilder = new LadderLinkBuilder(mesh, meshdetail);
        return this.ladderlinkbuilder.init();
    }
    updatemesh()
    {
        const merged = this.return();
        this.nav.mesh = merged.mesh;
        this.nav.meshdetail = merged.meshdetail;
        this.nav.links = merged.links;
    }
    return() {
        if (TILE_OPTIMIZATION_1)
            return {
                mesh: this.prunemesh,
                meshdetail: this.prunemeshdetail,
                links: this.prunelinks
            }
        return {
            mesh: this.mesh,
            meshdetail: this.meshdetail,
            links: this.links
        };
    }

    /**
     * 初始化，什么都没有
     * @param {tile} tileBuilder
     */
    rebuildAll(tileBuilder) {
        this.tiles.clear();
        this.tileRanges.clear();
        this.mesh = { verts: [], polys: [], regions: [], neighbors: [] };
        this.meshdetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        this.baseLinks = [];
        this.links = [];
        this.supprlink = [];
        this.Extlink=[];

        const timing = {
            hfInit: 0,
            region: 0,
            contour: 0,
            poly: 0,
            detail: 0,
            merge: 0,
            jumpLinks: 0,
        };

        /** @type {{tx:number,ty:number}[]} */
        const errorTiles = [];

        for (let ty = 0; ty < tileBuilder.tilesY; ty++) {
            for (let tx = 0; tx < tileBuilder.tilesX; tx++) {
                const tileData = tileBuilder.buildTile(tx, ty);
                timing.hfInit += tileData.timing.hfInit;
                timing.region += tileData.timing.region;
                timing.contour += tileData.timing.contour;
                timing.poly += tileData.timing.poly;
                timing.detail += tileData.timing.detail;
                timing.merge += tileData.timing.merge;
                timing.jumpLinks += tileData.timing.jumpLinks;
                if (tileData.hasError) errorTiles.push({ tx, ty });
                const key = tileData.tileId;
                this.tiles.set(key, {
                    tileId: key,
                    tx: tileData.tx,
                    ty: tileData.ty,
                    mesh: tileData.mesh,
                    detail: tileData.detail,
                    links: tileData.links
                });
                this._appendTileData(key, tileData.mesh, tileData.detail, tileData.links);
                //this._rebuildDeferredLinks(true,false);
            }
        }
        this._rebuildDeferredLinks(true,true);
        if (errorTiles.length > 0) {
            const dedup = new Map();
            for (const tile of errorTiles) dedup.set(`${tile.tx}|${tile.ty}`, tile);
            const drawTiles = Array.from(dedup.values());
            tileBuilder.debugDrawErrorTiles(drawTiles, 60);
            Instance.Msg(`Tile报错统计: ${drawTiles.length} 个tile存在步骤报错，已在地图高亮`);
        }
        if (TILE_OPTIMIZATION_1) this.pruneUnreachablePolys();
        Instance.Msg(`Tile阶段耗时统计: 体素化=${timing.hfInit}ms, 区域=${timing.region}ms, 轮廓=${timing.contour}ms, 多边形=${timing.poly}ms, 细节=${timing.detail}ms, 合并=${timing.merge}ms`);
        return { timing, errorTiles };
    }

    /**
     * @param {string} tileId
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink[]} tileLinks
     */
    _appendTileData(tileId, tileMesh, tileDetail, tileLinks) {
        const vertBase = this.mesh.verts.length;
        const polyBase = this.mesh.polys.length;

        const detailVertBase = this.meshdetail.verts.length;
        const triBase = this.meshdetail.tris.length;
        const meshRecBase = this.meshdetail.meshes.length;
        //放入点
        for (const v of tileMesh.verts) this.mesh.verts.push({ x: v.x, y: v.y, z: v.z });
        //放入邻居关系
        for (let i = 0; i < tileMesh.polys.length; i++) {
            const poly = tileMesh.polys[i];
            this.mesh.polys.push(poly.map((vi) => vertBase + vi));
            const oldNei = tileMesh.neighbors[i];
            this.mesh.neighbors.push(oldNei.map((entry) => {
                /**
                 * @type {number[]}
                 */
                const mapped = [];
                for (const n of entry) if (n >= 0) mapped.push(polyBase + n);
                return mapped;
            }));
        }
        //放入细节点
        for (const v of tileDetail.verts) this.meshdetail.verts.push({ x: v.x, y: v.y, z: v.z });
        //放入细节三角形
        for (let i = 0; i < tileDetail.tris.length; i++) {
            const tri = tileDetail.tris[i];
            this.meshdetail.tris.push([
                detailVertBase + tri[0],
                detailVertBase + tri[1],
                detailVertBase + tri[2]
            ]);
            this.meshdetail.triTopoly.push(polyBase + tileDetail.triTopoly[i]);
        }
        //mesh相关数据，长度之类的
        for (const m of tileDetail.meshes) {
            this.meshdetail.meshes.push([
                detailVertBase + m[0],
                m[1],
                triBase + m[2],
                m[3]
            ]);
        }
        //放入链接
        for (const link of tileLinks) {
            if (link.PolyA < 0 || link.PolyB < 0) continue;
            this.baseLinks.push({
                ...link,
                PolyA: polyBase + link.PolyA,
                PolyB: polyBase + link.PolyB,
                PosA: { x: link.PosA.x, y: link.PosA.y, z: link.PosA.z },
                PosB: { x: link.PosB.x, y: link.PosB.y, z: link.PosB.z }
            });
        }
        //记录 tile 在全局 mesh/detail 中的范围
        this.tileRanges.set(tileId, {
            vertBase,
            vertCount: tileMesh.verts.length,
            polyBase,
            polyCount: tileMesh.polys.length,
            detailVertBase,
            detailVertCount: tileDetail.verts.length,
            triBase,
            triCount: tileDetail.tris.length,
            meshRecBase,
            meshRecCount: tileDetail.meshes.length
        });

        this._linkTileWithNeighborTiles(tileId);
    }

    /**
     * 新 tile append 后，增量补齐其与周围 4 个 tile 的跨 tile neighbors。
     * @param {string} tileId
     */
    _linkTileWithNeighborTiles(tileId) {
        const tileData = this.tiles.get(tileId);
        const curRange = this.tileRanges.get(tileId);
        if (!tileData || !curRange || curRange.polyCount <= 0) return;

        const neighborTiles = this._collectNeighborTiles(tileData.tx, tileData.ty, tileId);
        if (neighborTiles.length === 0) return;
        //邻居 tile 的“开放边”
        const openEdgeStore = {
            exact: new Map(),
            buckets: new Map()
        };
        //收集所有邻居中的多边形的开放边(无邻居边)
        for (const nei of neighborTiles) {
            const neiRange = this.tileRanges.get(nei);
            if (!neiRange || neiRange.polyCount <= 0) continue;

            const end = neiRange.polyBase + neiRange.polyCount;
            for (let poly = neiRange.polyBase; poly < end; poly++) {
                const gpoly = this.mesh.polys[poly];
                if (gpoly.length === 0) continue;
                const edgeList = this.mesh.neighbors[poly];

                for (let edge = 0; edge < gpoly.length; edge++) {
                    if (edgeList[edge].length > 0) continue;
                    const va = gpoly[edge];
                    const vb = gpoly[(edge + 1) % gpoly.length];
                    const edgeRec = this.buildOpenEdgeRecord(this.mesh, poly, edge, va, vb);
                    this.addOpenEdge(openEdgeStore, edgeRec);
                }
            }
        }
        const curEnd = curRange.polyBase + curRange.polyCount;
        for (let poly = curRange.polyBase; poly < curEnd; poly++) {
            const gpoly = this.mesh.polys[poly];
            if (gpoly.length === 0) continue;
            const edgeList = this.mesh.neighbors[poly];

            for (let edge = 0; edge < gpoly.length; edge++) {
                if (edgeList[edge].length > 0) continue;

                const va = gpoly[edge];
                const vb = gpoly[(edge + 1) % gpoly.length];

                const reverseKey = `${vb}|${va}`;
                const exactMatched = this.getOpenEdgesByExactKey(openEdgeStore, reverseKey);
                for (const cand of exactMatched) {
                    this.addNeighborLink(this.mesh, poly, edge, cand.poly, cand.edge);
                }

                const fuzzyMatched = this.findOpenEdgesByOverlap(this.mesh, openEdgeStore, poly, edge, curRange.polyBase);
                for (const cand of fuzzyMatched) {
                    this.addNeighborLink(this.mesh, poly, edge, cand.poly, cand.edge);
                }
                //可以维护一个所有tile的边界边
            }
        }
    }

    /**
     * @param {number} tx
     * @param {number} ty
     * @param {string} selfTileId
     * @param {boolean} [includeDiagonal] 是否包含对角线邻居和自己
     * @returns {string[]}
     */
    _collectNeighborTiles(tx, ty, selfTileId, includeDiagonal = false) {
        /** @type {string[]} */
        const out = [];
        for (const [tileId, tileData] of this.tiles.entries()) {
            if (tileId === selfTileId) continue;
            const rawDx = tileData.tx - tx;
            const rawDy = tileData.ty - ty;
            const dx = Math.abs(rawDx);
            const dy = Math.abs(rawDy);
            if (!includeDiagonal) {
                if (dx + dy == 1) {
                    out.push(tileId);
                }
            }
            else {
                if (dx <= 1 && dy <= 1) {
                    out.push(tileId);
                }
            }
        }
        return out;
    }

    /**
     * @param {string} tileId
     */
    _removeTileData(tileId) {
        // 1) 读取该 tile 在全局数组中的范围；没有范围说明未被 append，直接返回。
        const range = this.tileRanges.get(tileId);
        if (!range) return;

        // 2) 预先计算被删除区间的右边界，用于后续索引重映射判断。
        const vertEnd = range.vertBase + range.vertCount;
        const polyEnd = range.polyBase + range.polyCount;
        const dVertEnd = range.detailVertBase + range.detailVertCount;
        const triEnd = range.triBase + range.triCount;

        // 3) 从主 mesh 中删除该 tile 占用的顶点/多边形/邻接记录。
        this.mesh.verts.splice(range.vertBase, range.vertCount);
        this.mesh.polys.splice(range.polyBase, range.polyCount);
        this.mesh.neighbors.splice(range.polyBase, range.polyCount);

        // 4) 重映射剩余多边形中的顶点索引（位于删除区间之后的索引整体左移）。
        for (const poly of this.mesh.polys) {
            for (let i = 0; i < poly.length; i++) {
                if (poly[i] >= vertEnd) poly[i] -= range.vertCount;
            }
        }

        // 5) 重映射剩余多边形邻接：
        //    - 指向被删多边形的邻接要移除；
        //    - 位于被删区间之后的多边形索引要左移。
        for (let p = 0; p < this.mesh.neighbors.length; p++) {
            const edgeList = this.mesh.neighbors[p] || [];
            for (let e = 0; e < edgeList.length; e++) {
                const mapped = [];
                for (const n of edgeList[e]) {
                    if (n >= range.polyBase && n < polyEnd) continue;
                    mapped.push(n >= polyEnd ? n - range.polyCount : n);
                }
                edgeList[e] = mapped;
            }
        }

        // 6) 从 detail mesh 中删除该 tile 占用的 detail 顶点/三角形/triTopoly/mesh 记录。
        this.meshdetail.verts.splice(range.detailVertBase, range.detailVertCount);
        this.meshdetail.tris.splice(range.triBase, range.triCount);
        this.meshdetail.triTopoly.splice(range.triBase, range.triCount);
        this.meshdetail.meshes.splice(range.meshRecBase, range.meshRecCount);

        // 7) 重映射 detail 三角形顶点索引（位于删除区间之后的索引左移）。
        for (const tri of this.meshdetail.tris) {
            for (let i = 0; i < 3; i++) {
                if (tri[i] >= dVertEnd) tri[i] -= range.detailVertCount;
            }
        }

        // 8) 重映射 triTopoly（位于删除区间之后的 poly 索引左移）。
        for (let i = 0; i < this.meshdetail.triTopoly.length; i++) {
            const p = this.meshdetail.triTopoly[i];
            this.meshdetail.triTopoly[i] = p >= polyEnd ? p - range.polyCount : p;
        }

        // 9) 重映射 detail.meshes 里的 detail 顶点基址/三角形基址。
        for (const m of this.meshdetail.meshes) {
            if (m[0] >= dVertEnd) m[0] -= range.detailVertCount;
            if (m[2] >= triEnd) m[2] -= range.triCount;
        }

        // 10) 重映射 Links：
        //     - 端点落在被删 poly 区间内的 link 直接丢弃；
        //     - 其余 link 的 poly 索引按删除量左移。
        /** @param {NavMeshLink} link */
        const remapLink = (link) => {
            if (link.PolyA >= range.polyBase && link.PolyA < polyEnd) return null;
            if (link.PolyB >= range.polyBase && link.PolyB < polyEnd) return null;
            return {
                ...link,
                PolyA: link.PolyA >= polyEnd ? link.PolyA - range.polyCount : link.PolyA,
                PolyB: link.PolyB >= polyEnd ? link.PolyB - range.polyCount : link.PolyB,
            };
        };

        this.baseLinks = this.baseLinks.map(remapLink).filter((l) => !!l);
        this.supprlink = this.supprlink.map(remapLink).filter((l) => !!l);
        this.Extlink = this.Extlink.map(remapLink).filter((l) => !!l);

        // 11) 删除该 tile 的范围记录，并把其后 tile 的各类 base 统一左移。
        this.tileRanges.delete(tileId);
        for (const [k, r] of this.tileRanges.entries()) {
            if (r.vertBase > range.vertBase) r.vertBase -= range.vertCount;
            if (r.polyBase > range.polyBase) r.polyBase -= range.polyCount;
            if (r.detailVertBase > range.detailVertBase) r.detailVertBase -= range.detailVertCount;
            if (r.triBase > range.triBase) r.triBase -= range.triCount;
            if (r.meshRecBase > range.meshRecBase) r.meshRecBase -= range.meshRecCount;
            this.tileRanges.set(k, r);
        }
    }
    /**
     * @param {boolean} Extjump
     * @param {boolean} ladder  
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     */
    _rebuildDeferredLinks(Extjump,ladder,targettileId) {
        if(Extjump)this.Extlink = new JumpLinkBuilder(this.mesh).initInterTile(this._buildPolyTileKeys(targettileId), this.Extlink,targettileId);
        if(ladder)this.supprlink= this.buildLadderLinksForMesh(this.mesh, this.meshdetail);

        this.links = [...this.baseLinks,...this.Extlink, ...this.supprlink];
    }

    /**
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     * @returns {(string)[]}
     */
    _buildPolyTileKeys(targettileId) {
        /**
         * @type {string[]}
         */
        let neitileid = [];
        const polyTileKeys = new Array(this.mesh.polys.length);
        if (targettileId) {
            const tileData = this.tiles.get(targettileId);
            if (tileData) neitileid=this._collectNeighborTiles(tileData.tx, tileData.ty, targettileId, true);
            for (const tileId of neitileid) {
                const range=this.tileRanges.get(tileId);
                if(!range)continue;
                const end = range.polyBase + range.polyCount;
                for (let p = range.polyBase; p < end; p++) {
                    polyTileKeys[p] = tileId;
                }
            }
        }
        else {
            for (const [tileId, range] of this.tileRanges.entries()) {
                const end = range.polyBase + range.polyCount;
                for (let p = range.polyBase; p < end; p++) {
                    polyTileKeys[p] = tileId;
                }
            }
        }
        return polyTileKeys;
    }

    /**
     * @param {tile} tileBuilder
     * @param {{x:number,y:number,z:number}} pos
     */
    rebuildAtPos(tileBuilder, pos) {
        const tileData = tileBuilder.buildTileNavMeshAtPos(pos);
        if (!tileData) return null;
        this.updatetile(tileData.tileId, tileData.tx, tileData.ty, tileData.mesh, tileData.detail, tileData.links);
        if (TILE_OPTIMIZATION_1) this.pruneUnreachablePolys();
        return tileData;
    }

    /**
     * 切换 pos 所在 tile：
     * - 若已存在则删除
     * - 若不存在则构建并添加
     * @param {tile} tileBuilder
     * @param {{x:number,y:number,z:number}} pos
     */
    reversetile(tileBuilder, pos) {
        const tileId = tileBuilder.fromPosGetTile(pos);
        if (this.tiles.has(tileId)) {
            this.removetile(tileId);
            if (TILE_OPTIMIZATION_1) this.pruneUnreachablePolys();
            return false;
        }
        const tileData = tileBuilder.buildTileNavMeshAtPos(pos);
        this.addtile(tileId, tileData.tx, tileData.ty, tileData.mesh, tileData.detail, tileData.links || []);
        if (TILE_OPTIMIZATION_1) this.pruneUnreachablePolys();
        return true;
    }

    pruneUnreachablePolys() {
        if (!this.mesh || !this.meshdetail || this.mesh.polys.length === 0) return;

        const polyCount = this.mesh.polys.length;
        const reachable = new Uint8Array(polyCount);
        const uf = new UnionFind(polyCount);

        for (let p = 0; p < polyCount; p++) {
            const nei = this.mesh.neighbors[p];
            for (const edgeNei of nei) {
                for (const n of edgeNei) {
                    if (n >= 0 && n < polyCount) uf.union(p, n);
                }
            }
        }

        /** @type {NavMeshLink[]} */
        const links = this.links;
        for (const link of links) {
            if (link.PolyA >= 0 && link.PolyA < polyCount && link.PolyB >= 0 && link.PolyB < polyCount) {
                uf.union(link.PolyA, link.PolyB);
            }
        }
        Tool.buildSpatialIndex(this.mesh);
        /** @type {number[]} */
        const seedPolys = [];
        const slist = Instance.FindEntitiesByClass("info_target");
        for (const ent of slist) {
            if (ent.GetEntityName() === "navmesh") {
                const seed = Tool.findNearestPoly(ent.GetAbsOrigin(), this.mesh).poly;
                if (seed >= 0 && seed < polyCount) seedPolys.push(seed);
            }
        }

        if (seedPolys.length === 0) {
            Instance.Msg("可达性筛选跳过: 未找到 info_target{name=navmesh} 种子");
            return;
        }

        const keepRoots = new Set();
        for (const seed of seedPolys) keepRoots.add(uf.find(seed));
        for (let i = 0; i < polyCount; i++) {
            if (keepRoots.has(uf.find(i))) reachable[i] = 1;
        }

        let keepCount = 0;
        for (let i = 0; i < polyCount; i++) if (reachable[i]) keepCount++;
        if (keepCount === polyCount) return;

        const oldToNewPoly = new Int32Array(polyCount).fill(-1);
        let newPolyCount = 0;
        for (let i = 0; i < polyCount; i++) {
            if (reachable[i]) oldToNewPoly[i] = newPolyCount++;
        }

        /** @type {NavMeshMesh} */
        const newMesh = { verts: [], polys: [], regions: [], neighbors: [] };
        const oldToNewMeshVert = new Map();

        for (let oldPi = 0; oldPi < polyCount; oldPi++) {
            if (!reachable[oldPi]) continue;

            const oldPoly = this.mesh.polys[oldPi];
            const newPoly = [];
            for (const oldVi of oldPoly) {
                if (!oldToNewMeshVert.has(oldVi)) {
                    oldToNewMeshVert.set(oldVi, newMesh.verts.length);
                    newMesh.verts.push(this.mesh.verts[oldVi]);
                }
                newPoly.push(oldToNewMeshVert.get(oldVi));
            }
            newMesh.polys.push(newPoly);

            const oldNei = this.mesh.neighbors[oldPi];
            newMesh.neighbors.push(oldNei.map((entry) => {
                const mapped = [];
                for (const n of entry) {
                    if (n >= 0 && reachable[n]) mapped.push(oldToNewPoly[n]);
                }
                return mapped;
            }));
        }

        /** @type {NavMeshDetail} */
        const newDetail = { verts: [], tris: [], triTopoly: [], meshes: [] };
        const oldToNewDetailVert = new Map();

        for (let oldPi = 0; oldPi < polyCount; oldPi++) {
            if (!reachable[oldPi]) continue;
            const newPi = oldToNewPoly[oldPi];

            const meshRec = this.meshdetail.meshes[oldPi];
            if (!meshRec) {
                newDetail.meshes.push([0, 0, newDetail.tris.length, 0]);
                continue;
            }

            const baseTri = meshRec[2];
            const triCount = meshRec[3];
            const triStartNew = newDetail.tris.length;

            for (let ti = baseTri; ti < baseTri + triCount; ti++) {
                const oldTri = this.meshdetail.tris[ti];
                if (!oldTri) continue;
                /** @type {number[]} */
                const remappedTri = [];
                for (const oldDvi of oldTri) {
                    if (!oldToNewDetailVert.has(oldDvi)) {
                        oldToNewDetailVert.set(oldDvi, newDetail.verts.length);
                        newDetail.verts.push(this.meshdetail.verts[oldDvi]);
                    }
                    remappedTri.push(oldToNewDetailVert.get(oldDvi));
                }
                newDetail.tris.push(remappedTri);
                newDetail.triTopoly.push(newPi);
            }

            newDetail.meshes.push([0, 0, triStartNew, newDetail.tris.length - triStartNew]);
        }

        this.prunemesh = newMesh;
        this.prunemeshdetail = newDetail;

        const remappedLinks = [];
        for (const link of this.links) {
            const na = oldToNewPoly[link.PolyA];
            const nb = oldToNewPoly[link.PolyB];
            if (na < 0 || nb < 0) continue;
            remappedLinks.push({
                ...link,
                PolyA: na,
                PolyB: nb,
            });
        }
        this.prunelinks = remappedLinks;

        Instance.Msg(`可达性筛选完成: ${polyCount} -> ${keepCount}`);
    }
    /**
     * @param {NavMeshMesh} mesh
     * @param {number} poly
     * @param {number} edge
     * @param {number} va
     * @param {number} vb
     */
    buildOpenEdgeRecord(mesh, poly, edge, va, vb) {
        const a = mesh.verts[va];
        const b = mesh.verts[vb];
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const len = Math.hypot(dx, dy);
        const major = Math.abs(dx) >= Math.abs(dy) ? 0 : 1;
        const lineCoord = major === 0 ? (a.y + b.y) * 0.5 : (a.x + b.x) * 0.5;
        const pa = major === 0 ? a.x : a.y;
        const pb = major === 0 ? b.x : b.y;
        const projMin = Math.min(pa, pb);
        const projMax = Math.max(pa, pb);
        const dirX = len > 1e-6 ? dx / len : 0;
        const dirY = len > 1e-6 ? dy / len : 0;
        const centerZ = (a.z + b.z) * 0.5;
        const bucketScale = Math.max(1e-4, MESH_CELL_SIZE_XY * 0.6);
        const bucketId = Math.round(lineCoord / bucketScale);
        return { poly, edge, va, vb, exactKey: `${va}|${vb}`, major, lineCoord, projMin, projMax, dirX, dirY, centerZ, bucketId, };
    }

    /**
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} store
     * @param {any} edgeRec
     */
    addOpenEdge(store, edgeRec) {
        const list = Tool.getOrCreateArray(store.exact, edgeRec.exactKey);
        if (!list.includes(edgeRec)) list.push(edgeRec);

        const bucketKey = `${edgeRec.major}|${edgeRec.bucketId}`;
        const bucket = Tool.getOrCreateArray(store.buckets, bucketKey);
        if (!bucket.includes(edgeRec)) bucket.push(edgeRec);
    }

    /**
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} store
     * @param {string} key
     */
    getOpenEdgesByExactKey(store, key) {
        const list = store.exact.get(key);
        if (!list || list.length === 0) return [];
        return list;
    }

    /**
    * @param {NavMeshMesh} mesh
     * @param {{exact:Map<string,any[]>,buckets:Map<string,any[]>}} store
     * @param {number} poly
    * @param {number} edge
    * @param {number} tilePolyStart
     */
    findOpenEdgesByOverlap(mesh, store, poly, edge, tilePolyStart) {
        const gpoly = mesh.polys[poly];
        const va = gpoly[edge];
        const vb = gpoly[(edge + 1) % gpoly.length];
        const cur = this.buildOpenEdgeRecord(mesh, poly, edge, va, vb);

        const lineTol = MESH_CELL_SIZE_XY * 0.6;
        const maxProjGapXY = MESH_CELL_SIZE_XY;
        const minXYOverlap = 0.1;
        const maxZDiff = MAX_WALK_HEIGHT * MESH_CELL_SIZE_Z;

        /** @type {any[]} */
        const candidates = [];
        const dedup = new Set();
        for (let b = cur.bucketId - 1; b <= cur.bucketId + 1; b++) {
            const bucketKey = `${cur.major}|${b}`;
            const bucket = store.buckets.get(bucketKey);
            if (!bucket || bucket.length === 0) continue;

            for (const candidate of bucket) {
                if (candidate.poly === poly) continue;
                if (candidate.poly >= tilePolyStart) continue;
                if (Math.abs(candidate.lineCoord - cur.lineCoord) > lineTol) continue;

                const dot = cur.dirX * candidate.dirX + cur.dirY * candidate.dirY;
                if (dot > -0.8) continue;

                const curSeg = this.getEdgeProjectionSegments(mesh, cur);
                const candSeg = this.getEdgeProjectionSegments(mesh, candidate);
                const projGapX = this.getProjectionGap(curSeg.xMin, curSeg.xMax, candSeg.xMin, candSeg.xMax);
                const projGapY = this.getProjectionGap(curSeg.yMin, curSeg.yMax, candSeg.yMin, candSeg.yMax);
                const projGapXY = Math.hypot(projGapX, projGapY);
                if (projGapXY >= maxProjGapXY) continue;

                const overlapMin = Math.max(cur.projMin, candidate.projMin);
                const overlapMax = Math.min(cur.projMax, candidate.projMax);
                if (overlapMax < overlapMin) continue;
                if ((overlapMax - overlapMin) < minXYOverlap) continue;

                const curOverlapZ = this.getEdgeZRangeOnMajorOverlap(mesh, cur, cur.major, overlapMin, overlapMax);
                const candOverlapZ = this.getEdgeZRangeOnMajorOverlap(mesh, candidate, cur.major, overlapMin, overlapMax);
                const projGapZ = this.getProjectionGap(curOverlapZ.min, curOverlapZ.max, candOverlapZ.min, candOverlapZ.max);
                if (projGapZ >= maxZDiff) continue;

                const ck = `${candidate.poly}|${candidate.edge}`;
                if (dedup.has(ck)) continue;
                dedup.add(ck);
                candidates.push(candidate);
            }
        }

        return candidates;
    }

    /**
     * 返回边在 x/y/z 轴上的投影区间。
    * @param {NavMeshMesh} mesh
     * @param {{va:number,vb:number}} edgeRec
     */
    getEdgeProjectionSegments(mesh, edgeRec) {
        const a = mesh.verts[edgeRec.va];
        const b = mesh.verts[edgeRec.vb];
        return {
            xMin: Math.min(a.x, b.x),
            xMax: Math.max(a.x, b.x),
            yMin: Math.min(a.y, b.y),
            yMax: Math.max(a.y, b.y),
            zMin: Math.min(a.z, b.z),
            zMax: Math.max(a.z, b.z),
        };
    }

    /**
     * 两个一维投影区间的间距（有重叠时为0）。
     * @param {number} minA
     * @param {number} maxA
     * @param {number} minB
     * @param {number} maxB
     */
    getProjectionGap(minA, maxA, minB, maxB) {
        const overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
        if (overlap >= 0) return 0;
        return -overlap;
    }

    /**
     * 取边在“主轴重叠区间”内的 Z 投影区间。
    * @param {NavMeshMesh} mesh
     * @param {{va:number,vb:number}} edgeRec
    * @param {number} major 0:x 轴, 1:y 轴
     * @param {number} overlapMin
     * @param {number} overlapMax
     */
    getEdgeZRangeOnMajorOverlap(mesh, edgeRec, major, overlapMin, overlapMax) {
        const a = mesh.verts[edgeRec.va];
        const b = mesh.verts[edgeRec.vb];

        const ca = major === 0 ? a.x : a.y;
        const cb = major === 0 ? b.x : b.y;
        const dc = cb - ca;

        if (Math.abs(dc) <= 1e-6) {
            const zMin = Math.min(a.z, b.z);
            const zMax = Math.max(a.z, b.z);
            return { min: zMin, max: zMax };
        }

        const t0 = Math.max(0, Math.min(1, (overlapMin - ca) / dc));
        const t1 = Math.max(0, Math.min(1, (overlapMax - ca) / dc));

        const z0 = a.z + (b.z - a.z) * t0;
        const z1 = a.z + (b.z - a.z) * t1;

        return {
            min: Math.min(z0, z1),
            max: Math.max(z0, z1),
        };
    }
    /**
    * @param {NavMeshMesh} mesh
     * @param {number} polyA
     * @param {number} edgeA
     * @param {number} polyB
     * @param {number} edgeB
     */
    addNeighborLink(mesh, polyA, edgeA, polyB, edgeB) {
        const listA = mesh.neighbors[polyA][edgeA];
        const listB = mesh.neighbors[polyB][edgeB];
        if (!listA.includes(polyB)) listA.push(polyB);
        if (!listB.includes(polyA)) listB.push(polyA);
        mesh.neighbors[polyA][edgeA] = listA;
        mesh.neighbors[polyB][edgeB] = listB;
    }
}
