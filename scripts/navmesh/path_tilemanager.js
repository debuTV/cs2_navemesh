/** @typedef {import("./path_manager").NavMeshMesh} NavMeshMesh */
/** @typedef {import("./path_manager").NavMeshDetail} NavMeshDetail */
/** @typedef {import("./path_manager").NavMeshLink} NavMeshLink */
/** @typedef {import("./path_tile").tile} tile */
import { Instance } from "cs_script/point_script";
import { LADDER, MAX_LINKS, MAX_POLYS, MAX_TRIS, MAX_VERTS, MAX_WALK_HEIGHT, MESH_CELL_SIZE_XY, MESH_CELL_SIZE_Z, PathState, TILE_OPTIMIZATION_1 } from "./path_const";
import { Tool, UnionFind } from "./util/tool";
import { LadderLinkBuilder } from "./path_ladderlinkbuild";
import { JumpLinkBuilder } from "./path_jumplinkbuild";
import { NavMesh } from "./path_manager";
import { MapJUMPLinkBuilder } from "./path_mapjumplinkbuild";
import { PortalLinkBuilder } from "./path_portallinkbuild";

/**
 * @typedef {{
 *  tileId:string,
 *  tx:number,
 *  ty:number,
 *  mesh:NavMeshMesh,
 *  detail:NavMeshDetail,
 *  links:NavMeshLink
 * }} TileData
 */
export function newmesh()
{
    return{
        verts: new Float32Array(MAX_VERTS*3),
        vertslength: 0,
        polys: new Int32Array(MAX_POLYS*2),
        polyslength: 0,
        regions: new Int16Array(0),///这里和之后都不会用到，先放个空数组占位
        neighbors: new Array(MAX_POLYS)
    };
}
export function newdetailmesh()
{
    return{
        verts: new Float32Array(MAX_TRIS*3*3),
        vertslength: 0,
        tris: new Uint16Array(MAX_TRIS*3),
        trislength: 0,
        triTopoly: new Uint16Array(MAX_TRIS),
        baseVert: new Uint16Array(MAX_POLYS),
        vertsCount: new Uint16Array(MAX_POLYS),
        baseTri: new Uint16Array(MAX_POLYS),
        triCount: new Uint16Array(MAX_POLYS)
    };
}
export function newlink()
{
    return{
        poly:new Uint16Array(MAX_LINKS*2),
        cost:new Float32Array(MAX_LINKS),
        type:new Uint8Array(MAX_LINKS),
        pos:new Float32Array(MAX_LINKS*6),
        length:0
    };
}
export class TileManager {
    /**
     * @param {NavMesh} nav
     */
    constructor(nav) {
        this.nav=nav;
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();
        /** @type {NavMeshMesh} */
        this.mesh=newmesh();
        /** @type {NavMeshDetail} */
        this.meshdetail=newdetailmesh();
        /** @type {NavMeshLink} */
        this.links= newlink();

        /** @type {NavMeshMesh} */
        this.prunemesh;
        /** @type {NavMeshDetail} */
        this.prunemeshdetail;
        /** @type {NavMeshLink} */
        this.prunelinks;

        /** @type {NavMeshLink} */
        this.supprlink= newlink();//ladder连接
        /** @type {NavMeshLink} */
        this.Extlink = newlink();//tile间连接
        /** @type {NavMeshLink} */
        this.baseLinks =newlink();//tile内连接

        /** @type {Map<string, {vertBase:number,vertCount:number,polyBase:number,polyCount:number,detailVertBase:number,detailVertCount:number,triBase:number,triCount:number,meshRecBase:number,meshRecCount:number}>} */
        this.tileRanges = new Map();
    }

    /**
     * @param {string} key
     * @param {number} tx
     * @param {number} ty
     * @param {NavMeshMesh} tileMesh
     * @param {NavMeshDetail} tileDetail
     * @param {NavMeshLink} tileLinks
     */
    addtile(key, tx, ty, tileMesh, tileDetail, tileLinks) {
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
     * @param {NavMeshLink} tileLinks
     */
    updatetile(key, tx, ty, tileMesh, tileDetail, tileLinks) {
        this.addtile(key, tx, ty, tileMesh, tileDetail, tileLinks);//48ms
    }
    /**
     * @param {NavMeshMesh} mesh
     */
    buildSupperLinksForMesh(mesh) {
        let merged = this.copyLinks(new LadderLinkBuilder(mesh).init(), new MapJUMPLinkBuilder(mesh).init());
        return this.copyLinks(merged, new PortalLinkBuilder(mesh).init());
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
        this.mesh=newmesh();
        this.meshdetail = newdetailmesh();
        this.links = newlink();
        this.supprlink = newlink();
        this.Extlink=newlink();
        this.baseLinks =newlink();
        
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
                this._rebuildDeferredLinks(true,false,key);
            }
        }
        this._rebuildDeferredLinks(false,true);
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
     * @param {NavMeshLink} tileLinks
     */
    _appendTileData(tileId, tileMesh, tileDetail, tileLinks) {
        const mesh = this.mesh;
        const meshdetail = this.meshdetail;
        const baseLinks = this.baseLinks;
        // 记录本次追加前的全局基址（用于后续写入时做偏移）
        const vertBase = mesh.vertslength; // 顶点基址（顶点数，不是浮点数长度）
        const polyBase = mesh.polyslength; // 多边形基址（多边形计数）

        // 记录 detail 层的基址（细节顶点与细节三角）
        const detailVertBase = meshdetail.vertslength;
        const triBase = meshdetail.trislength;
        const meshRecBase = polyBase; // mesh record 基址与 polyBase 对齐（每个 poly 一条 record）
        
        // =========================
        // 1) 追加多边形：把 tile 的每个 poly 的顶点按顺序追加到全局 verts 中，
        //    并在 polys 中记录该 poly 在 verts 中的 start/end 索引区间
        // =========================
        // append polys
        for (let i = 0; i < tileMesh.polyslength; i++) {
            const tstart = tileMesh.polys[i<<1];
            const tend = tileMesh.polys[(i<<1)+1];
            // poly 在全局 verts 中的起始顶点索引
            const start= mesh.vertslength;
            for (let k = tstart; k <= tend; k++) {

                const sx = tileMesh.verts[k * 3];
                const sy = tileMesh.verts[k * 3 + 1];
                const sz = tileMesh.verts[k * 3 + 2];
                const writeIndex = (mesh.vertslength) * 3;
                mesh.verts[writeIndex] = sx;
                mesh.verts[writeIndex + 1] = sy;
                mesh.verts[writeIndex + 2] = sz;

                mesh.vertslength++;
            }
            const end = mesh.vertslength - 1;
            // 将该 poly 的 start/end 写入 polys（每个 poly 占两个 Int32）
            const pi = mesh.polyslength * 2;
            mesh.polys[pi] = start;
            mesh.polys[pi + 1] = end;
            

            // 把 tile 本地的邻接关系（如果有）映射到全局 poly 索引空间
            const vertCount = tend - tstart + 1;
            mesh.neighbors[mesh.polyslength]=new Array(vertCount);
            for (let ei = 0; ei < vertCount; ei++) 
            {
                const nc=tileMesh.neighbors[i][ei][0];
                mesh.neighbors[mesh.polyslength][ei]=new Int16Array(100);
                mesh.neighbors[mesh.polyslength][ei][0]=nc;
                for(let ni=1;ni<=nc;ni++)
                {
                    const nei = tileMesh.neighbors[i][ei][ni];
                    const mappedNei = polyBase + nei;
                    mesh.neighbors[mesh.polyslength][ei][ni] = mappedNei;
                }
            }
            mesh.polyslength++;
        }

        meshdetail.verts.set(tileDetail.verts.subarray(0, tileDetail.vertslength * 3), detailVertBase * 3);
        meshdetail.vertslength+=tileDetail.vertslength;
        // =========================
        // 3) 追加 detail 三角形（tris）和 tri->poly 映射（triTopoly）到 TypedArray
        //    tris 以三元组存储顶点索引（每个值指向 meshdetail.verts 的顶点索引）
        // =========================

        for (let i = 0; i < tileDetail.trislength; i++) {

            let a = detailVertBase + tileDetail.tris[i * 3];
            let b = detailVertBase + tileDetail.tris[i * 3 + 1];
            let c = detailVertBase + tileDetail.tris[i * 3 + 2];

            const writeIdx = meshdetail.trislength * 3;
            meshdetail.tris[writeIdx] = a;
            meshdetail.tris[writeIdx + 1] = b;
            meshdetail.tris[writeIdx + 2] = c;

            meshdetail.triTopoly[meshdetail.trislength] = polyBase + tileDetail.triTopoly[i];
            meshdetail.trislength++;
        }

        // =========================
        // 4) 追加每个 poly 对应的 mesh record（baseVert, vertsCount, baseTri, triCount）
        //    这些数组以 poly 索引为下标，存储该 poly 的细节数据在全局数组中的起点与计数
        // =========================
        for (let i = 0; i < tileMesh.polyslength; i++) {

            const gi = meshRecBase + i;

            meshdetail.baseVert[gi] = detailVertBase + tileDetail.baseVert[i];
            meshdetail.vertsCount[gi] = tileDetail.vertsCount[i];
            meshdetail.baseTri[gi] = triBase + tileDetail.baseTri[i];
            meshdetail.triCount[gi] = tileDetail.triCount[i];
        }
        // 追加link
        const blid=baseLinks.length;
        baseLinks.cost.set(tileLinks.cost.subarray(0, tileLinks.length), blid);
        baseLinks.type.set(tileLinks.type.subarray(0, tileLinks.length), blid);
        baseLinks.pos.set(tileLinks.pos.subarray(0, tileLinks.length * 6), blid * 6);

        for (let i=0;i<tileLinks.length;i++)
        {
            baseLinks.poly[(blid+i)<<1]=polyBase+tileLinks.poly[i<<1];
            baseLinks.poly[((blid+i)<<1)+1]=polyBase+tileLinks.poly[(i<<1)+1];
        }
        baseLinks.length+=tileLinks.length;
        //记录 tile 在全局 mesh/detail 中的范围
        this.tileRanges.set(tileId, {
            vertBase,
            vertCount: mesh.vertslength-vertBase,
            polyBase,
            polyCount: tileMesh.polyslength,
            detailVertBase,
            detailVertCount: tileDetail.vertslength,
            triBase,
            triCount: tileDetail.trislength,
            meshRecBase,
            meshRecCount: tileMesh.polyslength
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

        const neighborTiles = this._collectNeighborTiles(tileData.tx, tileData.ty);
        if (neighborTiles.length === 0) return;
        //邻居 tile 的“开放边”
        const openEdgeStorebuckets = new Map();
        // =========================
        // 1️⃣ 收集邻居 tile 的开放边
        // =========================
        //收集所有邻居中的多边形的开放边(无邻居边)
        for (const nei of neighborTiles) {
            const neiRange = this.tileRanges.get(nei);
            if (!neiRange || neiRange.polyCount <= 0) continue;

            const end = neiRange.polyBase + neiRange.polyCount;
            for (let poly = neiRange.polyBase; poly < end; poly++) {
                const polyStart = this.mesh.polys[poly << 1];
                const polyEnd   = this.mesh.polys[(poly << 1) + 1];
                const vertCount = polyEnd - polyStart + 1;
                for (let edge = 0; edge < vertCount; edge++) 
                {
                    if (this.mesh.neighbors[poly][edge][0] > 0) continue; // 有邻居
                    const va = polyStart + edge;
                    const vb = polyStart + ((edge + 1) % vertCount);
                    const edgeRec = this.buildOpenEdgeRecord(this.mesh, poly, edge, va, vb);

                    const bucketKey = `${edgeRec.major}|${edgeRec.bucketId}`;
                    const bucket = Tool.getOrCreateArray(openEdgeStorebuckets, bucketKey);
                    bucket.push(edgeRec);
                }
            }
        }
        // =========================
        // 2️⃣ 当前 tile 尝试匹配
        // =========================
        const dedup = new Set();
        /**
         * @type {any[]}
         */
        const candidates=[];
        const curEnd = curRange.polyBase + curRange.polyCount;
        for (let poly = curRange.polyBase; poly < curEnd; poly++) {
            const polyStart = this.mesh.polys[poly << 1];
            const polyEnd   = this.mesh.polys[(poly << 1) + 1];
            const vertCount = polyEnd - polyStart + 1;
            for (let edge = 0; edge < vertCount; edge++) 
            {
                if (this.mesh.neighbors[poly][edge][0] > 0) continue;
                dedup.clear();
                candidates.length = 0;
                // ===== 2️⃣ 模糊匹配 =====
                this.findOpenEdgesByOverlap(
                    this.mesh,
                    openEdgeStorebuckets,
                    poly,
                    edge,
                    curRange.polyBase,
                    candidates,
                    dedup
                );

                for (const cand of candidates) {
                    this.addNeighborLink(this.mesh, poly, edge, cand.poly, cand.edge);
                }
                //可以维护一个所有tile的边界边
            }
        }
    }

    /**
     * @param {number} tx
     * @param {number} ty
     * @param {boolean} [includeDiagonal] 是否包含对角线邻居和自己
     * @returns {string[]}
     */
    _collectNeighborTiles(tx, ty, includeDiagonal = false) {
        /** @type {string[]} */
        const out = [];
        // 4/8邻居偏移
        const offsets = includeDiagonal
            ? [
                [-1, -1], [0, -1], [1, -1],
                [-1,  0], [0,  0], [1,  0],
                [-1,  1], [0,  1], [1,  1]
            ]
            : [
                [0, -1], [-1, 0], [1, 0], [0, 1]
            ];
        for (const [dx, dy] of offsets) {
            const ntx = tx + dx;
            const nty = ty + dy;
            // 构造 tileId，需与 addtile 时一致
            const tileId = `${ntx}_${nty}`;
            if (this.tiles.has(tileId)) out.push(tileId);
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
        const mesh = this.mesh;
        const detail = this.meshdetail;

        // 2) 预先计算被删除区间的右边界，用于后续索引重映射判断。
        const vertEnd = range.vertBase + range.vertCount;
        const polyEnd = range.polyBase + range.polyCount;
        const dVertEnd = range.detailVertBase + range.detailVertCount;
        const triEnd = range.triBase + range.triCount;

        // 3) 从主 mesh 中删除该 tile 占用的顶点/多边形/邻接记录。
        // =========================
        // 1️⃣ 删除 mesh verts（float x3）
        // =========================
        const vertMoveCount = mesh.vertslength - vertEnd;
        if (vertMoveCount > 0) {
            mesh.verts.copyWithin(
                range.vertBase * 3,
                vertEnd * 3,
                mesh.vertslength * 3
            );
        }
        mesh.vertslength -= range.vertCount;
        // =========================
        // 2️⃣ 删除 polys
        // =========================
        const polyMoveCount = mesh.polyslength - polyEnd;
        const oldpolylen=mesh.polyslength;
        if (polyMoveCount > 0) {
            mesh.polys.copyWithin(
                range.polyBase * 2,
                polyEnd * 2,
                mesh.polyslength * 2
            );
        }
        mesh.polyslength -= range.polyCount;

        // neighbors 也要左移
        mesh.neighbors.splice(range.polyBase, range.polyCount);

        // =========================
        // 3️⃣ 重映射 poly 顶点索引
        // =========================
        for (let i = range.polyBase; i < mesh.polyslength; i++) {

            const pi = i << 1;

            let start = mesh.polys[pi];
            let end   = mesh.polys[pi + 1];

            if (start >= vertEnd) {
                start -= range.vertCount;
                end   -= range.vertCount;
                mesh.polys[pi] = start;
                mesh.polys[pi + 1] = end;
            }
        }
        // =========================
        // 4️⃣ 重映射 neighbors poly index
        // =========================
        for (let p = 0; p < mesh.polyslength; p++) {

            const ppolyStart = mesh.polys[p << 1];
            const ppolyEnd   = mesh.polys[(p << 1) + 1];
            const vertCount = ppolyEnd - ppolyStart + 1;

            for (let e = 0; e < vertCount; e++) {

                const list = mesh.neighbors[p][e];
                const count = list[0];

                let write = 1;

                for (let i = 1; i <= count; i++) {

                    const n = list[i];

                    if (n >= range.polyBase && n < polyEnd) {
                        continue; // 删除
                    }

                    list[write++] = n >= polyEnd
                        ? n - range.polyCount
                        : n;
                }

                list[0] = write - 1;
            }
        }

        // =========================
        // 5️⃣ 删除 detail verts
        // =========================
        const dMove = detail.vertslength - dVertEnd;
        if (dMove > 0) {
            detail.verts.copyWithin(
                range.detailVertBase * 3,
                dVertEnd * 3,
                detail.vertslength * 3
            );
        }
        detail.vertslength -= range.detailVertCount;
        // =========================
        // 6️⃣ 删除 detail tris
        // =========================
        const triMove = detail.trislength - triEnd;
        if (triMove > 0) {
            detail.tris.copyWithin(
                range.triBase * 3,
                triEnd * 3,
                detail.trislength * 3
            );

            detail.triTopoly.copyWithin(
                range.triBase,
                triEnd,
                detail.trislength
            );
        }
        detail.trislength -= range.triCount;

        // =========================
        // 7️⃣ 重映射 detail tris 顶点
        // =========================
        for (let i = range.triBase*3; i < detail.trislength * 3; i++) {
            const v = detail.tris[i];
            if (v >= dVertEnd) {
                detail.tris[i] = v - range.detailVertCount;
            }
        }

        // =========================
        // 8️⃣ 重映射 triTopoly
        // =========================
        for (let i = range.triBase; i < detail.trislength; i++) {
            const p = detail.triTopoly[i];
            if (p >= polyEnd) {
                detail.triTopoly[i] = p - range.polyCount;
            }
        }

        detail.baseVert.copyWithin(range.polyBase, polyEnd, oldpolylen);
        detail.vertsCount.copyWithin(range.polyBase, polyEnd, oldpolylen);
        detail.baseTri.copyWithin(range.polyBase, polyEnd, oldpolylen);
        detail.triCount.copyWithin(range.polyBase, polyEnd, oldpolylen);
        for (let i = range.polyBase; i < mesh.polyslength; i++) {
            if (detail.baseVert[i] >= dVertEnd) detail.baseVert[i] -= range.detailVertCount;
            if (detail.baseTri[i]  >= triEnd)   detail.baseTri[i]  -= range.triCount;
        }

        // =========================
        // 9️⃣ 重映射 Links（TypedArray 版本）
        // =========================
        const remapLinks = (/** @type {NavMeshLink} */ linkSet) => {

            let write = 0;

            for (let i = 0; i < linkSet.length; i++) {

                const a = linkSet.poly[i << 1];
                const b = linkSet.poly[(i << 1) + 1];

                if (
                    (a >= range.polyBase && a < polyEnd) ||
                    (b >= range.polyBase && b < polyEnd)
                ) {
                    continue;
                }

                linkSet.poly[write << 1] =
                    a >= polyEnd ? a - range.polyCount : a;

                linkSet.poly[(write << 1) + 1] =
                    b >= polyEnd ? b - range.polyCount : b;

                linkSet.cost[write] = linkSet.cost[i];
                linkSet.type[write] = linkSet.type[i];

                for (let k = 0; k < 6; k++) {
                    linkSet.pos[write * 6 + k] =
                        linkSet.pos[i * 6 + k];
                }

                write++;
            }

            linkSet.length = write;
        };

        remapLinks(this.baseLinks);
        remapLinks(this.Extlink);
        remapLinks(this.supprlink);

        // =========================
        // 🔟 更新 tileRanges
        // =========================
        this.tileRanges.delete(tileId);

        for (const [k, r] of this.tileRanges.entries()) {

            if (r.vertBase > range.vertBase)
                r.vertBase -= range.vertCount;

            if (r.polyBase > range.polyBase)
                r.polyBase -= range.polyCount;

            if (r.detailVertBase > range.detailVertBase)
                r.detailVertBase -= range.detailVertCount;

            if (r.triBase > range.triBase)
                r.triBase -= range.triCount;

            if (r.meshRecBase > range.meshRecBase)
                r.meshRecBase -= range.meshRecCount;

            this.tileRanges.set(k, r);
        }
    }
    /**
     * @param {string} targettileId
     */
    getedgebytileid(targettileId)
    {
        /**
         * @type {string[]}
         */
        let neitileid = [];
        const tileData = this.tiles.get(targettileId);
        if (tileData) neitileid=this._collectNeighborTiles(tileData.tx, tileData.ty, true);
        const tilemark=new Uint8Array(4096*3);
        const result = new Uint16Array(4096 * 3);
        let edgeCount = 0;
        for (const tileId of neitileid) {
            const range=this.tileRanges.get(tileId);
            if(!range)continue;
            const end = range.polyBase + range.polyCount;
            for (let p = range.polyBase; p < end; p++) {
                const polyStart = this.mesh.polys[p << 1];
                const polyEnd   = this.mesh.polys[(p << 1) + 1];
                const vertCount = polyEnd - polyStart + 1;
                if(targettileId===tileId)tilemark[p]=2;
                else tilemark[p]=1;
                for (let j = 0; j < vertCount; j++) {
                    // 如果没有邻居，就是边界边
                    if (this.mesh.neighbors[p][j][0] === 0) {
                        const vi1 = polyStart + j;
                        const vi2 = polyStart + ((j + 1) % vertCount);
                        const idx =  edgeCount*3;
                        result[idx] = p;
                        result[idx+1] = vi1;
                        result[idx + 2] = vi2;
                        edgeCount++;
                   }
                }
            }
        }
        return { edgeCount, result, tilemark };
    }
    /**
     * @param {boolean} Extjump
     * @param {boolean} Supperlink
     * @param {string} [targettileId] //不传则为全局 tile间 生成，传入则为指定 tile 与其他 tile 之间生成
     */
    _rebuildDeferredLinks(Extjump,Supperlink,targettileId) {
        if(Extjump&&targettileId)
        {
            const { edgeCount, result, tilemark } = this.getedgebytileid(targettileId);
            if(Extjump)this.Extlink = new JumpLinkBuilder(this.mesh).initInterTileIn(edgeCount,result,tilemark,this.Extlink);//15ms
        }
        if(Supperlink)
        {
            Tool.buildSpatialIndex(this.mesh);//ladder最后才会运行，弄完后才会裁剪，裁剪也会使用这个
            this.supprlink= this.buildSupperLinksForMesh(this.mesh);
        }
        let merged = this.copyLinks(this.baseLinks, this.Extlink);
        merged = this.copyLinks(merged, this.supprlink);
        this.links = merged;
    }
    /**
     * 把 b 追加到 a 后面，返回新的 link
     * @param {NavMeshLink} a
     * @param {NavMeshLink} b
     * @returns {NavMeshLink}
     */
    copyLinks(a, b) {
        const total = a.length + b.length;
        /** @type {NavMeshLink} */
        const merged = {
            poly: new Uint16Array(total * 2),
            cost: new Float32Array(total),
            type: new Uint8Array(total),
            pos:  new Float32Array(total * 6),
            length: total
        };

        let linkOff = 0;
        let polyOff = 0;
        let posOff  = 0;

        const append = (/** @type {NavMeshLink} */ src) => {
            if (!src || src.length === 0) return;

            merged.poly.set(src.poly.subarray(0, src.length * 2), polyOff);
            merged.cost.set(src.cost.subarray(0, src.length), linkOff);
            merged.type.set(src.type.subarray(0, src.length), linkOff);
            merged.pos.set(src.pos.subarray(0, src.length * 6), posOff);

            polyOff += src.length * 2;
            linkOff += src.length;
            posOff  += src.length * 6;
        };

        append(a); // 先 a
        append(b); // 再 b（追加到后面）
        return merged;
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
        const polyTileKeys = new Array(this.mesh.polyslength);
        
        if (targettileId) {
            const tileData = this.tiles.get(targettileId);
            if (tileData) neitileid=this._collectNeighborTiles(tileData.tx, tileData.ty, true);
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

    pruneUnreachablePolys() {//15ms
        const mesh = this.mesh;
        const detail = this.meshdetail;
        const polyCount = mesh.polyslength;

        if (polyCount === 0) return;
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
        const reachable = new Uint8Array(polyCount);
        const queue = new Int32Array(polyCount);
        let keepCount = 0;
        let qh = 0, qt = 0;
        // 入队 seed
        for (const s of seedPolys) {
            if (reachable[s]) continue;
            reachable[s] = 1;
            keepCount++;
            queue[qt++] = s;
        }

        // 先把 links 建成按 poly 的邻接（一次性）
        const linkAdj = new Array(polyCount);
        for (let i = 0; i < polyCount; i++) linkAdj[i] = [];
        for (let i = 0; i < this.links.length; i++) 
        {
            const a = this.links.poly[i << 1];
            const b = this.links.poly[(i << 1) + 1];
            if (a >= 0 && a < polyCount && b >= 0 && b < polyCount)
            {
                linkAdj[a].push(b);
                linkAdj[b].push(a);
            }
        }

        // BFS
        while (qh < qt) 
        {
            const p = queue[qh++];

            // 走 neighbors
            const ps = mesh.polys[p << 1];
            const pe = mesh.polys[(p << 1) + 1];
            const edgeCount = pe - ps + 1;
            const edges = mesh.neighbors[p];
            for (let e = 0; e < edgeCount; e++) 
            {
                const list = edges[e];
                const count = list[0] | 0;
                for (let k = 1; k <= count; k++) {
                const n = list[k];
                if (n < 0 || n >= polyCount || reachable[n]) continue;
                reachable[n] = 1;
                keepCount++;
                queue[qt++] = n;
                }
            }

            // 走 links
            const la = linkAdj[p];
            for (let i = 0; i < la.length; i++) 
            {
                const n = la[i];
                if (reachable[n]) continue;
                reachable[n] = 1;
                keepCount++;
                queue[qt++] = n;
            }
        }

        const oldToNewPoly = new Int32Array(polyCount).fill(-1);

        let newPolyCount = 0;
        for (let i = 0; i < polyCount; i++) {
            if (reachable[i]) oldToNewPoly[i] = newPolyCount++;
        }
        // =========================
        // 5️⃣ 统计新 verts 数量
        // =========================

        const vertUsed = new Uint8Array(mesh.vertslength);
        let newVertCount = 0;

        for (let p = 0; p < polyCount; p++) {

            if (!reachable[p]) continue;

            const start = mesh.polys[p<<1];
            const end   = mesh.polys[(p<<1)+1];

            for (let v = start; v <= end; v++) {
                if (!vertUsed[v]) {
                    vertUsed[v] = 1;
                    newVertCount++;
                }
            }
        }

        const vertRemap = new Int32Array(mesh.vertslength).fill(-1);

        let writeV = 0;
        for (let i = 0; i < mesh.vertslength; i++) {
            if (vertUsed[i])
                vertRemap[i] = writeV++;
        }
        // =========================
        // 6️⃣ 构建 prunemesh
        // =========================
        /** @type {NavMeshMesh} */
        const newMesh = {
            verts: new Float32Array(newVertCount * 3),
            polys: new Int32Array(newPolyCount * 2),
            neighbors: new Array(newPolyCount),
            regions: new Int16Array(0),//无用
            polyslength: newPolyCount,
            vertslength: newVertCount
        };
        // verts copy
        for (let i = 0; i < mesh.vertslength; i++) {

            if (!vertUsed[i]) continue;

            const nv = vertRemap[i];

            newMesh.verts[nv*3]     = mesh.verts[i*3];
            newMesh.verts[nv*3 + 1] = mesh.verts[i*3 + 1];
            newMesh.verts[nv*3 + 2] = mesh.verts[i*3 + 2];
        }
        // polys copy
        for (let p = 0; p < polyCount; p++) {

            if (!reachable[p]) continue;

            const np = oldToNewPoly[p];

            const start = mesh.polys[p<<1];
            const end   = mesh.polys[(p<<1)+1];

            newMesh.polys[np<<1]     = vertRemap[start];
            newMesh.polys[(np<<1)+1] = vertRemap[end];

            // neighbors
            //////////////////////
            const edgeList = mesh.neighbors[p];
            const vertCount = end - start + 1;
            const newEdges = new Array(vertCount);

            for (let e = 0; e < vertCount; e++) {

                const list = edgeList[e];
                const count = list[0];

                const newList = new Int16Array(count + 1);

                let w = 1;

                for (let i = 1; i <= count; i++) {

                    const newIdx = oldToNewPoly[list[i]];
                    if (newIdx !== -1)newList[w++] = newIdx;
                }

                newList[0] = w - 1;
                newEdges[e] = newList;
            }

            newMesh.neighbors[np] = newEdges;
        }
        // =========================
        // 7️⃣ 统计 tri 数量
        // =========================

        let newTriCount = 0;

        for (let p = 0; p < polyCount; p++) {

            if (!reachable[p]) continue;
            newTriCount += detail.triCount[p];
        }
        let newDetailVertCount = 0;

        const detailVertRemap = new Int32Array(detail.vertslength);
        detailVertRemap.fill(-1);
        for (let t = 0; t < detail.trislength; t++) {
            if (!reachable[detail.triTopoly[t]]) continue;
            const base = t * 3;
            detailVertRemap[detail.tris[base]]     = newDetailVertCount++;
            detailVertRemap[detail.tris[base + 1]] = newDetailVertCount++;
            detailVertRemap[detail.tris[base + 2]] = newDetailVertCount++;
        }

        /**@type {NavMeshDetail} */
        const newDetail = {
            verts: new Float32Array(newDetailVertCount * 3),
            vertslength: newDetailVertCount,
            tris: new Uint16Array(newTriCount * 3),
            triTopoly: new Uint16Array(newTriCount),
            trislength: newTriCount,
            baseVert: new Uint16Array(newPolyCount),
            vertsCount: new Uint16Array(newPolyCount),
            baseTri: new Uint16Array(newPolyCount),
            triCount: new Uint16Array(newPolyCount)
        };
        for (let i = 0; i < detail.vertslength; i++) {

            const newIdx = detailVertRemap[i];
            if (newIdx === -1) continue;

            newDetail.verts[newIdx*3]     = detail.verts[i*3];
            newDetail.verts[newIdx*3 + 1] = detail.verts[i*3 + 1];
            newDetail.verts[newIdx*3 + 2] = detail.verts[i*3 + 2];
        }
        let writeTri = 0;

        for (let oldP = 0; oldP < polyCount; oldP++) {
            if (!reachable[oldP]) continue;
            const newP = oldToNewPoly[oldP];

            const triBase  = detail.baseTri[oldP];
            const triCount = detail.triCount[oldP];

            newDetail.baseVert[newP] = detail.baseVert[oldP];
            newDetail.vertsCount[newP] = detail.vertsCount[oldP];
            newDetail.baseTri[newP] = writeTri;
            newDetail.triCount[newP] = triCount;

            for (let t = 0; t < triCount; t++) {

                const oldTriIdx = triBase + t;

                const baseOld = oldTriIdx * 3;
                const baseNew = writeTri * 3;

                newDetail.tris[baseNew] =
                    detailVertRemap[detail.tris[baseOld]];

                newDetail.tris[baseNew + 1] =
                    detailVertRemap[detail.tris[baseOld + 1]];

                newDetail.tris[baseNew + 2] =
                    detailVertRemap[detail.tris[baseOld + 2]];

                newDetail.triTopoly[writeTri] = newP;

                writeTri++;
            }
        }
        this.prunemesh = newMesh;
        this.prunemeshdetail = newDetail;
        // =========================
        // 8️⃣ link copy
        // =========================

        const linkSet = this.links;

        let newLinkCount = 0;

        for (let i = 0; i < linkSet.length; i++) {
            const a = oldToNewPoly[linkSet.poly[i<<1]];
            const b = oldToNewPoly[linkSet.poly[(i<<1)+1]];
            if (a !== -1 && b !== -1)
                newLinkCount++;
        }
        /**@type {NavMeshLink} */
        const newLinks = {
            poly: new Uint16Array(newLinkCount * 2),
            cost: new Float32Array(newLinkCount),
            type: new Uint8Array(newLinkCount),
            pos:  new Float32Array(newLinkCount * 6),
            length: newLinkCount
        };

        let w = 0;

        for (let i = 0; i < linkSet.length; i++) {

            const na = oldToNewPoly[linkSet.poly[i<<1]];
            const nb = oldToNewPoly[linkSet.poly[(i<<1)+1]];

            if (na === -1 || nb === -1) continue;

            newLinks.poly[w<<1]     = na;
            newLinks.poly[(w<<1)+1] = nb;
            newLinks.cost[w] = linkSet.cost[i];
            newLinks.type[w] = linkSet.type[i];

            for (let k=0;k<6;k++)
                newLinks.pos[w*6+k] = linkSet.pos[i*6+k];

            w++;
        }
        this.prunelinks = newLinks;
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
        const ax = mesh.verts[va * 3];
        const ay = mesh.verts[va * 3 + 1];
        const az = mesh.verts[va * 3 + 2];

        const bx = mesh.verts[vb * 3];
        const by = mesh.verts[vb * 3 + 1];
        const bz = mesh.verts[vb * 3 + 2];

        const dx = bx - ax;
        const dy = by - ay;

        const len = Math.hypot(dx, dy);
        const major = Math.abs(dx) >= Math.abs(dy) ? 0 : 1;
        const lineCoord = major === 0
        ? (ay + by) * 0.5
        : (ax + bx) * 0.5;

        const pa = major === 0 ? ax : ay;
        const pb = major === 0 ? bx : by;

        const projMin = Math.min(pa, pb);
        const projMax = Math.max(pa, pb);
        const invLen = len > 1e-6 ? 1 / len : 0;

        const dirX = dx * invLen;
        const dirY = dy * invLen;

        const centerZ = (az + bz) * 0.5;

        const bucketScale = Math.max(1e-4, MESH_CELL_SIZE_XY * 0.6);
        const bucketId = Math.round(lineCoord / bucketScale);

        return { poly, edge, va, vb, exactKey: `${va}|${vb}`, major, lineCoord, projMin, projMax, dirX, dirY, centerZ, bucketId, };
    }

    /**
     * @param {NavMeshMesh} mesh
     * @param {Map<string,any[]>} buckets
     * @param {number} poly
     * @param {number} edge
     * @param {number} tilePolyStart
     * @param {any[]} candidates
     * @param {Set<string>} dedup
     */
    findOpenEdgesByOverlap(mesh, buckets, poly, edge, tilePolyStart,candidates,dedup) {

        const polys = mesh.polys;
        const verts = mesh.verts;

        const polyStart = polys[poly << 1];
        const polyEnd   = polys[(poly << 1) + 1];
        const vertCount = polyEnd - polyStart + 1;

        const va = polyStart + edge;
        const vb = polyStart + ((edge + 1) % vertCount);

        const ax = verts[va * 3];
        const ay = verts[va * 3 + 1];
        const az = verts[va * 3 + 2];

        const bx = verts[vb * 3];
        const by = verts[vb * 3 + 1];
        const bz = verts[vb * 3 + 2];

        const dx = bx - ax;
        const dy = by - ay;

        const len = Math.hypot(dx, dy);
        const invLen = len > 1e-6 ? 1 / len : 0;

        const dirX = dx * invLen;
        const dirY = dy * invLen;

        const major = Math.abs(dx) >= Math.abs(dy) ? 0 : 1;

        const lineCoord = major === 0
            ? (ay + by) * 0.5
            : (ax + bx) * 0.5;

        const pa = major === 0 ? ax : ay;
        const pb = major === 0 ? bx : by;

        const projMin = pa < pb ? pa : pb;
        const projMax = pa > pb ? pa : pb;

        const bucketScale = Math.max(1e-4, MESH_CELL_SIZE_XY * 0.6);
        const bucketId = Math.round(lineCoord / bucketScale);

        const lineTol = MESH_CELL_SIZE_XY * 0.6;
        const maxProjGapXY = MESH_CELL_SIZE_XY;
        const minXYOverlap = 0.1;
        const maxZDiff = MAX_WALK_HEIGHT * MESH_CELL_SIZE_Z;

        for (let b = bucketId - 1; b <= bucketId + 1; b++) {

            const bucketKey = `${major}|${b}`;
            const bucket = buckets.get(bucketKey);
            if (!bucket) continue;

            for (let i = 0; i < bucket.length; i++) {

                const candidate = bucket[i];

                if (candidate.poly === poly) continue;
                if (candidate.poly >= tilePolyStart) continue;
                if (Math.abs(candidate.lineCoord - lineCoord) > lineTol) continue;

                const dot = dirX * candidate.dirX + dirY * candidate.dirY;
                if (dot > -0.8) continue;

                // ===== XY 投影 gap =====

                const cva = candidate.va;
                const cvb = candidate.vb;

                const cax = verts[cva * 3];
                const cay = verts[cva * 3 + 1];
                const caz = verts[cva * 3 + 2];

                const cbx = verts[cvb * 3];
                const cby = verts[cvb * 3 + 1];
                const cbz = verts[cvb * 3 + 2];

                const curXMin = ax < bx ? ax : bx;
                const curXMax = ax > bx ? ax : bx;
                const curYMin = ay < by ? ay : by;
                const curYMax = ay > by ? ay : by;

                const candXMin = cax < cbx ? cax : cbx;
                const candXMax = cax > cbx ? cax : cbx;
                const candYMin = cay < cby ? cay : cby;
                const candYMax = cay > cby ? cay : cby;

                const gapX = Math.max(0, Math.max(curXMin, candXMin) - Math.min(curXMax, candXMax));
                const gapY = Math.max(0, Math.max(curYMin, candYMin) - Math.min(curYMax, candYMax));

                if (Math.hypot(gapX, gapY) >= maxProjGapXY) continue;

                // ===== 主轴 overlap =====

                const overlapMin = projMin > candidate.projMin ? projMin : candidate.projMin;
                const overlapMax = projMax < candidate.projMax ? projMax : candidate.projMax;

                if (overlapMax <= overlapMin) continue;
                if ((overlapMax - overlapMin) < minXYOverlap) continue;

                // ===== Z overlap =====

                const ca = major === 0 ? ax : ay;
                const cb = major === 0 ? bx : by;
                const cdc = cb - ca;

                let zMinA, zMaxA;

                if (Math.abs(cdc) <= 1e-6) {
                    zMinA = az < bz ? az : bz;
                    zMaxA = az > bz ? az : bz;
                } else {
                    const inv = 1 / cdc;
                    const t0 = (overlapMin - ca) * inv;
                    const t1 = (overlapMax - ca) * inv;

                    const z0 = az + (bz - az) * t0;
                    const z1 = az + (bz - az) * t1;

                    zMinA = z0 < z1 ? z0 : z1;
                    zMaxA = z0 > z1 ? z0 : z1;
                }

                const cca = major === 0 ? cax : cay;
                const ccb = major === 0 ? cbx : cby;
                const cdc2 = ccb - cca;

                let zMinB, zMaxB;

                if (Math.abs(cdc2) <= 1e-6) {
                    zMinB = caz < cbz ? caz : cbz;
                    zMaxB = caz > cbz ? caz : cbz;
                } else {
                    const inv2 = 1 / cdc2;
                    const t0 = (overlapMin - cca) * inv2;
                    const t1 = (overlapMax - cca) * inv2;

                    const z0 = caz + (cbz - caz) * t0;
                    const z1 = caz + (cbz - caz) * t1;

                    zMinB = z0 < z1 ? z0 : z1;
                    zMaxB = z0 > z1 ? z0 : z1;
                }

                const gapZ = Math.max(0, Math.max(zMinA, zMinB) - Math.min(zMaxA, zMaxB));
                if (gapZ >= maxZDiff) continue;

                const key = candidate.poly + "|" + candidate.edge;
                if (dedup.has(key)) continue;

                dedup.add(key);
                candidates.push(candidate);
            }
        }

        return ;
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
        // list[0] 存数量
        const countA = listA[0];
        let exists = false;

        for (let i = 1; i <= countA; i++) {
            if (listA[i] === polyB) {
                exists = true;
                break;
            }
        }

        if (!exists) {
            listA[0]++;
            listA[listA[0]] = polyB;
        }

        const countB = listB[0];
        exists = false;

        for (let i = 1; i <= countB; i++) {
            if (listB[i] === polyA) {
                exists = true;
                break;
            }
        }

        if (!exists) {
            listB[0]++;
            listB[listB[0]] = polyA;
        }
    }
}
