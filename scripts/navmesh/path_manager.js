import { PolyGraphAStar } from "./path_Astar";
import { FunnelPath } from "./path_funnel";
import { Instance } from "cs_script/point_script";
import { ADJUST_HEIGHT, ADJUST_HEIGHT_DISTANCE, LINK_DEBUG, LOAD_DEBUG, LOAD_STATIC_MESH, OFF_MESH_LINK_COST_SCALE, PLUGIN_ENABLED, POLY_DEBUG, POLY_DETAIL_DEBUG, PRINT_NAV_MESH, TILE_DEBUG, TILE_OPTIMIZATION_1 } from "./path_const";
import { StaticData } from "./path_navemeshstatic";
import { FunnelHeightFixer } from "./path_funnelheightfixer";
import { tile } from "./path_tile";
import { NavMeshDebugTools } from "./path_navmeshdebug";
import { TileManager } from "./path_tilemanager";
import { Tool } from "./util/tool";
import { NVplugin } from "./plugin/plugin_manager";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/** @typedef {import("./path_tilemanager").TileData} TileData */
/**
 * @typedef {{
 *  verts: Float32Array<ArrayBufferLike>,
 *  vertslength: number,
 *  polys: Int32Array<ArrayBufferLike>,
 *  polyslength: number,
 *  regions: Int16Array<ArrayBufferLike>,
 *  neighbors: Int16Array<ArrayBufferLike>[][]
 * }} NavMeshMesh
 */

/**
 * @typedef {{
 *  verts: Float32Array<ArrayBufferLike>,
 *  vertslength: number,
 *  tris: Uint16Array<ArrayBufferLike>,
 *  trislength: number,
 *  triTopoly: Uint16Array<ArrayBufferLike>,
 *  baseVert: Uint16Array<ArrayBufferLike>,
 *  vertsCount: Uint16Array<ArrayBufferLike>,
 *  baseTri: Uint16Array<ArrayBufferLike>,
 *  triCount: Uint16Array<ArrayBufferLike>
 * }} NavMeshDetail
 */

/**
 * @typedef {{
 *  poly: Uint16Array,
 *  cost: Float32Array,
 *  type: Uint8Array,
 *  pos: Float32Array,
 *  length: number
 * }} NavMeshLink
 */

/**
 * @typedef {{
 *  PolyA:number,
 *  PolyB:number,
 *  PosA:Vector,
 *  PosB:Vector,
 *  cost:number,
 *  type:number
 * }} NavMeshLinkARRAY
 */
export class NavMesh {
    constructor() {
        /**@type {PolyGraphAStar} */
        this.astar;
        /**@type {NavMeshMesh} */
        this.mesh;
        /**@type {NavMeshDetail} */
        this.meshdetail;
        /**@type {FunnelPath} */
        this.funnel;
        /**@type {FunnelHeightFixer} */
        this.heightfixer;
        /**@type {NavMeshLink} */
        this.links;
        /** @type {TileManager} */
        this.tileManager = new TileManager(this);
        /** @type {tile} */
        this.tile = new tile();
        this.debugTools = new NavMeshDebugTools(this);
        if(PLUGIN_ENABLED)this.plugin=new NVplugin(this);
        //删除prop_door_rotating实体？也许应该弄一个目录，让作者把门一类的实体名字放里面
    }
    /**
     * 导出导航网格数据为文本字符串
     */
    exportNavData() {
        const charsPerLine = 500;
        const data = {
            tiles: Array.from(this.tileManager.tiles, ([key, td]) => [key, Tool._compactTileData(td)])
        };
        // 使用 JSON 序列化
        const jsonStr = JSON.stringify(data);
        // 2. 将字符串切割成指定长度的块
        Instance.Msg("--- NAV DATA START ---");
        for (let i = 0; i < jsonStr.length; i += charsPerLine) {
            Instance.Msg("+`"+jsonStr.substring(i, i + charsPerLine)+"`");
        }
        Instance.Msg("--- NAV DATA END ---");
    }
    /**
     * 从文本字符串恢复导航网格
     * @param {string} jsonStr 
     */
    importNavData(jsonStr) {
        try {
            const cleanJson = jsonStr.replace(/\s/g, "");

            const data = JSON.parse(cleanJson);

            // 1. 恢复核心网格数据
            for (const tile of data.tiles) {
                const tiledata=tile[1];
                const key = tiledata.tileId;
                const mesh = Tool.toTypedMesh(tiledata.mesh);
                const detail = Tool.toTypedDetail(tiledata.detail);
                const links = Tool.toTypedLinks(tiledata.links);
                this.tileManager.tiles.set(key, {
                    tileId: key,
                    tx: tiledata.tx,
                    ty: tiledata.ty,
                    mesh: mesh,
                    detail: detail,
                    links: links
                });
                this.tileManager._appendTileData(key, mesh, detail, links);
                this.tileManager._rebuildDeferredLinks(true,false,key);
            }
            this.tileManager._rebuildDeferredLinks(false,true);
            if (TILE_OPTIMIZATION_1)this.tileManager.pruneUnreachablePolys();
            this.tileManager.updatemesh();
            Instance.Msg(`导航数据加载成功！多边形数量: ${this.mesh.polyslength-1}`);
            return true;
        } catch (e) {
            Instance.Msg(`加载导航数据失败: ${e}`);
            return false;
        }
    }
    //初始化网格
    init() {
        this.tileManager = new TileManager(this);
        if(LOAD_STATIC_MESH) {
            this.importNavData(new StaticData().Data);
        }
        else {
            this.tileManager.rebuildAll(this.tile);
            this.tileManager.updatemesh();
        }
        if(PRINT_NAV_MESH)this.exportNavData();
        this._refreshRuntime();
        if(PLUGIN_ENABLED)this.plugin?.init(this.tileManager,this.tile);
    }
    /**
     * 仅更新 pos 所在 tile 的导航网格。不存在就创建
     * @param {Vector} pos
     */
    update(pos)
    {
        this.tileManager.rebuildAtPos(this.tile, pos);
        this.tileManager.updatemesh();
        this._refreshRuntime();
    }
    _refreshRuntime() {
        Tool.buildSpatialIndex(this.mesh);
        
//        /** @type {Map<number, number>} */
//        const degree = new Map();
//        const globalLinks = this.links;
//        const globalLen = globalLinks?.length ?? 0;
//
//        // 1) 先统计每个 poly 需要多少条 link（双向展开）
//        for (let i = 0; i < globalLen; i++) {
//            const a = globalLinks.poly[i * 2];
//            const b = globalLinks.poly[i * 2 + 1];
//            if (a < 0 || b < 0) continue;
//
//            degree.set(a, (degree.get(a) ?? 0) + 1);
//            degree.set(b, (degree.get(b) ?? 0) + 1);
//        }
//
//        /**@type {NavMeshLink[]} */
//        const links=new Array();
//        // 2) 按统计容量分配，避免固定 32 溢出
//        for (const [poly, cnt] of degree.entries()) {
//            links[poly] = {
//                poly: new Uint16Array(cnt * 2),
//                cost: new Float32Array(cnt),
//                type: new Uint8Array(cnt),
//                pos: new Float32Array(cnt * 6),
//                length: 0
//            };
//        }
//
//        // 3) 写入双向 link（reverse 方向要交换 start/end）
//        for (let i = 0; i < globalLen; i++) {
//            const polyA = globalLinks.poly[i * 2];
//            const polyB = globalLinks.poly[i * 2 + 1];
//            if (polyA < 0 || polyB < 0) continue;
//
//            const cost = globalLinks.cost[i] * OFF_MESH_LINK_COST_SCALE;
//            const type = globalLinks.type[i];
//            const srcPosBase = i * 6;
//
//            const la = links[polyA];
//            const lb = links[polyB];
//            if (!la || !lb) continue;
//
//            // A -> B
//            let wa = la.length;
//            la.poly[wa * 2] = polyA;
//            la.poly[wa * 2 + 1] = polyB;
//            la.cost[wa] = cost;
//            la.type[wa] = type;
//            la.pos[wa * 6] = globalLinks.pos[srcPosBase];
//            la.pos[wa * 6 + 1] = globalLinks.pos[srcPosBase + 1];
//            la.pos[wa * 6 + 2] = globalLinks.pos[srcPosBase + 2];
//            la.pos[wa * 6 + 3] = globalLinks.pos[srcPosBase + 3];
//            la.pos[wa * 6 + 4] = globalLinks.pos[srcPosBase + 4];
//            la.pos[wa * 6 + 5] = globalLinks.pos[srcPosBase + 5];
//            la.length = wa + 1;
//
//            // B -> A（交换端点）
//            let wb = lb.length;
//            lb.poly[wb * 2] = polyB;
//            lb.poly[wb * 2 + 1] = polyA;
//            lb.cost[wb] = cost;
//            lb.type[wb] = type;
//            lb.pos[wb * 6] = globalLinks.pos[srcPosBase + 3];
//            lb.pos[wb * 6 + 1] = globalLinks.pos[srcPosBase + 4];
//            lb.pos[wb * 6 + 2] = globalLinks.pos[srcPosBase + 5];
//            lb.pos[wb * 6 + 3] = globalLinks.pos[srcPosBase];
//            lb.pos[wb * 6 + 4] = globalLinks.pos[srcPosBase + 1];
//            lb.pos[wb * 6 + 5] = globalLinks.pos[srcPosBase + 2];
//            lb.length = wb + 1;
//        }
        /**@type {Map<number,NavMeshLinkARRAY[]>} */
        const links = new Map();
        for (let i = 0; i < this.links.length; i++) {
            const polyA = this.links.poly[i * 2];
            const polyB = this.links.poly[i * 2 + 1];
            if (polyA < 0 || polyB < 0) continue;
            const cost = this.links.cost[i] * OFF_MESH_LINK_COST_SCALE;
            const type = this.links.type[i];
            const srcPosBase = i * 6;
            if (!links.has(polyA)) links.set(polyA, []);
            if (!links.has(polyB)) links.set(polyB, []);
            const link={
                PolyA: polyA,
                PolyB: polyB,
                PosA: {
                    x: this.links.pos[srcPosBase],
                    y: this.links.pos[srcPosBase + 1],
                    z: this.links.pos[srcPosBase + 2]
                },
                PosB: {
                    x: this.links.pos[srcPosBase + 3],
                    y: this.links.pos[srcPosBase + 4],
                    z: this.links.pos[srcPosBase + 5]
                },
                cost: cost,
                type: type
            }
            links.get(polyA)?.push(link);
            links.get(polyB)?.push(link);
        }
        this.heightfixer = new FunnelHeightFixer(this.mesh, this.meshdetail, ADJUST_HEIGHT_DISTANCE);
        this.astar = new PolyGraphAStar(this.mesh, links, this.heightfixer);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers, links);
    }
    /**
     * @param {Vector} [pos] //玩家所在位置
     */
    tick(pos)
    {
        if(PLUGIN_ENABLED)this.plugin?.tick();
        if(TILE_DEBUG&&pos)
        {
            Instance.DebugScreenText({
                text:`当前所在tileKey:${this.tile.fromPosGetTile(pos)}`,
                x:200,
                y:200,
                duration:1
            })
        }
    }
    /**
     * 只调试“可持久保存”的数据（mesh/detail/links）。
     * @param {number} duration
     */
    debug(duration = 60) {
        if (POLY_DETAIL_DEBUG) {
            this.debugTools.debugDrawMeshDetail(duration);
        }
        if(TILE_DEBUG)
        {
            this.debugTools.debugDrawALLTiles(duration);
        }
        if (LOAD_DEBUG) {
            try{
                Instance.Msg("debug");
                Instance.Msg(`多边形总数: ${this.mesh.polyslength-1}  跳点总数: ${this.links.length-1}`);
                this.debugTools.debugDrawMeshPolys(duration);
                this.debugTools.debugDrawMeshConnectivity(duration);
                this.debugTools.debugLinks(duration);
            }
            catch(e)
            {
            }
            return;
        }
        if (POLY_DEBUG) {
            this.debugTools.debugDrawMeshPolys(duration);
            this.debugTools.debugDrawMeshConnectivity(duration);
        }
        if(LINK_DEBUG)
        {
            this.debugTools.debugLinks(duration);
            Instance.Msg(`跳点总数: ${this.links.length-1}`);
        }
    }
    /**
     * 输入起点终点，返回世界坐标路径点
     * @param {Vector} start
     * @param {Vector} end
     * @returns {{pos:Vector,mode:number}[]}
     */
    findPath(start, end) {
        //Instance.DebugLine({start,end,duration:1,color:{r:0,g:255,b:0}});
        const polyPath=this.astar.findPath(start,end);
        //this.debugTools.debugDrawPolyPath(polyPath.path,1);
        //if (!polyPath || polyPath.path.length === 0) return [];
        const funnelPath = this.funnel.build(polyPath.path, polyPath.start, polyPath.end);
        //this.debugTools.debugDrawfunnelPath(funnelPath,1);
        if(ADJUST_HEIGHT)
        {
            const ans=this.heightfixer.fixHeight(funnelPath,polyPath.path);
            //this.debugTools.debugDrawPath(ans,1);
            return ans;
        }
        else return funnelPath;
        //if (!ans || ans.length === 0) return [];
        //多边形总数：649跳点数：82
        //100次A*           30ms
        //100次funnelPath   46ms-30=16ms
        //100次200fixHeight    100ms-46=54ms
    }
}