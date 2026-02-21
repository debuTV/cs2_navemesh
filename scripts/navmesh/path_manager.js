import { PolyGraphAStar } from "./path_Astar";
import { FunnelPath } from "./path_funnel";
import { Instance } from "cs_script/point_script";
import { ADJUST_HEIGHT_DISTANCE, LINK_DEBUG, LOAD_DEBUG, LOAD_STATIC_MESH, POLY_DEBUG, POLY_DETAIL_DEBUG, PRINT_NAV_MESH, TILE_OPTIMIZATION_1 } from "./path_const";
import { StaticData } from "./path_navemeshstatic";
import { FunnelHeightFixer } from "./path_funnelheightfixer";
import { tile } from "./path_tile";
import { NavMeshDebugTools } from "./path_navmeshdebug";
import { TileManager } from "./path_tilemanager";
import { Tool } from "./util/tool";
/** @typedef {import("cs_script/point_script").Vector} Vector */
/**
 * @typedef {{
 *  verts:Vector[],
 *  polys:number[][],
 *  regions:number[],
 *  neighbors:number[][][]
 * }} NavMeshMesh
 */

/**
 * @typedef {{
 *  verts:Vector[],
 *  tris:number[][],
 *  triTopoly:number[],
 *  meshes:number[][]
 * }} NavMeshDetail
 */

/**
 * @typedef {{
 *  PolyA:number,
 *  PolyB:number,
 *  PosA:Vector,
 *  PosB:Vector,
 *  cost:number,
 *  type:number
 * }} NavMeshLink
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
        /**@type {NavMeshLink[]} */
        this.links;
        /** @type {TileManager} */
        this.tileManager = new TileManager();
        /** @type {tile} */
        this.tile = new tile();
        this.debugTools = new NavMeshDebugTools(this);
        //删除prop_door_rotating实体？也许应该弄一个目录，让作者把门一类的实体名字放里面
    }
    /**
     * 导出导航网格数据为文本字符串
     */
    exportNavData() {
        const charsPerLine = 500;
        if (!this.mesh) {
            Instance.Msg("错误：没有可导出的数据！请先生成网格。");
        }
        this.mesh.verts = this.mesh.verts.map(v => ({
            x: Math.round(v.x * 100) / 100,
            y: Math.round(v.y * 100) / 100,
            z: Math.round(v.z * 100) / 100
        }));
        this.meshdetail.verts=this.meshdetail.verts.map(v => ({
            x: Math.round(v.x * 100) / 100,
            y: Math.round(v.y * 100) / 100,
            z: Math.round(v.z * 100) / 100
        }));
        const data = {
            mesh: this.mesh,           // 包含 verts, polys, regions, neighbors
            meshdetail: this.meshdetail,
            links: this.links || [],
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
            this.mesh = data.mesh;
            this.links=data.links;
            this.meshdetail=data.meshdetail;
            Instance.Msg(`导航数据加载成功！多边形数量: ${this.mesh.polys.length}`);
            return true;
        } catch (e) {
            Instance.Msg(`加载导航数据失败: ${e}`);
            return false;
        }
    }
    init() {
        if(LOAD_STATIC_MESH) {
            this.importNavData(new StaticData().Data);
        }
        else {
            this.tileManager = new TileManager();
            this.tileManager.rebuildAll(this.tile);
            let merged = this.tileManager.return();
            this.mesh = merged.mesh;
            this.meshdetail = merged.meshdetail;
            this.links = merged.links;
        }
        if(PRINT_NAV_MESH)this.exportNavData();
        this._refreshRuntime();
    }

    /**
     * 仅更新 pos 所在 tile 的导航网格。
     * @param {Vector} pos
     */
    initpos(pos) {
        //加入scriptin？让其可以根据act实体位置进行tile刷新？
        this.tileManager.rebuildAtPos(this.tile, pos);
        const merged = this.tileManager.return();
        this.mesh = merged.mesh;
        this.meshdetail = merged.meshdetail;
        this.links = merged.links;
        this._refreshRuntime();
    }
    /**
     * @param {Vector} pos
     */
    update(pos)
    {
        this.tileManager.reversetile(this.tile, pos);
        const merged = this.tileManager.return();
        this.mesh = merged.mesh;
        this.meshdetail = merged.meshdetail;
        this.links = merged.links;
        this._refreshRuntime();
    }
    _refreshRuntime() {
        Tool.buildSpatialIndex(this.mesh);
        /**@type {Map<number,NavMeshLink[]>} */
        const links = new Map();
        for (const link of this.links) {
            const polyA = link.PolyA;
            const polyB = link.PolyB;
            if (!links.has(polyA)) links.set(polyA, []);
            if (!links.has(polyB)) links.set(polyB, []);
            links.get(polyA)?.push(link);
            links.get(polyB)?.push(link);
        }
        this.heightfixer = new FunnelHeightFixer(this.mesh, this.meshdetail, ADJUST_HEIGHT_DISTANCE);
        this.astar = new PolyGraphAStar(this.mesh, links, this.heightfixer);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers, links);
    }
    /**
     * 只调试“可持久保存”的数据（mesh/detail/links）。
     * @param {number} duration
     */
    debug(duration = 60) {
        if (POLY_DETAIL_DEBUG) {
            this.debugTools.debugDrawMeshDetail(duration);
        }
        if (LOAD_DEBUG) {
            Instance.Msg(`多边形总数: ${this.mesh.polys.length}`);
            this.debugTools.debugDrawMeshPolys(duration);
            this.debugTools.debugDrawMeshConnectivity(duration);
            this.debugTools.debugLinks(duration);
            return;
        }
        if (POLY_DEBUG) {
            this.debugTools.debugDrawMeshPolys(duration);
            this.debugTools.debugDrawMeshConnectivity(duration);
        }
        if(LINK_DEBUG)
        {
            this.debugTools.debugLinks(duration);
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
        //this.debugTools.debugDrawPolyPath(polyPath.path,1/2);
        //if (!polyPath || polyPath.path.length === 0) return [];
        const funnelPath = this.funnel.build(polyPath.path, polyPath.start, polyPath.end);
        //this.debugTools.debugDrawfunnelPath(funnelPath,1/2);
        const ans=this.heightfixer.fixHeight(funnelPath,polyPath.path);
        //this.debugTools.debugDrawPath(ans,1/2);
        //if (!ans || ans.length === 0) return [];
        //多边形总数：649跳点数：82
        //100次A*           42ms
        //100次funnelPath   55ms-42=13ms
        //100次50fixHeight    106ms-55=51ms
        //100次200fixHeight    70ms-55=15ms
        return ans;
    }
}