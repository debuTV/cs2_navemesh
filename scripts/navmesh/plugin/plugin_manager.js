import { Instance } from "cs_script/point_script";
import { TileManager } from "../path_tilemanager";
import { tile } from "../path_tile";
import { NVpluginStaticData } from "./plugin_static";
import { StaticData } from "../path_navemeshstatic";
import { TILE_OPTIMIZATION_1 } from "../path_const";
import { NavMesh } from "../path_manager";
import { Tool } from "../util/tool";
import { JumpLinkBuilder } from "../path_jumplinkbuild";
/** @typedef {import("../path_tilemanager").TileData} TileData */
export class NVplugin {
    /**
     * @param {NavMesh} nav
     */
    constructor(nav) {
        this.nav=nav;
        //aaabbccc_1_2,tiledata
        //aaabbccc_2_2,tiledata
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();
        /** @type {Map<string, TileData>} */
        this.deftiles=new Map();
        this.up=0;//下一tick更新什么
        /**
         * @type {{name:string, td: TileData; }[]}
         */
        this.updata=[];
        /** @type {TileManager} */
        this.tileManager;
        this.importNavData(new NVpluginStaticData().Data,new StaticData().Data);
    }
    /**
     * @param {TileManager} tilemanager
     * @param {tile} tile
     */
    init(tilemanager,tile)
    {
        /** @type {TileManager} */
        this.tileManager = tilemanager;

        /** @type {tile} */
        this.tile=tile;
        
        Instance.OnScriptInput("addtile",(e)=>{
            Instance.Msg("addtile");
            if(!e.caller)return;
            let name=e.caller?.GetEntityName();
            if (!name.startsWith("navmesh_")) return;
            const sp=name.split("_");
            if(sp.length<3)return;
            this.addTile(sp[1],sp.slice(2));
        });
        Instance.OnScriptInput("settile",(e)=>{
            if(!e.caller)return;
            let name=e.caller?.GetEntityName();
            if (!name.startsWith("navmesh_")) return;
            const sp=name.split("_");
            if(sp.length<3)return;
            if(sp[1]=="default")this.setTile(sp[1],sp.slice(2),true);
            else this.setTile(sp[1],sp.slice(2));
        });
    }
    /**
     * 导出导航网格数据为文本字符串
     */
    exportNavData() {
        const charsPerLine = 500;
        const data = {
            tiles: Array.from(this.tiles, ([key, td]) => [key, Tool._compactTileData(td)])
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
     * @param {string} defaultJsonStr
     */
    importNavData(jsonStr, defaultJsonStr) {
        try {
            const cleanJson = jsonStr.replace(/\s/g, "");
            if(cleanJson.length==0)throw new Error("空数据");
            const data = JSON.parse(cleanJson);
            for (const [key,value] of data.tiles) {
                const mesh = Tool.toTypedMesh(value.mesh);
                const detail = Tool.toTypedDetail(value.detail);
                const links = Tool.toTypedLinks(value.links);
                value.mesh=mesh;
                value.detail=detail;
                value.links=links;
                this.tiles.set(key, value);
            }

            const dfcleanJson = defaultJsonStr.replace(/\s/g, "");
            if(dfcleanJson.length==0)throw new Error("空数据");

            const dfdata = JSON.parse(dfcleanJson);

            for (const [key,value] of dfdata.tiles) {
                const mesh = Tool.toTypedMesh(value.mesh);
                const detail = Tool.toTypedDetail(value.detail);
                const links = Tool.toTypedLinks(value.links);
                value.mesh=mesh;
                value.detail=detail;
                value.links=links;
                this.deftiles.set(value.tileId, value);
            }
            Instance.Msg(`加载备选数据成功！`);
            return true;
        } catch (e) {
            Instance.Msg(`加载备选数据失败: ${e}`);
            return false;
        }
    }
    /**
     * @param {string} name
     * @param {string[]} key
     */
    addTile(name,key)
    {
        if(!this.tile)return;
        for(let k of key)
        {
            k=k.replace("-","_");
            const tx=parseInt(k.split("_")[0]);
            const ty=parseInt(k.split("_")[1]);
            //这里获取数据
            this.tiles.set(name+"_"+k,this.tile.buildTile(tx, ty));
        }
        this.exportNavData();
    }
    tick()
    {
        let start=new Date();
        switch(this.up)
        {
            case 0:
                if(this.updata.length!=0)this.up=1;
                break;
            case 1:
                //5ms;
                this.tileManager.removetile(this.updata[0].name);
                const td=this.updata[0].td;
                this.tileManager.tiles.set(td.tileId, {
                    tileId: td.tileId,
                    tx: td.tx,
                    ty: td.ty,
                    mesh: td.mesh,
                    detail: td.detail,
                    links: td.links
                });
                this.tileManager._appendTileData(td.tileId, td.mesh, td.detail, td.links);
                this.up++;
                break;
            case 2:
                //1ms
                const { edgeCount, result, tilemark } = this.tileManager.getedgebytileid(this.updata[0].name);
                this.edgeCount=edgeCount;
                this.result=result;
                this.tilemark=tilemark;
                this.up++;
                break;
            case 3:
                //7ms
                if(this.edgeCount&&this.result&&this.tilemark)this.tileManager.Extlink = new JumpLinkBuilder(this.tileManager.mesh).initInterTileIn(this.edgeCount,this.result,this.tilemark,this.tileManager.Extlink);
                this.up++;
                break;
            case 4:
                //3ms
                Tool.buildSpatialIndex(this.tileManager.mesh);
                this.up++;
                break;
            case 5:
                //2ms
                this.tileManager.supprlink= this.tileManager.buildSupperLinksForMesh(this.tileManager.mesh);
                let merged = this.tileManager.copyLinks(this.tileManager.baseLinks, this.tileManager.Extlink);
                merged = this.tileManager.copyLinks(merged, this.tileManager.supprlink);
                this.tileManager.links = merged;
                this.up++;
                break;
            case 6:
                //7ms
                if(TILE_OPTIMIZATION_1)this.tileManager.pruneUnreachablePolys();
                this.tileManager.updatemesh();
                this.nav._refreshRuntime();
                this.updata.shift();
                this.up++;
                break;
            default:
                this.up=0;
                break;
        }
        let end=new Date();
        if(this.up>0)Instance.Msg(`${this.up} ${end.getTime() - start.getTime()} ms`);
    }
    /**
     * @param {string} name
     * @param {string[]} key
     * @param {boolean} [pre] //是否恢复默认设置
     */
    setTile(name,key,pre=false)
    {
        if(!this.tileManager)return;
        if(pre)
        {   //2-3->2_3
            for(let k of key)
            {
                k=k.replace("-","_");
                const td=this.deftiles.get(k);
                if(!td)continue;
                //这里替换数据
                this.updata.push({name:k,td:td});
            }
        }
        else
        {
            for(let k of key)
            {
                k=k.replace("-","_");
                let td=this.tiles.get(name+"_"+k);
                if(!td)
                {//没有数据，必定是开发环境
                    this.addTile(name,[k]);
                    td=this.tiles.get(name+"_"+k);
                }
                if(!td)continue;
                //这里替换数据
                this.updata.push({name:k,td:td});
            }
        }
    }
}
