import { Instance } from "cs_script/point_script";
import { TileManager } from "../path_tilemanager";
import { tile } from "../path_tile";
import { NVpluginStaticData } from "./plugin_static";
import { StaticData } from "../path_navemeshstatic";
import { TILE_OPTIMIZATION_1 } from "../path_const";
/** @typedef {import("../path_tilemanager").TileData} TileData */
export class NVplugin {
    constructor() {
        //aaabbccc_1_2,tiledata
        //aaabbccc_2_2,tiledata
        /** @type {Map<string, TileData>} */
        this.tiles = new Map();
        /** @type {Map<string, TileData>} */
        this.deftiles=new Map();
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
            tiles: Array.from(this.tiles)
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
            
            /**@type {{tiles: [string, TileData][]}} */
            const data = JSON.parse(cleanJson);
            for (const [key,value] of data.tiles??[]) {
                this.tiles.set(key, value);
            }

            const dfcleanJson = defaultJsonStr.replace(/\s/g, "");
            /**@type {{tiles: TileData[]}} */
            const dfdata = JSON.parse(dfcleanJson);

            for (const value of dfdata.tiles??[]) {
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
    /**
     * @param {string} name
     * @param {string[]} key
     * @param {boolean} [pre] //是否恢复默认设置
     */
    setTile(name,key,pre=false)
    {
        let start = new Date();
        if(!this.tileManager)return;
        if(pre)
        {   //2-3->2_3
            for(let k of key)
            {
                k=k.replace("-","_");
                const td=this.deftiles.get(k);
                if(!td)continue;
                //这里替换数据
                this.tileManager.updatetile(k,td.tx,td.ty,td.mesh,td.detail,td.links);
                if(TILE_OPTIMIZATION_1)this.tileManager.pruneUnreachablePolys();
                this.tileManager.updatemesh();
            }
        }
        else
        {
            for(let k of key)
            {
                k=k.replace("-","_");
                let td=this.tiles.get(name+"_"+k);
                if(!td)
                {
                    this.addTile(name,[k]);
                    td=this.tiles.get(name+"_"+k);
                }
                if(!td)continue;
                //这里替换数据
                this.tileManager.updatetile(k,td.tx,td.ty,td.mesh,td.detail,td.links);
                if(TILE_OPTIMIZATION_1)this.tileManager.pruneUnreachablePolys();
                this.tileManager.updatemesh();
            }
        }
        let end = new Date();
        Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
    }

}
