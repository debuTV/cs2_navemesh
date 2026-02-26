import {CSPlayerPawn, Instance} from "cs_script/point_script";
import { NavMesh } from "../navmesh/path_manager";
/** @typedef {import("cs_script/point_script").Vector} Vector */
Instance.ServerCommand("bot_kick");
Instance.ServerCommand("mp_warmup_offline_enabled 1");
Instance.ServerCommand("mp_warmup_pausetimer 1");
Instance.ServerCommand("mp_roundtime 60");
Instance.ServerCommand("mp_freezetime 1");
Instance.ServerCommand("mp_ignore_round_win_conditions 1");
Instance.ServerCommand("weapon_accuracy_nospread 1");
Instance.ServerCommand("mat_fullbright 1");
//for(let l=0;l<30;l++)
//{
//    Instance.ServerCommand("bot_add");
//}
let pathfinder = new NavMesh();
//Instance.OnScriptReload({
//    before: () => {
//    },
//    after: () => {
//        //let start = new Date();
//        //Instance.Msg("导航初始化中");
//        //pathfinder.init();
//        //let end = new Date();
//        //Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
//    }
//});
let path_ini=false;
/**
 * @param {Vector} pos
 */
function initpos(pos)
{
    //if(path_ini)return;
    let start = new Date();
    Instance.Msg("导航初始化中");
    pathfinder.update(pos);
    let end = new Date();
    Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
    //path_ini=true;
}

function init()
{
    if(path_ini)return;
    let start = new Date();
    Instance.Msg("导航初始化中");
    pathfinder.init();
    let end = new Date();
    Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
    path_ini=true;
}
let start={x:3457,y:-984,z:-352};
let end={x:-2960,y:-625,z:-416};
let pd=false;
let sss=true;
let ttt=0;
Instance.SetThink(() => {
    if(pd==true)
    {
        //pathfinder.randomTest(10);
        //var players=Instance.FindEntitiesByClass("player");
        //players.forEach((e)=>{
        //    if(e&&e instanceof CSPlayerPawn)
        //    {
        //        var p=e.GetPlayerController()?.GetPlayerPawn();
        //        if(p)
        //        {
        //            const pos=p.GetAbsOrigin();
        //            pathfinder.tick(pos);
        //            //end={x:pos.x,y:pos.y,z:pos.z};
        //            return;
        //        }
        //    }
        //})
        pathfinder.tick();
        ttt++;
        if(ttt%32==0)pathfinder.debug(1);
        for(let i=0;i<1;i++)pathfinder.findPath(start,end);
    }
    Instance.SetNextThink(Instance.GetGameTime()+1/32);
});
Instance.SetNextThink(Instance.GetGameTime()+1/32);
Instance.OnBulletImpact((event)=>{
    //let start = new Date();
    ////pathfinder.update(event.position);
    //let end = new Date();
    //Instance.Msg(`导航更新完成,耗时${end.getTime()-start.getTime()}ms`);
    //pathfinder.debug(30);
    if(sss)end=event.position;
    else start=event.position;
    //sss=!sss;
    //pathfinder.findPath(start,end);
    //pathfinder.findPath(start,end);
});
//init();
Instance.OnPlayerChat((event) => {
    const text = (event.text || "").trim().toLowerCase().split(' ')[0];
    if (text === "debug" || text === "!debug")
    {
        //每秒200000次tracebox,每tick 3125次，每个怪物平均10次，总共312只怪物
        //10v10 1000次tracebox
        init();
        //多边形总数: 651  跳点总数: 116_>91???
        //多边形总数: 651  跳点总数: 91
        //多边形总数: 635  跳点总数: 78
        Instance.Msg("开始调试");
        //for(let i=0;i<1;i++)pathfinder.findPath(start,end);
        //pathfinder.debug(60);
        //pathfinder.debugTools.debug(60);
        //pathfinder.debugTools.testinit();
        pd=true;
        //pathfinder.debugTools.testinit();
        //pathfinder.debugTools.randomTest(num);
    }
    //if (text === "c" || text === "!c")
    //{
    //    const p=event.player?.GetPlayerPawn();
    //    if(p)
    //    {
    //        const pos=p.GetAbsOrigin();
    //        start={x:pos.x,y:pos.y,z:pos.z};
    //        //Instance.Msg(`${Math.floor(pos.x)}  ${Math.floor(pos.y)}  ${Math.floor(pos.z)}`);
    //    }
    //}
    //if (text === "v" || text === "!v")
    //{
    //    const p=event.player?.GetPlayerPawn();
    //    if(p)
    //    {
    //        const pos=p.GetAbsOrigin();
    //        end={x:pos.x,y:pos.y,z:pos.z};
    //        //const path = pathfinder.findPath(start,end);
    //        //Instance.Msg(`${Math.floor(end.x)}  ${Math.floor(end.y)}  ${Math.floor(end.z)}`);
    //        //pathfinder.debugDrawPath(path,30);
    //    }
    //}
});