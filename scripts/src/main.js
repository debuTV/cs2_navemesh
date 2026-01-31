import {CSPlayerPawn, Instance,CSPlayerController, Entity} from "cs_script/point_script";
import { NavMesh } from "../navmesh/path_manager";
Instance.ServerCommand("mp_warmup_offline_enabled 1");
Instance.ServerCommand("mp_warmup_pausetimer 1");
Instance.ServerCommand("mp_roundtime 60");
Instance.ServerCommand("mp_freezetime 1");
Instance.ServerCommand("mp_ignore_round_win_conditions 1");
Instance.ServerCommand("weapon_accuracy_nospread 1");
let pathfinder = new NavMesh();
Instance.OnScriptReload({
    before: () => {
    },
    after: () => {
        //let start = new Date();
        //Instance.Msg("导航初始化中");
        //pathfinder.init();
        //let end = new Date();
        //Instance.Msg(`导航初始化完成,耗时${end.getTime()-start.getTime()}ms`);
    }
});
let path_ini=false;
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
let start={x:-896,y:-783,z:117};
let end={x:351,y:2352,z:-110};
let pd=false;
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
        //            end={x:pos.x,y:pos.y,z:pos.z};
        //            return;
        //        }
        //    }
        //})
        for(let i=0;i<1;i++)pathfinder.findPath(start,end);
    }
    Instance.SetNextThink(Instance.GetGameTime()+1/2);
});
Instance.SetNextThink(Instance.GetGameTime()+1/2);
Instance.OnBulletImpact((event)=>{
    end=event.position;
    //pathfinder.findPath(start,end);
    //pathfinder.findPath(start,end);
});
Instance.OnPlayerChat((event) => {
    const text = (event.text || "").trim().toLowerCase().split(' ')[0];
    const num=Number((event.text || "").trim().toUpperCase().split(' ')[1]);
    if (text === "debug" || text === "!debug")
    {
        init();
        pathfinder.debug(60);
        //pathfinder.testinit();
        pd=true;
        //pathfinder.testinit();
        //pathfinder.randomTest(num);
    }
    if (text === "c" || text === "!c")
    {
        const p=event.player?.GetPlayerPawn();
        if(p)
        {
            const pos=p.GetAbsOrigin();
            start={x:pos.x,y:pos.y,z:pos.z};
            //Instance.Msg(`${Math.floor(pos.x)}  ${Math.floor(pos.y)}  ${Math.floor(pos.z)}`);
        }
    }
    if (text === "v" || text === "!v")
    {
        const p=event.player?.GetPlayerPawn();
        if(p)
        {
            const pos=p.GetAbsOrigin();
            end={x:pos.x,y:pos.y,z:pos.z};
            //const path = pathfinder.findPath(start,end);
            //Instance.Msg(`${Math.floor(end.x)}  ${Math.floor(end.y)}  ${Math.floor(end.z)}`);
            //pathfinder.debugDrawPath(path,30);
        }
    }
});