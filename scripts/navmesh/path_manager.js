import { PolyGraphAStar } from "./path_Astar";
import { FunnelPath } from "./path_funnel";
import { RegionGenerator } from "./path_regiongenerator";
import { ContourBuilder } from "./path_contourbuilder";
import { PolyMeshBuilder } from "./path_polymeshbuilder";
import { Instance } from "cs_script/point_script";
import { PolyMeshDetailBuilder } from "./path_polydetail";
import { CONTOUR_DEBUG, JUMP_LINK_DEBUG, LOAD_DEBUG, LOAD_STATIC_MESH, MESH_DEBUG, MESH_OPTIMIZATION_1, MESH_OPTIMIZATION_2, MESH_OPTIMIZATION_3, POLY_DEBUG, PRINT_NAV_MESH, REGION_DEBUG } from "./path_const";
import { OpenHeightfield } from "./path_openheightfield";
import { JumpLinkBuilder } from "./path_jumplinkbuild";
import { StaticData } from "./path_navemeshstatic";

export class NavMesh {
    constructor() {
        /**@type {OpenHeightfield} */
        this.hf;
        /**@type {RegionGenerator} */
        this.regionGen;
        /**@type {ContourBuilder} */
        this.contourBuilder;
        /**@type {PolyMeshBuilder} */
        this.polyMeshGenerator;
        /**@type {PolyMeshDetailBuilder} */
        this.polidetail;
        /**@type {JumpLinkBuilder} */
        this.jumplinkbuilder;
        /**@type {PolyGraphAStar} */
        this.astar;
        /**@type {{verts: {x: number;y: number;z: number;}[];polys: number[][];regions: number[];neighbors: number[][];}} */
        this.mesh;
        /**@type {{verts: {x: number;y: number;z: number;}[];tris:number[][];meshes: number[][];}} */
        this.meshdetail;
        /**@type {FunnelPath} */
        this.funnel;
        /**@type {{ PolyA: number; PolyB: number; PosA: { x: number; y: number; z: number; }; PosB: { x: number; y: number; z: number; }; cost: number;type: number; }[]} */
        this.links;
        /** @type {number[]}*/
        this._polyAreas = [];
        /**@type {number[]}*/
        this._polyPrefix = [];
        /**@type {number}*/
        this._totalPolyArea;
    }
    /**
     * @param {number} [duration]
     */
    debug(duration = 5) {
        if(MESH_DEBUG)this.hf.debug(30);
        if(REGION_DEBUG)this.regionGen.debugDrawRegions(30);
        if(CONTOUR_DEBUG)this.contourBuilder.debugDrawContours(30);
        if(POLY_DEBUG)
        {
            this.polyMeshGenerator.debugDrawPolys(30);
            this.polyMeshGenerator.debugDrawAdjacency(30);
        }
        if(JUMP_LINK_DEBUG)this.jumplinkbuilder.debugDraw(30);
        if(LOAD_DEBUG)this.debugLoad(30);
    }
    debugLoad(duration = 5) {
        for (let pi = 0; pi < this.mesh.polys.length; pi++) {
            const poly = this.mesh.polys[pi];
            const color = {r:255,g:255,b:0};
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({ start, end, color, duration });
            }
        }
        for (const link of this.links) {
            Instance.DebugLine({
                start: link.PosA,
                end: link.PosB,
                color: { r: 0, g: (link.type==1?255:0), b: 255 },
                duration
            });
            Instance.DebugSphere({ center: link.PosA, radius: 4, color: { r: 0, g: 255, b: 0 }, duration });
            //Instance.DebugSphere({ center: link.endPos, radius: 4, color: { r: 255, g: 0, b: 0 }, duration });
            const poly = this.mesh.polys[link.PolyB];
            for (let i = 0; i < poly.length; i++) {
                const start = this.mesh.verts[poly[i]];
                const end = this.mesh.verts[poly[(i + 1) % poly.length]];
                Instance.DebugLine({start,end,color:{ r: 255, g: 0, b: 255 },duration});
                //Instance.DebugSphere({center:start,radius:6,color,duration});
            }
        }
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
        const data = {
            mesh: this.mesh,           // 包含 verts, polys, regions, neighbors
            links: this.jumplinkbuilder ? this.jumplinkbuilder.links : [],
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
            Instance.Msg(`导航数据加载成功！多边形数量: ${this.mesh.polys.length}`);
            return true;
        } catch (e) {
            Instance.Msg(`加载导航数据失败: ${e}`);
            return false;
        }
    }
    init() {
        if(LOAD_STATIC_MESH)this.importNavData(new StaticData().Data);
        else
        {
            let start = new Date();
            //创建世界网格
            this.hf = new OpenHeightfield();
            this.hf.init();
            if(MESH_OPTIMIZATION_1)this.hf.findcanwalk();
            if(MESH_OPTIMIZATION_2)this.hf.deleteboundary();
            if(MESH_OPTIMIZATION_3)this.hf.deletealone();
            let end = new Date();
            Instance.Msg(`网格生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;
            //分割区域
            start = new Date();
            this.regionGen = new RegionGenerator(this.hf);
            this.regionGen.init();
            end = new Date();
            Instance.Msg(`区域生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;
            //轮廓生成
            start = new Date();
            this.contourBuilder = new ContourBuilder(this.hf);
            this.contourBuilder.init();
            end = new Date();
            Instance.Msg(`轮廓生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;
            //生成多边形网格
            start = new Date();
            this.polyMeshGenerator = new PolyMeshBuilder(this.contourBuilder.contours);
            this.polyMeshGenerator.init();
            this.mesh = this.polyMeshGenerator.return();
            end = new Date();
            Instance.Msg(`多边形生成完成,耗时${end.getTime() - start.getTime()}ms`);
            //return;

            //构建细节网格
            //start = new Date();
            ////this.polidetail = new PolyMeshDetailBuilder(this.mesh, this.hf);
            ////this.meshdetail = this.polidetail.init();
            //end = new Date();
            //Instance.Msg(`细节多边形生成完成,耗时${end.getTime() - start.getTime()}ms`);

            //return;
            start = new Date();
            this.jumplinkbuilder=new JumpLinkBuilder(this.mesh);
            this.links=this.jumplinkbuilder.init();
            end = new Date();
            Instance.Msg(`跳点生成完成,耗时${end.getTime() - start.getTime()}ms`);
        }

        //构建A*寻路
        this.astar = new PolyGraphAStar(this.mesh,this.links);
        this.funnel = new FunnelPath(this.mesh, this.astar.centers,this.links);
        if(PRINT_NAV_MESH)this.exportNavData();
    }
    /**
     * 输入起点终点，返回世界坐标路径点
     * @param {{x:number,y:number,z:number}} start
     * @param {{x:number,y:number,z:number}} end
     * @returns {{pos:{x:number,y:number,z:number},mode:number}[]}
     */
    findPath(start, end) {
        const polyPath=this.astar.findPath(start,end);

        if (!polyPath || polyPath.length === 0) return [];

        const ans = this.funnel.build(polyPath, start, end);
        // 用 detail mesh 修正高度
        //for (const p of ans) {
        //    const poly = this.findContainingPoly(p);
        //    if (poly >= 0) {
        //        p.z = this.getHeightOnPolyDetail(poly, p.x, p.y);
        //    }
        //}
        this.debugDrawPolyPath(polyPath, 1 / 2);
        this.debugDrawPath(ans,1/2);
        return ans;
    }
    /**
     * @param {number} polyIndex
     * @param {number} x
     * @param {number} y
     */
    getHeightOnPolyDetail(polyIndex, x, y) {
        const [vbase, vcount, tbase, tcount] = this.meshdetail.meshes[polyIndex];

        for (let i = 0; i < tcount; i++) {
            const tri = this.meshdetail.tris[tbase + i];

            const a = this.meshdetail.verts[tri[0]];
            const b = this.meshdetail.verts[tri[1]];
            const c = this.meshdetail.verts[tri[2]];

            if (this.pointInTri2D(x, y, a, b, c)) {
                return this.baryZ(x, y, a, b, c);
            }
        }

        const center = this.polyCenter(polyIndex);
        return center.z;
    }
    /**
     * @param {number} pi
     */
    polyCenter(pi) {
        const poly = this.mesh.polys[pi];
        let x = 0, y = 0, z = 0;

        for (const vi of poly) {
            const v = this.mesh.verts[vi];
            x += v.x;
            y += v.y;
            z += v.z;
        }

        const n = poly.length;
        return { x: x / n, y: y / n, z: z / n };
    }
    /**
     * @param {number} px
     * @param {number} py
     * @param {{ x: number; y: number; z: number; }} a
     * @param {{ x: number; y: number; z: number; }} b
     * @param {{ x: number; y: number; z: number; }} c
     */
    pointInTri2D(px, py, a, b, c) {
        const area = (/** @type {{ x: number; y: number; z: number; }} */ p, /** @type {{ x: number; y: number; z: number; }} */ q, /** @type {{ x: number; y: number; }} */ r) =>
            (q.x - p.x) * (r.y - p.y) -
            (q.y - p.y) * (r.x - p.x);

        const p = { x: px, y: py };
        const ab = area(a, b, p);
        const bc = area(b, c, p);
        const ca = area(c, a, p);

        return ab >= 0 && bc >= 0 && ca >= 0;
    }
    /**
     * @param {number} px
     * @param {number} py
     * @param {{ x: number; y: number; z: number; }} a
     * @param {{ x: number; y: number; z: number; }} b
     * @param {{ x: number; y: number; z: number; }} c
     */
    baryZ(px, py, a, b, c) {
        const v0x = b.x - a.x, v0y = b.y - a.y;
        const v1x = c.x - a.x, v1y = c.y - a.y;
        const v2x = px - a.x, v2y = py - a.y;

        const d00 = v0x * v0x + v0y * v0y;
        const d01 = v0x * v1x + v0y * v1y;
        const d11 = v1x * v1x + v1y * v1y;
        const d20 = v2x * v0x + v2y * v0y;
        const d21 = v2x * v1x + v2y * v1y;

        const denom = d00 * d11 - d01 * d01;
        if (denom === 0) return a.z;

        const v = (d11 * d20 - d01 * d21) / denom;
        const w = (d00 * d21 - d01 * d20) / denom;
        const u = 1 - v - w;

        return u * a.z + v * b.z + w * c.z;
    }
    /**
     * @param {{pos:{x:number,y:number,z:number},mode:number}[]} path
     */
    debugDrawPath(path, duration = 10) {
        if (!path || path.length < 2) {
            Instance.Msg("No path to draw");
            return;
        }
        const color = {
            r: Math.floor(255),
            g: Math.floor(0),
            b: Math.floor(0),
        };
        const colorJ = {
            r: Math.floor(255),
            g: Math.floor(255),
            b: Math.floor(0),
        };

        const last = path[0].pos;
        Instance.DebugSphere({
            center: { x: last.x, y: last.y, z: last.z },
            radius: 3,
            color: { r: 255, g: 0, b: 0 },
            duration
        });
        for (let i = 1; i < path.length; i++) {
            const a = path[i-1].pos;
            const b = path[i].pos;

            Instance.DebugLine({
                start: { x: a.x, y: a.y, z: a.z },
                end: { x: b.x, y: b.y, z: b.z },
                color:path[i].mode==2?colorJ:color,
                duration
            });

            Instance.DebugSphere({
                center: { x: b.x, y: b.y, z: b.z },
                radius: 3,
                color:path[i].mode==2?colorJ:color,
                duration
            });
        }
    }
    /**
     * @param {{id:number,mode:number}[]} polyPath
     */
    debugDrawPolyPath(polyPath, duration = 10) {
        if (!polyPath || polyPath.length === 0) return;

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
            const poly = this.mesh.polys[pi.id];

            // poly 中心
            let cx = 0, cy = 0, cz = 0;
            for (const vi of poly) {
                const v = this.mesh.verts[vi];
                cx += v.x; cy += v.y; cz += v.z;
            }
            cx /= poly.length;
            cy /= poly.length;
            cz /= poly.length;

            const center = { x: cx, y: cy, z: cz };
            if(pi.mode==2)
            {
                Instance.DebugSphere({
                    center,
                    radius: 10,
                    color:colorJ,
                    duration
                });

                if (prev) {
                    Instance.DebugLine({
                        start: prev,
                        end: center,
                        color:colorJ,
                        duration
                    });
                }
            }
            else
            {
                Instance.DebugSphere({
                    center,
                    radius: 10,
                    color,
                    duration
                });

                if (prev) {
                    Instance.DebugLine({
                        start: prev,
                        end: center,
                        color,
                        duration
                    });
                }
                }
            prev = center;
        }
    }
    /**
     * @param {{left:{x:number,y:number,z:number},right:{x:number,y:number,z:number}}[]} portals
     */
    debugDrawPortals(portals, duration = 10) {
        if (!portals) return;

        for (let i = 0; i < portals.length; i++) {
            const { left, right } = portals[i];

            Instance.DebugLine({
                start: { x: left.x, y: left.y, z: left.z },
                end: { x: right.x, y: right.y, z: right.z },
                color: { r: 0, g: 180, b: 255 },
                duration
            });

            Instance.DebugSphere({
                center: { x: left.x, y: left.y, z: left.z },
                radius: 5,
                color: { r: 0, g: 255, b: 255 },
                duration
            });
            Instance.DebugSphere({
                center: { x: right.x, y: right.y, z: right.z },
                radius: 5,
                color: { r: 255, g: 0, b: 255 },
                duration
            });
        }
    }
    /**
     * @param {{left:{x:number,y:number,z:number},right:{x:number,y:number,z:number}}[]} portals
     * @param {import("cs_script/point_script").Vector} start
     * @param {import("cs_script/point_script").Vector} end
     */
    debugDrawFunnel(portals, start, end, duration = 10) {
        let prevL = start;
        let prevR = start;

        for (const p of portals) {
            const l = p.left;
            const r = p.right;

            Instance.DebugLine({
                start: { x: prevL.x, y: prevL.y, z: prevL.z },
                end: { x: l.x, y: l.y, z: l.z },
                color: { r: 0, g: 255, b: 0 },
                duration
            });

            Instance.DebugLine({
                start: { x: prevR.x, y: prevR.y, z: prevR.z },
                end: { x: r.x, y: r.y, z: r.z },
                color: { r: 255, g: 0, b: 0 },
                duration
            });

            prevL = l;
            prevR = r;
        }

        Instance.DebugLine({
            start: { x: prevL.x, y: prevL.y, z: prevL.z },
            end: { x: end.x, y: end.y, z: end.z },
            color: { r: 0, g: 255, b: 0 },
            duration
        });

        Instance.DebugLine({
            start: { x: prevR.x, y: prevR.y, z: prevR.z },
            end: { x: end.x, y: end.y, z: end.z },
            color: { r: 255, g: 0, b: 0 },
            duration
        });
    }

    testinit() {
        const polys = this.mesh.polys;
        const verts = this.mesh.verts;

        this._polyAreas = [];
        this._polyPrefix = [];

        let total = 0;

        for (let i = 0; i < polys.length; i++) {
            const poly = polys[i];
            let area = 0;

            for (let j = 0; j < poly.length; j++) {
                const a = verts[poly[j]];
                const b = verts[poly[(j + 1) % poly.length]];
                area += (a.x * b.y - b.x * a.y);
            }

            area = Math.abs(area) * 0.5;
            this._polyAreas.push(area);

            total += area;
            this._polyPrefix.push(total);
        }

        this._totalPolyArea = total;
    }
    _randomPoint() {
        // 1. 按面积随机选 poly
        const r = Math.random() * this._totalPolyArea;
        let lo = 0, hi = this._polyPrefix.length - 1;

        while (lo < hi) {
            const mid = (lo + hi) >> 1;
            if (r <= this._polyPrefix[mid]) hi = mid;
            else lo = mid + 1;
        }

        const pi = lo;
        const poly = this.mesh.polys[pi];
        const verts = this.mesh.verts;

        // 2. 扇形三角化采样
        const v0 = verts[poly[0]];
        let total = 0;
        const tris = [];

        for (let i = 1; i < poly.length - 1; i++) {
            const v1 = verts[poly[i]];
            const v2 = verts[poly[i + 1]];

            const area = Math.abs(
                (v1.x - v0.x) * (v2.y - v0.y) -
                (v1.y - v0.y) * (v2.x - v0.x)
            ) * 0.5;

            tris.push({ v0, v1, v2, area });
            total += area;
        }

        let t = Math.random() * total;
        for (const tri of tris) {
            if (t <= tri.area) {
                return this._randomPointInTri(tri.v0, tri.v1, tri.v2);
            }
            t -= tri.area;
        }

        return null;
    }
    /**
     * @param {{ x: number; y: number; z: number; }} a
     * @param {{ x: number; y: number; z: number; }} b
     * @param {{ x: number; y: number; z: number; }} c
     */
    _randomPointInTri(a, b, c) {
        let r1 = Math.random();
        let r2 = Math.random();

        if (r1 + r2 > 1) {
            r1 = 1 - r1;
            r2 = 1 - r2;
        }

        return {
            x: a.x + r1 * (b.x - a.x) + r2 * (c.x - a.x),
            y: a.y + r1 * (b.y - a.y) + r2 * (c.y - a.y),
            z: a.z + r1 * (b.z - a.z) + r2 * (c.z - a.z)
        };
    }
    /**
     * NavMesh 压力测试
     * @param {number} count  随机寻路次数
     */
    randomTest(count = 1000) {
        let success = 0;
        let fail = 0;
        for (let i = 0; i < count; i++) {
            const start = this._randomPoint();
            const end = this._randomPoint();

            if (!start || !end) {
                fail++;
                continue;
            }

            const path = this.findPath(start, end);
            //this.debugDrawPolyPath(path,30);
            if (path && path.length > 0) {
                success++;
            } else {
                fail++;
            }
        }
    }

}