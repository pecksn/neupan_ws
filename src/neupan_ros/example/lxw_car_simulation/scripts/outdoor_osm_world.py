#!/usr/bin/env python3
"""Create a local, flat Gazebo scene from an existing OSM file (no downloads).

Road centre lines and building outlines retain their local ENU projection.
Missing widths/heights are explicit assumptions in scene_manifest.json.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

DRIVABLE={'motorway','motorway_link','trunk','trunk_link','primary','primary_link',
           'secondary','secondary_link','tertiary','tertiary_link','unclassified',
           'residential','service','living_street','road'}


def number(value):
    if value is None:return None
    match=re.fullmatch(r'\s*([0-9]+(?:\.[0-9]+)?)\s*(?:m)?\s*',value)
    return float(match[1]) if match else None


def triangles(polygon):
    """Ear clipping for a simple outline. Reject self-intersection/degeneracy."""
    points=list(polygon)
    if points and points[0]==points[-1]:points.pop()
    if len(points)<3:raise ValueError('Building has fewer than three distinct vertices')
    area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))
    if abs(area)<1e-7:raise ValueError('Degenerate building outline')
    if area<0:points.reverse()
    def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def inside(p,a,b,c):return cross(a,b,p)>=-1e-8 and cross(b,c,p)>=-1e-8 and cross(c,a,p)>=-1e-8
    indices=list(range(len(points)));faces=[]
    while len(indices)>3:
        found=False
        for j,i in enumerate(indices):
            prev,nxt=indices[j-1],indices[(j+1)%len(indices)]
            a,b,c=points[prev],points[i],points[nxt]
            if cross(a,b,c)<=1e-8:continue
            if any(inside(points[k],a,b,c) for k in indices if k not in (prev,i,nxt)):continue
            faces.append((a,b,c));indices.pop(j);found=True;break
        if not found:raise ValueError('Building outline cannot be triangulated safely')
    faces.append(tuple(points[i] for i in indices))
    return points,faces


def building_mesh(path, outline, height):
    points,roof=triangles(outline)
    facets=[tuple((x,y,height) for x,y in f) for f in roof]
    facets += [tuple((x,y,0) for x,y in reversed(f)) for f in roof]
    for a,b in zip(points,points[1:]+points[:1]):
        facets.extend([((a[0],a[1],0),(b[0],b[1],0),(b[0],b[1],height)),
                       ((a[0],a[1],0),(b[0],b[1],height),(a[0],a[1],height))])
    with path.open('w') as stream:
        stream.write('solid building\n')
        for face in facets:
            a,b,c=face;u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
            normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
            norm=math.sqrt(sum(x*x for x in normal))
            normal=[x/norm for x in normal] if norm else [0,0,1]
            stream.write('facet normal {} {} {}\nouter loop\n'.format(*normal))
            for p in face:stream.write('vertex {} {} {}\n'.format(*p))
            stream.write('endloop\nendfacet\n')
        stream.write('endsolid building\n')


def generate(args):
    source=Path(args.osm).resolve();folder=Path(args.output_dir).resolve()
    if folder.exists() and any(folder.iterdir()):raise ValueError('Use an empty output directory')
    folder.mkdir(parents=True,exist_ok=True);meshes=folder/'meshes';meshes.mkdir()
    materials=folder/'materials/scripts';materials.mkdir(parents=True)
    (materials/'campus.material').write_text('material Campus/Road\n{\n technique\n {\n  pass\n  {\n   ambient 0.17 0.19 0.2\n   diffuse 0.17 0.19 0.2 1\n   depth_bias 4\n  }\n }\n}\n')
    root=ET.parse(source).getroot();scale=6378137*math.pi/180;coslat=math.cos(math.radians(args.lat))
    nodes={n.get('id'):((float(n.get('lon'))-args.lon)*scale*coslat,(float(n.get('lat'))-args.lat)*scale)
           for n in root.findall('node')}
    sdf=ET.Element('sdf',version='1.6');world=ET.SubElement(sdf,'world',name='gdut_osm_geometry')
    scene=ET.SubElement(world,'scene');ET.SubElement(scene,'ambient').text='.75 .75 .75 1'
    ET.SubElement(scene,'background').text='.65 .8 .94 1';ET.SubElement(scene,'shadows').text='false'
    light=ET.SubElement(world,'light',name='sun',type='directional')
    ET.SubElement(light,'direction').text='-.5 -.5 -1';ET.SubElement(light,'diffuse').text='.8 .8 .8 1'
    physics=ET.SubElement(world,'physics',type='ode')
    ET.SubElement(physics,'max_step_size').text='.001';ET.SubElement(physics,'real_time_update_rate').text='500'
    gui=ET.SubElement(world,'gui');cam=ET.SubElement(gui,'camera',name='campus_camera')
    ET.SubElement(cam,'pose').text='0 {} {} 0 .9 1.57'.format(-args.radius*1.2,args.radius*1.5)
    def item(name,pose,geometry,color,collision=False):
        model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,pose));link=ET.SubElement(model,'link',name='body')
        for tag in (('collision','visual') if collision else ('visual',)):
            v=ET.SubElement(link,tag,name=tag);g=ET.SubElement(v,'geometry')
            if geometry[0]=='box':ET.SubElement(ET.SubElement(g,'box'),'size').text=' '.join(map(str,geometry[1]))
            else:ET.SubElement(ET.SubElement(g,'mesh'),'uri').text=str(geometry[1])
            if tag=='visual':
                mat=ET.SubElement(v,'material');ET.SubElement(mat,'ambient').text=color;ET.SubElement(mat,'diffuse').text=color
                if name.startswith('road_'):
                    script=ET.SubElement(mat,'script');ET.SubElement(script,'uri').text=materials.as_uri()
                    ET.SubElement(script,'name').text='Campus/Road'
    item('ground',(0,0,-.06,0,0,0),('box',(args.radius*3,args.radius*3,.1)),'.25 .38 .2 1',True)
    def segment_distance(a,b):
        dx,dy=b[0]-a[0],b[1]-a[1];norm=dx*dx+dy*dy
        t=max(0.,min(1.,-(a[0]*dx+a[1]*dy)/norm)) if norm else 0.
        return math.hypot(a[0]+t*dx,a[1]+t*dy)
    road_rows=[];building_rows=[];skipped=[]
    for way in root.findall('way'):
        tags={t.get('k'):t.get('v') for t in way.findall('tag')}
        references=[n.get('ref') for n in way.findall('nd')]
        if any(ref not in nodes for ref in references):
            skipped.append({'id':way.get('id'),'reason':'missing referenced OSM node'})
            continue
        points=[nodes[ref] for ref in references]
        # Keep complete nearby segments/outlines; never invent missing OSM nodes.
        if len(points)<2 or min(segment_distance(a,b) for a,b in zip(points,points[1:]))>args.radius:continue
        ident=way.get('id')
        if not ident or not re.fullmatch(r'-?\d+',ident):
            skipped.append({'id':ident,'reason':'invalid OSM way id'})
            continue
        if tags.get('highway') in DRIVABLE:
            width=number(tags.get('width'));lanes=number(tags.get('lanes'))
            origin='OSM width' if width else 'assumed 3.2m per OSM lane' if lanes else 'assumed default width'
            width=width or (lanes*3.2 if lanes else args.default_width)
            if not 1<=width<=40:skipped.append({'id':ident,'reason':'invalid road width'});continue
            segments=0
            for index,(a,b) in enumerate(zip(points,points[1:])):
                if segment_distance(a,b)>args.radius:continue
                length=math.dist(a,b)
                if length<.05:continue
                yaw=math.atan2(b[1]-a[1],b[0]-a[0]);mid=((a[0]+b[0])/2,(a[1]+b[1])/2)
                item('road_{}_{}'.format(ident,index),(*mid,.001,0,0,yaw),('box',(length+.2,width,.015)),'.17 .19 .2 1')
                segments+=1
            road_rows.append({'way_id':ident,'name':tags.get('name'),'width_m':width,'width_source':origin,'segments':segments})
        if 'building' in tags and tags.get('building')!='no':
            if points[0]!=points[-1] or len(points)<4:
                skipped.append({'id':ident,'reason':'open or incomplete building footprint'});continue
            height=number(tags.get('height'));levels=number(tags.get('building:levels'))
            origin='OSM height' if height else 'assumed 3m per OSM level' if levels else 'assumed default height'
            height=height or (levels*3 if levels else args.default_height)
            if not 1<=height<=200:skipped.append({'id':ident,'reason':'invalid building height'});continue
            cx=sum(x for x,y in points[:-1])/(len(points)-1);cy=sum(y for x,y in points[:-1])/(len(points)-1)
            try:building_mesh(meshes/(ident+'.stl'),[(x-cx,y-cy) for x,y in points],height)
            except ValueError as error:skipped.append({'id':ident,'reason':str(error)});continue
            item('building_'+ident,(cx,cy,0,0,0,0),('mesh',(meshes/(ident+'.stl')).resolve()),'.64 .61 .54 1',True)
            building_rows.append({'way_id':ident,'height_m':height,'height_source':origin})
    ET.ElementTree(sdf).write(str(folder/'campus.world'),encoding='utf-8',xml_declaration=True)
    manifest={'osm_source':str(source),'osm_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
        'road_rendering':'OGRE depth_bias 4; geometric heights unchanged',
        'origin_lat_lon':[args.lat,args.lon],'radius_m':args.radius,'roads':road_rows,'buildings':building_rows,'skipped':skipped,
        'limits':['Flat ground; no measured elevations, textures or traffic rules simulation',
                  'Road widths and building heights can be assumptions, listed per object',
                  'OSM relation multipolygon buildings are not reconstructed',
                  'Scene preview only: closed-loop navigation on this real map is not verified']}
    (folder/'scene_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (folder/'README.md').write_text('# 校区 OSM 几何场景\n\n源地图：`'+str(source)+'`。道路中心线及建筑轮廓来自该文件；路宽、楼高的来源逐项写入 scene_manifest.json。\n\n'+
        '生成道路 {} 条、建筑 {} 栋。平坦地面，无真实纹理与实测高程；未重建多面关系建筑，不是照片级数字孪生。本场景只验证加载展示，未完成真实路网闭环导航。\n'.format(len(road_rows),len(building_rows)))
    print(json.dumps({'world':str(folder/'campus.world'),'roads':len(road_rows),'buildings':len(building_rows),'skipped':len(skipped)},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--osm',default='/home/nuc/lxw/ros_maps/map.osm');p.add_argument('--output-dir',required=True)
    p.add_argument('--lat',type=float,default=23.0398);p.add_argument('--lon',type=float,default=113.39002)
    p.add_argument('--radius',type=float,default=250);p.add_argument('--default-width',type=float,default=6)
    p.add_argument('--default-height',type=float,default=9)
    generate(p.parse_args())
