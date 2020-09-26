# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 17:27:29 2020

@author: 1700003918
"""
import json
import matplotlib.path as mpltPath
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
from scipy.spatial import ConvexHull
#from threading import Thread
#import matplotlib as mpl
"""
def slm():
    #print("slm")

s=Thread(target=slm)
s.start()
"""
with open('sim0.json') as f:
    data = json.load(f)


def point_control(zones,point):
    i=0

    tik=0
    for i in range (len(zones)):
        bolge=mpltPath.Path(zones[i])
        inside=bolge.contains_points([point])
        if inside[0]==True:
            tik=1
            pack=[inside,zones[i]]
            return pack

        if tik==0:
            pack=[inside,0]

    return pack

def slice_control(dilim,bas,sinir,ustsinir,zones,data,px):
    pack=[]
    start=1
    #print(dilim,bas,sinir,ustsinir)
    for i in range(int(bas),sinir,-px):
        #print(dilim,bas)
        make_point=[dilim,i]
        inside=point_control(zones,make_point)
        if (start==1) and (inside[0][0]==False):
            upper=make_point
            start=0
        if start==0:
            if (inside[0][0]==True) or (i-px<=sinir):
                if inside[0][0]==True:
                    ustsinir=inside[1]
                lower=make_point
                if inside[0][0]==True:
                    pack=[upper,lower,inside[1],ustsinir]
                    return pack
                if inside[0][0]==False:
                    pack=[upper,lower,0,ustsinir]
                    return pack

    return pack

def unpack (zone,dilim,data,px):
    #print("unpack")
    top=data["world_boundaries"]
    top=max(top)
    zone_stop=-9999999999
    for i in range(len(zone)):
        if zone[i][0]>zone_stop:
            zone_stop=zone[i][0]
    top=top[1]
    while True:
        #print("unpack while")
        make_point=[dilim,top]
        top=top-px
        pack=point_control([zone],make_point)
        wtfpack=point_control([data["world_boundaries"]],make_point)
        if wtfpack[0][0]==False:
            start=[make_point,zone,zone_stop]
            break
        if pack[0][0]==True:
            start=[make_point,zone,zone_stop]
            break
    return start

def BCD(zones,area,data,px):
    area=np.array(area)
    sol=min(area[:,0])
    top=max(area[:,1])
    start=[sol,top]
    dilim=start[0]
    bas=start[1]
    sinir=min(area[:,1])
    altsinir=[]
    upper=[]
    lower=[]
    #solsinir=sol
    ustsinir=max(area[:,1])
    sagsinir=max(area[:,0])
    cells=[]
    cell=[]
    stack_point=[]
    stack_area=[]
    stack_stop=[]
    start=1
    denied_start=0
    bringworst=0
    while True:
        try:
            #print(len(cells))
            #print(dilim,bas,sinir,ustsinir)
            paket=slice_control(dilim,bas,sinir,ustsinir,zones,data,px)
            #print(paket[3],paket[2])
            if start==1:
                altsinir=paket[2]
                ustsinir=paket[3]
                upper.append(paket[0])
                lower.append(paket[1])
                start=0
                dilim=dilim+px
            #print(paket[2])
            #print(paket[3])
            if start==0 and altsinir==paket[2] and ustsinir==paket[3]:
                upper.append(paket[0])
                lower.append(paket[1])
                dilim=dilim+px
            if start==0:
                if (altsinir!=paket[2]) or (ustsinir!=paket[3]):
                    if (altsinir!=paket[2]) and (ustsinir!=0):
                        denied_start=dilim
                    temp=[]
                    temp=lower[::-1]
                    cell=temp+upper
                    cells.append(cell)
                    cell=[]
                    lower=[]
                    upper=[]
                    altsinir=paket[2]
                    ustsinir=paket[3]
                if paket[2]!=0:
                    new_start=unpack(paket[2],denied_start,data,px)
                    if new_start[0] not in stack_point:
                        stack_area.append(new_start[1])
                        stack_point.append(new_start[0])
                        stack_stop.append(new_start[2])
                if dilim>sagsinir and len(stack_point)!=0:
                    temp=[]
                    temp=lower[::-1]
                    cell=temp+upper
                    cells.append(cell)
                    cell=[]
                    lower=[]
                    upper=[]
                    altsinir=paket[2]
                    ustsinir=paket[3]
                    pop=stack_point.pop()
                    stack_area.pop()
                    zone_stop=stack_stop.pop()
                    dilim=pop[0]
                    bas=pop[1]
                    sagsinir=int(zone_stop)
                if dilim>sagsinir and len(stack_point)==0:
                    temp=[]
                    temp=lower[::-1]
                    cell=temp+upper
                    cells.append(cell)
                    #print("slm")
                    break
        except:
            #print("except cells",len(cells))
            dilim=dilim+px
            if len(cells)>100:
                bringworst=1

    return cells
tall_index = 0
while(data['special_assets'][tall_index]['type'] != 'tall_building'):
    tall_index += 1
special_assets = []
tall_count = len(data['special_assets'][tall_index]['locations'])

#if data['special_assets'][tall_index]['width'][0] > data['special_assets'][tall_index]['width'][1]:
#    bridge_length = data['special_assets'][tall_index]['width'][0] * 100
#else:
#    bridge_length = data['special_assets'][tall_index]['width'][1] * 2.5
bridge_length =200
cluster_count = 0
cluster_element_treshold = 3

deniedZones = data['denied_zones']
position_offset = float(data['world_length'] / 2)
color_cursor = 0
colors = [
    'black',
    'red',
    'green',
    'blue',
    'magenta',
    'yellow',
    'cyan',
]

def inDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return True
    return False

def notInDeniedZone(p):
    for polygon in deniedZones:
        path = mpltPath.Path(polygon)
        if path.contains_points(p):
            return False
    return True

def forAll(l):
    for i in l:
        print(i)

def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)



def makeClusters():
    for building in data['special_assets']:
        if building['type'] == 'tall_building':
            for p in building['locations']:
                if notInDeniedZone([p]):
                    special_assets.append({
                        'p':[
                            float(p[0] + position_offset),
                            float(p[1] + position_offset)
                        ],
                        'c': 0
                    })
        else:
            special_assets.append({
                'p':[
                    float(building['location']['x'] + position_offset),
                    float(building['location']['y'] + position_offset)
                ],
                'c': 0
            })
            special_assets.append({
                'p':[
                    float(building['location']['x'] + position_offset),
                    float(building['location']['y'] + position_offset)
                ],
                'c': 0
            })
    global cluster_count
    global cluster_element_treshold
    for i in range(len(special_assets)):
        neighbour_index_list = [i]
        base_point = special_assets[i]
        if not cluster_count:
            for j in range(len(special_assets)):
                if(j != i):
                    d = dist(base_point['p'], special_assets[j]['p'])
                    if d <= bridge_length:
                        neighbour_index_list.append(j)
            if len(neighbour_index_list) > cluster_element_treshold:
                cluster_count = cluster_count + 1
                for j in neighbour_index_list:
                    special_assets[j]['c'] = cluster_count
        else:
            if base_point['c']:
                for j in range(len(special_assets)):
                    if(j != i) and (not special_assets[j]['c']):
                        d = dist(base_point['p'], special_assets[j]['p'])
                        if d <= bridge_length:
                            special_assets[j]['c'] = base_point['c']
            else:
                for j in range(len(special_assets)):
                    d = dist(base_point['p'], special_assets[j]['p'])
                    if d <= bridge_length:
                        if(j != i) and (special_assets[j]['c']):
                            special_assets[i]['c'] = special_assets[j]['c']
                            break
                        neighbour_index_list.append(j)
                if special_assets[i]['c']:
                    continue
                elif len(neighbour_index_list) > cluster_element_treshold:
                    cluster_count += 1
                    for j in neighbour_index_list:
                        special_assets[j]['c'] = cluster_count
def normalPos( p):
    return [p[0] - position_offset, p[1] - position_offset]

def unpacked_cluster(clusters,width):
    mask_for_cluster=[]
    for i in range(len(clusters)):
        mask_for_cluster.append([])
    for i in range(len(clusters)):
        for j in range(len(clusters[i])):
            clusters[i][j]
            tmp=[clusters[i][j][0]-float((width/2)),clusters[i][j][1]+float((width/2))]
            tmps=[[tmp[0],tmp[1]],[tmp[0]+width,tmp[1]],[tmp[0]+width,tmp[1]-width],[tmp[0],tmp[1]-width]]
            mask_for_cluster[i].append(tmps)
    return mask_for_cluster

def findPath(hashno,allpoints_dict,px):
    hashno=str(hashno)
    points=allpoints_dict[hashno]
    start=points.pop(0)
    make_point=start
    new_path=[]
    new_path.append(start)
    back=0
    while len(points)!=0:
        if [make_point[0],make_point[1]+px] in points:
            back=0
            points.remove([make_point[0],make_point[1]+px])
            new_path.append([make_point[0],make_point[1]+px])
            make_point=[make_point[0],make_point[1]+px]
            continue

        elif [make_point[0]+px,make_point[1]] in points:
            back=0
            points.remove([make_point[0]+px,make_point[1]])
            new_path.append([make_point[0]+px,make_point[1]])
            make_point=[make_point[0]+px,make_point[1]]
            continue

        elif [make_point[0],make_point[1]-px] in points:
            back=0
            points.remove([make_point[0],make_point[1]-px])
            new_path.append([make_point[0],make_point[1]-px])
            make_point=[make_point[0],make_point[1]-px]
            continue
        elif [make_point[0]-px,make_point[1]] in points:
            back=0
            points.remove([make_point[0]-px,make_point[1]])
            new_path.append([make_point[0]-px,make_point[1]])
            make_point=[make_point[0]-px,make_point[1]]
            continue

        else:
            try:

                back=back+1
                make_point=new_path[-(2*back)]
                new_path.append([make_point[0],make_point[1]])
            except:
                break

    return new_path
def sortSubareas(subareas,maxQ):
    sub_dist={}
    point=maxQ[0]
    point=np.array(point)
    x = point[:,0]
    y = point[:,1]
    center = [sum(x) / len(point), sum(y) / len(point)]
    for i in range(len(maxQ)-1):
        point=maxQ[i+1]
        point=np.array(point)
        x = point[:,0]
        y = point[:,1]
        temp_center = [sum(x) / len(point), sum(y) / len(point)]
        center=[(center[0]+temp_center[0])/2,(center[1]+temp_center[1])/2]

    i=0
    for i in range(len(subareas)):
        hashno=str(hash(str(subareas[i])))
        point=subareas[i]
        point=np.array(point)
        x = point[:,0]
        y = point[:,1]
        sub_center = [sum(x) / len(point), sum(y) / len(point)]
        distofcenter=dist(sub_center,center)
        sub_dist[hashno]=distofcenter
    i=0
    temp=subareas
    hashkeys=[]
    for i in range(len(temp)):
        hashno=str(hash(str(temp[i])))
        hashkeys.append(hashno)


    hashlist=[]
    temp=0
    for j in range(len(hashkeys)):
        for i in range(len(hashkeys)):
            try:

                low=-sub_dist[hashkeys[i]]
                if temp>low:
                    temp=low
                    hashkey=hashkeys[i]
                    index=i
            except Exception as e:
                print(e)
                continue


        if hashkey not in hashlist:
            hashlist.append(hashkey)

            sub_dist.pop(hashkey)
            hashkeys.pop(index)

            temp=0
    return hashlist
def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return np.rad2deg((ang1 - ang2) % (2 * np.pi))

def findTurnSide(denied,instantloc,finish):
    if denied==0:
        return 0
    point=denied
    point=np.array(point)
    xx = point[:,0]
    yy = point[:,1]
    centerofdenied = [sum(xx) / len(point), sum(yy) / len(point)]

    x=instantloc[0]
    y=instantloc[1]
    new_centerofdenied=[centerofdenied[0]-x,centerofdenied[1]-y]
    new_finish=[finish[0]-x,finish[1]-y]
    angle=angle_between(new_finish,new_centerofdenied)
    return angle


def findRotationPath(deniedzones,start,finish,px):
    deltax=finish[0]-start[0]
    deltay=finish[1]-start[1]
    distance=dist(start,finish)
    rotationPath=[]
    distance_px=distance/px
    x_px=deltax/distance_px
    y_px=deltay/distance_px
    start_t=start
    A=0
    B=0
    C=0
    #planA
    i=0
    rotationPath.append(start_t)
    for i in range(int(distance_px)):
        make_point=[start_t[0]+x_px,start_t[1]+y_px]
        start_t=make_point
        rotationPath.append(make_point)
        distance=dist(start_t,finish)
        pack=point_control(deniedzones,make_point)
        if pack[0]==True:
            A=1
    tempA=rotationPath
    rotationPath=[]
    start_t=start
    #planB
    i=0
    rotationPath.append(start_t)
    for i in range(int(distance_px)):
        make_point=[start_t[0]+x_px,start_t[1]]
        start_t=make_point
        rotationPath.append(make_point)
        distance=dist(start_t,finish)
        pack=point_control(deniedzones,make_point)
        if pack[0]==True:
            B=1
        #print(distance)
    i=0
    for i in range(int(distance_px)):
        make_point=[start_t[0],start_t[1]+y_px]
        start_t=make_point
        rotationPath.append(make_point)
        distance=dist(start_t,finish)
        pack=point_control(deniedzones,make_point)
        if pack[0]==True:
            B=1
        #print(distance)
    tempB=rotationPath
    rotationPath=[]
    #planC
    start_t=start
    i=0
    rotationPath.append(start_t)
    for i in range(int(distance_px)):
        make_point=[start_t[0],start_t[1]+y_px]
        start_t=make_point
        rotationPath.append(make_point)
        distance=dist(start_t,finish)
        pack=point_control(deniedzones,make_point)
        if pack[0]==True:
            C=1
        #print(distance)
    i=0
    for i in range(int(distance_px)):
        make_point=[start_t[0]+x_px,start_t[1]]
        start_t=make_point
        rotationPath.append(make_point)
        distance=dist(start_t,finish)
        pack=point_control(deniedzones,make_point)
        if pack[0]==True:
            C=1
    tempC=rotationPath
    rotationPath=[tempA,tempB,tempC]
    if A==0:
        rotationPath=tempA
    elif B==0:
        rotationPath=tempB
    elif C==0:
        rotationPath=tempC
    start_t=start
    if A==1 and B==1 and C==1:
        #planA
        rotationPath=[]
        i=0
        dodge=0
        rotationPath.append(start_t)
        whiledist=dist(start,finish)
        while whiledist>10:
            #come=dist(finish,start_t)
            aci=math.atan2(start_t[0]-finish[0],start_t[1]-finish[1])
            aci=math.degrees(aci)
            if dodge==0:
                make_point=[start_t[0]+x_px,start_t[1]+y_px]
                pack=point_control(deniedzones,make_point)
            if dodge==0 and pack[0]==False:
                rotationPath.append(make_point)
                start_t=make_point
            if pack[0]==True:
                dodge=1
            if dodge==1:
                make_point=[start_t[0],start_t[1]]
                angle=findTurnSide(pack[1],start_t,finish)
                if aci<=-45 and aci>=-135:
                    if angle>0 and angle<180:
                        point_pack=[[start_t[0]+px,start_t[1]],[start_t[0],start_t[1]+px]]
                    if angle<360 and angle>180:
                        point_pack=[[start_t[0]+px,start_t[1]],[start_t[0],start_t[1]-px]]
                if aci<=45 and aci>=-45:
                    if angle>0 and angle<180:
                        point_pack=[[start_t[0]+px,start_t[1]],[start_t[0],start_t[1]-px]]
                    if angle<360 and angle>180:
                        point_pack=[[start_t[0]-px,start_t[1]],[start_t[0],start_t[1]-px]]
                if aci>=45 and aci<=135:
                    if angle>0 and angle<180:
                        point_pack=[[start_t[0]-px,start_t[1]],[start_t[0],start_t[1]-px]]
                    if angle<360 and angle>180:
                        point_pack=[[start_t[0],start_t[1]+px],[start_t[0]-px,start_t[1]]]
                if (aci>=135 and aci<=180) or (aci<=-135 and aci>=-180):
                    if angle>0 and angle<180:
                        point_pack=[[start_t[0],start_t[1]+px],[start_t[0]-px,start_t[1]]]
                    if angle<360 and angle>180:
                        point_pack=[[start_t[0]+px,start_t[1]],[start_t[0],start_t[1]+px]]
                angle=findTurnSide(pack[1],start_t,finish)
                a=0
                for i in range(len(point_pack)):
                    pack1=point_control(deniedzones,point_pack[i-a])
                    if pack1[0]==True:
                        point_pack.pop(i-a)
                        a=a+1
                distt=dist(finish,point_pack[0])
                lowest=distt
                loc=point_pack[0]
                for i in range(len(point_pack)):
                    pack2=point_control(deniedzones,point_pack[i])
                    if lowest >dist(finish,point_pack[i]) and pack2[0][0]==False:
                        lowest=dist(finish,point_pack[i])
                        loc=point_pack[i]
                start_t=loc
                whiledist=dist(start_t,finish)
                rotationPath.append(start_t)
    return rotationPath
def biggerdenied(denied_zones,px):
    for i in range(len(denied_zones)):
        point=np.array(denied_zones[i])
        x = point[:,0]
        y = point[:,1]
        center = [sum(x) / len(point), sum(y) / len(point)]
        for j in range(len(denied_zones[i])):
            distance=dist([denied_zones[i][j][0],denied_zones[i][j][1]],[center[0],center[1]])
            px_x=denied_zones[i][j][0]-center[0]
            px_y=denied_zones[i][j][1]-center[1]
            px_x=(px_x/distance)*px
            px_y=(px_y/distance)*px
            denied_zones[i][j][0]=denied_zones[i][j][0]+px_x
            denied_zones[i][j][1]=denied_zones[i][j][1]+px_y
    return denied_zones
bigger_denied_zones=biggerdenied(data["denied_zones"],15)
data["denied_zones"]=bigger_denied_zones

def findDRS(path_array):
        drs_array=[]
        drs_array.append(path_array[0])
        for i in range(len(path_array)-2):
            t_aci=math.atan2(path_array[i][0]-path_array[i+1][0],path_array[i][1]-path_array[i+1][1])
            t_aci=math.degrees(t_aci)
            aci=math.atan2(path_array[i+1][0]-path_array[i+2][0],path_array[i+1][1]-path_array[i+2][1])
            aci=math.degrees(aci)
            aci=t_aci-aci
            aci=math.sqrt(aci**2)
            if aci>10:
                make_point=[path_array[i+1][0],path_array[i+1][1]]
                drs_array.append(make_point)
        drs_array.append(path_array[-1])
        return drs_array

def bounderisbig(world_boundaries,width):
    point=world_boundaries
    point=np.array(point)
    x = point[:,0]
    y = point[:,1]
    temp_center = [sum(x) / len(point), sum(y) / len(point)]
   # for i in range(len(world_boundaries)):
    #    =dist(world_boundaries[i],temp_center)
    return temp_center

center=bounderisbig(data["world_boundaries"],200)
    
    





makeClusters()
for i in range(len(special_assets)):
    special_assets[i]['p'] = normalPos(special_assets[i]['p'])





clusters=[]
for i in range(cluster_count+1):
    clusters.append([])
for i in special_assets:
    clusters[i["c"]].append(i["p"])


mask_for_cluster=unpacked_cluster(clusters,150)
merge_tall=[]
temp_mask_for_cluster=[]
for j in range(len(mask_for_cluster)):
    for i in range(len(mask_for_cluster[j])):
        merge_tall=merge_tall+mask_for_cluster[j][i]
    merge_tall=np.array(merge_tall)
    temp_mask_for_cluster.append(merge_tall)
    merge_tall=[]


i=0
maxQ_Areas=[]
for i in range(1,len(temp_mask_for_cluster)):
    points = temp_mask_for_cluster[i]
    hull = ConvexHull(points)
    temp=list(points[hull.vertices])
    maxQ_Areas.append(temp)


tall_locs_=[]
for t in range(len(data["special_assets"])):
    if data["special_assets"][t]["type"]=="tall_building":
        tall_width=max(data["special_assets"][t]["width"])+35
        tall_locs=data["special_assets"][t]["locations"]
        for i in range(len(tall_locs)):
            tmp=[tall_locs[i][0]-(tall_width/2),tall_locs[i][1]+(tall_width/2)]
            tmps=[[tmp[0],tmp[1]],[tmp[0]+tall_width,tmp[1]],[tmp[0]+tall_width,tmp[1]-tall_width],[tmp[0],tmp[1]-tall_width]]
            tall_locs_.append(tmps)
            
h_locs=[]
i=0
for i in range(len(data["special_assets"])):
    if data["special_assets"][i]["type"]=="hospital":
        xtemp=[data["special_assets"][i]["location"]["x"],data["special_assets"][i]["location"]["y"]]
        h_tmp=[data["special_assets"][i]["location"]["x"]-30,data["special_assets"][i]["location"]["y"]+40]
        h_tmps=[[h_tmp[0],h_tmp[1]],[h_tmp[0]+60,h_tmp[1]],[h_tmp[0]+60,h_tmp[1]-80],[h_tmp[0],h_tmp[1]-80]]
        h_locs.append(h_tmps)


data["denied_zones"]
all_denied=[]
all_denied=tall_locs_+h_locs+data["denied_zones"]



denied_for_bcd=[]
denied_for_bcd=data["denied_zones"]+mask_for_cluster[0]+maxQ_Areas
area=data["world_boundaries"]
subareas=BCD(denied_for_bcd,area,data,20)
#print(subareas)
i=0
temp=[]
subarea_dict={}
for i in range (len(subareas)):
    if len(subareas[i])>=2:
        temp.append(subareas[i])
subareas=temp

for i in range(len(subareas)):
    subarea_dict[str(hash(str(subareas[i])))]=subareas[i]

#buyukten kucuge
temp=[]
sorted_subareas=sortSubareas(subareas,maxQ_Areas)
for i in range(len(sorted_subareas)):
    temp.append(subarea_dict[sorted_subareas[i]])
sorted_subareas=temp
#kucukten buyuge
sorted_subareas=sorted_subareas[::-1]

#temp=[]
#for i in range(len(sorted_subareas)):
#    temp.append(subarea_dict[sorted_subareas[i]])
#sorted_subareas=temp



sorted_subareas=maxQ_Areas+sorted_subareas

subareas=maxQ_Areas+subareas



path_for_subareas={}
temp=[]
px=25
path_keys=[]
top_right=max(data["world_boundaries"])
for path_point in range(0,data["world_length"],px):
    for path_point1 in range (0,data["world_width"],px):
        make_point=[top_right[0]-path_point,top_right[1]-path_point1]
        ekle=1

        for i in range (len(all_denied)):
            paths=mpltPath.Path(all_denied[i])
            inside=paths.contains_points([make_point])
            if inside==True:
                ekle=0
            elif inside==False:
                continue
        if ekle==1:
            pack=point_control(subareas,make_point)
            hashh=hash(str(pack[1]))
            if str(str(hashh)) in path_for_subareas:
                
                temp=path_for_subareas[str(hashh)]
                temp.append(make_point)
                if hashh not in path_keys:
                    path_keys.append(hashh)
                path_for_subareas[str(hashh)]=temp
            else:
                path_for_subareas[str(hashh)]=[make_point]

            #path.append(make_point)





#tüm bölgelere yol cizildi

for i in range(len(path_keys)):
    hashh=path_keys[i]
    new_path=findPath(hashh,path_for_subareas,px)
    path_for_subareas[str(hashh)]=new_path





#bitir=[232,467]
#ornek2=[-9,66]
#basla=[515,473]

#angle=findTurnSide(ornek2,ornek3,ornek1)
#rotationpath=findRotationPath(data["denied_zones"],ornek2,basla,bitir,10)



#görüntü bölgesi

#girilmesi yasak bölgeler
fig, ax = plt.subplots()
patches=[]
for i in range(len(all_denied)):
    polygon = Polygon(all_denied[i], True)
    patches.append(polygon)

k = PatchCollection(patches)
ax.add_collection(k)


#plt.plot(bitir[0], bitir[1], 'go')
#plt.plot(basla[0], basla[1], 'bo')


"""
#tarama bölgeleri
patches=[]
for i in range(len(subareas)):
    polygon = Polygon(subareas[i])
    patches.append(polygon)

colors = 100*np.random.rand(len(patches))
k = PatchCollection(patches, alpha=0.4)
k.set_array(np.array(colors))
ax.add_collection(k)
fig.colorbar(k, ax=ax)
"""






patches=[]
for i in range(len(sorted_subareas)):
    polygon = Polygon(sorted_subareas[i])
    patches.append(polygon)
colors = np.ones(len(patches))


for i in range(len(patches)):
    colors[i]=colors[i]*(i+1)
colors=colors*10
k = PatchCollection(patches, alpha=0.4)
k.set_array(np.array(colors))
ax.add_collection(k)
fig.colorbar(k, ax=ax)


#cb1 = mpl.colorbar.ColorbarBase(k, cmap=cmap,
                                #norm=norm,
                                #orientation='vertical')
#fig.colorbar(k, ax=ax)


#noktalar
#hashh=hash(str(subareas[0]))
#hashh=np.array(path_for_subareas[str(hashh)])
#plt.plot(hashh[:,0],hashh[:,1],marker='.',color='black',linewidth=0)



for i in range(len(path_keys)):
    hashno=str(path_keys[i])
    plot_path=path_for_subareas[hashno]
    plot_path=np.array(plot_path)
    plt.plot(plot_path[:,0], plot_path[:,1], 'k-')



# ==================test===========================================================
top=0
for i in range(len(path_keys)):
    hashno=str(path_keys[i])
    path=path_for_subareas[hashno]
    path=findDRS(path)
    leng=len(path)
    top=(top+leng)
top=top/9
def cam_sensor_width(data):
        aci=data["logical_camera_horizontal_fov"]/2
        scan_height=data["logical_camera_height_max"]-0.5
        baci=180-(aci+90)
        #print(baci)
        baci_r=math.radians(baci)
        baci_sin=math.sin(baci_r)
        #print(baci_sin)
        baci_cos=math.cos(baci_r)
        hipo=scan_height/baci_sin
        #print(hipo)
        #print(hipo)
        width=hipo*baci_cos*2#feet
        #print(width)
        width=(0.3048*width)#metre
        return width
hm=cam_sensor_width(data)
print(hm)


tasks_hash=[]
for i in range(data["uav_count"]):
    tasks_hash.append([])
i=0
for i in range(len(path_keys)):
    j=i%data["uav_count"]
    #print(j)
    hashno=str(path_keys[i])
    tasks_hash[j].append([hashno])

#for i in range(len(path_keys)):
#    for j in range(len(subareas)):
#        hm=str(hash(str(subareas[j])))
#        if str(path_keys[i])==hm:
#            #print("bum")
    
    
aray=[-6500.0,300.0],[-6550.0,350.0],[-6600.0,400.0],[-6650.0,450.0],[-6700.0,500.0],[-6750.0,550.0],[-6800.0,450.0],[-6850.0,500],[-6900.0,550.0],[-6950.0,600.0],[-9800.0,900.0],[-9800.0,840.0],[-9700.0,870.0],[-9760.0,860.0],[-6700.0,1600.0],[-6800.0,1600],[-6900.0,1600.0],[-7000.0,1600.0],[-8000.0,870.0],[-8050.0,860.0],[-8050.0,850.0],[-8190.0,400],[-8190.0,480.0],[-7000.0,300.0],[-7000.0,350.0],[-7050.0,400.0],[-7200.0,400.0],[-7200.0,300],[-7300.0,350]

aray=np.array(aray)
plt.plot(aray[:,0], aray[:,1], 'ro')
#hashno=hash(str(subareas[2]))
#slow_path=path_for_subareas[str(hashno)]
#t_aci=None


"""       
hashno=hash(str(subareas[2]))
rot_pat=path_for_subareas[str(hashno)]
test_rotation=findRotationPath(data["denied_zones"],[3300,-310],rot_pat[0],10)
test_rotation=np.array(test_rotation)
plt.plot(test_rotation[:,0], test_rotation[:,1], 'r-')

for i in range(len(test_rotation)-2):
    
    
    t_aci=math.atan2(test_rotation[i][0]-test_rotation[i+1][0],test_rotation[i][1]-test_rotation[i+1][1])
    t_aci=math.degrees(t_aci)
    aci=math.atan2(test_rotation[i+1][0]-test_rotation[i+2][0],test_rotation[i+1][1]-test_rotation[i+2][1])
    aci=math.degrees(aci)
    aci=t_aci-aci
    aci=math.sqrt(aci**2)

    if aci>10:
        plt.plot(test_rotation[i][0], test_rotation[i][1], 'g.')
"""

    #if aci==t_aci:
        
    #    t_aci=aci
    #if aci!=t_aci:
    #    print(aci,t_aci)
    #    plt.plot(plot_path[:,0], plot_path[:,1], 'r.')
    #    t_aci=aci


#rotationplot=np.array(rotationpath)
#plt.plot(rotationplot[:,0], rotationplot[:,1], 'r-')
# 
# ===============================test==============================================
          
sorted_keys=[]
for i in range(len(sorted_subareas)):
    x=str(hash(str(sorted_subareas[i])))
    sorted_keys.append(x)


#x=path_keys[12]
"""
for i in range(len(path_keys)):
    path=path_for_subareas[str(path_keys[i])]
    t_drs=findDRS(path)
    t_drs=np.array(t_drs)
    plt.plot(t_drs[:,0], t_drs[:,1], 'r.')
"""   


ax.autoscale_view()
plt.show()
#print(sorted_keys)
#print()
#print()
#print(path_for_subareas)