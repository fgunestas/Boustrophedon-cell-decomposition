# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 17:27:29 2020

@author: 1700003918
"""
import matplotlib.path as mpltPath
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from matplotlib.collections import PatchCollection
#import matplotlib as mpl



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

def slice_control(dilim,bas,sinir,ustsinir,zones,px):
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

def unpack (zone,dilim,area,px):
    top=area
    top=np.array(top)
    top=max(top[:,1])

    zone_stop=-9999999999
    for i in range(len(zone)):
        if zone[i][0]>zone_stop:
            zone_stop=zone[i][0]
    while True:
        make_point=[dilim,top]
        top=top-px
        pack=point_control([zone],make_point)
        wtfpack=point_control([area],make_point)
        if wtfpack[0][0]==False:
            start=[make_point,zone,zone_stop]
            break
        if pack[0][0]==True:
            start=[make_point,zone,zone_stop]
            break
    return start

def BCD(zones,area,px):
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
    while True:

        #print(len(cells))
        #print(dilim,bas,sinir,ustsinir)
        paket=slice_control(dilim,bas,sinir,ustsinir,zones,px)
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
                new_start=unpack(paket[2],denied_start,area,px)
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
    return cells
def dist(position1, position2):
    sum = 0
    for i in range(len(position1)):
        diff = position1[i]-position2[i]
        sum += diff * diff
    return math.sqrt(sum)




#görüntü bölgesi


fig, ax = plt.subplots()

area=[[1000,1000],[-1000,1000],[-1000,-1000],[1000,-1000]]


obstacles=[]
obstacles_forplot = []

circle = Circle((750, 50), 100)
obstacles_forplot.append(circle)
verts=circle.get_path().vertices
trans = circle.get_patch_transform()
circle_corners = list(trans.transform(verts))
obstacles.append(circle_corners)


triangle_corners=[[-600,750],[-250,250],[-800,208]]
obstacles.append(triangle_corners)

triangle = Polygon(np.array(triangle_corners), True)
obstacles_forplot.append(triangle)

square_corners=[[-100,250],[400,250],[400,-250],[-100,-250]]
obstacles.append(square_corners)

square = Polygon(np.array(square_corners), True)
obstacles_forplot.append(square)


p = PatchCollection(obstacles_forplot, alpha=0.4,facecolors="black")
ax.add_collection(p)



cells=BCD(obstacles,area,10)

cellplot=[]
for i in range(len(cells)):
    cell=Polygon(np.array(cells[i]),True)
    cellplot.append(cell)



colors = 100*np.random.rand(len(cellplot))
c = PatchCollection(cellplot, alpha=0.4)
c.set_array(np.array(colors))
ax.add_collection(c)
fig.colorbar(c, ax=ax)






ax.autoscale_view()
plt.show()


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





tasks_hash=[]
for i in range(data["uav_count"]):
    tasks_hash.append([])
i=0
for i in range(len(path_keys)):
    j=i%data["uav_count"]
    #print(j)
    hashno=str(path_keys[i])
    tasks_hash[j].append([hashno])


aray=np.array(aray)
plt.plot(aray[:,0], aray[:,1], 'ro')


          
sorted_keys=[]
for i in range(len(sorted_subareas)):
    x=str(hash(str(sorted_subareas[i])))
    sorted_keys.append(x)

ax.autoscale_view()
plt.show()
"""
