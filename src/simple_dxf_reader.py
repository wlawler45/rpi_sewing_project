import dxfgrabber
import math
import matplotlib
import rospy
import matplotlib.pyplot as plt
import numpy as np
import csv

def distance_calculator(start,end):
    return math.sqrt( (start[0]-end[0])**2 + (start[1]-end[1])**2)

filename="ARM_PANT_DXF.dxf"
dxf=dxfgrabber.readfile(filename)
layer_count = len(dxf.layers)

#print(layer_count)
#print(len(dxf.blocks))
for entity in dxf.entities:
    print(entity.dxftype)
all_blocks= [entity for entity in dxf.entities if entity.dxftype == 'INSERT']
all_splines=[entity for entity in dxf.entities if entity.dxftype == 'SPLINE']
#print(all_splines)
if(len(all_blocks)!=0):
    test_block=dxf.blocks[all_blocks[0].name]
    all_polylines= [entity for entity in test_block if entity.dxftype == 'POLYLINE']
else:
    all_polylines=[entity for entity in dxf.entities if entity.dxftype == 'POLYLINE']

#print(len(all_polylines))
x=0
last_in=False
removed=[]
flag=False
vertices=[]
sine_vals=[]
#with open('eggs.csv', 'w') as csvfile:
#    spamwriter = csv.writer(csvfile, delimiter=' ',
#                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
for i in range(len(all_polylines)):
    start=all_polylines[i].points[0]
    end=all_polylines[i].points[-1]
    
    
    for e in range(len(all_polylines)):
        if(i==e):
            continue
        else:
        
            if(all_polylines[e].points[0]==start and all_polylines[e].points[-1]==end):
                #print("identical start found")
                #print(start)
                #print(i)
                #print(e)
            #if(all_polylines[e].points[-1]==end):
                #print("identical end found")
                #print(end)
                #print(i)
                #print(e)
                if(len(all_polylines[i].points)==len(all_polylines[e].points) and not flag):
                    flag=True
                    removed.append(all_polylines[e])
                if(len(all_polylines[i].points)>len(all_polylines[e].points)):
                    
                    #print(len(all_polylines[i].points))
                    #print(len(all_polylines[e].points))
                    removed.append(all_polylines[e])
                    

#print(vertices)
outside=0
for entry in removed:
    
    all_polylines.remove(entry)
#print(len(all_polylines))    
"""for i in range(len(all_polylines)):
    for point in all_polylines[i].points:
        vertices.append(point)
"""
for i in all_polylines[1].points:
    print(i)
    vertices.append(i)
shape=matplotlib.path.Path(vertices,closed=True)       
       
colors=['b','g','r','c','m','y','k']
seam_counter=0
entity=all_polylines[1]
color=colors[x]
sine_vals=[]
motion_points=[]
for i in range(len(entity.points)-1):
    
    plt.plot([entity.points[i][0],entity.points[i+1][0]],[entity.points[i][1],entity.points[i+1][1]],color)
    #spamwriter.writerow("%f %f"%(entity.points[i][0],entity.points[i][1]))
    if(i==0):
        magnitude=distance_calculator(entity.points[i],entity.points[i+1])
        x_vals=[entity.points[i][0],entity.points[i+1][0]]
        y_vals=[entity.points[i][1],entity.points[i+1][1]]
    else:
        
        magnitude=distance_calculator(entity.points[i-1],entity.points[i+1])
        x_vals=[entity.points[i-1][0],entity.points[i][0],entity.points[i+1][0]]
        y_vals=[entity.points[i-1][1],entity.points[i][1],entity.points[i+1][1]]
    
    #bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
    
    #m=(entity.points[i+1][1]-entity.points[i][1])/(entity.points[i+1][0]-entity.points[i][0])
    #print(entity.points[i+1][0]-entity.points[i][0])
    
    #print(magnitude/bestfitlineslope)
    
    bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)
    #print("best fit line:")
    #print(bestfitlineslope)
    #print(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
    #plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
    #plt.plot(entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1)),'bo')                
    if(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))])):
        outside+=1
        value=bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))/(-1/(math.sqrt(bestfitlineslope**2+1)))
        plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(math.sqrt(bestfitlineslope**2+1)),-1/(math.sqrt(bestfitlineslope**2+1)))
        sine_vals.append(math.atan2(-1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))))
    else:
        value=bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1))/(1/(math.sqrt(bestfitlineslope**2+1)))
        #print([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))])
        plt.arrow(entity.points[i][0],entity.points[i][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
        sine_vals.append(math.atan2(1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1))))
        #plt.plot(entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1)),'bo')
    #plt.arrow(entity.points[i][0],entity.points[i][1],-(entity.points[i+1][1]-entity.points[i][1])/magnitude,(entity.points[i+1][0]-entity.points[i][0])/magnitude)
    
    #print(distance_calculator(entity.points[i],[entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
    #print(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))]))
    #print(end)
    #plt.plot([entity.points[i][0],entity.points[i][1]],[entity.points[i][0]-end,entity.points[i][1]+end])

    
    motion_points.append(entity.points[i])
    

x_vals=[entity.points[-2][0],entity.points[-1][0]]
y_vals=[entity.points[-2][1],entity.points[-1][1]]

bestfitlineslope, offset=np.polyfit(x_vals,y_vals,1)

if(shape.contains_point([entity.points[i][0]+bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),entity.points[i][1]+1/(math.sqrt(bestfitlineslope**2+1))])):
    outside+=1
    
    value=bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))/(-1/(math.sqrt(bestfitlineslope**2+1)))  
    plt.arrow(entity.points[-1][0],entity.points[-1][1],bestfitlineslope/(math.sqrt(bestfitlineslope**2+1)),-1/(math.sqrt(bestfitlineslope**2+1)))
    sine_vals.append(math.atan2(-1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(math.sqrt(bestfitlineslope**2+1))))
else:
    plt.arrow(entity.points[-1][0],entity.points[-1][1],bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1)),1/(math.sqrt(bestfitlineslope**2+1)))
    sine_vals.append(math.atan2(1/(math.sqrt(bestfitlineslope**2+1)),bestfitlineslope/(-math.sqrt(bestfitlineslope**2+1))))

x+=1
#sine_vals.append(math.asin(value))
motion_points.append(entity.points[-1])
#print(sine_vals)


#plan, fraction, points=self.plan_robot_motion(motion_points,sine_vals)
#if(fraction<1.0):
#    self.execute_consecutive_motions(points)
#else:

#self.display_robot_trajectory(plan)
#self.execute_planned_trajectory(plan)
if(x+1>len(colors)):
    x=0
seam_counter+=1
#spamwriter.writerow("")
#print("end")


#print(outside)
plt.show()
"""if(all_lines[0].start[0]>all_lines[1].start[0]):
    bottom_hem=all_lines[0]
    waist_hem=all_lines[1]
else:
    bottom_hem=all_lines[1]
    waist_hem=all_lines[0]
print(bottom_hem.start)
print(bottom_hem.end)
print(distance_calculator(all_lines[0].start,all_lines[0].end))
print(waist_hem.start)
print(waist_hem.end)
print(distance_calculator(all_lines[1].start,all_lines[1].end))
for control_points in all_splines[0].fit_points:
    print(control_points)
    
"""
